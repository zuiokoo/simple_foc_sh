param(
    [string]$Port = "COM7",
    [int]$BaudRate = 115200,
    [int]$ResetCount = 3,
    [int]$PerResetTimeoutSeconds = 8,
    [int]$RunBeforeNextResetMilliseconds = 2000
)

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.ReadTimeout = 300
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$calibrations = [System.Collections.Generic.List[object]]::new()

try {
    $serial.Open()

    for ($cycle = 1; $cycle -le $ResetCount; $cycle++) {
        $serial.DiscardInBuffer()
        $serial.RtsEnable = $true
        Start-Sleep -Milliseconds 150
        $serial.RtsEnable = $false

        $deadline = [DateTime]::UtcNow.AddSeconds($PerResetTimeoutSeconds)
        $calibrationLine = $null
        $reachedControlTask = $false

        while ([DateTime]::UtcNow -lt $deadline) {
            try {
                $line = $serial.ReadLine()
            }
            catch [System.TimeoutException] {
                continue
            }

            if ($line -match 'CURRENT_SENSE: calibration complete: U zero=(\d+) mV, V zero=(\d+) mV') {
                $calibrationLine = $line.Trim()
                $calibrations.Add([pscustomobject]@{
                    Cycle = $cycle
                    UZeroMv = [int]$Matches[1]
                    VZeroMv = [int]$Matches[2]
                })
            }

            if ($line -like '*FOC_CURRENT: Over-current or invalid current:*') {
                Write-Error "Reset cycle $cycle entered over-current before the first control sample. $($line.Trim()) Calibration: $calibrationLine"
                exit 1
            }

            if ($line -like '*FOC_OPEN_LOOP: PWM started, rotor alignment begins*') {
                $reachedControlTask = $true
                Write-Output "cycle=$cycle status=running calibration=[$calibrationLine]"
                break
            }
        }

        if (-not $reachedControlTask) {
            Write-Error "Reset cycle $cycle did not reach the open-loop task within $PerResetTimeoutSeconds seconds. Calibration: $calibrationLine"
            exit 2
        }

        if ($cycle -lt $ResetCount) {
            Start-Sleep -Milliseconds $RunBeforeNextResetMilliseconds
        }
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

$uMinimum = ($calibrations | Measure-Object -Property UZeroMv -Minimum).Minimum
$uMaximum = ($calibrations | Measure-Object -Property UZeroMv -Maximum).Maximum
$vMinimum = ($calibrations | Measure-Object -Property VZeroMv -Minimum).Minimum
$vMaximum = ($calibrations | Measure-Object -Property VZeroMv -Maximum).Maximum

Write-Output "u_zero_range=$uMinimum..$uMaximum mV, v_zero_range=$vMinimum..$vMaximum mV"

if ($calibrations.Count -ne $ResetCount) {
    Write-Error "Expected $ResetCount calibration results, captured $($calibrations.Count)."
    exit 2
}

foreach ($calibration in $calibrations) {
    if ($calibration.UZeroMv -lt 1000 -or $calibration.UZeroMv -gt 2300 -or
        $calibration.VZeroMv -lt 1000 -or $calibration.VZeroMv -gt 2300) {
        Write-Error "Calibration cycle $($calibration.Cycle) is outside the plausible zero-current range: U=$($calibration.UZeroMv) mV, V=$($calibration.VZeroMv) mV."
        exit 1
    }
}

if (($uMaximum - $uMinimum) -gt 150 -or ($vMaximum - $vMinimum) -gt 150) {
    Write-Error "Current zero calibration is not repeatable across spinning resets."
    exit 1
}

Write-Output "restart_calibration_test=passed cycles=$ResetCount"
exit 0
