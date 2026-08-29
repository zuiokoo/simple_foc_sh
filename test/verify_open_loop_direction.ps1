param(
    [string]$Port = "COM7",
    [int]$BaudRate = 115200,
    [int]$SampleCount = 6,
    [int]$TimeoutSeconds = 15
)

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.ReadTimeout = 500
$serial.DtrEnable = $false
$serial.RtsEnable = $false

$samples = [System.Collections.Generic.List[object]]::new()
$startupSeen = $false

try {
    $serial.Open()

    # Reset the ESP32 so this test also proves the open-loop task starts at boot.
    $serial.RtsEnable = $true
    Start-Sleep -Milliseconds 150
    $serial.RtsEnable = $false

    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)

    while ([DateTime]::UtcNow -lt $deadline -and $samples.Count -lt $SampleCount) {
        try {
            $line = $serial.ReadLine()
        }
        catch [System.TimeoutException] {
            continue
        }

        if ($line -match 'FOC_OPEN_LOOP: PWM started, rotor alignment begins') {
            $startupSeen = $true
            $samples.Clear()
            continue
        }

        if (-not $startupSeen) {
            continue
        }

        if ($line -match 'FOC_OPEN_LOOP_TEST:.*\belapsed:\s*(\d+)\s*ms,\s*cmd_elec_angle:\s*([-+]?\d+(?:\.\d+)?),\s*cmd_elec_speed:\s*([-+]?\d+(?:\.\d+)?)\s*rad/s,\s*mech:\s*([-+]?\d+(?:\.\d+)?),\s*mech_velocity:\s*([-+]?\d+(?:\.\d+)?)\s*rad/s') {
            $commandSpeed = [double]$Matches[3]

            # Ignore alignment and ramp samples; judge only the steady +5 rad/s command.
            if ([Math]::Abs($commandSpeed) -ge 4.5) {
                $samples.Add([pscustomobject]@{
                    ElapsedMs          = [int]$Matches[1]
                    CommandAngleRad    = [double]$Matches[2]
                    CommandSpeedRadS   = $commandSpeed
                    MechanicalAngleRad = [double]$Matches[4]
                    MechanicalSpeedRadS = [double]$Matches[5]
                })
            }
        }
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

if ($samples.Count -lt $SampleCount) {
    Write-Error "Only captured $($samples.Count) full-speed FOC_OPEN_LOOP_TEST samples from $Port; expected $SampleCount."
    exit 2
}

$averageCommandSpeed = ($samples | Measure-Object -Property CommandSpeedRadS -Average).Average
$averageMechanicalSpeed = ($samples | Measure-Object -Property MechanicalSpeedRadS -Average).Average
$minimumMechanicalSpeed = ($samples | Measure-Object -Property MechanicalSpeedRadS -Minimum).Minimum
$maximumMechanicalSpeed = ($samples | Measure-Object -Property MechanicalSpeedRadS -Maximum).Maximum

$sensorDirection = if ($averageMechanicalSpeed -gt 0.0) {
    "positive"
}
elseif ($averageMechanicalSpeed -lt 0.0) {
    "negative"
}
else {
    "stationary"
}

Write-Output ("samples={0}, avg_cmd_elec_speed={1:F3} rad/s, avg_mech_speed={2:F3} rad/s, min={3:F3}, max={4:F3}, as5600_direction={5}" -f `
    $samples.Count,
    $averageCommandSpeed,
    $averageMechanicalSpeed,
    $minimumMechanicalSpeed,
    $maximumMechanicalSpeed,
    $sensorDirection)

if ([Math]::Abs($averageMechanicalSpeed) -lt 0.05) {
    Write-Error ("Open-loop command is present, but AS5600 average speed is too small: {0:F3} rad/s." -f $averageMechanicalSpeed)
    exit 1
}

exit 0
