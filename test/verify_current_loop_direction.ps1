param(
    [string]$Port = "COM7",
    [int]$BaudRate = 115200,
    [int]$SampleCount = 5,
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

try {
    $serial.Open()
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)

    while ([DateTime]::UtcNow -lt $deadline -and $samples.Count -lt $SampleCount) {
        try {
            $line = $serial.ReadLine()
        }
        catch [System.TimeoutException] {
            continue
        }

        if ($line -match 'FOC_CURRENT_TEST:.*\bid:\s*([-+]?\d+(?:\.\d+)?),\s*iq:\s*([-+]?\d+(?:\.\d+)?),.*\biq_error:\s*([-+]?\d+(?:\.\d+)?),.*\bvq:\s*([-+]?\d+(?:\.\d+)?)\s*V') {
            $samples.Add([pscustomobject]@{
                IdA      = [double]$Matches[1]
                IqA      = [double]$Matches[2]
                IqErrorA = [double]$Matches[3]
                VqV      = [double]$Matches[4]
            })
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
    Write-Error "Only captured $($samples.Count) FOC samples from $Port; expected $SampleCount."
    exit 2
}

$averageIq = ($samples | Measure-Object -Property IqA -Average).Average
$averageIqError = ($samples | Measure-Object -Property IqErrorA -Average).Average
$averageVq = ($samples | Measure-Object -Property VqV -Average).Average

Write-Output ("samples={0}, avg_iq={1:F4} A, avg_iq_error={2:F4} A, avg_vq={3:F4} V" -f $samples.Count, $averageIq, $averageIqError, $averageVq)

if ($averageVq -le 0.0) {
    Write-Error ("Expected a positive q-axis voltage command for the forward-direction probe, but avg_vq={0:F4} V." -f $averageVq)
    exit 1
}

exit 0
