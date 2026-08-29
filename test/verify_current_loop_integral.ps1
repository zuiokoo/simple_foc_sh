param(
    [string]$Port = "COM7",
    [int]$BaudRate = 115200,
    [int]$SampleCount = 5,
    [int]$TimeoutSeconds = 15,
    [double]$QAxisKp = 2.0,
    [double]$MinimumAverageIntegralV = 0.01
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

        if ($line -match 'FOC_CURRENT_TEST:.*\biu:\s*([-+]?\d+(?:\.\d+)?),\s*iv:\s*([-+]?\d+(?:\.\d+)?),\s*iw:\s*([-+]?\d+(?:\.\d+)?),.*\biq:\s*([-+]?\d+(?:\.\d+)?),.*\biq_error:\s*([-+]?\d+(?:\.\d+)?),.*\bvq:\s*([-+]?\d+(?:\.\d+)?)\s*V') {
            $iqErrorA = [double]$Matches[5]
            $vqV = [double]$Matches[6]

            $samples.Add([pscustomobject]@{
                IuA               = [double]$Matches[1]
                IvA               = [double]$Matches[2]
                IwA               = [double]$Matches[3]
                IqA               = [double]$Matches[4]
                IqErrorA          = $iqErrorA
                VqV               = $vqV
                IntegralEstimateV = $vqV - ($QAxisKp * $iqErrorA)
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
$averageIntegral = ($samples | Measure-Object -Property IntegralEstimateV -Average).Average
$firstIntegral = $samples[0].IntegralEstimateV
$lastIntegral = $samples[$samples.Count - 1].IntegralEstimateV
$maximumPhaseCurrent = ($samples | ForEach-Object {
    [Math]::Max([Math]::Abs($_.IuA), [Math]::Max([Math]::Abs($_.IvA), [Math]::Abs($_.IwA)))
} | Measure-Object -Maximum).Maximum

Write-Output (
    "samples={0}, avg_iq={1:F4} A, avg_iq_error={2:F4} A, avg_vq={3:F4} V, avg_integral={4:F4} V, first_integral={5:F4} V, last_integral={6:F4} V, max_phase_current={7:F4} A" -f
    $samples.Count,
    $averageIq,
    $averageIqError,
    $averageVq,
    $averageIntegral,
    $firstIntegral,
    $lastIntegral,
    $maximumPhaseCurrent
)

if ($averageIntegral -le $MinimumAverageIntegralV) {
    Write-Error (
        "Expected q-axis integral contribution above {0:F3} V, but average Vq - Kp*IqError was {1:F4} V." -f
        $MinimumAverageIntegralV,
        $averageIntegral
    )
    exit 1
}

exit 0
