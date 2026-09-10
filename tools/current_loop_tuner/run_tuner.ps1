$ErrorActionPreference = "Stop"
$toolRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = Join-Path $toolRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python)) {
    $python = "py"
    $args = @("-3", "-m", "current_loop_tuner.app")
} else {
    $args = @("-m", "current_loop_tuner.app")
}
$env:PYTHONPATH = $toolRoot
Push-Location $toolRoot
try {
    & $python @args
    exit $LASTEXITCODE
} finally {
    Pop-Location
}