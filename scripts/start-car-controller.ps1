param(
    [string]$Port = 'COM17'
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Python = 'F:\zephyr\.venv\Scripts\python.exe'
$Controller = Join-Path $ProjectRoot 'pc\car_controller.py'

& $Python $Controller --port $Port
