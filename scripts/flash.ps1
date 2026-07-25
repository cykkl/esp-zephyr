param(
    [string]$Port = 'COM17'
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$ZephyrEnvironment = 'F:\zephyr\zephyr-env.ps1'
$BuildDirectory = Join-Path $ProjectRoot 'build'
$Firmware = Join-Path $BuildDirectory 'zephyr\zephyr.bin'

if (-not (Test-Path -LiteralPath $Firmware)) {
    throw "Firmware not found. Run .\scripts\build.ps1 first."
}

. $ZephyrEnvironment

west flash `
    -d $BuildDirectory `
    --runner esp32 `
    --esp-device $Port

