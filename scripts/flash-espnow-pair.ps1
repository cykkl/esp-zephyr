param(
    [string]$FirstPort = 'COM17',
    [string]$SecondPort = 'COM19'
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$BuildDirectory = Join-Path $ProjectRoot 'build-espnow'
$Firmware = Join-Path $BuildDirectory 'zephyr\zephyr.bin'
$ZephyrEnvironment = 'F:\zephyr\zephyr-env.ps1'

if (-not (Test-Path -LiteralPath $Firmware)) {
    throw "ESP-NOW firmware not found. Run .\scripts\build-espnow.ps1 first."
}

. $ZephyrEnvironment

foreach ($Port in @($FirstPort, $SecondPort)) {
    Write-Host "Flashing ESP-NOW firmware to $Port"
    west flash `
        -d $BuildDirectory `
        --runner esp32 `
        --esp-device $Port
}

