param(
    [switch]$Pristine
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Application = Join-Path $ProjectRoot 'apps\espnow_peer'
$BuildDirectory = Join-Path $ProjectRoot 'build-espnow'
$ZephyrEnvironment = 'F:\zephyr\zephyr-env.ps1'
$PristineMode = if ($Pristine) { 'always' } else { 'auto' }

. $ZephyrEnvironment

west build `
    -p $PristineMode `
    -b esp32c6_devkitc/esp32c6/hpcore `
    $Application `
    -d $BuildDirectory

