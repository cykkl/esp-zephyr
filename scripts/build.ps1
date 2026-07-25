param(
    [switch]$Pristine
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$ZephyrEnvironment = 'F:\zephyr\zephyr-env.ps1'
$BuildDirectory = Join-Path $ProjectRoot 'build'
$PristineMode = if ($Pristine) { 'always' } else { 'auto' }

if (-not (Test-Path -LiteralPath $ZephyrEnvironment)) {
    throw "Zephyr environment script not found: $ZephyrEnvironment"
}

. $ZephyrEnvironment

west build `
    -p $PristineMode `
    -b esp32c6_devkitc/esp32c6/hpcore `
    $ProjectRoot `
    -d $BuildDirectory

