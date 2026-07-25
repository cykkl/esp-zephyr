param(
    [ValidateSet('Pair', 'Master', 'Slave')]
    [string]$Role = 'Pair',
    [switch]$Pristine
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Application = Join-Path $ProjectRoot 'apps\espnow_peer'
$ZephyrEnvironment = 'F:\zephyr\zephyr-env.ps1'
$PristineMode = if ($Pristine) { 'always' } else { 'auto' }

. $ZephyrEnvironment

$Roles = if ($Role -eq 'Pair') { @('Master', 'Slave') } else { @($Role) }

foreach ($CurrentRole in $Roles) {
    $RoleName = $CurrentRole.ToLowerInvariant()
    $BuildDirectory = Join-Path $ProjectRoot "build-espnow-$RoleName"
    $RoleConfig = Join-Path $Application "$RoleName.conf"

    Write-Host "Building ESP-NOW $CurrentRole firmware"
    west build `
        -p $PristineMode `
        -b esp32c6_devkitc/esp32c6/hpcore `
        $Application `
        -d $BuildDirectory `
        -- `
        "-DEXTRA_CONF_FILE=$RoleConfig"

    if ($LASTEXITCODE -ne 0) {
        throw "ESP-NOW $CurrentRole build failed with exit code $LASTEXITCODE"
    }
}
