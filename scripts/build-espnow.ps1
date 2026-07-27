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
    $RoleOverlay = Join-Path $Application "boards\esp32c6_devkitc_hpcore_$RoleName.overlay"

    Write-Host "Building ESP-NOW $CurrentRole firmware"
    $WestArguments = @(
        'build',
        '-p', $PristineMode,
        '-b', 'esp32c6_devkitc/esp32c6/hpcore',
        $Application,
        '-d', $BuildDirectory,
        '--',
        "-DEXTRA_CONF_FILE=$RoleConfig"
    )
    if (Test-Path -LiteralPath $RoleOverlay) {
        $WestArguments += "-DEXTRA_DTC_OVERLAY_FILE=$RoleOverlay"
    }

    west @WestArguments

    if ($LASTEXITCODE -ne 0) {
        throw "ESP-NOW $CurrentRole build failed with exit code $LASTEXITCODE"
    }
}
