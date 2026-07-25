param(
    [string]$FirstPort = 'COM17',
    [string]$SecondPort = 'COM19'
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$ZephyrEnvironment = 'F:\zephyr\zephyr-env.ps1'

. $ZephyrEnvironment

$Targets = @(
    @{ Role = 'master'; Port = $FirstPort },
    @{ Role = 'slave'; Port = $SecondPort }
)

foreach ($Target in $Targets) {
    $Role = $Target.Role
    $Port = $Target.Port
    $BuildDirectory = Join-Path $ProjectRoot "build-espnow-$Role"
    $Firmware = Join-Path $BuildDirectory 'zephyr\zephyr.bin'

    if (-not (Test-Path -LiteralPath $Firmware)) {
        throw "ESP-NOW $Role firmware not found. Run .\scripts\build-espnow.ps1 first."
    }

    Write-Host "Flashing ESP-NOW $Role firmware to $Port"
    west flash `
        -d $BuildDirectory `
        --runner esp32 `
        --esp-device $Port

    if ($LASTEXITCODE -ne 0) {
        throw "ESP-NOW $Role flash failed with exit code $LASTEXITCODE"
    }
}
