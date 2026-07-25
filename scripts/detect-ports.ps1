$ErrorActionPreference = 'Stop'

Get-CimInstance Win32_PnPEntity |
    Where-Object {
        $_.Name -match 'CH343|USB JTAG|USB Serial|ESP32|COM\d+'
    } |
    Select-Object Status, Name, PNPDeviceID |
    Format-Table -AutoSize

