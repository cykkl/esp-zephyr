param(
    [string]$Port = 'COM17',
    [int]$BaudRate = 115200
)

$ErrorActionPreference = 'Stop'
$Python = 'F:\zephyr\.venv\Scripts\python.exe'

if (-not (Test-Path -LiteralPath $Python)) {
    throw "Zephyr Python environment not found: $Python"
}

& $Python -m serial.tools.miniterm $Port $BaudRate

