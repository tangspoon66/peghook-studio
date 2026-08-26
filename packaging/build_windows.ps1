$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $Root
if (Test-Path build) { Remove-Item build -Recurse -Force }
if (Test-Path dist) { Remove-Item dist -Recurse -Force }
python -m PyInstaller --noconfirm --clean packaging/PegHookStudio.spec
$Archive = "dist/PegHookStudio-win-x64.zip"
if (Test-Path $Archive) { Remove-Item $Archive -Force }
Compress-Archive -Path "dist/PegHookStudio\*" -DestinationPath $Archive
Write-Host "Created $Archive"
