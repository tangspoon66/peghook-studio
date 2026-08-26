#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
rm -rf build dist
python -m PyInstaller --noconfirm --clean packaging/PegHookStudio.spec

ARCH="$(uname -m)"
DMG="dist/PegHookStudio-mac-${ARCH}.dmg"
rm -f "$DMG"
hdiutil create -volname "PegHook Studio" -srcfolder "dist/PegHookStudio.app" -ov -format UDZO "$DMG"
echo "Created $DMG"
