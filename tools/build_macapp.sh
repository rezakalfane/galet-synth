#!/usr/bin/env bash
# Package the GaletSynth voice-tuner GUI (tools/voicelab_gui.py) as a standalone
# macOS .app using PyInstaller, then (optionally) install it to /Applications.
#
#   tools/build_macapp.sh            # build dist/VoiceLab.app
#   tools/build_macapp.sh --install  # build, then copy to /Applications
#
# Self-contained: bundles Python + PySide6 + pyserial + the galetsynth package,
# so the app keeps working even if this repo is moved or the .venv is removed.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"
VENV="$ROOT/.venv"
APP="VoiceLab"

[ -x "$VENV/bin/python" ] || { echo "No .venv — run: python3 -m venv .venv && .venv/bin/pip install pyserial PySide6"; exit 1; }
"$VENV/bin/pip" install --quiet --upgrade pyinstaller

ICON="$ROOT/tools/voicelab.icns"
[ -f "$ICON" ] || "$VENV/bin/python" tools/make_icon.py   # regenerate if missing

# Keep PyInstaller's work/spec out of the tracked firmware build/ dir.
OUT="$ROOT/build-macapp"
"$VENV/bin/pyinstaller" \
  --name "$APP" \
  --windowed \
  --noconfirm --clean \
  --paths tools \
  --collect-submodules galetsynth \
  --icon "$ICON" \
  --osx-bundle-identifier com.galetsynth.voicelab \
  --distpath "$OUT/dist" \
  --workpath "$OUT/work" \
  --specpath "$OUT" \
  tools/voicelab_gui.py

echo "Built: $OUT/dist/$APP.app"

if [ "${1:-}" = "--install" ]; then
    DEST="/Applications/$APP.app"
    rm -rf "$DEST"
    cp -R "$OUT/dist/$APP.app" "$DEST"
    echo "Installed: $DEST"
fi
