#!/usr/bin/env bash
# Build the browser installer site: one merged firmware image per board role
# plus ESP Web Tools manifests, next to install/index.html.
#
#   (cd firmware/targets/esp32s3 && pio run -e pod -e ground)
#   tools/make_installer.sh [OUT_DIR]          # default build/site
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FW="$ROOT/firmware/targets/esp32s3/.pio/build"
OUT="${1:-$ROOT/build/site}"
PIO="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}"
BOOT_APP0="$PIO/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
if python3 -c "import esptool" 2>/dev/null; then ESPTOOL=(python3 -m esptool)
else ESPTOOL=(python3 "$PIO/packages/tool-esptoolpy/esptool.py"); fi
VERSION="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo dev)"
mkdir -p "$OUT"

for role in pod ground; do
  B="$FW/$role"
  [ -f "$B/firmware.bin" ] || { echo "missing $B/firmware.bin: build with pio first"; exit 1; }
  # ESP32-S3 layout: bootloader 0x0, partition table 0x8000, OTA data 0xe000, app 0x10000.
  "${ESPTOOL[@]}" --chip esp32s3 merge_bin -o "$OUT/guided-recovery-$role.bin" \
    --flash_mode keep --flash_freq keep --flash_size 8MB \
    0x0 "$B/bootloader.bin" 0x8000 "$B/partitions.bin" 0xe000 "$BOOT_APP0" 0x10000 "$B/firmware.bin" >/dev/null
  name="Guided Recovery OS: S pod"
  [ "$role" = ground ] && name="Guided Recovery ground receiver"
  cat > "$OUT/manifest-$role.json" <<JSON
{
  "name": "$name",
  "version": "$VERSION",
  "new_install_prompt_erase": true,
  "builds": [
    { "chipFamily": "ESP32-S3", "parts": [ { "path": "guided-recovery-$role.bin", "offset": 0 } ] }
  ]
}
JSON
done
cp "$ROOT/install/index.html" "$OUT/index.html"
sed -i "s/__VERSION__/$VERSION/g" "$OUT/index.html"
ls -l "$OUT"
