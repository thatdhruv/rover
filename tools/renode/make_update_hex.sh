#!/bin/bash
# Turn an unsigned app binary into a hex file that stages it in slot 1 as a *test* update
# (swap requested, not confirmed), for the rollback test.
#   make_update_hex.sh <zephyr.bin> <out.hex> [version]
set -euo pipefail
bin="$1"; out="$2"; version="${3:-0.2.0}"
root="$(cd "$(dirname "$0")/../.." && pwd)"
imgtool="${IMGTOOL:-imgtool}"
key="${KEY:-$root/bootloader/mcuboot/root-ec-p256.pem}"
# slot1 is at 0x08060000, 256 KiB (see the F429ZI partitions); header size 0x200 matches
# CONFIG_ROM_START_OFFSET of MCUboot-enabled builds
"$imgtool" sign --key "$key" --header-size 0x200 --align 8 --version "$version" \
  --slot-size 0x40000 --pad --hex-addr 0x08060000 "$bin" "$out"
echo "wrote $out"
