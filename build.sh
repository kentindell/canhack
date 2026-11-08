#!/usr/bin/env bash
#
# Top-level build for the CAN FD CANHack firmware.
#
# Runs the whole chain: fetches MicroPython (v1.22.2, as a git submodule) and
# its port submodules (pico-sdk, tinyusb, ...), overlays this repo's firmware
# files via symlinks (no copying), builds, and drops the .uf2 in this directory.
#
# Quick start:
#   export ARM_TOOLCHAIN_PATH=/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi
#   ./build.sh
#   # -> fw_canhack_RPI_PICO.uf2
#
# See BUILDING.md for prerequisites and all options.
#
# Environment:
#   ARM_TOOLCHAIN_PATH   dir of the Arm GNU bare-metal toolchain (its bin/ is
#                        prepended to PATH). Not needed if arm-none-eabi-gcc is
#                        already on PATH.
#   BOARD=RPI_PICO       target board (RPI_PICO or RPI_PICO_W)
#   MODULES="CANHACK_BOARD CANHACK_FD"   firmware modules to enable
#   MPY_DIR=/path        use an existing MicroPython checkout instead of the
#                        bundled submodule (then nothing is downloaded)
#   JOBS=N               parallel build jobs (default: nproc)
#   CFLAGS_EXTRA         passed to the host mpy-cross build (default -Wno-error,
#                        needed on host GCC >= 15)
#
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
SRC="$REPO/CANPico/firmware/src"
BOARD="${BOARD:-RPI_PICO}"
MODULES="${MODULES:-CANHACK_BOARD CANHACK_FD}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"
export CFLAGS_EXTRA="${CFLAGS_EXTRA:--Wno-error}"

# --- toolchain -------------------------------------------------------------
if [ -n "${ARM_TOOLCHAIN_PATH:-}" ]; then
    export PATH="$ARM_TOOLCHAIN_PATH/bin:$PATH"
fi
missing=0
for t in arm-none-eabi-gcc cmake make git python3; do
    command -v "$t" >/dev/null 2>&1 || { echo "error: '$t' not found in PATH" >&2; missing=1; }
done
if [ "$missing" != 0 ]; then
    echo "" >&2
    echo "See BUILDING.md for prerequisites. For the Arm compiler, either put it on PATH or set" >&2
    echo "  export ARM_TOOLCHAIN_PATH=/opt/arm-gnu-toolchain-<version>-x86_64-arm-none-eabi" >&2
    exit 1
fi

# --- MicroPython checkout --------------------------------------------------
if [ -n "${MPY_DIR:-}" ]; then
    MPY_DIR="$(cd "$MPY_DIR" && pwd)"
else
    MPY_DIR="$REPO/CANPico/firmware/micropython"
    if [ ! -e "$MPY_DIR/ports/rp2/Makefile" ]; then
        echo ">> fetching MicroPython submodule (v1.22.2)"
        git -C "$REPO" submodule update --init CANPico/firmware/micropython
    fi
fi
RP2="$MPY_DIR/ports/rp2"
[ -d "$RP2" ] || { echo "error: no MicroPython rp2 port at $RP2" >&2; exit 1; }

echo ">> MicroPython : $MPY_DIR"
echo ">> toolchain   : $(command -v arm-none-eabi-gcc)  ($(arm-none-eabi-gcc -dumpversion))"
echo ">> board       : $BOARD   modules: $MODULES"

echo ">> fetching port submodules (pico-sdk, tinyusb, ...)"
make -C "$RP2" BOARD="$BOARD" submodules >/dev/null

echo ">> linking overlay from $SRC"
link() { ln -sfn "$1" "$2"; }
link "$SRC/CMakeLists.txt"              "$RP2/CMakeLists.txt"
link "$SRC/main.c"                      "$RP2/main.c"
link "$SRC/modrp2.c"                    "$RP2/modrp2.c"
mkdir -p "$RP2/canis"
for f in "$SRC"/canis/*; do link "$f" "$RP2/canis/$(basename "$f")"; done
link "$SRC/tusb_config.h"               "$MPY_DIR/shared/tinyusb/tusb_config.h"
link "$SRC/mp_usbd_descriptor.c"        "$MPY_DIR/shared/tinyusb/mp_usbd_descriptor.c"
link "$SRC/RPI_PICO/mpconfigboard.h"    "$RP2/boards/RPI_PICO/mpconfigboard.h"
link "$SRC/RPI_PICO_W/mpconfigboard.h"  "$RP2/boards/RPI_PICO_W/mpconfigboard.h"

echo ">> enabling modules"
( cd "$RP2" && rm -f CHV_DEFCON30_BADGE CANHACK_BOARD CANPICO_BOARD CANHACK CANHACK_FD MIN_PROTOCOL
  for m in $MODULES; do touch "$m"; done )

hash="$(git -C "$REPO" rev-parse --short HEAD)"
git -C "$REPO" diff --quiet -- "$SRC" 2>/dev/null || hash="$hash+dirty"
export CANHACK_FD_GITHASH="$hash"
echo ">> git stamp   : $CANHACK_FD_GITHASH"

echo ">> building (clean configure so the build timestamp is fresh)"
rm -rf "$RP2/build-$BOARD"
make -C "$RP2" BOARD="$BOARD" -j"$JOBS"

out="$REPO/fw_canhack_${BOARD}.uf2"
cp "$RP2/build-$BOARD/firmware.uf2" "$out"
echo ""
echo ">> done: $out"
echo "   flash it (hold BOOTSEL), then check:  rp2.CANHack(bit_rate=500).build_info()"
