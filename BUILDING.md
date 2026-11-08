# Building the CAN FD CANHack firmware

This builds the MicroPython firmware for a Raspberry Pi Pico (RP2040) with the
CAN FD extension of the CANHack toolkit (fixed bit rates: 500 kbit/s arbitration,
2 Mbit/s data). Hardware: a plain Pico plus a CAN transceiver on GP21 (RX) / GP22
(TX) — the "CANHack board" wiring.

## Prerequisites

Host tools (Debian/Ubuntu package names in brackets):

- `git` [git]
- `cmake` >= 3.13 [cmake]
- `make` and a host C/C++ compiler [build-essential]
- `python3` [python3]
- The **Arm GNU bare-metal toolchain** `arm-none-eabi-gcc` — **recommended version
  13.3.rel1**. Newer host GCCs are fine for the host tools, but the firmware itself
  must be built with the Arm toolchain.

### Why 13.3.rel1 for the Arm toolchain

MicroPython v1.22.2 is from early 2024 and does not compile cleanly with
`arm-none-eabi-gcc` 15+ (new `-Werror` warnings). Use the 13.x Arm release. On some
distros the packaged `gcc-arm-none-eabi` is too new or ships without newlib
(`nosys.specs` missing) — in that case install the official Arm tarball:

1. Download "Arm GNU Toolchain → AArch32 bare-metal target (arm-none-eabi)",
   x86_64 Linux host, from developer.arm.com.
2. Unpack it somewhere, e.g. `/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi`.

## Configuring the compiler location

Either put `arm-none-eabi-gcc` on your `PATH`, or point the build at the toolchain
directory (its `bin/` is prepended to `PATH` for the build):

```sh
export ARM_TOOLCHAIN_PATH=/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi
```

Note: `export ARM_TOOLCHAIN_PATH=…` only sets the variable — it does **not** change your
interactive shell's `PATH`. `build.sh` prepends `$ARM_TOOLCHAIN_PATH/bin` to `PATH`
itself, so it uses that toolchain even if a different `arm-none-eabi-gcc` (e.g. a newer
distro one) comes first in your shell. Running `arm-none-eabi-gcc --version` by hand may
therefore still show the other version — that is fine. Confirm the build uses the right
one by watching the line build.sh prints:

```
>> toolchain   : /opt/arm-gnu-toolchain-13.3.rel1-.../bin/arm-none-eabi-gcc  (13.3.1)
```

It must say **13.3.x**. The firmware does **not** build with arm-none-eabi-gcc 15+. If
you also want your interactive shell to use it (for manual checks):

```sh
export PATH=$ARM_TOOLCHAIN_PATH/bin:$PATH
hash -r
arm-none-eabi-gcc --version                       # now 13.3.x
arm-none-eabi-gcc -print-file-name=nosys.specs    # must print a full path, not just "nosys.specs"
```

## Build

From the repository root:

```sh
./build.sh
```

That runs the whole chain:

1. fetches MicroPython v1.22.2 into `CANPico/firmware/micropython` (git submodule)
   if it is not there yet,
2. fetches the port submodules (pico-sdk, tinyusb, …) via `make submodules`,
3. symlinks this repo's firmware files (`CANPico/firmware/src/…`) into the port
   (no copying — edits take effect directly),
4. enables the modules, stamps the git hash, does a clean configure (fresh build
   timestamp) and builds,
5. writes **`fw_canhack_RPI_PICO.uf2`** to the repository root.

Flash it by holding BOOTSEL while plugging in the Pico and copying the `.uf2` onto
the `RPI-RP2` drive. Then confirm which build is running:

```python
import rp2
rp2.CANHack(bit_rate=500).build_info()
# -> 'canhack_fd <githash> built <YYYY-MM-DD HH:MM:SS> UTC'
```

## Options (environment variables)

| Variable | Default | Meaning |
|---|---|---|
| `ARM_TOOLCHAIN_PATH` | — | dir of the Arm toolchain; its `bin/` is prepended to `PATH` |
| `BOARD` | `RPI_PICO` | target board (`RPI_PICO` or `RPI_PICO_W`) |
| `MODULES` | `CANHACK_BOARD CANHACK_FD` | firmware modules to enable |
| `MPY_DIR` | (the submodule) | use an existing MicroPython checkout; nothing is downloaded |
| `JOBS` | `nproc` | parallel build jobs |
| `CFLAGS_EXTRA` | `-Wno-error` | passed to the host `mpy-cross` build (needed on host GCC ≥ 15) |

Examples:

```sh
# Pico W
BOARD=RPI_PICO_W ./build.sh

# reuse a MicroPython tree you already have (skips the download)
MPY_DIR=~/gh/micropython ./build.sh

# CANPico board that also has the MCP2517FD controller
MODULES="CANPICO_BOARD CANHACK CANHACK_FD" ./build.sh
```

## Notes

- The 250 MHz overclock runs the RP2040 outside its specified range; `main.c` raises
  the core voltage (VREG 1.20 V) before the clock switch for stability. This is
  required to meet the 2 Mbit/s data-phase timing.
- The MicroPython submodule tree will show the overlay files as modified/symlinked —
  that is expected; do not commit inside the submodule.
- For the current, hardware-verified feature status, known limits and a scope/PCAN
  test plan, see `CANPico/firmware/CANFD_STATUS.md`.
