#!/bin/sh
# Builds the tools with zig cc. ./arm/build.sh [pc|arm|all]
#   pc   build/chgbench.exe, build/chgtrace.exe   (Windows x86_64)
#   arm  build/chgbench-arm                       (MiSTer's Linux: Cortex-A9, ARMv7 hard float,
#                                                  static musl: needs nothing from the MiSTer)
set -e
cd "$(dirname "$0")/.."
ZIG=${ZIG:-zig}
E=emu/src
EMU="$E/rv32.c $E/bus.c $E/machine.c $E/st7735.c $E/audio.c $E/piezo_filter.c
     $E/loader.c $E/sdspi.c $E/sdcard.c $E/bootloader_image.c"
CFLAGS="-std=c11 -O3 -Wall -Wno-unused-parameter -I$E -DSDCARD_LABEL=\"CHGAME\""
mkdir -p build
what=${1:-all}
if [ "$what" = pc ] || [ "$what" = all ]; then
    $ZIG cc -target x86_64-windows-gnu $CFLAGS -o build/chgbench.exe arm/bench.c $EMU
    $ZIG cc -target x86_64-windows-gnu $CFLAGS -o build/chgtrace.exe arm/trace.c $EMU
    $ZIG cc -target x86_64-windows-gnu $CFLAGS -o build/chgsdimg.exe arm/sdimg.c $EMU
    echo build/chgbench.exe build/chgtrace.exe build/chgsdimg.exe
fi
if [ "$what" = arm ] || [ "$what" = all ]; then
    $ZIG cc -target arm-linux-musleabihf -mcpu=cortex_a9 -static -s -D_GNU_SOURCE $CFLAGS -o build/chgbench-arm arm/bench.c $EMU
    echo build/chgbench-arm
fi
