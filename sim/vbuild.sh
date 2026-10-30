#!/bin/sh
# Builds a Verilator testbench with zig (no make, no gcc). Git Bash.
#
#   sim/vbuild.sh <name> <top module> <testbench.cpp> <rtl files...> [-- <extra C files>]
#
# makes sim/out/<name>/<name>.exe. The emulator's sources (emu/src, all but
# main.c and gif.c) are always linked in, so a testbench can run the C model
# beside the RTL. Verilator: OSS CAD Suite in $OSSCAD (default
# $HOME/tools/oss-cad-suite). Set OSSCAD to the OSS CAD Suite folder if it is elsewhere.
set -e
cd "$(dirname "$0")/.."
OSSCAD=${OSSCAD:-$HOME/tools/oss-cad-suite}
[ -d "$OSSCAD/bin" ] || { echo "error: OSS CAD Suite not found at $OSSCAD; set OSSCAD to its folder" >&2; exit 1; }
ZIG=${ZIG:-zig}
name=$1 top=$2 tb=$3
shift 3
rtl="" extra=""
while [ $# -gt 0 ]; do
    if [ "$1" = "--" ]; then shift; extra="$*"; break; fi
    rtl="$rtl $1"; shift
done
out=sim/out/$name
mkdir -p $out/obj
export PATH="$OSSCAD/bin:$OSSCAD/lib:$PATH"
export VERILATOR_ROOT=$(cygpath -m "$OSSCAD/share/verilator")
VINC=$OSSCAD/share/verilator/include
"$OSSCAD/bin/verilator_bin.exe" --cc -O3 --x-assign fast --x-initial fast --noassert \
    -Wall -Wno-fatal -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-PINCONNECTEMPTY \
    -Icore/rtl -Icore/rtl/cpu -Icore/rtl/periph \
    --top-module $top --Mdir $out/obj --prefix V$top $rtl > $out/verilator.log 2>&1 \
    || { cat $out/verilator.log; exit 1; }
grep -E "%Warning|%Error" $out/verilator.log | head -40 || true

E=emu/src
EMU="$E/rv32.c $E/bus.c $E/machine.c $E/st7735.c $E/audio.c $E/piezo_filter.c
     $E/loader.c $E/sdspi.c $E/sdcard.c $E/bootloader_image.c"
# the emulator as C, once per testbench
objs=""
for c in $EMU $extra; do
    o=$out/obj/c_$(basename "${c%.c}").o
    if [ ! -f $o ] || [ $c -nt $o ]; then
        $ZIG cc -target x86_64-windows-gnu -std=c11 -O2 -I$E -DSDCARD_LABEL='"CHGAME"' -c $c -o $o
    fi
    objs="$objs $o"
done
# the model, the Verilator runtime and the testbench as C++
CXX="$ZIG c++ -target x86_64-windows-gnu -std=c++17 -O2 -I$out/obj -I$VINC -I$VINC/vltstd -I$E -Isim -DVL_IGNORE_UNKNOWN_ARCH -Wno-macro-redefined"
for f in $out/obj/V${top}*.cpp $VINC/verilated.cpp $VINC/verilated_threads.cpp sim/vl_glue.cpp; do
    o=$out/obj/x_$(basename "${f%.cpp}").o
    if [ ! -f $o ] || [ $f -nt $o ]; then $CXX -c $f -o $o & fi
done
wait
$CXX -c $tb -o $out/obj/tb.o
$CXX -o $out/$name.exe $out/obj/tb.o $out/obj/x_*.o $objs
echo $out/$name.exe
