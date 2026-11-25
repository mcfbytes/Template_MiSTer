#!/bin/sh
# ascal regression bench: build tb_ascal against an ascal source, run every config (or the
# named ones) in parallel, and write <out>/<config>/{vid,avl}.txt plus <out>/SHA256SUMS.
#
#   run.sh [-j jobs] [-o outdir] [-a ascal.vhd] [-c] [config ...]
#     -a  the ascal under test (default: ../../sys/ascal.vhd)
#     -c  compare against golden.sha256 (recorded from stock ascal): rc 1 if any <config>/vid
#         (every output pin, line-buffer write and written phase) or avl.txt differs. The
#         <config>/phase digest (the phase of every cycle of a line copy, written or not) is
#         reported but not gated: a restructure that accumulates rather than divides follows
#         a changed o_hsize some cycles later, which only a copy straddling a mode change sees.
#
# Needs GHDL (4.x or 5.x; the LLVM or GCC backend is several times faster than mcode) and
# python3. GHDL=... picks the binary. GHDL_IEEE_SRC=... points at GHDL's ieee2008 sources
# when they are not found next to its libraries; without them the bench runs, only slower.
set -eu
if [ "${1:-}" = --one ]; then # internal: one run, from xargs; OUT and SIM in the environment
	name=$2; shift 2
	mkdir -p "$OUT/$name" && cd "$OUT/$name"
	s=$(date +%s)
	if $SIM -gOUTDIR=. "$@" --ieee-asserts=disable > run.log 2>&1; then rc=0; else rc=$?; fi
	echo "$name rc=$rc $(( $(date +%s) - s ))s"
	exit $rc
fi
HERE=$(cd "$(dirname "$0")" && pwd)
ASCAL=$HERE/../../sys/ascal.vhd
OUT=$HERE/out
JOBS=4
CHECK=0
while getopts j:o:a:c f; do
	case $f in
		j) JOBS=$OPTARG ;; o) OUT=$OPTARG ;; a) ASCAL=$OPTARG ;; c) CHECK=1 ;; *) exit 2 ;;
	esac
done
shift $((OPTIND - 1))
GHDL=${GHDL:-ghdl}
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
B=$OUT/build
mkdir -p "$B/work"

# The design under test: simulation range fixes, then the observation probes (both are
# mechanical and identical for every ascal; see simfix.sed and probe.sed).
sed -f "$HERE/simfix.sed" "$ASCAL" | sed -f "$HERE/probe.sed" > "$B/ascal.vhd"
for n in $(grep -c 'p_hfrac1 <= o_hfrac(1);' "$B/ascal.vhd") $(grep -c '^USE work.ascal_probe_pkg.ALL;' "$B/ascal.vhd"); do
	[ "$n" = 1 ] || { echo "run.sh: probe insertion failed" >&2; exit 2; }
done

# A numeric_std with exact integer fast paths (fastnum.py), compiled as library ieee
LIBDIR=$($GHDL --dispconfig 2>/dev/null | sed -n 's/^library directory: //p')
SRC=${GHDL_IEEE_SRC:-}
for d in "$LIBDIR/../../src/ieee2008" "$LIBDIR/../src/ieee2008" "$LIBDIR/src/ieee2008"; do
	[ -z "$SRC" ] && [ -f "$d/numeric_std-body.vhdl" ] && SRC=$d
done
P=
if [ -n "$SRC" ]; then
	python3 "$HERE/fastnum.py" "$SRC" "$B/fastsrc"
	mkdir -p "$B/ieee"
	(cd "$B/ieee" && $GHDL -a --std=08 -O2 --work=ieee --workdir=. ../fastsrc/std_logic_1164.vhdl \
		../fastsrc/std_logic_1164-body.vhdl ../fastsrc/numeric_std.vhdl ../fastsrc/numeric_std-body.vhdl)
	P=-P$B/ieee
else
	echo "run.sh: GHDL's ieee2008 sources not found, running with the stock numeric_std" >&2
fi

cd "$B/work"
$GHDL -a --std=08 -O2 -frelaxed -Wno-hide $P "$HERE/ascal_probe_pkg.vhd" ../ascal.vhd "$HERE/tb_ascal.vhd"
$GHDL -e --std=08 -O2 -frelaxed $P tb_ascal
if [ -x ./tb_ascal ]; then SIM="$B/work/tb_ascal"; else SIM="$GHDL -r --std=08 -frelaxed --workdir=$B/work $P tb_ascal"; fi

preset() {
	case $1 in
		@1080) echo "-gO_CLK_PS=6734 -gHTOTAL=2200 -gHSSTART=2008 -gHSEND=2052 -gHDISP=1920 -gVTOTAL=1125 -gVSSTART=1084 -gVSEND=1089 -gVDISP=1080 -gVMIN=0 -gVMAX=1079" ;;
		@720) echo "-gO_CLK_PS=13468 -gHTOTAL=1650 -gHSSTART=1390 -gHSEND=1430 -gHDISP=1280 -gVTOTAL=750 -gVSSTART=725 -gVSEND=730 -gVDISP=720 -gVMIN=0 -gVMAX=719" ;;
		@480) echo "-gO_CLK_PS=39722 -gHTOTAL=800 -gHSSTART=656 -gHSEND=752 -gHDISP=640 -gVTOTAL=525 -gVSSTART=490 -gVSEND=492 -gVDISP=480 -gVMIN=0 -gVMAX=479" ;;
		@nes) echo "-gIN_W=256 -gIN_H=224 -gIN_HTOT=341 -gIN_VTOT=262 -gI_CE_DIV=2" ;;
		@gen) echo "-gIN_W=320 -gIN_H=224 -gIN_HTOT=427 -gIN_VTOT=262 -gI_CE_DIV=2" ;;
		@psx) echo "-gIN_W=563 -gIN_H=240 -gIN_HTOT=640 -gIN_VTOT=263 -gI_CE_DIV=1" ;;
		@pal576i) echo "-gIN_W=704 -gIN_H=288 -gIN_HTOT=864 -gIN_VTOT=312 -gIN_INTER=true -gI_CE_DIV=1" ;;
		@ntsc480i) echo "-gIN_W=640 -gIN_H=240 -gIN_HTOT=780 -gIN_VTOT=262 -gIN_INTER=true -gI_CE_DIV=1" ;;
		@vga) echo "-gIN_W=640 -gIN_H=480 -gIN_HTOT=800 -gIN_VTOT=525 -gI_CE_DIV=1" ;;
		@svga) echo "-gIN_W=800 -gIN_H=600 -gIN_HTOT=1056 -gIN_VTOT=628 -gI_CE_DIV=1" ;;
		*) echo "$1" ;;
	esac
}

# One line per run: <name> <generics...>
LIST=$OUT/runs.txt
: > "$LIST"
grep -v '^#' "$HERE/configs.txt" | grep '|' | while IFS='|' read -r name gens; do
	name=$(echo "$name" | tr -d ' ')
	if [ $# -gt 0 ]; then
		case " $* " in *" $name "*) ;; *) continue ;; esac
	fi
	g=
	for t in $gens; do g="$g $(preset "$t")"; done
	echo "$name$g" >> "$LIST"
done

rc=0
OUT=$OUT SIM=$SIM xargs -P "$JOBS" -L 1 "$HERE/run.sh" --one < "$LIST" || rc=1

cd "$OUT"
digest() { sha256sum | cut -d' ' -f1; }
cut -d' ' -f1 "$LIST" | while read -r name; do
	echo "$(awk '$1=="L"{NF=10} 1' "$name/vid.txt" | digest)  $name/vid"
	echo "$(awk '$1=="L"{print $2, $3, $11} $1=="F"' "$name/vid.txt" | digest)  $name/phase"
	echo "$(digest < "$name/avl.txt")  $name/avl.txt"
done > SHA256SUMS
if [ $CHECK = 1 ]; then
	bad=$(grep -v '/phase$' SHA256SUMS | grep -vxFf "$HERE/golden.sha256" | sed 's/^[0-9a-f]*  //' || true)
	soft=$(grep '/phase$' SHA256SUMS | grep -vxFf "$HERE/golden.sha256" | sed 's/^[0-9a-f]*  //' || true)
	[ -n "$soft" ] && echo "run.sh: copy-cycle phase differs (not gated):" $soft
	if [ -n "$bad" ]; then
		echo "run.sh: MISMATCH against golden.sha256:" $bad; rc=1
	else
		echo "run.sh: all $(wc -l < "$LIST") configs match golden.sha256 (vid, avl)"
	fi
fi
exit $rc
