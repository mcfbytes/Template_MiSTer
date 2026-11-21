#!/bin/sh
# LW bridge framework testbench (tb/lw_tb.sv, tb/top_tb.sv and the scripts below), one run per arm:
#   S  the off-state guard (tb/offstate.sh): without MISTER_LW, sys/ reads upstream's text under every upstream
#      macro set; every LW-only file is empty without its macro and keeps no macro defined; sys.qip lists
#      sys/lw_bridge.sdc after sys_top.sdc, and that file constrains nothing without MISTER_LW; Template.sv reads
#      upstream's text without MISTER_LW, and its CONF_STR value is upstream's, or with MISTER_LW the example's
#   N  static checks on the preprocessed top level (tb/static.sh): the GPI terms with both LW instances or
#      neither, the f2h_irq lines, the LW nets arm E checks, in every configuration arm E builds; and the
#      crossing structure: every crossing built from the cells of sys/lw_cdc.sv, every source register named;
#      and the directory's lines: each entry's LINE word one ordinal of its own, picking that entry's line;
#      and sys/lw_bridge.sdc: both ends on every bound, every pattern matched, every crossing source covered;
#      and the wording of the LW files, tb/ and docs/LW_BRIDGE.md (the guide also in the host interface's terms)
#   E  sys_top.v elaborated (tb/top_tb.sv, against the stand-ins of tb/sys_stubs.sv) in every configuration of
#      "tb/static.sh --configs": off and each of the 32 LW tier sets, each with no upstream macro,
#      MISTER_DEBUG_NOHDMI and MISTER_DUAL_SDRAM (99). Fails if it does not elaborate, if an LW run warns of an
#      implicit net or a port width that the off run of the same upstream variant does not, or if a check of
#      top_tb fails (an LW net that reads z, an HPS_BUS bit driven other than as the bit map says, among them)
#   Z  MISTER_LW alone (the floor): no block listed, the enable and the firewall whole
#   1  LW_DIAG=1 (debug build: diagnostics compiled in), every tier but the lane
#   0  LW_DIAG=0 (shipping build), every tier but the lane
#   M  LW_DIAG=0 with MENU_CORE (the OSD's header rows only: stored over LW, drawn as over SPI)
#   F  LW_DIAG=0 with the floor, PAD and EVT (the Template's own configuration): no SECTOR files, no muxes, no OSD
#   G  LW_DIAG=0 with the SECTOR block at the default geometry (128-word primary, 256-word save slots, 32-bit LBAs)
#   W  LW_DIAG=0 with hps_io at WIDE=0 (byte strobes), a 4096-word primary on hps_io slot 9, save slots of 510 and
#      256 words with 32-bit LBAs on slots 12 and 15, strobes 5 cycles apart (LW_TB_GEOMW): T-SEC6
#   L  LW_DIAG=0 with every tier, the frame lane (MISTER_LW_LANE) included: the link nets through lw_link
#   X  each tier macro without MISTER_LW: the compile of hps_io fails, naming lw_tier_needs_MISTER_LW (and
#      compiles without the macro, and with MISTER_LW); and each unservable SECTOR geometry: the compile of the
#      window fails, naming lw_sector_unservable_geometry (and the servable edges compile); and the Template's
#      block reader outside hps_io slots 1-9: emu's compile fails, naming lw_template_block_slot_1_to_9
#   T  the short tier run (LW_TB_TIER) in each of the 32 tier sets: the directory, the decode, the causes and
#      lines, and the linked blocks' OWNED and OPEN bits of the set built
#   B  the short tier run with every tier and a core side that leaves a piece out: a 46-bit bus at the core
#      (LW_TB_BUS46), no SECTOR mux (LW_TB_NOMUX), save slot 1's mux missing (LW_TB_NOSAVE1), hps_io's commit
#      mark missing (LW_TB_NOMARK)
#   R12 R20 R21 R33 R53 R62 R100 R150  arm L with clk_sys at 12.0, 20.0, 21.477, 33.8688, 53.69, 62.5, 100.0 and
#      150.0 MHz (the other arms run at 33.8688 MHz without the models); LW_TB_SYS_HP_PS is the half period in ps,
#      plus 50 ppm. Each runs
#      with the crossing cells' delay models (LW_TB_CDC, sys/lw_cdc.sv) under each seed of LW_TB_SEEDS (1 2 3)
#   V  front-end lint: slang elaborates every LW module at its defaults (the vendor atom left unknown), and
#      the window with every block, at both LW_DIAG values, with every tier macro; and the floor modules with
#      MISTER_LW alone. Any error fails (a name used before its declaration is one, which the Icarus arms
#      accept). Without slang on PATH the arm reports SKIPPED and does not fail.
#   H  the HPS-side probe, tb/hps/lw_probe.c, compiled with warnings as errors by the host's C compiler (cc, or
#      $CC) and, where one is on PATH, by arm-linux-gnueabihf-gcc; the testbench never runs it. And its own logic
#      run on the build host: tb/hps/lw_probe_test.c includes the probe against a model of the window (its cases
#      in its header), compiled the same way and run once. It all runs before any container starts (the image
#      has no C compiler). Without a C compiler the arm reports SKIPPED and does not fail.
# LW_TB_ARMS selects arms (default: all); LW_TB_QUICK=1 runs S, N, E, F and 0. LW_TB_JOBS (8) runs at a time in
# arms E, T and B. Needs Icarus Verilog >= 12
# (11 cannot parse hps_io's default-less CONF_STR). Without a local one, set LW_TB_IMAGE to a container image
# that has it; the tree is mounted at /w.
# Logs go to $LW_TB_OUT (tb/out). The last lines are one result per arm; the exit status is 0 only if all pass.
cd "$(dirname "$0")/.." || exit 2
[ -n "$LW_TB_QUICK" ] && : ${LW_TB_ARMS:=S N E F 0}
ARMS=${LW_TB_ARMS:-S N E Z 1 0 M F G W L X T B R12 R20 R21 R33 R53 R62 R100 R150 H V}
O=${LW_TB_OUT:-tb/out}

# probe LOG: arm H, on the host. The log's first line names this run, so a run in the container reads only its own.
probe() {
  echo "run $LW_TB_HRUN" > "$1"; n=0; f=0
  W="-std=c11 -Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -pedantic -Werror -O2"
  for c in "${CC:-cc}" arm-linux-gnueabihf-gcc; do
    if ! command -v "$c" >/dev/null 2>&1; then
      [ $n -eq 0 ] && { echo "SKIPPED: no C compiler ($c) on PATH" >> "$1"; return 0; }
      continue
    fi
    for src in lw_probe lw_probe_test; do
      echo "-- $c: tb/hps/$src.c" >> "$1"; n=$((n + 1)); rm -f "$O/$src.$(basename "$c")"
      x=""; [ $src = lw_probe_test ] && x=-pthread
      if $c $W $x -o "$O/$src.$(basename "$c")" tb/hps/$src.c >> "$1" 2>&1; then
        echo "pass $c: compiles, no warning" >> "$1"
      else f=$((f + 1)); echo "FAIL $c: $src.c does not compile without a warning" >> "$1"
      fi
    done
  done
  # the probe's own logic, on the build host (the first compiler's binary)
  t=$O/lw_probe_test.$(basename "${CC:-cc}"); r=0
  if [ -x "$t" ]; then
    "$t" > "$O/lw_probe_test.log" 2>&1; st=$?
    cat "$O/lw_probe_test.log" >> "$1"
    r=$(sed -n 's/^lw_probe_test: \([0-9]*\) checks.*/\1/p' "$O/lw_probe_test.log"); nf=$(grep -c '^FAIL ' "$O/lw_probe_test.log")
    f=$((f + nf))
    if [ $nf -eq 0 ] && { [ $st -ne 0 ] || [ -z "$r" ]; }; then
      f=$((f + 1)); echo "FAIL lw_probe_test: status $st, no failed check named" >> "$1"
    fi
    r=${r:-0}
  fi
  if [ $f -eq 0 ]; then echo "PASSED: $n compiles, $r probe checks, 0 failures" >> "$1"
  else echo "FAILED: $n compiles, $r probe checks, $f failures" >> "$1"; fi
}
if [ -z "$LW_TB_INNER" ]; then
  LW_TB_HRUN=$$; export LW_TB_HRUN
  case " $ARMS " in *" H "*) mkdir -p "$O"; probe "$O/lw_probeH.log";; esac
fi

if [ -z "$LW_TB_INNER" ] && ! iverilog -V 2>/dev/null | grep -q 'version 1[2-9]'; then
  if [ -n "$LW_TB_IMAGE" ]; then
    sh tb/offstate.sh --extract   # arm S's upstream tree: the container has no git
    exec docker run --rm -u "$(id -u):$(id -g)" -e LW_TB_OUT -e LW_TB_ARMS -e LW_TB_QUICK -e LW_TB_SEEDS -e LW_TB_JOBS -e LW_TB_HRUN -e LW_TB_INNER=1 -v "$PWD:/w" -w /w \
      "$LW_TB_IMAGE" sh tb/run.sh
  fi
  echo "tb/run.sh: needs Icarus Verilog >= 12 on PATH, or LW_TB_IMAGE set to an image with it" >&2; exit 2
fi

# Expected failures, one "arm|case" per line, the case as its "-- " heading prints. An arm with entries passes
# only if exactly its listed cases fail: an unlisted failure, or a listed case that passes under every seed the
# arm runs, fails the arm.
# The list is empty.
XFAIL=''
export XFAIL

# lint LOG: arm V's slang runs; status 0 if every run elaborates without an error
LWF="sys/lw_cdc.sv sys/hps_lw_bridge.sv sys/lw_sys_window.sv sys/lw_pad.sv sys/lw_evt.sv sys/lw_sector.sv sys/lw_lane.sv sys/lw_osd.sv sys/lw_blk_mux.sv"
LWM="lw_sync lw_arst lw_qstat lw_xfer lw_fence hps_lw_bridge lw_axi_regbus lw_sys_window lw_pad lw_evt lw_sector lw_lane lw_link lw_osd lw_blk_mux lw_save_mux"
LW0="lw_sync lw_arst lw_qstat lw_xfer lw_fence hps_lw_bridge lw_axi_regbus lw_sys_window"
LWD="-DMISTER_LW -DMISTER_LW_PAD -DMISTER_LW_EVT -DMISTER_LW_SECTOR_SLOT=1 -DMISTER_LW_LANE -DMISTER_LW_OSD"
lint() {
  : > "$1"; st=0
  for d in 0 1; do
    T=""; for m in $LWM; do T="$T --top $m"; done
    T0=""; for m in $LW0; do T0="$T0 --top $m"; done
    for run in "$LWD|$T" "$LWD|--top lw_sys_window -G PAD_EN=1 -G EVT_EN=1 -G OSD_EN=1 -G SEC_EN=1 -G LANE_EN=1" \
               "-DMISTER_LW|$T0"; do
      m=${run%%|*}; t=${run#*|}
      echo "slang -DLW_DIAG=$d $m $t" >> "$1"
      slang -q --ignore-unknown-modules -DLW_DIAG=$d $m $t $LWF >> "$1" 2>&1 || st=1
    done
  done
  return $st
}

# elab LOG: arm E, one elaboration and simulation of tb/top_tb.sv per configuration; the log ends with one
# PASSED/FAILED line for the arm. Files from sys.qip and files.qip, less those sys_stubs.sv stands in for. The
# configurations run $LW_TB_JOBS (8) at a time, the three off ones first: each LW run's warnings are compared
# with those of the off run of its upstream variant.
ESTUB="sys/yc_out.sv sys/mcp23009.sv sys/audio_out.sv sys/f2sdram_safe_terminator.sv sys/ddr_svc.sv rtl/cos.sv"
elab() {
  E=$O/e; rm -rf "$E"; mkdir -p "$E"; : > "$1"
  printf '`define BUILD_DATE "000000"\n' > "$E/build_id.v"
  F=""
  for q in sys/sys.qip files.qip; do
    for f in $(tr -d '\r' < $q |
               sed -n 's/^set_global_assignment -name \(SYSTEM\)\{0,1\}VERILOG_FILE *\(\[file join \$::quartus(qip_path) \)\{0,1\}\([^] ]*\).*/\3/p'); do
      case $q in sys/*) f=sys/$f;; esac
      case " $ESTUB " in *" $f "*) ;; *) F="$F $f";; esac
    done
  done
  sh tb/static.sh --configs > "$E/configs" || { echo "FAILED: 1 (tb/static.sh --configs)" >> "$1"; return 1; }
  # one configuration: its section of the log in $E/<name>.log, its warnings in $E/<name>.w
  cat > "$E/one.sh" <<EOS
c=\$1; shift; d="\$*"; L="$E/\$c.log"
mkdir -p "$E/\$c"; : > "\$L"
sh tb/static.sh --nets "$E/\$c/lw_e_nets.vh" \$d >> "\$L" 2>&1 || echo "FAIL \$c: no net list" >> "\$L"
if ! iverilog -g2012 -Wimplicit \$d -I sys -I "$E" -I "$E/\$c" -s top_tb -o "$E/\$c/top.vvp" tb/top_tb.sv \\
     tb/sys_stubs.sv $F > "$E/\$c/err" 2>&1; then
  echo "FAIL \$c: does not elaborate" >> "\$L"
  grep -E 'error|sorry' "$E/\$c/err" | head -20 | sed 's/^/   /' >> "\$L"; exit 0
fi
grep -E 'warning: (implicit definition|Port .* expects)' "$E/\$c/err" | grep -v '^tb/' | sort -u > "$E/\$c.w"
echo "-- \$c: tb/top_tb.sv" >> "\$L"
vvp -n "$E/\$c/top.vvp" > "$E/\$c/log" 2>&1
sed -e "s/^-- /-- \$c: /" -e "s/^\\(PASSED\\|FAILED\\): /\$c: \\1: /" -e '/\\\$finish called/d' "$E/\$c/log" >> "\$L"
grep -Eq '^(PASSED|FAILED): [0-9]' "$E/\$c/log" ||
  printf -- '-- %s: tb/top_tb.sv ends\\nFAIL %s: tb/top_tb.sv gave no result\\n' "\$c" "\$c" >> "\$L"
EOS
  # (xargs -L joins a line that ends in a blank to the next one: no trailing blank)
  grep '^off' "$E/configs" | sed 's/|$//; s/|/ /' | xargs -P "${LW_TB_JOBS:-8}" -L 1 sh "$E/one.sh"
  grep -v '^off' "$E/configs" | sed 's/|$//; s/|/ /' | xargs -P "${LW_TB_JOBS:-8}" -L 1 sh "$E/one.sh"
  while IFS='|' read -r c d; do
    v=${c#*.}; [ "$v" = "$c" ] && v=""; b=off${v:+.$v}
    echo "-- $c: elaborates, with no implicit net and no port width beyond $b's ($d)" >> "$1"
    if [ -f "$E/$c.w" ] && [ "$c" != "$b" ]; then
      comm -13 "$E/$b.w" "$E/$c.w" | sed "s/^/FAIL $c: beyond $b: /" >> "$1"
    fi
    cat "$E/$c.log" >> "$1"
  done < "$E/configs"
  awk -v n="$(wc -l < "$E/configs")" '/^FAIL / { f++ } / PASSED: [0-9]+ checks/ { c += $3 }
    END { if (f) printf "FAILED: %d failures over %d configurations\n", f, n
          else printf "PASSED: %d configurations, %d checks\n", n, c }' "$1" > "$E/result"
  cat "$E/result" >> "$1"
}

# The testbench's files, for every lw_tb arm.
TBF="tb/lw_tb.sv tb/lw_stub_atom.sv sys/lw_cdc.sv sys/hps_lw_bridge.sv sys/lw_sys_window.sv sys/lw_pad.sv"
TBF="$TBF sys/lw_evt.sv sys/lw_sector.sv sys/lw_lane.sv sys/lw_osd.sv sys/lw_blk_mux.sv sys/hps_io.sv sys/osd.v"

# xarm LOG: arm X. hps_io.sv alone under a stand-in top, per tier macro: without MISTER_LW the compile fails and
# names the guard; without the macro, and with MISTER_LW and it, the same files compile.
xarm() {
  : > "$1"; n=0; f=0
  printf 'module lw_x_top; hps_io #(.CONF_STR("X;")) hps_io(); endmodule\n' > "$O/lw_x_top.sv"
  for m in MISTER_LW_PAD MISTER_LW_EVT MISTER_LW_SECTOR_SLOT=1 MISTER_LW_LANE MISTER_LW_OSD; do
    echo "-- $m without MISTER_LW" >> "$1"
    iverilog -g2012 -D$m -I sys -s lw_x_top -o "$O/lw_x.vvp" "$O/lw_x_top.sv" sys/hps_io.sv > "$O/lw_x.err" 2>&1
    r=$?; n=$((n + 1))
    if [ $r -ne 0 ] && grep -q 'lw_tier_needs_MISTER_LW' "$O/lw_x.err"; then echo "pass $m: stops, naming the guard" >> "$1"
    else f=$((f + 1)); echo "FAIL $m without MISTER_LW: status $r, the guard $(grep -c lw_tier_needs_MISTER_LW "$O/lw_x.err") times in the errors" >> "$1"
    fi
    echo "-- $m with MISTER_LW" >> "$1"
    iverilog -g2012 -DMISTER_LW -D$m -I sys -s lw_x_top -o "$O/lw_x.vvp" "$O/lw_x_top.sv" sys/hps_io.sv > "$O/lw_x.err" 2>&1
    r=$?; n=$((n + 1))
    if [ $r -eq 0 ]; then echo "pass $m with MISTER_LW: compiles" >> "$1"
    else f=$((f + 1)); echo "FAIL $m with MISTER_LW: does not compile" >> "$1"; head -5 "$O/lw_x.err" | sed 's/^/   /' >> "$1"
    fi
  done
  echo "-- no tier macro" >> "$1"
  iverilog -g2012 -I sys -s lw_x_top -o "$O/lw_x.vvp" "$O/lw_x_top.sv" sys/hps_io.sv > "$O/lw_x.err" 2>&1
  r=$?; n=$((n + 1))
  if [ $r -eq 0 ]; then echo "pass no macro: compiles" >> "$1"
  else f=$((f + 1)); echo "FAIL no macro: does not compile" >> "$1"; head -5 "$O/lw_x.err" | sed 's/^/   /' >> "$1"
  fi
  # the SECTOR geometry, through the window's parameters (sys_top.v maps the macros onto them one to one): each
  # unservable one stops the compile naming the guard; the servable edges, and an undeclared save slot whatever
  # its geometry, compile
  cat > "$O/lw_xg_top.sv" <<'EOS'
module lw_xg_top #(parameter [31:0] SLOT = 1, FRAME = 128, GAP = 2, S0EN = 1, S0SLOT = 2, S0WSRC = 32'h30000000,
                   S0FRAME = 256, S0LBA = 32, S1EN = 0, S1SLOT = 3, S1WSRC = 32'h30100000, S1FRAME = 256, S1LBA = 32);
lw_sys_window #(.SEC_EN(1), .SEC_SLOT(SLOT), .SEC_FRAME(FRAME), .SEC_GAP(GAP), .SAVE0_EN(S0EN), .SAVE0_SLOT(S0SLOT),
                .SAVE0_WSRC(S0WSRC), .SAVE0_FRAME(S0FRAME), .SAVE0_LBA_BITS(S0LBA), .SAVE1_EN(S1EN),
                .SAVE1_SLOT(S1SLOT), .SAVE1_WSRC(S1WSRC), .SAVE1_FRAME(S1FRAME), .SAVE1_LBA_BITS(S1LBA)) win();
endmodule
EOS
  XG="sys/lw_cdc.sv sys/lw_sys_window.sv sys/lw_sector.sv"
  while IFS='|' read -r want what p; do
    echo "-- geometry: $what" >> "$1"
    P=""; for a in $p; do P="$P -Plw_xg_top.$a"; done
    iverilog -g2012 -DMISTER_LW -DMISTER_LW_SECTOR_SLOT=1 $P -I sys -s lw_xg_top -o "$O/lw_xg.vvp" "$O/lw_xg_top.sv" \
      $XG > "$O/lw_xg.err" 2>&1
    r=$?; n=$((n + 1)); g=$(grep -c lw_sector_unservable_geometry "$O/lw_xg.err")
    if [ "$want" = bad ]; then
      if [ $r -ne 0 ] && [ "$g" -gt 0 ]; then echo "pass $what: stops, naming the guard" >> "$1"
      else f=$((f + 1)); echo "FAIL $what: status $r, the guard $g times in the errors" >> "$1"
      fi
    elif [ $r -eq 0 ]; then echo "pass $what: compiles" >> "$1"
    else f=$((f + 1)); echo "FAIL $what: does not compile" >> "$1"; head -5 "$O/lw_xg.err" | sed 's/^/   /' >> "$1"
    fi
  done <<'EOG'
bad|a primary frame of 0 words|FRAME=0
bad|a primary frame of 4097 words|FRAME=4097
bad|a gap of 1|GAP=1
bad|a gap of 16|GAP=16
bad|the primary on hps_io slot 16|SLOT=16
bad|a save-slot frame of 0 words|S0FRAME=0
bad|a save-slot frame of 511 words (the FIFO's entries - 1)|S0FRAME=511
bad|a save slot keeping 0 LBA bits|S0LBA=0
bad|a save slot keeping 33 LBA bits|S0LBA=33
bad|a save slot on hps_io slot 16|S0SLOT=16
bad|a save slot on the primary's hps_io slot|S0SLOT=1
bad|a save slot without an image base|S0WSRC=0
bad|two save slots on one hps_io slot|S1EN=1 S1SLOT=2
bad|save slot 1's frame of 511 words|S1EN=1 S1FRAME=511
bad|save slot 1 without an image base|S1EN=1 S1WSRC=0
ok|the default geometry|
ok|the edges: a 4096-word primary on slot 15, gap 15, save slots of 510 words keeping 1 and 32 LBA bits|FRAME=4096 GAP=15 SLOT=15 S0SLOT=0 S0FRAME=510 S0LBA=1 S1EN=1 S1SLOT=14 S1FRAME=510 S1LBA=32
ok|a one-word primary, gap 2, a one-word save slot|FRAME=1 S0FRAME=1
ok|an undeclared save slot, whatever its geometry|S1EN=0 S1SLOT=1 S1FRAME=0 S1LBA=0 S1WSRC=0
EOG
  # the Template's block reader (emu, Template.sv, with stand-ins for its PLL, its core's lcell and cos): on hps_io
  # slots 1-9 it compiles; on slot 0 (the CONF_STR's own P2S0 line) and above 9 (an S line names one digit, hps_io
  # takes ten slots) the compile stops, naming lw_template_block_slot_1_to_9
  printf '`define BUILD_DATE "000000"\n' > "$O/build_id.v"
  cat > "$O/lw_xt_stubs.sv" <<'EOS'
module pll(input wire refclk, input wire rst, output wire outclk_0, output wire locked);
assign outclk_0 = refclk, locked = 1'b1;
endmodule
module lcell(input wire in, output wire out); assign out = in; endmodule
module cos(input wire [9:0] x, output wire [7:0] y); assign y = x[7:0]; endmodule
EOS
  for t in bad:0 ok:1 ok:9 bad:10 bad:15; do
    want=${t%%:*}; k=${t#*:}
    echo "-- Template: the block reader on hps_io slot $k" >> "$1"
    iverilog -g2012 -DMISTER_LW -DMISTER_LW_SECTOR_SLOT=$k -I "$O" -I sys -s emu -o "$O/lw_xt.vvp" Template.sv \
      sys/hps_io.sv sys/lw_blk_mux.sv rtl/mycore.v rtl/lfsr.v "$O/lw_xt_stubs.sv" > "$O/lw_xt.err" 2>&1
    r=$?; n=$((n + 1)); g=$(grep -c lw_template_block_slot_1_to_9 "$O/lw_xt.err")
    if [ "$want" = bad ]; then
      if [ $r -ne 0 ] && [ "$g" -gt 0 ]; then echo "pass slot $k: stops, naming the guard" >> "$1"
      else f=$((f + 1)); echo "FAIL Template, slot $k: status $r, the guard $g times in the errors" >> "$1"
      fi
    elif [ $r -eq 0 ]; then echo "pass slot $k: compiles" >> "$1"
    else f=$((f + 1)); echo "FAIL Template, slot $k: does not compile" >> "$1"; grep -i error "$O/lw_xt.err" | head -5 | sed 's/^/   /' >> "$1"
    fi
  done
  if [ $f -eq 0 ]; then echo "PASSED: $n checks, 0 failures" >> "$1"; else echo "FAILED: $n checks, $f failures" >> "$1"; fi
}

# sets ARM LOG: arm T (the 32 tier sets) or B (every tier, three core sides), one compile and short run of
# lw_tb.sv's LW_TB_TIER per set, $LW_TB_JOBS at a time; the log holds each set's run behind "== set NAME".
sets() {
  Q=$O/sets$1; rm -rf "$Q"; mkdir -p "$Q"
  if [ "$1" = T ]; then
    m=0
    while [ $m -lt 32 ]; do
      d=""; nm="lw"; k=0
      for t in PAD EVT SECTOR_SLOT=1 LANE OSD; do
        if [ $(( (m >> k) & 1 )) -eq 1 ]; then d="$d -DMISTER_LW_$t"; nm="$nm-${t%%_*}"; fi
        k=$((k + 1))
      done
      echo "$nm$d"; m=$((m + 1))
    done > "$Q/list"
  else
    A="-DMISTER_LW_PAD -DMISTER_LW_EVT -DMISTER_LW_SECTOR_SLOT=1 -DMISTER_LW_LANE -DMISTER_LW_OSD"
    printf 'bus46 %s -DLW_TB_BUS46\nnomux %s -DLW_TB_NOMUX\nnosave1 %s -DLW_TB_NOSAVE1\nnomark %s -DLW_TB_NOMARK\n' \
      "$A" "$A" "$A" "$A" > "$Q/list"
  fi
  cat > "$Q/one.sh" <<EOS
n=\$1; shift
if iverilog -g2012 -DMISTER_LW -DLW_TB_PROBES -DLW_DIAG=0 -DLW_TB_TIER \$* -I sys -o "$Q/\$n.vvp" $TBF > "$Q/\$n.err" 2>&1
then vvp -n "$Q/\$n.vvp" > "$Q/\$n.log" 2>&1
else { echo "-- \$n: compile"; echo "FAIL \$n: does not compile"; head -20 "$Q/\$n.err"; } > "$Q/\$n.log"
fi
EOS
  xargs -P "${LW_TB_JOBS:-8}" -L 1 sh "$Q/one.sh" < "$Q/list"
  : > "$2"
  while read -r n d; do
    echo "== set $n ($d)" >> "$2"
    cat "$Q/$n.log" >> "$2"
    grep -Eq '^(PASSED|FAILED): [0-9]' "$Q/$n.log" || echo "FAIL $n: gave no result" >> "$2"
  done < "$Q/list"
}

# verdict ARM LOG: one result line; status 0 if the arm passes
verdict() {
  awk -v arm="$1" '
    BEGIN { n = split(ENVIRON["XFAIL"], L, "\n")
            for (i = 1; i <= n; i++) if (L[i] != "") { p = index(L[i], "|")
              if (substr(L[i], 1, p - 1) == arm) { want[substr(L[i], p + 1)] = 1; nw++ } } }
    /^-- /             { h = substr($0, 4); next }
    /^FAILED: timeout/ { why = "timeout"; next }
    /^FAIL /           { if (h in want) got[h] = 1
                         else { bad++; if (!(h in seen)) { seen[h] = 1; cases = cases " [" h "]" } } }
    /^== (seed|set) /  { seed = $2 " " $3 " " }
    /^PASSED: /        { res = res (res != "" ? "; " : "") (seed != "" ? seed : "") $0 }
    /^FAILED: [0-9]/   { res = res (res != "" ? "; " : "") (seed != "" ? seed : "") $0 }
    END { for (h in want) if (!(h in got)) { xpass++; xp = xp " [" h "]" }
          if (res == "" && why == "") why = "no result line"
          if (bad || xpass || why != "") {
            printf "arm %s: FAILED%s%s%s%s\n", arm, why != "" ? " (" why ")" : "", bad ? ", failing:" cases : "",
                   xpass ? ", expected to fail but passed:" xp : "", res != "" ? "; " res : ""
            exit 1 }
          printf "arm %s: %s%s\n", arm, nw ? "PASSED with " nw " expected-fail cases; " : "", res }' "$2"
}

mkdir -p "$O"; rc=0; R=""
for g in $ARMS; do
  if [ "$g" = H ]; then
    echo "== arm H: tb/hps/lw_probe.c compiled with warnings as errors, its logic run on the build host"
    if [ ! -f "$O/lw_probeH.log" ] || [ "$(head -1 "$O/lw_probeH.log")" != "run $LW_TB_HRUN" ]; then
      V="arm H: FAILED (no compile from this run)"; rc=1
    elif grep -q '^SKIPPED' "$O/lw_probeH.log"; then V="arm H: $(grep '^SKIPPED' "$O/lw_probeH.log")"
    else
      grep -v "^pass \|^run " "$O/lw_probeH.log"
      V=$(verdict H "$O/lw_probeH.log") || rc=1
    fi
    R="$R$V
"
    continue
  fi
  if [ "$g" = N ]; then
    echo "== arm N: tb/static.sh"
    sh tb/static.sh > "$O/lw_staticN.log" 2>&1
    grep -v "^pass " "$O/lw_staticN.log" | grep -v '^[a-z.-]*: [0-9]* LW nets'
    V=$(verdict N "$O/lw_staticN.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = E ]; then
    echo "== arm E: sys_top.v elaborated against tb/sys_stubs.sv"
    elab "$O/lw_elabE.log"
    grep -E '^FAIL|^   |^[a-z.-]*: (PASSED|FAILED)|^(PASSED|FAILED)' "$O/lw_elabE.log"
    V=$(verdict E "$O/lw_elabE.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = S ]; then
    echo "== arm S: tb/offstate.sh"
    sh tb/offstate.sh > "$O/lw_offS.log" 2>&1
    grep -v "^pass " "$O/lw_offS.log"
    V=$(verdict S "$O/lw_offS.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = V ]; then
    echo "== arm V: slang"
    if ! command -v slang >/dev/null 2>&1; then V="arm V: SKIPPED (no slang on PATH)"
    elif lint "$O/lw_lintV.log"; then V="arm V: PASSED: slang, 6 runs, no error"
    else grep -E 'error|warning' "$O/lw_lintV.log"; V="arm V: FAILED (slang errors, $O/lw_lintV.log)"; rc=1
    fi
    R="$R$V
"
    continue
  fi
  if [ "$g" = X ]; then
    echo "== arm X: a tier macro without MISTER_LW stops the build in hps_io; an unservable SECTOR geometry in the window"
    xarm "$O/lw_tierX.log"
    grep -v "^pass " "$O/lw_tierX.log"
    V=$(verdict X "$O/lw_tierX.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = T ] || [ "$g" = B ]; then
    echo "== arm $g: the short tier run"
    sets "$g" "$O/lw_tb$g.log"
    grep -E '^FAIL|^== set|^(PASSED|FAILED)' "$O/lw_tb$g.log"
    V=$(verdict "$g" "$O/lw_tb$g.log") || rc=1
    R="$R$V
"
    continue
  fi
  B="-DMISTER_LW -DLW_TB_PROBES"; S="-DMISTER_LW_SECTOR_SLOT=1"; P="-DMISTER_LW_PAD -DMISTER_LW_EVT"
  A="$P -DMISTER_LW_OSD $S"; L="$A -DLW_DIAG=0 -DMISTER_LW_LANE"
  C="$L -DLW_TB_CDC"; SEEDS=1
  case $g in
    Z) D="$B -DLW_DIAG=0";;
    1|0) D="$B $A -DLW_DIAG=$g";;
    M) D="$B $A -DLW_DIAG=0 -DMENU_CORE";;
    F) D="$B $P -DLW_DIAG=0";;
    G) D="$B $A -DLW_DIAG=0 -DLW_TB_DEFGEOM";;
    W) D="$B $A -DLW_DIAG=0 -DLW_TB_GEOMW";;
    L) D="$B $L";;
    R12) D="$B $C -DLW_TB_SYS_HP_PS=41667";;
    R20) D="$B $C -DLW_TB_SYS_HP_PS=25000";;
    R21) D="$B $C -DLW_TB_SYS_HP_PS=23281";;
    R33) D="$B $C -DLW_TB_SYS_HP_PS=14762";;
    R53) D="$B $C -DLW_TB_SYS_HP_PS=9313";;
    R62) D="$B $C -DLW_TB_SYS_HP_PS=8000";;
    R100) D="$B $C -DLW_TB_SYS_HP_PS=5000";;
    R150) D="$B $C -DLW_TB_SYS_HP_PS=3333";;
    *) echo "tb/run.sh: unknown arm $g" >&2; exit 2;;
  esac
  case $g in R*) SEEDS=${LW_TB_SEEDS:-1 2 3};; esac
  echo "== arm $g: $D"
  if iverilog -g2012 $D -I sys -o "$O/lw_tb$g.vvp" $TBF 2>"$O/lw_tb$g.err"; then
    for s in $SEEDS; do vvp -n "$O/lw_tb$g.vvp" +lw_seed=$s > "$O/lw_tb$g.s$s.log" & done; wait   # seeds side by side
    : > "$O/lw_tb$g.log"
    for s in $SEEDS; do
      [ "$SEEDS" = 1 ] || echo "== seed $s" >> "$O/lw_tb$g.log"
      cat "$O/lw_tb$g.s$s.log" >> "$O/lw_tb$g.log"; rm -f "$O/lw_tb$g.s$s.log"
    done
    grep -v "^pass " "$O/lw_tb$g.log"
    V=$(verdict "$g" "$O/lw_tb$g.log") || rc=1
  else
    cat "$O/lw_tb$g.err"; V="arm $g: FAILED (does not compile)"; rc=1
  fi
  R="$R$V
"
done
echo "== results"; printf '%s' "$R"; exit $rc
