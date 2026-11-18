#!/bin/sh
# LW bridge framework testbench (tb/lw_tb.sv, tb/top_tb.sv and the scripts below), one run per arm:
#   S  the off-state guard (tb/offstate.sh): without MISTER_LW, sys/ reads upstream's text under every upstream
#      macro set; every LW-only file is empty without its macro and keeps no macro defined
#   N  static checks on the preprocessed top level (tb/static.sh): the GPI terms with both LW instances or
#      neither, the f2h_irq lines, the LW nets arm E checks, in every configuration arm E builds; and the
#      crossing structure: every crossing built from the cells of sys/lw_cdc.sv, every source register named;
#      and the directory's lines: each entry's LINE word one ordinal of its own, picking that entry's line
#   E  sys_top.v elaborated (tb/top_tb.sv, against the stand-ins of tb/sys_stubs.sv) in every configuration of
#      "tb/static.sh --configs": off and each of the 32 LW tier sets, each with no upstream macro,
#      MISTER_DEBUG_NOHDMI and MISTER_DUAL_SDRAM (99). Fails if it does not elaborate, if an LW run warns of an
#      implicit net or a port width that the off run of the same upstream variant does not, or if a check of
#      top_tb fails (an LW net that reads z, an HPS_BUS bit driven other than as the bit map says, among them)
#   Z  MISTER_LW alone (the floor): no block listed, the enable and the firewall whole
#   1  LW_DIAG=1 (debug build: diagnostics compiled in), every tier but the lane
#   0  LW_DIAG=0 (shipping build), every tier but the lane
#   M  LW_DIAG=0 with MENU_CORE (the osd header-row gate only)
#   F  LW_DIAG=0 with the floor, PAD and EVT (the Template's own configuration): no SECTOR files, no muxes, no OSD
#   G  LW_DIAG=0 with the SECTOR block at the default geometry (128-word primary, 256-word save slots)
#   L  LW_DIAG=0 with every tier, the frame lane (MISTER_LW_LANE) included: the link nets through lw_link
#   X  each tier macro without MISTER_LW: the compile of hps_io fails, naming lw_tier_needs_MISTER_LW (and
#      compiles without the macro, and with MISTER_LW)
#   T  the short tier run (LW_TB_TIER) in each of the 32 tier sets: the directory, the decode, the causes and
#      lines, and the linked blocks' OWNED and OPEN bits of the set built
#   B  the short tier run with every tier and a core side that leaves a piece out: a 46-bit bus at the core
#      (LW_TB_BUS46), no SECTOR mux (LW_TB_NOMUX), save slot 1's mux missing (LW_TB_NOSAVE1)
#   R12 R20 R21 R33 R53 R62 R100 R150  arm L with clk_sys at 12.0, 20.0, 21.477, 33.8688, 53.69, 62.5, 100.0 and
#      150.0 MHz (the other arms run at 33.8688 MHz without the models); LW_TB_SYS_HP_PS is the half period in ps,
#      plus 50 ppm. Each runs
#      with the crossing cells' delay models (LW_TB_CDC, sys/lw_cdc.sv) under each seed of LW_TB_SEEDS (1 2 3)
#   V  front-end lint: slang elaborates every LW module at its defaults (the vendor atom left unknown), and
#      the window with every block, at both LW_DIAG values, with every tier macro; and the floor modules with
#      MISTER_LW alone. Any error fails (a name used before its declaration is one, which the Icarus arms
#      accept). Without slang on PATH the arm reports SKIPPED and does not fail.
# LW_TB_ARMS selects arms (default: all); LW_TB_QUICK=1 runs S, N, E, F and 0. LW_TB_JOBS (8) runs at a time in
# arms E, T and B. Needs Icarus Verilog >= 12
# (11 cannot parse hps_io's default-less CONF_STR). Without a local one, set LW_TB_IMAGE to a container image
# that has it; the tree is mounted at /w.
# Logs go to $LW_TB_OUT (tb/out). The last lines are one result per arm; the exit status is 0 only if all pass.
cd "$(dirname "$0")/.." || exit 2
if [ -z "$LW_TB_INNER" ] && ! iverilog -V 2>/dev/null | grep -q 'version 1[2-9]'; then
  if [ -n "$LW_TB_IMAGE" ]; then
    sh tb/offstate.sh --extract   # arm S's upstream tree: the container has no git
    exec docker run --rm -u "$(id -u):$(id -g)" -e LW_TB_OUT -e LW_TB_ARMS -e LW_TB_QUICK -e LW_TB_SEEDS -e LW_TB_JOBS -e LW_TB_INNER=1 -v "$PWD:/w" -w /w \
      "$LW_TB_IMAGE" sh tb/run.sh
  fi
  echo "tb/run.sh: needs Icarus Verilog >= 12 on PATH, or LW_TB_IMAGE set to an image with it" >&2; exit 2
fi

# Expected failures, one "arm|case" per line, the case as its "-- " heading prints. An arm with entries passes
# only if exactly its listed cases fail: an unlisted failure, or a listed case that passes under every seed the
# arm runs, fails the arm.
# SECTOR's reject bundle ({tag, slot} sampled as levels beside its toggle) tears under the delay models above
# about 60 MHz, until requests and their rejects cross as records. Which reject meets a torn sample, if any, depends
# on the phase the cases before it leave, so the list names the cases that meet one under today's seeds.
XFAIL='R62|adjacent rejects of two slots: both requests re-open
R150|adjacent rejects of two slots: both requests re-open
R150|core reset during a card load: the stale BEGIN_S is rejected, the request re-opens'
export XFAIL

# lint LOG: arm V's slang runs; status 0 if every run elaborates without an error
LWF="sys/lw_cdc.sv sys/hps_lw_bridge.sv sys/lw_sys_window.sv sys/lw_pad.sv sys/lw_evt.sv sys/lw_sector.sv sys/lw_lane.sv sys/lw_blk_mux.sv"
LWM="lw_sync lw_arst lw_qstat lw_xfer lw_fence hps_lw_bridge lw_axi_regbus lw_sys_window lw_pad lw_evt lw_sector lw_lane lw_link lw_blk_mux lw_card_mux"
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
TBF="$TBF sys/lw_evt.sv sys/lw_sector.sv sys/lw_lane.sv sys/lw_blk_mux.sv sys/hps_io.sv sys/osd.v"

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
    printf 'bus46 %s -DLW_TB_BUS46\nnomux %s -DLW_TB_NOMUX\nnosave1 %s -DLW_TB_NOSAVE1\n' "$A" "$A" "$A" > "$Q/list"
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

O=${LW_TB_OUT:-tb/out}; mkdir -p "$O"; rc=0; R=""
[ -n "$LW_TB_QUICK" ] && : ${LW_TB_ARMS:=S N E F 0}
for g in ${LW_TB_ARMS:-S N E Z 1 0 M F G L X T B R12 R20 R21 R33 R53 R62 R100 R150 V}; do
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
    echo "== arm X: a tier macro without MISTER_LW stops the build in hps_io"
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
