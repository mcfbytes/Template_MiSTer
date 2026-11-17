#!/bin/sh
# LW bridge framework testbench (tb/lw_tb.sv, tb/top_tb.sv and the scripts below), one run per arm:
#   S  the off-state guard (tb/offstate.sh): without MISTER_LW, sys/ reads upstream's text under every upstream
#      macro set; every LW-only file is empty without its macro and keeps no macro defined
#   N  static checks on the preprocessed top level (tb/static.sh): the GPI terms with both LW instances or
#      neither, the f2h_irq lines, the LW nets arm E checks, in every configuration arm E builds; and the
#      crossing structure: every crossing built from the cells of sys/lw_cdc.sv, every source register named;
#      and the directory's lines: each entry's LINE word one ordinal of its own, picking that entry's line
#   E  sys_top.v elaborated (tb/top_tb.sv, against the stand-ins of tb/sys_stubs.sv) in every configuration of
#      "tb/static.sh --configs": off and each LW tier set, each with no upstream macro, MISTER_DEBUG_NOHDMI and
#      MISTER_DUAL_SDRAM. Fails if it does not elaborate, if an LW run warns of an implicit net or a port width
#      that the off run of the same upstream variant does not, or if a check of top_tb fails (an LW net that
#      reads z among them)
#   1  LW_DIAG=1 (debug build: diagnostics compiled in)
#   0  LW_DIAG=0 (shipping build)
#   M  LW_DIAG=0 with MENU_CORE (the osd header-row gate only)
#   F  LW_DIAG=0 without MISTER_LW_SECTOR_SLOT (the Template's own configuration): no SECTOR files, no muxes
#   G  LW_DIAG=0 with the SECTOR block at the default geometry (128-word primary, 256-word save slots)
#   L  LW_DIAG=0 with the SECTOR block and the frame lane (MISTER_LW_LANE): the link nets through lw_link
#   R12 R20 R21 R33 R53 R62 R100 R150  arm L with clk_sys at 12.0, 20.0, 21.477, 33.8688, 53.69, 62.5, 100.0 and
#      150.0 MHz (the other arms run at 33.8688 MHz without the models); LW_TB_SYS_HP_PS is the half period in ps,
#      plus 50 ppm. Each runs
#      with the crossing cells' delay models (LW_TB_CDC, sys/lw_cdc.sv) under each seed of LW_TB_SEEDS (1 2 3)
#   V  front-end lint: slang elaborates every LW module at its defaults (the vendor atom left unknown), and
#      the window with every block, at both LW_DIAG values, with every tier macro; and the floor modules with
#      MISTER_LW alone. Any error fails (a name used before its declaration is one, which the Icarus arms
#      accept). Without slang on PATH the arm reports SKIPPED and does not fail.
# LW_TB_ARMS selects arms (default: all); LW_TB_QUICK=1 runs S, N, E, F and 0. Needs Icarus Verilog >= 12
# (11 cannot parse hps_io's default-less CONF_STR). Without a local one, set LW_TB_IMAGE to a container image
# that has it; the tree is mounted at /w.
# Logs go to $LW_TB_OUT (tb/out). The last lines are one result per arm; the exit status is 0 only if all pass.
cd "$(dirname "$0")/.." || exit 2
if [ -z "$LW_TB_INNER" ] && ! iverilog -V 2>/dev/null | grep -q 'version 1[2-9]'; then
  if [ -n "$LW_TB_IMAGE" ]; then
    sh tb/offstate.sh --extract   # arm S's upstream tree: the container has no git
    exec docker run --rm -u "$(id -u):$(id -g)" -e LW_TB_OUT -e LW_TB_ARMS -e LW_TB_QUICK -e LW_TB_SEEDS -e LW_TB_INNER=1 -v "$PWD:/w" -w /w \
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
LWF="sys/lw_cdc.sv sys/hps_lw_bridge.sv sys/lw_sys_window.sv sys/lw_sector.sv sys/lw_sys_status.sv sys/lw_lane.sv sys/lw_blk_mux.sv"
LWM="lw_sync lw_arst lw_qstat lw_xfer lw_fence hps_lw_bridge lw_axi_regbus lw_sys_window lw_sector lw_sys_status lw_lane lw_link lw_blk_mux lw_card_mux"
LW0="lw_sync lw_arst lw_qstat lw_xfer lw_fence hps_lw_bridge lw_axi_regbus lw_sys_window lw_sys_status"
LWD="-DMISTER_LW -DMISTER_LW_SECTOR_SLOT=1 -DMISTER_LW_LANE"
lint() {
  : > "$1"; st=0
  for d in 0 1; do
    T=""; for m in $LWM; do T="$T --top $m"; done
    T0=""; for m in $LW0; do T0="$T0 --top $m"; done
    for run in "$LWD|$T" "$LWD|--top lw_sys_window -G SEC_EN=1 -G LANE_EN=1" "-DMISTER_LW|$T0"; do
      m=${run%%|*}; t=${run#*|}
      echo "slang -DLW_DIAG=$d $m $t" >> "$1"
      slang -q --ignore-unknown-modules -DLW_DIAG=$d $m $t $LWF >> "$1" 2>&1 || st=1
    done
  done
  return $st
}

# elab LOG: arm E, one elaboration and simulation of tb/top_tb.sv per configuration; the log ends with one
# PASSED/FAILED line for the arm. Files from sys.qip and files.qip, less those sys_stubs.sv stands in for.
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
  while IFS='|' read -r c d; do
    v=${c#*.}; [ "$v" = "$c" ] && v=""; b=off${v:+.$v}
    mkdir -p "$E/$c"
    echo "-- $c: elaborates, with no implicit net and no port width beyond $b's ($d)" >> "$1"
    sh tb/static.sh --nets "$E/$c/lw_e_nets.vh" $d >> "$1" 2>&1 || echo "FAIL $c: no net list" >> "$1"
    if ! iverilog -g2012 -Wimplicit $d -I sys -I "$E" -I "$E/$c" -s top_tb -o "$E/$c/top.vvp" tb/top_tb.sv \
         tb/sys_stubs.sv $F > "$E/$c/err" 2>&1; then
      echo "FAIL $c: does not elaborate" >> "$1"
      grep -E 'error|sorry' "$E/$c/err" | head -20 | sed 's/^/   /' >> "$1"; continue
    fi
    grep -E 'warning: (implicit definition|Port .* expects)' "$E/$c/err" | grep -v '^tb/' | sort -u > "$E/$c.w"
    [ "$c" = "$b" ] || comm -13 "$E/$b.w" "$E/$c.w" | sed "s/^/FAIL $c: beyond $b: /" >> "$1"
    echo "-- $c: tb/top_tb.sv" >> "$1"
    vvp -n "$E/$c/top.vvp" > "$E/$c/log" 2>&1
    sed -e "s/^-- /-- $c: /" -e "s/^\(PASSED\|FAILED\): /$c: \1: /" -e '/\$finish called/d' "$E/$c/log" >> "$1"
    grep -Eq '^(PASSED|FAILED): [0-9]' "$E/$c/log" ||
      printf -- '-- %s: tb/top_tb.sv ends\nFAIL %s: tb/top_tb.sv gave no result\n' "$c" "$c" >> "$1"
  done < "$E/configs"
  awk -v n="$(wc -l < "$E/configs")" '/^FAIL / { f++ } / PASSED: [0-9]+ checks/ { c += $3 }
    END { if (f) printf "FAILED: %d failures over %d configurations\n", f, n
          else printf "PASSED: %d configurations, %d checks\n", n, c }' "$1" > "$E/result"
  cat "$E/result" >> "$1"
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
    /^== seed /        { seed = $3 " " }
    /^PASSED: /        { res = res (res != "" ? "; " : "") (seed != "" ? "seed " seed : "") $0 }
    /^FAILED: [0-9]/   { res = res (res != "" ? "; " : "") (seed != "" ? "seed " seed : "") $0 }
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
for g in ${LW_TB_ARMS:-S N E 1 0 M F G L R12 R20 R21 R33 R53 R62 R100 R150 V}; do
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
  B="-DMISTER_LW -DLW_TB_PROBES"; S="-DMISTER_LW_SECTOR_SLOT=1"; L="$S -DLW_DIAG=0 -DMISTER_LW_LANE"
  C="$L -DLW_TB_CDC"; SEEDS=1
  case $g in
    1|0) D="$B $S -DLW_DIAG=$g";;
    M) D="$B $S -DLW_DIAG=0 -DMENU_CORE";;
    F) D="$B -DLW_DIAG=0";;
    G) D="$B $S -DLW_DIAG=0 -DLW_TB_DEFGEOM";;
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
  if iverilog -g2012 $D -I sys -o "$O/lw_tb$g.vvp" tb/lw_tb.sv tb/lw_stub_atom.sv sys/lw_cdc.sv sys/hps_lw_bridge.sv \
       sys/lw_sys_window.sv sys/lw_sector.sv sys/lw_sys_status.sv sys/lw_lane.sv sys/lw_blk_mux.sv sys/hps_io.sv \
       sys/osd.v 2>"$O/lw_tb$g.err"; then
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
