#!/bin/sh
# LW bridge framework testbench (tb/lw_tb.sv), one run per arm:
#   1  LW_DIAG=1 (debug build: diagnostics compiled in)
#   0  LW_DIAG=0 (shipping build)
#   M  LW_DIAG=0 with MENU_CORE (the osd header-row gate only)
#   N  LW_DIAG=0 without a SECTOR block (the Template's own configuration)
#   G  LW_DIAG=0 with the SECTOR block at the default geometry (128-word primary, 256-word save slots)
#   L  LW_DIAG=0 with the SECTOR block and the frame lane (MISTER_LW_LANE): the link nets through lw_link
#   R12 R21 R53 R62 R100 R150  arm L with clk_sys at 12.0, 21.477, 53.69, 62.5, 100.0 and 150.0 MHz
#      (the other arms run it at 33.8688 MHz); LW_TB_SYS_HP_PS is the half period in ps, plus 50 ppm
# LW_TB_ARMS selects arms (default: all). Needs Icarus Verilog >= 12 (11 cannot parse hps_io's default-less
# CONF_STR). Without a local one, set LW_TB_IMAGE to a container image that has it; the tree is mounted at /w.
# Logs go to $LW_TB_OUT (tb/out). The last lines are one result per arm; the exit status is 0 only if all pass.
cd "$(dirname "$0")/.." || exit 2
if [ -z "$LW_TB_INNER" ] && ! iverilog -V 2>/dev/null | grep -q 'version 1[2-9]'; then
  if [ -n "$LW_TB_IMAGE" ]; then
    exec docker run --rm -u "$(id -u):$(id -g)" -e LW_TB_OUT -e LW_TB_ARMS -e LW_TB_INNER=1 -v "$PWD:/w" -w /w \
      "$LW_TB_IMAGE" sh tb/run.sh
  fi
  echo "tb/run.sh: needs Icarus Verilog >= 12 on PATH, or LW_TB_IMAGE set to an image with it" >&2; exit 2
fi

# Expected failures, one "arm|case" per line, the case as its "-- " heading prints. An arm with entries passes
# only if exactly its listed cases fail: an unlisted failure, or a listed case that passes, fails the arm.
XFAIL=''
export XFAIL

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
    /^PASSED: /        { res = $0 }
    /^FAILED: [0-9]/   { res = $0 }
    END { for (h in want) if (!(h in got)) { xpass++; xp = xp " [" h "]" }
          if (res == "" && why == "") why = "no result line"
          if (bad || xpass || why != "") {
            printf "arm %s: FAILED%s%s%s%s\n", arm, why != "" ? " (" why ")" : "", bad ? ", failing:" cases : "",
                   xpass ? ", expected to fail but passed:" xp : "", res != "" ? "; " res : ""
            exit 1 }
          printf "arm %s: %s%s\n", arm, nw ? "PASSED with " nw " expected-fail cases; " : "", res }' "$2"
}

O=${LW_TB_OUT:-tb/out}; mkdir -p "$O"; rc=0; R=""
for g in ${LW_TB_ARMS:-1 0 M N G L R12 R21 R53 R62 R100 R150}; do
  B="-DMISTER_LW -DLW_TB_PROBES"; S="-DMISTER_LW_SECTOR_SLOT=1"; L="$S -DLW_DIAG=0 -DMISTER_LW_LANE"
  case $g in
    1|0) D="$B $S -DLW_DIAG=$g";;
    M) D="$B $S -DLW_DIAG=0 -DMENU_CORE";;
    N) D="$B -DLW_DIAG=0 -DLW_TB_NOSEC";;
    G) D="$B $S -DLW_DIAG=0 -DLW_TB_DEFGEOM";;
    L) D="$B $L";;
    R12) D="$B $L -DLW_TB_SYS_HP_PS=41667";;
    R21) D="$B $L -DLW_TB_SYS_HP_PS=23281";;
    R53) D="$B $L -DLW_TB_SYS_HP_PS=9313";;
    R62) D="$B $L -DLW_TB_SYS_HP_PS=8000";;
    R100) D="$B $L -DLW_TB_SYS_HP_PS=5000";;
    R150) D="$B $L -DLW_TB_SYS_HP_PS=3333";;
    *) echo "tb/run.sh: unknown arm $g" >&2; exit 2;;
  esac
  echo "== arm $g: $D"
  if iverilog -g2012 $D -I sys -o "$O/lw_tb$g.vvp" tb/lw_tb.sv tb/lw_stub_atom.sv sys/hps_lw_bridge.sv \
       sys/lw_sys_window.sv sys/lw_sector.sv sys/lw_sys_status.sv sys/lw_lane.sv sys/lw_blk_mux.sv sys/hps_io.sv \
       sys/osd.v 2>"$O/lw_tb$g.err"; then
    vvp -n "$O/lw_tb$g.vvp" > "$O/lw_tb$g.log"
    grep -v "^pass " "$O/lw_tb$g.log"
    V=$(verdict "$g" "$O/lw_tb$g.log") || rc=1
  else
    cat "$O/lw_tb$g.err"; V="arm $g: FAILED (does not compile)"; rc=1
  fi
  R="$R$V
"
done
echo "== results"; printf '%s' "$R"; exit $rc
