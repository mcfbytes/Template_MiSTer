#!/bin/sh
# LW bridge framework testbench (tb/lw_tb.sv), run four times:
#   1  LW_DIAG=1 (debug build: diagnostics compiled in)
#   0  LW_DIAG=0 (shipping build)
#   M  LW_DIAG=0 with MENU_CORE (the osd header-row gate only)
#   N  LW_DIAG=0 without a SECTOR block (the Template's own configuration)
# Needs Icarus Verilog >= 12 (11 cannot parse hps_io's default-less CONF_STR). Without a local one, set
# LW_TB_IMAGE to a container image that has it; the tree is mounted at /w. Logs go to $LW_TB_OUT (tb/out).
cd "$(dirname "$0")/.." && C='O=${LW_TB_OUT:-tb/out}; mkdir -p "$O"; rc=0; for g in 1 0 M N; do
  B="-DMISTER_LW -DLW_TB_PROBES"
  case $g in
    M) D="$B -DMISTER_LW_SECTOR_SLOT=1 -DLW_DIAG=0 -DMENU_CORE";;
    N) D="$B -DLW_DIAG=0 -DLW_TB_NOSEC";;
    *) D="$B -DMISTER_LW_SECTOR_SLOT=1 -DLW_DIAG=$g";;
  esac
  echo "== $D"
  iverilog -g2012 $D -I sys -o "$O/lw_tb$g.vvp" tb/lw_tb.sv tb/lw_stub_atom.sv sys/hps_lw_bridge.sv sys/lw_sys_window.sv sys/lw_sector.sv sys/lw_sys_status.sv sys/lw_blk_mux.sv sys/hps_io.sv sys/osd.v 2>"$O/lw_tb$g.err" || { cat "$O/lw_tb$g.err"; exit 1; }
  vvp -n "$O/lw_tb$g.vvp" | tee "$O/lw_tb$g.log" | grep -v "^pass "; grep -q "^PASSED" "$O/lw_tb$g.log" || rc=1
done; exit $rc'
if iverilog -V 2>/dev/null | grep -q 'version 1[2-9]'; then sh -c "$C"
elif [ -n "$LW_TB_IMAGE" ]; then docker run --rm -u "$(id -u):$(id -g)" -e LW_TB_OUT -v "$PWD:/w" -w /w "$LW_TB_IMAGE" sh -c "$C"
else echo "tb/run.sh: needs Icarus Verilog >= 12 on PATH, or LW_TB_IMAGE set to an image with it" >&2; exit 2; fi
