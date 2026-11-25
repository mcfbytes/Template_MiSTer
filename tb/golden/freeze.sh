#!/bin/sh
# Freezes the mailbox pair and the two LW modules arms C and D compare against, as gold_ modules, for arm J of
# tb/run.sh and the gold twins of tb/sfeed_tb.sv and tb/fio_tb.sv: the copies are this script's output from one
# commit, never edited by hand. Run from anywhere in the tree: sh tb/golden/freeze.sh [REV] (default: the SHA in
# the copies' header lines, or HEAD when there are none). Needs git.
#   gold_hps_ocram_master.sv  sys/hps_ocram_master.sv with hps_ocram_{master,tsink,diag,evt} renamed gold_*, and
#                             without the trailing undefs of MISTER_OCRAM_MARK and MISTER_OCRAM_POLL (the copy is
#                             compiled ahead of the live file, which must still see the bench's values)
#   gold_hps_ocram_rx.sv      sys/hps_ocram_rx.sv with hps_ocram_{rx,sfeed,fplay} renamed gold_*, its build macro
#                             GOLD_OCRAM_RX_BUILT
#   gold_lw_sector.sv         sys/lw_sector.sv, lw_sector renamed gold_lw_sector
#   gold_lw_lane.sv           the lw_lane module of sys/lw_lane.sv alone (lw_link stays shared), as gold_lw_lane
# The crossing cells of sys/lw_cdc.sv and lw_link are shared, not frozen: no step changes them.
cd "$(dirname "$0")/../.." || exit 2
D=tb/golden
rev=$1
[ -n "$rev" ] || rev=$(sed -n '1s/^\/\/ FROZEN from \([0-9a-f]\{40\}\):.*/\1/p' $D/gold_hps_ocram_master.sv 2>/dev/null)
[ -n "$rev" ] || rev=HEAD
sha=$(git rev-parse --verify "$rev^{commit}") || { echo "freeze.sh: no commit $rev" >&2; exit 2; }
hdr() { echo "// FROZEN from $sha:$1 by tb/golden/freeze.sh: never edit, re-run the script"; }
R='s/\bhps_ocram_(master|tsink|diag|evt|rx|sfeed|fplay)\b/gold_hps_ocram_\1/g; s/gold_(hps_ocram_[a-z]+\.sv)/\1/g'
{ hdr sys/hps_ocram_master.sv
  git show "$sha:sys/hps_ocram_master.sv" | sed -E "$R" | sed -E '/^`undef MISTER_OCRAM_(MARK|POLL)$/d'
} > $D/gold_hps_ocram_master.sv || exit 1
{ hdr sys/hps_ocram_rx.sv
  git show "$sha:sys/hps_ocram_rx.sv" | sed -E "$R; s/\bLW_OCRAM_RX_BUILT\b/GOLD_OCRAM_RX_BUILT/g"
} > $D/gold_hps_ocram_rx.sv || exit 1
{ hdr sys/lw_sector.sv
  git show "$sha:sys/lw_sector.sv" | sed -E 's/\blw_sector\b/gold_lw_sector/g; s/gold_(lw_sector\.sv)/\1/g'
} > $D/gold_lw_sector.sv || exit 1
{ hdr sys/lw_lane.sv
  printf '`ifdef MISTER_LW\n`ifdef MISTER_LW_LANE\n`default_nettype none\n\n'
  git show "$sha:sys/lw_lane.sv" | awk '/^module lw_lane\>/ { on = 1 } on { print } on && /^endmodule/ { exit }' |
    sed -E 's/^module lw_lane\b/module gold_lw_lane/'
  printf '\n`default_nettype wire\n`endif\n`endif\n'
} > $D/gold_lw_lane.sv || exit 1
for f in gold_hps_ocram_master gold_hps_ocram_rx gold_lw_sector gold_lw_lane; do
  grep -q '^module gold_' $D/$f.sv || { echo "freeze.sh: $D/$f.sv has no gold_ module" >&2; exit 1; }
done
echo "freeze.sh: tb/golden frozen from $sha"
