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
#   P  LW_DIAG=0 with MISTER_DISABLE_SPI_PAD: the link's eight pad opcodes (0x02, 0x03, 0x10-0x13, 0x1A, 0x3D) are
#      ignored and the LW PAD block's are taken; PAD K9 bit 0 reads 1 (the other arms prove the link's are taken)
#   K  LW_DIAG=0 with every tier but the lane and MISTER_LW_KBM (keys, mouse and analog triggers as PAD stores), seed
#      1: K1, PAD K0 0x040F003C, K2 0x40 and K8 0x00100018; every key frame (make, break, extended make and break,
#      print screen made and broken, pause) on the link (0x05) and as a store at 0x05 index 0, ps2_key the same and
#      toggled once each; two mouse packets on the link (0x04) and as stores (the extension at index 1 toggling
#      nothing, the packet at index 0 toggling once), the packet and its extension the link's; the six trigger pairs
#      on the link (0x46) and as stores, each landing alone; stores at no key ignored; a trigger a level (50 stores
#      with clk_sys stopped cross once). K2, the event FIFO: with clk_sys stopped a host that does not pace queues
#      16 of 20 key stores (STAT level 16, drops + 4) and the 16 arrive in order; a host that paces by STAT's level
#      (one entry a key, two a mouse packet) meets the full FIFO, waits, and its 32 keys and 8 packets all arrive in
#      store order with no drop. And verilator --lint-only -Wall on lw_pad.sv with MISTER_LW_KBM, which must be silent
#   X  each tier macro without MISTER_LW: the compile of hps_io fails, naming lw_tier_needs_MISTER_LW (and
#      compiles without the macro, and with MISTER_LW); and each unservable SECTOR geometry: the compile of the
#      window fails, naming lw_sector_unservable_geometry (and the servable edges compile); and the Template's
#      block reader outside hps_io slots 1-9: emu's compile fails, naming lw_template_block_slot_1_to_9; and the
#      OCRAM SECTOR and SPI-pad guards: MISTER_OCRAM_SECTOR_SLOT without MISTER_LW, with MISTER_OCRAM_DIAG, beside
#      the LW SECTOR block, or at an unservable geometry, and MISTER_DISABLE_SPI_PAD without MISTER_LW_PAD, each
#      fails naming its guard (and its legal neighbour compiles); and the OCRAM download tier's: MISTER_OCRAM_FIO
#      beside MISTER_LW_LANE (link_on_two_players), a gap outside 4-7 or a ring outside 2-10 and 13
#      (ocram_fio_unservable_geometry), a ring of 13 beside the SECTOR tier or DIAG (ocram_fio_ring_on_ring4_span);
#      and the two removals': MISTER_DISABLE_SPI_OSD without an OSD tier (spi_osd_off_needs_an_osd_tier), and
#      MISTER_DISABLE_SPI_BLK with no mailbox block tier, with a slot below VDNUM neither on a tier nor named in
#      MISTER_DISABLE_SPI_BLK_UNUSED, or with a tier slot named in it (spi_blk_off_needs_a_block_tier,
#      spi_blk_off_slot_without_a_tier, spi_blk_off_unused_slot_on_a_tier), or with a tier slot at or above VDNUM
#      (spi_blk_off_tier_slot_outside_vdnum); and the OCRAM mount slots with MISTER_OCRAM_BLKSZ (default 2) not
#      hps_io's BLKSZ (ocram_blksz_differs_from_hps_io), each with its legal neighbours; and the
#      key tier's: MISTER_LW_KBM without MISTER_LW_PAD (lw_kbm_needs_MISTER_LW_PAD) or on a core with PS2DIV
#      (lw_kbm_on_a_PS2DIV_core), MISTER_DISABLE_SPI_KBM without MISTER_LW_KBM (spi_kbm_off_needs_MISTER_LW_KBM);
#      and tier 3's: MISTER_OCRAM_EVT without MISTER_LW, beside MISTER_LW_EVT or without the floor
#      (ocram_evt_needs_MISTER_LW, evt_on_two_tiers, ocram_evt_needs_MISTER_OCRAM), MISTER_OCRAM_LANE without the
#      download tier (ocram_lane_needs_MISTER_OCRAM_FIO), MISTER_DISABLE_SPI_EVT without MISTER_OCRAM_EVT, each with
#      its legal neighbours, the events tier and the capture lane without the SECTOR tier among them
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
#      MISTER_LW alone; and the OCRAM master, hps_ocram_rx with its download player and lw_link with
#      MISTER_OCRAM_FIO and no LW macro. Any error fails (a name used before its declaration is one, which the Icarus arms
#      accept). Without slang on PATH the arm reports SKIPPED and does not fail.
#   H  the HPS-side probe, tb/hps/lw_probe.c, compiled with warnings as errors by the host's C compiler (cc, or
#      $CC) and, where one is on PATH, by arm-linux-gnueabihf-gcc; the testbench never runs it. And its own logic
#      run on the build host: tb/hps/lw_probe_test.c includes the probe against a model of the window (its cases
#      in its header), compiled the same way and run once. The OCRAM probe, tb/hps/ocram_probe.c, the same way
#      (as built for the HPS, and with -DOCRAM_FAKE), and tb/hps/ocram_probe_test.c, which drives it against the
#      model of tb/hps/ocram_probe_fake.h (its cases in its header, the part-full --band mode among them), run once.
#      It all runs before any container starts (the image has no C compiler). Without a C compiler the arm reports
#      SKIPPED and does not fail.
#   O  the OCRAM floor's and OSD tier's bench (tb/ocram_tb.sv) under Icarus, every seed of LW_TB_SEEDS once
#      plain and once with the crossing cells' delay models (LW_TB_CDC), compiled twice: with DIAG (MARK records,
#      the torture sink, the freeze knob) and without (the OSD cases in the shipping shape); and verilator
#      --lint-only -Wall on hps_ocram_master.sv and hps_ocram_rx.sv (and its hps_ocram_sfeed) with lw_cdc.sv and
#      lw_lane.sv (the player's lw_link_seq), under each OCRAM macro set, which must be warning-free. A third shape
#      builds the SECTOR tier (MISTER_OCRAM_SECTOR_SLOT, a 588-word frame) beside the OSD, and a fourth the download tier (MISTER_OCRAM_FIO,
#      with lw_link and a real hps_io: the mark line's rise, fall and never unarmed; a 6,000-word window with the
#      mark half a ring behind the head, never dry, PLAYED once; a link transaction at every frame edge; RUN, the
#      ID view and a core reset through GPO mid-frame; a stopped clk_sys) beside the OSD; each shape also runs with
#      clk_sys at 33.8688 and 20 MHz (a CD core's, the Template's); verilator lints the download shape too
#   Q  hps_io's 0x14 readout of a synthetic CONF_STR over 3 KB (tb/conf_str_3k.vh) with CONF_STR_BRAM 0 and 1
#      (tb/confstr_tb.sv): every byte equal between the two and to the string, at strobe gaps of 2 and 5. Icarus
#      reads a copy of hps_io.sv with byte_cnt declared above confstr_rom's port connection, which names it first
#   C  the trace compare (tb/sfeed_tb.sv): lw_sector's primary feeder and the OCRAM SECTOR feeder serve the same
#      frames, {ack, wr, adr, dat} compared on every clk_sys edge from each ack's rise: GAP 2 and 5, wide and byte
#      strobes, plus the consumer-reset and session-end fills, at 33.8688 MHz; a 588-word frame and the
#      Template's 20 MHz at GAP 2 wide; every seed, plain and with the crossing cells' delay models. In the plain
#      runs lw_sector also runs beside its frozen twin (tb/golden/gold_lw_sector.sv), every output equal on every edge
#   D  the download trace compare (tb/fio_tb.sv): the LW lane's player and the OCRAM download player play the same
#      windows through lw_link into a real hps_io each, and fio_block's outputs (download, index, wr, addr, dout,
#      file_ext) with the select and strobe are compared edge for edge, frame by frame: GAP 4 and 7, WIDE 1 and 0,
#      an odd frame, aux words, waits on every write and on every second write; the wait raised GAP - 3 cycles
#      after ioctl_wr, held 1 cycle to 10^4, with no word lost or doubled, and one cycle later, the overrun shown;
#      at 33.8688 MHz, and the Template's 20 MHz at GAP 4 wide; every seed, plain and with the crossing models
#      In the plain runs lw_lane also runs beside its frozen twin (tb/golden/gold_lw_lane.sv), every output equal
#      on every edge
#   Y  the removals MISTER_DISABLE_SPI_OSD and MISTER_DISABLE_SPI_BLK (the lean profile's bench arm R1; arm X has
#      their guards): lw_tb.sv's short tier run with the LW OSD tier and MISTER_DISABLE_SPI_OSD (OSDBUF K2 bit 0
#      reads 1, rows over LW land and draw, an SPI row write (0x20, 0x28) changes no byte and no box height, the 0x4x
#      words still enable and disable); tb/ocram_tb.sv's OSD shape with MISTER_DISABLE_SPI_OSD and its mount shape
#      with both, every seed (C_CAPS bit 17, and 16 with the mount shape, read in the hello's exact C_CAPS; every
#      SPI row changes nothing in osd A, which then takes the row through its tier port as the reference, so the
#      OSD rows from the mailbox and the blocks from rings 0 and 4 pass as in arm O); tb/blkoff_tb.sv with
#      MISTER_DISABLE_SPI_BLK (0x16 reads 0 with requests raised, 0x17 and 0x18 raise no ack and no strobe, the
#      commit mark reads 0 on every cycle, present) and without it (the same words show the arms alive); and
#      six runs that each break one of the core's obligations, each named by hps_io's simulation checks (a mount
#      request of more than one block among them, with and without MISTER_DISABLE_SPI_BLK).
#      And the removal MISTER_DISABLE_SPI_KBM: lw_tb.sv's short tier run with MISTER_LW_KBM and it, PAD K9 bit 1
#      reads 1, arm K's K1 with the link's 0x04, 0x05 and 0x46 changing nothing and the stores landing as the link's
#      frames would, and K2 as in arm K; and
#      arm I's events shape with MISTER_DISABLE_SPI_EVT, every seed: C_CAPS bit 18 in the hello's exact word, the
#      link's 0x29, 0x36, 0x3C and 0x?3F replies 0, the records as in arm I
#   U  the UIO and OSD frames on ring 1 (MISTER_OCRAM_UIO, the lean profile's U1; arm X has its guard):
#      tb/ocram_tb.sv's download shape with MISTER_OCRAM_UIO, every seed, plain and with the crossing models, at
#      33.8688 and 20 MHz, and once with MISTER_DEBUG_NOHDMI: C_K[9] exact (mask 0x000F, 0x000B without HDMI) and
#      C_CAPS unchanged; each frame of the set (hps_io's ten write arms, sys_top's write arms) against its twin sent
#      over the link into a second hps_io: the UIO strobe stream equal, hps_io's registers equal, RTC's and
#      TIMESTAMP's toggles once a frame; the OSD control words as sel 2 then 3 against the link's one transaction
#      into two osds each (no sel-2 strobe without HDMI); a capture frame and a tagged frame skipped by length, no
#      fault; and every download case of arm O's shape in the same run. Then verilator lints the master, the walker
#      and the player with the macro, with and without MISTER_DEBUG_NOHDMI
#   A  the capture lane (MISTER_OCRAM_LANE, the lean profile's L1; arm X has its guards): tb/ocram_tb.sv's shape
#      with MISTER_LW, the SECTOR, download and UIO tiers and MISTER_OCRAM_LANE, every seed, plain and with the
#      crossing models, at 33.8688 and 20 MHz: C_CAPS bit 12 in the hello's exact word; capture frames on the UIO
#      select, odd and even and of one word, each REPLY {0x22, the frame's arg, len} with the tag echoed and each
#      word the twin hps_io's in-band replies to the same link words, low half first; a write frame between them
#      posts none; a capture frame held while the EVT ring lacks len + 1 words, unconsumed and unplayed, while the
#      OSD and SECTOR rings are served, then played once there is room; and every case of arm O's SECTOR and
#      download shapes and arm U's in the same run (a capture frame on a build without the lane: arm U's U2). Then
#      verilator lints the master, the walker and the player with the macro
#   I  the events tier's records (MISTER_OCRAM_EVT, the lines design's E1; arm X has its guards, arm Y its removal):
#      tb/ocram_tb.sv's SECTOR shape with MISTER_LW and MISTER_OCRAM_EVT, a real hps_io on the event port through
#      hps_ocram_evt and a twin hps_io polled over its own link, every seed, plain and with the crossing models, at
#      33.8688 and 20 MHz: C_CAPS bit 11 in the hello's exact word; nothing posted while C_BIND bit 8 is clear, then
#      a BIND record and the causes standing at the bind (the status transfer its first STATUS); each STATUS, INFO,
#      UPLOAD and RUMBLE against the twin's 0x29, 0x36, 0x3C and 0x?3F reply, a level back to 0 included; a STATUS
#      torn by a second transfer abandoned and the second published whole; each hello's levels from 0; a REQUEST
#      owed with a STATUS first; and every SECTOR case of arm O's shape in the same run. A second shape has the
#      events tier without the SECTOR tier (OSD beside it, at 33.8688 MHz): the same cases less the REQUEST order,
#      and no BIND record at the bind. Then verilator lints the master and hps_ocram_evt with the macro, without the
#      SECTOR tier, and with MISTER_DISABLE_SPI_EVT
#   J  the twin (tb/golden/, frozen by tb/golden/freeze.sh from one commit): every tb/ocram_tb.sv shape that arms O,
#      U, A, I and Y build (diag, osd, sec, fio, mnt at 33.8688 and 20 MHz, uio at both and without HDMI, lane, evt,
#      and Y's three removal shapes; not arm I's events shape without SECTOR, which the frozen master stops),
#      compiled with OCRAM_TB_TWIN: the gold master, rx (and hps_ocram_evt) on the live pair's bench inputs, every
#      output of the pair compared x-aware after every edge of clk and clk_sys, the nets between master and rx not
#      compared; plain only, under every seed. Passes when every run passes and prints
#      "TWIN: N edges compared, 0 mismatches" with N above 0. The live files are sys/hps_ocram_*.sv and sys/ocram_*.sv
# LW_TB_ARMS selects arms (default: all); LW_TB_QUICK=1 runs S, N, E, F, 0 and O. LW_TB_JOBS (8) runs at a time
# in arms E, T and B. Needs Icarus Verilog >= 12
# (11 cannot parse hps_io's default-less CONF_STR). Without a local one, set LW_TB_IMAGE to a container image
# that has it; the tree is mounted at /w.
# Logs go to $LW_TB_OUT (tb/out). The last lines are one result per arm; the exit status is 0 only if all pass.
cd "$(dirname "$0")/.." || exit 2
[ -n "$LW_TB_QUICK" ] && : ${LW_TB_ARMS:=S N E F 0 O}
ARMS=${LW_TB_ARMS:-S N E Z 1 0 M F G W L X T B R12 R20 R21 R33 R53 R62 R100 R150 H V O C D P Q Y K U A I J}
O=${LW_TB_OUT:-tb/out}
# the OCRAM master's own layers, beside sys/hps_ocram_master.sv wherever it is built or linted
OCMF="sys/ocram_axi.sv sys/ocram_arb.sv sys/ocram_evt_wr.sv"
# the walker, the crossing and the sinks, beside sys/hps_ocram_rx.sv wherever it is built or linted
OCRF="sys/ocram_xing.sv sys/ocram_ring_rd.sv sys/ocram_bind.sv sys/ocram_mnt_wr.sv sys/ocram_osd_wr.sv sys/ocram_blk_req.sv"

# probe LOG: arm H, on the host. The log's first line names this run, so a run in the container reads only its own.
probe() {
  echo "run $LW_TB_HRUN" > "$1"; n=0; f=0
  W="-std=c11 -Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -pedantic -Werror -O2"
  for c in "${CC:-cc}" arm-linux-gnueabihf-gcc; do
    if ! command -v "$c" >/dev/null 2>&1; then
      [ $n -eq 0 ] && { echo "SKIPPED: no C compiler ($c) on PATH" >> "$1"; return 0; }
      continue
    fi
    for src in lw_probe lw_probe_test ocram_probe ocram_probe_fake ocram_probe_test; do
      echo "-- $c: tb/hps/$src.c" >> "$1"; n=$((n + 1)); rm -f "$O/$src.$(basename "$c")"
      x=""; s=$src
      case $src in
        lw_probe_test) x=-pthread;;
        ocram_probe_fake) x="-DOCRAM_FAKE -pthread"; s=ocram_probe;;
        ocram_probe_test) x="-DOCRAM_FAKE -pthread";;
      esac
      if $c $W $x -o "$O/$src.$(basename "$c")" tb/hps/$s.c >> "$1" 2>&1; then
        echo "pass $c: compiles, no warning" >> "$1"
      else f=$((f + 1)); echo "FAIL $c: $s.c does not compile without a warning" >> "$1"
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
  # the OCRAM probe's own logic (--band and the first probe's modes), on the model of tb/hps/ocram_probe_fake.h
  t=$O/ocram_probe_test.$(basename "${CC:-cc}"); q=0
  if [ -x "$t" ]; then
    "$t" > "$O/ocram_probe_test.log" 2>&1; st=$?
    cat "$O/ocram_probe_test.log" >> "$1"
    q=$(sed -n 's/^ocram_probe_test: \([0-9]*\) checks.*/\1/p' "$O/ocram_probe_test.log"); nf=$(grep -c '^FAIL ' "$O/ocram_probe_test.log")
    f=$((f + nf))
    if [ $nf -eq 0 ] && { [ $st -ne 0 ] || [ -z "$q" ]; }; then
      f=$((f + 1)); echo "FAIL ocram_probe_test: status $st, no failed check named" >> "$1"
    fi
    r=$((r + ${q:-0}))
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
LWM="lw_sync lw_arst lw_qstat lw_xfer lw_fence hps_lw_bridge lw_axi_regbus lw_sys_window lw_pad lw_evt lw_sector lw_lane lw_link lw_link_seq lw_osd lw_blk_mux lw_save_mux"
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
    # the download tier without the LW floor: lw_lane.sv gives lw_link alone
    m="-DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_FIO"
    t="--top hps_ocram_master --top hps_ocram_rx --top hps_ocram_fplay --top lw_link -G FIO_EN=1"
    echo "slang -DLW_DIAG=$d $m $t" >> "$1"
    slang -q --ignore-unknown-modules -I . -DLW_DIAG=$d $m $t sys/lw_cdc.sv sys/lw_lane.sv sys/hps_ocram_master.sv \
      $OCMF sys/hps_ocram_rx.sv $OCRF >> "$1" 2>&1 || st=1
    # the mount slots beside the SECTOR tier: the master, the walker and feeder, the window's save-slot nets, the muxes
    m="-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3"
    t="--top hps_ocram_master --top hps_ocram_rx --top hps_ocram_sfeed --top lw_sys_window --top lw_save_mux"
    t="$t --top lw_blk_mux -G SEC_EN=1 -G MNT_EN=3 -G SEC_EXT=1 -G MNT_EXT=3"
    echo "slang -DLW_DIAG=$d $m $t" >> "$1"
    slang -q --ignore-unknown-modules -I . -DLW_DIAG=$d $m $t $LWF sys/hps_ocram_master.sv $OCMF sys/hps_ocram_rx.sv \
      $OCRF >> "$1" 2>&1 || st=1
  done
  return $st
}

# sfeedc LOG: arm C, the trace compare: each shape compiled once, run under every seed, plain and with the models.
sfeedc() {
  : > "$1"
  SFD="-DMISTER_LW -DMISTER_LW_SECTOR_SLOT=1 -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1"
  SFF="tb/sfeed_tb.sv tb/golden/gold_lw_sector.sv sys/lw_sector.sv sys/hps_ocram_rx.sv $OCRF sys/lw_cdc.sv"
  for sh in "g2w1:-DSFEED_TB_GAP=2 -DSFEED_TB_WIDE=1" "g2w0:-DSFEED_TB_GAP=2 -DSFEED_TB_WIDE=0" \
            "g5w1:-DSFEED_TB_GAP=5 -DSFEED_TB_WIDE=1" "g5w0:-DSFEED_TB_GAP=5 -DSFEED_TB_WIDE=0" \
            "f588:-DSFEED_TB_GAP=2 -DSFEED_TB_WIDE=1 -DSFEED_TB_FRAME=588" \
            "tpl:-DSFEED_TB_GAP=2 -DSFEED_TB_WIDE=1 -DSFEED_TB_SYS_HP_PS=25000"; do
    t=${sh%%:*}; X=${sh#*:}   # every shape also compares the mount slot k = 1 against the save slot X0
    for m in plain cdc; do
      d=""; [ "$m" = cdc ] && d=-DLW_TB_CDC
      echo "-- the trace bench compiles ($t, $m)" >> "$1"
      if iverilog -g2012 $SFD $X $d -I sys -o "$O/sfeedC_${t}_$m.vvp" $SFF > "$O/sfeedC_${t}_$m.err" 2>&1; then
        echo "pass bench: compiles ($t, $m)" >> "$1"
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          vvp -n "$O/sfeedC_${t}_$m.vvp" +lw_seed=$s > "$O/sfeedC_${t}_$m.$s.log" 2>&1 &
        done; wait
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          echo "== seed $s $t $m" >> "$1"; cat "$O/sfeedC_${t}_$m.$s.log" >> "$1"
          grep -Eq '^(PASSED|FAILED): [0-9]' "$O/sfeedC_${t}_$m.$s.log" || echo "FAIL bench $s $t $m: gave no result" >> "$1"
        done
      else
        echo "FAIL bench: does not compile ($t, $m)" >> "$1"; head -20 "$O/sfeedC_${t}_$m.err" | sed 's/^/   /' >> "$1"
      fi
    done
  done
}

# fioc LOG: arm D, the download trace compare: each shape compiled once, run under every seed, plain and with the
# models. hps_io is compiled with MISTER_LW for lw_lane and MISTER_OCRAM_OSD for hps_ocram_rx.sv's player.
fioc() {
  : > "$1"
  FID="-DMISTER_LW -DMISTER_LW_LANE -DMISTER_OCRAM -DMISTER_OCRAM_OSD"
  FIF="tb/fio_tb.sv tb/golden/gold_lw_lane.sv sys/lw_lane.sv sys/hps_ocram_rx.sv $OCRF sys/lw_cdc.sv sys/hps_io.sv"
  for sh in "g4w1:-DFIO_TB_GAP=4 -DFIO_TB_WIDE=1" "g4w0:-DFIO_TB_GAP=4 -DFIO_TB_WIDE=0" \
            "g7w1:-DFIO_TB_GAP=7 -DFIO_TB_WIDE=1" "g7w0:-DFIO_TB_GAP=7 -DFIO_TB_WIDE=0" \
            "tpl:-DFIO_TB_GAP=4 -DFIO_TB_WIDE=1 -DFIO_TB_SYS_HP_PS=25000"; do
    t=${sh%%:*}; X=${sh#*:}
    for m in plain cdc; do
      d=""; [ "$m" = cdc ] && d=-DLW_TB_CDC
      echo "-- the download trace bench compiles ($t, $m)" >> "$1"
      if iverilog -g2012 $FID $X $d -I sys -s fio_tb -o "$O/fioD_${t}_$m.vvp" $FIF > "$O/fioD_${t}_$m.err" 2>&1; then
        echo "pass bench: compiles ($t, $m)" >> "$1"
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          vvp -n "$O/fioD_${t}_$m.vvp" +lw_seed=$s > "$O/fioD_${t}_$m.$s.log" 2>&1 &
        done; wait
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          echo "== seed $s $t $m" >> "$1"; cat "$O/fioD_${t}_$m.$s.log" >> "$1"
          grep -Eq '^(PASSED|FAILED): [0-9]' "$O/fioD_${t}_$m.$s.log" || echo "FAIL bench $s $t $m: gave no result" >> "$1"
        done
      else
        echo "FAIL bench: does not compile ($t, $m)" >> "$1"; head -20 "$O/fioD_${t}_$m.err" | sed 's/^/   /' >> "$1"
      fi
    done
  done
}

# ocram LOG: arm O. The bench compiles once per shape (with DIAG; the OSD shape, the OSD + SECTOR shape, the download
# shape and the OSD + SECTOR + mount slots shape, each at two clk_sys rates), each plain and
# with the crossing cells' delay models, runs under every seed of LW_TB_SEEDS, then verilator lints
# hps_ocram_master.sv and hps_ocram_rx.sv with lw_cdc.sv (each OCRAM macro set), warnings as errors.
ocram() {
  : > "$1"; n=0; f=0
  OCB="tb/ocram_tb.sv sys/hps_ocram_master.sv $OCMF sys/hps_ocram_rx.sv $OCRF sys/lw_cdc.sv sys/osd.v"
  OCD="-DMISTER_OCRAM -DMISTER_OCRAM_DIAG -DMISTER_OCRAM_OSD -DMISTER_OCRAM_MARK=14 -DMISTER_OCRAM_POLL=10"
  OCE="-DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_POLL=10"
  OCS="$OCE -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_SECTOR_FRAME=588"
  OCF="$OCE -DMISTER_OCRAM_FIO"
  OCM="$OCS -DMISTER_OCRAM_MOUNT0_SLOT=3 -DMISTER_OCRAM_MOUNT1_SLOT=4 -DMISTER_OCRAM_MOUNT_FRAME=512"
  # shape:macros; diag at the bench's 50 MHz, the others at 33.8688 MHz and the Template's 20 MHz
  for m in plain cdc; do
    for c in diag osd33 osd20 sec33 sec20 fio33 fio20 mnt33 mnt20; do
      case $c in diag) D=$OCD;; osd33) D="$OCE -DOCRAM_TB_SYS_HP_PS=14762";; osd20) D="$OCE -DOCRAM_TB_SYS_HP_PS=25000";;
        sec33) D="$OCS -DOCRAM_TB_SYS_HP_PS=14762";; sec20) D="$OCS -DOCRAM_TB_SYS_HP_PS=25000";;
        fio33) D="$OCF -DOCRAM_TB_SYS_HP_PS=14762";; fio20) D="$OCF -DOCRAM_TB_SYS_HP_PS=25000";;
        mnt33) D="$OCM -DOCRAM_TB_SYS_HP_PS=14762";; mnt20) D="$OCM -DOCRAM_TB_SYS_HP_PS=25000";; esac
      d=""; [ "$m" = cdc ] && d=-DLW_TB_CDC
      F=$OCB; case $c in fio*) F="$OCB sys/lw_lane.sv sys/hps_io.sv";; esac
      echo "-- the bench compiles ($c, $m)" >> "$1"
      if iverilog -g2012 $D $d -I sys -s ocram_tb -o "$O/ocramO_${c}_$m.vvp" $F > "$O/ocramO_${c}_$m.err" 2>&1; then
        echo "pass bench: compiles ($c, $m)" >> "$1"; n=$((n + 1))
      else
        f=$((f + 1)); echo "FAIL bench: does not compile ($c, $m)" >> "$1"
        head -20 "$O/ocramO_${c}_$m.err" | sed 's/^/   /' >> "$1"
      fi
    done
  done
  for m in plain cdc; do
    for c in diag osd33 osd20 sec33 sec20 fio33 fio20 mnt33 mnt20; do
      [ -f "$O/ocramO_${c}_$m.vvp" ] || continue
      for s in ${LW_TB_SEEDS:-1 2 3}; do
        vvp -n "$O/ocramO_${c}_$m.vvp" +lw_seed=$s > "$O/ocramO_${c}_$m.$s.log" 2>&1 &
      done
      wait                                     # the seeds side by side, one shape at a time
    done
  done
  for s in ${LW_TB_SEEDS:-1 2 3}; do
    for c in diag osd33 osd20 sec33 sec20 fio33 fio20 mnt33 mnt20; do
      for m in plain cdc; do
        [ -f "$O/ocramO_${c}_$m.vvp" ] || continue
        echo "== seed $s $c $m" >> "$1"
        cat "$O/ocramO_${c}_$m.$s.log" >> "$1"
        grep -Eq '^(PASSED|FAILED): [0-9]' "$O/ocramO_${c}_$m.$s.log" ||
          echo "FAIL bench $s $c $m: gave no result" >> "$1"
      done
    done
  done
  echo "-- verilator --lint-only -Wall on hps_ocram_master.sv and hps_ocram_rx.sv with lw_cdc.sv, which must be silent" >> "$1"
  if command -v verilator >/dev/null 2>&1; then
    vok=1
    for VS in "diag:-DMISTER_OCRAM -DMISTER_OCRAM_DIAG -DMISTER_OCRAM_OSD" "osd:-DMISTER_OCRAM -DMISTER_OCRAM_OSD" \
              "sec:-DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=2" "both:-DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_SECTOR_SLOT=2" \
              "fio:-DMISTER_OCRAM -DMISTER_OCRAM_FIO" "fioall:-DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_FIO" \
              "mnt:-DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_MOUNT0_SLOT=3 -DMISTER_OCRAM_MOUNT1_SLOT=4"; do
      vn=${VS%%:*}; VF=${VS#*:}
      for t in hps_ocram_master hps_ocram_rx hps_ocram_sfeed hps_ocram_fplay; do
        G=""; fl=$t
        [ $t = hps_ocram_sfeed ] && { case $vn in sec|both|fioall) fl=hps_ocram_rx;; mnt) fl=hps_ocram_rx; G="-GMNT_EN=2'b11";;
          *) continue;; esac; }
        [ $t = hps_ocram_fplay ] && { case $vn in fio|fioall) fl=hps_ocram_rx;; *) continue;; esac; }
        [ $t = hps_ocram_rx ] && case $vn in sec) G="-GOSD_EN=0 -GSEC_EN=1";; both) G="-GSEC_EN=1";;
          fio) G="-GOSD_EN=0 -GFIO_EN=1";; fioall) G="-GSEC_EN=1 -GFIO_EN=1";; mnt) G="-GOSD_EN=0 -GSEC_EN=1 -GMNT_EN=2'b11";; esac
        if ! verilator --lint-only -Wall $VF $G --top-module $t -Isys "sys/$fl.sv" $OCMF $OCRF sys/lw_cdc.sv \
             sys/lw_lane.sv > "$O/ocramV_${t}_$vn.log" 2>&1 || [ -s "$O/ocramV_${t}_$vn.log" ]; then
          vok=0; echo "FAIL verilator: $t ($vn)" >> "$1"; head -20 "$O/ocramV_${t}_$vn.log" | sed 's/^/   /' >> "$1"
        fi
      done
    done
    if [ $vok -eq 1 ]; then echo "pass verilator: no warning" >> "$1"; n=$((n + 1))
    else f=$((f + 1)); fi
  else
    echo "SKIPPED verilator: not on PATH" >> "$1"
  fi
  if [ $f -eq 0 ]; then echo "PASSED: $n compiles and runs, 0 failures" >> "$1"
  else echo "FAILED: $n compiles and runs, $f failures" >> "$1"; fi
}

# kbmk LOG: arm K, lw_tb.sv's full run with MISTER_LW_KBM (seed 1), then verilator on lw_pad.sv with it.
kbmk() {
  : > "$1"; n=0; f=0
  D="-DMISTER_LW -DLW_TB_PROBES -DLW_DIAG=0 -DMISTER_LW_PAD -DMISTER_LW_EVT -DMISTER_LW_OSD -DMISTER_LW_SECTOR_SLOT=1"
  D="$D -DMISTER_LW_KBM"
  echo "-- lw_tb compiles (MISTER_LW_KBM)" >> "$1"
  if iverilog -g2012 $D -I sys -o "$O/kbmK.vvp" $TBF > "$O/kbmK.err" 2>&1; then
    echo "pass lw_tb: compiles" >> "$1"; n=$((n + 1))
    vvp -n "$O/kbmK.vvp" +lw_seed=1 > "$O/kbmK.log" 2>&1
    echo "== seed 1 kbm" >> "$1"; cat "$O/kbmK.log" >> "$1"
    grep -Eq '^(PASSED|FAILED): [0-9]' "$O/kbmK.log" || echo "FAIL lw_tb: gave no result" >> "$1"
  else
    f=$((f + 1)); echo "FAIL lw_tb: does not compile" >> "$1"; head -20 "$O/kbmK.err" | sed 's/^/   /' >> "$1"
  fi
  echo "-- verilator --lint-only -Wall on lw_pad.sv with MISTER_LW_KBM, which must be silent" >> "$1"
  if command -v verilator >/dev/null 2>&1; then
    if ! verilator --lint-only -Wall -DMISTER_LW -DMISTER_LW_PAD -DMISTER_LW_KBM --top-module lw_pad -Isys \
         sys/lw_pad.sv sys/lw_cdc.sv > "$O/kbmV.log" 2>&1 || [ -s "$O/kbmV.log" ]; then
      f=$((f + 1)); echo "FAIL verilator: lw_pad (MISTER_LW_KBM)" >> "$1"; head -20 "$O/kbmV.log" | sed 's/^/   /' >> "$1"
    else echo "pass verilator: no warning" >> "$1"; n=$((n + 1)); fi
  else
    echo "SKIPPED verilator: not on PATH" >> "$1"
  fi
  if [ $f -eq 0 ]; then echo "PASSED: $n compiles and runs, 0 failures" >> "$1"
  else echo "FAILED: $n compiles and runs, $f failures" >> "$1"; fi
}

# rmy LOG: arm Y, the two removals. Each shape compiles once and runs under every seed (blkoff_tb once); the
# violation runs are judged by the one hps_io line each must print, not by their own result.
rmy() {
  : > "$1"; n=0; f=0
  # lw_tb.sv's short tier run, the LW OSD tier with MISTER_DISABLE_SPI_OSD
  YL="-DMISTER_LW -DLW_TB_PROBES -DLW_DIAG=0 -DLW_TB_TIER -DMISTER_LW_PAD -DMISTER_LW_EVT -DMISTER_LW_OSD"
  YL="$YL -DMISTER_LW_SECTOR_SLOT=1 -DMISTER_DISABLE_SPI_OSD"
  echo "-- lw_tb compiles (the tier run with MISTER_DISABLE_SPI_OSD)" >> "$1"
  if iverilog -g2012 $YL -I sys -o "$O/rmY_lw.vvp" $TBF > "$O/rmY_lw.err" 2>&1; then
    echo "pass lw_tb: compiles" >> "$1"; n=$((n + 1))
    vvp -n "$O/rmY_lw.vvp" > "$O/rmY_lw.log" 2>&1
    echo "== seed 1 lw" >> "$1"; cat "$O/rmY_lw.log" >> "$1"
    grep -Eq '^(PASSED|FAILED): [0-9]' "$O/rmY_lw.log" || echo "FAIL lw_tb: gave no result" >> "$1"
  else
    f=$((f + 1)); echo "FAIL lw_tb: does not compile" >> "$1"; head -20 "$O/rmY_lw.err" | sed 's/^/   /' >> "$1"
  fi
  # lw_tb.sv's short tier run, the key tier with MISTER_DISABLE_SPI_KBM
  YK="-DMISTER_LW -DLW_TB_PROBES -DLW_DIAG=0 -DLW_TB_TIER -DMISTER_LW_PAD -DMISTER_LW_EVT -DMISTER_LW_KBM"
  YK="$YK -DMISTER_DISABLE_SPI_KBM"
  echo "-- lw_tb compiles (the tier run with MISTER_DISABLE_SPI_KBM)" >> "$1"
  if iverilog -g2012 $YK -I sys -o "$O/rmY_kbm.vvp" $TBF > "$O/rmY_kbm.err" 2>&1; then
    echo "pass lw_tb: compiles (kbm)" >> "$1"; n=$((n + 1))
    vvp -n "$O/rmY_kbm.vvp" > "$O/rmY_kbm.log" 2>&1
    echo "== seed 1 kbm" >> "$1"; cat "$O/rmY_kbm.log" >> "$1"
    grep -Eq '^(PASSED|FAILED): [0-9]' "$O/rmY_kbm.log" || echo "FAIL lw_tb kbm: gave no result" >> "$1"
  else
    f=$((f + 1)); echo "FAIL lw_tb: does not compile (kbm)" >> "$1"; head -20 "$O/rmY_kbm.err" | sed 's/^/   /' >> "$1"
  fi
  # ocram_tb.sv: the OSD shape with MISTER_DISABLE_SPI_OSD, the mount shape with both removals
  OCB="tb/ocram_tb.sv sys/hps_ocram_master.sv $OCMF sys/hps_ocram_rx.sv $OCRF sys/lw_cdc.sv sys/osd.v"
  OCE="-DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_POLL=10 -DOCRAM_TB_SYS_HP_PS=14762"
  OCM="$OCE -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_SECTOR_FRAME=588 -DMISTER_OCRAM_MOUNT0_SLOT=3"
  OCM="$OCM -DMISTER_OCRAM_MOUNT1_SLOT=4 -DMISTER_OCRAM_MOUNT_FRAME=512"
  for c in osd mnt; do
    case $c in osd) D="$OCE -DMISTER_DISABLE_SPI_OSD";;
      mnt) D="$OCM -DMISTER_DISABLE_SPI_OSD -DMISTER_DISABLE_SPI_BLK";; esac
    echo "-- ocram_tb compiles ($c)" >> "$1"
    if iverilog -g2012 $D -I sys -s ocram_tb -o "$O/rmY_$c.vvp" $OCB > "$O/rmY_$c.err" 2>&1; then
      echo "pass ocram_tb: compiles ($c)" >> "$1"; n=$((n + 1))
      for s in ${LW_TB_SEEDS:-1 2 3}; do vvp -n "$O/rmY_$c.vvp" +lw_seed=$s > "$O/rmY_$c.$s.log" 2>&1 & done; wait
      for s in ${LW_TB_SEEDS:-1 2 3}; do
        echo "== seed $s $c" >> "$1"; cat "$O/rmY_$c.$s.log" >> "$1"
        grep -Eq '^(PASSED|FAILED): [0-9]' "$O/rmY_$c.$s.log" || echo "FAIL ocram_tb $s $c: gave no result" >> "$1"
      done
    else
      f=$((f + 1)); echo "FAIL ocram_tb: does not compile ($c)" >> "$1"; head -20 "$O/rmY_$c.err" | sed 's/^/   /' >> "$1"
    fi
  done
  # ocram_tb.sv: arm I's events shape with MISTER_DISABLE_SPI_EVT (C_CAPS bit 18 in the hello's exact word; the
  # link's 0x29, 0x36, 0x3C, 0x?3F replies read 0; the records arrive, held to the values the core drove)
  echo "-- ocram_tb compiles (evt, MISTER_DISABLE_SPI_EVT)" >> "$1"
  if iverilog -g2012 $ID -DMISTER_DISABLE_SPI_EVT -DOCRAM_TB_SYS_HP_PS=14762 -I sys -s ocram_tb -o "$O/rmY_evt.vvp" $IB \
       > "$O/rmY_evt.err" 2>&1; then
    echo "pass ocram_tb: compiles (evt)" >> "$1"; n=$((n + 1))
    for s in ${LW_TB_SEEDS:-1 2 3}; do vvp -n "$O/rmY_evt.vvp" +lw_seed=$s > "$O/rmY_evt.$s.log" 2>&1 & done; wait
    for s in ${LW_TB_SEEDS:-1 2 3}; do
      echo "== seed $s evt" >> "$1"; cat "$O/rmY_evt.$s.log" >> "$1"
      grep -Eq '^(PASSED|FAILED): [0-9]' "$O/rmY_evt.$s.log" || echo "FAIL ocram_tb $s evt: gave no result" >> "$1"
    done
  else
    f=$((f + 1)); echo "FAIL ocram_tb: does not compile (evt)" >> "$1"; head -20 "$O/rmY_evt.err" | sed 's/^/   /' >> "$1"
  fi
  # blkoff_tb.sv: hps_io with MISTER_DISABLE_SPI_BLK, and without it (the control)
  YB="-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3"
  YB="$YB -DMISTER_OCRAM_MOUNT_FRAME=256"
  YO="-DMISTER_DISABLE_SPI_BLK -DMISTER_DISABLE_SPI_BLK_UNUSED=1"
  for c in off ctl; do
    D=$YB; [ $c = off ] && D="$YB $YO"
    echo "-- blkoff_tb compiles ($c)" >> "$1"
    if iverilog -g2012 $D -I sys -s blkoff_tb -o "$O/rmY_b$c.vvp" tb/blkoff_tb.sv sys/hps_io.sv > "$O/rmY_b$c.err" 2>&1
    then
      echo "pass blkoff_tb: compiles ($c)" >> "$1"; n=$((n + 1))
      vvp -n "$O/rmY_b$c.vvp" > "$O/rmY_b$c.log" 2>&1
      echo "== seed 1 $c" >> "$1"; cat "$O/rmY_b$c.log" >> "$1"
      grep -Eq '^(PASSED|FAILED): [0-9]' "$O/rmY_b$c.log" || echo "FAIL blkoff_tb $c: gave no result" >> "$1"
    else
      f=$((f + 1)); echo "FAIL blkoff_tb: does not compile ($c)" >> "$1"; head -20 "$O/rmY_b$c.err" | sed 's/^/   /' >> "$1"
    fi
  done
  # the core's obligations: each broken once, each named by hps_io's checks (pat once, a lines from hps_io in all;
  # run 6 is built without MISTER_DISABLE_SPI_BLK)
  while IFS='|' read -r k pat na what; do
    echo "-- the core's word broken: $what" >> "$1"
    D="$YB $YO"; [ "$k" = 6 ] && D=$YB
    if iverilog -g2012 $D -DBLKOFF_TB_VIOLATE=$k -I sys -s blkoff_tb -o "$O/rmY_v$k.vvp" tb/blkoff_tb.sv \
         sys/hps_io.sv > "$O/rmY_v$k.err" 2>&1; then
      vvp -n "$O/rmY_v$k.vvp" > "$O/rmY_v$k.log" 2>&1; n=$((n + 1))
      g=$(grep -c "^FAIL hps_io: .*$pat" "$O/rmY_v$k.log")
      a=$(grep -c '^FAIL hps_io: ' "$O/rmY_v$k.log")
      if [ "$g" -eq 1 ] && [ "$a" -eq "$na" ]; then echo "pass $what: named once" >> "$1"
      else f=$((f + 1)); echo "FAIL $what: named $g times, $a hps_io checks in all" >> "$1"; fi
    else
      f=$((f + 1)); echo "FAIL $what: does not compile" >> "$1"; head -5 "$O/rmY_v$k.err" | sed 's/^/   /' >> "$1"
    fi
  done <<'EOV'
1|slot 0 is named unused and requests|1|slot 0, named unused, requests
2|the SECTOR slot 1 raises sd_wr|1|the SECTOR slot writes
3|mount slot 2 asks 384 words, more than 256|2|a mount read of 384 words on a 256-word frame (and of three blocks)
4|mount slot 2 asks 384 words, more than 256|2|the same read seen only on the save mux's core rd bit
5|mount slot 2 requests with sd_blk_cnt 1|1|a mount read of two blocks within the frame
6|mount slot 2 requests with sd_blk_cnt 1|1|the same, built without MISTER_DISABLE_SPI_BLK
EOV
  if [ $f -eq 0 ]; then echo "PASSED: $n compiles and runs, 0 failures" >> "$1"
  else echo "FAILED: $n compiles and runs, $f failures" >> "$1"; fi
}

# uarm LOG: arm U. tb/ocram_tb.sv's download shape with MISTER_OCRAM_UIO, each shape compiled once, run under every
# seed, plain and with the crossing models; then verilator on the three modules with the macro.
uarm() {
  : > "$1"; n=0; f=0
  UB="tb/ocram_tb.sv sys/hps_ocram_master.sv $OCMF sys/hps_ocram_rx.sv $OCRF sys/lw_cdc.sv sys/osd.v sys/lw_lane.sv sys/hps_io.sv"
  UD="-DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_POLL=10 -DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO"
  for m in plain cdc; do
    for c in uio33 uio20 uion33; do
      case $c in uio33) D="$UD -DOCRAM_TB_SYS_HP_PS=14762";; uio20) D="$UD -DOCRAM_TB_SYS_HP_PS=25000";;
        uion33) D="$UD -DMISTER_DEBUG_NOHDMI -DOCRAM_TB_SYS_HP_PS=14762";; esac
      d=""; [ "$m" = cdc ] && d=-DLW_TB_CDC
      echo "-- the bench compiles ($c, $m)" >> "$1"
      if iverilog -g2012 $D $d -I sys -s ocram_tb -o "$O/uioU_${c}_$m.vvp" $UB > "$O/uioU_${c}_$m.err" 2>&1; then
        echo "pass bench: compiles ($c, $m)" >> "$1"; n=$((n + 1))
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          vvp -n "$O/uioU_${c}_$m.vvp" +lw_seed=$s > "$O/uioU_${c}_$m.$s.log" 2>&1 &
        done; wait
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          echo "== seed $s $c $m" >> "$1"; cat "$O/uioU_${c}_$m.$s.log" >> "$1"
          grep -Eq '^(PASSED|FAILED): [0-9]' "$O/uioU_${c}_$m.$s.log" || echo "FAIL bench $s $c $m: gave no result" >> "$1"
        done
      else
        f=$((f + 1)); echo "FAIL bench: does not compile ($c, $m)" >> "$1"; head -20 "$O/uioU_${c}_$m.err" | sed 's/^/   /' >> "$1"
      fi
    done
  done
  echo "-- verilator --lint-only -Wall on the master, the walker and the player with MISTER_OCRAM_UIO" >> "$1"
  if command -v verilator >/dev/null 2>&1; then
    vok=1
    for VS in "uio:-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO" \
              "uion:-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO -DMISTER_DEBUG_NOHDMI" \
              "uioall:-DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO"; do
      vn=${VS%%:*}; VF=${VS#*:}
      for t in hps_ocram_master hps_ocram_rx hps_ocram_fplay; do
        G=""; fl=$t
        [ $t = hps_ocram_fplay ] && fl=hps_ocram_rx
        [ $t = hps_ocram_rx ] && case $vn in uioall) G="-GSEC_EN=1 -GFIO_EN=1";; *) G="-GOSD_EN=0 -GFIO_EN=1";; esac
        if ! verilator --lint-only -Wall $VF $G --top-module $t -Isys "sys/$fl.sv" $OCMF $OCRF sys/lw_cdc.sv \
             sys/lw_lane.sv > "$O/uioV_${t}_$vn.log" 2>&1 || [ -s "$O/uioV_${t}_$vn.log" ]; then
          vok=0; echo "FAIL verilator: $t ($vn)" >> "$1"; head -20 "$O/uioV_${t}_$vn.log" | sed 's/^/   /' >> "$1"
        fi
      done
    done
    if [ $vok -eq 1 ]; then echo "pass verilator: no warning" >> "$1"; n=$((n + 1))
    else f=$((f + 1)); fi
  else
    echo "SKIPPED verilator: not on PATH" >> "$1"
  fi
  if [ $f -eq 0 ]; then echo "PASSED: $n compiles and runs, 0 failures" >> "$1"
  else echo "FAILED: $n compiles and runs, $f failures" >> "$1"; fi
}

# aarm LOG: arm A. tb/ocram_tb.sv's capture shape, each shape compiled once, run under every seed, plain and with
# the crossing models; then verilator on the master, the walker and the player with the macro.
AB="tb/ocram_tb.sv sys/hps_ocram_master.sv $OCMF sys/hps_ocram_rx.sv $OCRF sys/lw_cdc.sv sys/osd.v sys/lw_lane.sv sys/hps_io.sv"
AD="-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_POLL=10 -DMISTER_OCRAM_SECTOR_SLOT=2"
AD="$AD -DMISTER_OCRAM_SECTOR_FRAME=588 -DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO -DMISTER_OCRAM_LANE"
aarm() {
  : > "$1"; n=0; f=0
  for m in plain cdc; do
    for c in lane33 lane20; do
      case $c in lane33) D="$AD -DOCRAM_TB_SYS_HP_PS=14762";; lane20) D="$AD -DOCRAM_TB_SYS_HP_PS=25000";; esac
      d=""; [ "$m" = cdc ] && d=-DLW_TB_CDC
      echo "-- the bench compiles ($c, $m)" >> "$1"
      if iverilog -g2012 $D $d -I sys -s ocram_tb -o "$O/laneA_${c}_$m.vvp" $AB > "$O/laneA_${c}_$m.err" 2>&1; then
        echo "pass bench: compiles ($c, $m)" >> "$1"; n=$((n + 1))
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          vvp -n "$O/laneA_${c}_$m.vvp" +lw_seed=$s > "$O/laneA_${c}_$m.$s.log" 2>&1 &
        done; wait
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          echo "== seed $s $c $m" >> "$1"; cat "$O/laneA_${c}_$m.$s.log" >> "$1"
          grep -Eq '^(PASSED|FAILED): [0-9]' "$O/laneA_${c}_$m.$s.log" || echo "FAIL bench $s $c $m: gave no result" >> "$1"
        done
      else
        f=$((f + 1)); echo "FAIL bench: does not compile ($c, $m)" >> "$1"; head -20 "$O/laneA_${c}_$m.err" | sed 's/^/   /' >> "$1"
      fi
    done
  done
  echo "-- verilator --lint-only -Wall on the master, the walker and the player with MISTER_OCRAM_LANE" >> "$1"
  if command -v verilator >/dev/null 2>&1; then
    vok=1
    for VS in "lane:-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_FIO -DMISTER_OCRAM_LANE" \
              "laneall:-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_MOUNT0_SLOT=3 -DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO -DMISTER_OCRAM_LANE -DMISTER_OCRAM_EVT"; do
      vn=${VS%%:*}; VF=${VS#*:}
      for t in hps_ocram_master hps_ocram_rx hps_ocram_fplay; do
        G=""; fl=$t
        [ $t = hps_ocram_fplay ] && fl=hps_ocram_rx
        [ $t = hps_ocram_rx ] && case $vn in laneall) G="-GSEC_EN=1 -GFIO_EN=1 -GMNT_EN=2'b01";; *) G="-GOSD_EN=0 -GSEC_EN=1 -GFIO_EN=1";; esac
        if ! verilator --lint-only -Wall $VF $G --top-module $t -Isys "sys/$fl.sv" $OCMF $OCRF sys/lw_cdc.sv \
             sys/lw_lane.sv > "$O/laneV_${t}_$vn.log" 2>&1 || [ -s "$O/laneV_${t}_$vn.log" ]; then
          vok=0; echo "FAIL verilator: $t ($vn)" >> "$1"; head -20 "$O/laneV_${t}_$vn.log" | sed 's/^/   /' >> "$1"
        fi
      done
    done
    if [ $vok -eq 1 ]; then echo "pass verilator: no warning" >> "$1"; n=$((n + 1))
    else f=$((f + 1)); fi
  else
    echo "SKIPPED verilator: not on PATH" >> "$1"
  fi
  if [ $f -eq 0 ]; then echo "PASSED: $n compiles and runs, 0 failures" >> "$1"
  else echo "FAILED: $n compiles and runs, $f failures" >> "$1"; fi
}

# iarm LOG: arm I. tb/ocram_tb.sv's SECTOR shape with the events tier, each shape compiled once, run under every
# seed, plain and with the crossing models; then verilator on the master and the port with the macro.
IB="tb/ocram_tb.sv sys/hps_ocram_master.sv $OCMF sys/hps_ocram_rx.sv $OCRF sys/lw_cdc.sv sys/osd.v sys/hps_io.sv"
ID="-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_POLL=10 -DMISTER_OCRAM_SECTOR_SLOT=2"
ID="$ID -DMISTER_OCRAM_SECTOR_FRAME=588 -DMISTER_OCRAM_EVT"
# the events tier without the SECTOR tier: no BIND record, no REQUEST
IN="-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_POLL=10 -DMISTER_OCRAM_EVT"
iarm() {
  : > "$1"; n=0; f=0
  for m in plain cdc; do
    for c in evt33 evt20 evtns33; do
      case $c in evt33) D="$ID -DOCRAM_TB_SYS_HP_PS=14762";; evt20) D="$ID -DOCRAM_TB_SYS_HP_PS=25000";;
        evtns33) D="$IN -DOCRAM_TB_SYS_HP_PS=14762";; esac
      d=""; [ "$m" = cdc ] && d=-DLW_TB_CDC
      echo "-- the bench compiles ($c, $m)" >> "$1"
      if iverilog -g2012 $D $d -I sys -s ocram_tb -o "$O/evtI_${c}_$m.vvp" $IB > "$O/evtI_${c}_$m.err" 2>&1; then
        echo "pass bench: compiles ($c, $m)" >> "$1"; n=$((n + 1))
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          vvp -n "$O/evtI_${c}_$m.vvp" +lw_seed=$s > "$O/evtI_${c}_$m.$s.log" 2>&1 &
        done; wait
        for s in ${LW_TB_SEEDS:-1 2 3}; do
          echo "== seed $s $c $m" >> "$1"; cat "$O/evtI_${c}_$m.$s.log" >> "$1"
          grep -Eq '^(PASSED|FAILED): [0-9]' "$O/evtI_${c}_$m.$s.log" || echo "FAIL bench $s $c $m: gave no result" >> "$1"
        done
      else
        f=$((f + 1)); echo "FAIL bench: does not compile ($c, $m)" >> "$1"; head -20 "$O/evtI_${c}_$m.err" | sed 's/^/   /' >> "$1"
      fi
    done
  done
  echo "-- verilator --lint-only -Wall on the master and hps_ocram_evt with MISTER_OCRAM_EVT" >> "$1"
  if command -v verilator >/dev/null 2>&1; then
    vok=1
    for VS in "evt:-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_EVT" \
              "evtns:-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_EVT" \
              "evtall:-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_MOUNT0_SLOT=3 -DMISTER_OCRAM_FIO -DMISTER_OCRAM_EVT" \
              "evtoff:-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_EVT -DMISTER_DISABLE_SPI_EVT"; do
      vn=${VS%%:*}; VF=${VS#*:}
      for t in hps_ocram_master hps_ocram_evt; do
        if ! verilator --lint-only -Wall $VF --top-module $t -Isys sys/hps_ocram_master.sv $OCMF sys/lw_cdc.sv \
             > "$O/evtV_${t}_$vn.log" 2>&1 || [ -s "$O/evtV_${t}_$vn.log" ]; then
          vok=0; echo "FAIL verilator: $t ($vn)" >> "$1"; head -20 "$O/evtV_${t}_$vn.log" | sed 's/^/   /' >> "$1"
        fi
      done
    done
    if [ $vok -eq 1 ]; then echo "pass verilator: no warning" >> "$1"; n=$((n + 1))
    else f=$((f + 1)); fi
  else
    echo "SKIPPED verilator: not on PATH" >> "$1"
  fi
  if [ $f -eq 0 ]; then echo "PASSED: $n compiles and runs, 0 failures" >> "$1"
  else echo "FAILED: $n compiles and runs, $f failures" >> "$1"; fi
}

# twinj LOG: arm J. Each shape compiled once with OCRAM_TB_TWIN, plain, run under every seed; the gold files come
# first: the live master undefines MISTER_OCRAM_POLL and _MARK at its end, and the gold copies keep them.
twinj() {
  : > "$1"; n=0; f=0
  JG="tb/golden/gold_hps_ocram_master.sv tb/golden/gold_hps_ocram_rx.sv"
  JL=$(ls sys/hps_ocram_*.sv sys/ocram_*.sv 2>/dev/null | tr '\n' ' ')
  JE="-DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_OCRAM_POLL=10"
  JS="$JE -DMISTER_OCRAM_SECTOR_SLOT=2 -DMISTER_OCRAM_SECTOR_FRAME=588"
  JM="$JS -DMISTER_OCRAM_MOUNT0_SLOT=3 -DMISTER_OCRAM_MOUNT1_SLOT=4 -DMISTER_OCRAM_MOUNT_FRAME=512"
  JU="$JE -DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO"
  H33=-DOCRAM_TB_SYS_HP_PS=14762; H20=-DOCRAM_TB_SYS_HP_PS=25000
  JC="diag osd33 osd20 sec33 sec20 fio33 fio20 mnt33 mnt20 uio33 uio20 uion33 lane33 lane20 evt33 evt20 yosd ymnt yevt"
  for c in $JC; do
    case $c in
      diag) D="-DMISTER_OCRAM -DMISTER_OCRAM_DIAG -DMISTER_OCRAM_OSD -DMISTER_OCRAM_MARK=14 -DMISTER_OCRAM_POLL=10";;
      osd33) D="$JE $H33";; osd20) D="$JE $H20";; sec33) D="$JS $H33";; sec20) D="$JS $H20";;
      fio33) D="$JE -DMISTER_OCRAM_FIO $H33";; fio20) D="$JE -DMISTER_OCRAM_FIO $H20";;
      mnt33) D="$JM $H33";; mnt20) D="$JM $H20";;
      uio33) D="$JU $H33";; uio20) D="$JU $H20";; uion33) D="$JU -DMISTER_DEBUG_NOHDMI $H33";;
      lane33) D="$AD $H33";; lane20) D="$AD $H20";; evt33) D="$ID $H33";; evt20) D="$ID $H20";;
      yosd) D="$JE $H33 -DMISTER_DISABLE_SPI_OSD";; ymnt) D="$JM $H33 -DMISTER_DISABLE_SPI_OSD -DMISTER_DISABLE_SPI_BLK";;
      yevt) D="$ID -DMISTER_DISABLE_SPI_EVT $H33";;
    esac
    X=""; case $c in fio*|uio*|lane*) X="sys/lw_lane.sv sys/hps_io.sv";; evt*|yevt) X="sys/hps_io.sv";; esac
    echo "-- the twin bench compiles ($c)" >> "$1"; rm -f "$O/twinJ_$c.vvp"
    if iverilog -g2012 $D -DOCRAM_TB_TWIN -I sys -s ocram_tb -o "$O/twinJ_$c.vvp" tb/ocram_tb.sv $JG $JL sys/lw_cdc.sv \
         sys/osd.v $X > "$O/twinJ_$c.err" 2>&1; then
      echo "pass bench: compiles ($c)" >> "$1"; n=$((n + 1))
    else
      f=$((f + 1)); echo "FAIL bench: does not compile ($c)" >> "$1"; head -20 "$O/twinJ_$c.err" | sed 's/^/   /' >> "$1"
    fi
  done
  for c in $JC; do
    [ -f "$O/twinJ_$c.vvp" ] || continue
    for s in ${LW_TB_SEEDS:-1 2 3}; do vvp -n "$O/twinJ_$c.vvp" +lw_seed=$s > "$O/twinJ_$c.$s.log" 2>&1 & done
    wait
  done
  for c in $JC; do
    [ -f "$O/twinJ_$c.vvp" ] || continue
    for s in ${LW_TB_SEEDS:-1 2 3}; do
      echo "== seed $s $c" >> "$1"; cat "$O/twinJ_$c.$s.log" >> "$1"
      grep -Eq '^(PASSED|FAILED): [0-9]' "$O/twinJ_$c.$s.log" || echo "FAIL bench $s $c: gave no result" >> "$1"
      echo "-- the twin ($c, seed $s): every output equal on every edge" >> "$1"
      t=$(sed -n 's/^TWIN: \([0-9]*\) edges compared, \([0-9]*\) mismatches$/\1 \2/p' "$O/twinJ_$c.$s.log")
      if [ -z "$t" ]; then echo "FAIL twin $s $c: no TWIN line" >> "$1"
      elif [ "${t#* }" != 0 ] || [ "${t% *}" = 0 ]; then echo "FAIL twin $s $c: ${t% *} edges, ${t#* } mismatches" >> "$1"
      else echo "pass twin $s $c: ${t% *} edges, 0 mismatches" >> "$1"; fi
    done
  done
  if [ $f -eq 0 ]; then echo "PASSED: $n compiles and runs, 0 failures" >> "$1"
  else echo "FAILED: $n compiles and runs, $f failures" >> "$1"; fi
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
  # the new guards: each macro pairing that cannot be built stops naming its guard, and its legal neighbour compiles
  cat > "$O/lw_xo_top.sv" <<'EOS'
module lw_xo_top;
`ifndef XO_VD
`define XO_VD 1
`endif
`ifndef XO_BLKSZ
`define XO_BLKSZ 2
`endif
hps_io #(.CONF_STR("X;"), .VDNUM(`XO_VD), .BLKSZ(`XO_BLKSZ)) hps_io();
`ifdef XO_OSD
osd o();
`endif
`ifdef XO_MASTER
hps_ocram_master m();
`endif
`ifdef XO_WIN
lw_sys_window #(.SEC_EN(1), .SEC_EXT(1)) w();
`endif
`ifdef XO_RX
hps_ocram_rx #(.OSD_EN(0), .SEC_EN(1), .FRAME(`XO_FRAME), .GAP(`XO_GAP)) r();
`endif
`ifdef XO_FRX
hps_ocram_rx #(.OSD_EN(0), .SEC_EN(`XO_SEC), .FIO_EN(1), .FIO_RING(`XO_RING), .FIO_GAP(`XO_FGAP)) f();
`endif
`ifdef XO_MMASTER
hps_ocram_master #(.SEC_SLOT(1), .MNT0_SLOT(`XO_M0), .MNT1_SLOT(`XO_M1), .MNT_FRAME(`XO_MF)) mm();
`endif
`ifdef XO_MRX
hps_ocram_rx #(.OSD_EN(0), .SEC_EN(0), .FIO_EN(1), .MNT_EN(2'b01)) mr();
`endif
endmodule
EOS
  XOF="$O/lw_xo_top.sv sys/hps_io.sv sys/lw_cdc.sv sys/hps_ocram_master.sv $OCMF sys/hps_ocram_rx.sv $OCRF sys/lw_sys_window.sv sys/lw_sector.sv"
  XOF="$XOF sys/osd.v sys/lw_lane.sv"
  while IFS='|' read -r want guard what d; do
    echo "-- $what" >> "$1"
    iverilog -g2012 $d -I sys -s lw_xo_top -o "$O/lw_xo.vvp" $XOF > "$O/lw_xo.err" 2>&1
    r=$?; n=$((n + 1)); g=$(grep -c "$guard" "$O/lw_xo.err")
    if [ "$want" = bad ]; then
      if [ $r -ne 0 ] && [ "$g" -gt 0 ]; then echo "pass $what: stops, naming $guard" >> "$1"
      else f=$((f + 1)); echo "FAIL $what: status $r, $guard $g times in the errors" >> "$1"; fi
    elif [ $r -eq 0 ]; then echo "pass $what: compiles" >> "$1"
    else f=$((f + 1)); echo "FAIL $what: does not compile" >> "$1"; head -5 "$O/lw_xo.err" | sed 's/^/   /' >> "$1"; fi
  done <<'EOG'
bad|ocram_sector_needs_MISTER_LW|MISTER_OCRAM_SECTOR_SLOT without MISTER_LW|-DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1
ok|-|MISTER_OCRAM_SECTOR_SLOT with MISTER_LW|-DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_LW
bad|spi_pad_off_needs_MISTER_LW_PAD|MISTER_DISABLE_SPI_PAD without MISTER_LW_PAD|-DMISTER_LW -DMISTER_DISABLE_SPI_PAD
bad|spi_pad_off_needs_MISTER_LW_PAD|MISTER_DISABLE_SPI_PAD alone|-DMISTER_DISABLE_SPI_PAD
ok|-|MISTER_DISABLE_SPI_PAD with MISTER_LW_PAD|-DMISTER_LW -DMISTER_LW_PAD -DMISTER_DISABLE_SPI_PAD
bad|ocram_sector_excludes_MISTER_OCRAM_DIAG|MISTER_OCRAM_SECTOR_SLOT with MISTER_OCRAM_DIAG|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_DIAG -DMISTER_OCRAM_SECTOR_SLOT=1 -DXO_MASTER
ok|-|the master with MISTER_OCRAM_SECTOR_SLOT|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DXO_MASTER
bad|sector_slot_on_two_feeders|the LW SECTOR block and the OCRAM feeder on one window|-DMISTER_LW -DMISTER_LW_SECTOR_SLOT=1 -DXO_WIN
bad|ocram_sector_unservable_geometry|an OCRAM SECTOR frame of 0 words|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DXO_RX -DXO_FRAME=0 -DXO_GAP=2
bad|ocram_sector_unservable_geometry|an OCRAM SECTOR frame of 4097 words|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DXO_RX -DXO_FRAME=4097 -DXO_GAP=2
bad|ocram_sector_unservable_geometry|an OCRAM SECTOR gap of 1|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DXO_RX -DXO_FRAME=588 -DXO_GAP=1
bad|ocram_sector_unservable_geometry|an OCRAM SECTOR gap of 16|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DXO_RX -DXO_FRAME=588 -DXO_GAP=16
ok|-|the OCRAM SECTOR edges: 4096 words, gap 15|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DXO_RX -DXO_FRAME=4096 -DXO_GAP=15
ok|-|the OCRAM SECTOR edges: 1 word, gap 2|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DXO_RX -DXO_FRAME=1 -DXO_GAP=2
bad|link_on_two_players|MISTER_OCRAM_FIO beside MISTER_LW_LANE|-DMISTER_LW -DMISTER_LW_LANE -DMISTER_OCRAM -DMISTER_OCRAM_FIO
ok|-|MISTER_OCRAM_FIO beside MISTER_LW and its other tiers|-DMISTER_LW -DMISTER_LW_PAD -DMISTER_LW_EVT -DMISTER_OCRAM -DMISTER_OCRAM_FIO
ok|-|MISTER_OCRAM_FIO without MISTER_LW|-DMISTER_OCRAM -DMISTER_OCRAM_FIO
bad|ocram_fio_unservable_geometry|a download gap of 3|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_FRX -DXO_SEC=0 -DXO_RING=10 -DXO_FGAP=3
bad|ocram_fio_unservable_geometry|a download gap of 8|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_FRX -DXO_SEC=0 -DXO_RING=10 -DXO_FGAP=8
bad|ocram_fio_unservable_geometry|a download ring of 2^1 words|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_FRX -DXO_SEC=0 -DXO_RING=1 -DXO_FGAP=4
bad|ocram_fio_unservable_geometry|a download ring of 2^11 words|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_FRX -DXO_SEC=0 -DXO_RING=11 -DXO_FGAP=4
bad|ocram_fio_unservable_geometry|a download ring of 2^14 words|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_FRX -DXO_SEC=0 -DXO_RING=14 -DXO_FGAP=4
bad|ocram_fio_ring_on_ring4_span|a download ring of 2^13 beside the SECTOR tier|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_FRX -DXO_SEC=1 -DXO_RING=13 -DXO_FGAP=4
bad|ocram_fio_ring_on_ring4_span|a download ring of 2^13 beside DIAG|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DMISTER_OCRAM_DIAG -DXO_FRX -DXO_SEC=0 -DXO_RING=13 -DXO_FGAP=4
ok|-|the download edges: 2^13 words alone, gap 7|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_FRX -DXO_SEC=0 -DXO_RING=13 -DXO_FGAP=7
ok|-|the download edges: 2^2 words, gap 4|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_FRX -DXO_SEC=0 -DXO_RING=2 -DXO_FGAP=4
ok|-|the download edges: 2^10 words, gap 7|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_FRX -DXO_SEC=0 -DXO_RING=10 -DXO_FGAP=7
bad|ocram_uio_needs_MISTER_OCRAM_FIO|MISTER_OCRAM_UIO alone|-DMISTER_OCRAM_UIO
bad|ocram_uio_needs_MISTER_OCRAM_FIO|MISTER_OCRAM_UIO with the floor and no download tier|-DMISTER_OCRAM -DMISTER_OCRAM_UIO -DXO_MASTER
bad|ocram_uio_needs_MISTER_OCRAM_FIO|MISTER_OCRAM_UIO and MISTER_OCRAM_FIO without the floor|-DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO
ok|-|MISTER_OCRAM_UIO with the download tier|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO -DXO_MASTER -DXO_FRX -DXO_SEC=0 -DXO_RING=10 -DXO_FGAP=4
bad|ocram_evt_needs_MISTER_LW|MISTER_OCRAM_EVT without MISTER_LW|-DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_EVT
bad|evt_on_two_tiers|MISTER_OCRAM_EVT beside MISTER_LW_EVT|-DMISTER_LW -DMISTER_LW_EVT -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_EVT
ok|-|MISTER_OCRAM_EVT without the SECTOR tier|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_EVT
bad|ocram_evt_needs_MISTER_OCRAM|MISTER_OCRAM_EVT without the floor|-DMISTER_LW -DMISTER_OCRAM_EVT
ok|-|the master with MISTER_OCRAM_EVT and no SECTOR tier|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_EVT -DXO_MASTER
ok|-|MISTER_OCRAM_EVT with the SECTOR tier|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_EVT -DXO_MASTER
bad|spi_evt_off_needs_MISTER_OCRAM_EVT|MISTER_DISABLE_SPI_EVT alone|-DMISTER_DISABLE_SPI_EVT
bad|spi_evt_off_needs_MISTER_OCRAM_EVT|MISTER_DISABLE_SPI_EVT with MISTER_LW_EVT only|-DMISTER_LW -DMISTER_LW_EVT -DMISTER_DISABLE_SPI_EVT
ok|-|MISTER_DISABLE_SPI_EVT with the OCRAM events tier|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_EVT -DMISTER_DISABLE_SPI_EVT -DXO_MASTER
bad|ocram_lane_needs_MISTER_OCRAM_FIO|MISTER_OCRAM_LANE without the download tier|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_LANE
bad|ocram_lane_needs_MISTER_OCRAM_FIO|MISTER_OCRAM_LANE without the floor|-DMISTER_LW -DMISTER_OCRAM_LANE
ok|-|MISTER_OCRAM_LANE without the SECTOR tier|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DMISTER_OCRAM_LANE
ok|-|the master with MISTER_OCRAM_LANE and no SECTOR tier|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DMISTER_OCRAM_LANE -DXO_MASTER
bad|ocram_lane_needs_MISTER_OCRAM_FIO|the master with MISTER_OCRAM_LANE and no download tier|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_LANE -DXO_MASTER
ok|-|MISTER_OCRAM_LANE with the SECTOR and download tiers|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_FIO -DMISTER_OCRAM_LANE -DXO_MASTER
ok|-|MISTER_OCRAM_UIO with the download tier, no HDMI osd|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DMISTER_OCRAM_UIO -DMISTER_DEBUG_NOHDMI -DXO_MASTER -DXO_FRX -DXO_SEC=0 -DXO_RING=10 -DXO_FGAP=4
bad|ocram_mount_needs_MISTER_OCRAM_SECTOR_SLOT|MISTER_OCRAM_MOUNT0_SLOT without the SECTOR tier|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_MOUNT0_SLOT=2
bad|ocram_mount_needs_MISTER_OCRAM_SECTOR_SLOT|MISTER_OCRAM_MOUNT1_SLOT without the SECTOR tier|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_MOUNT1_SLOT=3
bad|ocram_sector_needs_MISTER_LW|the mount slots without MISTER_LW|-DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2
bad|sector_slot_on_two_feeders|the mount slots beside the LW SECTOR block|-DMISTER_LW -DMISTER_LW_SECTOR_SLOT=1 -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DXO_WIN
bad|ocram_mount_needs_MISTER_OCRAM_SECTOR_SLOT|the walker with a mount slot and no SECTOR ring|-DMISTER_OCRAM -DMISTER_OCRAM_FIO -DXO_MRX
ok|-|two mount slots beside the SECTOR tier|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DXO_MMASTER -DXO_M0=2 -DXO_M1=3 -DXO_MF=256
bad|ocram_mount_unservable_geometry|a mount slot on the disc slot|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=1 -DXO_MMASTER -DXO_M0=1 -DXO_M1=3 -DXO_MF=256
bad|ocram_mount_unservable_geometry|two mount slots on one hps_io slot|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=2 -DXO_MMASTER -DXO_M0=2 -DXO_M1=2 -DXO_MF=256
bad|ocram_mount_unservable_geometry|a mount slot of 16|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT1_SLOT=16 -DXO_MMASTER -DXO_M0=2 -DXO_M1=16 -DXO_MF=256
bad|ocram_mount_unservable_geometry|a mount frame of 0 words|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DXO_MMASTER -DXO_M0=2 -DXO_M1=3 -DXO_MF=0
bad|ocram_mount_unservable_geometry|a mount frame of 1021 words|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DXO_MMASTER -DXO_M0=2 -DXO_M1=3 -DXO_MF=1021
ok|-|the mount edges: 1,020 words|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DXO_MMASTER -DXO_M0=2 -DXO_M1=3 -DXO_MF=1020
ok|-|the mount edges: 1 word, slot 15|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT1_SLOT=15 -DXO_MMASTER -DXO_M0=2 -DXO_M1=15 -DXO_MF=1
bad|spi_osd_off_needs_an_osd_tier|MISTER_DISABLE_SPI_OSD alone|-DMISTER_DISABLE_SPI_OSD -DXO_OSD
bad|spi_osd_off_needs_an_osd_tier|MISTER_DISABLE_SPI_OSD with MISTER_OCRAM_OSD and no MISTER_OCRAM|-DMISTER_OCRAM_OSD -DMISTER_DISABLE_SPI_OSD -DXO_OSD
bad|spi_osd_off_needs_an_osd_tier|MISTER_DISABLE_SPI_OSD with MISTER_LW_OSD and no MISTER_LW|-DMISTER_LW_OSD -DMISTER_DISABLE_SPI_OSD -DXO_OSD
bad|spi_osd_off_needs_an_osd_tier|MISTER_DISABLE_SPI_OSD beside MISTER_LW and its other tiers, no OSD tier|-DMISTER_LW -DMISTER_LW_PAD -DMISTER_OCRAM -DMISTER_DISABLE_SPI_OSD -DXO_OSD
ok|-|MISTER_DISABLE_SPI_OSD with the OCRAM OSD tier|-DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_DISABLE_SPI_OSD -DXO_OSD
ok|-|MISTER_DISABLE_SPI_OSD with the LW OSD tier|-DMISTER_LW -DMISTER_LW_OSD -DMISTER_DISABLE_SPI_OSD -DXO_OSD
ok|-|MISTER_DISABLE_SPI_OSD with both OSD tiers|-DMISTER_LW -DMISTER_LW_OSD -DMISTER_OCRAM -DMISTER_OCRAM_OSD -DMISTER_DISABLE_SPI_OSD -DXO_OSD
bad|spi_blk_off_needs_a_block_tier|MISTER_DISABLE_SPI_BLK alone|-DMISTER_DISABLE_SPI_BLK
bad|spi_blk_off_needs_a_block_tier|MISTER_DISABLE_SPI_BLK with the LW SECTOR block only (not counted)|-DMISTER_LW -DMISTER_LW_SECTOR_SLOT=0 -DMISTER_DISABLE_SPI_BLK
bad|spi_blk_off_needs_a_block_tier|MISTER_DISABLE_SPI_BLK with MISTER_OCRAM_SECTOR_SLOT and no MISTER_OCRAM (not counted)|-DMISTER_LW -DMISTER_OCRAM_SECTOR_SLOT=0 -DMISTER_DISABLE_SPI_BLK
bad|spi_blk_off_needs_a_block_tier|MISTER_DISABLE_SPI_BLK with every slot named unused, no tier|-DMISTER_DISABLE_SPI_BLK -DMISTER_DISABLE_SPI_BLK_UNUSED=1
ok|-|MISTER_DISABLE_SPI_BLK with the OCRAM SECTOR tier on the one slot|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=0 -DMISTER_DISABLE_SPI_BLK
bad|spi_blk_off_slot_without_a_tier|MISTER_DISABLE_SPI_BLK, VDNUM 4, slot 0 neither on a tier nor named unused|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DXO_VD=4 -DMISTER_DISABLE_SPI_BLK
bad|spi_blk_off_slot_without_a_tier|MISTER_DISABLE_SPI_BLK, VDNUM 4, the mount slots missing, slot 0 named unused|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DXO_VD=4 -DMISTER_DISABLE_SPI_BLK -DMISTER_DISABLE_SPI_BLK_UNUSED=1
bad|spi_blk_off_unused_slot_on_a_tier|MISTER_DISABLE_SPI_BLK, VDNUM 4, the SECTOR slot also named unused|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DXO_VD=4 -DMISTER_DISABLE_SPI_BLK -DMISTER_DISABLE_SPI_BLK_UNUSED=3
ok|-|MISTER_DISABLE_SPI_BLK, VDNUM 4: SECTOR slot 1, mount slots 2 and 3, slot 0 named unused|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DXO_VD=4 -DMISTER_DISABLE_SPI_BLK -DMISTER_DISABLE_SPI_BLK_UNUSED=1
ok|-|the same slots without MISTER_DISABLE_SPI_BLK|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DXO_VD=4
bad|spi_blk_off_tier_slot_outside_vdnum|MISTER_DISABLE_SPI_BLK, VDNUM 2: SECTOR slot 1, slot 0 named unused, the mount slots at and above VDNUM|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DXO_VD=2 -DMISTER_DISABLE_SPI_BLK -DMISTER_DISABLE_SPI_BLK_UNUSED=1
bad|spi_blk_off_tier_slot_outside_vdnum|MISTER_DISABLE_SPI_BLK, VDNUM 1: the SECTOR slot 1, slot 0 named unused|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_DISABLE_SPI_BLK -DMISTER_DISABLE_SPI_BLK_UNUSED=1
bad|ocram_blksz_differs_from_hps_io|the mount slots with MISTER_OCRAM_BLKSZ 3 and hps_io's BLKSZ 2|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DMISTER_OCRAM_BLKSZ=3
bad|ocram_blksz_differs_from_hps_io|one mount slot with MISTER_OCRAM_BLKSZ left at 2 and hps_io's BLKSZ 3|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DXO_BLKSZ=3
ok|-|the mount slots with MISTER_OCRAM_BLKSZ 3 and hps_io's BLKSZ 3|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DMISTER_OCRAM_BLKSZ=3 -DXO_BLKSZ=3
ok|-|the mount slots with MISTER_DISABLE_SPI_BLK at BLKSZ 3, VDNUM 4, slot 0 named unused|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_MOUNT1_SLOT=3 -DMISTER_OCRAM_BLKSZ=3 -DXO_BLKSZ=3 -DXO_VD=4 -DMISTER_DISABLE_SPI_BLK -DMISTER_DISABLE_SPI_BLK_UNUSED=1
ok|-|MISTER_OCRAM_BLKSZ 3 and hps_io's BLKSZ 2 with the SECTOR tier alone (no mount slot)|-DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_BLKSZ=3
ok|-|the mount macros and MISTER_OCRAM_BLKSZ 3 without MISTER_OCRAM, hps_io's BLKSZ 2|-DMISTER_LW -DMISTER_OCRAM_SECTOR_SLOT=1 -DMISTER_OCRAM_MOUNT0_SLOT=2 -DMISTER_OCRAM_BLKSZ=3
EOG
  # the key tier's guards: MISTER_LW_KBM needs the PAD tier and refuses a PS2DIV core; MISTER_DISABLE_SPI_KBM needs it.
  # No PS2DIV neighbour that compiles: stock hps_io with PS2DIV set does not compile under Icarus, so a PS2DIV case
  # can only be a refusal.
  cat > "$O/lw_xk_top.sv" <<'EOS'
module lw_xk_top;
`ifdef XK_PS2DIV
hps_io #(.CONF_STR("X;"), .PS2DIV(`XK_PS2DIV)) hps_io();
`else
hps_io #(.CONF_STR("X;")) hps_io();
`endif
endmodule
EOS
  while IFS='|' read -r want guard what d; do
    echo "-- $what" >> "$1"
    iverilog -g2012 $d -I sys -s lw_xk_top -o "$O/lw_xk.vvp" "$O/lw_xk_top.sv" sys/hps_io.sv > "$O/lw_xk.err" 2>&1
    r=$?; n=$((n + 1)); g=$(grep -c "$guard" "$O/lw_xk.err")
    if [ "$want" = bad ]; then
      if [ $r -ne 0 ] && [ "$g" -gt 0 ]; then echo "pass $what: stops, naming $guard" >> "$1"
      else f=$((f + 1)); echo "FAIL $what: status $r, $guard $g times in the errors" >> "$1"; fi
    elif [ $r -eq 0 ]; then echo "pass $what: compiles" >> "$1"
    else f=$((f + 1)); echo "FAIL $what: does not compile" >> "$1"
      grep -v ': warning: ' "$O/lw_xk.err" | head -5 | sed 's/^/   /' >> "$1"; fi
  done <<'EOG'
bad|lw_kbm_needs_MISTER_LW_PAD|MISTER_LW_KBM alone|-DMISTER_LW_KBM
bad|lw_kbm_needs_MISTER_LW_PAD|MISTER_LW_KBM with MISTER_LW and no MISTER_LW_PAD|-DMISTER_LW -DMISTER_LW_KBM
bad|lw_tier_needs_MISTER_LW|MISTER_LW_KBM with MISTER_LW_PAD and no MISTER_LW|-DMISTER_LW_PAD -DMISTER_LW_KBM
ok|-|MISTER_LW_KBM with MISTER_LW_PAD|-DMISTER_LW -DMISTER_LW_PAD -DMISTER_LW_KBM
bad|lw_kbm_on_a_PS2DIV_core|MISTER_LW_KBM on a core with PS2DIV|-DMISTER_LW -DMISTER_LW_PAD -DMISTER_LW_KBM -DXK_PS2DIV=2000
bad|spi_kbm_off_needs_MISTER_LW_KBM|MISTER_DISABLE_SPI_KBM alone|-DMISTER_DISABLE_SPI_KBM
bad|spi_kbm_off_needs_MISTER_LW_KBM|MISTER_DISABLE_SPI_KBM with MISTER_LW_PAD, no MISTER_LW_KBM|-DMISTER_LW -DMISTER_LW_PAD -DMISTER_DISABLE_SPI_KBM
ok|-|MISTER_DISABLE_SPI_KBM with MISTER_LW_KBM|-DMISTER_LW -DMISTER_LW_PAD -DMISTER_LW_KBM -DMISTER_DISABLE_SPI_KBM
ok|-|both pad removals with both pad tiers|-DMISTER_LW -DMISTER_LW_PAD -DMISTER_LW_KBM -DMISTER_DISABLE_SPI_PAD -DMISTER_DISABLE_SPI_KBM
bad|lw_kbm_on_a_PS2DIV_core|MISTER_DISABLE_SPI_KBM on a core with PS2DIV|-DMISTER_LW -DMISTER_LW_PAD -DMISTER_LW_KBM -DMISTER_DISABLE_SPI_KBM -DXK_PS2DIV=2000
EOG
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
  # the same reader on the OCRAM SECTOR tier (WIDE strobes, BLKSZ 3): it compiles on slot 3 and stops on slot 0
  for t in ok:3 bad:0; do
    want=${t%%:*}; k=${t#*:}
    echo "-- Template: the block reader on the OCRAM SECTOR tier, hps_io slot $k" >> "$1"
    iverilog -g2012 -DMISTER_LW -DMISTER_OCRAM -DMISTER_OCRAM_SECTOR_SLOT=$k -DMISTER_OCRAM_BLKSZ=3 -I "$O" -I sys \
      -s emu -o "$O/lw_xt.vvp" Template.sv sys/hps_io.sv sys/lw_blk_mux.sv rtl/mycore.v rtl/lfsr.v \
      "$O/lw_xt_stubs.sv" > "$O/lw_xt.err" 2>&1
    r=$?; n=$((n + 1)); g=$(grep -c lw_template_block_slot_1_to_9 "$O/lw_xt.err")
    if [ "$want" = bad ]; then
      if [ $r -ne 0 ] && [ "$g" -gt 0 ]; then echo "pass OCRAM slot $k: stops, naming the guard" >> "$1"
      else f=$((f + 1)); echo "FAIL Template, OCRAM slot $k: status $r, the guard $g times in the errors" >> "$1"; fi
    elif [ $r -eq 0 ]; then echo "pass OCRAM slot $k: compiles" >> "$1"
    else f=$((f + 1)); echo "FAIL Template, OCRAM slot $k: does not compile" >> "$1"; grep -i error "$O/lw_xt.err" | head -5 | sed 's/^/   /' >> "$1"; fi
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
  if [ "$g" = O ]; then
    echo "== arm O: tb/ocram_tb.sv, every seed, plain and with the CDC models; verilator lint"
    ocram "$O/ocram_tbO.log"
    grep -v "^pass " "$O/ocram_tbO.log"
    V=$(verdict O "$O/ocram_tbO.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = Q ]; then
    echo "== arm Q: tb/confstr_tb.sv, the CONF_STR readout from logic and from block RAM"
    awk '/^reg \[MAX_W:0\] byte_cnt;/ { next } /^wire \[7:0\] conf_byte;/ { print "reg [MAX_W:0] byte_cnt;" } { print }' \
      sys/hps_io.sv > "$O/confstrQ_hps_io.sv"
    if iverilog -g2012 -I tb -I sys -o "$O/confstrQ.vvp" tb/confstr_tb.sv "$O/confstrQ_hps_io.sv" > "$O/confstrQ.err" 2>&1
    then vvp -n "$O/confstrQ.vvp" > "$O/confstr_tbQ.log" 2>&1
    else { echo "FAIL bench: does not compile"; head -20 "$O/confstrQ.err"; } > "$O/confstr_tbQ.log"; fi
    grep -v "^pass " "$O/confstr_tbQ.log"
    V=$(verdict Q "$O/confstr_tbQ.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = C ]; then
    echo "== arm C: tb/sfeed_tb.sv, lw_sector against the OCRAM SECTOR feeder, cycle for cycle"
    sfeedc "$O/sfeed_tbC.log"
    grep -v "^pass " "$O/sfeed_tbC.log" | grep -v '^== seed'
    V=$(verdict C "$O/sfeed_tbC.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = D ]; then
    echo "== arm D: tb/fio_tb.sv, the LW lane's player against the OCRAM download player, edge for edge"
    fioc "$O/fio_tbD.log"
    grep -v "^pass " "$O/fio_tbD.log" | grep -v '^== seed'
    V=$(verdict D "$O/fio_tbD.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = V ]; then
    echo "== arm V: slang"
    if ! command -v slang >/dev/null 2>&1; then V="arm V: SKIPPED (no slang on PATH)"
    elif lint "$O/lw_lintV.log"; then V="arm V: PASSED: slang, 8 runs, no error"
    else grep -E 'error|warning' "$O/lw_lintV.log"; V="arm V: FAILED (slang errors, $O/lw_lintV.log)"; rc=1
    fi
    R="$R$V
"
    continue
  fi
  if [ "$g" = Y ]; then
    echo "== arm Y: the removals MISTER_DISABLE_SPI_OSD and MISTER_DISABLE_SPI_BLK"
    rmy "$O/rmY.log"
    grep -v "^pass " "$O/rmY.log" | grep -v '^== seed'
    V=$(verdict Y "$O/rmY.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = K ]; then
    echo "== arm K: MISTER_LW_KBM, the keys, the mouse and the triggers on the PAD block; verilator lint"
    kbmk "$O/kbm_tbK.log"
    grep -v "^pass " "$O/kbm_tbK.log" | grep -v '^== seed'
    V=$(verdict K "$O/kbm_tbK.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = A ]; then
    echo "== arm A: tb/ocram_tb.sv with MISTER_OCRAM_LANE: capture frames and their REPLY records"
    aarm "$O/laneA.log"
    grep -v "^pass " "$O/laneA.log" | grep -v '^== seed'
    V=$(verdict A "$O/laneA.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = I ]; then
    echo "== arm I: tb/ocram_tb.sv with MISTER_OCRAM_EVT: the events tier's records against their link polls"
    iarm "$O/evtI.log"
    grep -v "^pass " "$O/evtI.log" | grep -v '^== seed'
    V=$(verdict I "$O/evtI.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = J ]; then
    echo "== arm J: tb/ocram_tb.sv with the gold twin, the live pair against tb/golden/ on every edge"
    twinj "$O/twinJ.log"
    grep -v "^pass " "$O/twinJ.log" | grep -v '^== seed'
    V=$(verdict J "$O/twinJ.log") || rc=1
    R="$R$V
"
    continue
  fi
  if [ "$g" = U ]; then
    echo "== arm U: tb/ocram_tb.sv with MISTER_OCRAM_UIO: UIO and OSD frames on ring 1 against their link twins"
    uarm "$O/uioU.log"
    grep -v "^pass " "$O/uioU.log" | grep -v '^== seed'
    V=$(verdict U "$O/uioU.log") || rc=1
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
    P) D="$B $A -DLW_DIAG=0 -DMISTER_DISABLE_SPI_PAD";;
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
