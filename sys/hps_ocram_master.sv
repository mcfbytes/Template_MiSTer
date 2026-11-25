//============================================================================
//  hps_ocram_master.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 3 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program.  If not, see <https://www.gnu.org/licenses/>.
//============================================================================

`ifdef MISTER_OCRAM
`default_nettype none

// The floor of the OCRAM mailbox: the AXI3 master on the fpga2hps bridge (ocram_axi.sv, the bus; ocram_arb.sv, its
// request ports), its run latch, the poller, the hello, the EVT ring's records (ocram_evt_wr.sv) and interrupt line,
// its consumer port, and (with MISTER_OCRAM_DIAG) the diagnostic
// counters and the host-to-fabric torture sink. The design note is the law, spec/ocram-mailbox-design.md (the
// Template's reader-facing copy is docs/OCRAM_MAILBOX.md); this file is layout 1 only.
//
//  * It runs on clk_100m and no fabric reset holds it. It never abandons a transaction it started: a core reset,
//    a stopped clk_sys or a host that has gone cannot leave one in flight.
//  * It cannot address anything but the mailbox: every address is {16'hFFFF, off}, and off is a localparam
//    (below 0x100), an EVT-ring word, EVT_BASE + ((index & (EVT_WORDS - 1)) << 2), or a consumer's own bounded
//    offset (hps_ocram_rx.sv, hps_ocram_tsink below). tb/static.sh holds it.
//  * One transaction at a time, single beat, full 32-bit words. Write: AWVALID and WVALID together, each held
//    with its payload until its ready, then BREADY while the response is awaited. Read: ARVALID held until
//    ARREADY, then RREADY while the response is awaited.
//  * One arbiter, fixed order: the terminal store; the fault's C_STATE store; the poll's and the line's reads when
//    due; the hello's stores; a C_HELLO state store; a C_BIND store; the EVT ring's records; then the consumers,
//    round robin between the two ports (the last granted goes last). A consumer is granted only in P_SESSION and
//    never in a latch-set or latch-clear cycle. The records, in their own fixed order: BIND, the disc slot's REQUEST,
//    a mount slot's REQUEST or WRITE, REPLY, the events tier's, MARK, VERDICT.
//  * The run latch (§2.6): it sets on a rising edge of GPO bit 28 (RUN) seen while GPO bit 31 is 1 and no HPS
//    reset is seen; it clears on RUN low, bit 31 low or an HPS reset, and bit 31's return never sets it. Every GPO bit
//    and hps_rst_n enters through an lw_arst cell, never raw.
//  * On a stop: the transaction in flight completes, a record being written is abandoned (its head is not
//    stored), then one terminal store C_HELLO = 0x51540000. An HPS reset ends a run with no terminal store.
//  * An HPS reset makes the master start nothing; a transaction in flight keeps its valids and readies as AXI
//    requires and takes its response if one comes; in the cycle the reset level falls, one still unanswered is
//    dropped.
//  * The hello waits for every built consumer's clk_sys half to rest before its first store. While it waits it
//    starts nothing but the terminal store; a consumer that never rests (a frozen clk_sys) leaves the hello
//    unechoed, which is the design's answer, not a hang.
//  * MARK records, the counters that fill them, the torture sink and the GPO 27/26 test knobs exist only with
//    MISTER_OCRAM_DIAG; hps_ocram_diag and hps_ocram_tsink are separate modules so the fit reports their ALMs
//    apart. The host-to-core rings' consumer is hps_ocram_rx (MISTER_OCRAM_OSD, MISTER_OCRAM_SECTOR_SLOT).
//  * MISTER_OCRAM_SECTOR_SLOT (not with MISTER_OCRAM_DIAG: ring 4's span is shared): the SECTOR
//    ring r = 4, H_SECTOR read in the session poll, C_BIND bit 0 (stored as hps_ocram_rx reports it), C_K[0..7],
//    and the REQUEST record {0x21, 0, 1} then the LBA, written by the EVT ring's writer behind a room check of 2
//    words. After every C_BIND store it posts BIND {0x27, C_BIND[7:0], 0} (C_CAPS bit 10): a C_BIND store waits
//    for a record in flight and for an owed BIND, and no REQUEST or WRITE starts while either is owed.
//  * MISTER_OCRAM_MOUNT0_SLOT, _MOUNT1_SLOT (beside the SECTOR tier): the mount ring r = 0 (hps_ocram_rx plays its
//    FRAME records), C_CAPS bit 0, C_RING[0], C_K[10, 11], H_SECTOR and C_BIND bits 1, 2, and the mount slots'
//    records: REQUEST {0x21, {blk_cnt, k}, 1} then the LBA, and WRITE {0x26, {blk_cnt, k}, 1 + n} then the LBA and
//    the n words hps_ocram_rx hands over one at a time, behind a room check of n + 2 words; the head store
//    publishes it and m_pub tells hps_ocram_rx, whose feeder then drops the slot's ack.
//  * MISTER_OCRAM_FIO: the download ring r = 1 (hps_ocram_rx plays its LFRAME records), C_CAPS bit 1, C_RING[1]
//    with line ordinal 5, C_K[9] {sel mask, gap} (MISTER_OCRAM_UIO widens the mask to the UIO and OSD selects,
//    0x000F, or 0x000B with MISTER_DEBUG_NOHDMI, and takes no C_CAPS bit), and the ring's mark line: H_MARK[1] is
//    read in the session poll, and the line (mark_line, f2h_irq[12]) is high in a session while H_MARK[1] bit 31
//    is set and the ring's tail is at or past H_MARK[1][15:0] (modulo 2^16, within half the index space). While it
//    is high H_MARK[1] is re-read back to back, alternating with the EVT line's re-read and yielding to a waiting
//    consumer as it does.
//  * MISTER_OCRAM_EVT (with MISTER_LW, whose events span of HPS_BUS its port takes): C_CAPS bit 11, H_BIND
//    and C_BIND bit 8 (the events bind, read in the session poll until it is set, as the OSD bind is), and d's
//    records on the EVT ring: STATUS {0x20, 0, 4} then status_req[31:0] first, INFO {0x28, info, 0}, UPLOAD
//    {0x29, index, 0} and RUMBLE {0x2A, pad, 1} then {0, level}. hps_ocram_evt (below) carries hps_io's event port
//    one offer at a time: the master asks while C_BIND bit 8 is stored 1 and the offer it holds is spent. A STATUS
//    word 0 whose stflg is not the last one published opens a record; words 1-3 follow, each with stflg, and one
//    whose stflg moved abandons the record (its head is not stored). A RUMBLE whose level is the last published
//    (0 at each hello) is spent with no record; an INFO with arg 0 is no notice. d's records go after an owed BIND,
//    REQUEST or WRITE. MISTER_DISABLE_SPI_EVT sets C_CAPS bit 18.
//  * MISTER_OCRAM_LANE (beside the download tier): C_CAPS bit 12, and REPLY {0x22, arg, n} on the EVT
//    ring for each capture LFRAME hps_ocram_rx plays: arg the frame's, echoed whole, n its len; each payload word
//    two link replies, low half first, as the walker hands them over. The walker holds the frame until e_free (the
//    EVT ring's free words) covers n + 1; p_post asks for the record, which opens after an owed BIND, REQUEST and
//    WRITE and before d's, and stays open while the frame plays.
//
// Address map, layout 1 (offsets from 0xFFFF0000): the host half of the control page H_HELLO, H_BIND, H_LINE,
// H_IDX[2], H_IDX[3], H_IDX[4]; the fabric half C_HELLO, C_VERSION, C_CAPS, C_STATE, C_IDX[2], C_IDX[3],
// C_IDX[4], C_BIND, C_RING[2..4], C_K[8]; the EVT ring at EVT_BASE, 2,048 words; the torture ring at TS_BASE,
// 8,192 words (the SECTOR span, DIAG test cores only); H_SECTOR and C_K[0..2, 4, 5] with the SECTOR tier;
// H_MARK[1], C_IDX[1], C_RING[1] and C_K[9] with the download tier; C_IDX[0], C_RING[0] and C_K[10, 11] with the
// mount slots. The OSD, SECTOR, download and mount rings are hps_ocram_rx.sv's.
// Every other word is never read and never written.

`ifndef MISTER_OCRAM_POLL
`define MISTER_OCRAM_POLL 12
`endif
`ifndef MISTER_OCRAM_MARK
`define MISTER_OCRAM_MARK 20
`endif

module hps_ocram_master #(
	// The SECTOR tier's declaration (ocram_cfg.vh maps the MISTER_OCRAM_SECTOR_* macros), for C_K[0..7]
	parameter [31:0] SEC_SLOT  = 32'd1,
	parameter [31:0] SEC_FRAME = 32'd128,
	parameter [31:0] SEC_GAP   = 32'd2,
	parameter  [2:0] SEC_BLKSZ = 3'd2,
	// The download tier's declaration (MISTER_OCRAM_FIO_RING, MISTER_OCRAM_FIO_GAP), for C_RING[1] and C_K[9]
	parameter [31:0] FIO_RING  = 32'd10,
	parameter [31:0] FIO_GAP   = 32'd4,
	// the selects an LFRAME may name, {osd_vga, osd_hdmi, fpga, uio}: ocram_cfg.vh's OCRAM_FIO_SELS, also the walker's
	parameter  [3:0] FIO_SELS  = 4'b0010,
	// The mount slots' declaration (MISTER_OCRAM_MOUNT0_SLOT, _MOUNT1_SLOT, _MOUNT_FRAME), for C_K[10, 11]
	parameter [31:0] MNT0_SLOT = 32'd2,
	parameter [31:0] MNT1_SLOT = 32'd3,
	parameter [31:0] MNT_FRAME = 32'd128
)
(
	input  wire        clk,          // clk_100m == h2f_user0_clk
	input  wire        hps_rst_n,    // the HPS's fabric reset (h2f_rst_n), asynchronous
	/* verilator lint_off UNUSEDSIGNAL */   // the floor reads bits 31, 28 and (DIAG) 27, 26; GPO's other users own the rest
	input  wire [31:0] gp_out,       // raw GPO: bits 31 and 28, and with DIAG 27 and 26
	/* verilator lint_on UNUSEDSIGNAL */

	output wire        present,      // the constant 1: this floor is built
	output reg         line,         // EVT ring line ordinal 4 (f2h_irq[11]), a clk register

	// consumer port 0: the OSD walker (MISTER_OCRAM_OSD; tied off when unbuilt)
	input  wire        c0_req,
	input  wire        c0_we,
	input  wire [15:0] c0_off,
	input  wire [31:0] c0_wdata,
	output wire        c0_grant,
	output wire        c0_done,
	output wire        c0_ok,
	output wire [31:0] c0_rdata,
	output wire        s_hello,      // one cycle: the hello's first store is about to run
	output wire        s_live,       // P_SESSION
	output wire        s_tick,       // the poll period's pulse
	output wire        osd_bind,     // the OSD bind (H_BIND bit 11 read this session)
	input  wire        c0_rest,      // the OSD half's enable link is idle
	input  wire        c0_fault,     // one cycle: the OSD ring stopped on a fault
	input  wire        c0_mid,       // level: an OSD record is part-consumed
	input  wire        c0_open,      // the OSD enable link's OPEN
	/* verilator lint_off UNUSEDSIGNAL */   // the VERDICT record's (DIAG), the REQUEST's (SECTOR): one or the other
	input  wire [15:0] osd_rows,     // ROW records the OSD walker finished this session
	input  wire [15:0] osd_deepest,  // its deepest fill
	output wire  [2:0] sec_ask,      // H_SECTOR bits 2:0, read this session
	output wire        evt_stop,     // the EVT ring stopped on a fault
	input  wire        c0_sfault,    // one cycle: the SECTOR ring stopped on a fault
	input  wire        c0_smid,      // level: a SECTOR record is part-consumed
	input  wire  [2:0] c0_sbind,     // the C_BIND bits 2:0 to store
	input  wire        c0_wide,      // hps_io's io_wide, through a crossing cell (C_K[1])
	input  wire        rq_post,      // a REQUEST for rq_lba is owed
	input  wire [31:0] rq_lba,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        rq_start,     // one cycle: that REQUEST's header store is launched
	/* verilator lint_off UNUSEDSIGNAL */   // the mount slots' (MISTER_OCRAM_MOUNT0_SLOT, _MOUNT1_SLOT)
	input  wire        c0_mfault,    // one cycle: the mount ring stopped on a fault
	input  wire        c0_mmid,      // level: a mount record is part-consumed
	input  wire        m_post,       // a mount slot's REQUEST or WRITE is owed
	input  wire        m_write,      // it is a WRITE
	input  wire  [7:0] m_arg,        // {blk_cnt, k}
	input  wire [10:0] m_len,        // its len: 1 for a REQUEST, 1 + the block's words for a WRITE
	input  wire [31:0] m_lba,
	input  wire        m_wv,         // a WRITE's next word is in m_wd
	input  wire [31:0] m_wd,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        m_start,      // one cycle: that record's header store is launched
	output wire        m_wtake,      // one cycle: a WRITE word's store is launched
	output wire        m_pub,        // one cycle: the WRITE's head store is done
	/* verilator lint_off UNUSEDSIGNAL */   // the download tier's (MISTER_OCRAM_FIO)
	input  wire [15:0] f_tail,       // the download ring's tail, as last stored in C_IDX[1]
	input  wire        c0_ffault,    // one cycle: the download ring stopped on a fault
	input  wire        c0_fmid,      // level: a download record is part-consumed
	/* verilator lint_on UNUSEDSIGNAL */
	output reg         mark_line,    // the download ring's mark line, ordinal 5 (f2h_irq[12]), a clk register
	// the events tier's port (MISTER_OCRAM_EVT, hps_ocram_evt below): ask for the next offer, take it
	output wire        e_ask,        // one cycle: the master asks for hps_io's next offer
	/* verilator lint_off UNUSEDSIGNAL */   // without MISTER_OCRAM_EVT the offers are not read
	input  wire        e_v,          // one cycle: an offer is in e_d
	input  wire [41:0] e_d,          // {kind[1:0], arg[7:0], data[31:0]}, held until the next
	/* verilator lint_on UNUSEDSIGNAL */
	// the capture lane's REPLY (MISTER_OCRAM_LANE): hps_ocram_rx's capture frame and its replies
	output wire [15:0] e_free,       // the EVT ring's free words
	/* verilator lint_off UNUSEDSIGNAL */   // without MISTER_OCRAM_LANE the capture lane is not read
	input  wire        p_post,       // a capture frame is held for its REPLY record
	input  wire  [7:0] p_arg,        // the frame's arg
	input  wire [10:0] p_n,          // its len: the REPLY's payload words
	input  wire        p_wv,         // a payload word is in p_wd
	input  wire [31:0] p_wd,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        p_start,      // one cycle: the REPLY's header is written; the frame may play
	output wire        p_wtake,      // one cycle: the master takes p_wd

	// fpga2hps AXI3 master, 32-bit data
	output wire  [7:0] awid,
	output wire [31:0] awaddr,
	output wire  [3:0] awlen,
	output wire  [2:0] awsize,
	output wire  [1:0] awburst,
	output wire  [1:0] awlock,
	output wire  [3:0] awcache,
	output wire  [2:0] awprot,
	output wire  [4:0] awuser,
	output wire        awvalid,
	input  wire        awready,
	output wire  [7:0] wid,
	output wire [31:0] wdata,
	output wire  [3:0] wstrb,
	output wire        wlast,
	output wire        wvalid,
	input  wire        wready,
	/* verilator lint_off UNUSEDSIGNAL */   // single-beat, one outstanding: the floor never needs a response ID
	input  wire  [7:0] bid,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire  [1:0] bresp,
	input  wire        bvalid,
	output wire        bready,
	output wire  [7:0] arid,
	output wire [31:0] araddr,
	output wire  [3:0] arlen,
	output wire  [2:0] arsize,
	output wire  [1:0] arburst,
	output wire  [1:0] arlock,
	output wire  [3:0] arcache,
	output wire  [2:0] arprot,
	output wire  [4:0] aruser,
	output wire        arvalid,
	input  wire        arready,
	/* verilator lint_off UNUSEDSIGNAL */   // single-beat, one outstanding: the floor never needs a response ID
	input  wire  [7:0] rid,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [31:0] rdata,
	input  wire  [1:0] rresp,
	/* verilator lint_off UNUSEDSIGNAL */   // a single-beat read: RLAST is implied and never read
	input  wire        rlast,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        rvalid,
	output wire        rready
);

///////////////////////////////  the map  ///////////////////////////////////

// The tiers' enables, the OSD rows and C_CAPS, from the .qsf macros.
`include "sys/ocram_cfg.vh"

localparam [15:0] H_HELLO = 16'h0000;      // host: {0x4F43, nonce}, 0 = no session
localparam [15:0] H_BIND  = 16'h0004;      // host: bit 11 = the OSD bind
localparam [15:0] H_LINE  = 16'h0008;      // host: bit 3 lets the EVT line rise
localparam [15:0] H_SECTOR = 16'h000C;     // host: bit 0 asks for the SECTOR slot
localparam [15:0] H_IDX3  = 16'h001C;      // host: the EVT ring's tail
localparam [15:0] H_MARK1 = 16'h0034;      // host: the download ring's mark, {arm[31], 0, index[15:0]}
localparam [15:0] C_IDX0  = 16'h0090;      // fabric: the mount ring's tail
localparam [15:0] C_RING0 = 16'h00B0;      // fabric: the mount ring's descriptor
localparam [15:0] C_K10   = 16'h00F8;      // fabric: the mount slots' {RD, slot, frame words}
localparam [15:0] C_K11   = 16'h00FC;
localparam [15:0] C_HELLO = 16'h0080;      // fabric: the master's state
localparam [15:0] C_VERSION = 16'h0084;    // fabric: {major, minor} = 1.0
localparam [15:0] C_CAPS  = 16'h0088;      // fabric: bit 2 OSD, bit 3 EVT, bit 4 the torture sink
localparam [15:0] C_STATE = 16'h008C;      // fabric: hello count, fault and previous-session-mid bits
localparam [15:0] C_IDX1  = 16'h0094;      // fabric: the download ring's tail
localparam [15:0] C_IDX2  = 16'h0098;      // fabric: the OSD ring's tail
localparam [15:0] C_IDX3  = 16'h009C;      // fabric: the EVT ring's head
localparam [15:0] C_IDX4  = 16'h00A0;      // fabric: the torture ring's tail
localparam [15:0] C_BIND  = 16'h00A4;      // fabric: bit 11 = the OSD bind is in effect
localparam [15:0] C_RING1 = 16'h00B4;      // fabric: the download ring's descriptor
localparam [15:0] C_RING2 = 16'h00B8;      // fabric: the OSD ring's descriptor
localparam [15:0] C_RING3 = 16'h00BC;      // fabric: the EVT ring's descriptor
localparam [15:0] C_RING4 = 16'h00C0;      // fabric: the torture ring's descriptor
localparam [15:0] C_K0    = 16'h00D0;      // fabric: the SECTOR block's K0, K1, K2, K4, K5
localparam [15:0] C_K1    = 16'h00D4;
localparam [15:0] C_K2    = 16'h00D8;
localparam [15:0] C_K4    = 16'h00E0;
localparam [15:0] C_K5    = 16'h00E4;
localparam [15:0] C_K8    = 16'h00F0;      // fabric: OSDBUF's K0, {rows, row bytes}
localparam [15:0] C_K9    = 16'h00F4;      // fabric: the download player's {sel mask, gap}
localparam [15:0] EVT_BASE = 16'h4000;     // fabric: EVT word k at EVT_BASE + ((head + k) & 2047) * 4
localparam [15:0] EVT_WORDS = 16'd2048;

localparam [31:0] C_HELLO_ARMED = 32'h4F43_0000;
localparam [31:0] C_HELLO_WAIT  = 32'h5754_0000;
localparam [31:0] C_HELLO_STOP  = 32'h5154_0000;
localparam [15:0] HELLO_MAGIC   = 16'h4F43;
localparam [31:0] C_VERSION_1_0 = 32'h0001_0000;
localparam [31:0] C_RING2_OSD   = {5'd3, 5'd0, 4'd11, 4'd0, 14'h0800};    // OSD, no line, 2048 words, 0x2000
localparam [31:0] C_RING3_EVT   = {5'd4, 5'd4, 4'd11, 4'd0, 14'h1000};    // EVT, line 4, 2048 words, 0x4000
/* verilator lint_off UNUSEDPARAM */   // a DIAG build's words or a SECTOR build's, never both
localparam [31:0] C_RING4_TS    = {5'd31, 5'd0, 4'd13, 4'd0, 14'h1C00};   // DIAG, no line, 8192 words, 0x7000
localparam [31:0] C_RING4_SEC   = {5'd2, 5'd0, 4'd13, 4'd0, 14'h1C00};    // SECTOR, no line, 8192 words, 0x7000
// LANE, line 5, 2^FIO_RING words at 0x1000 (8,192 at 0x7000); the select mask (the FIO_SELS parameter, the one
// hps_ocram_rx plays by) and the gap.
localparam [13:0] FIO_OFF       = (FIO_RING == 32'd13) ? 14'h1C00 : 14'h0400;
localparam [31:0] C_RING1_FIO   = {5'd5, 5'd5, FIO_RING[3:0], 4'd0, FIO_OFF};
localparam [31:0] C_K9_FIO      = {12'd0, FIO_SELS, FIO_GAP[15:0]};
localparam [31:0] C_K0_SEC      = {24'd0, SEC_GAP[7:0]};                 // {FIFO entries 0, 0, strobe gap}
localparam [31:0] C_K4_SEC      = {8'h01, 4'd0, SEC_SLOT[3:0], 3'd0, SEC_FRAME[12:0]};   // {RD, slot, frame words}
localparam [31:0] REQ_HEADER    = {8'h21, 8'd0, 16'd1};                  // kind REQUEST, arg 0, len 1
// the mount ring: SECTOR, no line, 1,024 words at 0x6000
localparam [31:0] C_RING0_MNT   = {5'd2, 5'd0, 4'd10, 4'd0, 14'h1800};
localparam [31:0] C_K8_OSD      = {OCRAM_OSD_ROWS[15:0], 16'd256};
localparam [31:0] MARK_HEADER   = {8'h23, 8'h03, 16'd40};                // kind MARK, arg 3, len 40
localparam [31:0] VERDICT_HEADER= {8'h25, 8'd4, 16'd8};                  // kind VERDICT, arg 4, len 8
/* verilator lint_on UNUSEDPARAM */
localparam integer POLL = `MISTER_OCRAM_POLL;

localparam OSD_EN  = OCRAM_OSD_EN;
localparam TS_EN   = OCRAM_TS_EN;
localparam SEC_EN  = OCRAM_SEC_EN;
localparam FIO_EN  = OCRAM_FIO_EN;
localparam MNT0_EN = OCRAM_MNT0_EN;
localparam MNT1_EN = OCRAM_MNT1_EN;
localparam MNT_EN = MNT0_EN != 0 || MNT1_EN != 0;
localparam [2:0] SEC_MASK = {MNT1_EN != 0, MNT0_EN != 0, 1'b1};    // the H_SECTOR bits this build serves
// a mount slot's K word {RD, slot, frame words}, 0 if undeclared
/* verilator lint_off UNUSEDPARAM */   // a mount build's
localparam [31:0] C_K10_MNT     = MNT0_EN ? {8'h01, 4'd0, MNT0_SLOT[3:0], 3'd0, MNT_FRAME[12:0]} : 32'd0;
localparam [31:0] C_K11_MNT     = MNT1_EN ? {8'h01, 4'd0, MNT1_SLOT[3:0], 3'd0, MNT_FRAME[12:0]} : 32'd0;
/* verilator lint_on UNUSEDPARAM */
localparam RX_EN = OSD_EN != 0 || SEC_EN != 0 || FIO_EN != 0;     // hps_ocram_rx takes port 0

// A static assertion: an address that could leave the mailbox stops elaboration.
generate
	if (EVT_BASE + EVT_WORDS * 4 > 16'hF000) begin : g_evt_span
		ocram_evt_span_above_0xF000 guard();
	end
	if (H_HELLO >= 16'h0100 || H_BIND >= 16'h0100 || H_LINE >= 16'h0100 || H_SECTOR >= 16'h0100 ||
	    H_IDX3 >= 16'h0100 || H_MARK1 >= 16'h0100 || C_IDX1 >= 16'h0100 || C_RING1 >= 16'h0100 ||
	    C_K9 >= 16'h0100 || C_K0 >= 16'h0100 || C_K1 >= 16'h0100 || C_K2 >= 16'h0100 ||
	    C_K4 >= 16'h0100 || C_K5 >= 16'h0100 ||
	    C_HELLO >= 16'h0100 || C_VERSION >= 16'h0100 || C_CAPS >= 16'h0100 ||
	    C_STATE >= 16'h0100 || C_IDX2 >= 16'h0100 || C_IDX3 >= 16'h0100 || C_IDX4 >= 16'h0100 ||
	    C_BIND >= 16'h0100 || C_RING2 >= 16'h0100 || C_RING3 >= 16'h0100 || C_RING4 >= 16'h0100 ||
	    C_K8 >= 16'h0100 || C_IDX0 >= 16'h0100 || C_RING0 >= 16'h0100 || C_K10 >= 16'h0100 ||
	    C_K11 >= 16'h0100) begin : g_ctrl_span
		ocram_control_constant_above_0xFF guard();
	end
	// the mount slots: hps_io slots 0-15, none the disc slot's or the other's; a block of 1 to 1,020 words
	if ((MNT0_EN != 0 && (MNT0_SLOT > 15 || MNT0_SLOT == SEC_SLOT)) ||
	    (MNT1_EN != 0 && (MNT1_SLOT > 15 || MNT1_SLOT == SEC_SLOT)) ||
	    (MNT0_EN != 0 && MNT1_EN != 0 && MNT0_SLOT == MNT1_SLOT) ||
	    (MNT_EN && (MNT_FRAME == 0 || MNT_FRAME > 1020))) begin : g_mnt_geometry
		ocram_mount_unservable_geometry guard();
	end
endgenerate

// The SECTOR tier and the DIAG sink share ring 4's span: a build of both stops.
`ifdef MISTER_OCRAM_DIAG
`ifdef MISTER_OCRAM_SECTOR_SLOT
ocram_sector_excludes_MISTER_OCRAM_DIAG diag_guard();
`endif
`endif
// The capture lane's frames are the download tier's.
`ifdef MISTER_OCRAM_LANE
`ifndef MISTER_OCRAM_FIO
ocram_lane_needs_MISTER_OCRAM_FIO lane_fio_guard();
`endif
`endif

assign present = 1'b1;

////////////////////////////  clock-domain entry  ///////////////////////////

// Every GPO bit and hps_rst_n enters only through an lw_arst cell. idv_q is high in the ID view (bit 31 low),
// rn_q is high while RUN is 0 (a 0 of any length sets the chain asynchronously), hrst_q is high while the HPS
// reset is seen. With DIAG, nsn_q is high while the non-secure knob (bit 27) is 0 and fln_q while flood (bit 26)
// is 0; both are sampled at the latch set only.
wire idv_q, rn_q, hrst_q;
lw_arst idv_c (.clk(clk), .rst_n(gp_out[31]), .q(idv_q));
lw_arst rn_c  (.clk(clk), .rst_n(gp_out[28]), .q(rn_q));
lw_arst hrst_c(.clk(clk), .rst_n(hps_rst_n), .q(hrst_q));

reg rn_d = 1'b1;                    // lw_arst's q is high from configuration
always @(posedge clk) rn_d <= rn_q;
wire run_rise = rn_d & ~rn_q;       // a fall of lw_arst's q: RUN's 0 store was seen
wire run_set  = run_rise & ~idv_q & ~hrst_q;

reg run = 1'b0;
always @(posedge clk) begin
	if (rn_q | idv_q | hrst_q) run <= 1'b0;   // the clear wins
	else if (run_rise) run <= 1'b1;
end

reg hrst_d = 1'b1;
always @(posedge clk) hrst_d <= hrst_q;
wire hrst_rise = hrst_q & ~hrst_d;
wire hrst_fall = ~hrst_q & hrst_d;

reg run_d = 1'b0;
always @(posedge clk) run_d <= run;
wire run_fall = run_d & ~run;

`ifdef MISTER_OCRAM_DIAG
wire nsn_q, fln_q;
lw_arst ns_c(.clk(clk), .rst_n(gp_out[27]), .q(nsn_q));
lw_arst fl_c(.clk(clk), .rst_n(gp_out[26]), .q(fln_q));

reg run_ns = 1'b0, run_flood = 1'b0;   // sampled at the latch set, fixed for the run
always @(posedge clk) if (run_set) begin
	run_ns    <= ~nsn_q;
	run_flood <= ~fln_q;
end
wire ns = run_ns;
`else
wire ns = 1'b0;
`endif

/////////////////////////////////  the bus  /////////////////////////////////

// ocram_axi.sv is the engine; ocram_arb.sv (below, after the requesters) picks its commands.
wire        engine_idle, t_done, t_ok;
wire        t_go, t_we, t_counted;
wire [15:0] t_off;
wire [31:0] t_wdata;
/* verilator lint_off UNUSEDSIGNAL */   // the DIAG counters' inputs
wire        counted_q, drop_q;
/* verilator lint_on UNUSEDSIGNAL */

ocram_axi axi
(
	.clk(clk), .hrst_q(hrst_q), .hrst_fall(hrst_fall), .ns(ns),
	.t_go(t_go), .t_we(t_we), .t_off(t_off), .t_wdata(t_wdata), .t_counted(t_counted),
	.engine_idle(engine_idle), .t_done(t_done), .t_ok(t_ok), .t_counted_q(counted_q), .t_drop(drop_q),
	.awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awburst(awburst), .awlock(awlock),
	.awcache(awcache), .awprot(awprot), .awuser(awuser), .awvalid(awvalid), .awready(awready),
	.wid(wid), .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
	.bresp(bresp), .bvalid(bvalid), .bready(bready),
	.arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst), .arlock(arlock),
	.arcache(arcache), .arprot(arprot), .aruser(aruser), .arvalid(arvalid), .arready(arready),
	.rresp(rresp), .rvalid(rvalid), .rready(rready)
);

///////////////////////////////  the run  ///////////////////////////////////

// A run's phase. P_REST is the hello's wait for every consumer's clk_sys half; P_HELLO covers the hello's
// stores, which run in order and in one piece; P_SESSION is a session with the hello echoed.
localparam [2:0] P_OFF = 3'd0, P_START = 3'd1, P_ARMED = 3'd2, P_WAIT = 3'd3,
                 P_SESSION = 3'd4, P_HELLO = 3'd5, P_REST = 3'd6;

localparam [2:0] R_NONE = 3'd0, R_HELLO = 3'd1, R_LINE = 3'd2, R_IDX = 3'd3, R_BIND = 3'd4, R_SECT = 3'd5,
                 R_MARK = 3'd6;
localparam [2:0] W_NONE = 3'd0, W_HELLO = 3'd1, W_ONE = 3'd2, W_TERM = 3'd5, W_FAULT = 3'd6, W_BIND = 3'd7;

reg  [2:0]  phase = P_OFF;
reg  [7:0]  hello_cnt = 8'd0;          // hellos taken since configuration, no reset; counts each echo
reg  [31:0] echo_q = 32'd0;            // the H_HELLO value this session echoed
reg         line_en_q = 1'b0;          // H_LINE bit 3 at its last read
reg         stopped_q = 1'b0;          // the EVT ring stopped on a fault this session
reg         prev_mid_q = 1'b0;         // the EVT record of the session before this one was part-written
reg         osd_bind_q = 1'b0;         // H_BIND bit 11 read: the OSD bind asked for this session
reg  [3:0]  bind_q = 4'd0;             // the C_BIND bits {11, 2:0} last stored
`ifdef MISTER_OCRAM_EVT
reg         evt_bind_q = 1'b0;         // H_BIND bit 8 read: the events bind asked for this session
reg         evt_bound_q = 1'b0;        // C_BIND bit 8 as last stored
`endif
reg  [2:0]  sec_ask_q = 3'd0;          // H_SECTOR bits 2:0 read this session (those this build serves)
reg         bind_pend = 1'b0;          // a C_BIND store is owed
reg         osd_stopped_q = 1'b0;      // the OSD ring stopped on a fault this session
reg         ts_stopped_q = 1'b0;       // the torture ring stopped on a fault this session
reg         prev_mid_osd_q = 1'b0;     // the OSD record of the previous session was part-consumed
reg         prev_mid_ts_q = 1'b0;      // the torture record of the previous session was part-read
reg         fio_stopped_q = 1'b0;      // the download ring stopped on a fault this session
reg         prev_mid_fio_q = 1'b0;     // the download record of the previous session was part-consumed
reg         mnt_stopped_q = 1'b0;      // the mount ring stopped on a fault this session
reg         prev_mid_mnt_q = 1'b0;     // the mount record of the previous session was part-consumed
reg         mark_arm_q = 1'b0;         // H_MARK[1] bit 31 at its last read this session
reg  [15:0] mark_idx_q = 16'd0;        // H_MARK[1][15:0] at its last read
`ifdef MISTER_OCRAM_DIAG
reg  [15:0] deepest_q = 16'd0;         // deepest EVT waiting this session, for MARK word 0
reg         verdict_pend = 1'b0;       // a VERDICT follows the period's MARK
`endif
reg  [2:0]  poll_step = 3'd0;          // session poll: 1 H_BIND, 5 H_SECTOR, 6 H_MARK[1], 2 H_LINE, 3 H_IDX, then
                                       // 4 H_IDX again while the EVT line is high, 7 H_MARK[1] while the mark line is
reg         poll_pend = 1'b0;          // a poll period has passed
reg         boot_pend = 1'b0;          // the run's first H_HELLO read is not taken yet
reg         boot_first = 1'b1;         // it is tried at once; a failed one waits for the next poll
reg         term_pend = 1'b0;          // the terminal store of a stopped run is owed
reg         sto_pend = 1'b0;           // one C_HELLO state store is owed
reg  [31:0] sto_data = 32'd0;
reg         fault_pend = 1'b0;         // C_STATE's fault bits are owed "at once"
reg         hj_act = 1'b0;             // the hello's stores
reg  [4:0]  hj_k = 5'd0;
reg  [31:0] hj_state = 32'd0;          // C_STATE as stored at this hello
reg  [2:0]  rd_act = R_NONE;           // the read in flight
reg  [2:0]  wr_act = W_NONE;           // the write in flight (the record writer keeps its own)
localparam [POLL-1:0] POLL_T0 = {POLL{1'b0}};
localparam [POLL-1:0] POLL_ONE = 1;
reg  [POLL-1:0] poll_t = {POLL{1'b0}}; // free-running; its period is 2^POLL cycles
wire poll_tick = (poll_t == POLL_T0);

always @(posedge clk) poll_t <= run_set ? POLL_T0 : poll_t + POLL_ONE;   // a run's polls count from its start

wire run_ended = hrst_rise | run_fall;

// The EVT ring's writer's state, for the session (ocram_evt_wr below)
wire [15:0] w_head, w_tail;            // the ring's head, last published, and its tail, last read
/* verilator lint_off UNUSEDSIGNAL */   // the DIAG counters', the capture lane's
wire [15:0] w_head_nx, w_free;         // the open record's head store value; 2,048 - (head - tail)
/* verilator lint_on UNUSEDSIGNAL */
wire        w_act;                     // a record is open: it is part-written until its head store
/* verilator lint_off UNUSEDSIGNAL */   // the record sources' (and the bench's rec_k)
wire [10:0] rec_k;                     // its next store: 0 header, 1..len payload, len + 1 head
wire        w_first;                   // rec_k is 1
wire  [2:0] w_src;
wire        w_wfly;                    // a header or payload store of it is in flight
/* verilator lint_on UNUSEDSIGNAL */
wire        w_hfly;                    // its head store is in flight

// The consumers' session returns and the hello's wait for their rest. c1_* are the torture sink's outputs
// (driven below with DIAG, constant when unbuilt). The hello's first store runs only when every built
// consumer's clk_sys half reports rest; while it waits the arbiter starts only the terminal store.
wire c1_rest, c1_fault, c1_mid;
wire s_live_w = (phase == P_SESSION);
wire consumer_rest = (!RX_EN || c0_rest) && (!TS_EN || c1_rest);
wire hello_start = (phase == P_REST) && consumer_rest && !run_ended;
assign s_live   = s_live_w;
assign s_tick   = poll_tick;
assign s_hello  = hello_start;
assign osd_bind = osd_bind_q;
assign sec_ask  = sec_ask_q;
assign evt_stop = stopped_q;

// C_STATE: [7:0] hellos taken, [12:8] ring r stopped on a fault this session, [20:16] ring r ended the previous
// session mid-record; every other bit 0. The ring numbers are fixed: EVT r = 3 (bits 11, 19), OSD r = 2 (bits
// 10, 18), the torture ring r = 4 (bits 12, 20), the download ring r = 1 (bits 9, 17), the mount ring r = 0 (bits
// 8, 16). cnt is the hello count the word reports: the incremented one at a hello, the current one at a fault.
function [31:0] state_word(input [7:0] cnt, input fe, input fo, input ft, input ff, input fm, input me, input mo,
                           input mt, input mf, input mm);
	begin
		state_word = {24'd0, cnt};
		if (me) state_word[19] = 1'b1;
		if (mo) state_word[18] = 1'b1;
		if (mt) state_word[20] = 1'b1;
		if (mf) state_word[17] = 1'b1;
		if (mm) state_word[16] = 1'b1;
		if (fe) state_word[11] = 1'b1;
		if (fo) state_word[10] = 1'b1;
		if (ft) state_word[12] = 1'b1;
		if (ff) state_word[9] = 1'b1;
		if (fm) state_word[8] = 1'b1;
	end
endfunction

// The download ring's tail has reached a mark index: it is at or past it, modulo 2^16, within half the index space.
function mark_reached(input [15:0] tail, input [15:0] idx);
	mark_reached = ((tail - idx) & 16'h8000) == 16'd0;
endfunction

// The hello's stores, in their fixed order, the last C_HELLO (HJ_LAST). hj_has says whether a step is stored in this
// build; a step that is
// not is passed without a transaction. Args only: Quartus drops a module-scope read in a function.
localparam [4:0] HJ_LAST = 5'd23;
function hj_has(input [4:0] k);
	begin
		hj_has = (k <= 5'd23);
`ifndef MISTER_OCRAM_OSD
		if (k == 5'd3 || k == 5'd7 || k == 5'd10) hj_has = 1'b0;
`endif
`ifndef MISTER_OCRAM_DIAG
`ifndef MISTER_OCRAM_SECTOR_SLOT
		if (k == 5'd5 || k == 5'd9) hj_has = 1'b0;
`endif
`endif
`ifndef MISTER_OCRAM_SECTOR_SLOT
		if (k >= 5'd11 && k <= 5'd15) hj_has = 1'b0;
`endif
`ifndef MISTER_OCRAM_FIO
		if (k >= 5'd16 && k <= 5'd18) hj_has = 1'b0;
`endif
`ifndef MISTER_OCRAM_MOUNT0_SLOT
`ifndef MISTER_OCRAM_MOUNT1_SLOT
		if (k >= 5'd19 && k <= 5'd22) hj_has = 1'b0;
`endif
`endif
	end
endfunction

// The step after k. Without the download tier its three steps take no cycle, nor without the mount slots their four:
// C_HELLO follows the last step built at once.
function [4:0] hj_next(input [4:0] k);
	begin
		hj_next = k + 5'd1;
`ifndef MISTER_OCRAM_FIO
		if (k == 5'd15) hj_next = 5'd19;
`endif
`ifndef MISTER_OCRAM_MOUNT0_SLOT
`ifndef MISTER_OCRAM_MOUNT1_SLOT
		if (hj_next == 5'd19) hj_next = 5'd23;
`endif
`endif
	end
endfunction

function [15:0] hj_off(input [4:0] k);
	begin
		case (k)
		5'd0: hj_off = C_VERSION;
		5'd1: hj_off = C_CAPS;
		5'd2: hj_off = C_STATE;
		5'd3: hj_off = C_IDX2;
		5'd4: hj_off = C_IDX3;
		5'd5: hj_off = C_IDX4;
		5'd6: hj_off = C_BIND;
		5'd7: hj_off = C_RING2;
		5'd8: hj_off = C_RING3;
		5'd9: hj_off = C_RING4;
		5'd10: hj_off = C_K8;
		5'd11: hj_off = C_K0;
		5'd12: hj_off = C_K1;
		5'd13: hj_off = C_K2;
		5'd14: hj_off = C_K4;
		5'd15: hj_off = C_K5;
		5'd16: hj_off = C_IDX1;
		5'd17: hj_off = C_RING1;
		5'd18: hj_off = C_K9;
		5'd19: hj_off = C_IDX0;
		5'd20: hj_off = C_RING0;
		5'd21: hj_off = C_K10;
		5'd22: hj_off = C_K11;
		default: hj_off = C_HELLO;
		endcase
	end
endfunction

function [31:0] hello_data(input [4:0] k, input [31:0] state, input [31:0] echo, input wide);
	begin
		case (k)
		5'd0: hello_data = C_VERSION_1_0;
		5'd1: hello_data = OCRAM_C_CAPS;
		5'd2: hello_data = state;
		5'd3: hello_data = 32'd0;
		5'd4: hello_data = 32'd0;
		5'd5: hello_data = 32'd0;
		5'd6: hello_data = 32'd0;
		5'd7: hello_data = C_RING2_OSD;
		5'd8: hello_data = C_RING3_EVT;
`ifdef MISTER_OCRAM_SECTOR_SLOT
		5'd9: hello_data = C_RING4_SEC;
`else
		5'd9: hello_data = C_RING4_TS;
`endif
		5'd10: hello_data = C_K8_OSD;
		5'd11: hello_data = C_K0_SEC;
		5'd12: hello_data = {21'd0, SEC_BLKSZ, 7'd0, wide};         // {BLKSZ[10:8], wide[0]}
		5'd13: hello_data = 32'd1;                                   // one descriptor
		5'd14: hello_data = C_K4_SEC;
		5'd15: hello_data = 32'd32;                                  // LBA bits
		5'd16: hello_data = 32'd0;
		5'd17: hello_data = C_RING1_FIO;
		5'd18: hello_data = C_K9_FIO;
		5'd19: hello_data = 32'd0;
		5'd20: hello_data = C_RING0_MNT;
		5'd21: hello_data = C_K10_MNT;
		5'd22: hello_data = C_K11_MNT;
		default: hello_data = echo;
		endcase
	end
endfunction

// The hello the read found: a valid H_HELLO in ARMED moves to the consumers' rest wait. diag_clr clears the
// DIAG counters when the hello's first store is about to run, at the same edge the bench's own clear sees.
wire hello_hit = t_done && (rd_act == R_HELLO) && t_ok && (phase == P_ARMED) &&
                 (rdata[31:16] == HELLO_MAGIC) && (rdata[15:0] != 16'd0);
/* verilator lint_off UNUSEDSIGNAL */   // the DIAG counters' clear
wire diag_clr = hello_start;
/* verilator lint_on UNUSEDSIGNAL */

// The read the poll (or the boot, or the line) wants next, after the consumers' declarations below: a step-4
// read is the EVT line's back-to-back tail re-read and a step-7 read the mark line's H_MARK[1] re-read; each
// yields to a requesting consumer (line_yield_q). Steps 1-3, 5, 6 and the periodic H_HELLO poll are unchanged.
reg  poll_sel;
reg  [2:0] poll_rd_kind;

//////////////////////////////  the consumers  //////////////////////////////

// Port 0 is hps_ocram_rx's walker (ocram_ring_rd); port 1 is the torture sink, built below with DIAG.
// An unbuilt port's request is the constant 0. A consumer asks for one transaction at a time and holds c_req
// until c_grant; c_done is the R or B handshake's cycle, with c_ok and c_rdata. Round robin (ocram_arb): the last
// granted goes last, and only a session grants.
wire c0_req_v = RX_EN && c0_req;
wire  [1:0] a_cdone;                   // the consumer ports' completions
wire [31:0] a_crdata;
`ifdef MISTER_OCRAM_DIAG
wire        c1_req, c1_we;
wire [15:0] c1_off;
wire [31:0] c1_wdata;
wire        c1_grant, c1_done, c1_ok;
wire [31:0] c1_rdata;
wire [31:0] ts_records, ts_words, ts_errors, ts_first_g, ts_first_got, ts_sum;
wire [15:0] ts_skipped, ts_deepest;
wire c1_req_v = c1_req;
`else
wire        c1_we = 1'b0;
wire [15:0] c1_off = 16'd0;
wire [31:0] c1_wdata = 32'd0;
wire c1_req_v = 1'b0;
assign c1_rest = 1'b1;                 // the unbuilt port is always at rest
assign c1_fault = 1'b0;
assign c1_mid = 1'b0;
`endif

wire consumer_req = c0_req_v | c1_req_v;   // a consumer is waiting: the line's re-read gives it a turn
reg  line_yield_q = 1'b0;                  // a step-4 re-read ran and the consumers have not had their turn

always @(*) begin
	poll_sel = 1'b0; poll_rd_kind = R_NONE;
	if (phase == P_START) begin
		if (boot_pend && (boot_first || poll_pend)) begin
			poll_sel = 1'b1; poll_rd_kind = R_HELLO;
		end
	end
	else if (phase == P_ARMED || phase == P_WAIT) begin
		if (poll_pend) begin poll_sel = 1'b1; poll_rd_kind = R_HELLO; end
	end
	else if (phase == P_SESSION) begin
		if (poll_pend) begin
			poll_sel = 1'b1; poll_rd_kind = R_HELLO;
		end
		else if (poll_step == 3'd1) begin
			poll_sel = 1'b1; poll_rd_kind = R_BIND;
		end
		else if (poll_step == 3'd5) begin
			poll_sel = 1'b1; poll_rd_kind = R_SECT;
		end
		else if (poll_step == 3'd6) begin
			poll_sel = 1'b1; poll_rd_kind = R_MARK;
		end
		else if (poll_step == 3'd2) begin
			poll_sel = 1'b1; poll_rd_kind = R_LINE;
		end
		else if (poll_step == 3'd3) begin
			poll_sel = 1'b1; poll_rd_kind = R_IDX;
		end
		else if (poll_step == 3'd4 && !(line_yield_q && consumer_req)) begin
			poll_sel = 1'b1; poll_rd_kind = R_IDX;      // at most one re-read between two consumer grants
		end
		else if (poll_step == 3'd7 && !(line_yield_q && consumer_req)) begin
			poll_sel = 1'b1; poll_rd_kind = R_MARK;     // the mark line's, by the same rule
		end
	end
end

// the poll's read, a table of the kind
reg  [15:0] poll_off;
always @(*) begin
	case (poll_rd_kind)
	R_LINE:  poll_off = H_LINE;
	R_IDX:   poll_off = H_IDX3;
	R_BIND:  poll_off = H_BIND;
	R_SECT:  poll_off = H_SECTOR;
	R_MARK:  poll_off = H_MARK1;
	default: poll_off = H_HELLO;
	endcase
end

// the arbiter's ports, in its order: the terminal store, the fault's C_STATE store, the poll's or the line's reads
// when due, the hello's stores, a state store, a C_BIND store, the EVT ring's writer, then the two consumers
localparam integer NP = 7;
localparam [3:0] J_NONE = 4'd0, J_TERM = 4'd1, J_FAULT = 4'd2, J_POLL = 4'd3, J_HELLO = 4'd4,
                 J_STO = 4'd5, J_BIND = 4'd6, J_REC = 4'd7, J_C0 = 4'd10, J_C1 = 4'd11;
wire [NP+1:0] gnt;
reg   [3:0] t_job;                     // the job the arbiter chose this cycle, J_NONE if none
always @(*) begin
	t_job = J_NONE;
	if (gnt[0]) t_job = J_TERM;
	if (gnt[1]) t_job = J_FAULT;
	if (gnt[2]) t_job = J_POLL;
	if (gnt[3]) t_job = J_HELLO;
	if (gnt[4]) t_job = J_STO;
	if (gnt[5]) t_job = J_BIND;
	if (gnt[6]) t_job = J_REC;
	if (gnt[7]) t_job = J_C0;
	if (gnt[8]) t_job = J_C1;
end
wire c0_go = (t_job == J_C0);
/* verilator lint_off UNUSEDSIGNAL */   // port 1 is built with DIAG
wire c1_go = (t_job == J_C1);
/* verilator lint_on UNUSEDSIGNAL */

assign c0_grant = c0_go;
assign c0_done  = a_cdone[0];
assign c0_ok    = t_ok;
assign c0_rdata = a_crdata;
`ifdef MISTER_OCRAM_DIAG
assign c1_grant = c1_go;
assign c1_done  = a_cdone[1];
assign c1_ok    = t_ok;
assign c1_rdata = a_crdata;

hps_ocram_tsink tsink
(
	.clk       (clk),
	.s_hello   (hello_start),
	.s_live    (s_live_w),
	.s_tick    (poll_tick),
	.c_rest    (c1_rest),
	.c_fault   (c1_fault),
	.c_mid     (c1_mid),
	.c_req     (c1_req),
	.c_we      (c1_we),
	.c_off     (c1_off),
	.c_wdata   (c1_wdata),
	.c_grant   (c1_grant),
	.c_done    (c1_done),
	.c_ok      (c1_ok),
	.c_rdata   (c1_rdata),
	.v_records (ts_records),
	.v_words   (ts_words),
	.v_errors  (ts_errors),
	.v_first_g (ts_first_g),
	.v_first_got(ts_first_got),
	.v_sum     (ts_sum),
	.v_skipped (ts_skipped),
	.v_deepest (ts_deepest)
);
`else
/* verilator lint_off UNUSEDSIGNAL */   // port 1 is built with DIAG
wire a_c1done = a_cdone[1];
/* verilator lint_on UNUSEDSIGNAL */
`endif

/////////////////////////////  the EVT ring  ////////////////////////////////

// ocram_evt_wr.sv writes the ring's records. Each record source here asks with post and its header, offers its
// payload words (wv, wd) and sees its start, wtake and pub, in this fixed order: BIND, the disc slot's REQUEST, a
// mount slot's REQUEST or WRITE, the capture lane's REPLY, the events tier's records, DIAG's MARK and VERDICT. A
// source with no post (unbuilt) is the constant 0.
localparam integer NS = 7;
/* verilator lint_off UNUSEDPARAM */   // each build names its own sources
localparam [2:0] S_BIND = 3'd0, S_DISC = 3'd1, S_MNT = 3'd2, S_REP = 3'd3, S_EV = 3'd4, S_MARK = 3'd5, S_VD = 3'd6;
/* verilator lint_on UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */   // each build reads its own sources'
wire [NS-1:0]   w_start, w_wtake, w_pub;
/* verilator lint_on UNUSEDSIGNAL */
// the first payload word of a mount slot's record (its LBA) or the events tier's, as it was at the record's start
/* verilator lint_off UNUSEDSIGNAL */   // a mount build's, an events build's
reg  [31:0] rec_lba_q = 32'd0;
/* verilator lint_on UNUSEDSIGNAL */

`ifdef MISTER_OCRAM_SECTOR_SLOT
// The SECTOR tier's sources. BIND {0x27, C_BIND[7:0], 0}, owed after each C_BIND store (behind a room check of 1
// word); the disc slot's REQUEST {0x21, 0, 1} then the LBA (2); a mount slot's REQUEST {0x21, {blk_cnt, k}, 1} then
// the LBA, or its WRITE {0x26, {blk_cnt, k}, 1 + n} then the LBA and the n words as hps_ocram_rx hands them over
// (n + 2). A REQUEST or WRITE starts only while its slot's C_BIND bit is stored 1 and no C_BIND store or BIND is
// owed; a C_BIND store waits for a record in flight and for an owed BIND. With the EVT ring stopped no BIND is owed.
reg         brec_pend = 1'b0;          // the BIND record after a C_BIND store is owed
reg         rec_mw_q = 1'b0;           // the mount record open is a WRITE
/* verilator lint_off UNUSEDSIGNAL */   // the REPLY's and the events tier's
wire        brec_owed = brec_pend;
/* verilator lint_on UNUSEDSIGNAL */
wire        b_post = brec_pend;
wire [31:0] b_hdr  = {8'h27, 5'd0, bind_q[2:0], 16'd0};
wire        d_post = rq_post && c0_sbind[0] && bind_q[0] && !bind_pend && !brec_pend;
wire [31:0] d_hdr  = REQ_HEADER;
wire [31:0] d_wd   = rq_lba;           // the disc REQUEST's LBA is hps_ocram_rx's request register
wire        m_bound = (m_arg[1:0] == 2'd2) ? (c0_sbind[2] && bind_q[2]) : (c0_sbind[1] && bind_q[1]);
wire        n_post = MNT_EN && m_post && m_bound && !bind_pend && !brec_pend;
wire [31:0] n_hdr  = {m_write ? 8'h26 : 8'h21, m_arg, 5'd0, m_len};
wire        n_wv   = w_first || m_wv;  // a WRITE word waits for hps_ocram_rx's
wire [31:0] n_wd   = w_first ? rec_lba_q : m_wd;
wire        bind_hold = w_act || (brec_pend && !stopped_q);
assign rq_start = w_start[S_DISC];
assign m_start  = w_start[S_MNT];
assign m_wtake  = w_wtake[S_MNT] && rec_mw_q && !w_first;
assign m_pub    = w_pub[S_MNT] && rec_mw_q;
`else
/* verilator lint_off UNUSEDSIGNAL */   // the REPLY's and the events tier's
wire        brec_owed = 1'b0;
/* verilator lint_on UNUSEDSIGNAL */
wire        b_post = 1'b0, d_post = 1'b0, n_post = 1'b0, n_wv = 1'b0;
wire [31:0] b_hdr = 32'd0, d_hdr = 32'd0, d_wd = 32'd0, n_hdr = 32'd0, n_wd = 32'd0;
wire        bind_hold = 1'b0;
assign rq_start = 1'b0;
assign m_start = 1'b0, m_wtake = 1'b0, m_pub = 1'b0;
`endif

`ifdef MISTER_OCRAM_LANE
// The capture lane's REPLY {0x22, arg, n}: each payload word as the walker hands it over. It opens behind an owed
// BIND, REQUEST and WRITE, with room for its n words and its header.
wire        r_post = p_post && !bind_pend && !brec_owed;
wire [31:0] r_hdr  = {8'h22, p_arg, 5'd0, p_n};
wire        r_wv   = p_wv;
wire [31:0] r_wd   = p_wd;
assign p_start  = w_start[S_REP];
assign p_wtake  = w_wtake[S_REP];
assign e_free   = w_free;
`else
wire        r_post = 1'b0, r_wv = 1'b0;
wire [31:0] r_hdr = 32'd0, r_wd = 32'd0;
assign p_start = 1'b0, p_wtake = 1'b0, e_free = 16'd0;
`endif

`ifdef MISTER_OCRAM_EVT
// The events tier's records. The offer in hand (ev_have, e_d held by the crossing) is {kind, arg, data}: kind 0
// STATUS word arg[7:6] with stflg arg[3:0] and status_req's word in data; 1 INFO, arg the byte (0: no notice); 2
// UPLOAD, arg the index; 3 RUMBLE, arg the pad, data[15:0] its level. One offer is asked for at a time, while C_BIND
// bit 8 is stored 1 and the offer in hand is spent. An offer that needs no record is spent at once; one that opens
// a record waits for the writer behind an owed BIND, REQUEST and WRITE, and for room.
localparam [1:0] EK_STATUS = 2'd0, EK_INFO = 2'd1, EK_UPLOAD = 2'd2, EK_RUMBLE = 2'd3;
reg         ev_have = 1'b0;            // an offer is in e_d and not spent
reg         ev_wait = 1'b0;            // an offer is asked for and not yet in
reg   [1:0] rec_ek_q = EK_STATUS;      // the events record's kind
reg   [2:0] rec_pad_q = 3'd0;          // a RUMBLE's pad
reg   [3:0] st_flag_q = 4'd0;          // the STATUS being written: its stflg
reg   [3:0] st_last_q = 4'd0;          // the stflg of the last STATUS published this session (0 at the hello)
reg  [95:0] rum_last_q = 96'd0;        // each pad's level last published this session (0 at the hello)
/* verilator lint_off UNUSEDSIGNAL */   // the bench's: the protocol monitor forgets an abandoned record's words
reg         ev_abandon = 1'b0;         // one cycle: a STATUS record is abandoned, its head not stored
/* verilator lint_on UNUSEDSIGNAL */
wire  [1:0] ev_k    = e_d[41:40];
wire  [7:0] ev_arg  = e_d[39:32];
wire  [2:0] ev_pad  = (ev_arg[2:0] > 3'd5) ? 3'd5 : ev_arg[2:0];
wire [15:0] ev_rlast = rum_last_q[{ev_pad, 4'd0} +: 16];
wire        ev_bound = evt_bound_q && phase == P_SESSION && !stopped_q;
// the offer in hand opens a record
wire        ev_rec  = (ev_k == EK_STATUS) ? (ev_arg[7:6] == 2'd0 && ev_arg[3:0] != st_last_q) :
                      (ev_k == EK_INFO) ? (ev_arg != 8'd0) : (ev_k == EK_UPLOAD) ? 1'b1 : (e_d[15:0] != ev_rlast);
wire [31:0] ev_hdr  = (ev_k == EK_STATUS) ? {8'h20, 8'd0, 16'd4} : (ev_k == EK_INFO) ? {8'h28, ev_arg, 16'd0} :
                      (ev_k == EK_UPLOAD) ? {8'h29, ev_arg, 16'd0} : {8'h2A, 5'd0, ev_pad, 16'd1};
// its first payload word: a STATUS's word 0, a RUMBLE's level
wire [31:0] ev_w1   = (ev_k == EK_RUMBLE) ? {16'd0, e_d[15:0]} : e_d[31:0];
// a STATUS record's words 1-3: the offer in hand is that word with the record's stflg
wire        st_open = w_act && w_src == S_EV && rec_ek_q == EK_STATUS;
wire        st_wait = st_open && rec_k >= 11'd2 && rec_k <= 11'd4;
wire        st_wok  = ev_have && ev_k == EK_STATUS && {9'd0, ev_arg[7:6]} == rec_k - 11'd1 && ev_arg[3:0] == st_flag_q;
// a STATUS word that is not the open record's next, with its stflg, abandons that record before its head
wire        ev_drop = st_wait && ev_have && !st_wok && !w_wfly && !run_ended;
wire        e_post = ev_bound && ev_have && ev_rec && !bind_pend && !brec_owed;
wire [31:0] e_hdr  = ev_hdr;
wire        e_wv   = !st_wait || st_wok;   // a STATUS word not yet offered lets the consumers in
wire [31:0] e_wd   = st_wait ? e_d[31:0] : rec_lba_q;
assign e_ask = ev_bound && !ev_have && !ev_wait;
`else
wire        ev_drop = 1'b0;
wire        e_post = 1'b0, e_wv = 1'b0;
wire [31:0] e_hdr = 32'd0, e_wd = 32'd0;
/* verilator lint_off UNUSEDSIGNAL */   // no events record starts
wire [31:0] ev_w1 = 32'd0;
/* verilator lint_on UNUSEDSIGNAL */
assign e_ask = 1'b0;
`endif

`ifdef MISTER_OCRAM_DIAG
// DIAG's records. A MARK {0x23, 3, 40}, the counters' snapshot, opens when its period is due (at once with flood)
// with room for its 41 words and, for a period MARK, the VERDICT's 9 beside it: the period's record is followed at
// once by the verdict, and flood posts no verdict. A VERDICT {0x25, 4, 8} needs 9. A MARK's payload word 1 waits
// for the counter snapshot still draining older transactions, and the consumers wait with it.
localparam integer MARKW = `MISTER_OCRAM_MARK;
localparam [MARKW-1:0] MARK_T0 = {MARKW{1'b0}};
localparam [MARKW-1:0] MARK_ONE = 1;
reg         mark_due_q = 1'b0;
reg  [MARKW-1:0] mark_t = {MARKW{1'b0}};
wire [31:0] mark_word;
wire        snap_ready;
// the VERDICT's eight payload words, one instant at the record's start
reg  [31:0] vd0 = 32'd0, vd1 = 32'd0, vd2 = 32'd0, vd3 = 32'd0;
reg  [31:0] vd4 = 32'd0, vd5 = 32'd0, vd6 = 32'd0, vd7 = 32'd0;
reg  [31:0] vd_word;
always @(*) begin
	case (rec_k)
	11'd1: vd_word = vd0;
	11'd2: vd_word = vd1;
	11'd3: vd_word = vd2;
	11'd4: vd_word = vd3;
	11'd5: vd_word = vd4;
	11'd6: vd_word = vd5;
	11'd7: vd_word = vd6;
	11'd8: vd_word = vd7;
	default: vd_word = 32'd0;
	endcase
end
wire        mark_start = w_start[S_MARK];
wire        k_post = (run_flood || mark_due_q) && (run_flood || w_free >= 16'd50);
wire [31:0] k_hdr  = MARK_HEADER;
wire        k_wv   = !(w_first && !snap_ready);
wire        v_post = verdict_pend;
wire [31:0] v_hdr  = VERDICT_HEADER;
wire        w_hold = w_act && w_src == S_MARK && w_first && !snap_ready;
`else
wire        k_post = 1'b0, k_wv = 1'b0, v_post = 1'b0;
wire [31:0] k_hdr = 32'd0, mark_word = 32'd0, v_hdr = 32'd0, vd_word = 32'd0;
wire        w_hold = 1'b0;
`endif

// The session's abandonments of the record open: an HPS reset, a stop or a run's start; the session's end; a host
// tail past the ring's size; and the events tier's out-of-turn STATUS word. A head store in flight still lands.
wire        idx_fault = !stopped_q && (((w_head - rdata[15:0]) & 16'hFFFF) > EVT_WORDS);
wire        idx_done  = t_done && (rd_act == R_IDX) && !run_ended && t_ok;
wire        sess_end  = t_done && (rd_act == R_HELLO) && !run_ended && t_ok && (phase == P_SESSION) &&
                        (rdata != echo_q);
wire        w_drop = hrst_rise | run_fall | run_set | sess_end | (idx_done && idx_fault) | ev_drop;

wire        w_req, w_counted;
wire [15:0] w_off;
wire [31:0] w_wdata;
ocram_evt_wr #(.N(NS)) evt_wr
(
	.clk(clk), .clr(hello_start), .live((phase == P_SESSION) && !stopped_q), .drop(w_drop),
	.kill(hrst_rise | (run_set & ~run_fall)), .ended(run_ended), .t_set(idx_done && !idx_fault), .t_d(rdata[15:0]),
	.head(w_head), .tail(w_tail), .head_nx(w_head_nx), .free(w_free), .act(w_act), .k(rec_k), .first(w_first),
	.src(w_src), .wfly(w_wfly), .hfly(w_hfly),
	.post({v_post, k_post, e_post, r_post, n_post, d_post, b_post}),
	.hdr({v_hdr, k_hdr, e_hdr, r_hdr, n_hdr, d_hdr, b_hdr}),
	.wv({1'b1, k_wv, e_wv, r_wv, n_wv, 1'b1, 1'b1}),
	.wd({vd_word, mark_word, e_wd, r_wd, n_wd, d_wd, 32'd0}),
	.start(w_start), .wtake(w_wtake), .pub(w_pub),
	.req(w_req), .off(w_off), .wdata(w_wdata), .counted(w_counted), .grant(gnt[6]), .done(t_done)
);

`ifdef MISTER_OCRAM_EVT
wire [31:0] bind_word = {20'd0, c0_open, 2'd0, evt_bind_q, 5'd0, c0_sbind};
`else
wire [31:0] bind_word = {20'd0, c0_open, 8'd0, c0_sbind};
`endif

// The arbiter (ocram_arb.sv): each port's request, offset and data from registers or a table; a consumer only in a
// session, and never while a MARK's first payload word waits.
ocram_arb #(.F(NP)) arb
(
	.clk(clk), .en(engine_idle && !hrst_q && !run_fall && !run_set), .kill(hrst_rise),
	.req({(phase == P_SESSION) && c1_req_v && !w_hold, (phase == P_SESSION) && c0_req_v && !w_hold, w_req,
	      bind_pend && !bind_hold, sto_pend, hj_act && hj_has(hj_k), poll_sel, fault_pend, term_pend}),
	.we({c1_we, c0_we, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1}),
	.off({c1_off, c0_off, w_off, C_BIND, C_HELLO, hj_off(hj_k), poll_off, C_STATE, C_HELLO}),
	.wdata({c1_wdata, c0_wdata, w_wdata, bind_word, sto_data, hello_data(hj_k, hj_state, echo_q, c0_wide), 32'd0,
	        state_word(hello_cnt, stopped_q, osd_stopped_q, ts_stopped_q, fio_stopped_q, mnt_stopped_q,
	                   prev_mid_q, prev_mid_osd_q, prev_mid_ts_q, prev_mid_fio_q, prev_mid_mnt_q),
	        C_HELLO_STOP}),
	.counted({1'b1, 1'b1, w_counted, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0}),
	.grant(gnt),
	.t_go(t_go), .t_we(t_we), .t_off(t_off), .t_wdata(t_wdata), .t_counted(t_counted), .t_done(t_done),
	.rdata(rdata), .c_done(a_cdone), .c_rdata(a_crdata)
);

`ifdef MISTER_OCRAM_DIAG
// The hello starts the torture sink's counters; a MARK record restarts its period. The period counter marks the
// period due once and holds it until the record begins.
wire mark_restart = diag_clr | mark_start;
always @(posedge clk) begin
	if (mark_restart) begin
		mark_t <= MARK_T0;
		mark_due_q <= 1'b0;
	end
	else if (mark_t == {MARKW{1'b1}}) mark_due_q <= 1'b1;
	else mark_t <= mark_t + MARK_ONE;
end
`endif

//////////////////////////////  the sequencer  //////////////////////////////

wire fault_go = (t_job == J_FAULT);

always @(posedge clk) begin
	// poll bookkeeping: a launched poll read clears the pending period; a new one sets it
	if (t_go && !t_we && poll_sel && (poll_rd_kind == R_HELLO)) poll_pend <= 1'b0;
	else if (poll_tick && run && !hrst_q) poll_pend <= 1'b1;

	// the line's back-to-back tail re-read yields one turn to the consumers after each of its own reads, and
	// takes it back once a consumer is granted or none is waiting
	if ((t_job == J_POLL) && (((poll_step == 3'd4) && (poll_rd_kind == R_IDX)) ||
	                          ((poll_step == 3'd7) && (poll_rd_kind == R_MARK))))
		line_yield_q <= 1'b1;
	else if ((t_job == J_C0) || (t_job == J_C1) || !consumer_req)
		line_yield_q <= 1'b0;

	// a consumer fault: the ring's bit and the C_STATE store it owes (a fault in the store's own cycle is kept)
	if (OSD_EN != 0 && c0_fault) osd_stopped_q <= 1'b1;
	if (TS_EN != 0 && c1_fault) ts_stopped_q <= 1'b1;
	if (SEC_EN != 0 && c0_sfault) ts_stopped_q <= 1'b1;
	if (FIO_EN != 0 && c0_ffault) fio_stopped_q <= 1'b1;
	if (MNT_EN && c0_mfault) mnt_stopped_q <= 1'b1;
	if (fault_go) fault_pend <= 1'b0;
	if ((OSD_EN != 0 && c0_fault) || (TS_EN != 0 && c1_fault) || (SEC_EN != 0 && c0_sfault) ||
	    (FIO_EN != 0 && c0_ffault) || (MNT_EN && c0_mfault)) fault_pend <= 1'b1;

	// the mark line, raised while a session is idle between polls: its re-read starts at once
	if (FIO_EN != 0 && phase == P_SESSION && poll_step == 3'd0 && mark_line) poll_step <= 3'd7;

	// the OSD bind's C_BIND store is owed whenever the enable link's open differs from the stored bit
	if (phase == P_SESSION && ({c0_open, c0_sbind} != bind_q)) bind_pend <= 1'b1;
`ifdef MISTER_OCRAM_EVT
	if (phase == P_SESSION && evt_bind_q != evt_bound_q) bind_pend <= 1'b1;
`endif

	// the hello clears the session's state when its first store is about to run (the writer its ring's)
	if (hello_start) begin
		phase       <= P_HELLO;
		hj_act      <= 1'b1;
		hj_k        <= 5'd0;
		stopped_q   <= 1'b0;
		line_en_q   <= 1'b0;
		poll_step   <= 3'd0;
		sto_pend    <= 1'b0;            // this hello's C_STATE and C_HELLO supersede a pending state word
		fault_pend  <= 1'b0;
		osd_stopped_q <= 1'b0;
		ts_stopped_q  <= 1'b0;
		fio_stopped_q <= 1'b0;
		mark_arm_q  <= 1'b0;
		osd_bind_q  <= 1'b0;
`ifdef MISTER_OCRAM_EVT
		evt_bind_q <= 1'b0; evt_bound_q <= 1'b0;
`endif
		bind_q      <= 4'd0;
		sec_ask_q   <= 3'd0;
		bind_pend   <= 1'b0;
		mnt_stopped_q <= 1'b0;
		hj_state    <= state_word(hello_cnt + 8'd1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
		                          prev_mid_q, prev_mid_osd_q, prev_mid_ts_q, prev_mid_fio_q, prev_mid_mnt_q);
`ifdef MISTER_OCRAM_DIAG
		verdict_pend<= 1'b0;
		deepest_q   <= 16'd0;
`endif
	end

	// the hello the read found: the stores wait for every consumer's rest
	if (hello_hit && !run_ended) begin
		phase  <= P_REST;
		echo_q <= rdata;
	end

	if (hrst_rise) begin
		// an HPS reset ends the run: nothing starts, no terminal store, the engine drops what is unanswered
		// when the level falls
		phase      <= P_OFF;
		term_pend  <= 1'b0;
		fault_pend <= 1'b0;
		sto_pend   <= 1'b0;
		hj_act     <= 1'b0;
		`ifdef MISTER_OCRAM_DIAG
			verdict_pend <= 1'b0;
		`endif
		`ifdef MISTER_OCRAM_SECTOR_SLOT
			brec_pend <= 1'b0;
		`endif
		boot_pend  <= 1'b0;
		boot_first <= 1'b1;
		poll_pend  <= 1'b0;
		poll_step  <= 3'd0;
		rd_act     <= R_NONE;
		wr_act     <= W_NONE;
		osd_bind_q <= 1'b0;
`ifdef MISTER_OCRAM_EVT
		evt_bind_q <= 1'b0; evt_bound_q <= 1'b0;
`endif
		bind_q     <= 4'd0;
		sec_ask_q  <= 3'd0;
		bind_pend  <= 1'b0;
		if (w_act) prev_mid_q <= 1'b1;
		if (OSD_EN != 0 && c0_mid) prev_mid_osd_q <= 1'b1;
		if (TS_EN != 0 && c1_mid) prev_mid_ts_q <= 1'b1;
		if (SEC_EN != 0 && c0_smid) prev_mid_ts_q <= 1'b1;
		if (FIO_EN != 0 && c0_fmid) prev_mid_fio_q <= 1'b1;
		if (MNT_EN && c0_mmid) prev_mid_mnt_q <= 1'b1;
	end
	else if (run_fall) begin
		// the stop: the transaction in flight completes in the engine; a record being written is abandoned and
		// its head is not stored; one terminal store follows unless the HPS reset ended the run. A record head
		// store already in flight still lands, so it is not an abandoned record.
		phase      <= P_OFF;
		if (w_act && !w_hfly) prev_mid_q <= 1'b1;
		if (OSD_EN != 0 && c0_mid) prev_mid_osd_q <= 1'b1;
		if (TS_EN != 0 && c1_mid) prev_mid_ts_q <= 1'b1;
		if (SEC_EN != 0 && c0_smid) prev_mid_ts_q <= 1'b1;
		if (FIO_EN != 0 && c0_fmid) prev_mid_fio_q <= 1'b1;
		if (MNT_EN && c0_mmid) prev_mid_mnt_q <= 1'b1;
		`ifdef MISTER_OCRAM_DIAG
			verdict_pend <= 1'b0;
		`endif
		`ifdef MISTER_OCRAM_SECTOR_SLOT
			brec_pend <= 1'b0;
		`endif
		hj_act     <= 1'b0;
		sto_pend   <= 1'b0;
		fault_pend <= 1'b0;
		boot_pend  <= 1'b0;
		boot_first <= 1'b1;
		poll_pend  <= 1'b0;
		poll_step  <= 3'd0;
		rd_act     <= R_NONE;
		osd_bind_q <= 1'b0;
`ifdef MISTER_OCRAM_EVT
		evt_bind_q <= 1'b0; evt_bound_q <= 1'b0;
`endif
		bind_q     <= 4'd0;
		sec_ask_q  <= 3'd0;
		bind_pend  <= 1'b0;
		if (!hrst_q) term_pend <= 1'b1;
	end
	else if (run_set) begin
		phase      <= P_START;
		boot_pend  <= 1'b1;
		boot_first <= 1'b1;
		poll_pend  <= 1'b0;
		poll_step  <= 3'd0;
		fault_pend <= 1'b0;
		sto_pend   <= 1'b0;
		hj_act     <= 1'b0;
		`ifdef MISTER_OCRAM_DIAG
			verdict_pend <= 1'b0;
		`endif
		`ifdef MISTER_OCRAM_SECTOR_SLOT
			brec_pend <= 1'b0;
		`endif
		rd_act     <= R_NONE;
		wr_act     <= W_NONE;
	end
	else if (t_go) begin
		// take the command
		if (t_we) begin
			case (t_job)
			J_TERM: begin wr_act <= W_TERM; term_pend <= 1'b0; end
			J_FAULT: begin wr_act <= W_FAULT; fault_pend <= 1'b0; end
			J_HELLO: wr_act <= W_HELLO;
			J_STO: begin wr_act <= W_ONE; sto_pend <= 1'b0; end
			J_BIND: begin
				wr_act <= W_BIND; bind_pend <= 1'b0; bind_q <= {c0_open, c0_sbind};
`ifdef MISTER_OCRAM_EVT
				evt_bound_q <= evt_bind_q;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
				brec_pend <= !stopped_q;
`endif
			end
			default: ;
			endcase
		end
		else begin
			rd_act <= poll_rd_kind;
			if (poll_sel && (poll_rd_kind == R_HELLO) && (phase == P_START)) boot_first <= 1'b0;
		end
		// a record's start: its source takes what the record keeps
`ifdef MISTER_OCRAM_SECTOR_SLOT
		if (w_start[S_BIND]) brec_pend <= 1'b0;
		if (w_start[S_MNT]) begin rec_lba_q <= m_lba; rec_mw_q <= m_write; end
`endif
`ifdef MISTER_OCRAM_EVT
		if (w_start[S_EV]) begin
			// the offer is spent into the record: a STATUS's word 0 or a RUMBLE's level is its first payload
			rec_lba_q <= ev_w1;
			rec_ek_q  <= ev_k;
			rec_pad_q <= ev_pad;
			st_flag_q <= ev_arg[3:0];
		end
`endif
`ifdef MISTER_OCRAM_DIAG
		if (w_start[S_VD]) begin
			// the eight verdict words are one instant, registered at the record's start
			vd0 <= ts_records;
			vd1 <= ts_words;
			vd2 <= ts_errors;
			vd3 <= ts_first_g;
			vd4 <= ts_first_got;
			vd5 <= ts_sum;
			vd6 <= {ts_skipped, ts_deepest};
			vd7 <= {osd_rows, osd_deepest};
			verdict_pend <= 1'b0;
		end
		if (w_start[S_MARK] && !run_flood) verdict_pend <= 1'b1;
`endif
	end

	// a write's completion advances its job; BRESP non-OKAY is counted by the diag and otherwise ignored
	if (t_done && !run_ended) begin
		if (wr_act == W_HELLO) begin
			wr_act <= W_NONE;
			if (hj_act) begin
				if (hj_k == HJ_LAST) begin
					hj_act    <= 1'b0;
					phase     <= P_SESSION;
					poll_step <= 3'd0;        // the first poll reads H_HELLO, then H_BIND and the rest
					hello_cnt <= hello_cnt + 8'd1;
					prev_mid_q<= 1'b0;
					prev_mid_osd_q <= 1'b0;
					prev_mid_ts_q  <= 1'b0;
					prev_mid_fio_q <= 1'b0;
					prev_mid_mnt_q <= 1'b0;
				end
				else hj_k <= hj_next(hj_k);
			end
		end
		else if (wr_act != W_NONE) wr_act <= W_NONE;
		// a record published: the DIAG deepest fill, the events tier's last STATUS stflg and RUMBLE level
`ifdef MISTER_OCRAM_DIAG
		if ((w_pub != {NS{1'b0}}) && ((w_head_nx - w_tail) > deepest_q)) deepest_q <= w_head_nx - w_tail;
`endif
`ifdef MISTER_OCRAM_EVT
		if (w_pub[S_EV] && rec_ek_q == EK_STATUS) st_last_q <= st_flag_q;
		if (w_pub[S_EV] && rec_ek_q == EK_RUMBLE) rum_last_q[{rec_pad_q, 4'd0} +: 16] <= rec_lba_q[15:0];
`endif
	end

	// a hello step that this build does not store is passed without a transaction
	if (hj_act && !hj_has(hj_k)) hj_k <= hj_next(hj_k);

	if (t_done && (rd_act != R_NONE) && !run_ended) begin
		rd_act <= R_NONE;
		if (!t_ok) begin
			// a read with a non-OKAY response yields no value: act as if it had not happened
			if (phase == P_SESSION) poll_step <= 3'd0;
			if ((rd_act == R_HELLO) && (phase == P_START)) boot_first <= 1'b0;
		end
		else if (rd_act == R_HELLO) begin
			if (phase == P_START) begin
				boot_pend <= 1'b0;
				phase     <= (rdata == 32'd0) ? P_ARMED : P_WAIT;
				sto_pend  <= 1'b1;
				sto_data  <= (rdata == 32'd0) ? C_HELLO_ARMED : C_HELLO_WAIT;
			end
			else if (phase == P_ARMED) begin
				if (((rdata[31:16] != HELLO_MAGIC) || (rdata[15:0] == 16'd0)) && (rdata != 32'd0)) begin
					phase    <= P_WAIT;
					sto_pend <= 1'b1;
					sto_data <= C_HELLO_WAIT;
				end
			end
			else if (phase == P_WAIT) begin
				if (rdata == 32'd0) begin
					phase    <= P_ARMED;
					sto_pend <= 1'b1;
					sto_data <= C_HELLO_ARMED;
				end
			end
			else if (phase == P_SESSION) begin
`ifdef MISTER_OCRAM_EVT
				if (rdata == echo_q) poll_step <= ((OSD_EN != 0 && !osd_bind_q) || !evt_bind_q) ? 3'd1 :
				                                  SEC_EN != 0 ? 3'd5 : FIO_EN != 0 ? 3'd6 : 3'd2;
`else
				if (rdata == echo_q) poll_step <= (OSD_EN != 0 && !osd_bind_q) ? 3'd1 : SEC_EN != 0 ? 3'd5 :
				                                  FIO_EN != 0 ? 3'd6 : 3'd2;
`endif
				else begin
					// the session ends: its line drops, and each record part-written is recorded for the next hello
					`ifdef MISTER_OCRAM_SECTOR_SLOT
						brec_pend <= 1'b0;
					`endif
					if (w_act) prev_mid_q <= 1'b1;
					if (OSD_EN != 0 && c0_mid) prev_mid_osd_q <= 1'b1;
					if (TS_EN != 0 && c1_mid) prev_mid_ts_q <= 1'b1;
					if (SEC_EN != 0 && c0_smid) prev_mid_ts_q <= 1'b1;
					if (FIO_EN != 0 && c0_fmid) prev_mid_fio_q <= 1'b1;
					if (MNT_EN && c0_mmid) prev_mid_mnt_q <= 1'b1;
					poll_step <= 3'd0;
					sto_pend  <= 1'b1;
					sto_data  <= (rdata == 32'd0) ? C_HELLO_ARMED : C_HELLO_WAIT;
					phase     <= (rdata == 32'd0) ? P_ARMED : P_WAIT;
					osd_bind_q<= 1'b0;
`ifdef MISTER_OCRAM_EVT
					evt_bind_q <= 1'b0; evt_bound_q <= 1'b0;
`endif
					bind_q    <= 4'd0;
					sec_ask_q <= 3'd0;
					bind_pend <= 1'b0;
				end
			end
		end
		else if (rd_act == R_BIND) begin
			if (rdata[11]) osd_bind_q <= 1'b1;
`ifdef MISTER_OCRAM_EVT
			if (rdata[8]) evt_bind_q <= 1'b1;
`endif
			poll_step <= SEC_EN != 0 ? 3'd5 : FIO_EN != 0 ? 3'd6 : 3'd2;
		end
		else if (rd_act == R_SECT) begin
			sec_ask_q <= rdata[2:0] & SEC_MASK;
			poll_step <= FIO_EN != 0 ? 3'd6 : 3'd2;
		end
		else if (rd_act == R_MARK) begin
			// the periodic read goes on to H_LINE; a re-read gives the EVT line its turn, or goes again while the
			// word it read keeps the mark line up
			mark_arm_q <= rdata[31];
			mark_idx_q <= rdata[15:0];
			if (poll_step == 3'd6) poll_step <= 3'd2;
			else if (line_en_q && !stopped_q && (w_head != w_tail)) poll_step <= 3'd4;
			else if (rdata[31] && mark_reached(f_tail, rdata[15:0])) poll_step <= 3'd7;
			else poll_step <= 3'd0;
		end
		else if (rd_act == R_LINE) begin
			line_en_q <= rdata[3];
			poll_step <= 3'd3;
		end
		else if (rd_act == R_IDX) begin
			// the tail itself is the writer's (t_set)
			if (idx_fault) begin
				// a host defect: bit 11 at once, the ring stops for the rest of the session
				stopped_q  <= 1'b1;
				fault_pend <= 1'b1;
				poll_step  <= 3'd0;
				`ifdef MISTER_OCRAM_SECTOR_SLOT
					brec_pend <= 1'b0;
				`endif
				if (w_act) prev_mid_q <= 1'b1;
			end
			else if (!stopped_q) begin
`ifdef MISTER_OCRAM_DIAG
				if (((w_head - rdata[15:0]) & 16'hFFFF) > deepest_q)
					deepest_q <= (w_head - rdata[15:0]) & 16'hFFFF;
`endif
				poll_step <= (FIO_EN != 0 && mark_line) ? 3'd7 : (line_en_q && (w_head != rdata[15:0])) ? 3'd4 : 3'd0;
			end
			else poll_step <= 3'd0;
		end
	end

`ifdef MISTER_OCRAM_EVT
	// the events tier's offers: one asked for at a time; spent into a record, or at once when it needs none
	ev_abandon <= 1'b0;
	if (e_ask) ev_wait <= 1'b1;
	if (e_v) begin ev_wait <= 1'b0; ev_have <= 1'b1; end
	else if (w_start[S_EV] || (w_wtake[S_EV] && st_wait)) ev_have <= 1'b0;
	else if (ev_have && ev_bound && !st_open && !ev_rec) ev_have <= 1'b0;
	if (ev_drop) ev_abandon <= 1'b1;
	if (hello_start) begin
		st_last_q  <= 4'd0;
		rum_last_q <= 96'd0;
	end
`endif

	// the EVT line: high in a session, allowed by H_LINE, the ring not stopped, the head not the tail
	line <= (phase == P_SESSION) && line_en_q && !stopped_q && (w_head != w_tail);
	// the mark line: high in a session while H_MARK[1] is armed and the download ring's tail has reached its index
	mark_line <= FIO_EN != 0 && (phase == P_SESSION) && mark_arm_q && mark_reached(f_tail, mark_idx_q);
end

`ifdef MISTER_OCRAM_DIAG
hps_ocram_diag diag
(
	.clk        (clk),
	.clr        (diag_clr),
	.mark_start (mark_start),
	.snap_ready (snap_ready),
	.mark_i     (rec_k[5:0] - 6'd1),
	.mark_word  (mark_word),
	.counted    (counted_q),
	.awvalid    (awvalid),
	.arvalid    (arvalid),
	.bvalid     (bvalid),
	.bready     (bready),
	.bresp      (bresp),
	.rvalid     (rvalid),
	.rready     (rready),
	.rresp      (rresp),
	.drop       (drop_q),
	.deepest    (deepest_q),
	.ns         (run_ns),
	.flood      (run_flood),
	.hello_cnt  (hello_cnt)
);
`endif

endmodule


`ifdef MISTER_OCRAM_DIAG
//============================================================================
//  hps_ocram_tsink -- the DIAG host-to-fabric torture sink (MISTER_OCRAM_DIAG only)
//
//  A host-to-core ring, r = 4, in the SECTOR span (base 0x7000, 8,192 words), consumed on clk_100m through the
//  master's consumer port 1 in every session, with no bind. Records are {kind 0x50 TSEQ, seq[7:0], len[15:0]}
//  then len payload words, where payload word g (the count of payload words since the hello, from 0) must equal
//  v(g) = g ^ {g[15:0], g[31:16]} ^ 0x9E3779B9, and seq is the record's number since the hello, mod 256. A kind
//  other than 0x50 is skipped by its length and counted as skipped. A head more than the ring's size ahead, or a
//  length past the head, stops the ring for the session and pulses c_fault (C_STATE bit 12).
//
//  Counters, cleared at the hello: records (complete TSEQ records), words and the wrap-around sum of every
//  payload word read, errors (bad words and bad seqs), first_g and first_got (the first bad word's global index
//  and value; 0 until one), skipped and the deepest (head - tail) read. They are the VERDICT record's payload,
//  which the master snapshots at the record's start: {kind 0x25, arg 4, len 8}, posted by the EVT writer after
//  the MARK a period starts, with {osd_rows, osd_deepest} in its last word.
//
//  Address bound: H_IDX[4] or TS_BASE + ((index & (TS_WORDS - 1)) << 2); C_IDX[4] for its tail. Both controls
//  are below 0x100 and the span ends at 0xF000; a generate-time guard stops elaboration otherwise.
//============================================================================

/* verilator lint_off DECLFILENAME */   // the sink shares this file so the fit reports its ALMs apart
module hps_ocram_tsink
(
	input  wire        clk,          // clk_100m
	input  wire        s_hello,      // one cycle: the hello's first store is about to run
	input  wire        s_live,       // P_SESSION
	input  wire        s_tick,       // the poll period's pulse
	output wire        c_rest,       // the walker is idle: the half is at rest
	output wire        c_fault,      // one cycle: the ring stopped on a fault
	output wire        c_mid,        // level: a record is part-read

	output wire        c_req,
	output wire        c_we,
	output wire [15:0] c_off,
	output wire [31:0] c_wdata,
	input  wire        c_grant,
	input  wire        c_done,
	input  wire        c_ok,
	input  wire [31:0] c_rdata,

	output wire [31:0] v_records,
	output wire [31:0] v_words,
	output wire [31:0] v_errors,
	output wire [31:0] v_first_g,
	output wire [31:0] v_first_got,
	output wire [31:0] v_sum,
	output wire [15:0] v_skipped,
	output wire [15:0] v_deepest
);

localparam [15:0] H_IDX4 = 16'h0020;       // host: the torture ring's head
localparam [15:0] C_IDX4 = 16'h00A0;       // fabric: the torture ring's tail
localparam [15:0] TS_BASE = 16'h7000;      // the ring's first word
localparam [15:0] TS_WORDS = 16'd8192;

generate
	if (TS_BASE + TS_WORDS * 4 > 16'hF000) begin : g_ts_span
		ocram_ts_span_above_0xF000 guard();
	end
	if (H_IDX4 >= 16'h0100 || C_IDX4 >= 16'h0100) begin : g_ts_ctrl
		ocram_ts_control_constant_above_0xFF guard();
	end
endgenerate

localparam [2:0] T_IDLE = 3'd0, T_HEAD = 3'd1, T_HDR = 3'd2, T_PAY = 3'd3, T_TAIL = 3'd4, T_STOP = 3'd5;

reg  [2:0]  tst = T_IDLE;
reg         req = 1'b0;
reg         req_we = 1'b0;
reg  [15:0] req_off = 16'd0;
reg  [31:0] req_wdata = 32'd0;
reg  [15:0] tail = 16'd0;
reg  [15:0] head_c = 16'd0;
reg         head_due = 1'b1;
reg  [15:0] rec_len = 16'd0;
reg  [15:0] pay_i = 16'd0;             // the payload word being read, 0..len-1
reg         mid_q = 1'b0;
reg         fault_q = 1'b0;
reg         tseq = 1'b0;               // the record being read is a TSEQ
reg  [7:0]  exp_seq = 8'd0;
reg  [31:0] g = 32'd0;
reg  [31:0] records = 32'd0, words = 32'd0, errors = 32'd0;
reg  [31:0] first_g = 32'd0, first_got = 32'd0, sum = 32'd0;
reg         got_bad = 1'b0;
reg  [15:0] skipped = 16'd0, deepest = 16'd0;

assign c_req    = req;
assign c_we     = req_we;
assign c_off    = req_off;
assign c_wdata  = req_wdata;
assign c_fault  = fault_q;
assign c_mid    = mid_q;
assign c_rest   = (tst == T_IDLE) && !req;
assign v_records = records;
assign v_words   = words;
assign v_errors  = errors;
assign v_first_g = first_g;
assign v_first_got = first_got;
assign v_sum     = sum;
assign v_skipped = skipped;
assign v_deepest = deepest;

wire [15:0] pay1_idx = tail + 16'd1;
wire [15:0] pay2_idx = tail + 16'd1 + pay_i + 16'd1;
wire [15:0] hdr_off  = TS_BASE + ((tail & (TS_WORDS - 1)) << 2);
wire [15:0] pay1_off = TS_BASE + ((pay1_idx & (TS_WORDS - 1)) << 2);
wire [15:0] pay2_off = TS_BASE + ((pay2_idx & (TS_WORDS - 1)) << 2);
wire [15:0] head_gap = head_c - tail;
wire [15:0] new_gap  = d_data_q[15:0] - tail;          // every decision is one cycle after the response
wire [16:0] hdr_words = {1'b0, d_data_q[15:0]} + 17'd1;
wire [16:0] gap_words = {1'b0, head_gap};
wire [31:0] v_expect  = g ^ {g[15:0], g[31:16]} ^ 32'h9E37_79B9;

// The consumer's read word is registered before any decision: the fpga2hps atom's rdata reaches d_data_q and
// nothing else in the response cycle, so the header compare, the length arithmetic and the next offset sit on
// registers. The state machine acts one cycle after c_done, on that copy; a test sink can afford the cycle.
reg         d_v_q = 1'b0;              // a completion is held for this cycle
reg         d_ok_q = 1'b0;
reg  [31:0] d_data_q = 32'd0;

always @(posedge clk) begin
	fault_q <= 1'b0;
	d_v_q   <= 1'b0;
	if (c_done) begin
		d_v_q    <= 1'b1;
		d_ok_q   <= c_ok;
		d_data_q <= c_rdata;
	end

	if (s_hello) begin
		tst     <= T_IDLE;
		req     <= 1'b0;
		tail    <= 16'd0;
		head_c  <= 16'd0;
		head_due<= 1'b1;
		mid_q   <= 1'b0;
		tseq    <= 1'b0;
		exp_seq <= 8'd0;
		g       <= 32'd0;
		records <= 32'd0;
		words   <= 32'd0;
		errors  <= 32'd0;
		first_g <= 32'd0;
		first_got <= 32'd0;
		sum     <= 32'd0;
		got_bad <= 1'b0;
		skipped <= 16'd0;
		deepest <= 16'd0;
		pay_i   <= 16'd0;
		d_v_q   <= 1'b0;               // a completion of the ending session is not acted on
	end
	else begin
		if (s_tick) head_due <= 1'b1;

		case (tst)
		T_IDLE: if (s_live) begin
			if (head_due && (head_c == tail)) begin
				req      <= 1'b1;
				req_we   <= 1'b0;
				req_off  <= H_IDX4;
				head_due <= 1'b0;
				tst      <= T_HEAD;
			end
			else if (head_gap != 16'd0) begin
				req     <= 1'b1;
				req_we  <= 1'b0;
				req_off <= hdr_off;
				mid_q   <= 1'b1;
				tst     <= T_HDR;
			end
		end
		T_HEAD: begin
			if (!s_live) begin req <= 1'b0; tst <= T_IDLE; end
			else if (c_grant) req <= 1'b0;
			else if (d_v_q) begin
				if (!d_ok_q) req <= 1'b1;
				else if (new_gap > TS_WORDS) begin
					// a head more than the ring's size ahead: a host defect
					fault_q   <= 1'b1;
					req       <= 1'b0;
					mid_q     <= 1'b0;
					tst       <= T_STOP;
				end
				else begin
					head_c <= d_data_q[15:0];
					if (new_gap > deepest) deepest <= new_gap;
					if (d_data_q[15:0] == tail) tst <= T_IDLE;
					else begin
						req     <= 1'b1;
						req_we  <= 1'b0;
						req_off <= hdr_off;
						mid_q   <= 1'b1;
						tst     <= T_HDR;
					end
				end
			end
		end
		T_HDR: begin
			if (!s_live) begin req <= 1'b0; tst <= T_IDLE; end
			else if (c_grant) req <= 1'b0;
			else if (d_v_q) begin
				if (!d_ok_q) begin
					req <= 1'b1;
				end
				else if (hdr_words > gap_words || hdr_words > {1'b0, TS_WORDS}) begin
					// a length past the head or the ring: a host defect
					fault_q   <= 1'b1;
					req       <= 1'b0;
					mid_q     <= 1'b0;
					tst       <= T_STOP;
				end
				else if (d_data_q[31:24] != 8'h50) begin
					// an unknown kind: skipped by its length, counted
					skipped   <= skipped + 16'd1;
					tail      <= tail + 16'd1 + d_data_q[15:0];
					req       <= 1'b1;
					req_we    <= 1'b1;
					req_off   <= C_IDX4;
					req_wdata <= {16'd0, tail + 16'd1 + d_data_q[15:0]};
					tseq      <= 1'b0;
					tst       <= T_TAIL;
				end
				else begin
					if (d_data_q[23:16] != exp_seq) errors <= errors + 32'd1;
					exp_seq <= exp_seq + 8'd1;
					rec_len <= d_data_q[15:0];
					pay_i   <= 16'd0;
					tseq    <= 1'b1;
					req     <= 1'b1;
					req_we  <= 1'b0;
					if (d_data_q[15:0] == 16'd0) begin
						// a TSEQ with no payload words: the header alone is the record
						tail      <= tail + 16'd1;
						req_we    <= 1'b1;
						req_off   <= C_IDX4;
						req_wdata <= {16'd0, tail + 16'd1};
						tst       <= T_TAIL;
					end
					else begin
						req_off <= pay1_off;
						tst     <= T_PAY;
					end
				end
			end
		end
		T_PAY: begin
			if (!s_live) begin req <= 1'b0; tst <= T_IDLE; end
			else if (c_grant) req <= 1'b0;
			else if (d_v_q) begin
				if (!d_ok_q) req <= 1'b1;
				else if (!s_live) tst <= T_IDLE;
				else begin
					words <= words + 32'd1;
					sum   <= sum + d_data_q;
					if (d_data_q != v_expect) begin
						errors <= errors + 32'd1;
						if (!got_bad) begin
							first_g   <= g;
							first_got <= d_data_q;
							got_bad   <= 1'b1;
						end
					end
					g <= g + 32'd1;
					if (pay_i == rec_len - 16'd1) begin
						tail      <= tail + 16'd1 + rec_len;
						req       <= 1'b1;
						req_we    <= 1'b1;
						req_off   <= C_IDX4;
						req_wdata <= {16'd0, tail + 16'd1 + rec_len};
						tst       <= T_TAIL;
					end
					else begin
						pay_i   <= pay_i + 16'd1;
						req     <= 1'b1;
						req_we  <= 1'b0;
						req_off <= pay2_off;
					end
				end
			end
		end
		T_TAIL: begin
			if (!s_live) begin
				req <= 1'b0;                            // the session ended: the tail is not stored
				tst <= T_IDLE;
			end
			else if (c_grant) begin
				req   <= 1'b0;
				mid_q <= 1'b0;
				if (tseq) records <= records + 32'd1;
				head_due <= 1'b1;
			end
			else if (d_v_q && !d_ok_q) req <= 1'b1;
			else if (d_v_q) tst <= T_IDLE;
		end
		T_STOP: ;
		default: tst <= T_IDLE;
		endcase
	end
end

endmodule
/* verilator lint_on DECLFILENAME */
`endif

`ifdef MISTER_OCRAM_DIAG
//============================================================================
//  hps_ocram_diag -- the MARK record's counters (MISTER_OCRAM_DIAG only)
//
//  A counted transaction's latency is timed by a per-direction counter that starts at its address handshake
//  and advances one a clk, so no 32-bit subtract sits in front of the histogram logic. The accounting is
//  pipelined in three registered stages, one transaction in each:
//    A  the R/B handshake's edge: the latency, the error flag, the direction, and a tag saying whether the
//       transaction handed off at or before the snapshot request;
//    B  the next edge: the latency's histogram bin, its 16-bit saturated form, and the flags;
//    C  the next edge: every counter of that transaction together -- count, sum, min, max, errors and its
//       histogram bin -- so a snapshot never holds half a transaction.
//  The snapshot is requested by the MARK record's start; it completes when A and B hold no transaction tagged
//  older than the request, so it has every counted handshake at or before the request's edge and none after.
//  snap_ready then lets the master send the record's first payload word. A hello empties the pipeline and the
//  counters; an HPS reset drops only the transaction it orphaned.
//  Every array is logic (no block memory), and each histogram has one adder and one read mux with a decoded
//  write enable.
//============================================================================

/* verilator lint_off DECLFILENAME */   // the sheet keeps the counters in this file so the fit reports their ALMs apart
module hps_ocram_diag
(
	input  wire        clk,
	input  wire        clr,          // a hello: clear every counter and empty the pipeline
	input  wire        mark_start,   // a MARK record begins at this edge: request the snapshot
	output reg         snap_ready,   // the snapshot is complete; payload word 1 may be sent
	input  wire  [5:0] mark_i,       // payload word 0-39
	output reg  [31:0] mark_word,

	input  wire        counted,      // the transaction launched is a counted one
	input  wire        awvalid,
	input  wire        arvalid,
	input  wire        bvalid,
	input  wire        bready,
	input  wire  [1:0] bresp,
	input  wire        rvalid,
	input  wire        rready,
	input  wire  [1:0] rresp,
	input  wire        drop,         // the HPS reset dropped an unanswered transaction

	input  wire [15:0] deepest,
	input  wire        ns,
	input  wire        flood,
	input  wire  [7:0] hello_cnt
);

// The latency: one more each clk from the address handshake's cycle; the handshake cycle's value is the
// number of edges between ARVALID's/AWVALID's first rise and the R/B handshake, as before.
reg [31:0] w_age = 32'd0, r_age = 32'd0;
reg        w_meas = 1'b0, r_meas = 1'b0;

reg aw_d = 1'b0, ar_d = 1'b0;
wire aw_rise = awvalid & ~aw_d;
wire ar_rise = arvalid & ~ar_d;
always @(posedge clk) begin aw_d <= awvalid; ar_d <= arvalid; end

wire b_fire = bvalid & bready;
wire r_fire = rvalid & rready;
wire w_cap = b_fire & w_meas;
wire r_cap = r_fire & r_meas;

// stage A registers
reg        a_v = 1'b0, a_we = 1'b0, a_err = 1'b0, a_old = 1'b1;
reg [31:0] a_lat = 32'd0;
// stage B registers
reg        b_v = 1'b0, b_we = 1'b0, b_err = 1'b0, b_old = 1'b1;
reg [31:0] b_lat = 32'd0;
reg [15:0] b_lat16 = 16'd0;
reg  [3:0] b_bin = 4'd0;

// counts, sums, extrema and errors
reg [31:0] rd_n = 32'd0, wr_n = 32'd0;
reg [31:0] rd_sum = 32'd0, wr_sum = 32'd0;
reg [15:0] rd_max = 16'd0, wr_max = 16'd0, rd_min = 16'hFFFF, wr_min = 16'hFFFF;
reg [15:0] rd_err = 16'd0, wr_err = 16'd0;

(* ramstyle = "logic" *) reg [31:0] rd_hist [0:15];
(* ramstyle = "logic" *) reg [31:0] wr_hist [0:15];
// The snapshot a MARK record reads, word by word: word 0 carries the ring's deepest fill, the DIAG knobs and
// C_STATE's hello count; words 1-7 the counters; 8-23 and 24-39 the two histograms.
(* ramstyle = "logic" *) reg [31:0] snap [0:39];

// bins (cycles): 0-7, 8-15, ... 56-63, 64-127, 128-255, 256-511, 512-1023, 1024-2047, 2048-4095,
// 4096-65535, 65536 and more. Each boundary is a power of two, so the decode is bit tests, not compares.
/* verilator lint_off UNUSEDSIGNAL */   // bins 0-7 are 8 cycles wide, so bits [2:0] select nothing
function [3:0] bin_of(input [31:0] lat);
	begin
		if (|lat[31:17] || lat[16]) bin_of = 4'd15;
		else if (|lat[15:12])       bin_of = 4'd14;
		else if (lat[11])           bin_of = 4'd13;
		else if (lat[10])           bin_of = 4'd12;
		else if (lat[9])            bin_of = 4'd11;
		else if (lat[8])            bin_of = 4'd10;
		else if (lat[7])            bin_of = 4'd9;
		else if (lat[6])            bin_of = 4'd8;
		else                        bin_of = {1'b0, lat[5:3]};
	end
endfunction
/* verilator lint_on UNUSEDSIGNAL */

function [15:0] sat16(input [31:0] lat);
	sat16 = (|lat[31:16]) ? 16'hFFFF : lat[15:0];
endfunction

wire [31:0] w_inc = wr_hist[b_bin] + 32'd1;
wire [31:0] r_inc = rd_hist[b_bin] + 32'd1;

// the snapshot request; snap_ready is its completion level
reg snap_req = 1'b0;

integer hi;
always @(posedge clk) begin
	if (clr) begin
		rd_n <= 32'd0; wr_n <= 32'd0;
		rd_sum <= 32'd0; wr_sum <= 32'd0;
		rd_max <= 16'd0; wr_max <= 16'd0; rd_min <= 16'hFFFF; wr_min <= 16'hFFFF;
		rd_err <= 16'd0; wr_err <= 16'd0;
		for (hi = 0; hi < 16; hi = hi + 1) begin
			rd_hist[hi] <= 32'd0;
			wr_hist[hi] <= 32'd0;
		end
		a_v <= 1'b0; b_v <= 1'b0;
		w_meas <= 1'b0; r_meas <= 1'b0;
		snap_req <= 1'b0; snap_ready <= 1'b0;
	end
	else begin
		// stage C: all of B's counters land at this edge
		if (b_v && b_we) begin
			wr_n   <= wr_n + 32'd1;
			wr_sum <= wr_sum + b_lat;
			if (b_lat16 > wr_max) wr_max <= b_lat16;
			if (b_lat16 < wr_min) wr_min <= b_lat16;
			if (b_err && (wr_err != 16'hFFFF)) wr_err <= wr_err + 16'd1;
			for (hi = 0; hi < 16; hi = hi + 1)
				if (b_bin == hi[3:0]) wr_hist[hi] <= w_inc;
		end
		else if (b_v) begin
			rd_n   <= rd_n + 32'd1;
			rd_sum <= rd_sum + b_lat;
			if (b_lat16 > rd_max) rd_max <= b_lat16;
			if (b_lat16 < rd_min) rd_min <= b_lat16;
			if (b_err && (rd_err != 16'hFFFF)) rd_err <= rd_err + 16'd1;
			for (hi = 0; hi < 16; hi = hi + 1)
				if (b_bin == hi[3:0]) rd_hist[hi] <= r_inc;
		end

		// stage B: the bin, the saturated latency and the flags
		b_v     <= a_v;
		b_we    <= a_we;
		b_lat   <= a_lat;
		b_lat16 <= sat16(a_lat);
		b_bin   <= bin_of(a_lat);
		b_err   <= a_err;
		b_old   <= a_old;

		// stage A: the handshake, and the latency counters that feed it
		a_v <= 1'b0;
		if (w_cap) begin
			a_v <= 1'b1; a_we <= 1'b1; a_lat <= w_age; a_err <= (bresp != 2'b00); a_old <= ~snap_req;
			w_meas <= 1'b0;
		end
		else if (aw_rise && counted) begin
			w_meas <= 1'b1; w_age <= 32'd1;
		end
		else if (w_meas) w_age <= w_age + 32'd1;
		if (r_cap) begin
			a_v <= 1'b1; a_we <= 1'b0; a_lat <= r_age; a_err <= (rresp != 2'b00); a_old <= ~snap_req;
			r_meas <= 1'b0;
		end
		else if (ar_rise && counted) begin
			r_meas <= 1'b1; r_age <= 32'd1;
		end
		else if (r_meas) r_age <= r_age + 32'd1;
		if (drop) begin w_meas <= 1'b0; r_meas <= 1'b0; end

		// the snapshot: requested at the record's start, taken once no older transaction is in A or B. A
		// transaction that hands off at the request's own edge is in A by the next, so this edge cannot take it.
		if (mark_start) begin snap_req <= 1'b1; snap_ready <= 1'b0; end
		if (snap_req && !((a_v && a_old) || (b_v && b_old))) begin
			snap[0] <= {hello_cnt, 6'd0, flood, ns, deepest};
			snap[1] <= rd_n;
			snap[2] <= wr_n;
			snap[3] <= {rd_max, rd_min};
			snap[4] <= {wr_max, wr_min};
			snap[5] <= rd_sum;
			snap[6] <= wr_sum;
			snap[7] <= {rd_err, wr_err};
			for (hi = 0; hi < 16; hi = hi + 1) begin
				snap[8  + hi] <= rd_hist[hi];
				snap[24 + hi] <= wr_hist[hi];
			end
			snap_req <= 1'b0;
			snap_ready <= 1'b1;
		end
	end
end

integer mi;
always @(*) begin
	mark_word = 32'd0;
	for (mi = 0; mi < 40; mi = mi + 1)
		if (mark_i == mi[5:0]) mark_word = snap[mi];
end

endmodule
/* verilator lint_on DECLFILENAME */
`endif

`ifdef MISTER_OCRAM_EVT
//============================================================================
//  hps_ocram_evt -- the events tier's port between hps_io and the master (MISTER_OCRAM_EVT only)
//
//  hps_io presents one offer at a time on HPS_BUS[374:333], {kind[1:0], arg[7:0], data[31:0]}, its rotor's item
//  (sys/hps_io.sv). The master's ask (clk) crosses to clk_sys in an lw_xfer; there the offer is launched back in a
//  second lw_xfer, and take (HPS_BUS[382]) is high in the launch cycle, so hps_io steps its rotor, and spends a
//  taken INFO or UPLOAD notice, at the edge that captures the offer. An ask met while the reply cell is busy waits
//  for it; an ask while the ask cell is busy waits too. A module of its own so the fit reports its ALMs apart.
//============================================================================

/* verilator lint_off DECLFILENAME */   // the port shares the master's file, as the sink does
module hps_ocram_evt
(
	input  wire        clk,          // clk_100m: the master's side
	input  wire        ask,          // one cycle: the master asks for the next offer
	output wire        e_v,          // one cycle: the offer is in e_d
	output wire [41:0] e_d,          // held until the next offer
	input  wire        clk_sys,
	input  wire [41:0] o_d,          // hps_io's offer
	output wire        take          // one cycle: the offer is launched; hps_io's rotor steps
);

wire a_busy, a_v, r_busy;
reg  a_owed = 1'b0;                  // clk: an ask not yet launched
reg  r_owed = 1'b0;                  // clk_sys: an ask arrived, its offer not yet launched
/* verilator lint_off PINCONNECTEMPTY */   // the ask carries no data
lw_xfer #(.W(1)) ask_x(.s_clk(clk), .s_go(ask || a_owed), .s_d(1'b1), .s_busy(a_busy),
                       .d_clk(clk_sys), .d_q(), .d_v(a_v));
/* verilator lint_on PINCONNECTEMPTY */
always @(posedge clk) a_owed <= (ask || a_owed) && a_busy;

assign take = (a_v || r_owed) && !r_busy;
always @(posedge clk_sys) r_owed <= (a_v || r_owed) && r_busy;
lw_xfer #(.W(42)) rep_x(.s_clk(clk_sys), .s_go(take), .s_d(o_d), .s_busy(r_busy),
                        .d_clk(clk), .d_q(e_d), .d_v(e_v));

endmodule
/* verilator lint_on DECLFILENAME */
`endif

`default_nettype wire
`endif
`undef MISTER_OCRAM_MARK
`undef MISTER_OCRAM_POLL
