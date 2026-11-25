//============================================================================
//  hps_ocram_rx.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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
`ifdef MISTER_OCRAM_OSD
`define LW_OCRAM_RX_BUILT
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_OCRAM_RX_BUILT
`endif
`ifdef MISTER_OCRAM_FIO
`define LW_OCRAM_RX_BUILT
`endif
`ifdef LW_OCRAM_RX_BUILT
`default_nettype none

// The host-to-core consumer of the OCRAM mailbox: ONE walker on clk_100m for every host-to-core ring it serves,
// one held-data word crossing each way, and the sinks on clk_sys. OSD_EN (MISTER_OCRAM_OSD) serves the OSD ring
// (r = 2, base 0x2000, 2,048 words) into every osd's osd_buffer through the LW OSDBUF writer's port; SEC_EN
// (MISTER_OCRAM_SECTOR_SLOT, which needs MISTER_LW) serves the SECTOR ring (r = 4, base 0x7000, 8,192 words) into
// one hps_io slot through the window's xsec feeder nets and lw_blk_mux; FIO_EN (MISTER_OCRAM_FIO) serves the
// download ring (r = 1, base 0x1000, 2^FIO_RING words; 8,192 at 0x7000) onto the link nets through lw_link; MNT_EN
// (MISTER_OCRAM_MOUNT0_SLOT, _MOUNT1_SLOT, beside SEC_EN) serves the mount ring (r = 0, base 0x6000, 1,024 words)
// into the mount slots k = 1, 2 through the window's save-slot nets and lw_save_mux, and takes their writes.
//
//  * clk_100m: the walker. It takes the master's consumer port 0, one transaction at a time, and serves the rings at
//    record granularity, round robin over those with work (ring ids 0 OSD, 1 SECTOR, 2 download, 3 mount).
//    Per ring it keeps its tail and avail = head - tail, read
//    at a head read; a head more than the ring's size ahead, or a length past avail, stops that ring for the session
//    with a one-cycle fault. One adder addresses every ring word: tail + k (the header is k = 0, the tail store is
//    the same sum at the record's end). Every read response is registered before a decision, except a payload
//    word, which goes straight into the forward cell's hold register (the one register rdata reaches); no read
//    whose response the cell takes is issued while the cell is busy. A head is read only when the ring is caught
//    up and a poll tick, a finished record or (SECTOR) a new request asks for it.
//  * The forward lw_xfer carries {tag, d}: 0 OSD_HEAD (the ROW header, row in d[20:16]), 1 SEC_HEAD (the LBA), 2
//    DATA (of the last head's ring), 3 FIO_HEAD (the LFRAME header as read). The done lw_xfer answers each: 0 took,
//    1 accept, 2 mismatch, 3 no request. One word is outstanding at a time. With MNT_EN the forward word is
//    {tag, wr, k, d} (a SEC_HEAD names its slot and, for a WRITE, its direction) and the answer {d, code} (a WRITE
//    word's answer carries the word).
//  * OSD: a ROW record crosses as its header and 64 DATA words. The writer on clk_sys writes each word's four bytes
//    from the capture register, little-endian, one a cycle, under the OSDBUF guard (both osd selects low and the
//    I/O view, in that cycle and the two before), and answers after the fourth; a word met with the enable low is
//    answered unwritten. An unknown kind, a ROW of another length or a row outside the OSD is skipped by length.
//  * SECTOR: a FRAME {0x10, 0, FRAME + 1}, then the LBA and FRAME words. The walker acts only while the slot is
//    bound (C_BIND bit 0 stored 1) and the request record says a request stands: it reads the header and the LBA
//    and crosses SEC_HEAD. Accept: the FRAME words stream, then the tail is stored. Mismatch: the tail moves past
//    the frame. No request: the frame stays until the request record changes or a poll tick. Other records are
//    skipped by length. No REQUEST is asked for unless a head read made after the request landed finds the ring
//    empty, once per request (rq_post, rq_start: the master's record writer).
//  * Download: an LFRAME {0x30, {odd, 0, sel}, len}, then len words of two link words each, low half first; odd
//    drops the last high half. The walker crosses its header as FIO_HEAD in the read's response cycle when the kind
//    is 0x30, arg[6:2] is 0, sel is in the select mask (FIO_SELS: 1, the FPGA select; with MISTER_OCRAM_UIO also 0,
//    the UIO select, and 2, 3, the OSD selects) and len is not 0, then each word as DATA; the tail is stored after
//    the last word's answer, which the player gives once the frame has ended. Every other record is skipped by
//    length. The ring is walked in every session; it has no bind.
//  * Capture (MISTER_OCRAM_LANE): an LFRAME with capture (arg[2]) and len 1-1,020 takes any tag (arg[6:3]); one
//    without capture still needs tag 0. Its header is held until e_free covers len + 1 words: short of that the
//    walker leaves it unconsumed and ring 1 waits, the other rings served; with room it asks the master for the
//    REPLY (p_post) and crosses the header once the record's header is written (p_start). The player samples
//    l_dout at each link word's acknowledge and answers each ring word with its two replies, low half first (the
//    high half 0 past an odd frame's end); the walker holds each answer in p_wd until the master takes it.
//  * The player (hps_ocram_fplay, clk_sys) is the LW lane player's: a frame starts with both link selects low for
//    two cycles, the link not busy and the enable up; each strobe is followed by GAP consecutive cycles with
//    l_wait low; the select drops between frames. A frame the session's end cuts ends short at the next word it
//    waits for with the enable low for two cycles.
//  * The bind: sec_ask (H_SECTOR bit 0) raises the bind level to clk_sys; c_sbind (C_BIND bit 0) is 1 once the
//    feeder's en_sys reads back 1. When the bit clears (or ring 3 or 4 faults) a frame being streamed finishes,
//    the level falls, the walker reads H_IDX[4] and stores C_IDX[4] = that head (every queued frame discarded), and
//    only then, with en_sys read back 0, c_sbind falls: a host that has read C_BIND = 0 finds no stale frame.
//  * The feeder (clk_sys) re-creates hps_io's strobes as lw_sector's primary slot does: en_sys follows the bind level
//    only while it is idle and the slot's SPI ack is low, and rises only while the commit mark reads 0; the ack
//    rises at an accept and the first strobe follows GAP cycles later; a word's strobes are GAP apart (two 16-bit
//    strobes with io_wide, else four bytes), its done goes with its last strobe, and the ack falls GAP - 1 cycles
//    after the frame's last strobe. A consumer reset, or the session's enable level low for two cycles while a word
//    is awaited, zero-fills from the next word; words arriving then are answered unplayed. The request record
//    {offered, lba} crosses whole at every change of offered (the slot's mux here, the core's request raised, the
//    ack low and two cycles past its fall).
//  * Mounts (MNT_EN): slot k = 1, 2's request crosses as its own record {k, wr, blk_cnt, offered, lba}, the two
//    slots' records in turn as each changes, and the walker keeps the last of each; offered only for a block of at
//    most MFRAME words, and never launched while a mount slot's ack is up. A read is posted at once: m_post asks the
//    master for REQUEST {0x21, {blk_cnt, k}, 1} once C_BIND bit k is stored 1. Ring 0 carries FRAME {0x10,
//    {blk_cnt, k}, 1 + n}: one that answers slot k's standing read (blk_cnt and len) crosses its LBA as SEC_HEAD
//    {k}; accept streams its words, and mismatch, no request and any other record are skipped by length. A WRITE
//    crosses SEC_HEAD {wr, k, lba}; on accept the feeder raises the ack, m_post asks for WRITE {0x26, {blk_cnt, k},
//    1 + n}, and each of the n words is fetched from the feeder by one DATA token, held in m_wd until the master
//    takes it; after the head store (m_pub) an end token (d[0] = 1) drops the ack. A fault of ring 3 sends the end
//    token at the next word. The binds (H_SECTOR bits 1, 2; C_BIND bits 1, 2) follow the disc slot's rules with
//    no flush: no frame waits in ring 0.
//  * The enable link (lw_fence) carries the session: hen = s_live, take while idle, drain while no word waits for
//    its done, fin = 1. c_open (C_BIND bit 11) is the link's open and the OSD bind; c_rest is its idle, so the
//    hello waits for the clk_sys half to rest. A record the session's end cuts is abandoned; c_mid / c_smid are
//    high from its header read to its tail store's grant.
//  * MISTER_OCRAM_DIAG (OSD only): GPO 25 is the freeze knob through lw_sync (no byte, no done, no rest while 1),
//    and the rows finished and the deepest OSD fill for the VERDICT record.
//
// Address bound: H_IDX[0], H_IDX[1], H_IDX[2], H_IDX[4], C_IDX[0], C_IDX[1], C_IDX[2], C_IDX[4], or a ring word
// BASE + ((index & (WORDS - 1)) << 2). The controls are below 0x100 and every span ends below 0xF000; generate-time
// guards stop elaboration otherwise.
// tb/static.sh holds it.

module hps_ocram_rx #(parameter ROWS = 16, parameter OSD_EN = 1, parameter SEC_EN = 0,
                      parameter [31:0] FRAME = 32'd128, parameter [31:0] GAP = 32'd2,
                      parameter FIO_EN = 0, parameter [31:0] FIO_RING = 32'd10, parameter [31:0] FIO_GAP = 32'd4,
                      parameter [1:0] MNT_EN = 2'd0, parameter [31:0] MFRAME = 32'd128, parameter [2:0] BLKSZ = 3'd2,
                      parameter [3:0] FIO_SELS = 4'b0010)
(
	input  wire        clk,          // clk_100m: the walker
	input  wire        clk_sys,      // the core clock: the sinks

	/* verilator lint_off UNUSEDSIGNAL */   // GPO 25 is the DIAG freeze knob; without DIAG it is never read
	input  wire        gp_out25,
	/* verilator lint_on UNUSEDSIGNAL */

	// The master's session interface
	input  wire        s_hello,      // one cycle: the hello's first store is about to run
	input  wire        s_live,       // P_SESSION
	input  wire        s_tick,       // the poll period's pulse
	input  wire        bind_en,      // the OSD bind (H_BIND bit 11 read)
	input  wire  [2:0] sec_ask,      // H_SECTOR bits 2:0, as last read
	input  wire        evt_stop,     // the EVT ring stopped on a fault
	output wire        c_rest,       // the enable link's idle: the clk_sys half is at rest
	output wire        c_fault,      // one cycle: the OSD ring stopped on a fault
	output wire        c_mid,        // level: an OSD record is part-consumed
	output wire        c_open,       // C_BIND bit 11
	output wire        c_sfault,     // one cycle: the SECTOR ring stopped on a fault
	output wire        c_smid,       // level: a SECTOR record is part-consumed
	output wire  [2:0] c_sbind,      // C_BIND bits 2:0
	output wire        c_wide,       // hps_io's io_wide, for C_K[1]
	output wire        rq_post,      // a REQUEST is owed for rq_lba
	output wire [31:0] rq_lba,
	input  wire        rq_start,     // one cycle: the master begins that REQUEST
	output wire [15:0] osd_rows,     // ROW records finished this session (DIAG)
	output wire [15:0] osd_deepest,  // the deepest OSD fill read this session (DIAG)
	output wire [15:0] f_tail,       // the download ring's tail, as last stored in C_IDX[1]
	output wire        c_ffault,     // one cycle: the download ring stopped on a fault
	output wire        c_fmid,       // level: a download record is part-consumed
	output wire        c_mfault,     // one cycle: the mount ring stopped on a fault
	output wire        c_mmid,       // level: a mount record is part-consumed
	// the mount slots' records, for the master's EVT writer (MNT_EN)
	output wire        m_post,       // a REQUEST or a WRITE is owed for a mount slot's request
	output wire        m_write,      // it is a WRITE
	output wire  [7:0] m_arg,        // {blk_cnt, k}
	output wire [10:0] m_n,          // the block's words
	output wire [31:0] m_lba,
	output wire        m_wv,         // a WRITE's next word is in m_wd
	output wire [31:0] m_wd,
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the master's answers are not read
	input  wire        m_start,      // one cycle: the master begins that record
	input  wire        m_wtake,      // one cycle: the master takes m_wd
	input  wire        m_pub,        // one cycle: the WRITE's head store is done
	/* verilator lint_on UNUSEDSIGNAL */
	// the capture lane's REPLY (MISTER_OCRAM_LANE), for the master's EVT writer
	output wire        p_post,       // a capture frame is held, with room, for its REPLY
	output wire  [7:0] p_arg,        // its arg
	output wire [10:0] p_n,          // its len
	output wire        p_wv,         // a reply word is in p_wd
	output wire [31:0] p_wd,
	/* verilator lint_off UNUSEDSIGNAL */   // without MISTER_OCRAM_LANE the master's answers are not read
	input  wire [15:0] e_free,       // the EVT ring's free words
	input  wire        p_start,      // one cycle: the REPLY's header is written
	input  wire        p_wtake,      // one cycle: the master takes p_wd
	input  wire [15:0] l_dout,       // clk_sys: the link's reply word (io_dout | io_dout_sys)
	/* verilator lint_on UNUSEDSIGNAL */

	// The master's consumer port: one transaction at a time
	output wire        c_req,
	output wire        c_we,
	output wire [15:0] c_off,
	output wire [31:0] c_wdata,
	input  wire        c_grant,
	input  wire        c_done,
	input  wire        c_ok,
	input  wire [31:0] c_rdata,

	// The OSDBUF write port every osd takes (osd.v), on clk_sys
	input  wire  [1:0] sel,          // both osd selects as each osd sees them, {vga, hdmi}
	input  wire        view,         // GPO[31]: 1 = the link's I/O view
	output wire        we,
	output wire [12:0] addr,
	output wire  [7:0] dbyte,

	// The SECTOR feeder (the window's xsec nets), on clk_sys
	/* verilator lint_off UNUSEDSIGNAL */   // without SEC_EN the feeder's inputs are not read
	input  wire        x_reset,      // the consumer's reset: zero-fills an open frame
	input  wire        x_hps_ack,    // the slot's SPI ack
	input  wire        x_req,        // the core's request, wired
	input  wire [31:0] x_lba,
	input  wire        x_commit,     // hps_io's commit mark
	input  wire        x_wide,       // hps_io's io_wide, a constant
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        x_en,         // en_sys: the slot's mux is on this feeder
	output wire        x_ack,
	output wire        x_wr,
	output wire [15:0] x_dat,
	output wire [13:0] x_adr,
	// the mount slots k = 1, 2 (the window's save-slot nets), {k = 2, k = 1}
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the mount slots' inputs are not read
	input  wire  [1:0] xm_hps_ack,   // each slot's SPI ack
	input  wire  [1:0] xm_rd,        // the core's requests, wired
	input  wire  [1:0] xm_wr,
	input  wire [63:0] xm_lba,       // the LBA, or while the slot's ack is up its buffer word
	input  wire [11:0] xm_cnt,       // the requests' blk_cnt
	/* verilator lint_on UNUSEDSIGNAL */
	output wire  [1:0] xm_en,        // en_sys: the slot's mux is on this feeder
	output wire  [1:0] xm_ack,

	// The download player's side of lw_link, on clk_sys
	/* verilator lint_off UNUSEDSIGNAL */   // without FIO_EN the player's inputs are not read
	input  wire        l_busy,       // any of the link's own selects, or its ID view
	input  wire        l_wait,       // io_wait | vs_wait
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        l_own,
	output wire        l_hold,
	output wire  [3:0] l_sel,
	output wire        l_strobe,
	output wire [15:0] l_din
);

localparam [15:0] H_IDX0    = 16'h0010;     // host: the mount ring's head
localparam [15:0] H_IDX1    = 16'h0014;     // host: the download ring's head
localparam [15:0] H_IDX2    = 16'h0018;     // host: the OSD ring's head
localparam [15:0] H_IDX4    = 16'h0020;     // host: the SECTOR ring's head
localparam [15:0] C_IDX0    = 16'h0090;     // fabric: the mount ring's tail
localparam [15:0] C_IDX1    = 16'h0094;     // fabric: the download ring's tail
localparam [15:0] C_IDX2    = 16'h0098;     // fabric: the OSD ring's tail
localparam [15:0] C_IDX4    = 16'h00A0;     // fabric: the SECTOR ring's tail
localparam [15:0] OSD_BASE  = 16'h2000;
localparam [15:0] OSD_WORDS = 16'd2048;
localparam [15:0] SEC_BASE  = 16'h7000;
localparam [15:0] SEC_WORDS = 16'd8192;
localparam [15:0] FIO_BASE  = (FIO_RING == 32'd13) ? 16'h7000 : 16'h1000;
localparam [15:0] FIO_WORDS = 16'd1 << FIO_RING[3:0];
// FIO_SELS, the selects an LFRAME may name ({osd_vga, osd_hdmi, fpga, uio}), is sys_top's OCRAM_FIO_SELS, the same
// parameter hps_ocram_master declares in C_K[9].
`ifdef MISTER_OCRAM_LANE
localparam        CAP_EN    = 1;            // the capture lane: capture LFRAMEs and their REPLY records
`else
localparam        CAP_EN    = 0;
`endif
localparam [15:0] MNT_BASE  = 16'h6000;
localparam [15:0] MNT_WORDS = 16'd1024;
`ifdef MISTER_OCRAM_DIAG
localparam        TS_SPAN   = 1;            // the DIAG torture sink walks ring 4's span
`else
localparam        TS_SPAN   = 0;
`endif
localparam  [7:0] LAST_ROW  = ROWS - 1;
localparam [15:0] SEC_LEN   = FRAME[15:0] + 16'd1;      // a FRAME record's len: the LBA and the words
localparam [13:0] SEC_K     = FRAME[13:0] + 14'd1;      // its last payload word

generate
	if (OSD_BASE + OSD_WORDS * 4 > 16'hF000 || SEC_BASE + SEC_WORDS * 4 > 16'hF000 ||
	    FIO_BASE + FIO_WORDS * 4 > 16'hF000 || MNT_BASE + MNT_WORDS * 4 > 16'hF000) begin : g_rx_span
		ocram_rx_span_above_0xF000 guard();
	end
	if (H_IDX0 >= 16'h0100 || H_IDX1 >= 16'h0100 || H_IDX2 >= 16'h0100 || H_IDX4 >= 16'h0100 ||
	    C_IDX0 >= 16'h0100 || C_IDX1 >= 16'h0100 || C_IDX2 >= 16'h0100 || C_IDX4 >= 16'h0100) begin : g_rx_ctrl
		ocram_rx_control_constant_above_0xFF guard();
	end
	if (SEC_EN != 0 && (FRAME == 0 || FRAME > 4096 || GAP < 2 || GAP > 15)) begin : g_geometry
		ocram_sector_unservable_geometry guard();
	end
	// the download ring: 2^2 to 2^10 words at 0x1000, or 2^13 at 0x7000; strobes 4 to 7 cycles apart
	if (FIO_EN != 0 && (FIO_GAP < 4 || FIO_GAP > 7 || !((FIO_RING >= 2 && FIO_RING <= 10) || FIO_RING == 13)))
	begin : g_fio_geometry
		ocram_fio_unservable_geometry guard();
	end
	// at 0x7000 the download ring takes ring 4's span, which the SECTOR tier and the DIAG sink also walk
	if (FIO_EN != 0 && FIO_RING == 13 && (SEC_EN != 0 || TS_SPAN != 0)) begin : g_fio_span
		ocram_fio_ring_on_ring4_span guard();
	end
	// the mount slots share the disc slot's feeder, its REQUEST writer and its bind rules
	if (MNT_EN != 0 && SEC_EN == 0) begin : g_mnt_sec
		ocram_mount_needs_MISTER_OCRAM_SECTOR_SLOT guard();
	end
endgenerate

///////////////////////////////  the crossings  //////////////////////////////

`ifdef MISTER_OCRAM_DIAG
(* preserve *) reg frz_lwx = 1'b0;
always @(posedge clk) frz_lwx <= gp_out25;
wire frz;
lw_sync #(.W(1)) frz_sync(.clk(clk_sys), .d(frz_lwx), .q(frz));
`else
wire frz = 1'b0;
`endif

localparam [1:0] TG_OHEAD = 2'd0, TG_SHEAD = 2'd1, TG_DATA = 2'd2, TG_FHEAD = 2'd3;
localparam [1:0] RG_OSD = 2'd0, RG_SEC = 2'd1, RG_FIO = 2'd2, RG_MNT = 2'd3;   // the walker's ring ids
localparam [1:0] A_TOOK = 2'd0, A_ACCEPT = 2'd1, A_MISMATCH = 2'd2;   // 3: no request

// The crossings' words, padded: {tag, wr, k, d} forward, {d, code} back; without MNT_EN only {tag, d} and the code
// cross (the forward cell's wr and k, and the answer's d, read 0).
wire [36:0] word_q;                    // clk_sys: the capture register {tag, wr, k, d}
wire        word_v;
wire        fwd_busy;
wire [33:0] done_q;                    // clk: {d, code}
wire        done_v;
wire        back_busy;
reg         done_go = 1'b0;            // clk_sys
reg   [1:0] done_code = 2'd0;
wire        x_go;                      // clk: the forward launch
/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the forward word's wr and k, and the answer's d, do not cross
reg  [31:0] done_dat = 32'd0;
wire [36:0] x_d;
/* verilator lint_on UNUSEDSIGNAL */

generate
if (MNT_EN != 0 || CAP_EN != 0) begin : xw
	lw_xfer #(.W(37)) rx_fwd(.s_clk(clk), .s_go(x_go), .s_d(x_d), .s_busy(fwd_busy),
	                         .d_clk(clk_sys), .d_q(word_q), .d_v(word_v));
	lw_xfer #(.W(34)) rx_done(.s_clk(clk_sys), .s_go(done_go), .s_d({done_dat, done_code}), .s_busy(back_busy),
	                          .d_clk(clk), .d_q(done_q), .d_v(done_v));
end
else begin : xn
	wire [33:0] fq;
	wire  [1:0] dq;
	lw_xfer #(.W(34)) rx_fwd(.s_clk(clk), .s_go(x_go), .s_d({x_d[36:35], x_d[31:0]}), .s_busy(fwd_busy),
	                         .d_clk(clk_sys), .d_q(fq), .d_v(word_v));
	lw_xfer #(.W(2)) rx_done(.s_clk(clk_sys), .s_go(done_go), .s_d(done_code), .s_busy(back_busy),
	                         .d_clk(clk), .d_q(dq), .d_v(done_v));
	assign word_q = {fq[33:32], 3'd0, fq[31:0]};
	assign done_q = {32'd0, dq};
end
endgenerate
wire  [1:0] done_c = done_q[1:0];

// the request record {offered, lba} and the bind levels each way, by slot index k
wire [32:0] rq_q;
wire        rq_v;
wire        rc_busy;
wire        rc_go;
/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN bits 2:1 do not cross
(* preserve *) reg [2:0] bind_lwx = 3'd0;   // clk: the bind levels
(* preserve *) reg [2:0] en_lwx = 3'd0;     // clk_sys: en_sys
/* verilator lint_on UNUSEDSIGNAL */
(* preserve *) reg wide_lwx = 1'b0;         // clk_sys: io_wide
wire  [2:0] bind_s, en_b;
wire        wide_b;
reg         offd = 1'b0;               // clk_sys: offered, registered
wire [32:0] rec = {offd, x_lba};
lw_xfer #(.W(33)) rx_req(.s_clk(clk_sys), .s_go(rc_go), .s_d(rec), .s_busy(rc_busy),
                         .d_clk(clk), .d_q(rq_q), .d_v(rq_v));
generate
if (MNT_EN != 0) begin : sw
	lw_sync #(.W(3)) bind_sync(.clk(clk_sys), .d(bind_lwx), .q(bind_s));
	lw_sync #(.W(4)) back_sync(.clk(clk), .d({wide_lwx, en_lwx}), .q({wide_b, en_b}));
end
else begin : sn
	// the disc slot's levels alone: the mount bits stay 0 and do not cross
	lw_sync #(.W(1)) bind_sync(.clk(clk_sys), .d(bind_lwx[0]), .q(bind_s[0]));
	lw_sync #(.W(2)) back_sync(.clk(clk), .d({wide_lwx, en_lwx[0]}), .q({wide_b, en_b[0]}));
	assign bind_s[2:1] = 2'd0, en_b[2:1] = 2'd0;
end
endgenerate
assign c_wide = wide_b;

// the mount channel's record {k, wr, blk_cnt, offered, lba}
wire [41:0] mq;
wire        mq_v;
wire        mrc_busy;
/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the channel's record is not sent
wire        mrc_go;
wire [41:0] mrec;
/* verilator lint_on UNUSEDSIGNAL */
generate
if (MNT_EN != 0) begin : mx
	lw_xfer #(.W(42)) rx_mreq(.s_clk(clk_sys), .s_go(mrc_go), .s_d(mrec), .s_busy(mrc_busy),
	                          .d_clk(clk), .d_q(mq), .d_v(mq_v));
end
else begin : nomx
	assign mq = 42'd0, mq_v = 1'b0, mrc_busy = 1'b0;
end
endgenerate

wire        f_open, f_idle, hen_s, rest;
reg         word_out = 1'b0;           // a word launched, its done not back
wire        take = s_live & f_idle;
/* verilator lint_off PINCONNECTEMPTY */   // OWNED, the launched level and the fall have no user here
lw_fence fence
(
	.clk(clk), .hen(s_live), .take(take), .drain(!word_out), .fin(1'b1),
	.open(f_open), .owned(), .idle(f_idle), .lvl(), .fall(),
	.clk_sys(clk_sys), .rest(rest), .hen_s(hen_s)
);
/* verilator lint_on PINCONNECTEMPTY */

///////////////////////////////  the walker  ////////////////////////////////

localparam [3:0] W_IDLE = 4'd0, W_HEAD = 4'd1, W_HDR = 4'd2, W_LBA = 4'd3, W_HGO = 4'd4, W_ANS = 4'd5,
                 W_PAY = 4'd6, W_XFER = 4'd7, W_TAIL = 4'd8, W_FLUSH = 4'd9, W_FTAIL = 4'd10,
                 W_MTK = 4'd11, W_MWD = 4'd12, W_MWH = 4'd13, W_MWP = 4'd14;
`ifdef MISTER_OCRAM_LANE
localparam [3:0] W_CAP = 4'd15;           // a capture frame waits for its REPLY's header
`endif

reg   [3:0] wst = W_IDLE;
reg         req = 1'b0;
reg         req_we = 1'b0;
reg  [15:0] req_off = 16'd0;
reg         rd_pend = 1'b0;            // a ring word read is owed at tail + k
reg         st_pend = 1'b0;            // the ring's tail store is owed: tail + k
reg   [1:0] ring = RG_OSD;             // the record's ring
reg   [1:0] turn = RG_OSD;             // the ring served last
reg  [15:0] tail_o = 16'd0, tail_s = 16'd0, tail_f = 16'd0, tail_m = 16'd0;
reg  [11:0] avail_o = 12'd0;
reg  [13:0] avail_s = 14'd0, avail_f = 14'd0;
reg  [10:0] avail_m = 11'd0;
reg         due_o = 1'b1, due_s = 1'b1, due_f = 1'b1, due_m = 1'b1;   // read the head when caught up
reg         stop_o = 1'b0, stop_s = 1'b0, stop_f = 1'b0, stop_m = 1'b0;
reg  [13:0] k = 14'd0;
reg         mid_o = 1'b0, mid_s = 1'b0, mid_f = 1'b0, mid_m = 1'b0;
reg         flt_o = 1'b0, flt_s = 1'b0, flt_f = 1'b0, flt_m = 1'b0;
reg   [4:0] row_q = 5'd0;              // the ROW being sent: its OSD_HEAD carries the row alone
reg  [13:0] flen_q = 14'd0;            // the LFRAME or mount FRAME being sent: its len, the k of its last word
`ifdef MISTER_OCRAM_DIAG
reg         was_row = 1'b0;            // the OSD record being finished is a ROW
`endif
reg         s_wait = 1'b0;             // a no-request answer: wait for the record or a tick
reg         sb = 1'b0;                 // C_BIND bit 0
reg         fl_pend = 1'b0;            // the unbind's flush is owed
reg         rq_posted = 1'b0, fresh = 1'b0;
reg         drop_s = 1'b0;             // the bind is being dropped
reg   [1:0] sbm = 2'd0;                // C_BIND bits 2:1
reg   [1:0] drop_m = 2'd0;             // a mount bind is being dropped
reg   [1:0] m_posted = 2'd0;           // each slot's request has its REQUEST or WRITE begun
reg   [1:0] mfk = 2'd1;                // the slot of the mount FRAME or WRITE being walked
reg  [39:0] mr1 = 40'd0, mr2 = 40'd0;  // each slot's last request record {wr, blk_cnt, offered, lba}
reg         mw_job = 1'b0;             // the walker serves slot mfk's WRITE
reg         w_acc = 1'b0;              // the feeder accepted it: the ack is up
reg         w_started = 1'b0;          // the master has begun its WRITE record
reg   [1:0] w_done = 2'd0;             // that slot's WRITE is over (served, refused or cut): none again for its record
reg         m_end = 1'b0;              // the next token is the end token
reg         m_have = 1'b0;             // m_wd holds a WRITE word for the master
`ifdef MISTER_OCRAM_DIAG
reg  [15:0] rows_q = 16'd0, deepest_q = 16'd0;
assign osd_rows = rows_q;
assign osd_deepest = deepest_q;
`else
assign osd_rows = 16'd0;
assign osd_deepest = 16'd0;
`endif

// each mount slot's request: standing, its direction, blk_cnt and LBA, and its block's words (at most MFRAME, 1,020:
// a longer request is never offered)
/* verilator lint_off UNUSEDSIGNAL */   // a block of at most 1,020 words
function automatic [10:0] mwn(input [5:0] cnt);
	reg [18:0] w;
	begin w = ({13'd0, cnt} + 19'd1) << ({1'b0, BLKSZ} + 4'd5); mwn = w[10:0]; end
endfunction
/* verilator lint_on UNUSEDSIGNAL */
wire  [1:0] mst = {mr2[32], mr1[32]} & MNT_EN;
wire  [1:0] mwr = {mr2[39], mr1[39]};
wire [10:0] mn1 = mwn(mr1[38:33]), mn2 = mwn(mr2[38:33]);
// the slot whose record is offered to the master: the WRITE being walked, else a bound read not yet posted
wire  [1:0] rd_ok = mst & ~mwr & sbm & ~drop_m & bind_lwx[2:1] & ~m_posted;
wire  [1:0] wj    = mst & mwr & sbm & ~drop_m & ~w_done;
wire  [1:0] pk    = mw_job ? mfk : rd_ok[0] ? 2'd1 : 2'd2;
wire        pk2   = pk == 2'd2;
/* verilator lint_off UNUSEDSIGNAL */   // its direction and offered bit are the walker's
wire [39:0] pr    = pk2 ? mr2 : mr1;
/* verilator lint_on UNUSEDSIGNAL */
wire [10:0] mn    = (mfk == 2'd2) ? mn2 : mn1;        // the walked slot's block

wire [15:0] tl     = (ring == RG_SEC) ? tail_s : (ring == RG_FIO) ? tail_f : (ring == RG_MNT) ? tail_m : tail_o;
wire [15:0] widx   = tl + {2'd0, k};
wire [15:0] osd_off = OSD_BASE + ((widx & (OSD_WORDS - 1)) << 2);
wire [15:0] sec_off = SEC_BASE + ((widx & (SEC_WORDS - 1)) << 2);
wire [15:0] fio_off = FIO_BASE + ((widx & (FIO_WORDS - 1)) << 2);
wire [15:0] mnt_off = MNT_BASE + ((widx & (MNT_WORDS - 1)) << 2);
// a response is decoded in its own cycle, as the bridge's rdata reaches each register in one stage
wire [15:0] new_gap = c_rdata[15:0] - tl;
wire [16:0] rlen    = {1'b0, c_rdata[15:0]} + 17'd1;
wire        c_new   = c_done && c_ok;
wire        c_bad   = c_done && !c_ok;
wire [16:0] ravail  = (ring == RG_SEC) ? {3'd0, avail_s} : (ring == RG_FIO) ? {3'd0, avail_f} :
                      (ring == RG_MNT) ? {6'd0, avail_m} : {5'd0, avail_o};
wire [15:0] rwords  = (ring == RG_SEC) ? SEC_WORDS : (ring == RG_FIO) ? FIO_WORDS : (ring == RG_MNT) ? MNT_WORDS :
                      OSD_WORDS;
// an LFRAME the player takes: kind 0x30, arg {odd, 0, sel} with sel in the mask, a len of at least one word; with
// the capture lane also arg {odd, tag, 1, sel} with a len of at most 1,020
`ifdef MISTER_OCRAM_LANE
wire        f_cap   = c_rdata[18];
wire        f_arg   = f_cap ? c_rdata[15:0] <= 16'd1020 : c_rdata[22:19] == 4'd0;
`else
wire        f_cap   = 1'b0;
wire        f_arg   = c_rdata[22:18] == 5'd0;
`endif
wire        f_rec   = c_rdata[31:24] == 8'h30 && f_arg && FIO_SELS[c_rdata[17:16]] && c_rdata[15:0] != 16'd0;
// a mount FRAME that answers the read in hand: its slot, blk_cnt and len
wire        m_rec   = c_rdata[31:24] == 8'h10 &&
                      ((c_rdata[17:16] == 2'd1 && mst[0] && !mwr[0] && c_rdata[23:18] == mr1[38:33] &&
                        c_rdata[15:0] == {5'd0, mn1} + 16'd1) ||
                       (c_rdata[17:16] == 2'd2 && mst[1] && !mwr[1] && c_rdata[23:18] == mr2[38:33] &&
                        c_rdata[15:0] == {5'd0, mn2} + 16'd1));

wire        stand  = rq_q[32];
wire        walk_o = OSD_EN != 0 && s_live && f_open && bind_en && !stop_o;
wire        walk_s = SEC_EN != 0 && s_live && f_open && !stop_s;
wire        walk_f = FIO_EN != 0 && s_live && f_open && !stop_f;
wire        walk_m = MNT_EN != 0 && s_live && f_open && !stop_m;
wire        want_s = walk_s && sec_ask[0] && !evt_stop;
wire  [1:0] want_m = MNT_EN & sec_ask[2:1] & {2{walk_m && !evt_stop}};
wire        m_wjob = wj != 2'd0 && !mq_v && !evt_stop;
wire        job_o  = walk_o && (avail_o != 12'd0 || due_o);
wire        job_s  = walk_s && sb && !drop_s && ((avail_s != 14'd0 && stand && !s_wait) || (avail_s == 14'd0 && due_s));
`ifdef MISTER_OCRAM_LANE
// a capture frame left for want of room holds ring 1 until the EVT ring has it
reg         cap_held = 1'b0;
reg  [10:0] cap_n_q = 11'd0;
reg  [31:0] cap_hdr_q = 32'd0;
reg         cap_q = 1'b0;              // the frame being played is a capture frame
reg         p_have = 1'b0;             // p_wd holds a reply word for the master
wire        cap_room = {5'd0, cap_n_q} + 16'd1 <= e_free && !evt_stop;
wire        job_f  = walk_f && (avail_f != 14'd0 || due_f) && !(cap_held && !cap_room);
assign p_post = wst == W_CAP;
assign p_arg  = cap_hdr_q[23:16];
assign p_n    = cap_n_q;
assign p_wv   = p_have;
assign p_wd   = done_q[33:2];
`else
wire        job_f  = walk_f && (avail_f != 14'd0 || due_f);
assign p_post = 1'b0;
assign p_arg  = 8'd0;
assign p_n    = 11'd0;
assign p_wv   = 1'b0;
assign p_wd   = 32'd0;
`endif
wire        job_m  = walk_m && (m_wjob || avail_m != 11'd0 || due_m);
wire        streaming = ring == RG_SEC && wst != W_IDLE;
wire        mstream   = ring == RG_MNT && wst != W_IDLE;

// Round robin by ring id over the rings with work, the one served last going last
function [2:0] rr_pick(input [3:0] jobs, input [1:0] last);   // {found, id}
	integer i;
	reg [1:0] c;
	begin
		rr_pick = 3'd0;
		for (i = 4; i >= 1; i = i - 1) begin
			c = last + i[1:0];
			if (jobs[c]) rr_pick = {1'b1, c};
		end
	end
endfunction
wire  [2:0] pick   = rr_pick({job_m, job_f, job_s, job_o}, turn);

assign c_req    = req;
assign c_we     = req_we;
assign c_off    = req_off;
assign c_wdata  = {16'd0, widx};          // a tail store's value: k and the tail hold until its done
assign c_fault  = flt_o;
assign c_mid    = mid_o;
assign c_sfault = flt_s;
assign c_smid   = mid_s;
assign c_ffault = flt_f;
assign c_fmid   = mid_f;
assign c_mfault = flt_m;
assign c_mmid   = mid_m;
assign f_tail   = tail_f;
assign c_open   = f_open & bind_en;
assign c_rest   = f_idle;
assign c_sbind  = {sbm, sb};
assign rq_lba   = rq_q[31:0];
// not in a record's landing cycle: its fresh read is the next one
assign rq_post  = SEC_EN != 0 && sb && !drop_s && bind_lwx[0] && stand && !rq_posted && fresh && !rq_v &&
                  avail_s == 14'd0 && !stop_s;
// a mount read is posted at once; a WRITE once the feeder has raised its ack
assign m_post   = MNT_EN != 0 && !mq_v &&
                  (mw_job ? (w_acc && !w_started) : (rd_ok != 2'd0 && !stop_m));
assign m_write  = mw_job;
assign m_arg    = {pr[38:33], pk};
assign m_n      = pk2 ? mn2 : mn1;
assign m_lba    = pr[31:0];
assign m_wv     = m_have;
assign m_wd     = done_q[33:2];         // the answer to the last token: held until the next one is launched

// A payload word, a frame's LBA and an LFRAME's header go to the cell in their read's response cycle (each read is
// issued only with the cell idle, and nothing else launches before its response); a ROW's head, its row alone,
// from row_q; a WRITE's head, its slot and LBA, and each token, from the walker's state.
wire   f_hgo = wst == W_HDR && ring == RG_FIO && c_new && f_rec && !f_cap && rlen <= ravail;
wire   x_rsp = ((wst == W_PAY || wst == W_LBA) && c_new || f_hgo) && s_live;
wire   x_tok = wst == W_MTK && !fwd_busy && s_live;
assign x_go = x_rsp || (wst == W_HGO && !fwd_busy && s_live) || x_tok;
assign x_d  = (wst == W_PAY) ? {TG_DATA, 3'd0, c_rdata} :
              (wst == W_LBA) ? {TG_SHEAD, 1'b0, (ring == RG_MNT) ? mfk : 2'd0, c_rdata} :
              (wst == W_HDR) ? {TG_FHEAD, 3'd0, c_rdata} :
              (wst == W_MTK) ? {TG_DATA, 3'd0, 31'd0, m_end} :
`ifdef MISTER_OCRAM_LANE
              (ring == RG_FIO) ? {TG_FHEAD, 3'd0, cap_hdr_q} :     // a capture frame's header, once its REPLY is open
`endif
              (ring == RG_MNT) ? {TG_SHEAD, 1'b1, mfk, (mfk == 2'd2) ? mr2[31:0] : mr1[31:0]} :
              {TG_OHEAD, 3'd0, 11'd0, row_q, 16'd0};

always @(posedge clk) begin
	flt_o <= 1'b0;
	flt_s <= 1'b0;
	flt_f <= 1'b0;
	flt_m <= 1'b0;
	if (c_grant) req <= 1'b0;

	// the bind level: up while asked; it falls only between records
	if (want_s && !drop_s) bind_lwx[0] <= 1'b1;
	else if (!(streaming && s_live)) bind_lwx[0] <= 1'b0;
	if (bind_lwx[0] && en_b[0] && !drop_s) sb <= 1'b1;
	if (sb && !want_s && !drop_s && !(streaming && s_live)) begin
		drop_s  <= 1'b1;
		fl_pend <= !stop_s;
	end
	if (drop_s && !fl_pend && !en_b[0]) begin
		drop_s <= 1'b0;
		sb     <= 1'b0;
	end
	// the mount binds: as the disc slot's, with no flush
	if (want_m[0] && !drop_m[0]) bind_lwx[1] <= 1'b1;
	else if (!(mstream && s_live)) bind_lwx[1] <= 1'b0;
	if (want_m[1] && !drop_m[1]) bind_lwx[2] <= 1'b1;
	else if (!(mstream && s_live)) bind_lwx[2] <= 1'b0;
	if (bind_lwx[1] && en_b[1] && !drop_m[0]) sbm[0] <= 1'b1;
	if (bind_lwx[2] && en_b[2] && !drop_m[1]) sbm[1] <= 1'b1;
	if (sbm[0] && !want_m[0] && !drop_m[0] && !(mstream && s_live)) drop_m[0] <= 1'b1;
	if (sbm[1] && !want_m[1] && !drop_m[1] && !(mstream && s_live)) drop_m[1] <= 1'b1;
	if (drop_m[0] && !en_b[1]) begin drop_m[0] <= 1'b0; sbm[0] <= 1'b0; end
	if (drop_m[1] && !en_b[2]) begin drop_m[1] <= 1'b0; sbm[1] <= 1'b0; end

	// the request record: a new one asks for one fresh head read, and voids the post
	if (rq_v) begin
		rq_posted <= 1'b0;
		fresh     <= 1'b0;
		s_wait    <= 1'b0;
		if (rq_q[32]) due_s <= 1'b1;
	end
	else if (rq_start) rq_posted <= 1'b1;
	// the mount record: a new one voids the post and the WRITE's end
	if (mq_v && mq[41:40] == 2'd1) begin mr1 <= mq[39:0]; m_posted[0] <= 1'b0; w_done[0] <= 1'b0; end
	if (mq_v && mq[41:40] == 2'd2) begin mr2 <= mq[39:0]; m_posted[1] <= 1'b0; w_done[1] <= 1'b0; end
	if (m_start) m_posted[pk2] <= 1'b1;
	if (m_start && mw_job) w_started <= 1'b1;   // once: the slot's next record may land before the ack's fall
	if (s_tick) begin due_o <= 1'b1; due_s <= 1'b1; due_f <= 1'b1; due_m <= 1'b1; s_wait <= 1'b0; end

	// a ring word read, addressed by the one adder; never while the forward cell is busy
	if (rd_pend && !req && !fwd_busy) begin
		rd_pend <= 1'b0;
		req     <= 1'b1;
		req_we  <= 1'b0;
		case (ring)
		RG_SEC:  req_off <= sec_off;
		RG_FIO:  req_off <= fio_off;
		RG_MNT:  req_off <= mnt_off;
		default: req_off <= osd_off;
		endcase
	end
	if (st_pend && !req) begin
		st_pend   <= 1'b0;
		req       <= 1'b1;
		req_we    <= 1'b1;
		case (ring)
		RG_SEC:  req_off <= C_IDX4;
		RG_FIO:  req_off <= C_IDX1;
		RG_MNT:  req_off <= C_IDX0;
		default: req_off <= C_IDX2;
		endcase
	end

	if (s_hello) begin
		wst <= W_IDLE; req <= 1'b0; rd_pend <= 1'b0; st_pend <= 1'b0; word_out <= 1'b0;
		tail_o <= 16'd0; tail_s <= 16'd0; tail_f <= 16'd0; tail_m <= 16'd0;
		avail_o <= 12'd0; avail_s <= 14'd0; avail_f <= 14'd0; avail_m <= 11'd0;
		due_o <= 1'b1; due_s <= 1'b1; due_f <= 1'b1; due_m <= 1'b1;
		stop_o <= 1'b0; stop_s <= 1'b0; stop_f <= 1'b0; stop_m <= 1'b0;
		mid_o <= 1'b0; mid_s <= 1'b0; mid_f <= 1'b0; mid_m <= 1'b0; s_wait <= 1'b0; turn <= RG_OSD;
		sb <= 1'b0; drop_s <= 1'b0; fl_pend <= 1'b0; bind_lwx <= 3'd0; fresh <= 1'b0; rq_posted <= 1'b0;
		sbm <= 2'd0; drop_m <= 2'd0; m_posted <= 2'd0; mw_job <= 1'b0; w_acc <= 1'b0; w_done <= 2'd0;
		m_end <= 1'b0; m_have <= 1'b0;
`ifdef MISTER_OCRAM_LANE
		cap_held <= 1'b0; cap_q <= 1'b0; p_have <= 1'b0;
`endif
`ifdef MISTER_OCRAM_DIAG
		rows_q <= 16'd0; deepest_q <= 16'd0;
`endif
	end
	else if (!s_live && wst != W_IDLE) begin
		// the session ended: the record is abandoned, its mid bit left for the master
		wst <= W_IDLE; req <= 1'b0; rd_pend <= 1'b0; st_pend <= 1'b0; word_out <= 1'b0;
		mw_job <= 1'b0; w_acc <= 1'b0; m_have <= 1'b0;
`ifdef MISTER_OCRAM_LANE
		cap_q <= 1'b0; p_have <= 1'b0;
`endif
	end
	else if (!s_live) begin
		sb <= 1'b0; drop_s <= 1'b0; fl_pend <= 1'b0; sbm <= 2'd0; drop_m <= 2'd0;
	end
	else case (wst)
	W_IDLE: begin
`ifdef MISTER_OCRAM_LANE
		cap_q <= 1'b0;
`endif
		mid_o <= 1'b0;
		mid_s <= 1'b0;
		mid_f <= 1'b0;
		mid_m <= 1'b0;
		if (SEC_EN != 0 && fl_pend) begin
			ring <= RG_SEC; req <= 1'b1; req_we <= 1'b0; req_off <= H_IDX4; wst <= W_FLUSH;
		end
		else if (pick[2]) begin
			ring <= pick[1:0]; turn <= pick[1:0];
			case (pick[1:0])
			RG_SEC: if (avail_s == 14'd0) begin
				req <= 1'b1; req_we <= 1'b0; req_off <= H_IDX4; due_s <= 1'b0; wst <= W_HEAD;
			end
			else begin k <= 14'd0; rd_pend <= 1'b1; mid_s <= 1'b1; wst <= W_HDR; end
			RG_FIO: if (avail_f == 14'd0) begin
				req <= 1'b1; req_we <= 1'b0; req_off <= H_IDX1; due_f <= 1'b0; wst <= W_HEAD;
			end
			else begin k <= 14'd0; rd_pend <= 1'b1; mid_f <= 1'b1; wst <= W_HDR; end
			RG_MNT: if (m_wjob) begin
				// a WRITE: its head crosses with the slot and the LBA; the feeder answers
				mw_job <= 1'b1; mfk <= wj[0] ? 2'd1 : 2'd2; w_started <= 1'b0; m_end <= 1'b0; word_out <= 1'b1;
				wst <= W_HGO;
			end
			else if (avail_m == 11'd0) begin
				req <= 1'b1; req_we <= 1'b0; req_off <= H_IDX0; due_m <= 1'b0; wst <= W_HEAD;
			end
			else begin k <= 14'd0; rd_pend <= 1'b1; mid_m <= 1'b1; wst <= W_HDR; end
			default: if (avail_o == 12'd0) begin
				req <= 1'b1; req_we <= 1'b0; req_off <= H_IDX2; due_o <= 1'b0; wst <= W_HEAD;
			end
			else begin k <= 14'd0; rd_pend <= 1'b1; mid_o <= 1'b1; wst <= W_HDR; end
			endcase
		end
	end
	W_HEAD, W_FLUSH: if (c_done) begin
		if (!c_ok) begin req <= 1'b1; req_we <= 1'b0; end            // no value: read it again
		else if (new_gap > rwords) begin
			// a head more than the ring's size ahead: a host defect
			case (ring)
			RG_SEC:  begin stop_s <= 1'b1; flt_s <= 1'b1; fl_pend <= 1'b0; end
			RG_FIO:  begin stop_f <= 1'b1; flt_f <= 1'b1; end
			RG_MNT:  begin stop_m <= 1'b1; flt_m <= 1'b1; end
			default: begin stop_o <= 1'b1; flt_o <= 1'b1; end
			endcase
			wst <= W_IDLE;
		end
		else if (wst == W_FLUSH) begin
			// the unbind: every queued frame is discarded by one tail store
			k <= new_gap[13:0]; st_pend <= 1'b1; wst <= W_FTAIL;
		end
		else begin
			case (ring)
			RG_SEC:  begin avail_s <= new_gap[13:0]; fresh <= 1'b1; end
			RG_FIO:  avail_f <= new_gap[13:0];
			RG_MNT:  avail_m <= new_gap[10:0];
			default: avail_o <= new_gap[11:0];
			endcase
`ifdef MISTER_OCRAM_DIAG
			if (ring == RG_OSD && new_gap > deepest_q) deepest_q <= new_gap;
`endif
			wst <= W_IDLE;
		end
	end
	W_HDR: if (c_done) begin
		if (!c_ok) rd_pend <= 1'b1;
		else if (rlen > ravail) begin
			// a length past the head: a host defect
			case (ring)
			RG_SEC:  begin stop_s <= 1'b1; flt_s <= 1'b1; mid_s <= 1'b0; end
			RG_FIO:  begin stop_f <= 1'b1; flt_f <= 1'b1; mid_f <= 1'b0; end
			RG_MNT:  begin stop_m <= 1'b1; flt_m <= 1'b1; mid_m <= 1'b0; end
			default: begin stop_o <= 1'b1; flt_o <= 1'b1; mid_o <= 1'b0; end
			endcase
			wst <= W_IDLE;
		end
		else if (ring == RG_OSD && c_rdata[31:24] == 8'h01 && c_rdata[15:0] == 16'd64 &&
		         c_rdata[23:16] <= LAST_ROW) begin
			row_q <= c_rdata[20:16]; k <= 14'd0; word_out <= 1'b1; wst <= W_HGO;
`ifdef MISTER_OCRAM_DIAG
			was_row <= 1'b1;
`endif
		end
		else if (ring == RG_SEC && c_rdata[31:24] == 8'h10 && c_rdata[23:16] == 8'd0 && c_rdata[15:0] == SEC_LEN) begin
			k <= 14'd1; rd_pend <= 1'b1; wst <= W_LBA;
		end
`ifdef MISTER_OCRAM_LANE
		else if (ring == RG_FIO && f_rec && f_cap) begin
			// a capture frame: held for its REPLY while the EVT ring has room, else left where it is
			cap_hdr_q <= c_rdata; cap_n_q <= c_rdata[10:0]; k <= 14'd0;
			if ({5'd0, c_rdata[10:0]} + 16'd1 <= e_free && !evt_stop) begin cap_held <= 1'b0; wst <= W_CAP; end
			else begin cap_held <= 1'b1; mid_f <= 1'b0; wst <= W_IDLE; end
		end
`endif
		else if (ring == RG_FIO && f_rec) begin
			flen_q <= c_rdata[13:0]; k <= 14'd0; word_out <= 1'b1; wst <= W_XFER;   // x_go launched FIO_HEAD
		end
		else if (ring == RG_MNT && m_rec) begin
			flen_q <= c_rdata[13:0]; mfk <= c_rdata[17:16]; k <= 14'd1; rd_pend <= 1'b1; wst <= W_LBA;
		end
		else begin
			// an unknown kind, a record of another length or argument: skipped by its length
			k <= rlen[13:0]; st_pend <= 1'b1; wst <= W_TAIL;
`ifdef MISTER_OCRAM_DIAG
			was_row <= 1'b0;
`endif
		end
	end
	W_LBA: if (c_done) begin
		if (!c_ok) rd_pend <= 1'b1;
		else begin word_out <= 1'b1; wst <= W_ANS; end               // x_go launched SEC_HEAD with the LBA
	end
	W_HGO: if (!fwd_busy) wst <= (ring == RG_MNT) ? W_ANS : W_XFER;   // x_go launches the ROW's or WRITE's head
	W_ANS: if (done_v) begin
		word_out <= 1'b0;
		if (mw_job) begin
			// the WRITE's head: accepted, its ack is up and its words follow; else it is over for this record
			if (done_c == A_ACCEPT) begin w_acc <= 1'b1; k <= 14'd0; wst <= W_MTK; end
			else begin mw_job <= 1'b0; w_done[mfk == 2'd2] <= 1'b1; wst <= W_IDLE; end
		end
		else if (done_c == A_ACCEPT) begin k <= 14'd2; rd_pend <= 1'b1; wst <= W_PAY; end
		else if (done_c == A_MISMATCH || ring == RG_MNT) begin
			k <= ((ring == RG_MNT) ? flen_q : SEC_K) + 14'd1; st_pend <= 1'b1; wst <= W_TAIL;
		end
		else begin s_wait <= 1'b1; wst <= W_IDLE; end                 // no request: the frame stays
	end
	W_PAY: if (c_done) begin
		if (!c_ok) rd_pend <= 1'b1;                                   // read it again
		else begin word_out <= 1'b1; wst <= W_XFER; end              // x_go launched it
	end
`ifdef MISTER_OCRAM_LANE
	W_CAP: if (p_start) begin
		// the REPLY's header is written: the frame's header crosses, then its words as a download frame's
		flen_q <= {3'd0, cap_n_q}; cap_q <= 1'b1; word_out <= 1'b1; wst <= W_HGO;
	end
	else if (!cap_room) begin cap_held <= 1'b1; mid_f <= 1'b0; wst <= W_IDLE; end
	W_XFER: if (p_have) begin
		// a reply word in p_wd: the next ring word waits until the master has it (or the EVT ring has stopped)
		if (p_wtake || evt_stop) begin
			p_have <= 1'b0;
			if (k == flen_q) begin k <= k + 14'd1; st_pend <= 1'b1; wst <= W_TAIL; end
			else begin k <= k + 14'd1; rd_pend <= 1'b1; wst <= W_PAY; end
		end
	end
	else if (done_v) begin
		word_out <= 1'b0;
		if (cap_q && ring == RG_FIO && k != 14'd0 && !evt_stop) p_have <= 1'b1;   // a ring word's two replies
		else if (k == ((ring == RG_SEC) ? SEC_K : (ring == RG_OSD) ? 14'd64 : flen_q)) begin
			k <= k + 14'd1; st_pend <= 1'b1; wst <= W_TAIL;
		end
		else begin k <= k + 14'd1; rd_pend <= 1'b1; wst <= W_PAY; end
	end
`else
	W_XFER: if (done_v) begin
		word_out <= 1'b0;
		if (k == ((ring == RG_SEC) ? SEC_K : (ring == RG_OSD) ? 14'd64 : flen_q)) begin
			k <= k + 14'd1; st_pend <= 1'b1; wst <= W_TAIL;
		end
		else begin k <= k + 14'd1; rd_pend <= 1'b1; wst <= W_PAY; end
	end
`endif
	W_MTK: if (!fwd_busy) begin word_out <= 1'b1; wst <= W_MWD; end   // x_go launches the token
	W_MWD: if (done_v) begin
		word_out <= 1'b0;
		if (m_end) begin mw_job <= 1'b0; w_acc <= 1'b0; w_done[mfk == 2'd2] <= 1'b1; wst <= W_IDLE; end   // ack down
		else if (evt_stop) begin m_end <= 1'b1; wst <= W_MTK; end
		else begin m_have <= 1'b1; wst <= W_MWH; end
	end
	W_MWH: if (m_wtake) begin
		m_have <= 1'b0;
		k      <= k + 14'd1;
		wst    <= (k + 14'd1 == {3'd0, mn}) ? W_MWP : W_MTK;
	end
	else if (evt_stop) begin m_have <= 1'b0; m_end <= 1'b1; wst <= W_MTK; end
	W_MWP: if (m_pub || evt_stop) begin m_end <= 1'b1; wst <= W_MTK; end   // published, or never: the ack falls
	W_TAIL, W_FTAIL: begin
		if (c_new) begin
			// the tail store is done: the record is consumed
			wst <= W_IDLE;
			case (ring)
			RG_SEC: begin
				tail_s <= widx; avail_s <= (wst == W_FTAIL) ? 14'd0 : avail_s - k; mid_s <= 1'b0; fresh <= 1'b0;
				if (avail_s == k || wst == W_FTAIL) due_s <= 1'b1;
				if (wst == W_FTAIL) fl_pend <= 1'b0;
			end
			RG_FIO: begin
				tail_f <= widx; avail_f <= avail_f - k; mid_f <= 1'b0;
				if (avail_f == k) due_f <= 1'b1;
			end
			RG_MNT: begin
				tail_m <= widx; avail_m <= avail_m - k[10:0]; mid_m <= 1'b0;
				if ({3'd0, avail_m} == k) due_m <= 1'b1;
			end
			default: begin
				tail_o <= widx; avail_o <= avail_o - k[11:0]; mid_o <= 1'b0;
				if (avail_o == k[11:0]) due_o <= 1'b1;
`ifdef MISTER_OCRAM_DIAG
				if (was_row) rows_q <= rows_q + 16'd1;
`endif
			end
			endcase
		end
		else if (c_bad) req <= 1'b1;                               // a failed store is tried again, same value
	end
	default: wst <= W_IDLE;
	endcase
end

///////////////////////////////  clk_sys  ///////////////////////////////////

reg   [1:0] cur = RG_OSD;              // the ring of the last head
wire  [1:0] tag = word_q[36:35];
/* verilator lint_off UNUSEDSIGNAL */   // the feeder's, with SEC_EN
wire        fwr = word_q[34];              // a SEC_HEAD's direction and slot index (MNT_EN)
wire  [1:0] fk  = word_q[33:32];
/* verilator lint_on UNUSEDSIGNAL */
wire [31:0] wd  = word_q[31:0];
wire        o_word_v = OSD_EN != 0 && word_v && (tag == TG_OHEAD || (tag == TG_DATA && cur == RG_OSD));
/* verilator lint_off UNUSEDSIGNAL */   // the feeder's, with SEC_EN; the player's, with FIO_EN
wire        s_word_v = SEC_EN != 0 && word_v && (tag == TG_SHEAD || (tag == TG_DATA && cur == RG_SEC));
wire        f_word_v = FIO_EN != 0 && word_v && (tag == TG_FHEAD || (tag == TG_DATA && cur == RG_FIO));
/* verilator lint_on UNUSEDSIGNAL */

// the OSD writer: the word waits in the capture register until its four bytes are written
reg         o_have = 1'b0;             // a DATA word to write
reg         o_hans = 1'b0;             // a header to answer
reg   [4:0] o_row = 5'd0;
reg   [5:0] o_word = 6'd0;
reg   [1:0] o_b = 2'd0;
reg   [1:0] qn = 2'd0;
wire        quiet = (sel == 2'b00) && view;
wire        wnow  = hen_s && !frz && o_have && quiet && qn == 2'd2;
assign we    = wnow;
assign addr  = {o_row, o_word, o_b};
assign dbyte = wd[{o_b, 3'd0} +: 8];

// the SECTOR feeder, for the disc slot and the mount slots
wire        f_go, f_rest, f_ack;
wire  [1:0] f_code;
wire [31:0] f_dat;
generate
if (SEC_EN != 0) begin : sec
	hps_ocram_sfeed #(.FRAME(FRAME), .GAP(GAP), .MNT_EN(MNT_EN), .BLKSZ(BLKSZ)) feeder
	(
		.clk_sys(clk_sys), .hen_s(hen_s), .bind_s(bind_s),
		.w_v(s_word_v), .w_head(tag == TG_SHEAD), .w_wr(fwr), .w_k(fk), .w_d(wd),
		.a_go(f_go), .a_code(f_code), .a_d(f_dat), .rest(f_rest),
		.c_reset(x_reset), .hps_ack(x_hps_ack), .c_req(x_req), .c_lba(x_lba), .c_commit(x_commit), .c_wide(x_wide),
		.m_hps_ack(xm_hps_ack), .m_rd(xm_rd), .m_wr(xm_wr), .m_lba(xm_lba), .m_cnt(xm_cnt),
		.en_sys(x_en), .m_en(xm_en), .o_ack(f_ack), .m_ack(xm_ack), .o_wr(x_wr), .o_dat(x_dat), .o_adr(x_adr)
	);
end
else begin : nosec
	assign f_go = 1'b0, f_code = 2'd0, f_dat = 32'd0, f_rest = 1'b1, f_ack = 1'b0;
	assign x_en = 1'b0, x_wr = 1'b0, x_dat = 16'd0, x_adr = 14'd0, xm_en = 2'd0, xm_ack = 2'd0;
end
endgenerate
assign x_ack = f_ack;

// the download player
wire        p_go, p_rest;
/* verilator lint_off UNUSEDSIGNAL */   // without MISTER_OCRAM_LANE the replies are not read
wire [31:0] p_dat;
/* verilator lint_on UNUSEDSIGNAL */
generate
if (FIO_EN != 0) begin : fio
	hps_ocram_fplay #(.GAP(FIO_GAP[2:0])) player
	(
		.clk_sys(clk_sys), .hen_s(hen_s),
		.w_v(f_word_v), .w_head(tag == TG_FHEAD), .w_d(wd),
		.a_go(p_go), .a_d(p_dat), .rest(p_rest), .l_dout(l_dout),
		.l_busy(l_busy), .l_wait(l_wait),
		.own(l_own), .hold(l_hold), .sel(l_sel), .strobe(l_strobe), .din(l_din)
	);
end
else begin : nofio
	assign p_go = 1'b0, p_rest = 1'b1, p_dat = 32'd0;
	assign l_own = 1'b0, l_hold = 1'b0, l_sel = 4'd0, l_strobe = 1'b0, l_din = 16'd0;
end
endgenerate

// the request record: offered = the mux here, the core's request raised, the ack low and two cycles past its fall
reg         dirty = 1'b0;
reg   [1:0] rearm = 2'd0;
wire        off_d = SEC_EN != 0 && x_en && x_req && !f_ack && rearm == 2'd0;
assign rc_go = dirty && !rc_busy;

// the mount channel's records, one per slot, launched in turn when changed; offered = the slot's mux here, its
// request raised, a
// block of at most MFRAME words, its ack low and two cycles past its fall. The other slot takes the channel only
// once the record in hand, sent, reads not offered; no record launches while a mount slot's ack is up.
reg   [1:0] moffd = 2'd0;
reg   [1:0] mdirty = 2'd0;
reg         mlast = 1'b0;              // the slot launched last: 0 is k = 1, 1 is k = 2
reg   [3:0] mrearm = 4'd0;
function automatic [18:0] mwords(input [5:0] cnt);
	mwords = ({13'd0, cnt} + 19'd1) << ({1'b0, BLKSZ} + 4'd5);
endfunction
wire  [1:0] mfit  = {mwords(xm_cnt[11:6]) <= MFRAME[18:0], mwords(xm_cnt[5:0]) <= MFRAME[18:0]};
wire  [1:0] moff  = MNT_EN & xm_en & (xm_rd | xm_wr) & ~xm_ack & {mrearm[3:2] == 2'd0, mrearm[1:0] == 2'd0} & mfit;
wire        mrs   = (mdirty == 2'b11) ? ~mlast : mdirty[1];   // the slot to launch
assign mrec   = mrs ? {2'd2, xm_wr[1], xm_cnt[11:6], moffd[1], xm_lba[63:32]} :
                      {2'd1, xm_wr[0], xm_cnt[5:0], moffd[0], xm_lba[31:0]};
assign mrc_go = mdirty != 2'd0 && !mrc_busy && xm_ack == 2'd0;

// The half is at rest with the enable and the bind levels low, no word held or arriving, nothing owed, every
// handshake settled. While frozen it is never at rest, which is what a stopped clk_sys looks like to the link.
assign rest = !frz && !hen_s && bind_s == 3'd0 && f_rest && p_rest && !o_have && !o_hans && !back_busy && !word_v &&
              !done_go && !offd && !dirty && !rc_busy && moffd == 2'd0 && mdirty == 2'd0 && !mrc_busy;

always @(posedge clk_sys) begin
	done_go  <= 1'b0;
	en_lwx   <= {xm_en, x_en};
	wide_lwx <= x_wide;
	qn <= !quiet ? 2'd0 : (qn == 2'd2) ? 2'd2 : qn + 2'd1;

	// a change of offered in a launch cycle stays dirty for the next record
	offd  <= off_d;
	dirty <= (off_d ^ offd) | (dirty & ~rc_go);
	rearm <= {rearm[0], f_ack | x_hps_ack};
	moffd  <= moff;
	mdirty <= (moff ^ moffd) | (mdirty & ~({mrs, ~mrs} & {2{mrc_go}}));
	if (mrc_go) mlast <= mrs;
	mrearm <= {mrearm[2], xm_ack[1] | xm_hps_ack[1], mrearm[0], xm_ack[0] | xm_hps_ack[0]};

	if (word_v && tag != TG_DATA) cur <= (tag == TG_SHEAD) ? RG_SEC : (tag == TG_FHEAD) ? RG_FIO : RG_OSD;
	if (o_word_v) begin
		if (tag == TG_OHEAD) begin o_row <= wd[20:16]; o_word <= 6'd0; o_hans <= 1'b1; end
		else begin o_have <= 1'b1; o_b <= 2'd0; end
	end

	// one answer a cycle: a word is outstanding alone, so the two never meet
	if (f_go) begin
		done_go   <= 1'b1;
		done_code <= f_code;
		done_dat  <= f_dat;
	end
	else if (p_go) begin
		done_go   <= 1'b1;
		done_code <= A_TOOK;
`ifdef MISTER_OCRAM_LANE
		done_dat  <= p_dat;                 // the ring word's two link replies
`endif
	end
	else if (o_hans && !frz) begin
		o_hans    <= 1'b0;
		done_go   <= 1'b1;
		done_code <= A_TOOK;
	end
	else if (o_have && !frz && !hen_s) begin
		// with the enable low the word is discarded and still answered, so both halves converge
		o_have    <= 1'b0;
		done_go   <= 1'b1;
		done_code <= A_TOOK;
	end
	else if (wnow) begin
		o_b <= o_b + 2'd1;
		if (o_b == 2'd3) begin
			o_have    <= 1'b0;
			o_word    <= o_word + 6'd1;
			done_go   <= 1'b1;
			done_code <= A_TOOK;
		end
	end
end

endmodule

//============================================================================
//  hps_ocram_sfeed -- the SECTOR feeder of hps_ocram_rx, on clk_sys (MISTER_OCRAM_SECTOR_SLOT)
//
//  lw_sector's feeder with the forward crossing's capture register as its one-entry queue: w_v marks an entry (a
//  SEC_HEAD with its slot index and LBA, or a DATA word) held in w_d until it is answered. Strobes as hps_io's: with
//  c_wide two 16-bit strobes a word at the halfword index, low half first, else four bytes at the byte index;
//  GAP cycles apart, the first GAP after the ack. A word is popped only once its first strobe can issue in the cycle
//  after (GAP - 1 after the previous strobe at the earliest), so a word in hand is strobed as lw_sector strobes a
//  FIFO entry. A word's answer goes with its last strobe; a SEC_HEAD's with its decision. The disc slot (k = 0)
//  plays FRAME words; a mount slot (k = 1, 2, MNT_EN) plays its request's (blk_cnt + 1) << (BLKSZ + 5). A mount
//  slot's WRITE head (w_wr) raises its ack and reads the core's buffer word by word at the same addresses, each
//  lane GAP cycles after its address; each DATA token is answered with the next word (a_d), and the end token
//  (d[0] = 1) drops the ack. A module of its own so the trace bench can drive it beside lw_sector and the fit
//  reports its ALMs apart.
//============================================================================

/* verilator lint_off DECLFILENAME */   // the feeder shares hps_ocram_rx's file, as the sink shares the master's
module hps_ocram_sfeed #(parameter [31:0] FRAME = 32'd128, parameter [31:0] GAP = 32'd2,
                         parameter [1:0] MNT_EN = 2'd0, parameter [2:0] BLKSZ = 3'd2)
(
	input  wire        clk_sys,
	input  wire        hen_s,        // the session's enable level
	input  wire  [2:0] bind_s,       // the bind levels, by slot index k
	input  wire        w_v,          // one cycle: an entry is in w_d
	input  wire        w_head,       // it is a SEC_HEAD
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN a head is the disc slot's read
	input  wire        w_wr,         // a head's direction: a mount slot's WRITE
	input  wire  [1:0] w_k,          // a head's slot index
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [31:0] w_d,          // held until the entry is answered
	output reg         a_go = 1'b0,  // one cycle: the entry's answer
	output reg   [1:0] a_code = 2'd0,// 0 took, 1 accept, 2 mismatch, 3 no request
	output reg  [31:0] a_d = 32'd0,  // a WRITE token's answer: the core's buffer word
	output wire        rest,         // idle: no entry, no frame, every mux on SPI

	input  wire        c_reset,
	input  wire        hps_ack,
	input  wire        c_req,
	input  wire [31:0] c_lba,
	input  wire        c_commit,
	input  wire        c_wide,
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the mount slots' inputs are not read
	input  wire  [1:0] m_hps_ack,    // the mount slots', {k = 2, k = 1}: the SPI ack,
	input  wire  [1:0] m_rd,         //   the core's requests,
	input  wire  [1:0] m_wr,
	input  wire [63:0] m_lba,        //   the LBA, or while the slot's ack is up its buffer word,
	input  wire [11:0] m_cnt,        //   and blk_cnt
	/* verilator lint_on UNUSEDSIGNAL */
	output reg         en_sys = 1'b0,
	output reg   [1:0] m_en = 2'd0,
	output wire        o_ack,
	output wire  [1:0] m_ack,
	output reg         o_wr = 1'b0,
	output reg  [15:0] o_dat = 16'd0,
	output reg  [13:0] o_adr = 14'd0
);

localparam [1:0] A_TOOK = 2'd0, A_ACCEPT = 2'd1, A_MISMATCH = 2'd2, A_NOREQ = 2'd3;
localparam [1:0] F_IDLE = 2'd0, F_WRITE = 2'd1, F_STREAM = 2'd2, F_FILL = 2'd3;
localparam [12:0] FLAST = FRAME[12:0] - 13'd1;

reg   [1:0] fs = F_IDLE;
reg         have = 1'b0;               // an entry waits
reg         qv = 1'b0;                 // the entry popped last cycle
reg   [2:0] fack = 3'd0;               // the acks, by slot index
reg   [1:0] fsel = 2'd0;               // the slot of the open frame or WRITE
reg  [12:0] flast = FLAST;             // its last word
reg   [3:0] gc = 4'd0;
reg   [1:0] sn = 2'd0;
reg         em = 1'b0;
reg         fz = 1'b0;                 // the word being strobed is the fill's zero
reg  [12:0] fexp = 13'd0;
reg   [1:0] mt = 2'd0, ml = 2'd0;
reg         wrd = 1'b0;                // a WRITE's word is read whole
reg  [31:0] rword = 32'd0;             // its lanes

wire        fidle  = fs == F_IDLE;
wire        head   = w_head;
wire  [1:0] slast  = c_wide ? 2'd1 : 2'd3;
wire        gap_ok = gc >= GAP[3:0];
wire        nxt_ok = gc >= GAP[3:0] - 4'd1;
wire        wend   = em && sn >= slast && nxt_ok;
// an entry is popped in the cycle it arrives (w_v) or later, as lw_sector pops a FIFO entry the cycle it is visible
wire        wplay  = w_v && (head ? fidle : fs == F_STREAM);     // the entry just arrived and can be played
wire        hv     = have || wplay;
wire        pop    = hv && !qv && ((fidle && head) || (fs == F_STREAM && (!em || wend) && nxt_ok));
// the head's slot: the disc slot's request, or a declared mount slot's in the head's direction
wire        k1     = MNT_EN[0] && w_k == 2'd1;
wire        k2     = MNT_EN[1] && w_k == 2'd2;
wire        mup    = k1 ? (w_wr ? m_wr[0] : m_rd[0]) : k2 ? (w_wr ? m_wr[1] : m_rd[1]) : 1'b0;
wire        mlive  = (k1 && m_en[0] && (m_rd[0] | m_wr[0])) || (k2 && m_en[1] && (m_rd[1] | m_wr[1]));
wire        bmatch = (w_k == 2'd0) ? (!w_wr && en_sys && c_req && w_d == c_lba) :
                     (((k1 && m_en[0] && w_d == m_lba[31:0]) || (k2 && m_en[1] && w_d == m_lba[63:32])) && mup);
wire        live   = (w_k == 2'd0) ? (en_sys && c_req) : mlive;
/* verilator lint_off UNUSEDSIGNAL */   // a block of at most MFRAME (1,020) words
wire [18:0] mwn    = ({13'd0, k2 ? m_cnt[11:6] : m_cnt[5:0]} + 19'd1) << ({1'b0, BLKSZ} + 4'd5);
/* verilator lint_on UNUSEDSIGNAL */
// the frame in flight gets no more words: the disc slot's bind fell (a mount slot's is held while its record is
// walked), or the enable did; it is filled from its next word
wire        cut    = fs == F_STREAM && !have && !qv && !em &&
                     ((fsel == 2'd0 && !bind_s[0] && mt == 2'd2) || (!hen_s && ml == 2'd2));
wire [31:0] wsrc   = fz ? 32'd0 : w_d;
wire [15:0] din    = (fsel == 2'd2) ? m_lba[47:32] : m_lba[15:0];   // the WRITE's buffer word, through its mux
function automatic [13:0] sadr(input wide, input [12:0] w, input [1:0] j);
	sadr = wide ? {w, j[0]} : {w[11:0], j};
endfunction

assign o_ack = fack[0];
assign m_ack = fack[2:1] & MNT_EN;
assign rest  = fidle && !have && !qv && !em && fack == 3'd0 && !o_wr && !en_sys && m_en == 2'd0 && !a_go;

always @(posedge clk_sys) begin
	a_go <= 1'b0;
	o_wr <= 1'b0;

	// an entry: a word with no frame to play (idle, or a fill) is answered unplayed at once
	if (w_v) begin
		if (!head && (fidle || fs == F_FILL)) begin a_go <= 1'b1; a_code <= A_TOOK; end
		else if (!pop) have <= 1'b1;
	end
	else if (have && !head && fs == F_FILL) begin
		have <= 1'b0; a_go <= 1'b1; a_code <= A_TOOK;
	end

	qv <= pop;
	if (pop) have <= 1'b0;
	gc <= (gc == 4'd15) ? gc : gc + 4'd1;
	mt <= (have || bind_s[0]) ? 2'd0 : (mt == 2'd2) ? 2'd2 : mt + 2'd1;
	ml <= (have || hen_s) ? 2'd0 : (ml == 2'd2) ? 2'd2 : ml + 2'd1;

	// each slot's mux follows its bind level only while the feeder is off that slot and its SPI ack is low, and
	// moves here only while the commit mark reads 0
	if ((fidle || fsel != 2'd0) && !hps_ack) en_sys <= bind_s[0] & (en_sys | ~c_commit);
	if (MNT_EN[0] && (fidle || fsel != 2'd1) && !m_hps_ack[0]) m_en[0] <= bind_s[1] & (m_en[0] | ~c_commit);
	if (MNT_EN[1] && (fidle || fsel != 2'd2) && !m_hps_ack[1]) m_en[1] <= bind_s[2] & (m_en[1] | ~c_commit);

	if (fs == F_FILL && !em && gap_ok) begin
		o_wr  <= 1'b1;
		o_dat <= 16'd0;
		o_adr <= sadr(c_wide, fexp, 2'd0);
		fz    <= 1'b1;
		em    <= 1'b1;
		sn    <= 2'd0;
		gc    <= 4'd1;
	end

	if (fidle) fack <= 3'd0;

	if (qv) case (fs)
		F_IDLE: begin
			// a SEC_HEAD: accept the live request's LBA; a request for another one, or none, is answered so
			a_go <= 1'b1;
			if (hen_s && bmatch) begin
				a_code <= A_ACCEPT;
				fexp   <= 13'd0;
				fsel   <= w_k;
				flast  <= (w_k == 2'd0) ? FLAST : mwn[12:0] - 13'd1;
				fack[w_k] <= 1'b1;
				gc     <= 4'd1;
				if (w_wr) begin
					// a WRITE: its first word's lanes are read from the buffer, GAP cycles after each address
					o_adr <= sadr(c_wide, 13'd0, 2'd0);
					sn    <= 2'd0;
					wrd   <= 1'b0;
					fs    <= F_WRITE;
				end
				else fs <= F_STREAM;
			end
			else a_code <= (hen_s && live) ? A_MISMATCH : A_NOREQ;
		end
		F_STREAM:
			if (head) begin
				// a head while a frame streams (no walker sends one): fill the frame, answer no request
				fs <= F_FILL; a_go <= 1'b1; a_code <= A_NOREQ;
			end
			else begin
				o_wr  <= 1'b1;
				o_dat <= c_wide ? w_d[15:0] : {8'd0, w_d[7:0]};
				o_adr <= sadr(c_wide, fexp, 2'd0);
				fz    <= 1'b0;
				em    <= 1'b1;
				sn    <= 2'd0;
				gc    <= 4'd1;
			end
		default: ;
	endcase

	// a word's later strobes, GAP apart, its answer with the last; GAP - 1 after it the word ends
	if (em) begin
		if (sn < slast) begin
			if (gap_ok) begin
				o_wr  <= 1'b1;
				o_dat <= c_wide ? wsrc[31:16] : {8'd0, wsrc[{sn + 2'd1, 3'd0} +: 8]};
				o_adr <= sadr(c_wide, fexp, sn + 2'd1);
				sn    <= sn + 2'd1;
				gc    <= 4'd1;
				if (sn + 2'd1 == slast && !fz) begin a_go <= 1'b1; a_code <= A_TOOK; end
			end
		end
		else if (nxt_ok) begin
			em <= 1'b0;
			sn <= 2'd0;
			if (fexp >= flast) begin   // or past it: a frame ends from any count
				fexp <= 13'd0;
				fack <= 3'd0;
				fs   <= F_IDLE;
			end
			else fexp <= fexp + 13'd1;
		end
	end

	// a WRITE: each lane is read GAP cycles after its address; a token takes the word read, the end token drops
	// the ack; with the enable low for two cycles and no token in hand the WRITE ends unpublished
	if (fs == F_WRITE) begin
		if (!wrd && gap_ok) begin
			if (c_wide) begin
				if (sn[0]) rword[31:16] <= din;
				else       rword[15:0]  <= din;
			end
			else rword[{sn, 3'd0} +: 8] <= din[7:0];
			if (sn == slast) wrd <= 1'b1;
			else begin
				sn    <= sn + 2'd1;
				o_adr <= sadr(c_wide, fexp, sn + 2'd1);
				gc    <= 4'd1;
			end
		end
		if (have && head) begin have <= 1'b0; a_go <= 1'b1; a_code <= A_NOREQ; end
		else if (have && w_d[0]) begin
			have <= 1'b0; a_go <= 1'b1; a_code <= A_TOOK;
			fack <= 3'd0;
			fs   <= F_IDLE;
		end
		else if (have && wrd) begin
			have  <= 1'b0; a_go <= 1'b1; a_code <= A_TOOK; a_d <= rword;
			wrd   <= 1'b0;
			sn    <= 2'd0;
			fexp  <= fexp + 13'd1;
			o_adr <= sadr(c_wide, fexp + 13'd1, 2'd0);
			gc    <= 4'd1;
		end
		else if (!have && ml == 2'd2) begin
			fack <= 3'd0;
			fs   <= F_IDLE;
		end
	end

	if (cut) fs <= F_FILL;
	// the consumer's reset zero-fills the disc slot's frame; a mount slot's plays out whole
	if (c_reset && fs == F_STREAM && fsel == 2'd0) fs <= F_FILL;
end

endmodule
/* verilator lint_on DECLFILENAME */

//============================================================================
//  hps_ocram_fplay -- the download player of hps_ocram_rx, on clk_sys (MISTER_OCRAM_FIO)
//
//  lw_lane's player with the forward crossing's capture register as its one-entry queue: w_v marks an entry (an
//  LFRAME header, or a ring word of two link words, low half first) held in w_d until it is answered. A header is
//  answered at once and opens a frame of 2 * len - odd link words on its select. The frame starts once its first
//  word is in hand, both link selects have been low for two cycles, the link is not busy and the enable is up: own
//  and hold rise and the select with them. Each link word is one strobe, then GAP consecutive cycles with l_wait
//  low; the next word's strobe follows in the next cycle if its word is in hand. A ring word is answered with its
//  second strobe's last gap cycle, the frame's last word only when the frame has ended: the select drops, own two
//  cycles later, hold one after that. A frame whose next word does not come while the enable has been low for two
//  cycles ends there. A word with no frame open is answered unplayed. A module of its own so the trace bench can
//  drive it beside lw_lane and the fit reports its ALMs apart.
//============================================================================

/* verilator lint_off DECLFILENAME */   // the player shares hps_ocram_rx's file, as the feeder does
module hps_ocram_fplay #(parameter [2:0] GAP = 3'd4)   // 4..7
(
	input  wire        clk_sys,
	input  wire        hen_s,        // the session's enable level
	input  wire        w_v,          // one cycle: an entry is in w_d
	input  wire        w_head,       // it is an LFRAME header
	input  wire [31:0] w_d,          // held until the entry is answered
	output reg         a_go = 1'b0,  // one cycle: the entry's answer
	output reg  [31:0] a_d = 32'd0,  // with a ring word's answer, its two link replies, low half first (LANE)
	output wire        rest,         // idle: no frame, no entry, idle outputs

	/* verilator lint_off UNUSEDSIGNAL */   // without MISTER_OCRAM_LANE no reply is sampled
	input  wire [15:0] l_dout,       // the link's reply word, sampled at each link word's acknowledge
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        l_busy,       // any of the link's own selects, or its ID view
	input  wire        l_wait,       // io_wait | vs_wait: the term that withholds the link's ack
	output reg         own    = 1'b0,// the player drives the link nets
	output reg         hold   = 1'b0,// own, plus one cycle: the link's ack path stays frozen
	output reg   [3:0] sel    = 4'd0,// {osd_vga, osd_hdmi, fpga, uio}
	output reg         strobe = 1'b0,
	output wire [15:0] din           // the link word in hand: held from the cycle before its strobe
);

localparam [2:0] S_IDLE = 3'd0, S_STB = 3'd1, S_ACK = 3'd2, S_NEXT = 3'd3, S_GAP = 3'd4, S_REL = 3'd5;

reg   [2:0] ps    = S_IDLE;
reg   [2:0] cnt   = 3'd0;
reg   [1:0] quiet = 2'd0;              // cycles the link's selects have been low, saturating at 2
reg   [1:0] ml    = 2'd0;              // cycles with the enable low and no word in hand, saturating at 2
reg         fpend = 1'b0;              // a header is taken and its frame not started
reg   [1:0] fsel  = 2'd0;
reg  [14:0] n     = 15'd0;             // the frame's link words still to strobe, the one in hand included
reg         have  = 1'b0;              // a ring word is in w_d
reg         half  = 1'b0;              // the link word in hand is the ring word's high half

wire        wpres = have || (w_v && !w_head);
wire        t_beg = ps == S_IDLE && fpend && wpres && quiet == 2'd2 && !l_busy && hen_s;
wire        t_ack = ps == S_ACK && !l_wait && cnt == GAP - 3'd1;

assign din  = half ? w_d[31:16] : w_d[15:0];
assign rest = ps == S_IDLE && !own && !hold && !strobe && sel == 4'd0 && !fpend && !have && !a_go;

always @(posedge clk_sys) begin
	a_go  <= 1'b0;
	quiet <= l_busy ? 2'd0 : (quiet == 2'd2) ? 2'd2 : quiet + 2'd1;
	ml    <= (hen_s || wpres) ? 2'd0 : (ml == 2'd2) ? 2'd2 : ml + 2'd1;

	// an entry: a header opens a frame (with the enable up) and is answered at once; a word waits in w_d
	if (w_v && w_head) begin
		a_go  <= 1'b1;
		fpend <= hen_s;
		fsel  <= w_d[17:16];
		n     <= {w_d[13:0], 1'b0} - {14'd0, w_d[23]};
	end
	else if (w_v) have <= 1'b1;

	case (ps)
		S_IDLE: if (t_beg) begin
			own   <= 1'b1;
			hold  <= 1'b1;
			sel   <= 4'b0001 << fsel;
			half  <= 1'b0;
			fpend <= 1'b0;
			ps    <= S_STB;
		end
		else begin
			own    <= 1'b0;
			hold   <= 1'b0;
			sel    <= 4'd0;
			strobe <= 1'b0;
			if (fpend && !hen_s) fpend <= 1'b0;          // the session ended before the frame began
			if (have && !fpend) begin                    // a word with no frame: answered unplayed
				have <= 1'b0;
				a_go <= 1'b1;
			end
		end
		S_STB: begin
			strobe <= 1'b1;
			cnt    <= 3'd0;
			n      <= n - 15'd1;
			ps     <= S_ACK;
		end
		S_ACK: begin
			strobe <= 1'b0;
			if (l_wait) cnt <= 3'd0;
			else if (t_ack) begin
				ps <= S_NEXT;
`ifdef MISTER_OCRAM_LANE
				// the reply as the HPS reads it at this word's acknowledge: a low half alone, or both halves
				a_d <= half ? {l_dout, a_d[15:0]} : {16'd0, l_dout};
`endif
				if (n != 15'd0) begin
					// the ring word's second half is acknowledged: answer it; the frame's last waits for its end
					if (half) begin have <= 1'b0; a_go <= 1'b1; end
					half <= ~half;
				end
			end
			else cnt <= cnt + 3'd1;
		end
		S_NEXT: if (n != 15'd0 && wpres) ps <= S_STB;
		else if (n == 15'd0 || ml == 2'd2) begin
			sel <= 4'd0;
			cnt <= 3'd0;
			ps  <= S_GAP;
		end
		S_GAP: if (cnt == 3'd1) begin
			own <= 1'b0;
			ps  <= S_REL;
		end
		else cnt <= cnt + 3'd1;
		S_REL: begin
			hold <= 1'b0;
			half <= 1'b0;
			if (have) begin have <= 1'b0; a_go <= 1'b1; end
			ps   <= S_IDLE;
		end
		default: begin
			own    <= 1'b0;
			hold   <= 1'b0;
			sel    <= 4'd0;
			strobe <= 1'b0;
			ps     <= S_IDLE;
		end
	endcase
end

endmodule
/* verilator lint_on DECLFILENAME */

`default_nettype wire
`endif
`undef LW_OCRAM_RX_BUILT
`endif
