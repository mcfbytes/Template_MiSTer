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
`ifdef LW_OCRAM_RX_BUILT
`default_nettype none

// The host-to-core consumer of the OCRAM mailbox: ONE walker on clk_100m for every host-to-core ring it serves, one
// held-data word crossing each way, and the sinks on clk_sys.

module hps_ocram_rx #(parameter ROWS = 16, parameter OSD_EN = 1, parameter SEC_EN = 0,
                      parameter [31:0] FRAME = 32'd128, parameter [31:0] GAP = 32'd2,
                      parameter FIO_EN = 0, parameter [31:0] FIO_RING = 32'd10, parameter [31:0] FIO_GAP = 32'd4,
                      parameter [1:0] MNT_EN = 2'd0, parameter [31:0] MFRAME = 32'd128, parameter [2:0] BLKSZ = 3'd2,
                      parameter [3:0] FIO_SELS = 4'b0010)
(
	input  wire        clk,          // clk_100m: the walker
	input  wire        clk_sys,      // the core clock: the sinks

	/* verilator lint_off UNUSEDSIGNAL */
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
	output wire [15:0] osd_rows,
	output wire [15:0] osd_deepest,
	output wire [15:0] f_tail,       // the download ring's tail, as last stored in C_IDX[1]
	output wire        c_ffault,     // one cycle: the download ring stopped on a fault
	output wire        c_fmid,       // level: a download record is part-consumed
	output wire        c_mfault,     // one cycle: the mount ring stopped on a fault
	output wire        c_mmid,       // level: a mount record is part-consumed
	// the mount slots' records, for the master's EVT writer (MNT_EN)
	output wire        m_post,       // a REQUEST or a WRITE is owed for a mount slot's request
	output wire        m_write,      // it is a WRITE
	output wire  [7:0] m_arg,        // {blk_cnt, k}
	output wire [10:0] m_len,        // the record's len: 1 for a REQUEST, 1 + the block's words for a WRITE
	output wire [31:0] m_lba,
	output wire        m_wv,         // a WRITE's next word is in m_wd
	output wire [31:0] m_wd,
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the master's answers are not read
	input  wire        m_start,      // one cycle: the master begins that record
	input  wire        m_wtake,      // one cycle: the master takes m_wd
	input  wire        m_pub,        // one cycle: the WRITE's head store is done
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        p_post,       // a capture frame is held, with room, for its REPLY
	output wire  [7:0] p_arg,        // its arg
	output wire [10:0] p_n,          // its len
	output wire        p_wv,         // a reply word is in p_wd
	output wire [31:0] p_wd,
	/* verilator lint_off UNUSEDSIGNAL */
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

// The pieces, by clock: the walker and its parts on clk (ocram_ring_rd: the ring table, the binds, the mount
// WRITE), the sinks on clk_sys (ocram_osd_wr, hps_ocram_sfeed, hps_ocram_fplay, ocram_blk_req), and ocram_xing
// between them, which holds every crossing cell and is the one module on both clocks.

localparam        TS_SPAN   = 0;

generate
	if (SEC_EN != 0 && (FRAME == 0 || FRAME > 4096 || GAP < 2 || GAP > 15)) begin : g_geometry
		ocram_sector_unservable_geometry guard();
	end
	// the download ring: 2^2 to 2^10 words at 0x1000, or 2^13 at 0x7000; strobes 4 to 7 cycles apart
	if (FIO_EN != 0 && (FIO_GAP < 4 || FIO_GAP > 7 || !((FIO_RING >= 2 && FIO_RING <= 10) || FIO_RING == 13)))
	begin : g_fio_geometry
		ocram_fio_unservable_geometry guard();
	end
	if (FIO_EN != 0 && FIO_RING == 13 && (SEC_EN != 0 || TS_SPAN != 0)) begin : g_fio_span
		ocram_fio_ring_on_ring4_span guard();
	end
	// the mount slots share the disc slot's feeder, its REQUEST writer and its bind rules
	if (MNT_EN != 0 && SEC_EN == 0) begin : g_mnt_sec
		ocram_mount_needs_MISTER_OCRAM_SECTOR_SLOT guard();
	end
endgenerate

// clk: the walker and the crossing's clk side
wire        x_go, x_out, fwd_busy, done_v, rq_v, mq_v, f_open;
wire [36:0] x_d;
wire [33:0] done_q;
wire  [2:0] bind_d, bind_q, en_b;
wire [32:0] rq_q;
wire [41:0] mq;
// clk_sys: the sinks and the crossing's clk_sys side
wire        hen_s, frz, o_v, o_head;
wire [31:0] w_d;
/* verilator lint_off UNUSEDSIGNAL */   // the feeder's, with SEC_EN; the player's, with FIO_EN
wire        w_wr, s_v, s_head, f_v, f_head;
wire  [1:0] w_k;
wire  [2:0] bind_s;
/* verilator lint_on UNUSEDSIGNAL */
wire        o_go, o_rest, f_go, f_rest, p_go, p_rest, rc_go, rc_busy, mrc_go, mrc_busy, q_rest;
wire  [1:0] f_code;
wire [31:0] f_dat;
/* verilator lint_off UNUSEDSIGNAL */
wire [31:0] p_dat;
/* verilator lint_on UNUSEDSIGNAL */
wire [32:0] rc_d;
wire [41:0] mrc_d;

ocram_ring_rd #(.ROWS(ROWS), .OSD_EN(OSD_EN), .SEC_EN(SEC_EN), .FRAME(FRAME), .FIO_EN(FIO_EN), .FIO_RING(FIO_RING),
                .MNT_EN(MNT_EN), .BLKSZ(BLKSZ), .FIO_SELS(FIO_SELS)) rd
(
	.clk(clk),
	.s_hello(s_hello), .s_live(s_live), .s_tick(s_tick), .bind_en(bind_en), .sec_ask(sec_ask), .evt_stop(evt_stop),
	.c_fault(c_fault), .c_mid(c_mid), .c_open(c_open), .c_sfault(c_sfault), .c_smid(c_smid), .c_sbind(c_sbind),
	.rq_post(rq_post), .rq_lba(rq_lba), .rq_start(rq_start), .osd_rows(osd_rows), .osd_deepest(osd_deepest),
	.f_tail(f_tail), .c_ffault(c_ffault), .c_fmid(c_fmid), .c_mfault(c_mfault), .c_mmid(c_mmid),
	.m_post(m_post), .m_write(m_write), .m_arg(m_arg), .m_len(m_len), .m_lba(m_lba), .m_wv(m_wv), .m_wd(m_wd),
	.m_start(m_start), .m_wtake(m_wtake), .m_pub(m_pub),
	.p_post(p_post), .p_arg(p_arg), .p_n(p_n), .p_wv(p_wv), .p_wd(p_wd), .e_free(e_free), .p_start(p_start),
	.p_wtake(p_wtake),
	.c_req(c_req), .c_we(c_we), .c_off(c_off), .c_wdata(c_wdata), .c_grant(c_grant), .c_done(c_done), .c_ok(c_ok),
	.c_rdata(c_rdata),
	.x_go(x_go), .x_d(x_d), .x_out(x_out), .fwd_busy(fwd_busy), .done_q(done_q), .done_v(done_v),
	.bind_d(bind_d), .bind_q(bind_q), .en_b(en_b), .rq_q(rq_q), .rq_v(rq_v), .mq(mq), .mq_v(mq_v), .f_open(f_open)
);

ocram_xing #(.OSD_EN(OSD_EN), .SEC_EN(SEC_EN), .FIO_EN(FIO_EN), .MNT_EN(MNT_EN)) xing
(
	.clk(clk), .gp_out25(gp_out25), .s_live(s_live), .x_go(x_go), .x_d(x_d), .fwd_busy(fwd_busy), .x_out(x_out),
	.done_q(done_q), .done_v(done_v), .bind_d(bind_d), .bind_q(bind_q), .en_b(en_b), .wide_b(c_wide),
	.rq_q(rq_q), .rq_v(rq_v), .mq(mq), .mq_v(mq_v), .f_open(f_open), .f_idle(c_rest),
	.clk_sys(clk_sys), .hen_s(hen_s), .frz(frz), .bind_s(bind_s),
	.w_d(w_d), .w_wr(w_wr), .w_k(w_k), .o_v(o_v), .o_head(o_head), .s_v(s_v), .s_head(s_head), .f_v(f_v),
	.f_head(f_head),
	.o_go(o_go), .o_rest(o_rest), .f_go(f_go), .f_code(f_code), .f_dat(f_dat), .f_rest(f_rest),
	.p_go(p_go), .p_dat(p_dat), .p_rest(p_rest),
	.rc_go(rc_go), .rc_d(rc_d), .rc_busy(rc_busy), .mrc_go(mrc_go), .mrc_d(mrc_d), .mrc_busy(mrc_busy),
	.q_rest(q_rest), .x_en(x_en), .xm_en(xm_en), .x_wide(x_wide)
);

// the OSD writer
ocram_osd_wr owr
(
	.clk_sys(clk_sys), .hen_s(hen_s), .frz(frz), .w_v(o_v), .w_head(o_head), .w_d(w_d), .a_go(o_go), .rest(o_rest),
	.sel(sel), .view(view), .we(we), .addr(addr), .dbyte(dbyte)
);

// the SECTOR feeder, for the disc slot and the mount slots
generate
if (SEC_EN != 0) begin : sec
	hps_ocram_sfeed #(.FRAME(FRAME), .GAP(GAP), .MNT_EN(MNT_EN), .BLKSZ(BLKSZ)) feeder
	(
		.clk_sys(clk_sys), .hen_s(hen_s), .bind_s(bind_s),
		.w_v(s_v), .w_head(s_head), .w_wr(w_wr), .w_k(w_k), .w_d(w_d),
		.a_go(f_go), .a_code(f_code), .a_d(f_dat), .rest(f_rest),
		.c_reset(x_reset), .hps_ack(x_hps_ack), .c_req(x_req), .c_lba(x_lba), .c_commit(x_commit), .c_wide(x_wide),
		.m_hps_ack(xm_hps_ack), .m_rd(xm_rd), .m_wr(xm_wr), .m_lba(xm_lba), .m_cnt(xm_cnt),
		.en_sys(x_en), .m_en(xm_en), .o_ack(x_ack), .m_ack(xm_ack), .o_wr(x_wr), .o_dat(x_dat), .o_adr(x_adr)
	);
end
else begin : nosec
	assign f_go = 1'b0, f_code = 2'd0, f_dat = 32'd0, f_rest = 1'b1;
	assign x_en = 1'b0, x_ack = 1'b0, x_wr = 1'b0, x_dat = 16'd0, x_adr = 14'd0, xm_en = 2'd0, xm_ack = 2'd0;
end
endgenerate

assign p_go = 1'b0, p_rest = 1'b1, p_dat = 32'd0;
assign l_own = 1'b0, l_hold = 1'b0, l_sel = 4'd0, l_strobe = 1'b0, l_din = 16'd0;

// the block slots' request records
ocram_blk_req #(.SEC_EN(SEC_EN), .MNT_EN(MNT_EN), .MFRAME(MFRAME), .BLKSZ(BLKSZ)) breq
(
	.clk_sys(clk_sys), .x_en(x_en), .x_req(x_req), .x_lba(x_lba), .x_ack(x_ack), .x_hps_ack(x_hps_ack),
	.xm_en(xm_en), .xm_rd(xm_rd), .xm_wr(xm_wr), .xm_lba(xm_lba), .xm_cnt(xm_cnt), .xm_ack(xm_ack),
	.xm_hps_ack(xm_hps_ack),
	.rc_go(rc_go), .rc_d(rc_d), .rc_busy(rc_busy), .mrc_go(mrc_go), .mrc_d(mrc_d), .mrc_busy(mrc_busy), .rest(q_rest)
);

endmodule

//============================================================================
// Strobes as hps_io's: with c_wide two 16-bit strobes a word at the halfword index, low half first, else four bytes
// at the byte index; GAP cycles apart, the first GAP after the ack.

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

`default_nettype wire
`endif
`undef LW_OCRAM_RX_BUILT
`endif
