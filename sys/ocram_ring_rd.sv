//============================================================================
//  ocram_ring_rd.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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
`define LW_OCRAM_RING_RD_BUILT
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_OCRAM_RING_RD_BUILT
`endif
`ifdef MISTER_OCRAM_FIO
`define LW_OCRAM_RING_RD_BUILT
`endif
`ifdef LW_OCRAM_RING_RD_BUILT
`default_nettype none

// hps_ocram_rx's walker, on clk_100m alone: the host-to-core rings' reader (hps_ocram_rx.sv's header has the
// rules). The ring table is indexed by ring id (0 OSD, 1 SECTOR, 2 download, 3 mount): each ring's tail, avail
// (head - tail, within its ring's width), due, stop, mid and fault, its head and tail offsets and its size. The
// walker states mean one thing on every ring; the record classifiers in W_HDR are each ring's own rules. The binds
// (ocram_bind, one per slot) and the mount WRITE (ocram_mnt_wr, which borrows the forward cell) are its parts; the
// crossing (ocram_xing) holds every cell and every crossing source register: x_* launches a word, done_* is its
// answer, bind_d is the bind levels' next value and bind_q the levels.

module ocram_ring_rd #(parameter ROWS = 16, parameter OSD_EN = 1, parameter SEC_EN = 0,
                       parameter [31:0] FRAME = 32'd128,
                       parameter FIO_EN = 0, parameter [31:0] FIO_RING = 32'd10,
                       parameter [1:0] MNT_EN = 2'd0, parameter [2:0] BLKSZ = 3'd2,
                       parameter [3:0] FIO_SELS = 4'b0010)
(
	input  wire        clk,

	// The master's session interface
	input  wire        s_hello,
	input  wire        s_live,
	input  wire        s_tick,
	input  wire        bind_en,
	input  wire  [2:0] sec_ask,
	input  wire        evt_stop,
	output wire        c_fault,
	output wire        c_mid,
	output wire        c_open,
	output wire        c_sfault,
	output wire        c_smid,
	output wire  [2:0] c_sbind,
	output wire        rq_post,
	output wire [31:0] rq_lba,
	input  wire        rq_start,
	output wire [15:0] osd_rows,
	output wire [15:0] osd_deepest,
	output wire [15:0] f_tail,
	output wire        c_ffault,
	output wire        c_fmid,
	output wire        c_mfault,
	output wire        c_mmid,
	output wire        m_post,
	output wire        m_write,
	output wire  [7:0] m_arg,
	output wire [10:0] m_len,
	output wire [31:0] m_lba,
	output wire        m_wv,
	output wire [31:0] m_wd,
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the master's answers are not read
	input  wire        m_start,
	input  wire        m_wtake,
	input  wire        m_pub,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        p_post,
	output wire  [7:0] p_arg,
	output wire [10:0] p_n,
	output wire        p_wv,
	output wire [31:0] p_wd,
	/* verilator lint_off UNUSEDSIGNAL */   // without MISTER_OCRAM_LANE the master's answers are not read
	input  wire [15:0] e_free,
	input  wire        p_start,
	input  wire        p_wtake,
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

	// The crossing's clk side
	output wire        x_go,         // launch x_d
	output wire [36:0] x_d,          // {tag, wr, k, d}
	output wire        x_out,        // a word launched, its done not back
	input  wire        fwd_busy,
	input  wire [33:0] done_q,       // {d, code}
	input  wire        done_v,
	output wire  [2:0] bind_d,       // the bind levels' next value, by slot index k
	input  wire  [2:0] bind_q,       // the bind levels
	input  wire  [2:0] en_b,         // en_sys, back
	input  wire [32:0] rq_q,         // the disc slot's request record {offered, lba}
	input  wire        rq_v,
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the mount records are not read
	input  wire [41:0] mq,           // a mount slot's request record {k, wr, blk_cnt, offered, lba}
	input  wire        mq_v,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        f_open        // the enable link's open
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
localparam [15:0] MNT_BASE  = 16'h6000;
localparam [15:0] MNT_WORDS = 16'd1024;
// FIO_SELS, the selects an LFRAME may name ({osd_vga, osd_hdmi, fpga, uio}), is ocram_cfg.vh's OCRAM_FIO_SELS (sys_top
// passes it), the value hps_ocram_master declares in C_K[9].
localparam  [7:0] LAST_ROW  = ROWS - 1;
localparam [15:0] SEC_LEN   = FRAME[15:0] + 16'd1;      // a FRAME record's len: the LBA and the words
localparam [13:0] SEC_K     = FRAME[13:0] + 14'd1;      // its last payload word

localparam [1:0] TG_OHEAD = 2'd0, TG_SHEAD = 2'd1, TG_DATA = 2'd2, TG_FHEAD = 2'd3;
localparam [1:0] RG_OSD = 2'd0, RG_SEC = 2'd1, RG_FIO = 2'd2, RG_MNT = 2'd3;   // the ring ids
localparam [1:0] A_ACCEPT = 2'd1, A_MISMATCH = 2'd2;   // 0 took, 3 no request

// The ring table's constants, 16 bits an entry, by ring id {3 mount, 2 download, 1 SECTOR, 0 OSD}
localparam [63:0] H_TAB   = {H_IDX0, H_IDX1, H_IDX4, H_IDX2};             // each ring's head
localparam [63:0] C_TAB   = {C_IDX0, C_IDX1, C_IDX4, C_IDX2};             // each ring's tail
localparam [63:0] W_TAB   = {MNT_WORDS, FIO_WORDS, SEC_WORDS, OSD_WORDS};  // each ring's size
localparam [63:0] AV_MASK = {16'h07FF, 16'h3FFF, 16'h3FFF, 16'h0FFF};     // each ring's avail width

generate
	if (OSD_BASE + OSD_WORDS * 4 > 16'hF000 || SEC_BASE + SEC_WORDS * 4 > 16'hF000 ||
	    FIO_BASE + FIO_WORDS * 4 > 16'hF000 || MNT_BASE + MNT_WORDS * 4 > 16'hF000) begin : g_rx_span
		ocram_rx_span_above_0xF000 guard();
	end
	if (H_IDX0 >= 16'h0100 || H_IDX1 >= 16'h0100 || H_IDX2 >= 16'h0100 || H_IDX4 >= 16'h0100 ||
	    C_IDX0 >= 16'h0100 || C_IDX1 >= 16'h0100 || C_IDX2 >= 16'h0100 || C_IDX4 >= 16'h0100) begin : g_rx_ctrl
		ocram_rx_control_constant_above_0xFF guard();
	end
endgenerate

localparam [3:0] W_IDLE = 4'd0, W_HEAD = 4'd1, W_HDR = 4'd2, W_LBA = 4'd3, W_HGO = 4'd4, W_ANS = 4'd5,
                 W_PAY = 4'd6, W_XFER = 4'd7, W_TAIL = 4'd8, W_FLUSH = 4'd9, W_FTAIL = 4'd10,
                 W_MNT = 4'd11;           // the mount WRITE has the forward cell
`ifdef MISTER_OCRAM_LANE
localparam [3:0] W_CAP = 4'd15;           // a capture frame waits for its REPLY's header
`endif

reg   [3:0] wst = W_IDLE;
reg         req = 1'b0;
reg         req_we = 1'b0;
reg  [15:0] req_off = 16'd0;
reg  [15:0] req_wd = 16'd0;            // the request's data word: tail + k
reg         rd_pend = 1'b0;            // a ring word read is owed at tail + k
reg         st_pend = 1'b0;            // the ring's tail store is owed: tail + k
reg         word_out = 1'b0;           // a word launched, its done not back
reg   [1:0] ring = RG_OSD;             // the record's ring
reg   [1:0] turn = RG_OSD;             // the ring served last
// the ring table, by ring id, 16 bits an entry
reg  [63:0] tail  = 64'd0;
reg  [63:0] avail = 64'd0;             // head - tail, each within its ring's width (AV_MASK)
reg   [3:0] due   = 4'b1111;           // read the head when caught up
reg   [3:0] stop  = 4'b0000;
reg   [3:0] mid   = 4'b0000;
reg   [3:0] flt   = 4'b0000;
reg  [13:0] k = 14'd0;
reg   [4:0] row_q = 5'd0;              // the ROW being sent: its OSD_HEAD carries the row alone
reg  [13:0] flen_q = 14'd0;            // the record being sent: the k of its last payload word
`ifdef MISTER_OCRAM_DIAG
reg         was_row = 1'b0;            // the OSD record being finished is a ROW
`endif
reg         s_wait = 1'b0;             // a no-request answer: wait for the record or a tick
reg         fl_pend = 1'b0;            // the unbind's flush is owed
reg         rq_posted = 1'b0, fresh = 1'b0;
reg   [1:0] m_posted = 2'd0;           // each slot's request has its REQUEST or WRITE begun
reg   [1:0] mfk = 2'd1;                // the slot of the mount FRAME or WRITE being walked
reg  [39:0] mr1 = 40'd0, mr2 = 40'd0;  // each slot's last request record {wr, blk_cnt, offered, lba}
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
localparam [18:0] MW_0 = 19'd1 << ({1'b0, BLKSZ} + 4'd5);
/* verilator lint_on UNUSEDSIGNAL */
wire  [1:0] mst = {mr2[32], mr1[32]} & MNT_EN;
wire  [1:0] mwr = {mr2[39], mr1[39]};
// each slot's block words, registered as its request record lands (bit 0 is 0: the WRITE len is the words | 1)
reg  [10:0] mn1_q = MW_0[10:0], mn2_q = MW_0[10:0];
wire [10:0] mn1 = mn1_q, mn2 = mn2_q;

// the binds, by slot index k: 0 the disc slot (its unbind flushes ring 1), 1 and 2 the mount slots
wire  [2:0] b_want, b_hold, b_bound, b_drop;
/* verilator lint_off UNUSEDSIGNAL */   // only the disc slot's unbind flushes
wire  [2:0] b_go;
/* verilator lint_on UNUSEDSIGNAL */
wire        b_clr = !s_live && wst == W_IDLE;
wire        sb     = b_bound[0], drop_s = b_drop[0];
wire  [1:0] sbm    = b_bound[2:1], drop_m = b_drop[2:1];

// the mount WRITE
wire        mw_job, mw_post, mw_have, mw_go, mw_out, mw_k0, mw_k1, mw_fin;
wire [36:0] mw_d;
wire  [1:0] w_done;
// the slot whose record is offered to the master: the WRITE being walked, else a bound read not yet posted
wire  [1:0] rd_ok = mst & ~mwr & sbm & ~drop_m & bind_q[2:1] & ~m_posted;
wire  [1:0] wj    = mst & mwr & sbm & ~drop_m & ~w_done;
wire  [1:0] pk    = mw_job ? mfk : rd_ok[0] ? 2'd1 : 2'd2;
wire        pk2   = pk == 2'd2;
/* verilator lint_off UNUSEDSIGNAL */   // its direction and offered bit are the walker's
wire [39:0] pr    = pk2 ? mr2 : mr1;
/* verilator lint_on UNUSEDSIGNAL */
wire [10:0] mn    = (mfk == 2'd2) ? mn2 : mn1;        // the walked slot's block

// the record's ring's entries
wire [15:0] av_r    = avail[{ring, 4'd0} +: 16];
wire [15:0] am_r    = AV_MASK[{ring, 4'd0} +: 16];
// a response is decoded in its own cycle, as the bridge's rdata reaches each register in one stage
wire [16:0] rlen    = {1'b0, c_rdata[15:0]} + 17'd1;
wire        c_new   = c_done && c_ok;
wire        c_bad   = c_done && !c_ok;
wire [16:0] ravail  = {1'b0, av_r};
wire [15:0] rwords  = W_TAB[{ring, 4'd0} +: 16];
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
wire        walk_o = OSD_EN != 0 && s_live && f_open && bind_en && !stop[RG_OSD];
wire        walk_s = SEC_EN != 0 && s_live && f_open && !stop[RG_SEC];
wire        walk_f = FIO_EN != 0 && s_live && f_open && !stop[RG_FIO];
wire        walk_m = MNT_EN != 0 && s_live && f_open && !stop[RG_MNT];
wire        want_s = walk_s && sec_ask[0] && !evt_stop;
wire  [1:0] want_m = MNT_EN & sec_ask[2:1] & {2{walk_m && !evt_stop}};
wire        m_wjob = wj != 2'd0 && !mq_v && !evt_stop;
wire        mt_o   = avail[16*RG_OSD +: 16] == 16'd0;
wire        mt_s   = avail[16*RG_SEC +: 16] == 16'd0;
wire        mt_f   = avail[16*RG_FIO +: 16] == 16'd0;
wire        mt_m   = avail[16*RG_MNT +: 16] == 16'd0;
wire        job_o  = walk_o && (!mt_o || due[RG_OSD]);
wire        job_s  = walk_s && sb && !drop_s && ((!mt_s && stand && !s_wait) || (mt_s && due[RG_SEC]));
`ifdef MISTER_OCRAM_LANE
// a capture frame left for want of room holds ring 1 until the EVT ring has it
reg         cap_held = 1'b0;
reg  [10:0] cap_n_q = 11'd0;
reg  [31:0] cap_hdr_q = 32'd0;
reg         cap_q = 1'b0;              // the frame being played is a capture frame
reg         p_have = 1'b0;             // p_wd holds a reply word for the master
wire        cap_room = {5'd0, cap_n_q} + 16'd1 <= e_free && !evt_stop;
wire        job_f  = walk_f && (!mt_f || due[RG_FIO]) && !(cap_held && !cap_room);
assign p_post = wst == W_CAP;
assign p_arg  = cap_hdr_q[23:16];
assign p_n    = cap_n_q;
assign p_wv   = p_have;
assign p_wd   = done_q[33:2];
`else
wire        job_f  = walk_f && (!mt_f || due[RG_FIO]);
assign p_post = 1'b0;
assign p_arg  = 8'd0;
assign p_n    = 11'd0;
assign p_wv   = 1'b0;
assign p_wd   = 32'd0;
`endif
wire        job_m  = walk_m && (m_wjob || !mt_m || due[RG_MNT]);
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
wire  [1:0] pr_id  = pick[1:0];
wire        flush  = SEC_EN != 0 && fl_pend;
wire        mw_take = wst == W_IDLE && !flush && pick[2] && pr_id == RG_MNT && m_wjob;   // the WRITE begins

// idle, the tail is the next record's ring's (tl_p): a request raised then registers that ring's sum; tl_q holds
// it from the cycle the walk leaves idle (ring <= ring_p), so out of idle it is the walked ring's tail
wire  [1:0] ring_p = flush ? RG_SEC : pr_id;
wire [15:0] tl_p   = tail[{ring_p, 4'd0} +: 16];
reg  [15:0] tl_q   = 16'd0;
wire [15:0] widx_i = tl_p + {2'd0, k};      // the idle head read's
wire [15:0] widx   = tl_q + {2'd0, k};      // every request of the walk
wire [15:0] osd_off = OSD_BASE + ((widx & (OSD_WORDS - 1)) << 2);
wire [15:0] sec_off = SEC_BASE + ((widx & (SEC_WORDS - 1)) << 2);
wire [15:0] fio_off = FIO_BASE + ((widx & (FIO_WORDS - 1)) << 2);
wire [15:0] mnt_off = MNT_BASE + ((widx & (MNT_WORDS - 1)) << 2);
wire [63:0] R_TAB   = {mnt_off, fio_off, sec_off, osd_off};    // each ring's word at tail + k
wire [15:0] new_gap = c_rdata[15:0] - tl_q;

assign c_req    = req;
assign c_we     = req_we;
assign c_off    = req_off;
assign c_wdata  = {16'd0, req_wd};        // tail + k as the request rose: k and the tail hold until its done
assign c_fault  = flt[RG_OSD];
assign c_mid    = mid[RG_OSD];
assign c_sfault = flt[RG_SEC];
assign c_smid   = mid[RG_SEC];
assign c_ffault = flt[RG_FIO];
assign c_fmid   = mid[RG_FIO];
assign c_mfault = flt[RG_MNT];
assign c_mmid   = mid[RG_MNT];
assign f_tail   = tail[16*RG_FIO +: 16];
assign c_open   = f_open & bind_en;
assign c_sbind  = {sbm, sb};
assign rq_lba   = rq_q[31:0];
// not in a record's landing cycle: its fresh read is the next one
assign rq_post  = SEC_EN != 0 && sb && !drop_s && bind_q[0] && stand && !rq_posted && fresh && !rq_v &&
                  mt_s && !stop[RG_SEC];
// a mount read is posted at once; a WRITE once the feeder has raised its ack
assign m_post   = MNT_EN != 0 && !mq_v && (mw_job ? mw_post : (rd_ok != 2'd0 && !stop[RG_MNT]));
assign m_write  = mw_job;
assign m_arg    = {pr[38:33], pk};
assign m_len    = mw_job ? ((pk2 ? mn2_q : mn1_q) | 11'd1) : 11'd1;
assign m_lba    = pr[31:0];
assign m_wv     = mw_have;
assign m_wd     = done_q[33:2];         // the answer to the last token: held until the next one is launched

// A payload word, a frame's LBA and an LFRAME's header go to the cell in their read's response cycle (each read is
// issued only with the cell idle, and nothing else launches before its response); a ROW's head, its row alone,
// from row_q; the mount WRITE's head and tokens from ocram_mnt_wr.
wire   f_hgo = wst == W_HDR && ring == RG_FIO && c_new && f_rec && !f_cap && rlen <= ravail;
wire   x_rsp = ((wst == W_PAY || wst == W_LBA) && c_new || f_hgo) && s_live;
assign x_go  = x_rsp || (wst == W_HGO && !fwd_busy && s_live) || mw_go;
assign x_d   = (wst == W_PAY) ? {TG_DATA, 3'd0, c_rdata} :
               (wst == W_LBA) ? {TG_SHEAD, 1'b0, (ring == RG_MNT) ? mfk : 2'd0, c_rdata} :
               (wst == W_HDR) ? {TG_FHEAD, 3'd0, c_rdata} :
               (wst == W_MNT) ? mw_d :
`ifdef MISTER_OCRAM_LANE
               (ring == RG_FIO) ? {TG_FHEAD, 3'd0, cap_hdr_q} :     // a capture frame's header, once its REPLY is open
`endif
               {TG_OHEAD, 3'd0, 11'd0, row_q, 16'd0};
assign x_out = word_out | mw_out;

// the binds: the disc slot's level is held while a frame streams, a mount slot's while a mount record is walked
assign b_want = {want_m, want_s};
assign b_hold = {{2{mstream && s_live}}, streaming && s_live};
genvar bi;
generate
	for (bi = 0; bi < 3; bi = bi + 1) begin : bnd
		ocram_bind slot
		(
			.clk(clk), .s_hello(s_hello), .clr(b_clr), .want(b_want[bi]), .hold(b_hold[bi]),
			.flush(bi == 0 && fl_pend), .lvl(bind_q[bi]), .en(en_b[bi]),
			.lvl_d(bind_d[bi]), .bound(b_bound[bi]), .drop(b_drop[bi]), .drop_go(b_go[bi])
		);
	end
endgenerate

ocram_mnt_wr mwr_u
(
	.clk(clk), .s_hello(s_hello), .s_live(s_live), .go(mw_take), .slot(mfk),
	.lba((mfk == 2'd2) ? mr2[31:0] : mr1[31:0]), .mn(mn), .k(k),
	.fwd_busy(fwd_busy), .done_v(done_v), .done_c(done_q[1:0]), .evt_stop(evt_stop),
	.m_start(m_start), .m_wtake(m_wtake), .m_pub(m_pub),
	.land({mq_v && mq[41:40] == 2'd2, mq_v && mq[41:40] == 2'd1}),
	.x_go(mw_go), .x_d(mw_d), .x_out(mw_out), .k_zero(mw_k0), .k_step(mw_k1), .fin(mw_fin),
	.job(mw_job), .post(mw_post), .have(mw_have), .w_done(w_done)
);

always @(posedge clk) begin
	flt <= 4'b0000;
	if (wst == W_IDLE) tl_q <= tl_p;
	if (c_grant) req <= 1'b0;

	// the disc slot's unbind owes a flush of ring 1, unless the ring has stopped
	if (b_go[0]) fl_pend <= !stop[RG_SEC];

	// the request record: a new one asks for one fresh head read, and voids the post
	if (rq_v) begin
		rq_posted <= 1'b0;
		fresh     <= 1'b0;
		s_wait    <= 1'b0;
		if (rq_q[32]) due[RG_SEC] <= 1'b1;
	end
	else if (rq_start) rq_posted <= 1'b1;
	// the mount record: a new one voids the post (and the WRITE's end, in ocram_mnt_wr)
	if (mq_v && mq[41:40] == 2'd1) begin mr1 <= mq[39:0]; mn1_q <= mwn(mq[38:33]); m_posted[0] <= 1'b0; end
	if (mq_v && mq[41:40] == 2'd2) begin mr2 <= mq[39:0]; mn2_q <= mwn(mq[38:33]); m_posted[1] <= 1'b0; end
	if (m_start) m_posted[pk2] <= 1'b1;
	if (s_tick) begin due <= 4'b1111; s_wait <= 1'b0; end

	// a ring word read, addressed by the one adder; never while the forward cell is busy
	if (rd_pend && !req && !fwd_busy) begin
		rd_pend <= 1'b0;
		req     <= 1'b1;
		req_we  <= 1'b0;
		req_wd  <= widx;
		req_off <= R_TAB[{ring, 4'd0} +: 16];
	end
	if (st_pend && !req) begin
		st_pend   <= 1'b0;
		req       <= 1'b1;
		req_we    <= 1'b1;
		req_wd    <= widx;
		req_off   <= C_TAB[{ring, 4'd0} +: 16];
	end

	if (s_hello) begin
		wst <= W_IDLE; req <= 1'b0; rd_pend <= 1'b0; st_pend <= 1'b0; word_out <= 1'b0;
		tail <= 64'd0; avail <= 64'd0; due <= 4'b1111; stop <= 4'b0000; mid <= 4'b0000;
		s_wait <= 1'b0; turn <= RG_OSD; fl_pend <= 1'b0; fresh <= 1'b0; rq_posted <= 1'b0; m_posted <= 2'd0;
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
`ifdef MISTER_OCRAM_LANE
		cap_q <= 1'b0; p_have <= 1'b0;
`endif
	end
	else if (!s_live) fl_pend <= 1'b0;
	else case (wst)
	W_IDLE: begin
`ifdef MISTER_OCRAM_LANE
		cap_q <= 1'b0;
`endif
		mid <= 4'b0000;
		if (flush) begin
			ring <= RG_SEC; req <= 1'b1; req_we <= 1'b0; req_off <= H_IDX4; req_wd <= widx_i; wst <= W_FLUSH;
		end
		else if (pick[2]) begin
			ring <= pr_id; turn <= pr_id;
			if (mw_take) begin mfk <= wj[0] ? 2'd1 : 2'd2; wst <= W_MNT; end   // the WRITE's head crosses
			else if (avail[{pr_id, 4'd0} +: 16] == 16'd0) begin
				req <= 1'b1; req_we <= 1'b0; req_off <= H_TAB[{pr_id, 4'd0} +: 16]; req_wd <= widx_i; due[pr_id] <= 1'b0;
				wst <= W_HEAD;
			end
			else begin k <= 14'd0; rd_pend <= 1'b1; mid[pr_id] <= 1'b1; wst <= W_HDR; end
		end
	end
	W_HEAD, W_FLUSH: if (c_done) begin
		if (!c_ok) begin req <= 1'b1; req_we <= 1'b0; req_wd <= widx; end   // no value: read it again
		else if (new_gap > rwords) begin
			// a head more than the ring's size ahead: a host defect
			stop[ring] <= 1'b1; flt[ring] <= 1'b1;
			if (ring == RG_SEC) fl_pend <= 1'b0;
			wst <= W_IDLE;
		end
		else if (wst == W_FLUSH) begin
			// the unbind: every queued frame is discarded by one tail store
			k <= new_gap[13:0]; st_pend <= 1'b1; wst <= W_FTAIL;
		end
		else begin
			avail[{ring, 4'd0} +: 16] <= new_gap & am_r;
			if (ring == RG_SEC) fresh <= 1'b1;
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
			stop[ring] <= 1'b1; flt[ring] <= 1'b1; mid[ring] <= 1'b0;
			wst <= W_IDLE;
		end
		else if (ring == RG_OSD && c_rdata[31:24] == 8'h01 && c_rdata[15:0] == 16'd64 &&
		         c_rdata[23:16] <= LAST_ROW) begin
			row_q <= c_rdata[20:16]; flen_q <= 14'd64; k <= 14'd0; word_out <= 1'b1; wst <= W_HGO;
`ifdef MISTER_OCRAM_DIAG
			was_row <= 1'b1;
`endif
		end
		else if (ring == RG_SEC && c_rdata[31:24] == 8'h10 && c_rdata[23:16] == 8'd0 && c_rdata[15:0] == SEC_LEN) begin
			flen_q <= SEC_K; k <= 14'd1; rd_pend <= 1'b1; wst <= W_LBA;
		end
`ifdef MISTER_OCRAM_LANE
		else if (ring == RG_FIO && f_rec && f_cap) begin
			// a capture frame: held for its REPLY while the EVT ring has room, else left where it is
			cap_hdr_q <= c_rdata; cap_n_q <= c_rdata[10:0]; k <= 14'd0;
			if ({5'd0, c_rdata[10:0]} + 16'd1 <= e_free && !evt_stop) begin cap_held <= 1'b0; wst <= W_CAP; end
			else begin cap_held <= 1'b1; mid[RG_FIO] <= 1'b0; wst <= W_IDLE; end
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
	W_HGO: if (!fwd_busy) wst <= W_XFER;                            // x_go launches the ROW's or capture's head
	W_ANS: if (done_v) begin
		// a frame's head: accept streams it; mismatch (a mount frame: any refusal) skips it; no request keeps it
		word_out <= 1'b0;
		if (done_q[1:0] == A_ACCEPT) begin k <= 14'd2; rd_pend <= 1'b1; wst <= W_PAY; end
		else if (done_q[1:0] == A_MISMATCH || ring == RG_MNT) begin
			k <= flen_q + 14'd1; st_pend <= 1'b1; wst <= W_TAIL;
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
	else if (!cap_room) begin cap_held <= 1'b1; mid[RG_FIO] <= 1'b0; wst <= W_IDLE; end
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
		else if (k == flen_q) begin k <= k + 14'd1; st_pend <= 1'b1; wst <= W_TAIL; end
		else begin k <= k + 14'd1; rd_pend <= 1'b1; wst <= W_PAY; end
	end
`else
	W_XFER: if (done_v) begin
		word_out <= 1'b0;
		if (k == flen_q) begin k <= k + 14'd1; st_pend <= 1'b1; wst <= W_TAIL; end
		else begin k <= k + 14'd1; rd_pend <= 1'b1; wst <= W_PAY; end
	end
`endif
	W_MNT: begin
		// ocram_mnt_wr walks the WRITE on the walker's counter
		if (mw_k0) k <= 14'd0;
		if (mw_k1) k <= k + 14'd1;
		if (mw_fin) wst <= W_IDLE;
	end
	W_TAIL, W_FTAIL: begin
		if (c_new) begin
			// the tail store is done: the record is consumed (a flush, every queued frame)
			wst <= W_IDLE;
			tail[{ring, 4'd0} +: 16]  <= widx;
			avail[{ring, 4'd0} +: 16] <= (wst == W_FTAIL) ? 16'd0 : (av_r - {2'd0, k}) & am_r;
			mid[ring] <= 1'b0;
			if (av_r == {2'd0, k} || wst == W_FTAIL) due[ring] <= 1'b1;
			if (ring == RG_SEC) fresh <= 1'b0;
			if (wst == W_FTAIL) fl_pend <= 1'b0;
`ifdef MISTER_OCRAM_DIAG
			if (ring == RG_OSD && was_row) rows_q <= rows_q + 16'd1;
`endif
		end
		else if (c_bad) begin req <= 1'b1; req_wd <= widx; end     // a failed store is tried again, same value
	end
	default: wst <= W_IDLE;
	endcase
end

endmodule

`default_nettype wire
`endif
`undef LW_OCRAM_RING_RD_BUILT
`endif
