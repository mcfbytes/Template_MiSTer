//============================================================================
//  lw_sector.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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

`ifdef MISTER_LW
`ifdef MISTER_LW_SECTOR_SLOT
`default_nettype none

// SECTOR block (LW directory kind 2, minor 1, "SEC2"): one hps_io read slot (the primary, slot index 0) plus two
// save slots (X0, X1: slot indices 1, 2) served over the LW bridge by ONE feeder. A save slot READ is a BEGIN_S
// frame of XFRAME words with a strobe address; a save slot WRITE is answered by WACK_S (ack, no strobes): the HPS
// takes the bytes from the core's declared DDR image. Fixed priority: primary, X0, X1. A slot's EN bit can be set
// only while the slot is wired (its mux's presence bit on HPS_BUS reads 1).
// Requests cross as records. On clk_sys a slot's request is offered while its mux is on this block (en_sys), the
// core's request is raised and the slot's ack is low and has been for two cycles (so a request still raised then
// is offered again, as hps_io shows it to an SPI host). A rise of offered sets the slot's new mark, a rejected
// answer its rej mark and that entry's tag, and any change marks the slot dirty. One lw_xfer carries one dirty
// slot's record at a time, round robin: {slot, offered, new, rej, tag, wr, rd, LBA}, latched at launch; a change in
// the launch cycle keeps the slot dirty, so its next record carries it. On clk_100m, the cycle after a record
// lands, REQ, LBA and LBA_S take it; the slot's answered mark clears on new, on a withdrawn request, or on a reject
// of its latest answer (the entry's tag counts the slot's BEGIN/WACK entries) unless an answer enters in that
// cycle; and the slot's CAUSE bit rises for an offered request that is new or re-opened. So a cause needs the
// slot's EN, and a reject arrives with the LBA the core asks for now.
// CAUSE: bit k = slot k; a full-word store clears each bit written as 1, and a raise in the store's cycle wins.
// Reads change nothing.
// clk_sys side: the feeder that re-creates hps_io's ack + 16-bit write strobes for each slot's consumer. An answer
// it cannot serve (EN, direction or LBA not the live request's), and a BEGIN met while a frame streams (which
// zero-fills that frame), is rejected through its slot's record and dropped with the DATA behind it.
// The host enable (hen, the window's en_q; it falls at an ID view, a core reset through GPO or an HPS reset)
// reaches the feeder through the block's enable link alone (lw_fence, lw_cdc.sv): stores reach this block only
// while the link is open, which is after the feeder has seen the enable.
// Every EN, TEST included, is cleared in the cycle the link's level falls, four cycles after the last stored word
// has been launched toward clk_sys, so each slot's mux returns to SPI once the feeder is off it and its SPI ack is
// low, and a frame whose every word was stored still completes with its data. A frame in flight that waits for
// data is zero-filled from its next word once the FIFO has been empty for two cycles with the enable level low
// (any slot), or with the primary's EN low (the primary), so the core's ack completes. While the level is low and
// the feeder is idle it pops and discards a queued entry in every cycle; an answer among them goes with its DATA,
// and its request stands for SPI. At a taken ENABLE the cause, every answered mark and every offered record are
// cleared: the links are at rest, so nothing is queued or in flight.
// An ABORT store that finds the FIFO full is held and enters in the first cycle with room; until then every other
// store is refused as by a full FIFO. The enable's fall drops it.
// Crossings: the FIFO's Gray pointers, the control bits (en, test, xen), the feeder's status levels and DIAG toggles
// (sys_lwx), each through lw_sync; the records through lw_xfer (lw_cdc.sv).
// DIAG: each done event and the SUM of the last data frame cross together in one lw_xfer word, {done count, SUM},
// so FRAMES, TSTAMP_DONE and SUM move on the same clk_100m edge. Events that come while a word is in flight
// coalesce into the next word: FRAMES adds them all, SUM shows the latest data frame's.
// Bridge side has no reset and answers in fixed cycles (no back-pressure); a full FIFO drops the store and
// counts it.
// DIAG=0 (LW_DIAG=0, shipping) compiles out STATUS[31:4], DROPS, COUNTS, GEN, TSTAMP_*, CYCLES, SUM and TEST:
// they read 0 and CTRL.TEST/CLR are ignored. CAUSE, REQ, LBA, BEGIN, CTRL.EN/ABORT, STATUS[3:0], ACCEPT, ID stay.

module lw_sector #(parameter [7:0] SLOT = 1, parameter [9:0] FRAME = 588, parameter [2:0] BLKSZ = 3,
                   parameter AW = 9, parameter DIAG = 1, parameter [3:0] XSLOT0 = 2, parameter [3:0] XSLOT1 = 3,
                   parameter [9:0] XFRAME = 256)
(
	input wire        clk,          // clk_100m

	input wire [12:0] raddr,        // block-relative
	input wire        rsel,
	output reg [31:0] rdata,
	input wire [12:0] waddr,
	input wire        wsel,
	input wire        write,
	input wire [31:0] wdata,
	input wire  [3:0] be,
	input wire        hen,          // the host enable
	input wire        take,         // an ENABLE store is taken in this cycle
	output wire       f_open,       // the enable link: stores are taken (OPEN)
	output wire       f_owned,      // OWNED
	output wire       f_idle,       // the link lets an ENABLE through
	input wire  [2:0] wired,        // {X1, X0, primary}: the slot's mux is present; an EN bit is set only if wired

	output wire       doorbell,     // level, high while CAUSE != 0

	input wire        clk_sys,
	input wire        c_reset,      // consumer reset (zero-fills an open frame)
	input wire        c_req,
	input wire [31:0] c_lba,
	input wire        hps_ack,      // the SPI path's ack for this slot
	output reg        en_sys = 0,   // mux state: 1 = this block owns the consumer's ack/write/data
	output wire       o_ack,
	output reg        o_wr   = 0,
	output reg [15:0] o_dat  = 0,
	output reg  [8:0] o_adr  = 0,   // save-slot strobe address: {word, half}, hps_io's sd_buff_addr order

	input wire  [1:0] x_rd,         // save slots X0/X1 (clk_sys): read and write requests, LBA, SPI ack
	input wire  [1:0] x_wr,
	input wire [13:0] x_lba,        // {X1, X0}, 7 bits each
	input wire  [1:0] x_hps_ack,
	output reg  [1:0] x_en_sys = 0,
	output wire [1:0] x_ack
);

localparam [15:0] SDSTAT = {1'b1, 6'd0, BLKSZ, SLOT[3:0], 1'b0, 1'b1};
localparam [3:0]  PSLOT  = SLOT[3:0];
localparam [1:0]  T_DATA = 2'd0, T_BEGIN = 2'd1, T_ABORT = 2'd2;

// Servable geometry: frames of 1-1023 words; a save-slot frame of at most 256 words (the 9-bit strobe address).
// synthesis translate_off
initial if (FRAME == 0 || XFRAME == 0 || XFRAME > 256) begin
	$display("lw_sector: unservable geometry FRAME=%0d XFRAME=%0d", FRAME, XFRAME);
	$finish;
end
// synthesis translate_on

function automatic [2:0] onehot(input [1:0] k);
	onehot = (k == 2'd0) ? 3'b001 : (k == 2'd1) ? 3'b010 : (k == 2'd2) ? 3'b100 : 3'b000;
endfunction

/////////////////////////////// FIFO (M10K) ///////////////////////////////

localparam DEPTH = 1 << AW;

(* ramstyle = "M10K" *) reg [43:0] fram [0:DEPTH-1];

reg  [AW:0] wptr = 0;
reg  [AW:0] rptr = 0;
(* preserve *) reg [AW:0] sector_w_lwgray = 0;   // clk_100m
(* preserve *) reg [AW:0] sector_r_lwgray = 0;   // clk_sys
wire [AW:0] rg_s, wg_s;

function automatic [AW:0] g2b(input [AW:0] g);
	integer n;
	begin
		g2b[AW] = g[AW];
		for (n = AW - 1; n >= 0; n = n - 1) g2b[n] = g2b[n + 1] ^ g[n];
	end
endfunction

wire [AW:0] used  = wptr - g2b(rg_s);
wire        ffull = used[AW];

/////////////////////////////// clk_100m //////////////////////////////////

localparam [1:0] F_IDLE = 2'd0, F_ACK = 2'd1, F_STREAM = 2'd2, F_FILL = 2'd3;

reg   [1:0] fs = F_IDLE;
reg   [9:0] fexp = 0;
reg         em = 0, qv = 0, tmode = 0;
reg   [1:0] ph = 0, ackc = 0;
reg  [15:0] hi = 0;
reg  [43:0] q;
reg  [19:0] fsum = 0;
reg   [3:0] evt = 0;                // {reset_fill, abort, skip, reject} toggles (DIAG)
reg   [3:0] dcnt = 0, dsent = 0;    // DIAG: done events counted, and the count last launched to clk_100m
reg  [19:0] dsum = 0;               // DIAG: the last data frame's SUM

reg   [2:0] cause = 0;              // CAUSE: bit k = slot k's request opened or re-opened
reg  [31:0] gen = 0, tsreq = 0, tsdone = 0, cycles = 0;
reg         rq_off0 = 0;            // the primary's last record: offered, LBA
reg  [31:0] rq_lba = 0;
reg   [1:0] rq_xoff = 0, rq_xrd = 0, rq_xwr = 0;   // the save slots' last records: offered, rd, wr, LBA
reg  [13:0] rq_xlba = 0;
(* preserve *) reg       en_lwx   = 0;   // CTRL[0]
(* preserve *) reg       test_lwx = 0;   // CTRL[3]
(* preserve *) reg [1:0] xen_lwx  = 0;   // CTRL[4+XSLOT0], CTRL[4+XSLOT1]
reg   [2:0] ans = 0;                // {X1, X0, primary}: a BEGIN/WACK for the offered request entered the FIFO
reg   [8:0] btag = 0;               // per slot, 3 bits: the count of its BEGIN/WACK entries, carried in each entry
reg         abort_pend = 0;         // an ABORT store found the FIFO full: it enters in the first cycle with room
reg   [9:0] flen_w = FRAME;
reg   [9:0] exp_w = 0, fdrops = 0;
reg         bfail = 0;
reg  [15:0] drops = 0, frames = 0;
reg   [7:0] hw = 0;
reg   [7:0] n_rej = 0, n_skip = 0, n_abort = 0, n_rfill = 0;
reg  [19:0] sum = 0;
reg   [3:0] dseen = 0;              // the done count of the last word taken
wire        f_lvl, f_fall, hen_s, rest;   // the enable link's level and its fall; on clk_sys, the level and rest

// clk_sys -> clk_100m: the feeder's status levels and its DIAG event toggles
(* preserve *) reg [6:0] sys_lwx = 0;
wire  [6:0] sb1;
reg   [6:0] sb2 = 0;
wire [23:0] dn_q;                   // DIAG: {done count, SUM} from the feeder
wire        dn_v, dn_busy;
wire  [3:0] dn_add = dn_v ? dn_q[23:20] - dseen : 4'd0;

lw_sync #(.W(AW + 1), .GRAY(1)) rg_sync (.clk(clk), .d(sector_r_lwgray), .q(rg_s));
lw_sync #(.W(7)) sys_sync (.clk(clk), .d(sys_lwx), .q(sb1));
wire  [2:0] st_s = sb1[2:0];          // {streaming, ack, en_sys}
wire  [3:0] ev   = sb1[6:3] ^ sb2[6:3]; // {reset_fill, abort, skip, reject}

// clk_sys -> clk_100m: the request records, {slot[41:40], offered, new, rej, tag[36:34], wr, rd, lba[31:0]}
wire [41:0] rc_q;
wire        rc_v;
wire  [2:0] rc_sel = {3{rc_v}} & onehot(rc_q[41:40]);
wire        rc_off = rc_q[39], rc_new = rc_q[38], rc_rej = rc_q[37];
wire  [2:0] rc_tag = rc_q[36:34];

wire        rreg  = rsel & ~raddr[12] & ~|raddr[11:7];
wire        wreg  = wsel & write & ~waddr[12] & ~|waddr[11:7] & &be;
wire        wdat  = wsel & write &  waddr[12] & &be & (waddr[11:2] < flen_w);   // the OPEN frame's length
wire  [4:0] widx  = waddr[6:2];
wire  [3:0] ws    = {2'b00, waddr[3:2]};          // BEGIN_S/WACK_S slot number
wire        w_cause = wreg && widx == 5'h00;
wire        w_ctrl  = wreg && widx == 5'h04;
wire        w_bs    = wreg && (widx == 5'h03 || widx[4:2] == 3'b100);   // BEGIN, BEGIN_S[s]
wire        w_wk    = wreg && widx[4:2] == 3'b101;                       // WACK_S[s]
wire  [3:0] bslot   = (widx == 5'h03) ? PSLOT : ws;
wire        b_p     = w_bs && bslot == PSLOT;
wire        b_x0    = (w_bs | w_wk) && ws == XSLOT0 && widx != 5'h03;
wire        b_x1    = (w_bs | w_wk) && ws == XSLOT1 && widx != 5'h03;
wire        w_begin = b_p | b_x0 | b_x1;
wire  [1:0] bsel    = b_x0 ? 2'd1 : b_x1 ? 2'd2 : 2'd0;
wire  [9:0] blen    = w_wk ? 10'd0 : b_p ? FRAME : XFRAME;
wire        w_abort = w_ctrl & wdata[1];
wire        clr     = w_ctrl & wdata[2];
// a store's entry is taken only with room and no ABORT held; the held ABORT takes the first cycle with room
wire        a_go  = abort_pend & ~ffull;
wire        p_ok  = (w_begin | w_abort | wdat) & ~ffull & ~abort_pend;
wire        p_bad = (w_begin | wdat) & ~p_ok;           // a store refused: dropped and counted
wire        fwe   = a_go | p_ok;

wire  [2:0] bent  = {b_x1, b_x0, b_p} & {3{p_ok}};
wire  [8:0] btag1 = {btag[8:6] + 3'd1, btag[5:3] + 3'd1, btag[2:0] + 3'd1};
wire  [2:0] etag  = (bsel == 2'd1) ? btag1[5:3] : (bsel == 2'd2) ? btag1[8:6] : btag1[2:0];
// a reject of the slot's LATEST answer re-opens the request: REQ shows it again and its cause bit rises
wire  [2:0] latest = {rc_tag == btag[8:6], rc_tag == btag[5:3], rc_tag == btag[2:0]};
wire  [2:0] reopen = rc_sel & {3{rc_rej}} & latest & ~bent;
wire  [2:0] unans  = (rc_sel & {3{rc_new | ~rc_off}}) | reopen;
wire  [2:0] raise  = rc_sel & {3{rc_off}} & ({3{rc_new}} | reopen);
wire        fire   = |raise;
wire        w_en  = wdata[0] & wired[0];                                     // the EN bits a store sets
wire  [1:0] w_xen = {wdata[4 + XSLOT1], wdata[4 + XSLOT0]} & wired[2:1];
wire  [2:0] clr_c = {3{w_cause}} & wdata[2:0];
wire [43:0] entry = a_go ? {T_ABORT, 42'd0} : w_begin ? {T_BEGIN, 4'd0, etag, w_wk, bsel, wdata} :
                    w_abort ? {T_ABORT, 42'd0} : {T_DATA, waddr[11:2], wdata};

// REQ: the lowest-index slot whose EN is set, whose record shows its request offered, and that is not answered
wire  [2:0] cand   = {xen_lwx, en_lwx} & {rq_xoff, rq_off0} & ~ans;
wire        any    = |cand;
wire  [1:0] pick   = cand[0] ? 2'd0 : cand[1] ? 2'd1 : 2'd2;
wire [15:0] xst0   = {1'b1, 6'd0, BLKSZ, XSLOT0, rq_xwr[0], rq_xrd[0]};
wire [15:0] xst1   = {1'b1, 6'd0, BLKSZ, XSLOT1, rq_xwr[1], rq_xrd[1]};
wire [15:0] pst    = !any ? 16'd0 : pick == 2'd0 ? SDSTAT : pick == 2'd1 ? xst0 : xst1;

always @(posedge clk) if (fwe) fram[wptr[AW-1:0]] <= entry;

function automatic [7:0] sat8(input [7:0] v, input inc);
	sat8 = (inc && ~&v) ? v + 8'd1 : v;
endfunction

always @(posedge clk) begin
	if (DIAG) cycles <= cycles + 32'd1;
	sb2 <= sb1;

	// the request records, the cycle after each lands; an ENABLE finds every link at rest and nothing offered
	if (rc_sel[0]) begin rq_off0 <= rc_off; rq_lba <= rc_q[31:0]; end
	if (rc_sel[1]) begin rq_xoff[0] <= rc_off; {rq_xwr[0], rq_xrd[0]} <= rc_q[33:32]; rq_xlba[6:0] <= rc_q[6:0]; end
	if (rc_sel[2]) begin rq_xoff[1] <= rc_off; {rq_xwr[1], rq_xrd[1]} <= rc_q[33:32]; rq_xlba[13:7] <= rc_q[6:0]; end
	if (take) begin rq_off0 <= 0; rq_xoff <= 0; end

	// CAUSE: a raise beats a concurrent clear; an ENABLE clears the word
	gen   <= gen + fire;
	cause <= raise | (cause & ~clr_c & ~{3{take}});
	// answered: a BEGIN/WACK for the slot entered the FIFO; it ends with a record that shows the request new or
	// withdrawn, or rejects that latest answer, and at an ENABLE (one discarded without the enable leaves its
	// request standing)
	ans  <= bent | (ans & ~unans & ~{3{take}});
	btag <= {bent[2] ? btag1[8:6] : btag[8:6], bent[1] ? btag1[5:3] : btag[5:3], bent[0] ? btag1[2:0] : btag[2:0]};
	if (DIAG && fire) tsreq <= cycles;

	if (DIAG && dn_v) begin
		dseen <= dn_q[23:20];
		sum   <= dn_q[19:0];
		if (|dn_add) tsdone <= cycles;
	end

	if (f_fall || !f_lvl) begin
		en_lwx   <= 0;
		xen_lwx  <= 0;
		test_lwx <= 0;
	end
	else if (w_ctrl) begin
		en_lwx   <= w_en;
		xen_lwx  <= w_xen;
		test_lwx <= DIAG ? wdata[3] : 1'b0;
	end

	// a full FIFO holds an ABORT (until the enable falls) and drops and counts any other store
	if (!hen) abort_pend <= 0;
	else abort_pend <= (abort_pend & ~a_go) | (w_abort & ffull & ~abort_pend);

	// store path: never back-pressured
	if (p_bad) begin
		if (DIAG && ~&drops) drops <= drops + 16'd1;
		if (~&fdrops) fdrops <= fdrops + 10'd1;
	end
	if (fwe) wptr <= wptr + 1'd1;
	sector_w_lwgray <= wptr ^ (wptr >> 1);

	if (w_begin) begin
		exp_w  <= 0;
		flen_w <= blen;
		bfail  <= ~p_ok;
		fdrops <= p_ok ? 10'd0 : 10'd1;
	end
	else if (wdat && p_ok && waddr[11:2] == exp_w) exp_w <= exp_w + 10'd1;

	if (DIAG) begin
		frames <= (clr ? 16'd0 : frames) + {12'd0, dn_add};   // a done arriving with CLR counts after it
		if (clr) begin
			drops   <= 0;
			hw      <= 0;
			{n_rej, n_skip, n_abort, n_rfill} <= 0;
		end
		else begin
			if (used > hw) hw <= (used > 255) ? 8'd255 : used[7:0];
			n_rej   <= sat8(n_rej,   ev[0]);
			n_skip  <= sat8(n_skip,  ev[1]);
			n_abort <= sat8(n_abort, ev[2]);
			n_rfill <= sat8(n_rfill, ev[3]);
		end
	end
end

assign doorbell = |cause;

// The enable link. Drain: no ABORT held, every stored word visible to clk_sys.
lw_fence fence (.clk(clk), .hen(hen), .take(take),
                .drain(!abort_pend && sector_w_lwgray == (wptr ^ (wptr >> 1))), .fin(1'b1),
                .open(f_open), .owned(f_owned), .idle(f_idle), .lvl(f_lvl), .fall(f_fall),
                .clk_sys(clk_sys), .rest(rest), .hen_s(hen_s));

wire ok = (exp_w == flen_w) & ~bfail;

always @(*) begin
	rdata = 32'd0;
	if (rreg) case (raddr[6:2])
		5'h00: rdata = {29'd0, cause};                     // CAUSE: one event bit per slot
		5'h01: rdata = {any, 15'd0, pst};
		5'h02: rdata = rq_lba;
		5'h04: begin
			rdata = {28'd0, test_lwx, 2'b00, en_lwx};
			rdata[4 + XSLOT0] = xen_lwx[0];
			rdata[4 + XSLOT1] = xen_lwx[1];
		end
		5'h05: rdata = DIAG ? {frames, hw, 5'd0, st_s} : {29'd0, st_s};
		5'h06: rdata = {ok, 5'd0, fdrops, 6'd0, exp_w};
		5'h07: rdata = DIAG ? {16'd0, drops} : 32'd0;
		5'h08: rdata = DIAG ? {n_rej, n_skip, n_abort, n_rfill} : 32'd0;
		5'h09: rdata = DIAG ? gen : 32'd0;
		5'h0A: rdata = DIAG ? tsreq : 32'd0;
		5'h0B: rdata = DIAG ? tsdone : 32'd0;
		5'h0C: rdata = DIAG ? cycles : 32'd0;
		5'h0D: rdata = DIAG ? {12'd0, sum} : 32'd0;
		5'h0E: rdata = 32'h32434553;     // "SEC2"
		5'h18, 5'h19, 5'h1A, 5'h1B:      // LBA_S[s]
			rdata = ({2'b00, raddr[3:2]} == PSLOT)  ? rq_lba :
			        ({2'b00, raddr[3:2]} == XSLOT0) ? {25'd0, rq_xlba[6:0]} :
			        ({2'b00, raddr[3:2]} == XSLOT1) ? {25'd0, rq_xlba[13:7]} : 32'd0;
		default: rdata = 32'd0;
	endcase
end

/////////////////////////////// clk_sys: feeder ///////////////////////////

wire [3:0] cs1;
lw_sync #(.W(4)) ctl_sync (.clk(clk_sys), .d({xen_lwx, test_lwx, en_lwx}), .q(cs1));
lw_sync #(.W(AW + 1), .GRAY(1)) wg_sync (.clk(clk_sys), .d(sector_w_lwgray), .q(wg_s));

wire       en_s = cs1[0], test_s = cs1[1];
wire [1:0] xen_s = cs1[3:2];

wire       rempty = (sector_r_lwgray == wg_s);
wire       fidle  = (fs == F_IDLE);
wire       dis    = ~hen_s & fidle & ~em & (rptr != g2b(wg_s));   // without the enable: discard, every cycle
wire       pop    = (~rempty & ~qv & (fs != F_FILL) & (fs != F_ACK) & (~em | ph == 2'd3)) | dis;
wire [1:0] qt = q[43:42];
wire [9:0] qi = q[41:32];
wire [31:0] qd = q[31:0];

reg  [1:0] fsel  = 0;               // the slot the open frame serves: 0 primary, 1 X0, 2 X1
reg        fwack = 0;               // the open frame is a WACK: ack only, no strobes
wire [9:0] flen  = (fsel == 2'd0) ? FRAME : XFRAME;
wire [1:0] bs    = qi[1:0];
wire       bwk   = qi[2];
wire [2:0] btg   = qi[5:3];
wire       bmatch = (bs == 2'd0) ? (!bwk && en_sys && c_req && qd == c_lba) :
                    (bs == 2'd1) ? (x_en_sys[0] && (bwk ? x_wr[0] : x_rd[0]) && qd == {25'd0, x_lba[6:0]}) :
                    (bs == 2'd2) ? (x_en_sys[1] && (bwk ? x_wr[1] : x_rd[1]) && qd == {25'd0, x_lba[13:7]}) : 1'b0;

always @(posedge clk_sys) q <= fram[rptr[AW-1:0]];

reg  [2:0] fack = 0;                // per-slot ack: {X1, X0, primary}
reg  [1:0] mt   = 0;                // cycles the FIFO has read empty with EN low, saturating at 2
reg  [1:0] ml   = 0;                // cycles the FIFO has read empty with the enable level low, saturating at 2
// the frame in flight will get no more data: the primary's EN fell, or the enable level did (any slot); fill it
// from its next word
wire       cut  = (fs == F_STREAM || (fs == F_ACK && !fwack)) && rempty && !qv && !em &&
                  ((fsel == 2'd0 && !en_s && mt == 2'd2) || (!hen_s && ml == 2'd2));

// An answer taken with the enable and not served: in idle, one whose slot, direction or LBA is not the live
// request's (outside TEST); while a frame streams, any BEGIN (the frame is zero-filled). Its slot's record says so.
wire       q_beg  = qv && hen_s && qt == T_BEGIN;
wire       rj     = q_beg && ((fidle && !(test_s && !bwk) && !bmatch) || fs == F_STREAM);
wire [2:0] rj_set = {3{rj}} & onehot(bs);

// The request records. offered: the slot's mux on this block, the core's request raised, the slot's ack low and
// two cycles past its fall (rearm), registered.
wire [2:0] up    = {x_rd[1] | x_wr[1], x_rd[0] | x_wr[0], c_req};
reg  [2:0] offd  = 0;               // offered
reg  [2:0] opn   = 0;               // new: offered rose since the slot's last record
reg  [2:0] rej   = 0;               // an answer was rejected since the slot's last record ...
reg  [8:0] rtag  = 0;               // ... and its entry's tag, 3 bits per slot
reg  [2:0] dirty = 0;
reg  [5:0] rearm = 0;               // per slot, 2 bits: the slot's ack (LW or SPI) in each of the two cycles before
reg  [1:0] rlast = 0;               // the slot of the last record launched
wire [2:0] rz    = {rearm[5:4] == 2'd0, rearm[3:2] == 2'd0, rearm[1:0] == 2'd0};
wire [2:0] off_d = {x_en_sys, en_sys} & up & ~fack & rz;
wire       rc_busy;
wire       rc_go = |dirty & ~rc_busy;
// round robin: the first dirty slot after the last one launched
wire [1:0] rs    = (rlast == 2'd0) ? (dirty[1] ? 2'd1 : dirty[2] ? 2'd2 : 2'd0) :
                   (rlast == 2'd1) ? (dirty[2] ? 2'd2 : dirty[0] ? 2'd0 : 2'd1) :
                                     (dirty[0] ? 2'd0 : dirty[1] ? 2'd1 : 2'd2);
wire [2:0] lsel  = {3{rc_go}} & onehot(rs);
wire [2:0] rtg_s = (rs == 2'd1) ? rtag[5:3] : (rs == 2'd2) ? rtag[8:6] : rtag[2:0];
wire [1:0] rwr_s = (rs == 2'd1) ? {x_wr[0], x_rd[0]} : (rs == 2'd2) ? {x_wr[1], x_rd[1]} : {1'b0, c_req};
wire [31:0] rlba = (rs == 2'd1) ? {25'd0, x_lba[6:0]} : (rs == 2'd2) ? {25'd0, x_lba[13:7]} : c_lba;
wire [41:0] rec  = {rs, offd[rs], opn[rs], rej[rs], rtg_s, rwr_s, rlba};

lw_xfer #(.W(42)) rec_x(.s_clk(clk_sys), .s_go(rc_go), .s_d(rec), .s_busy(rc_busy),
                        .d_clk(clk), .d_q(rc_q), .d_v(rc_v));

// rest, for the enable link: the level low, every EN and every mux on SPI, the feeder idle with its acks low and an
// empty pipeline, the FIFO empty and its Gray source current, no record or done word in flight or owed
wire       dn_idle;
assign rest = !hen_s && cs1 == 4'd0 && !en_sys && x_en_sys == 2'd0 && fidle && !qv && !em &&
              fack == 3'd0 && !o_wr && rempty && sector_r_lwgray == (rptr ^ (rptr >> 1)) && dn_idle &&
              offd == 3'd0 && dirty == 3'd0 && !rc_busy;

// DIAG builds only: {done count, the last data frame's SUM} to clk_100m, launched whenever the count moved
wire dn_go = dcnt != dsent;
generate
if (DIAG) begin : donex
	wire [23:0] dn_word = {dcnt, dsum};
	lw_xfer #(.W(24)) done_x(.s_clk(clk_sys), .s_go(dn_go), .s_d(dn_word), .s_busy(dn_busy),
	                         .d_clk(clk), .d_q(dn_q), .d_v(dn_v));
end
else begin : nodone
	assign dn_q = 24'd0, dn_v = 1'b0, dn_busy = 1'b1;
end
endgenerate
assign dn_idle = !DIAG || (!dn_go && !dn_busy);
assign o_ack = fack[0];
assign x_ack = fack[2:1];

always @(posedge clk_sys) begin
	sys_lwx <= {DIAG ? evt : 4'd0, fs == F_STREAM || fs == F_FILL || fs == F_ACK, o_ack | |x_ack, en_sys};

	if (DIAG && dn_go && !dn_busy) dsent <= dcnt;

	// the records: a change in a launch cycle keeps the slot dirty, its new state for the next record
	offd  <= off_d;
	opn   <= (off_d & ~offd) | (opn & ~lsel);
	rej   <= rj_set | (rej & ~lsel);
	dirty <= (off_d ^ offd) | rj_set | (dirty & ~lsel);
	if (rj_set[0]) rtag[2:0] <= btg;
	if (rj_set[1]) rtag[5:3] <= btg;
	if (rj_set[2]) rtag[8:6] <= btg;
	if (rc_go) rlast <= rs;
	rearm <= {rearm[4], fack[2] | x_hps_ack[1], rearm[2], fack[1] | x_hps_ack[0], rearm[0], fack[0] | hps_ack};

	qv <= pop;
	mt <= (!rempty || en_s) ? 2'd0 : (mt == 2'd2) ? 2'd2 : mt + 2'd1;
	ml <= (!rempty || hen_s) ? 2'd0 : (ml == 2'd2) ? 2'd2 : ml + 2'd1;
	if (pop) rptr <= rptr + 1'd1;
	sector_r_lwgray <= rptr ^ (rptr >> 1);

	o_wr <= 0;
	if (rj) evt[0] <= ~evt[0];

	// each slot's mux follows its EN only while the feeder is off that slot and its SPI ack is low
	if ((fidle || fsel != 2'd0) && !hps_ack) en_sys <= en_s;
	if ((fidle || fsel != 2'd1) && !x_hps_ack[0]) x_en_sys[0] <= xen_s[0];
	if ((fidle || fsel != 2'd2) && !x_hps_ack[1]) x_en_sys[1] <= xen_s[1];

	if (fs == F_ACK) begin
		ackc <= ackc - 1'd1;
		if (ackc == 1) begin
			if (fwack) begin
				fack[fsel] <= 1'b0;
				fs     <= F_IDLE;
				if (DIAG) dcnt <= dcnt + 4'd1;
			end
			else fs <= F_STREAM;
		end
	end

	if (fs == F_FILL && !em) begin
		o_wr  <= ~tmode;
		o_dat <= 0;
		o_adr <= {fexp[7:0], 1'b0};
		hi    <= 0;
		em    <= 1;
		ph    <= 1;
	end

	if (fidle) fack <= 3'd0;           // idle drives every ack low

	if (qv) case (fs)
		// without the enable an entry is discarded (an answer's request stands for SPI); a rejected answer is
		// dropped with the DATA behind it, which is popped here and does nothing, as an ABORT does
		F_IDLE:
			if (hen_s && qt == T_BEGIN) begin
				fexp <= 0;
				if (DIAG) fsum <= 0;
				if (test_s && !bwk) begin
					tmode <= 1;
					fsel  <= bs;
					fwack <= 0;
					fs    <= F_STREAM;
				end
				else if (bmatch) begin
					tmode <= 0;
					fsel  <= bs;
					fwack <= bwk;
					fack[bs] <= 1'b1;
					ackc  <= 2;
					fs    <= F_ACK;
				end
			end

		F_STREAM:
			if (qt != T_DATA) begin
				fs     <= F_FILL;
				evt[2] <= ~evt[2];
			end
			else if (qi == fexp) begin
				o_wr  <= ~tmode;
				o_dat <= qd[15:0];
				o_adr <= {fexp[7:0], 1'b0};
				hi    <= qd[31:16];
				em    <= 1;
				ph    <= 1;
				if (DIAG) fsum <= fsum + qd[31:24] + qd[23:16] + qd[15:8] + qd[7:0];
			end
			else evt[1] <= ~evt[1];

		default: ;
	endcase

	if (em) case (ph)
		2'd1: ph <= 2;
		2'd2: begin
			o_wr  <= ~tmode;
			o_dat <= hi;
			o_adr <= {fexp[7:0], 1'b1};
			ph    <= 3;
		end
		2'd3: begin
			em <= 0;
			ph <= 0;
			if (fexp >= flen - 1'd1) begin   // or past it: a frame ends from any count
				fexp   <= 0;
				fack[fsel] <= 1'b0;
				fs     <= F_IDLE;
				if (DIAG) begin
					dcnt <= dcnt + 4'd1;
					dsum <= fsum;
				end
			end
			else fexp <= fexp + 1'd1;
		end
		default: begin
			em <= 0;
			ph <= 0;
		end
	endcase

	if (cut) fs <= F_FILL;

	if (c_reset) begin
		if (fs == F_ACK && fwack) begin
			fack[fsel] <= 1'b0;
			fs <= F_IDLE;
		end
		else if (fs == F_STREAM || fs == F_ACK) begin
			fs     <= F_FILL;
			evt[3] <= ~evt[3];
		end
	end
end

endmodule

`default_nettype wire
`endif
`endif
