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

// SECTOR block (LW directory kind 2, minor 1, "SEC2"): one hps_io read slot (the primary) plus two
// save slots (X0, X1) served over the LW bridge by ONE feeder. A save slot READ is a BEGIN_S frame of
// XFRAME words with a strobe address; a save slot WRITE is answered by WACK_S (ack, no strobes): the HPS
// takes the bytes from the core's declared DDR image. Fixed priority: primary, X0, X1. A slot's EN bit can be set
// only while the slot is wired (its mux's presence bit on HPS_BUS reads 1).
// clk_100m side: the register map, the BLK doorbell cause and the store path into a dual-clock FIFO.
// CAUSE: bit 0, raised by each request rise (and each re-open after a reject); a full-word store with bit 0 set
// clears it, and a raise in the store's cycle wins. Reads change nothing: the request word is REQ's.
// clk_sys side: the feeder that re-creates hps_io's ack + 16-bit write strobes for each slot's consumer.
// The host enable (hen, the window's en_q; it falls at an ID view, a core reset through GPO or an HPS reset)
// reaches the feeder through the block's enable link alone (lw_fence, lw_cdc.sv): stores reach this block only
// while the link is open, which is after the feeder has seen the enable.
// Every EN, TEST included, is cleared in the cycle the link's level falls, four cycles after the last stored word
// has been launched toward clk_sys, so each slot's mux returns to SPI once the feeder is off it and its SPI ack is
// low, and a frame whose every word was stored still completes with its data. A frame in flight that waits for
// data is zero-filled from its next word once the FIFO has been empty for two cycles with the enable level low
// (any slot), or with the primary's EN low (the primary), so the core's ack completes. While the level is low and
// the feeder is idle it pops and discards a queued entry in every cycle; an answer among them goes with its DATA,
// and its request stands for SPI. At a taken ENABLE the cause and every answered mark are cleared; an EN rise raises the
// cause for a request that stands.
// Crossings: the FIFO's Gray pointers, the control bits (en, test, xen) and the feeder's levels and toggles
// (sys_lwx), each through lw_sync (lw_cdc.sv). The request LBAs are latched on clk_100m straight from the core's
// clk_sys registers, relying on their being stable while the request stands: the one such read tb/static.sh lists.
// DIAG: each done event and the SUM of the last data frame cross together in one lw_xfer word, {done count, SUM},
// so FRAMES, TSTAMP_DONE and SUM move on the same clk_100m edge. Events that come while a word is in flight
// coalesce into the next word: FRAMES adds them all, SUM shows the latest data frame's.
// Bridge side has no reset and answers in fixed cycles (no back-pressure); a full FIFO drops the store and
// counts it. Clock-ratio bound of the reject path: docs/LW_BRIDGE.md.
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

localparam F_IDLE = 3'd0, F_ACK = 3'd1, F_STREAM = 3'd2, F_FILL = 3'd3, F_REJECT = 3'd4;

reg   [2:0] fs = F_IDLE;
reg   [9:0] fexp = 0;
reg         em = 0, qv = 0, tmode = 0;
reg   [1:0] ph = 0, ackc = 0;
reg  [15:0] hi = 0;
reg  [43:0] q;
reg  [19:0] fsum = 0;
reg   [3:0] evt = 0;                // {reset_fill, abort, skip, reject} toggles
reg   [3:0] dcnt = 0, dsent = 0;    // DIAG: done events counted, and the count last launched to clk_100m
reg  [19:0] dsum = 0;               // DIAG: the last data frame's SUM

reg         pending = 0;
reg  [31:0] gen = 0, tsreq = 0, tsdone = 0, cycles = 0;
reg  [31:0] lba = 0;
(* preserve *) reg       en_lwx   = 0;   // CTRL[0]
(* preserve *) reg       test_lwx = 0;   // CTRL[3]
(* preserve *) reg [1:0] xen_lwx  = 0;   // CTRL[4+XSLOT0], CTRL[4+XSLOT1]
reg   [2:0] ans = 0;                // {X1, X0, primary}: a BEGIN/WACK for the open request entered the FIFO
reg   [2:0] btag = 0;               // per slot: toggled by each BEGIN/WACK entering, carried in its entry
reg  [13:0] xlba = 0;
reg   [9:0] flen_w = FRAME;
reg   [9:0] exp_w = 0, fdrops = 0;
reg         bfail = 0;
reg  [15:0] drops = 0, frames = 0;
reg   [7:0] hw = 0;
reg   [7:0] n_rej = 0, n_skip = 0, n_abort = 0, n_rfill = 0;
reg  [19:0] sum = 0;
reg   [3:0] dseen = 0;              // the done count of the last word taken
wire        f_lvl, f_fall, hen_s, rest;   // the enable link's level and its fall; on clk_sys, the level and rest

// clk_sys -> clk_100m: request levels, status bits, event toggles, the rejected answer's {tag, slot}
(* preserve *) reg [15:0] sys_lwx = 0;
wire [15:0] sb1;
reg  [15:0] sb2 = 0;
wire [23:0] dn_q;                   // DIAG: {done count, SUM} from the feeder
wire        dn_v, dn_busy;
wire  [3:0] dn_add = dn_v ? dn_q[23:20] - dseen : 4'd0;

lw_sync #(.W(AW + 1), .GRAY(1)) rg_sync (.clk(clk), .d(sector_r_lwgray), .q(rg_s));
lw_sync #(.W(16)) sys_sync (.clk(clk), .d(sys_lwx), .q(sb1));
wire        req_s = sb1[0];
wire        req_rise = sb1[0] & ~sb2[0];
wire  [3:0] st_s = sb1[4:1];          // {rejecting, streaming, ack, en_sys}
wire  [3:0] ev   = sb1[8:5] ^ sb2[8:5]; // {reset_fill, abort, skip, reject}
wire  [1:0] xrd_s = {sb1[11], sb1[9]};
wire  [1:0] xwr_s = {sb1[12], sb1[10]};
wire  [1:0] xreq_s = xrd_s | xwr_s;
wire  [1:0] xrise  = xreq_s & ~({sb2[12], sb2[10]} | {sb2[11], sb2[9]});
wire  [1:0] rjslot = sb1[14:13];   // the rejected BEGIN/WACK's slot and tag: stable a clk_sys cycle
wire        rjtag  = sb1[15];      // before ev[0] (the reject toggle, synced in every build) moves
wire  [2:0] rjsel  = (rjslot == 2'd0) ? 3'b001 : (rjslot == 2'd1) ? 3'b010 : (rjslot == 2'd2) ? 3'b100 : 3'b000;
wire  [2:0] cand   = {xreq_s & xen_lwx, req_s & en_lwx} & ~ans;
wire        any    = |cand;
wire  [1:0] pick   = cand[0] ? 2'd0 : cand[1] ? 2'd1 : 2'd2;
wire [15:0] xst0   = {1'b1, 6'd0, BLKSZ, XSLOT0, xwr_s[0], xrd_s[0]};
wire [15:0] xst1   = {1'b1, 6'd0, BLKSZ, XSLOT1, xwr_s[1], xrd_s[1]};
wire [15:0] pst    = !any ? 16'd0 : pick == 2'd0 ? SDSTAT : pick == 2'd1 ? xst0 : xst1;

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
wire        push  = w_begin | w_abort | wdat;
wire        fwe   = push & ~ffull;

wire  [2:0] bent  = {b_x1, b_x0, b_p} & {3{fwe}};
wire  [2:0] reqs  = {xreq_s, req_s};
// a reject of the LATEST answer re-opens the request: REQ shows it again and the doorbell re-rings
wire        rjlast = (rjtag == ((rjslot == 2'd0) ? btag[0] : (rjslot == 2'd1) ? btag[1] : btag[2]));
wire  [2:0] unans = ans & rjsel & {3{ev[0] & rjlast}} & ~bent;
// an EN rise rings for a request that stands unanswered
wire        w_en  = wdata[0] & wired[0];                                     // the EN bits a store sets
wire  [1:0] w_xen = {wdata[4 + XSLOT1], wdata[4 + XSLOT0]} & wired[2:1];
wire        en_up = w_ctrl & w_en & ~en_lwx & req_s & ~ans[0];
wire  [1:0] xen_up = {2{w_ctrl}} & w_xen & ~xen_lwx & xreq_s & ~ans[2:1];
wire        fire  = req_rise | |(xrise & xen_lwx) | |(unans & reqs & {xen_lwx, en_lwx}) | en_up | |xen_up;
wire        clr_c = w_cause & wdata[0];
wire        etag  = ~btag[bsel];
wire [43:0] entry = w_begin ? {T_BEGIN, 6'd0, etag, w_wk, bsel, wdata} : w_abort ? {T_ABORT, 42'd0} : {T_DATA, waddr[11:2], wdata};

always @(posedge clk) if (fwe) fram[wptr[AW-1:0]] <= entry;

function automatic [7:0] sat8(input [7:0] v, input inc);
	sat8 = (inc && ~&v) ? v + 8'd1 : v;
endfunction

always @(posedge clk) begin
	if (DIAG) cycles <= cycles + 32'd1;
	sb2 <= sb1;

	// doorbell: the request rise raises, a raise beats a concurrent clear
	gen     <= gen + fire;
	pending <= fire | (pending & ~clr_c & ~take);
	// latched at the rise, and again when a reject re-opens the request (its LBA may have moved)
	if (req_rise | unans[0]) lba <= c_lba;
	if (xrise[0] | unans[1]) xlba[6:0]  <= x_lba[6:0];
	if (xrise[1] | unans[2]) xlba[13:7] <= x_lba[13:7];
	// answered: a BEGIN/WACK for the slot entered the FIFO; it ends when the request falls, the feeder
	// rejects that latest answer (a stale LBA after a core reset restarted the request), or at an ENABLE: the
	// links are at rest, so no answer is queued, and one discarded without the enable leaves its request standing
	ans  <= bent | (ans & reqs & ~unans & ~{3{take}});
	btag <= btag ^ bent;
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

	// store path: never back-pressured, a full FIFO drops and counts
	if (push & ffull) begin
		if (DIAG && ~&drops) drops <= drops + 16'd1;
		if (~&fdrops) fdrops <= fdrops + 10'd1;
	end
	if (fwe) wptr <= wptr + 1'd1;
	sector_w_lwgray <= wptr ^ (wptr >> 1);

	if (w_begin) begin
		exp_w  <= 0;
		flen_w <= blen;
		bfail  <= ffull;
		fdrops <= ffull ? 10'd1 : 10'd0;
	end
	else if (wdat && !ffull && waddr[11:2] == exp_w) exp_w <= exp_w + 10'd1;

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

assign doorbell = pending;

// The enable link. Drain: every stored word visible to clk_sys.
lw_fence fence (.clk(clk), .hen(hen), .take(take), .drain(sector_w_lwgray == (wptr ^ (wptr >> 1))), .fin(1'b1),
                .open(f_open), .owned(f_owned), .idle(f_idle), .lvl(f_lvl), .fall(f_fall),
                .clk_sys(clk_sys), .rest(rest), .hen_s(hen_s));

wire ok = (exp_w == flen_w) & ~bfail;

always @(*) begin
	rdata = 32'd0;
	if (rreg) case (raddr[6:2])
		5'h00: rdata = {31'd0, pending};                  // CAUSE: the event bit
		5'h01: rdata = {any, 15'd0, pst};
		5'h02: rdata = lba;
		5'h04: begin
			rdata = {28'd0, test_lwx, 2'b00, en_lwx};
			rdata[4 + XSLOT0] = xen_lwx[0];
			rdata[4 + XSLOT1] = xen_lwx[1];
		end
		5'h05: rdata = DIAG ? {frames, hw, 4'd0, st_s} : {28'd0, st_s};
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
			rdata = ({2'b00, raddr[3:2]} == PSLOT)  ? lba :
			        ({2'b00, raddr[3:2]} == XSLOT0) ? {25'd0, xlba[6:0]} :
			        ({2'b00, raddr[3:2]} == XSLOT1) ? {25'd0, xlba[13:7]} : 32'd0;
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
wire       fidle  = (fs == F_IDLE || fs == F_REJECT);
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
wire       btg   = qi[3];
wire       bmatch = (bs == 2'd0) ? (!bwk && en_sys && c_req && qd == c_lba) :
                    (bs == 2'd1) ? (x_en_sys[0] && (bwk ? x_wr[0] : x_rd[0]) && qd == {25'd0, x_lba[6:0]}) :
                    (bs == 2'd2) ? (x_en_sys[1] && (bwk ? x_wr[1] : x_rd[1]) && qd == {25'd0, x_lba[13:7]}) : 1'b0;

always @(posedge clk_sys) q <= fram[rptr[AW-1:0]];

reg  [1:0] rjs = 0;                    // the last reject's slot and its entry's tag; evt[0] moves
reg        rjt = 0, rjp = 0;           // one cycle after them (rjp)


reg  [2:0] fack = 0;                // per-slot ack: {X1, X0, primary}
reg  [1:0] mt   = 0;                // cycles the FIFO has read empty with EN low, saturating at 2
reg  [1:0] ml   = 0;                // cycles the FIFO has read empty with the enable level low, saturating at 2
// the frame in flight will get no more data: the primary's EN fell, or the enable level did (any slot); fill it
// from its next word
wire       cut  = (fs == F_STREAM || (fs == F_ACK && !fwack)) && rempty && !qv && !em &&
                  ((fsel == 2'd0 && !en_s && mt == 2'd2) || (!hen_s && ml == 2'd2));

// rest, for the enable link: the level low, every EN and every mux on SPI, the feeder idle with its acks low and an
// empty pipeline, the FIFO empty and its Gray source current, no done word in flight
wire       dn_idle;
assign rest = !hen_s && cs1 == 4'd0 && !en_sys && x_en_sys == 2'd0 && fidle && !qv && !em && !rjp &&
              fack == 3'd0 && !o_wr && rempty && sector_r_lwgray == (rptr ^ (rptr >> 1)) && dn_idle;

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
	sys_lwx <= {rjt, rjs, x_wr[1], x_rd[1], x_wr[0], x_rd[0], DIAG ? evt[3:1] : 3'd0, evt[0], fs == F_REJECT,
	            fs == F_STREAM || fs == F_FILL || fs == F_ACK, o_ack | |x_ack, en_sys, c_req};

	if (DIAG && dn_go && !dn_busy) dsent <= dcnt;

	qv <= pop;
	mt <= (!rempty || en_s) ? 2'd0 : (mt == 2'd2) ? 2'd2 : mt + 2'd1;
	ml <= (!rempty || hen_s) ? 2'd0 : (ml == 2'd2) ? 2'd2 : ml + 2'd1;
	if (pop) rptr <= rptr + 1'd1;
	sector_r_lwgray <= rptr ^ (rptr >> 1);

	o_wr <= 0;
	if (rjp) evt[0] <= ~evt[0];
	rjp  <= 0;

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
		F_IDLE, F_REJECT:
			if (!hen_s) ;                // discarded: its request stands for SPI
			else if (qt == T_BEGIN) begin
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
				else begin
					fs  <= F_REJECT;
					rjs <= bs;
					rjt <= btg;
					rjp <= 1'b1;
				end
			end
			else if (qt == T_ABORT) fs <= F_IDLE;

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
	if (fs > F_REJECT) fs <= F_IDLE;     // from any state

	if (c_reset) begin
		if (fs == F_ACK && fwack) begin
			fack[fsel] <= 1'b0;
			fs <= F_IDLE;
		end
		else if (fs == F_STREAM || fs == F_ACK) begin
			fs     <= F_FILL;
			evt[3] <= ~evt[3];
		end
		else if (fs == F_REJECT) fs <= F_IDLE;
	end
end

endmodule

`default_nettype wire
`endif
`endif
