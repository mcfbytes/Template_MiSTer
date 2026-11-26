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

// SECTOR block (LW directory kind 2, version 2.0, "SEC3"): one hps_io read slot (the primary, slot index k = 0)
// plus two save slots (X0, X1: k = 1, 2) served over the LW bridge by ONE feeder.

module lw_sector #(parameter [7:0] SLOT = 1, parameter [12:0] FRAME = 588, parameter [2:0] BLKSZ = 3,
                   parameter AW = 9, parameter [3:0] XSLOT0 = 2, parameter [3:0] XSLOT1 = 3,
                   parameter [12:0] XFRAME0 = 256, parameter [12:0] XFRAME1 = 256, parameter [5:0] XLB0 = 7,
                   parameter [5:0] XLB1 = 7, parameter [3:0] GAP = 2)
(
	input wire        clk,          // clk_100m

	input wire [14:0] raddr,        // block-relative
	input wire        rsel,
	output reg [31:0] rdata,
	input wire [14:0] waddr,
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
	input wire        c_commit,     // hps_io's commit mark: the SPI host was shown a request it has not finished
	input wire        c_wide,       // hps_io's io_wide (a constant): 1 = 16-bit strobes, 0 = 8-bit
	output reg        en_sys = 0,   // mux state: 1 = this block owns the consumer's ack/write/data
	output wire       o_ack,
	output reg        o_wr   = 0,
	output reg [15:0] o_dat  = 0,
	output reg [13:0] o_adr  = 0,   // the strobe's halfword or byte index, hps_io's sd_buff_addr order

	input wire  [1:0] x_rd,         // save slots X0/X1 (clk_sys): read and write requests, LBA, SPI ack
	input wire  [1:0] x_wr,
	input wire [63:0] x_lba,        // {X1, X0}, 32 bits each
	input wire  [1:0] x_hps_ack,
	output reg  [1:0] x_en_sys = 0,
	output wire [1:0] x_ack
);

localparam [1:0]  T_DATA = 2'd0, T_BEGIN = 2'd1, T_ABORT = 2'd2;

// The geometry is checked where it is declared (lw_sys_window.sv): frames of 1-4096 words, a save slot's of at most
// 2^AW - 2, LBA widths of 1-32 bits, a gap of 2-15 cycles. A save slot's LBA, masked to its declared width:
localparam [31:0] XM0 = (XLB0 >= 6'd32) ? 32'hFFFFFFFF : (32'd1 << XLB0) - 32'd1;
localparam [31:0] XM1 = (XLB1 >= 6'd32) ? 32'hFFFFFFFF : (32'd1 << XLB1) - 32'd1;

function automatic [2:0] onehot(input [1:0] k);
	onehot = (k == 2'd0) ? 3'b001 : (k == 2'd1) ? 3'b010 : (k == 2'd2) ? 3'b100 : 3'b000;
endfunction

/////////////////////////////// FIFO (M10K) ///////////////////////////////

localparam DEPTH = 1 << AW;

(* ramstyle = "M10K" *) reg [45:0] fram [0:DEPTH-1];   // {type[1:0], index[11:0], data[31:0]}

reg  [AW:0] wspec = 0;              // the next entry
reg  [AW:0] wcom  = 0;              // past the last committed entry
reg  [AW:0] wvis  = 0;              // steps one entry per cycle toward wcom: its Gray code moves one bit
reg   [1:0] wsl   = 0;              // the slot of the frame being stored
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

/////////////////////////////// clk_100m //////////////////////////////////

localparam [1:0] F_IDLE = 2'd0, F_ACK = 2'd1, F_STREAM = 2'd2, F_FILL = 2'd3;

reg   [1:0] fs = F_IDLE;
reg  [12:0] fexp = 0;
reg         em = 0, qv = 0, tmode = 0;   // em: a word is being strobed
reg   [1:0] sn = 0, ackc = 0;   // sn: that word's strobe last issued
reg   [3:0] gc = 4'd15;         // clk_sys cycles since the last strobe or ack rise, saturating at 15
reg  [31:0] wv = 0;             // the word being strobed
reg  [45:0] q;

reg   [2:0] cause = 0;              // CAUSE: bit k = slot k's request opened or re-opened
reg         rq_off0 = 0, rq_rd0 = 0;   // the primary's last record: offered, rd, LBA
reg  [31:0] rq_lba = 0;
reg   [1:0] rq_xoff = 0, rq_xrd = 0, rq_xwr = 0;   // the save slots' last records: offered, rd, wr, LBA
reg  [63:0] rq_xlba = 0;
(* preserve *) reg       en_lwx   = 0;   // CTRL[0]
(* preserve *) reg [1:0] xen_lwx  = 0;   // CTRL[2:1]
reg   [2:0] ans = 0;                // {X1, X0, primary}: a BEGIN/WACK for the offered request entered the FIFO
reg   [8:0] btag = 0;               // per slot, 3 bits: the count of its BEGIN/WACK entries, carried in each entry
reg         abort_pend = 0;         // an ABORT store found the FIFO full: it enters in the first cycle with room
reg  [12:0] flen_w = FRAME;
reg  [12:0] exp_w = 0;
reg  [11:0] fdrops = 0;
reg         bfail = 0;
wire        f_lvl, f_fall, hen_s, rest;   // the enable link's level and its fall; on clk_sys, the level and rest

(* preserve *) reg [11:0] sys_lwx = 0;
wire [11:0] sb1;
reg  [11:0] sb2 = 0;

lw_sync #(.W(AW + 1), .GRAY(1)) rg_sync (.clk(clk), .d(sector_r_lwgray), .q(rg_s));
lw_sync #(.W(12)) sys_sync (.clk(clk), .d(sys_lwx), .q(sb1));
wire  [7:0] st_s = sb1[7:0];            // {sd_commit, busy, ack[2:0], en_sys[2:0]}, each by slot index k
wire  [3:0] ev   = sb1[11:8] ^ sb2[11:8]; // {reset_fill, abort, 0, reject}

// clk_sys -> clk_100m: the request records, {slot[41:40], offered, new, rej, tag[36:34], wr, rd, lba[31:0]}
wire [41:0] rc_q;
wire        rc_v;
wire  [2:0] rc_sel = {3{rc_v}} & onehot(rc_q[41:40]);
wire        rc_off = rc_q[39], rc_new = rc_q[38], rc_rej = rc_q[37];
wire  [2:0] rc_tag = rc_q[36:34];

// Registers in 0x000-0x1FF: the common words, then a 0x20-byte block per slot index k at 0x100 + 0x20k (word j:
// RSTAT_S, LBA_S, BEGIN_S, WACK_S); DATA from 0x4000.
wire        rreg  = rsel & ~raddr[14] & ~|raddr[13:9];
wire        wreg  = wsel & write & ~waddr[14] & ~|waddr[13:9] & &be;
wire        wdat  = wsel & write &  waddr[14] & &be & ({1'b0, waddr[13:2]} < flen_w);   // the OPEN frame's length
wire        wnext = wdat & ({1'b0, waddr[13:2]} == exp_w);   // its next word: the only one taken
wire  [6:0] widx  = waddr[8:2];
wire  [2:0] wk    = waddr[7:5];                   // the slot index of a per-slot word
wire  [2:0] wj    = waddr[4:2];
wire        w_cause = wreg && widx == 7'h00;
wire        w_ctrl  = wreg && widx == 7'h02;
wire        w_bs    = wreg && waddr[8] && wj == 3'd2;                   // BEGIN_S[k]
wire        w_wk    = wreg && waddr[8] && wj == 3'd3 && wk != 3'd0;     // WACK_S[k], save slots only
wire        b_p     = w_bs && wk == 3'd0;
wire        b_x0    = (w_bs | w_wk) && wk == 3'd1;
wire        b_x1    = (w_bs | w_wk) && wk == 3'd2;
wire        w_begin = b_p | b_x0 | b_x1;
wire  [1:0] bsel    = wk[1:0];
wire [12:0] blen    = w_wk ? 13'd0 : b_p ? FRAME : b_x0 ? XFRAME0 : XFRAME1;
wire        bx      = w_bs & (b_x0 | b_x1);             // a save slot's BEGIN_S: its frame is uncommitted
wire        w_abort = w_ctrl & wdata[8];
wire        clr     = w_ctrl & wdata[10];
// An uncommitted save-slot frame, and its withdrawal by an ABORT, BEGIN or WACK store: the store's entry, if any, is
// written at wcom
wire        unc   = wspec != wcom;
wire        wdr   = unc & (w_begin | w_abort);
wire [AW:0] wb    = wdr ? wcom : wspec;
wire [AW:0] used  = wb - g2b(rg_s);
wire        ffull = used[AW];
// a store's entry is taken only with room and no ABORT held; the held ABORT takes the first cycle with room; an
// ABORT that withdraws a frame enters nothing
wire        a_go  = abort_pend & ~ffull;
wire        p_ok  = (w_begin | (w_abort & ~unc) | wnext) & ~ffull & ~abort_pend;
wire        p_bad = (w_begin | wdat) & (ffull | abort_pend);   // a store refused by a full FIFO: dropped and counted
wire        fwe   = a_go | p_ok;
// the entry is committed at once, except a save slot's BEGIN_S and its words before the last
wire        cmt   = a_go | (p_ok & ~bx & (~wnext | wsl == 2'd0 | exp_w == flen_w - 13'd1));
wire  [2:0] wks   = onehot(wsl) & {3{wdr | (unc & ~hen)}};   // the slot whose frame is withdrawn

wire  [2:0] bent  = {b_x1, b_x0, b_p} & {3{p_ok}};
wire  [8:0] btag1 = {btag[8:6] + 3'd1, btag[5:3] + 3'd1, btag[2:0] + 3'd1};
wire  [2:0] etag  = (bsel == 2'd1) ? btag1[5:3] : (bsel == 2'd2) ? btag1[8:6] : btag1[2:0];
// a reject of the slot's LATEST answer re-opens the request: REQ shows it again and its cause bit rises
wire  [2:0] latest = {rc_tag == btag[8:6], rc_tag == btag[5:3], rc_tag == btag[2:0]};
wire  [2:0] reopen = rc_sel & {3{rc_rej}} & latest & ~bent;
wire  [2:0] unans  = (rc_sel & {3{rc_new | ~rc_off}}) | reopen;
// a withdrawn frame's request is open again, unless the store that withdrew it answers that slot
wire  [2:0] wraise = wks & {3{wdr}} & {xen_lwx, en_lwx} & {rq_xoff, rq_off0} & ~bent;
// so is the request of a BEGIN or WACK refused by a full FIFO or a held ABORT, unless an answer of its slot stands
wire  [2:0] fraise = {b_x1, b_x0, b_p} & {3{~p_ok}} & {xen_lwx, en_lwx} & {rq_xoff, rq_off0} & ~ans;
wire  [2:0] raise  = (rc_sel & {3{rc_off}} & ({3{rc_new}} | reopen)) | wraise | fraise;
wire        fire   = |raise;
wire        w_en  = wdata[0] & wired[0];                                     // the EN bits a store sets
wire  [1:0] w_xen = wdata[2:1] & wired[2:1];
wire  [2:0] clr_c = {3{w_cause}} & wdata[2:0];
wire [45:0] entry = a_go ? {T_ABORT, 44'd0} : w_begin ? {T_BEGIN, 6'd0, etag, w_wk, bsel, wdata} :
                    w_abort ? {T_ABORT, 44'd0} : {T_DATA, waddr[13:2], wdata};   // an index below flen_w

// REQ: the lowest-index slot whose EN is set, whose record shows its request offered, and that is not answered
wire  [2:0] cand   = {xen_lwx, en_lwx} & {rq_xoff, rq_off0} & ~ans;
wire        any    = |cand;
wire  [1:0] pick   = cand[0] ? 2'd0 : cand[1] ? 2'd1 : cand[2] ? 2'd2 : 2'd0;   // 0 when none
// each slot's GET_SDSTAT word, from its last record (hps_io's, with a zero block count)
wire [15:0] pst0   = {1'b1, 6'd0, BLKSZ, SLOT[3:0], 1'b0, rq_rd0};
wire [15:0] xst0   = {1'b1, 6'd0, BLKSZ, XSLOT0, rq_xwr[0], rq_xrd[0]};
wire [15:0] xst1   = {1'b1, 6'd0, BLKSZ, XSLOT1, rq_xwr[1], rq_xrd[1]};
wire [15:0] pst    = !any ? 16'd0 : pick == 2'd0 ? pst0 : pick == 2'd1 ? xst0 : xst1;

always @(posedge clk) if (fwe) fram[wb[AW-1:0]] <= entry;

function automatic [7:0] sat8(input [7:0] v, input inc);
	sat8 = (inc && ~&v) ? v + 8'd1 : v;
endfunction

always @(posedge clk) begin
	sb2 <= sb1;

	// the request records, the cycle after each lands; an ENABLE finds every link at rest and nothing offered
	if (rc_sel[0]) begin rq_off0 <= rc_off; rq_rd0 <= rc_q[32]; rq_lba <= rc_q[31:0]; end
	if (rc_sel[1]) begin rq_xoff[0] <= rc_off; {rq_xwr[0], rq_xrd[0]} <= rc_q[33:32]; rq_xlba[31:0] <= rc_q[31:0]; end
	if (rc_sel[2]) begin rq_xoff[1] <= rc_off; {rq_xwr[1], rq_xrd[1]} <= rc_q[33:32]; rq_xlba[63:32] <= rc_q[31:0]; end
	if (take) begin rq_off0 <= 0; rq_xoff <= 0; end

	// CAUSE: a raise beats a concurrent clear; an ENABLE clears the word
	cause <= raise | (cause & ~clr_c & ~{3{take}});
	// answered: a BEGIN/WACK for the slot entered the FIFO; it ends with a record that shows the request new or
	// withdrawn, or rejects that latest answer, with the slot's uncommitted frame, and at an ENABLE (one discarded
	// without the enable leaves its request standing)
	ans  <= bent | (ans & ~unans & ~wks & ~{3{take}});
	btag <= {bent[2] ? btag1[8:6] : btag[8:6], bent[1] ? btag1[5:3] : btag[5:3], bent[0] ? btag1[2:0] : btag[2:0]};

	if (f_fall || !f_lvl) begin
		en_lwx   <= 0;
		xen_lwx  <= 0;
	end
	else if (w_ctrl) begin
		en_lwx   <= w_en;
		xen_lwx  <= w_xen;
	end

	// a full FIFO holds an ABORT (until the enable falls) and drops and counts any other store
	if (!hen) abort_pend <= 0;
	else abort_pend <= (abort_pend & ~a_go) | (w_abort & ~unc & ffull & ~abort_pend);

	// store path: never back-pressured
	if (p_bad) begin
		if (~&fdrops) fdrops <= fdrops + 12'd1;
	end
	if (fwe) begin
		wspec <= wb + 1'd1;
		if (cmt) wcom <= wb + 1'd1;
	end
	else if (wdr || (unc && !hen)) wspec <= wcom;   // withdrawn: nothing of it was ever visible
	if (wvis != wcom) wvis <= wvis + 1'd1;
	sector_w_lwgray <= wvis ^ (wvis >> 1);

	// the frame being stored; a BEGIN refused opens none, and a withdrawn frame takes no more words
	if (w_begin) begin
		exp_w  <= 0;
		flen_w <= p_ok ? blen : 13'd0;
		wsl    <= bsel;
		bfail  <= ~p_ok;
		fdrops <= p_ok ? 12'd0 : 12'd1;
	end
	else if (wdr || (unc && !hen)) begin
		flen_w <= 0;
		bfail  <= 1;
	end
	else if (wnext && p_ok) exp_w <= exp_w + 13'd1;

end

assign doorbell = |cause;

// The enable link. Drain: no ABORT held, no uncommitted frame, every committed entry visible to clk_sys.
lw_fence fence (.clk(clk), .hen(hen), .take(take),
                .drain(!abort_pend && !unc && wvis == wcom && sector_w_lwgray == (wvis ^ (wvis >> 1))), .fin(1'b1),
                .open(f_open), .owned(f_owned), .idle(f_idle), .lvl(f_lvl), .fall(f_fall),
                .clk_sys(clk_sys), .rest(rest), .hen_s(hen_s));

wire ok = (exp_w == flen_w) & ~bfail;

always @(*) begin
	rdata = 32'd0;
	if (rreg) case (raddr[8:2])
		7'h00: rdata = {29'd0, cause};                     // CAUSE: one event bit per slot
		7'h01: rdata = {any, 13'd0, pick, pst};            // REQ
		7'h02: rdata = {21'd0, 3'd0, 5'd0, xen_lwx, en_lwx};   // CTRL
		7'h03: rdata = {16'd0, 5'd0, st_s[7], abort_pend, st_s[6], 1'b0, st_s[5:3], 1'b0,
		                st_s[2:0]};                        // STATUS
		7'h04: rdata = {ok, bfail, 2'd0, fdrops, 3'd0, exp_w};   // ACCEPT: {ok, closed, 0, drops, 0, next_index}
		7'h05: rdata = 32'h33434553;     // "SEC3"
		7'h40: rdata = {rq_off0,    ans[0], 14'd0, pst0};   // RSTAT_S[0]
		7'h41: rdata = rq_lba;                              // LBA_S[0]
		7'h48: rdata = {rq_xoff[0], ans[1], 14'd0, xst0};
		7'h49: rdata = rq_xlba[31:0];
		7'h50: rdata = {rq_xoff[1], ans[2], 14'd0, xst1};
		7'h51: rdata = rq_xlba[63:32];
		default: rdata = 32'd0;
	endcase
end

/////////////////////////////// clk_sys: feeder ///////////////////////////

wire [2:0] cs1;
lw_sync #(.W(3)) ctl_sync (.clk(clk_sys), .d({xen_lwx, en_lwx}), .q(cs1));
wire       en_s = cs1[0], test_s = 1'b0;
wire [1:0] xen_s = cs1[2:1];
lw_sync #(.W(AW + 1), .GRAY(1)) wg_sync (.clk(clk_sys), .d(sector_w_lwgray), .q(wg_s));

wire       rempty = (sector_r_lwgray == wg_s);
wire       fidle  = (fs == F_IDLE);
wire       dis    = ~hen_s & fidle & ~em & (rptr != g2b(wg_s));   // without the enable: discard, every cycle
// Strobes: GAP cycles apart, the first GAP after the ack. A word's entry is popped only once its first strobe can
// issue in the cycle it is valid: GAP - 1 cycles after the previous strobe, or the ack's rise, at the earliest.
wire [1:0] slast  = c_wide ? 2'd1 : 2'd3;   // the last strobe of a word
wire       gap_ok = gc >= GAP;              // a strobe may issue at this edge
wire       nxt_ok = gc >= GAP - 4'd1;       // the next word's entry may be popped
wire       wend   = em && sn >= slast && nxt_ok;   // the word ends here
wire       pop    = (~rempty & ~qv & (fs != F_FILL) & (fs != F_ACK) & (~em | wend) & (fs != F_STREAM | nxt_ok)) | dis;
wire [1:0] qt = q[45:44];
wire [11:0] qi = q[43:32];
wire [31:0] qd = q[31:0];
wire [31:0] xl0 = x_lba[31:0] & XM0, xl1 = x_lba[63:32] & XM1;   // the save slots' LBAs on their declared widths

reg  [1:0] fsel  = 0;               // the slot the open frame serves: 0 primary, 1 X0, 2 X1
reg        fwack = 0;               // the open frame is a WACK: ack only, no strobes
wire [12:0] flen = (fsel == 2'd0) ? FRAME : (fsel == 2'd1) ? XFRAME0 : XFRAME1;
wire [1:0] bs    = qi[1:0];
wire       bwk   = qi[2];
wire [2:0] btg   = qi[5:3];
wire       bmatch = (bs == 2'd0) ? (!bwk && en_sys && c_req && qd == c_lba) :
                    (bs == 2'd1) ? (x_en_sys[0] && (bwk ? x_wr[0] : x_rd[0]) && qd == xl0) :
                    (bs == 2'd2) ? (x_en_sys[1] && (bwk ? x_wr[1] : x_rd[1]) && qd == xl1) : 1'b0;

always @(posedge clk_sys) q <= fram[rptr[AW-1:0]];

reg  [2:0] fack = 0;                // per-slot ack: {X1, X0, primary}
// strobe j's address: the halfword index 2 x word + j with 16-bit strobes, else the byte index 4 x word + j
function automatic [13:0] strobe_adr(input [1:0] j);
	strobe_adr = c_wide ? {fexp[12:0], j[0]} : {fexp[11:0], j};
endfunction
reg  [1:0] mt   = 0;                // cycles the FIFO has read empty with EN low, saturating at 2
reg  [1:0] ml   = 0;                // cycles the FIFO has read empty with the enable level low, saturating at 2
// the frame in flight will get no more data: the primary's EN fell, or the enable level did (any slot: a save
// slot's frame is whole in the FIFO, so this meets one only after a clock upset); fill it from its next word
wire       cut  = fs == F_STREAM && rempty && !qv && !em &&
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
wire [31:0] rlba = (rs == 2'd1) ? xl0 : (rs == 2'd2) ? xl1 : c_lba;
wire [41:0] rec  = {rs, offd[rs], opn[rs], rej[rs], rtg_s, rwr_s, rlba};

lw_xfer #(.W(42)) rec_x(.s_clk(clk_sys), .s_go(rc_go), .s_d(rec), .s_busy(rc_busy),
                        .d_clk(clk), .d_q(rc_q), .d_v(rc_v));

// rest, for the enable link: the level low, every EN and every mux on SPI, the feeder idle with its acks low and an
// empty pipeline, the FIFO empty and its Gray source current, no record or done word in flight or owed
wire       dn_idle;
assign rest = !hen_s && ~|cs1 && !en_sys && x_en_sys == 2'd0 && fidle && !qv && !em &&
              fack == 3'd0 && !o_wr && rempty && sector_r_lwgray == (rptr ^ (rptr >> 1)) && dn_idle &&
              offd == 3'd0 && dirty == 3'd0 && !rc_busy;

assign dn_idle = 1'b1;
assign o_ack = fack[0];
assign x_ack = fack[2:1];

always @(posedge clk_sys) begin
	sys_lwx <= {4'd0, c_commit, !fidle, fack, x_en_sys, en_sys};

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
	gc <= (gc == 4'd15) ? gc : gc + 4'd1;
	mt <= (!rempty || en_s) ? 2'd0 : (mt == 2'd2) ? 2'd2 : mt + 2'd1;
	ml <= (!rempty || hen_s) ? 2'd0 : (ml == 2'd2) ? 2'd2 : ml + 2'd1;
	if (pop) rptr <= rptr + 1'd1;
	sector_r_lwgray <= rptr ^ (rptr >> 1);

	o_wr <= 0;

	// each slot's mux follows its EN only while the feeder is off that slot and its SPI ack is low, and moves to
	// this block only while hps_io's commit mark reads 0
	if ((fidle || fsel != 2'd0) && !hps_ack) en_sys <= en_s & (en_sys | ~c_commit);
	if ((fidle || fsel != 2'd1) && !x_hps_ack[0]) x_en_sys[0] <= xen_s[0] & (x_en_sys[0] | ~c_commit);
	if ((fidle || fsel != 2'd2) && !x_hps_ack[1]) x_en_sys[1] <= xen_s[1] & (x_en_sys[1] | ~c_commit);

	if (fs == F_ACK) begin
		ackc <= ackc - 1'd1;
		if (ackc == 1) begin
			if (fwack) begin
				fack[fsel] <= 1'b0;
				fs     <= F_IDLE;
			end
			else fs <= F_STREAM;
		end
	end

	if (fs == F_FILL && !em && gap_ok) begin
		o_wr  <= ~tmode;
		o_dat <= 0;
		o_adr <= strobe_adr(2'd0);
		wv    <= 0;
		em    <= 1;
		sn    <= 0;
		gc    <= 4'd1;
	end

	if (fidle) fack <= 3'd0;           // idle drives every ack low

	if (qv) case (fs)
		// without the enable an entry is discarded (an answer's request stands for SPI); a rejected answer is
		// dropped with the DATA behind it, which is popped here and does nothing, as an ABORT does
		F_IDLE:
			if (hen_s && qt == T_BEGIN) begin
				fexp <= 0;
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
					gc    <= 4'd1;
					ackc  <= 2;
					fs    <= bwk ? F_ACK : F_STREAM;   // a WACK: an ack pulse of two cycles, no strobes
				end
			end

		F_STREAM:
			if (qt != T_DATA) begin
				fs     <= F_FILL;
			end
			else begin                           // in order: the pop waited for the gap
				o_wr  <= ~tmode;
				o_dat <= c_wide ? qd[15:0] : {8'd0, qd[7:0]};
				o_adr <= strobe_adr(2'd0);
				wv    <= qd;
				em    <= 1;
				sn    <= 0;
				gc    <= 4'd1;
			end

		default: ;
	endcase

	// the word's later strobes, each GAP after the one before; GAP - 1 after its last, the word ends
	if (em) begin
		if (sn < slast) begin
			if (gap_ok) begin
				o_wr  <= ~tmode;
				o_dat <= c_wide ? wv[31:16] : {8'd0, wv[{sn + 2'd1, 3'd0} +: 8]};
				o_adr <= strobe_adr(sn + 2'd1);
				sn    <= sn + 2'd1;
				gc    <= 4'd1;
			end
		end
		else if (nxt_ok) begin
			em <= 0;
			sn <= 0;
			if (fexp >= flen - 1'd1) begin   // or past it: a frame ends from any count
				fexp   <= 0;
				fack[fsel] <= 1'b0;
				fs     <= F_IDLE;
			end
			else fexp <= fexp + 1'd1;
		end
	end

	if (cut) fs <= F_FILL;

	// the primary consumer's reset zero-fills the primary's frame; a save slot's frame plays out whole
	if (c_reset && fs == F_STREAM && fsel == 2'd0) begin
		fs     <= F_FILL;
	end
end

endmodule

`default_nettype wire
`endif
`endif
