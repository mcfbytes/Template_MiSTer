// FROZEN from d8fb0a729e14585e21ebe30dd7d711bfebbe9c23:sys/lw_lane.sv by tb/golden/freeze.sh: never edit, re-run the script
`ifdef MISTER_LW
`ifdef MISTER_LW_LANE
`default_nettype none

module gold_lw_lane #(parameter AW = 10, parameter [2:0] GAP = 3'd4, parameter RAW = 8)   // GAP: 2..7
(
	input wire         clk,          // clk_100m

	input wire  [11:0] raddr,        // block-relative
	output reg  [31:0] rdata,
	input wire  [11:0] waddr,
	input wire         wsel,
	input wire         write,
	input wire  [31:0] wdata,
	input wire   [3:0] be,
	input wire         hen,          // the host enable
	input wire         take,         // an ENABLE store is taken in this cycle
	output wire        f_open,       // the enable link: stores are taken (OPEN)
	output wire        f_owned,      // OWNED
	output wire        f_idle,       // the link lets an ENABLE through
	output wire        irq,          // level, high while CAUSE != 0

	input wire         clk_sys,
	input wire         l_busy,       // any of the link's own selects, or its ID view
	input wire         l_wait,       // io_wait | vs_wait: the term that withholds the link's ack
	input wire  [15:0] l_dout,       // the link's reply word (io_dout | io_dout_sys)
	output reg         own    = 0,   // the player drives the link nets
	output reg         hold   = 0,   // own, plus one cycle: the link's ack path stays frozen
	output reg   [3:0] sel    = 0,   // {osd_vga, osd_hdmi, fpga, uio}
	output reg         strobe = 0,
	output wire [15:0] din           // the head entry's data: held from the cycle before its strobe
);

localparam DEPTH  = 1 << AW;
localparam RDEPTH = 1 << RAW;
localparam [1:0] K_WORD = 2'd0, K_BEGIN = 2'd1, K_END = 2'd2, K_BEGINC = 2'd3;   // kind[0] = a BEGIN
localparam [AW:0] ROOM = {1'b0, {AW{1'b1}}}; // DEPTH - 1: a frame always keeps one entry for its END

function automatic [AW:0] g2b(input [AW:0] g);
	integer n;
	begin
		g2b[AW] = g[AW];
		for (n = AW - 1; n >= 0; n = n - 1) g2b[n] = g2b[n + 1] ^ g[n];
	end
endfunction

function automatic [RAW:0] g2r(input [RAW:0] g);
	integer n;
	begin
		g2r[RAW] = g[RAW];
		for (n = RAW - 1; n >= 0; n = n - 1) g2r[n] = g2r[n + 1] ^ g[n];
	end
endfunction

// {kind[1:0], sel[1:0], data[15:0]}
(* ramstyle = "M10K" *) reg [19:0] fram [0:DEPTH-1];
// reply words, written on clk_sys, read on clk_100m
(* ramstyle = "M10K" *) reg [15:0] rram [0:RDEPTH-1];

/////////////////////////////// clk_100m //////////////////////////////////

reg  [AW:0] wspec = 0;                      // the open frame's next entry
reg  [AW:0] wcom  = 0;                      // past the last committed END
reg  [AW:0] wvis  = 0;                      // steps one entry per clk toward wcom: the gray code moves one bit
(* preserve *) reg [AW:0] lane_w_lwgray = 0;
(* preserve *) reg [AW:0] lane_r_lwgray = 0;   // clk_sys
wire [AW:0] rg_s;
reg         open = 0, ovf = 0;
reg  [15:0] played = 0, drops = 0;          // free-running, 16-bit
(* preserve *) reg pt_lwx = 0;              // clk_sys: toggles once per played frame
wire        pt_s;
reg         pt_d = 0;

reg [RAW:0] rrp = 0;                        // reply FIFO read pointer
(* preserve *) reg [RAW:0] lane_rr_lwgray = 0;
(* preserve *) reg [RAW:0] lane_rw_lwgray = 0;   // clk_sys: reply FIFO write pointer
wire [RAW:0] rwg_s;
reg [RAW:0] rwg_d = 0;                      // rwg_s a cycle before: a difference is a reply word landed
reg  [15:0] rhead = 0;
reg  [15:0] rdrops = 0;                     // reply words dropped on a full reply FIFO, free-running
(* preserve *) reg rdt_lwx = 0;             // clk_sys: toggles once per dropped reply word
wire        rdt_s;
reg         rdt_d = 0;

lw_sync #(.W(AW + 1), .GRAY(1)) rg_sync (.clk(clk), .d(lane_r_lwgray), .q(rg_s));
lw_sync #(.W(RAW + 1), .GRAY(1)) rwg_sync (.clk(clk), .d(lane_rw_lwgray), .q(rwg_s));
lw_sync #(.W(1)) pt_sync (.clk(clk), .d(pt_lwx), .q(pt_s));
lw_sync #(.W(1)) rdt_sync (.clk(clk), .d(rdt_lwx), .q(rdt_s));

wire        st     = wsel & write & &be;
wire        w_beg  = st && waddr[11:2] == 10'd0;
wire        w_wrd  = st && waddr[11:2] == 10'd1 && open;
wire        w_end  = st && waddr[11:2] == 10'd2 && open;
wire [AW:0] rbin   = g2b(rg_s);
wire [AW:0] used   = (w_beg ? wcom : wspec) - rbin;   // entries in use below the one this store writes
wire        b_ok   = used < ROOM;           // BEGIN, and its END after it
wire        w_ok   = ~ovf && used < ROOM;
wire [RAW:0] rcount = g2r(rwg_s) - rrp;      // reply words waiting
wire        w_pop  = st && waddr[11:2] == 10'd3 && rcount != 0;
wire [RAW:0] rrnext = rrp + 1'd1;

// CAUSE: {DROP, REPLY, PLAYED}
localparam [2:0] C_PLAYED = 3'b001, C_REPLY = 3'b010, C_DROP = 3'b100;
reg   [2:0] cause  = 0;
wire        w_cause = st && waddr[11:2] == 10'd9;
wire  [2:0] clr    = w_cause ? wdata[2:0] : 3'd0;
wire  [2:0] raise  = ((pt_s ^ pt_d) ? C_PLAYED : 3'd0) | ((rwg_s != rwg_d) ? C_REPLY : 3'd0) |
                     (((hen && w_end && ovf) || (rdt_s ^ rdt_d)) ? C_DROP : 3'd0);
assign      irq    = |cause;

reg         we;
reg  [AW-1:0] wa;
reg  [19:0] wd;
always @(*) begin
	we = 0;
	wa = wspec[AW-1:0];
	wd = {K_WORD, 2'b00, wdata[15:0]};
	if (w_beg) begin
		we = b_ok;
		wa = wcom[AW-1:0];
		wd = {wdata[18] ? K_BEGINC : K_BEGIN, wdata[17:16], wdata[15:0]};
	end
	else if (w_wrd) we = w_ok;
	else if (w_end) begin
		we = ~ovf;
		wd = {K_END, 18'd0};
	end
end

always @(posedge clk) if (we) fram[wa] <= wd;
always @(posedge clk) rhead <= rram[rrp[RAW-1:0]];

always @(posedge clk) begin
	if (wvis != wcom) wvis <= wvis + 1'd1;
	lane_w_lwgray <= wvis ^ (wvis >> 1);

	pt_d <= pt_s;
	if (pt_s ^ pt_d) played <= played + 1'd1;

	if (w_pop || (!hen && rcount != 0)) rrp <= rrnext;
	lane_rr_lwgray <= rrp ^ (rrp >> 1);
	rdt_d <= rdt_s;
	if (rdt_s ^ rdt_d) rdrops <= rdrops + 1'd1;
	rwg_d <= rwg_s;
	cause <= raise | (cause & ~clr & ~{3{take}});   // a raise beats a clear in the same cycle; an ENABLE clears

	// BEGIN abandons any open frame: its entries past wcom are overwritten; so does the enable's fall
	if (!hen) begin
		open  <= 0;
		ovf   <= 0;
		wspec <= wcom;
	end
	else if (w_beg) begin
		open  <= 1;
		ovf   <= ~b_ok;
		wspec <= b_ok ? wcom + 1'd1 : wcom;
	end
	else if (w_wrd) begin
		if (w_ok) wspec <= wspec + 1'd1;
		else ovf <= 1;
	end
	else if (w_end) begin
		open <= 0;
		if (ovf) drops <= drops + 1'd1;
		else wcom <= wspec + 1'd1;
	end
end

// The enable link. Drain: no frame open, every committed entry visible to clk_sys. Last condition before IDLE:
// the reply FIFO stepped to empty.
wire        rest, hen_s;
wire        f_drain = !open && wspec == wcom && wvis == wcom && lane_w_lwgray == (wvis ^ (wvis >> 1));
lw_fence fence (.clk(clk), .hen(hen), .take(take), .drain(f_drain), .fin(rcount == 0),
                .open(f_open), .owned(f_owned), .idle(f_idle), .lvl(), .fall(),
                .clk_sys(clk_sys), .rest(rest), .hen_s(hen_s));

always @(*) begin
	case (raddr[11:2])
		10'd4:   rdata = {16'd0, played};
		10'd5:   rdata = {16'd0, drops};
		10'd6:   rdata = 32'h314E414C;    // "LAN1"
		10'd7:   rdata = {rdrops, {(15 - RAW){1'b0}}, rcount};
		10'd8:   rdata = (rcount != 0) ? {16'd0, rhead} : 32'd0;
		10'd9:   rdata = {29'd0, cause};
		default: rdata = 32'd0;
	endcase
end

/////////////////////////////// clk_sys ///////////////////////////////////

localparam [2:0] S_IDLE = 3'd0, S_STB = 3'd1, S_ACK = 3'd2, S_NEXT = 3'd3, S_GAP = 3'd4, S_REL = 3'd5;

wire [AW:0]  wg_s;
wire [RAW:0] rrg_s;
lw_sync #(.W(AW + 1), .GRAY(1)) wg_sync (.clk(clk_sys), .d(lane_w_lwgray), .q(wg_s));
lw_sync #(.W(RAW + 1), .GRAY(1)) rrg_sync (.clk(clk_sys), .d(lane_rr_lwgray), .q(rrg_s));
reg  [AW:0] rptr  = 0;
reg  [19:0] q     = 0;
reg         fresh = 0;                      // q holds fram[rptr]
reg   [2:0] ps    = S_IDLE;
reg   [1:0] quiet = 0;                      // cycles the link's selects have been low, saturating at 2
reg   [2:0] cnt   = 0;
reg         capf  = 0;                      // the frame playing records its replies
reg [RAW:0] rwp   = 0;
reg   [1:0] ml    = 0;                      // cycles the FIFO has read empty with the enable level low, to 2

wire [AW:0] rnext = rptr + 1'd1;
wire        empty = rptr == g2b(wg_s);
wire        avail = fresh && lane_r_lwgray != wg_s;
wire  [1:0] qk    = q[19:18];
wire        t_beg = ps == S_IDLE && avail && qk[0] && quiet == 2'd2 && !l_busy && hen_s;
wire        t_odd = ps == S_IDLE && avail && !qk[0] && hen_s;   // a stray entry
wire        t_dis = ps == S_IDLE && !empty && !hen_s;            // without the enable: every entry, every cycle
wire        t_ack = ps == S_ACK && !l_wait && cnt == GAP - 1'd1;
wire        t_wrd = ps == S_NEXT && avail && qk == K_WORD;
wire        t_end = ps == S_NEXT && avail && qk == K_END;
wire        t_cut = ps == S_NEXT && !avail && !hen_s && empty && ml == 2'd2;   // a word that cannot come
wire        pop   = t_odd | t_dis | ps == S_STB | t_end;   // q still holds the entry through its strobe cycle

assign rest = !hen_s && ps == S_IDLE && !own && !hold && !strobe && sel == 4'd0 && empty &&
              lane_r_lwgray == (rptr ^ (rptr >> 1)) && lane_rw_lwgray == (rwp ^ (rwp >> 1));
wire        r_push = t_ack & capf;
wire [RAW:0] r_used = rwp - g2r(rrg_s);
wire        r_full = r_used[RAW];
wire [RAW:0] rwnext = rwp + 1'd1;

assign din = q[15:0];

always @(posedge clk_sys) if (r_push & ~r_full) rram[rwp[RAW-1:0]] <= l_dout;

always @(posedge clk_sys) begin
	q     <= fram[rptr[AW-1:0]];
	fresh <= ~pop;
	if (pop) rptr <= rnext;
	lane_r_lwgray <= rptr ^ (rptr >> 1);
	quiet <= l_busy ? 2'd0 : (quiet == 2'd2) ? 2'd2 : quiet + 2'd1;
	ml    <= (hen_s || !empty) ? 2'd0 : (ml == 2'd2) ? 2'd2 : ml + 2'd1;

	// the reply to the strobe just acknowledged; the gray pointer follows rwp a cycle later, after the word is in
	// the RAM
	if (r_push) begin
		if (r_full) rdt_lwx <= ~rdt_lwx;
		else rwp <= rwnext;
	end
	lane_rw_lwgray <= rwp ^ (rwp >> 1);

	case (ps)
		S_IDLE: if (t_beg) begin
			own  <= 1;
			hold <= 1;
			sel  <= 4'b0001 << q[17:16];
			capf <= qk[1];
			ps   <= S_STB;
		end
		else begin
			own    <= 0;
			hold   <= 0;
			sel    <= 0;
			strobe <= 0;
		end
		S_STB: begin
			strobe <= 1;
			cnt    <= 0;
			ps     <= S_ACK;
		end
		S_ACK: begin
			strobe <= 0;
			if (l_wait) cnt <= 0;
			else if (t_ack) ps <= S_NEXT;
			else cnt <= cnt + 1'd1;
		end
		S_NEXT: if (t_wrd) ps <= S_STB;
		else if (avail || t_cut) begin
			sel <= 0;
			cnt <= 0;
			ps  <= S_GAP;
		end
		S_GAP: if (cnt == 1) begin
			own <= 0;
			ps  <= S_REL;
		end
		else cnt <= cnt + 1'd1;
		S_REL: begin
			hold   <= 0;
			pt_lwx <= ~pt_lwx;
			ps     <= S_IDLE;
		end
		default: begin
			own    <= 0;
			hold   <= 0;
			sel    <= 0;
			strobe <= 0;
			ps     <= S_IDLE;
		end
	endcase
end

endmodule

`default_nettype wire
`endif
`endif
