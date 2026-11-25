//============================================================================
//  fio_tb.sv -- the download trace bench (arm D of tb/run.sh): the OCRAM download player against the LW lane's
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
//
// Two link stacks side by side, each player -> lw_link -> a real hps_io -> a core model: A is lw_lane (the LW
// build's frame lane), fed over its raw store port (BEGIN, WORDs, END) well ahead of its player; B is
// hps_ocram_fplay (hps_ocram_rx's download player), fed by an ideal source: each LFRAME header and ring word in
// the cycle the player answers the entry before. Both see the same enable level (lw_lane's own hen_s), no link
// traffic of their own and the same core: a wait raised D cycles after each ioctl_wr it chooses, held for the
// table's cycles. Each frame's trace, {sel, strobe, ioctl_download, ioctl_index, ioctl_wr, ioctl_addr, ioctl_dout,
// ioctl_file_ext} on every clk_sys edge from own's rise to hold's fall, is compared frame by frame: the walker's
// crossing puts a gap between two mailbox frames that the lane does not have, so frames are aligned at their
// start. Every write the core sees is logged and checked against the window's words: none lost or doubled.
// The core counts an overrun when an ioctl_wr arrives while a wait it owes is still pending or up.
// Macros: FIO_TB_GAP (4), FIO_TB_WIDE (1), FIO_TB_SYS_HP_PS (14762: 33.8688 MHz).

`timescale 1ns/1ps
`default_nettype none

module fio_tb;

`ifdef FIO_TB_GAP
localparam [2:0] GAP = `FIO_TB_GAP;
`else
localparam [2:0] GAP = 3'd4;
`endif
`ifdef FIO_TB_WIDE
localparam WIDE = `FIO_TB_WIDE;
`else
localparam WIDE = 1;
`endif
`ifdef FIO_TB_SYS_HP_PS
localparam real SYS_HP = `FIO_TB_SYS_HP_PS / 1000.0;
`else
localparam real SYS_HP = 14.762;
`endif
localparam integer STEP = WIDE ? 2 : 1;

integer checks = 0, fails = 0, seed = 1;

task check(input string what, input [31:0] got, input [31:0] exp);
	begin
		checks = checks + 1;
		if (got !== exp) begin
			fails = fails + 1;
			$display("FAIL %s: got %08h exp %08h", what, got, exp);
		end
		else $display("pass %s = %08h", what, got);
	end
endtask

reg clk = 0;
always #5.0 clk = ~clk;
reg clk_sys = 0;
initial begin #2.1; forever #(SYS_HP) clk_sys = ~clk_sys; end

//////////////////////////////  the core model  /////////////////////////////

// The wait each write of a window asks for: c_hold[n - c_base] cycles (0: none), raised c_dly cycles after its
// ioctl_wr; c_base is the write count at the window's start, the same for both cores.
integer c_dly = 1, c_base = 0;
integer c_hold [0:8191];

/////////////////////////////////  stack A  /////////////////////////////////

reg  [11:0] waddr = 0;
reg         wsel = 0, write = 0;
reg  [31:0] wdata = 0;
reg         hen = 0, take = 0;
wire        f_open, f_owned, f_idle, irq_a;
wire [31:0] rdata_a;
wire        wait_a, wait_b;
wire        own_a, hold_a, stb_a;
wire  [3:0] sel_a;
wire [15:0] din_a;

lw_lane #(.AW(10), .GAP(GAP), .RAW(8)) u_lane
(
	.clk(clk), .raddr(12'd0), .rdata(rdata_a), .waddr(waddr), .wsel(wsel), .write(write), .wdata(wdata),
	.be(4'hF), .hen(hen), .take(take), .f_open(f_open), .f_owned(f_owned), .f_idle(f_idle), .irq(irq_a),
	.clk_sys(clk_sys), .l_busy(1'b0), .l_wait(wait_a), .l_dout(16'd0),
	.own(own_a), .hold(hold_a), .sel(sel_a), .strobe(stb_a), .din(din_a)
);

/////////////////////////////////  stack B  /////////////////////////////////

// the ideal source: entries in order, the next in the cycle the one before is answered
reg  [31:0] ent_d [0:8191];
reg         ent_h [0:8191];
integer     ent_n = 0, ent_i = 0;
reg         out_q = 0;
reg  [31:0] w_hold = 0;
reg         h_hold = 0;
wire        a_go, rest_b;
wire        w_v  = (!out_q || a_go) && ent_i < ent_n;
wire [31:0] w_d  = w_v ? ent_d[ent_i] : w_hold;
wire        w_hd = w_v ? ent_h[ent_i] : h_hold;
always @(posedge clk_sys) begin
	if (w_v) begin w_hold <= w_d; h_hold <= w_hd; ent_i <= ent_i + 1; out_q <= 1'b1; end
	else if (a_go) out_q <= 1'b0;
end

wire        own_b, hold_b, stb_b;
wire  [3:0] sel_b;
wire [15:0] din_b;
hps_ocram_fplay #(.GAP(GAP)) u_play
(
	.clk_sys(clk_sys), .hen_s(u_lane.hen_s),
	.w_v(w_v), .w_head(w_hd), .w_d(w_d), .a_go(a_go), .rest(rest_b),
	.l_busy(1'b0), .l_wait(wait_b),
	.own(own_b), .hold(hold_b), .sel(sel_b), .strobe(stb_b), .din(din_b)
);

////////////////////////////  the link fronts  //////////////////////////////

wire  [3:0] io_sel_a, io_sel_b;
wire        io_stb_a, io_stb_b, ack_a, ack_b;
wire [15:0] io_din_a, io_din_b;
lw_link link_a
(
	.clk_sys(clk_sys), .l_sel(4'd0), .l_clk(1'b0), .l_din(16'd0), .l_wait(wait_a),
	.p_own(own_a), .p_hold(hold_a), .p_sel(sel_a), .p_strobe(stb_a), .p_din(din_a),
	.io_sel(io_sel_a), .io_strobe(io_stb_a), .io_din(io_din_a), .io_ack(ack_a)
);
lw_link link_b
(
	.clk_sys(clk_sys), .l_sel(4'd0), .l_clk(1'b0), .l_din(16'd0), .l_wait(wait_b),
	.p_own(own_b), .p_hold(hold_b), .p_sel(sel_b), .p_strobe(stb_b), .p_din(din_b),
	.io_sel(io_sel_b), .io_strobe(io_stb_b), .io_din(io_din_b), .io_ack(ack_b)
);

////////////////////////////////  hps_io x2  ////////////////////////////////

wire [383:0] bus_a, bus_b;
assign (weak0, weak1) bus_a[383:46] = 338'd0;
assign (weak0, weak1) bus_b[383:46] = 338'd0;
assign bus_a[45:38] = 0;
assign bus_b[45:38] = 0;
assign bus_a[35:33] = {io_sel_a[1], io_sel_a[0], io_stb_a};
assign bus_b[35:33] = {io_sel_b[1], io_sel_b[0], io_stb_b};
assign bus_a[31:16] = io_din_a;
assign bus_b[31:16] = io_din_b;

localparam DW = WIDE ? 15 : 7;
wire        dl_a, dl_b, wr_a, wr_b;
wire [15:0] ix_a, ix_b;
wire [26:0] ad_a, ad_b;
wire [DW:0] do_a, do_b;
wire [31:0] fx_a, fx_b;

// the block ports' arrays, idle
reg  [31:0] z_lba [0:0];
reg   [5:0] z_cnt [0:0];
reg  [DW:0] z_din [0:0];
initial begin z_lba[0] = 0; z_cnt[0] = 0; z_din[0] = 0; end

hps_io #(.CONF_STR("FIO;;"), .WIDE(WIDE)) io_a
(
	.clk_sys(clk_sys), .HPS_BUS(bus_a),
	.sd_lba(z_lba), .sd_blk_cnt(z_cnt), .sd_rd(1'b0), .sd_wr(1'b0), .sd_buff_din(z_din), .ioctl_wait(wait_a), .ioctl_din({(DW+1){1'b0}}),
	.ioctl_download(dl_a), .ioctl_index(ix_a), .ioctl_wr(wr_a), .ioctl_addr(ad_a), .ioctl_dout(do_a),
	.ioctl_file_ext(fx_a)
);
hps_io #(.CONF_STR("FIO;;"), .WIDE(WIDE)) io_b
(
	.clk_sys(clk_sys), .HPS_BUS(bus_b),
	.sd_lba(z_lba), .sd_blk_cnt(z_cnt), .sd_rd(1'b0), .sd_wr(1'b0), .sd_buff_din(z_din), .ioctl_wait(wait_b), .ioctl_din({(DW+1){1'b0}}),
	.ioctl_download(dl_b), .ioctl_index(ix_b), .ioctl_wr(wr_b), .ioctl_addr(ad_b), .ioctl_dout(do_b),
	.ioctl_file_ext(fx_b)
);

wire [15:0] dx_a = do_a, dx_b = do_b;
fio_core core_a(.clk(clk_sys), .wr(wr_a), .addr(ad_a), .dout(dx_a), .wait_o(wait_a));
fio_core core_b(.clk(clk_sys), .wr(wr_b), .addr(ad_b), .dout(dx_b), .wait_o(wait_b));

//////////////////////////  the per-frame traces  ///////////////////////////

// Each frame's trace folds into a hash of every edge's vector, in order, and its length in edges; the first
// FIRSTC edges of each frame are also kept, to print where two differ.
localparam integer MAXF = 256, FIRSTC = 64;
localparam [63:0] HP = 64'h0000_0100_0000_01B3;
reg  [63:0] hs_a [0:MAXF-1];
reg  [63:0] hs_b [0:MAXF-1];
integer     len_a [0:MAXF-1];
integer     len_b [0:MAXF-1];
reg [127:0] fc_a [0:MAXF*FIRSTC-1];
reg [127:0] fc_b [0:MAXF*FIRSTC-1];
integer     fa = 0, fb = 0, ja = 0, jb = 0;
reg         in_a = 0, in_b = 0;
reg  [63:0] ha = 0, hb = 0;
wire [127:0] vec_a = {io_sel_a, io_stb_a, dl_a, ix_a, wr_a, ad_a, dx_a, fx_a};
wire [127:0] vec_b = {io_sel_b, io_stb_b, dl_b, ix_b, wr_b, ad_b, dx_b, fx_b};
always @(posedge clk_sys) begin
	#0.1;
	if (own_a && !in_a) begin in_a = 1; ja = 0; ha = 64'hCBF2_9CE4_8422_2325; end
	if (in_a) begin
		ha = (ha ^ vec_a[63:0]) * HP; ha = (ha ^ vec_a[127:64]) * HP;
		if (fa < MAXF && ja < FIRSTC) fc_a[fa * FIRSTC + ja] = vec_a;
		ja = ja + 1;
		if (!own_a && !hold_a) begin
			if (fa < MAXF) begin len_a[fa] = ja; hs_a[fa] = ha; end
			fa = fa + 1; in_a = 0;
		end
	end
	if (own_b && !in_b) begin in_b = 1; jb = 0; hb = 64'hCBF2_9CE4_8422_2325; end
	if (in_b) begin
		hb = (hb ^ vec_b[63:0]) * HP; hb = (hb ^ vec_b[127:64]) * HP;
		if (fb < MAXF && jb < FIRSTC) fc_b[fb * FIRSTC + jb] = vec_b;
		jb = jb + 1;
		if (!own_b && !hold_b) begin
			if (fb < MAXF) begin len_b[fb] = jb; hs_b[fb] = hb; end
			fb = fb + 1; in_b = 0;
		end
	end
end

// frames f0 .. fa-1 of both stacks, cycle for cycle
task cmp_frames(input string what, input integer f0);
	integer f, j, bad, nb;
	begin
		bad = 0; nb = 0;
		for (f = f0; f < fa && f < MAXF; f = f + 1) begin
			if (len_a[f] !== len_b[f] || hs_a[f] !== hs_b[f]) begin
				if (nb < 4) begin
					$display("   %s frame %0d: %0d edges on the lane, %0d on the mailbox, the traces %s", what, f,
					         len_a[f], len_b[f], hs_a[f] === hs_b[f] ? "equal" : "differ");
					for (j = 0; j < FIRSTC && j < len_a[f]; j = j + 1)
						if (fc_a[f * FIRSTC + j] !== fc_b[f * FIRSTC + j] && bad < 4) begin
							$display("     edge %0d: lane %h mailbox %h", j, fc_a[f * FIRSTC + j], fc_b[f * FIRSTC + j]);
							bad = bad + 1;
						end
				end
				nb = nb + 1;
			end
		end
		check({what, ": frames played by each"}, fb - f0, fa - f0);
		check({what, ": frames whose trace differs, edge for edge"}, nb, 0);
	end
endtask

/////////////////////////////////  windows  /////////////////////////////////

reg  [15:0] lw [0:8191];                 // the window's link words, frame by frame
integer     fs [0:255];                  // frame k: link words fs[k] .. fs[k+1]-1
integer     nf = 0, nw = 0;
reg  [15:0] dat [0:8191];                // the data words, in order
integer     nd = 0;
integer     a_base = 0;

task fr_open; begin fs[nf] = nw; nf = nf + 1; end endtask
task fr_word(input [15:0] w); begin lw[nw] = w; nw = nw + 1; end endtask

// index, optional info, the start with optional aux words, bursts of the given lengths, the end
task make_window(input [15:0] idx, input info, input aux, input integer nb, input integer lmin, input integer lmax);
	integer b, j, l;
	reg [15:0] d;
	begin
		nf = 0; nw = 0; nd = 0;
		fr_open; fr_word(16'h0055); fr_word(idx);
		if (info) begin fr_open; fr_word(16'h0056); fr_word($random(seed)); fr_word($random(seed)); end
		fr_open; fr_word(16'h0053); fr_word(16'h00FF);
		a_base = 0;
		if (aux) begin
			a_base = ($random(seed) & 32'h0003_FFFE);
			fr_word(a_base[15:0]); fr_word({5'd0, a_base[26:16]});
		end
		for (b = 0; b < nb; b = b + 1) begin
			l = lmin + (($random(seed) & 32'h7FFF_FFFF) % (lmax - lmin + 1));
			fr_open; fr_word(16'h0054);
			for (j = 0; j < l; j = j + 1) begin
				d = $random(seed);
				if (!WIDE) d = {8'd0, d[7:0]};
				fr_word(d); dat[nd] = d; nd = nd + 1;
			end
		end
		fr_open; fr_word(16'h0053); fr_word(16'h0000);
		fs[nf] = nw;
	end
endtask

task wstore(input [11:0] a, input [31:0] d);
	begin
		@(negedge clk); waddr = a; wdata = d; wsel = 1; write = 1;
		@(negedge clk); wsel = 0; write = 0;
	end
endtask

task open_link;
	integer t;
	begin
		hen = 1; t = 0;
		while (!f_idle && t < 1000) begin @(posedge clk); t = t + 1; end
		@(negedge clk); take = 1; @(negedge clk); take = 0;
		t = 0;
		while (!f_open && t < 1000) begin @(posedge clk); t = t + 1; end
		check("the lane's enable link opens", f_open, 1);
	end
endtask

// the window both ways: the lane's stores, the mailbox's entries (two link words a ring word, low half first, odd
// dropping the last high half), then both played out
task play_window(input string what);
	integer k, j, n, len, odd, t, wa0, wb0, f0, p0;
	begin
		f0 = fa; p0 = u_lane.played;
		wa0 = core_a.n; wb0 = core_b.n; c_base = wb0;
		// stack B's entries
		@(negedge clk_sys);
		for (k = 0; k < nf; k = k + 1) begin
			n = fs[k + 1] - fs[k];
			len = (n + 1) / 2; odd = n % 2;
			ent_d[ent_n] = {8'h30, odd[0], 5'd0, 2'd1, len[15:0]}; ent_h[ent_n] = 1; ent_n = ent_n + 1;
			for (j = 0; j < len; j = j + 1) begin
				ent_d[ent_n] = {(2 * j + 1 < n) ? lw[fs[k] + 2 * j + 1] : 16'd0, lw[fs[k] + 2 * j]};
				ent_h[ent_n] = 0; ent_n = ent_n + 1;
			end
		end
		// stack A's stores: BEGIN {sel 1, op}, the WORDs, END
		for (k = 0; k < nf; k = k + 1) begin
			wstore(12'h000, {14'd0, 2'd1, lw[fs[k]]});
			for (j = fs[k] + 1; j < fs[k + 1]; j = j + 1) wstore(12'h004, {16'd0, lw[j]});
			wstore(12'h008, 32'd0);
		end
		t = 0;
		while ((u_lane.played - p0 < nf || ent_i < ent_n || out_q || !rest_b || in_a || in_b) && t < 20000000) begin
			@(posedge clk); t = t + 1;
		end
		repeat (20) @(posedge clk_sys);
		check({what, ": the lane played every frame"}, u_lane.played - p0, nf);
		check({what, ": the lane dropped none"}, u_lane.drops, 0);
		cmp_frames(what, f0);
		check({what, ": the core's writes, mailbox"}, core_b.n - wb0, nd);
		check({what, ": the core's writes, lane"}, core_a.n - wa0, nd);
		n = 0;
		for (j = 0; j < nd; j = j + 1)
			if (core_b.lg_d[wb0 + j] !== dat[j] || core_b.lg_a[wb0 + j] !== a_base + j * STEP) begin
				if (n < 4) $display("   %s write %0d: addr %h data %h, exp %h %h", what, j, core_b.lg_a[wb0 + j],
				                    core_b.lg_d[wb0 + j], a_base + j * STEP, dat[j]);
				n = n + 1;
			end
		check({what, ": every word once, in order, at its address (mailbox)"}, n, 0);
		check({what, ": ioctl_index"}, ix_b, lw[1]);
		check({what, ": the download ended (mailbox, lane)"}, {dl_b, dl_a}, 2'b00);
	end
endtask

task holds(input integer kind, input integer hmin, input integer hmax);
	// kind 0: no wait; 1: every write; 2: every second write (a 32-bit word complete)
	integer j;
	begin
		for (j = 0; j < 8192; j = j + 1)
			c_hold[j] = (kind == 0 || (kind == 2 && !j[0])) ? 0 :
			            hmin + (($random(seed) & 32'h7FFF_FFFF) % (hmax - hmin + 1));
	end
endtask

integer ov0;
initial begin
	if ($value$plusargs("lw_seed=%d", seed)) ;
	$display("fio_tb: seed %0d, GAP %0d, WIDE %0d, clk_sys half period %0.3f ns", seed, GAP, WIDE, SYS_HP);
	holds(0, 0, 0);
	repeat (20) @(posedge clk);
	open_link;

	$display("-- F1: a window, no wait: index, start, nine bursts of 1-9 words, end, cycle for cycle");
	c_dly = 1; holds(0, 0, 0);
	make_window(16'h0003, 0, 0, 9, 1, 9);
	play_window("F1 plain");

	$display("-- F1: info (an odd frame), aux words, odd and even bursts, short waits on every write");
	holds(1, 1, 6);
	make_window(16'h0102, 1, 1, 12, 1, 60);
	play_window("F1 aux, info, waits");

	$display("-- F1/F2: the 32-bit write pattern: a wait on every second write, 8-40 cycles");
	holds(2, 8, 40);
	make_window(16'h0001, 0, 1, 8, 31, 64);
	play_window("F2 32-bit pattern");

	$display("-- F2: the wait raised GAP - 3 cycles after ioctl_wr, held 1-20 cycles and up to 10^4: no overrun");
	c_dly = GAP - 3; holds(1, 1, 20);
	c_hold[3] = 10000; c_hold[17] = 9999; c_hold[18] = 1;
	ov0 = core_b.overruns;
	make_window(16'h0004, 0, 0, 3, 8, 12);
	play_window("F2 at the deadline");
	check("F2 at the deadline: overruns (mailbox)", core_b.overruns - ov0, 0);

	$display("-- F2: the wait one cycle later (GAP - 2): the overrun is shown");
	c_dly = GAP - 2; holds(1, 6, 20);
	ov0 = core_b.overruns;
	make_window(16'h0005, 0, 0, 2, 8, 12);
	play_window("F2 one cycle late");
	check("F2 one cycle late: overruns shown (mailbox)", core_b.overruns - ov0 > 0, 1);

	check("frames compared", fa > 30, 1);
	if (fails == 0) $display("PASSED: %0d checks, 0 failures", checks);
	else $display("FAILED: %0d checks, %0d failures", checks, fails);
	$finish;
end

initial begin #400_000_000; $display("FAILED: timeout"); $finish; end

endmodule

// The core: logs every write, and asks for the wait the table gives it, c_dly cycles after its ioctl_wr.
module fio_core(input wire clk, input wire wr, input wire [26:0] addr, input wire [15:0] dout, output reg wait_o = 1'b0);
integer n = 0, dcnt = 0, hcnt = 0, hh = 0, overruns = 0;
reg [26:0] lg_a [0:8191];
reg [15:0] lg_d [0:8191];
always @(posedge clk) begin
	if (hcnt > 0) begin hcnt = hcnt - 1; if (hcnt == 0) wait_o <= 1'b0; end
	if (dcnt > 0) begin dcnt = dcnt - 1; if (dcnt == 0) begin wait_o <= 1'b1; hcnt = hh; end end
	if (wr) begin
		if (dcnt > 0 || hcnt > 0) overruns = overruns + 1;
		if (n < 8192) begin lg_a[n] = addr; lg_d[n] = dout; end
		hh = (n - fio_tb.c_base >= 0 && n - fio_tb.c_base < 8192) ? fio_tb.c_hold[n - fio_tb.c_base] : 0;
		if (hh > 0) begin
			if (fio_tb.c_dly <= 1) begin wait_o <= 1'b1; hcnt = hh; end
			else dcnt = fio_tb.c_dly - 1;
		end
		n = n + 1;
	end
end
endmodule

`default_nettype wire
