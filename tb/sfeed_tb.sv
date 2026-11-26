//============================================================================
//  sfeed_tb.sv -- the trace-compare bench (arm C of tb/run.sh): the OCRAM SECTOR feeder against lw_sector
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
// lw_sector (the LW build's primary slot, shipping) and hps_ocram_sfeed (hps_ocram_rx's SECTOR feeder) serve one
// consumer model each, side by side, and their {ack, wr, adr, dat} are compared on every clk_sys edge from the
// first ack rise of a frame to 8 cycles after the later ack's fall. Both feeders see the same enable and bind
// levels (lw_sector's own synchronised hen_s and en_s), the same consumer request, reset and io_wide.
// lw_sector is fed over its raw store port: a frame's BEGIN_S and words, kept ahead of the strobes. The mailbox
// feeder is fed by an ideal source: the frame's head in the cycle lw_sector pops its BEGIN_S, then each word in the
// cycle the feeder answers the one before. The walker's crossing round trip is longer than a word's strobes at
// GAP 2; this bench shows the feeder's timing, not the mailbox path's (ocram_tb covers the path).
// The mount slot k = 1 is compared the same way against lw_sector's save slot X0: a read of one block (blk_cnt 0,
// BLKSZ 2: 128 words, XFRAME0 128) through BEGIN_S[1]; all three acks and both slots' muxes are compared.
// In the plain runs a gold twin, lw_sector as frozen in tb/golden/, takes lw_sector's inputs, and every output of
// the two is compared on every edge: the feeders' compare stays against the frozen LW feeder when lw_sector changes.
// Macros: SFEED_TB_GAP (2), SFEED_TB_WIDE (1), SFEED_TB_FRAME (128), SFEED_TB_SYS_HP_PS (14762: 33.8688 MHz).

`timescale 1ns/1ps
`default_nettype none

module sfeed_tb;

`ifdef SFEED_TB_GAP
localparam [3:0] GAP = `SFEED_TB_GAP;
`else
localparam [3:0] GAP = 4'd2;
`endif
`ifdef SFEED_TB_WIDE
localparam WIDE = `SFEED_TB_WIDE;
`else
localparam WIDE = 1;
`endif
`ifdef SFEED_TB_FRAME
localparam integer FR = `SFEED_TB_FRAME;
`else
localparam integer FR = 128;
`endif
`ifdef SFEED_TB_SYS_HP_PS
localparam real SYS_HP = `SFEED_TB_SYS_HP_PS / 1000.0;
`else
localparam real SYS_HP = 14.762;
`endif

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

// lw_sector's store port (clk_100m)
reg  [14:0] waddr = 0;
reg         wsel = 0, write = 0;
reg  [31:0] wdata = 0;
reg         hen = 0, take = 0;
wire        f_open, f_owned, f_idle, doorbell;
wire [31:0] rdata;

// the consumer (clk_sys)
reg         c_reset = 0, c_req = 0;
reg  [31:0] c_lba = 0;
wire        en_a, ack_a, wr_a, en_b, ack_b, wr_b;
wire [15:0] dat_a, dat_b;
wire [13:0] adr_a, adr_b;
wire  [1:0] xen_a, xack_a, xen_b, xack_b;
reg         x_rd0 = 0;              // the save / mount slot's read request and LBA
reg  [31:0] x_lba0 = 0;
localparam integer MFR = 128;       // its block: blk_cnt 0 at BLKSZ 2

lw_sector #(.SLOT(1), .FRAME(FR[12:0]), .BLKSZ(3'd2), .GAP(GAP), .XFRAME0(MFR[12:0]),
            .XLB0(6'd32)) u_sec
(
	.clk(clk), .raddr(15'd0), .rsel(1'b0), .rdata(rdata),
	.waddr(waddr), .wsel(wsel), .write(write), .wdata(wdata), .be(4'hF),
	.hen(hen), .take(take), .f_open(f_open), .f_owned(f_owned), .f_idle(f_idle), .wired(3'b011),
	.doorbell(doorbell),
	.clk_sys(clk_sys), .c_reset(c_reset), .c_req(c_req), .c_lba(c_lba), .hps_ack(1'b0), .c_commit(1'b0),
	.c_wide(WIDE[0]), .en_sys(en_a), .o_ack(ack_a), .o_wr(wr_a), .o_dat(dat_a), .o_adr(adr_a),
	.x_rd({1'b0, x_rd0}), .x_wr(2'd0), .x_lba({32'd0, x_lba0}), .x_hps_ack(2'd0), .x_en_sys(xen_a), .x_ack(xack_a)
);

// the ideal source: the head with lw_sector's BEGIN_S pop, each word with the answer to the one before
reg  [31:0] fword [0:4095];
integer     src_n = 0;              // words of the current frame the source may give
integer     src_k = 0;              // the next word's index
reg         src_head = 0;           // a head is owed at lw_sector's BEGIN_S pop
reg   [1:0] src_kh = 0;             // the head's slot index, as lw_sector's entry names it
wire        a_go, rest_b;
wire  [1:0] a_code;
wire        beg_pop = u_sec.pop && u_sec.fs == 2'd0 && u_sec.q[45:44] == 2'd1 && u_sec.hen_s;
wire        w_head  = src_head && beg_pop;
wire        w_word  = !w_head && a_go && (a_code == 2'd0 || a_code == 2'd1) && src_k < src_n;
wire        w_v     = w_head || w_word;
wire [31:0] w_d     = w_head ? u_sec.q[31:0] : fword[src_k];
reg  [31:0] w_hold = 0;
reg         w_ishead = 0;
always @(posedge clk_sys) begin
	if (w_v) begin w_hold <= w_d; w_ishead <= w_head; end
	if (w_head) src_kh <= u_sec.q[33:32];
	if (w_head) src_head <= 1'b0;
	if (w_word) src_k <= src_k + 1;
end

/* verilator lint_off PINCONNECTEMPTY */
hps_ocram_sfeed #(.FRAME(FR), .GAP(GAP), .MNT_EN(2'b01), .BLKSZ(3'd2)) u_f
(
	.clk_sys(clk_sys), .hen_s(u_sec.hen_s), .bind_s({1'b0, u_sec.xen_s[0], u_sec.en_s}),
	.w_v(w_v), .w_head(w_v ? w_head : w_ishead), .w_wr(1'b0), .w_k(w_head ? u_sec.q[33:32] : src_kh),
	.w_d(w_v ? w_d : w_hold),
	.a_go(a_go), .a_code(a_code), .a_d(), .rest(rest_b),
	.c_reset(c_reset), .hps_ack(1'b0), .c_req(c_req), .c_lba(c_lba), .c_commit(1'b0), .c_wide(WIDE[0]),
	.m_hps_ack(2'd0), .m_rd({1'b0, x_rd0}), .m_wr(2'd0), .m_lba({32'd0, x_lba0}), .m_cnt(12'd0),
	.en_sys(en_b), .m_en(xen_b), .o_ack(ack_b), .m_ack(xack_b), .o_wr(wr_b), .o_dat(dat_b), .o_adr(adr_b)
);
/* verilator lint_on PINCONNECTEMPTY */

// the compare: from the first ack rise of a frame to 8 cycles past the later fall
integer cmp_n = 0, cmp_bad = 0, tail = 0, strobes_a = 0, strobes_b = 0;
reg     cmp_on = 0;
always @(posedge clk_sys) begin
	#0.1;
	if (ack_a || ack_b || xack_a[0] || xack_b[0]) begin cmp_on = 1; tail = 8; end
	else if (cmp_on) begin tail = tail - 1; if (tail == 0) cmp_on = 0; end
	if (cmp_on) begin
		cmp_n = cmp_n + 1;
		if ({xen_a[0], xack_a[0], ack_a, wr_a, wr_a ? adr_a : 14'd0, wr_a ? dat_a : 16'd0} !==
		    {xen_b[0], xack_b[0], ack_b, wr_b, wr_b ? adr_b : 14'd0, wr_b ? dat_b : 16'd0}) begin
			cmp_bad = cmp_bad + 1;
			if (cmp_bad <= 8)
				$display("   at %0t: lw_sector {x %b%b ack %b wr %b adr %h dat %h}, mailbox {x %b%b ack %b wr %b adr %h dat %h}",
				         $time, xen_a[0], xack_a[0], ack_a, wr_a, adr_a, dat_a,
				         xen_b[0], xack_b[0], ack_b, wr_b, adr_b, dat_b);
		end
	end
	if (wr_a) strobes_a = strobes_a + 1;
	if (wr_b) strobes_b = strobes_b + 1;
end

`ifndef LW_TB_CDC
// The gold twin: lw_sector as frozen in tb/golden/gold_lw_sector.sv on the same inputs, every output compared x-aware
// 100 ps after every edge of clk and clk_sys, so the compare above still holds the mailbox feeder to the frozen LW
// one when lw_sector itself changes. Plain only: under LW_TB_CDC each crossing cell draws its own delays.
wire [31:0] g_rdata;
wire        g_f_open, g_f_owned, g_f_idle, g_doorbell, g_en, g_ack, g_wr;
wire [15:0] g_dat;
wire [13:0] g_adr;
wire  [1:0] g_xen, g_xack;
gold_lw_sector #(.SLOT(1), .FRAME(FR[12:0]), .BLKSZ(3'd2), .DIAG(0), .GAP(GAP), .XFRAME0(MFR[12:0]),
                 .XLB0(6'd32)) g_sec
(
	.clk(clk), .raddr(15'd0), .rsel(1'b0), .rdata(g_rdata),
	.waddr(waddr), .wsel(wsel), .write(write), .wdata(wdata), .be(4'hF),
	.hen(hen), .take(take), .f_open(g_f_open), .f_owned(g_f_owned), .f_idle(g_f_idle), .wired(3'b011),
	.doorbell(g_doorbell),
	.clk_sys(clk_sys), .c_reset(c_reset), .c_req(c_req), .c_lba(c_lba), .hps_ack(1'b0), .c_commit(1'b0),
	.c_wide(WIDE[0]), .en_sys(g_en), .o_ack(g_ack), .o_wr(g_wr), .o_dat(g_dat), .o_adr(g_adr),
	.x_rd({1'b0, x_rd0}), .x_wr(2'd0), .x_lba({32'd0, x_lba0}), .x_hps_ack(2'd0), .x_en_sys(g_xen), .x_ack(g_xack)
);
wire [72:0] tw_live = {rdata, f_open, f_owned, f_idle, doorbell, en_a, ack_a, wr_a, dat_a, adr_a, xen_a, xack_a};
wire [72:0] tw_gold = {g_rdata, g_f_open, g_f_owned, g_f_idle, g_doorbell, g_en, g_ack, g_wr, g_dat, g_adr, g_xen, g_xack};
integer tw_edges = 0, tw_bad = 0;
task tw_cmp;
	begin
		tw_edges = tw_edges + 1;
		if (tw_live !== tw_gold) begin
			tw_bad = tw_bad + 1;
			if (tw_bad <= 8)
				$display("   at %0.3f ns: {rdata, open, owned, idle, bell, en, ack, wr, dat, adr, x_en, x_ack} lw_sector %h, gold %h",
				         $realtime, tw_live, tw_gold);
		end
	end
endtask
always @(clk) begin #0.1; tw_cmp; end
always @(clk_sys) begin #0.1; tw_cmp; end
`endif

task wstore(input [14:0] a, input [31:0] d);
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
		check("the link opens", f_open, 1);
		wstore(15'h008, 32'd1);                 // CTRL: the primary's EN
		t = 0;
		while (!(en_a && en_b) && t < 2000) begin @(posedge clk); t = t + 1; end
		check("both muxes move to the block with EN", {en_a, en_b}, 2'b11);
	end
endtask

// a frame: the request, then BEGIN_S and the words kept ahead of the strobes, at most 400 entries in lw_sector's FIFO
// (its own count: a reset frame's unplayed words still in it are discarded after the fill, so a frame's played
// strobes do not bound it, and a dropped store is never resent here);
// cut_at >= 0: the consumer's reset after that many strobes; part >= 0: only that many words, then the enable falls
task frame(input [31:0] lba, input integer cut_at, input integer part);
	integer j, t, sa0, sb0, nw, per;
	begin
		per = WIDE ? 2 : 4;
		nw = (part >= 0) ? part : FR;
		for (j = 0; j < FR; j = j + 1) fword[j] = $random(seed);
		@(negedge clk_sys); c_req = 1; c_lba = lba;
		src_n = nw; src_k = 0; src_head = 1;
		repeat (6) @(posedge clk_sys);
		sa0 = strobes_a; sb0 = strobes_b;
		wstore(15'h108, lba);                   // BEGIN_S[0]
		j = 0; t = 0;
		while (j < nw && t < 4000000) begin
			if (u_sec.used < 400) begin wstore(15'h4000 + j * 4, fword[j]); j = j + 1; end
			else begin @(posedge clk); t = t + 1; end
		end
		t = 0;
		while (!(ack_a && ack_b) && t < 100000) begin @(posedge clk_sys); t = t + 1; end
		@(negedge clk_sys); c_req = 0;
		if (cut_at >= 0) begin
			t = 0;
			while ((strobes_a - sa0) < cut_at && t < 1000000) begin @(posedge clk_sys); t = t + 1; end
			@(negedge clk_sys); c_reset = 1; @(negedge clk_sys); c_reset = 0;
		end
		if (part >= 0) begin
			t = 0;
			while ((strobes_a - sa0) < nw * per && t < 1000000) begin @(posedge clk_sys); t = t + 1; end
			repeat (10 + ($random(seed) & 15)) @(posedge clk_sys);
			@(negedge clk); hen = 0;
		end
		t = 0;
		while ((ack_a || ack_b || cmp_on) && t < 1000000) begin @(posedge clk_sys); t = t + 1; end
		check($sformatf("the frame for LBA %0d: lw_sector took every store", lba), u_sec.fdrops, 0);
		check($sformatf("the frame for LBA %0d: both strobed every strobe", lba), strobes_b - sb0,
		      (cut_at >= 0 || part >= 0) ? strobes_a - sa0 : FR * per);
		check($sformatf("the frame for LBA %0d: lw_sector's count", lba), strobes_a - sa0, FR * per);
	end
endtask

// a mount read: the slot's request, then BEGIN_S[1] and its words, committed whole; the request falls with both acks
task mframe(input [31:0] lba);
	integer j, t, sa0, sb0, per;
	begin
		per = WIDE ? 2 : 4;
		for (j = 0; j < MFR; j = j + 1) fword[j] = $random(seed);
		@(negedge clk_sys); x_rd0 = 1; x_lba0 = lba;
		src_n = MFR; src_k = 0; src_head = 1;
		repeat (6) @(posedge clk_sys);
		sa0 = strobes_a; sb0 = strobes_b;
		wstore(15'h128, lba);                   // BEGIN_S[1]
		for (j = 0; j < MFR; j = j + 1) wstore(15'h4000 + j * 4, fword[j]);
		t = 0;
		while (!(xack_a[0] && xack_b[0]) && t < 100000) begin @(posedge clk_sys); t = t + 1; end
		check($sformatf("the mount read for LBA %0d: both acks rise", lba), xack_a[0] && xack_b[0], 1);
		@(negedge clk_sys); x_rd0 = 0;
		t = 0;
		while ((xack_a[0] || xack_b[0] || cmp_on) && t < 1000000) begin @(posedge clk_sys); t = t + 1; end
		check($sformatf("the mount read for LBA %0d: lw_sector took every store", lba), u_sec.fdrops, 0);
		check($sformatf("the mount read for LBA %0d: both strobed every strobe", lba), strobes_b - sb0, MFR * per);
		check($sformatf("the mount read for LBA %0d: lw_sector's count", lba), strobes_a - sa0, MFR * per);
	end
endtask

integer i, c0;
initial begin
	if ($value$plusargs("lw_seed=%d", seed)) ;
	$display("sfeed_tb: seed %0d, GAP %0d, WIDE %0d, FRAME %0d, clk_sys half period %0.3f ns", seed, GAP, WIDE, FR,
	         SYS_HP);
	repeat (20) @(posedge clk);
	open_link;

	$display("-- three frames back to back: {ack, wr, adr, dat} equal on every clk_sys edge");
	c0 = cmp_bad;
	for (i = 0; i < 3; i = i + 1) frame(32'd100 + i, -1, -1);
	check("no cycle differs", cmp_bad - c0, 0);

	$display("-- the consumer's reset mid-frame: both zero-fill from the next word, cycle for cycle");
	c0 = cmp_bad;
	frame(32'd200, 6 + ($random(seed) & 63), -1);
	frame(32'd201, (WIDE ? 2 : 4) * FR - 2, -1);
	check("no cycle differs", cmp_bad - c0, 0);

	$display("-- the session ends mid-frame: the enable falls with part of the frame stored; both fill alike");
	c0 = cmp_bad;
	frame(32'd300, -1, 1 + (($random(seed) & 32'h7FFF_FFFF) % (FR - 1)));
	check("no cycle differs", cmp_bad - c0, 0);
	repeat (200) @(posedge clk);
	check("both muxes back on SPI after the fall", {en_a, en_b}, 2'b00);
	open_link;
	frame(32'd301, -1, -1);
	check("the next session's frame: no cycle differs", cmp_bad - c0, 0);

	$display("-- the mount slot k = 1 against the save slot X0: three block reads, then a disc frame between two");
	wstore(15'h008, 32'd3);                     // CTRL: the primary's EN and X0's
	i = 0;
	while (!(xen_a[0] && xen_b[0]) && i < 2000) begin @(posedge clk); i = i + 1; end
	check("both slot muxes move to the block with EN", {xen_a[0], xen_b[0]}, 2'b11);
	c0 = cmp_bad;
	for (i = 0; i < 3; i = i + 1) mframe(32'd400 + i);
	frame(32'd410, -1, -1);
	mframe(32'd411);
	check("no cycle differs", cmp_bad - c0, 0);

	check("edges compared", cmp_n > 10 * FR, 1);
`ifndef LW_TB_CDC
	$display("-- the gold twin: lw_sector against tb/golden/gold_lw_sector.sv, every output on every edge");
	check("the twin: edges compared", tw_edges > 10 * FR, 1);
	check("the twin: edges where an output differs", tw_bad, 0);
`endif
	if (fails == 0) $display("PASSED: %0d checks, 0 failures", checks);
	else $display("FAILED: %0d checks, %0d failures", checks, fails);
	$finish;
end

initial begin #200_000_000; $display("FAILED: timeout"); $finish; end

endmodule

`default_nettype wire
