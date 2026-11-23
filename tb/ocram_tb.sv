//============================================================================
//  ocram_tb.sv -- the OCRAM mailbox floor's bench (arm O of tb/run.sh)
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

// Standalone: hps_ocram_master (+ hps_ocram_diag) and sys/lw_cdc.sv with MISTER_OCRAM and MISTER_OCRAM_DIAG,
// a small MARK period and poll period, clk_100m. The AXI slave models the 64 KB OCRAM (a word array) with a
// seeded latency and ready-delay distribution, and modes normal, secure-refused (a non-OKAY response when
// AxPROT[1] = 0) and never-answer; it can also fail one chosen read and one chosen write once. The host model
// runs on its own 66.7 MHz clock and stores GPO 60-600 ns apart, and peeks and pokes the array as the HPS's
// strongly-ordered accesses. A protocol monitor checks every cycle's AXI constants, valid stability, address
// bounds and the one-transaction-at-a-time rule, and keeps its own latency counters and records.
// The cases are the design note's M3 bench, §6 of M3-BRIEF.md. Run: tb/run.sh (arm O), one seed per run.

`timescale 1ns/1ps
`default_nettype none

`ifndef MISTER_OCRAM_POLL
`define MISTER_OCRAM_POLL 12
`endif
`ifndef MISTER_OCRAM_MARK
`define MISTER_OCRAM_MARK 20
`endif

module ocram_tb;

localparam integer POLLC = `MISTER_OCRAM_POLL;
localparam integer MARKC = `MISTER_OCRAM_MARK;

localparam [15:0] H_HELLO = 16'h0000;
localparam [15:0] H_LINE  = 16'h0008;
localparam [15:0] H_IDX3  = 16'h001C;
localparam [15:0] C_HELLO = 16'h0080;
localparam [15:0] C_VERSION = 16'h0084;
localparam [15:0] C_CAPS  = 16'h0088;
localparam [15:0] C_STATE = 16'h008C;
localparam [15:0] C_IDX3  = 16'h009C;
localparam [15:0] C_BIND  = 16'h00A4;
localparam [15:0] C_RING3 = 16'h00BC;
localparam [15:0] EVT_BASE = 16'h4000;
localparam integer EVT_WORDS = 2048;

localparam [31:0] C_HELLO_ARMED = 32'h4F43_0000;
localparam [31:0] C_HELLO_WAIT  = 32'h5754_0000;
localparam [31:0] C_HELLO_STOP  = 32'h5154_0000;
localparam [31:0] MARK_HEADER   = 32'h2303_0028;
localparam [31:0] GPO_IO        = 32'h8000_0000;   // bit 31: the I/O view, RUN 0
localparam [31:0] GPO_RUN       = 32'h9000_0000;   // bit 31 and RUN

localparam integer M_NORMAL = 0, M_SECURE_REFUSE = 1, M_NEVER = 2;

integer checks = 0, fails = 0;
integer seed = 1;

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

////////////////////////////////  clocks  ///////////////////////////////////

reg clk = 0;                 // clk_100m
always #5.0 clk = ~clk;
reg clk_host = 0;            // the host's own clock: 66.7 MHz, not a multiple of 100 MHz
always #7.5 clk_host = ~clk_host;

//////////////////////////////  the DUT  ////////////////////////////////////

reg  [31:0] gpo_reg = 32'd0;
wire [31:0] gpo = gpo_reg;
reg         hps_rst_n = 0;

wire        present_wire, line_out;
wire  [7:0] awid, wid, bid, arid, rid;
wire [31:0] awaddr, wdata, araddr, rdata;
wire  [3:0] awlen, awcache, wstrb, arlen, arcache;
wire  [2:0] awsize, awprot, arsize, arprot;
wire  [1:0] awburst, awlock, bresp, arburst, arlock, rresp;
wire  [4:0] awuser, aruser;
wire        awvalid, awready, wlast, wvalid, wready, bvalid, bready;
wire        arvalid, arready, rlast, rvalid, rready;

hps_ocram_master dut
(
	.clk(clk),
	.hps_rst_n(hps_rst_n),
	.gp_out(gpo),
	.present(present_wire),
	.line(line_out),
	.awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awburst(awburst),
	.awlock(awlock), .awcache(awcache), .awprot(awprot), .awuser(awuser), .awvalid(awvalid),
	.awready(awready),
	.wid(wid), .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
	.bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
	.arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst),
	.arlock(arlock), .arcache(arcache), .arprot(arprot), .aruser(aruser), .arvalid(arvalid),
	.arready(arready),
	.rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast), .rvalid(rvalid), .rready(rready)
);

////////////////////////////////  the slave  ////////////////////////////////

// The 64 KB OCRAM, and the AXI3 slave of the fpga2hps bridge at 32 bits.
reg [31:0] mem [0:16383];
integer mi;
initial for (mi = 0; mi < 16384; mi = mi + 1) mem[mi] = 32'd0;

integer mode = M_NORMAL;
reg        inj_rd_en = 0, inj_wr_en = 0;
reg [15:0] inj_rd_addr = 0, inj_wr_addr = 0;

function integer lat_pick;
	integer r;
	begin
		r = ($random(seed) & 32'h7FFF_FFFF) % 1000;
		if (r < 50)        lat_pick = 1;
		else if (r < 950)  lat_pick = 20 + (($random(seed) & 32'h7FFF_FFFF) % 41);
		else               lat_pick = 200 + (($random(seed) & 32'h7FFF_FFFF) % 1801);
	end
endfunction
function integer rdy_pick;
	rdy_pick = ($random(seed) & 32'h7FFF_FFFF) % 4;
endfunction

// write channel
localparam [1:0] WS_IDLE = 0, WS_LAT = 1, WS_RSP = 2;
reg  [1:0]  wst = WS_IDLE;
reg         aw_seen = 0, w_seen = 0;
reg  [1:0]  awdly = 0, wdly = 0;
reg  [15:0] wlat = 0;
reg  [31:0] waddr_q = 0, wdata_q = 0;
reg  [2:0]  wprot_q = 0;
reg         werr_q = 0;
reg         bvalid_r = 0;
reg  [1:0]  bresp_r = 0;

assign awready = (mode != M_NEVER) && (wst == WS_IDLE) && !aw_seen && (awdly == 0);
assign wready  = (mode != M_NEVER) && (wst == WS_IDLE) && !w_seen && (wdly == 0);
assign bid     = 8'd0;
assign bresp   = bresp_r;
assign bvalid  = bvalid_r;

always @(posedge clk) begin
	if (!hps_rst_n) begin                 // the bridge resets with the HPS: its channels go idle
		wst <= WS_IDLE; aw_seen <= 1'b0; w_seen <= 1'b0;
		awdly <= 2'd0; wdly <= 2'd0; bvalid_r <= 1'b0;
	end
	else case (wst)
	WS_IDLE: begin
		bvalid_r <= 1'b0;
		if (awdly != 0) awdly <= awdly - 2'd1;
		if (wdly  != 0) wdly  <= wdly  - 2'd1;
		if (awvalid && awready) begin
			waddr_q <= awaddr;
			wprot_q <= awprot;
			aw_seen <= 1'b1;
			werr_q  <= (mode == M_SECURE_REFUSE && awprot[1] == 1'b0) ||
			           (inj_wr_en && (awaddr[15:0] == inj_wr_addr));
			if (inj_wr_en && (awaddr[15:0] == inj_wr_addr)) inj_wr_en <= 1'b0;
		end
		if (wvalid && wready) begin wdata_q <= wdata; w_seen <= 1'b1; end
		if ((aw_seen || (awvalid && awready)) && (w_seen || (wvalid && wready))) begin
			wst <= WS_LAT;
			wlat <= lat_pick();
			awdly <= rdy_pick();
			wdly  <= rdy_pick();
			aw_seen <= 1'b0;
			w_seen  <= 1'b0;
		end
	end
	WS_LAT: begin
		if (wlat == 0) begin
			wst <= WS_RSP;
			bvalid_r <= 1'b1;
			bresp_r  <= werr_q ? 2'b10 : 2'b00;
		end
		else begin wlat <= wlat - 16'd1; bvalid_r <= 1'b0; end
	end
	WS_RSP: begin
		if (bready) begin                  // the handshake ends it; a stale valid would be taken
			bvalid_r <= 1'b0;
			if (bresp_r == 2'b00) mem[waddr_q[15:2]] <= wdata_q;
			wst <= WS_IDLE;
		end
		else bvalid_r <= 1'b1;             // hold with its payload until BREADY
	end
	default: wst <= WS_IDLE;
	endcase
end

// read channel
localparam [1:0] RS_IDLE = 0, RS_LAT = 1, RS_RSP = 2;
reg  [1:0]  rst = RS_IDLE;
reg  [1:0]  ardy = 0;
reg  [15:0] rlat = 0;
reg  [31:0] raddr_q = 0, rdata_r = 0;
reg         rerr_q = 0;
reg         rvalid_r = 0;
reg  [1:0]  rresp_r = 0;

assign arready = (mode != M_NEVER) && (rst == RS_IDLE) && (ardy == 0);
assign rid     = 8'd0;
assign rdata   = rdata_r;
assign rresp   = rresp_r;
assign rlast   = 1'b1;
assign rvalid  = rvalid_r;

always @(posedge clk) begin
	if (!hps_rst_n) begin                 // the bridge resets with the HPS: its channels go idle
		rst <= RS_IDLE; ardy <= 2'd0; rvalid_r <= 1'b0;
	end
	else case (rst)
	RS_IDLE: if (ardy != 0) begin ardy <= ardy - 2'd1; rvalid_r <= 1'b0; end
	else if (arvalid && arready) begin
		raddr_q <= araddr;
		rdata_r <= mem[araddr[15:2]];
		rerr_q  <= (mode == M_SECURE_REFUSE && arprot[1] == 1'b0) ||
		           (inj_rd_en && (araddr[15:0] == inj_rd_addr));
		if (inj_rd_en && (araddr[15:0] == inj_rd_addr)) inj_rd_en <= 1'b0;
		rlat <= lat_pick();
		ardy <= rdy_pick();
		rst  <= RS_LAT;
	end
	RS_LAT: begin
		if (rlat == 0) begin rst <= RS_RSP; rvalid_r <= 1'b1; rresp_r <= rerr_q ? 2'b10 : 2'b00; end
		else begin rlat <= rlat - 16'd1; rvalid_r <= 1'b0; end
	end
	RS_RSP: begin
		if (rready) begin rvalid_r <= 1'b0; rst <= RS_IDLE; end
		else rvalid_r <= 1'b1;
	end
	default: rst <= RS_IDLE;
	endcase
end

///////////////////////////////  the host  //////////////////////////////////

// The host's strongly-ordered peeks and pokes, and its GPO stores 60-600 ns apart on its own clock.
reg [15:0] host_tail = 16'd0;
integer    consumed_seq = 0;
integer    exp_ns = 0, exp_flood = 0;

task host_wait;
	begin
		repeat (4 + (($random(seed) & 32'h7FFF_FFFF) % 37)) @(posedge clk_host);
	end
endtask
task gpo_store(input [31:0] v);
	begin host_wait; gpo_reg = v; end
endtask
task poke(input [15:0] off, input [31:0] v);
	begin mem[off >> 2] = v; end
endtask
// The host's own tail store: the array and the monitor's idea of the host's tail move together.
task store_tail(input [15:0] t);
	begin mem[H_IDX3 >> 2] = {16'd0, t}; host_tail = t; end
endtask
task peek(input [15:0] off, output [31:0] v);
	begin v = mem[off >> 2]; end
endtask
function [31:0] evt_word(input [15:0] idx);
	evt_word = mem[(EVT_BASE + ((idx & (EVT_WORDS - 1)) << 2)) >> 2];
endfunction

task zero_page;
	integer i;
	begin
		for (i = 0; i < 64; i = i + 1) mem[i] = 32'd0;
		host_tail = 16'd0;
		consumed_seq = 0;
	end
endtask

task reset_floor;
	begin
		mode = M_NORMAL;
		inj_rd_en = 0; inj_wr_en = 0;
		gpo_reg = GPO_IO;
		repeat (8) @(posedge clk);
		hps_rst_n = 0;
		repeat (12) @(posedge clk);
		hps_rst_n = 1;
		repeat (16) @(posedge clk);
		zero_page;
		exp_ns = 0; exp_flood = 0;
	end
endtask

task run_edge(input integer ns, input integer flood);
	reg [31:0] b;
	begin
		b = GPO_IO;
		if (ns)    b = b | 32'h0800_0000;
		if (flood) b = b | 32'h0400_0000;
		exp_ns = ns; exp_flood = flood;
		gpo_store(b);
		gpo_store(b | 32'h1000_0000);
	end
endtask

task wait_word(input [15:0] off, input [31:0] want, input integer us, output integer ok);
	integer n;
	reg [31:0] v;
	begin
		ok = 0;
		n = 0;
		while (!ok && n < us * 100) begin       // 1 us = 100 clk_100m cycles
			peek(off, v);
			if (v === want) ok = 1;
			else begin @(posedge clk); n = n + 1; end
		end
	end
endtask

task wait_armed(input integer us, output integer ok);
	begin wait_word(C_HELLO, C_HELLO_ARMED, us, ok); end
endtask
task wait_echo(input [15:0] nonce, input integer us, output integer ok);
	begin wait_word(C_HELLO, {16'h4F43, nonce}, us, ok); end
endtask
task wait_stopped(input integer us, output integer ok);
	begin wait_word(C_HELLO, C_HELLO_STOP, us, ok); end
endtask

task start_session(input integer ns, input integer flood, input integer line_en, input [15:0] nonce,
                   output integer ok);
	integer ok1, ok2;
	begin
		zero_page;
		poke(H_LINE, line_en ? 32'd8 : 32'd0);
		poke(H_IDX3, 32'd0);
		run_edge(ns, flood);
		wait_armed(1000, ok1);
		poke(H_HELLO, {16'h4F43, nonce});
		wait_echo(nonce, 1000, ok2);
		ok = ok1 && ok2;
		check($sformatf("session %04h: ARMED then the echo", nonce), ok, 1);
	end
endtask

//////////////////////////////  the monitor  ////////////////////////////////

// Every invariant of §3 on every cycle, and the bench's own copy of the counters and the records.
integer cyc = 0;
integer mon_err = 0;

task merr(input string what);
	begin
		mon_err = mon_err + 1;
		if (mon_err <= 8) $display("   protocol monitor at %0t: %s", $time, what);
	end
endtask

reg [4:0]  rst_grace = 5'd0;
reg        awv_d = 0, wv_d = 0, arv_d = 0, bv_d = 0, rv_d = 0;
reg        awr_d = 1, wr_d = 1, arr_d = 1;
reg [31:0] awaddr_s = 0, wdata_s = 0, araddr_s = 0;
reg  [3:0] awlen_s = 0, awcache_s = 0, wstrb_s = 0, arlen_s = 0, arcache_s = 0;
reg  [2:0] awsize_s = 0, awprot_s = 0, arsize_s = 0, arprot_s = 0;
reg  [1:0] awburst_s = 0, awlock_s = 0, arburst_s = 0, arlock_s = 0;
reg  [4:0] awuser_s = 0, aruser_s = 0;
reg  [7:0] awid_s = 0, wid_s = 0, arid_s = 0;
reg        wlast_s = 0;

reg        w_out = 0, r_out = 0;
reg        w_counted = 0, r_counted = 0;
reg [15:0] w_off_q = 0, r_off_q = 0;
reg [31:0] w_data_q = 0;
integer    w_t0 = 0, r_t0 = 0;

integer txn_starts = 0, txn_done = 0;
reg     first_arm = 0, first_seen = 0;
integer first_off = 0;
reg     first_we = 0;

// the monitor's counters (cleared at each hello, as the DIAG module's are)
integer m_rd = 0, m_wr = 0, m_rd_sum = 0, m_wr_sum = 0;
integer m_rd_max = 0, m_wr_max = 0, m_rd_min = 16'hFFFF, m_wr_min = 16'hFFFF;
integer m_rd_err = 0, m_wr_err = 0;
integer m_rd_hist [0:15];
integer m_wr_hist [0:15];
integer m_hello = 0;
integer m_term = 0;
integer m_hidx_reads = 0, m_hello_reads = 0;
integer m_deepest = 0;
integer tail_seen = 0;        // the master's last-read tail, not the host's last store

// records and the snapshot
integer records_started = 0, records_published = 0, words_published = 0;
integer rec_writes = 0;
integer pub_head = 0;
integer last_rec_start = 0, gap_min = 1000000, gap_max = 0, gap_n = 0;
reg     snap_arm = 0, snap_taken = 0, snap_done = 0;
integer snap_seq = 0;
integer s_rd = 0, s_wr = 0, s_rd_sum = 0, s_wr_sum = 0, s_hello = 0, s_deepest = 0;
integer s_rd_max = 0, s_wr_max = 0, s_rd_min = 0, s_wr_min = 0, s_rd_err = 0, s_wr_err = 0;
integer s_ns = 0, s_flood = 0;
integer s_rd_hist [0:15];
integer s_wr_hist [0:15];

function integer bin_of(input integer lat);
	begin
		if (lat < 64)        bin_of = lat / 8;
		else if (lat < 128)  bin_of = 8;
		else if (lat < 256)  bin_of = 9;
		else if (lat < 512)  bin_of = 10;
		else if (lat < 1024) bin_of = 11;
		else if (lat < 2048) bin_of = 12;
		else if (lat < 4096) bin_of = 13;
		else if (lat < 65536)bin_of = 14;
		else                 bin_of = 15;
	end
endfunction

function [31:0] snap_word(input integer i);
	begin
		snap_word = 32'd0;
		case (i)
		0:  snap_word = (s_hello << 24) | (s_flood << 17) | (s_ns << 16) | s_deepest;
		1:  snap_word = s_rd;
		2:  snap_word = s_wr;
		3:  snap_word = (s_rd_max << 16) | s_rd_min;
		4:  snap_word = (s_wr_max << 16) | s_wr_min;
		5:  snap_word = s_rd_sum;
		6:  snap_word = s_wr_sum;
		7:  snap_word = (s_rd_err << 16) | s_wr_err;
		default: begin
			if (i < 24) snap_word = s_rd_hist[i - 8];
			else        snap_word = s_wr_hist[i - 24];
		end
		endcase
	end
endfunction

task mon_stat_clear;
	begin
		gap_min = 1000000; gap_max = 0; gap_n = 0;
	end
endtask

always @(posedge clk) begin
	integer lat, b;
	cyc = cyc + 1;
	if (!hps_rst_n) begin
		rst_grace = 5'd16;
		w_out = 0; r_out = 0;
	end
	else if (rst_grace != 0) rst_grace = rst_grace - 1;

	// valid stability: while a valid is held without its ready, it and its payloads hold
	if (rst_grace == 0) begin
		if (awv_d && !awr_d && (!awvalid || awaddr !== awaddr_s || awlen !== awlen_s ||
		    awsize !== awsize_s || awburst !== awburst_s || awlock !== awlock_s ||
		    awcache !== awcache_s || awprot !== awprot_s || awuser !== awuser_s || awid !== awid_s))
			merr("AWVALID or its payload changed before AWREADY");
		if (wv_d && !wr_d && (!wvalid || wdata !== wdata_s || wstrb !== wstrb_s ||
		    wlast !== wlast_s || wid !== wid_s))
			merr("WVALID or its payload changed before WREADY");
		if (arv_d && !arr_d && (!arvalid || araddr !== araddr_s || arlen !== arlen_s ||
		    arsize !== arsize_s || arburst !== arburst_s || arlock !== arlock_s ||
		    arcache !== arcache_s || arprot !== arprot_s || aruser !== aruser_s || arid !== arid_s))
			merr("ARVALID or its payload changed before ARREADY");
		if (bvalid && !bready) merr("BVALID without BREADY");
		if (rvalid && !rready) merr("RVALID without RREADY");
	end
	if (awvalid && !awready) begin
		awaddr_s = awaddr; awlen_s = awlen; awsize_s = awsize; awburst_s = awburst;
		awlock_s = awlock; awcache_s = awcache; awprot_s = awprot; awuser_s = awuser; awid_s = awid;
	end
	if (wvalid && !wready) begin wdata_s = wdata; wstrb_s = wstrb; wlast_s = wlast; wid_s = wid; end
	if (arvalid && !arready) begin
		araddr_s = araddr; arlen_s = arlen; arsize_s = arsize; arburst_s = arburst;
		arlock_s = arlock; arcache_s = arcache; arprot_s = arprot; aruser_s = aruser; arid_s = arid;
	end
	// the constants of §3, on every cycle a valid is asserted
	if (awvalid) begin
		if (awlen !== 4'd0 || awsize !== 3'b010 || awburst !== 2'b01 || awlock !== 2'b00 ||
		    awcache !== 4'b0000 || awuser !== 5'd0 || awid !== 8'd0) merr("a write address constant");
		if (awaddr[31:16] !== 16'hFFFF) merr("AWADDR's upper half");
		if (!((awaddr[15:0] >= 16'h0080 && awaddr[15:0] <= 16'h00FF) ||
		      (awaddr[15:0] >= EVT_BASE && awaddr[15:0] <= EVT_BASE + EVT_WORDS * 4 - 1)))
			merr("AWADDR outside the fabric half and the EVT span");
		if (awprot !== {1'b0, exp_ns[0], 1'b0}) merr("AWPROT");
	end
	if (wvalid) begin
		if (wstrb !== 4'hF || wlast !== 1'b1 || wid !== 8'd0) merr("a write data constant");
	end
	if (arvalid) begin
		if (arlen !== 4'd0 || arsize !== 3'b010 || arburst !== 2'b01 || arlock !== 2'b00 ||
		    arcache !== 4'b0000 || aruser !== 5'd0 || arid !== 8'd0) merr("a read address constant");
		if (araddr[31:16] !== 16'hFFFF) merr("ARADDR's upper half");
		if (!(araddr[15:0] === H_HELLO || araddr[15:0] === H_LINE || araddr[15:0] === H_IDX3))
			merr("ARADDR outside H_HELLO, H_LINE and H_IDX3");
		if (arprot !== {1'b0, exp_ns[0], 1'b0}) merr("ARPROT");
	end

	// transaction starts and completions
	if (awvalid && !awv_d) begin
		if (w_out || r_out) merr("a write starts while a transaction is in flight");
		txn_starts = txn_starts + 1;
		w_out = 1;
		w_off_q = awaddr[15:0];
		w_data_q = wdata;
		w_t0 = cyc;
		w_counted = !((awaddr[15:0] >= EVT_BASE && awaddr[15:0] <= EVT_BASE + EVT_WORDS * 4 - 1)) &&
		            !((awaddr[15:0] == C_HELLO) && (wdata == C_HELLO_STOP));
		if (first_arm && !first_seen) begin
			first_off = awaddr[15:0]; first_we = 1; first_seen = 1;
		end
	end
	if (arvalid && !arv_d) begin
		if (w_out || r_out) merr("a read starts while a transaction is in flight");
		txn_starts = txn_starts + 1;
		r_out = 1;
		r_off_q = araddr[15:0];
		r_t0 = cyc;
		r_counted = 1;
		if (first_arm && !first_seen) begin
			first_off = araddr[15:0]; first_we = 0; first_seen = 1;
		end
	end
	if (bvalid && bready && w_out) begin
		w_out = 0;
		txn_done = txn_done + 1;
		if (w_counted) begin
			lat = cyc - w_t0;
			m_wr = m_wr + 1;
			m_wr_sum = m_wr_sum + lat;
			b = (lat > 65535) ? 65535 : lat;
			if (b > m_wr_max) m_wr_max = b;
			if (b < m_wr_min) m_wr_min = b;
			if (bresp != 2'b00 && m_wr_err < 65535) m_wr_err = m_wr_err + 1;
			m_wr_hist[bin_of(lat)] = m_wr_hist[bin_of(lat)] + 1;
		end
		if (w_off_q == C_HELLO && w_data_q == C_HELLO_STOP) m_term = m_term + 1;
		if (w_off_q == C_HELLO && w_data_q[31:16] == 16'h4F43 && w_data_q[15:0] != 16'd0)
			m_hello = m_hello + 1;
	end
	if (rvalid && rready && r_out) begin
		r_out = 0;
		txn_done = txn_done + 1;
		if (r_off_q == H_IDX3) begin
			m_hidx_reads = m_hidx_reads + 1;
			tail_seen = rdata[15:0];
			b = (pub_head - tail_seen) & 16'hFFFF;
			if (b <= EVT_WORDS && b > m_deepest) m_deepest = b;
		end
		if (r_off_q == H_HELLO) m_hello_reads = m_hello_reads + 1;
		if (r_counted) begin
			lat = cyc - r_t0;
			m_rd = m_rd + 1;
			m_rd_sum = m_rd_sum + lat;
			b = (lat > 65535) ? 65535 : lat;
			if (b > m_rd_max) m_rd_max = b;
			if (b < m_rd_min) m_rd_min = b;
			if (rresp != 2'b00 && m_rd_err < 65535) m_rd_err = m_rd_err + 1;
			m_rd_hist[bin_of(lat)] = m_rd_hist[bin_of(lat)] + 1;
		end
	end

	// the DIAG counters clear at a hello: the C_VERSION store is its first transaction
	if (awvalid && !awv_d && awaddr[15:0] == C_VERSION && wdata == 32'h0001_0000) begin
		m_rd = 0; m_wr = 0; m_rd_sum = 0; m_wr_sum = 0;
		m_rd_max = 0; m_wr_max = 0; m_rd_min = 16'hFFFF; m_wr_min = 16'hFFFF;
		m_rd_err = 0; m_wr_err = 0;
		for (b = 0; b < 16; b = b + 1) begin m_rd_hist[b] = 0; m_wr_hist[b] = 0; end
		m_deepest = 0; tail_seen = 0;
		records_started = 0; records_published = 0; words_published = 0;
		rec_writes = 0; pub_head = 0;
	end

	// the event-ring accounting from the stores the master makes
	if (awvalid && !awv_d) begin
		if (awaddr[15:0] >= EVT_BASE && awaddr[15:0] <= EVT_BASE + EVT_WORDS * 4 - 1) begin
			// this record's next 16-bit index; every EVT write is in the record being written, in its place,
			// and never over a word the host has not consumed
			b = (pub_head + rec_writes) & 16'hFFFF;
			if (rec_writes >= 41) merr("an EVT record longer than its header's length");
			if (((b - host_tail) & 16'hFFFF) < ((pub_head - host_tail) & 16'hFFFF) &&
			    ((pub_head - host_tail) & 16'hFFFF) <= EVT_WORDS)
				merr("an EVT word stored over one the host has not consumed");
			if (((awaddr[15:2] - (EVT_BASE >> 2)) & (EVT_WORDS - 1)) !== (b & (EVT_WORDS - 1)))
				merr("an EVT word outside its record's place");
			if (rec_writes == 0) begin
				if (wdata !== MARK_HEADER) merr("a record's header word");
				if (records_started > 0) begin
					b = cyc - last_rec_start;
					if (b < gap_min) gap_min = b;
					if (b > gap_max) gap_max = b;
					gap_n = gap_n + 1;
				end
				last_rec_start = cyc;
				if (snap_arm) begin
					s_rd = m_rd; s_wr = m_wr; s_rd_sum = m_rd_sum; s_wr_sum = m_wr_sum;
					s_rd_max = m_rd_max; s_wr_max = m_wr_max;
					s_rd_min = m_rd_min; s_wr_min = m_wr_min;
					s_rd_err = m_rd_err; s_wr_err = m_wr_err;
					for (b = 0; b < 16; b = b + 1) begin
						s_rd_hist[b] = m_rd_hist[b]; s_wr_hist[b] = m_wr_hist[b];
					end
					s_hello = m_hello; s_deepest = m_deepest;
					s_ns = exp_ns[0]; s_flood = exp_flood[0];
					snap_seq = records_started;
					snap_taken = 1;
					snap_arm = 0;
				end
				records_started = records_started + 1;
			end
			rec_writes = rec_writes + 1;
		end
		if (awaddr[15:0] == C_IDX3) begin
			if (rec_writes == 41) begin
				if (wdata[15:0] !== ((pub_head + 41) & 16'hFFFF)) merr("the published head");
				pub_head = wdata[15:0];
				b = (pub_head - (tail_seen & 16'hFFFF)) & 16'hFFFF;
				if (b <= EVT_WORDS && b > m_deepest) m_deepest = b;
				records_published = records_published + 1;
				words_published = words_published + 41;
			end
			else if (rec_writes == 0) begin
				pub_head = 0;               // the hello's C_IDX[3] = 0
			end
			else merr("a C_IDX[3] store with a half-written record");
			rec_writes = 0;
		end
	end

	// the edge detectors, for the next cycle's rise and stability checks
	awv_d = awvalid; awr_d = awready; wv_d = wvalid; wr_d = wready;
	arv_d = arvalid; arr_d = arready; bv_d = bvalid; rv_d = rvalid;
end

// the host's consumer: a published record is read at the tail, checked, then the tail is stored.
task consume_record(output integer ok);
	integer i, h;
	reg [31:0] v, hdr;
	begin
		ok = 0;
		peek(C_IDX3, v);
		h = v[15:0];
		if (((h - host_tail) & 16'hFFFF) >= 41) begin
			hdr = evt_word(host_tail);
			check("MARK record: header", hdr, MARK_HEADER);
			for (i = 1; i <= 40; i = i + 1) begin
				v = evt_word(host_tail + i[15:0]);
				if (snap_taken && (consumed_seq == snap_seq))
					check($sformatf("MARK record: payload word %0d against the monitor", i - 1), v, snap_word(i - 1));
			end
			if (snap_taken && (consumed_seq == snap_seq)) begin snap_taken = 0; snap_done = 1; end
			host_tail = (host_tail + 16'd41);
			poke(H_IDX3, {16'd0, host_tail});
			consumed_seq = consumed_seq + 1;
			ok = 1;
		end
		else repeat (20) @(posedge clk);
	end
endtask

////////////////////////////////  the cases  ////////////////////////////////

integer txn0, term0, w0, p0, h0, v0, ok, ok1, ok2, i, n, bad;
reg [31:0] v;

task wait_until_pub(input integer n, output integer ok);
	integer c;
	begin
		ok = 0; c = 0;
		while (!ok && c < 500000) begin          // 5 ms
			if ((records_published - p0) >= n) ok = 1;
			else begin @(posedge clk); c = c + 1; end
		end
	end
endtask
task wait_until_start(input integer n, output integer ok);
	integer c;
	begin
		ok = 0; c = 0;
		while (!ok && c < 500000) begin          // 5 ms
			if ((records_started - h0) >= n) ok = 1;
			else begin @(posedge clk); c = c + 1; end
		end
	end
endtask

// 1. stock host
task case1;
	begin
		$display("-- stock host: random GPO traffic on every bit but 28, long: zero transactions");
		reset_floor;
		txn0 = txn_starts;
		for (i = 0; i < 200; i = i + 1) gpo_store(($random(seed) & 32'h7FFF_FFFF) & 32'hEFFF_FFFF);
		repeat (200) @(posedge clk);
		check("stock host: transactions", txn_starts - txn0, 0);
		check("stock host: present stays 1", present_wire, 1);
	end
endtask

// 2. the hello
task case2;
	begin
		$display("-- the hello: ARMED, the echo and every C_ word; the count is 1, then 2 in a new run");
		reset_floor;
		first_arm = 1; first_seen = 0;
		run_edge(0, 0);
		wait_armed(1000, ok);
		check("ARMED within 1 ms of the edge", ok, 1);
		check("the run's first transaction reads H_HELLO", (first_seen && first_off == H_HELLO && !first_we), 1);
		first_arm = 0;
		peek(C_HELLO, v);   check("C_HELLO ARMED", v, C_HELLO_ARMED);
		peek(C_VERSION, v); check("no C_VERSION before the hello", v, 32'd0);
		peek(C_CAPS, v);    check("no C_CAPS before the hello", v, 32'd0);
		poke(H_HELLO, {16'h4F43, 16'h1234});
		wait_echo(16'h1234, 1000, ok);
		check("the echo within 1 ms", ok, 1);
		peek(C_HELLO, v);   check("C_HELLO is the echo", v, {16'h4F43, 16'h1234});
		peek(C_VERSION, v); check("C_VERSION 1.0", v, 32'h0001_0000);
		peek(C_CAPS, v);    check("C_CAPS: the EVT ring", v, 32'h0000_0008);
		peek(C_STATE, v);   check("C_STATE: one hello, no fault", v, 32'd1);
		peek(C_IDX3, v);    check("C_IDX[3] = 0", v, 32'd0);
		peek(C_BIND, v);    check("C_BIND = 0", v, 32'd0);
		peek(C_RING3, v);   check("C_RING[3] descriptor", v, 32'h212C_1000);
		peek(16'h00D0, v);  check("C_K[0] not stored", v, 32'd0);
		peek(16'h00E4, v);  check("the other C_RING words not stored", v, 32'd0);
		gpo_store(GPO_IO);
		wait_stopped(1000, ok);
		check("a stop stores the terminal value", ok, 1);
		reset_floor;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		run_edge(0, 0); wait_armed(1000, ok);
		poke(H_HELLO, {16'h4F43, 16'h2345});
		wait_echo(16'h2345, 1000, ok);
		check("a second hello in a new run", ok, 1);
		peek(C_STATE, v); check("C_STATE: two hellos", v, 32'd2);
	end
endtask

// 3. a stale page
task case3;
	begin
		$display("-- a stale hello on the page: WAITING, never an echo, until H_HELLO reads 0");
		reset_floor;
		poke(H_HELLO, {16'h4F43, 16'h0077});
		run_edge(0, 0);
		wait_word(C_HELLO, C_HELLO_WAIT, 1000, ok);
		check("a stale page stores WAITING", ok, 1);
		repeat (3 << POLLC) @(posedge clk);
		peek(C_HELLO, v);   check("never an echo: C_HELLO", v, C_HELLO_WAIT);
		peek(C_VERSION, v); check("never a hello: C_VERSION", v, 32'd0);
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		check("H_HELLO = 0 then ARMED", ok, 1);
	end
endtask

// 4. an edge, not a level
task case4;
	begin
		$display("-- edge, not level: RUN held across a bit-31 fall and return; a core-reset store");
		reset_floor;
		start_session(0, 0, 0, 16'h0101, ok);
		gpo_store(32'h1000_0000);                 // bit 31 low, RUN still 1
		wait_stopped(1000, ok);
		check("the bit-31 fall stores the terminal value", ok, 1);
		txn0 = txn_starts;
		gpo_store(GPO_RUN);                       // bit 31 back, RUN still 1
		repeat (4000) @(posedge clk);
		check("bit 31's return starts no transaction", txn_starts - txn0, 0);
		peek(C_HELLO, v); check("C_HELLO stays stopped", v, C_HELLO_STOP);
		reset_floor;
		start_session(0, 0, 0, 16'h0102, ok);
		gpo_store(32'h0000_0000);                 // a core-reset store: bits 31 and 28 both 0
		wait_stopped(1000, ok);
		check("the core-reset store stores the terminal value", ok, 1);
	end
endtask

// 5. the stop
task case5;
	integer mid;
	begin
		$display("-- the stop: the in-flight transaction completes, one terminal store, no head, bit 19 next");
		reset_floor;
		start_session(0, 1, 8, 16'h0201, ok);
		w0 = words_published;  p0 = records_published;
		term0 = m_term;
		// stop with a record part-written
		mid = 0;
		i = 0;
		while (!mid && i < 40000) begin
			@(posedge clk); i = i + 1;
			if (rec_writes > 0 && rec_writes < 41) mid = 1;
		end
		check("a record is part-written when RUN falls", mid, 1);
		gpo_store(GPO_IO);
		wait_stopped(1000, ok);
		check("the stop stores the terminal value", ok, 1);
		check("exactly one terminal store", m_term - term0, 1);
		peek(C_IDX3, v);
		check("the abandoned record's head was not stored", v[15:0], (pub_head & 16'hFFFF));
		check("the published head is the complete records'", pub_head & 16'hFFFF,
		      (records_published * 41) & 16'hFFFF);
		txn0 = txn_starts;
		repeat (4000) @(posedge clk);
		check("nothing after the terminal store", txn_starts - txn0, 0);
		reset_floor;
		start_session(0, 0, 0, 16'h0202, ok);
		peek(C_STATE, v);
		check("the next session's C_STATE bit 19", v[19], 1);
	end
endtask

// 6. the HPS reset
task case6;
	begin
		$display("-- HPS reset: no terminal, no start while it is seen, an unanswered transaction dropped");
		reset_floor;
		start_session(0, 1, 0, 16'h0301, ok);
		term0 = m_term;
		repeat (30) @(posedge clk);
		hps_rst_n = 0;
		wait (dut.hrst_q === 1'b1);
		repeat (8) @(posedge clk);
		txn0 = txn_starts;
		repeat (60) @(posedge clk);
		check("nothing starts while the HPS reset is seen", txn_starts - txn0, 0);
		check("no terminal store while it is high", m_term - term0, 0);
		hps_rst_n = 1;
		repeat (16) @(posedge clk);
		check("no terminal store after it", m_term - term0, 0);
		reset_floor;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		run_edge(0, 0); wait_armed(1000, ok);
		check("a new RUN edge after the reset works", ok, 1);
		$display("-- HPS reset with a transaction never answered: it is dropped at the reset's fall");
		reset_floor;
		mode = M_NEVER;
		zero_page;
		run_edge(0, 0);
		ok = 0; i = 0;
		while (!ok && i < 10000) begin @(posedge clk); i = i + 1; if (dut.arvalid === 1'b1) ok = 1; end
		check("the first read hangs", ok, 1);
		term0 = m_term;
		hps_rst_n = 0;
		wait (dut.hrst_q === 1'b1);
		repeat (8) @(posedge clk);
		hps_rst_n = 1;
		repeat (16) @(posedge clk);
		check("the reset drops the unanswered read", dut.arvalid, 0);
		check("no terminal store after the reset", m_term - term0, 0);
		mode = M_NORMAL;
	end
endtask

// 7. a 60 ns 0 store
task case7;
	begin
		$display("-- a 0 lasting one host store (60 ns): the latch sets");
		reset_floor;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		run_edge(0, 0); wait_armed(1000, ok);
		term0 = m_term;
		#60; gpo_reg = GPO_IO;       // RUN 0 for 60 ns
		#60; gpo_reg = GPO_RUN;      // RUN 1: the edge
		ok = 0; i = 0;
		while (!ok && i < 100000) begin            // 1 ms
			@(posedge clk); i = i + 1;
			if ((m_term - term0) == 1) ok = 1;
		end
		check("the 60 ns 0 store was seen: the terminal store", ok, 1);
		wait_armed(1000, ok);
		check("the new RUN edge arms again", ok, 1);
	end
endtask

// 8. the EVT ring: flood off, then flood on
task case8;
	integer wraps0, t0;
	begin
		$display("-- EVT: well-formed MARK records, published after their payload, never over the tail");
		$display("   flood off: one MARK per period");
		reset_floor;
		start_session(0, 0, 0, 16'h0401, ok);
		h0 = records_started;
		mon_stat_clear;
		wait_until_start(3, ok);
		check("three MARK records start", ok, 1);
		check("flood off: start-to-start gaps at least 2^MARK cycles",
		      (gap_n >= 2 && gap_min >= (1 << MARKC) - 2), 1);
		$display("-- EVT: flood on, the host drains; two wraps of the 2,048-word ring");
		reset_floor;
		start_session(0, 1, 0, 16'h0402, ok);
		wraps0 = words_published;
		mon_stat_clear;
		ok = 0; i = 0;
		while (!ok && i < 500000) begin          // 5 ms
			consume_record(ok1);
			i = i + 1;
			if ((words_published - wraps0) >= 2 * EVT_WORDS) ok = 1;
		end
		check("the ring wraps at least twice", ok, 1);
		check("flood on: the records run back to back", (gap_n > 3 && gap_min < (1 << MARKC)), 1);
		check("every started record is published", records_started - records_published, 0);
		check("no word was stored over an unconsumed one", mon_err, 0);
	end
endtask

// 9. the line
task case9;
	integer c0, b;
	begin
		$display("-- the line: H_LINE bit 3 high, records unread: the tail is re-read back to back");
		reset_floor;
		start_session(0, 0, 8, 16'h0501, ok);
		p0 = records_published;
		wait_until_pub(1, ok);
		ok1 = 0; i = 0;
		while (!ok1 && i < 100000) begin @(posedge clk); i = i + 1; if (line_out === 1'b1) ok1 = 1; end
		check("the line rises with records unread", ok1, 1);
		h0 = m_hidx_reads; v0 = m_hello_reads;
		repeat (4 << POLLC) @(posedge clk);
		check("the tail is re-read more often than it is polled", (m_hidx_reads - h0) > (m_hello_reads - v0), 1);
		repeat (40) @(posedge clk);
		peek(C_IDX3, v);
		c0 = txn_done;
		store_tail(v[15:0]);
		@(negedge clk); c0 = txn_starts;          // transactions issued after the store
		b = 0;
		while (line_out === 1'b1 && b < 200) begin @(negedge clk); b = txn_starts - c0; end
		check("the line falls within 4 transactions of the tail store", (line_out === 1'b0 && b <= 4), 1);
		$display("-- the line: H_LINE 0: never high");
		reset_floor;
		start_session(0, 1, 0, 16'h0502, ok);
		p0 = records_published;
		wait_until_pub(1, ok);
		bad = 0;
		for (i = 0; i < 2000; i = i + 1) begin @(posedge clk); if (line_out === 1'b1) bad = bad + 1; end
		check("H_LINE 0: cycles with the line high", bad, 0);
	end
endtask

// 10. a host tail beyond the ring
task case10;
	begin
		$display("-- a tail beyond the ring: C_STATE bit 11, no further record, the line low, until a hello");
		reset_floor;
		start_session(0, 1, 8, 16'h0601, ok);
		p0 = records_published;
		wait_until_pub(2, ok);
		repeat (40) @(posedge clk);
		peek(C_IDX3, v);
		store_tail(v[15:0] - 16'd2049);
		ok = 0; i = 0;
		while (!ok && i < 20000) begin
			@(posedge clk); i = i + 1;
			peek(C_STATE, v);
			if (v[11] === 1'b1) ok = 1;
		end
		check("C_STATE bit 11 is stored", ok, 1);
		check("the line is low", line_out, 0);
		w0 = words_published;
		repeat (4 << POLLC) @(posedge clk);
		check("no further record is published", words_published - w0, 0);
		// a new hello clears the fault
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		store_tail(16'd0);
		poke(H_LINE, 32'd8);
		poke(H_HELLO, {16'h4F43, 16'h0602});
		wait_echo(16'h0602, 1000, ok);
		peek(C_STATE, v);
		check("the new hello clears the fault bit", v[11], 0);
		p0 = records_published;
		wait_until_pub(1, ok);
		check("records resume after the new hello", ok, 1);
	end
endtask

// 11. the session ends when H_HELLO changes
task case11;
	integer cnt0;
	begin
		$display("-- the session's end by H_HELLO: 0 then ARMED, another value then WAITING");
		reset_floor;
		start_session(0, 0, 0, 16'h0701, ok);
		peek(C_STATE, v); cnt0 = v[7:0];
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		check("H_HELLO = 0: ARMED", ok, 1);
		poke(H_HELLO, {16'h4F43, 16'h0702});
		wait_echo(16'h0702, 1000, ok);
		check("a new nonce after 0: a new session", ok, 1);
		peek(C_STATE, v); check("the hello count advanced by one", v[7:0], (cnt0 + 1) & 8'hFF);
		cnt0 = v[7:0];
		poke(H_HELLO, 32'h1234_5678);
		wait_word(C_HELLO, C_HELLO_WAIT, 1000, ok);
		check("any other value: WAITING", ok, 1);
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		check("0 then ARMED again", ok, 1);
		poke(H_HELLO, {16'h4F43, 16'h0703});
		wait_echo(16'h0703, 1000, ok);
		check("a third session", ok, 1);
		peek(C_STATE, v); check("the hello count advanced again", v[7:0], (cnt0 + 1) & 8'hFF);
	end
endtask

// 12. the counters against the monitor
task case12;
	integer snap_seen, t0;
	begin
		$display("-- the counters: the next MARK's 40 words equal the monitor's numbers exactly");
		reset_floor;
		zero_page;
		poke(H_LINE, 32'd8); poke(H_IDX3, 32'd0);
		inj_wr_addr = C_CAPS; inj_wr_en = 1;
		inj_rd_addr = H_LINE; inj_rd_en = 1;
		run_edge(1, 0);
		wait_armed(1000, ok);
		poke(H_HELLO, {16'h4F43, 16'h0801});
		wait_echo(16'h0801, 1000, ok);
		ok = 0; i = 0;
		while (!ok && i < 200000) begin
			@(posedge clk); i = i + 1;
			if (m_rd_err >= 1 && m_wr_err >= 1) ok = 1;
		end
		check("the injected read and write errors are counted", ok, 1);
		snap_arm = 1; snap_taken = 0; snap_done = 0;
		ok = 0; i = 0;
		while (!ok && i < 500000) begin          // 5 ms
			consume_record(ok1);
			i = i + 1;
			if (snap_done) ok = 1;
		end
		check("the snapshot record is published and its 40 words compared", ok, 1);
	end
endtask

// 13. AxPROT and the test knob
task case13;
	integer d0, mr0;
	begin
		$display("-- AxPROT: GPO 27 = 0 is secure; with secure refused it never arms, and the errors count");
		reset_floor;
		mode = M_SECURE_REFUSE;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		d0 = dut.diag.rd_err; mr0 = m_rd_err;
		run_edge(0, 0);
		repeat (4 << POLLC) @(posedge clk);
		peek(C_HELLO, v); check("secure refused: never ARMED", v, 32'd0);
		check("secure refused: the monitor saw read errors", m_rd_err - mr0 > 0, 1);
		check("secure refused: the diag's read errors match", dut.diag.rd_err - d0, m_rd_err - mr0);
		mode = M_NORMAL;
		reset_floor;
		mode = M_SECURE_REFUSE;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		run_edge(1, 0);
		wait_armed(1000, ok);
		check("GPO 27 = 1: non-secure, and it arms", ok, 1);
		check("every AxPROT was 3'b010", mon_err, 0);
		mode = M_NORMAL;
	end
endtask

// 14. never answered
task case14;
	begin
		$display("-- never answered: the hung read holds its valid until an HPS reset drops it");
		reset_floor;
		mode = M_NEVER;
		zero_page;
		run_edge(0, 0);
		ok = 0; i = 0;
		while (!ok && i < 20000) begin @(posedge clk); i = i + 1; if (dut.arvalid === 1'b1) ok = 1; end
		repeat (2) @(posedge clk);
		check("the first read hangs", ok, 1);
		term0 = m_term; txn0 = txn_starts;
		gpo_store(GPO_IO);
		repeat (2000) @(posedge clk);
		check("the hung read keeps ARVALID", dut.arvalid, 1);
		check("no terminal store can complete", m_term - term0, 0);
		check("the master issues nothing new", txn_starts - txn0, 0);
		hps_rst_n = 0;
		wait (dut.hrst_q === 1'b1);
		repeat (8) @(posedge clk);
		hps_rst_n = 1;
		repeat (16) @(posedge clk);
		check("the HPS reset drops the hung read", dut.arvalid, 0);
		check("no terminal store after the reset", m_term - term0, 0);
		mode = M_NORMAL;
	end
endtask

///////////////////////////////  the run  ///////////////////////////////////

initial begin
	if (!$value$plusargs("lw_seed=%d", seed)) seed = 1;
	$display("ocram_tb: seed %0d, POLL %0d, MARK %0d", seed, POLLC, MARKC);
	hps_rst_n = 0;
	gpo_reg = 32'd0;
	mode = M_NORMAL;
	repeat (20) @(posedge clk);
	hps_rst_n = 1;
	repeat (20) @(posedge clk);
	gpo_reg = GPO_IO;

	$display("-- present: the floor drives GPI bit 26");
	check("present is the constant 1", present_wire, 1);

	case1;
	case2;
	case3;
	case4;
	case5;
	case6;
	case7;
	case8;
	case9;
	case10;
	case11;
	case12;
	case13;
	case14;

	$display("-- the protocol monitor over the whole suite");
	check("protocol invariant failures", mon_err, 0);
	check("present at the end", present_wire, 1);

	if (fails) begin
		$display("FAILED: %0d checks, %0d failures", checks, fails);
		$fatal(1, "ocram_tb: failures");
	end
	else begin
		$display("PASSED: %0d checks, 0 failures", checks);
		$finish;
	end
end

initial begin #200000000 $display("FAILED: timeout"); $finish; end

endmodule

`default_nettype wire
