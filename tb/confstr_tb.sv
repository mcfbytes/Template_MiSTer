//============================================================================
//  confstr_tb.sv -- hps_io's 0x14 CONF_STR readout with CONF_STR_BRAM 0 and 1 (arm Q of tb/run.sh)
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
// Two stock hps_io instances hold a synthetic 3 KB CONF_STR (tb/conf_str_3k.vh), one in logic (CONF_STR_BRAM 0), one
// in the confstr_rom block RAM (1). One link drives both: the 0x14 command, then a strobe per byte. Each reply byte is
// compared between the two and against the string, at strobe spacings of 2 and 5 clk_sys cycles, with a read past
// the string's end. clk_sys at 33.8688 MHz.

`timescale 1ns/1ps
`default_nettype none

module confstr_tb;

`define BUILD_DATE "261124"
`include "conf_str_3k.vh"
localparam integer STRLEN = $size(CONF_STR) >> 3;

integer checks = 0, fails = 0;
task check(input string what, input [31:0] got, input [31:0] exp);
	begin
		checks = checks + 1;
		if (got !== exp) begin fails = fails + 1; $display("FAIL %s: got %08h exp %08h", what, got, exp); end
		else $display("pass %s = %08h", what, got);
	end
endtask

reg clk_sys = 0;
always #14.762 clk_sys = ~clk_sys;

reg         io_enable = 0, io_strobe = 0;
reg  [15:0] io_din = 0;
wire [45:0] bus_l, bus_b;
wire [35:0] ext_l, ext_b;
assign bus_l[31:16] = io_din, bus_b[31:16] = io_din;
assign bus_l[33] = io_strobe, bus_b[33] = io_strobe;
assign bus_l[34] = io_enable, bus_b[34] = io_enable;
assign bus_l[35] = 1'b0, bus_b[35] = 1'b0;
assign ext_l[32] = 1'b0, ext_b[32] = 1'b0;
assign ext_l[15:0] = 16'd0, ext_b[15:0] = 16'd0;

hps_io #(.CONF_STR(CONF_STR), .CONF_STR_BRAM(0), .WIDE(1), .VDNUM(4), .BLKSZ(3)) hps_logic
(
	.clk_sys(clk_sys), .HPS_BUS(bus_l), .EXT_BUS(ext_l)
);
hps_io #(.CONF_STR(CONF_STR), .CONF_STR_BRAM(1), .WIDE(1), .VDNUM(4), .BLKSZ(3)) hps_bram
(
	.clk_sys(clk_sys), .HPS_BUS(bus_b), .EXT_BUS(ext_b)
);

task strobe(input [15:0] v, input integer gap);
	begin
		@(negedge clk_sys); io_din = v; io_strobe = 1;
		@(negedge clk_sys); io_strobe = 0;
		repeat (gap - 1) @(negedge clk_sys);
	end
endtask

// the readout: 0x14, then STRLEN + 2 strobes; reply k is the byte the strobe with byte_cnt = k left in io_dout
task readout(input integer gap);
	integer k, bad_lb, bad_s;
	reg [7:0] want;
	begin
		bad_lb = 0; bad_s = 0;
		@(negedge clk_sys); io_enable = 1;
		strobe(16'h0014, gap);
		for (k = 1; k <= STRLEN + 2; k = k + 1) begin
			strobe(16'h0000, gap);
			if (bus_l[7:0] !== bus_b[7:0]) begin
				if (bad_lb < 4) $display("   byte %0d: logic %02h, block RAM %02h", k, bus_l[7:0], bus_b[7:0]);
				bad_lb = bad_lb + 1;
			end
			if (k <= STRLEN) begin
				want = CONF_STR[(STRLEN - k) * 8 +: 8];
				if (bus_b[7:0] !== want) bad_s = bad_s + 1;
			end
		end
		io_enable = 0; repeat (4) @(negedge clk_sys);
		check($sformatf("gap %0d: the block RAM's bytes equal the logic's, past the end too", gap), bad_lb, 0);
		check($sformatf("gap %0d: each byte is the string's", gap), bad_s, 0);
	end
endtask

initial begin
	$display("confstr_tb: a synthetic CONF_STR, %0d bytes", STRLEN);
	repeat (10) @(negedge clk_sys);
	$display("-- the 0x14 readout, CONF_STR_BRAM 0 against 1");
	check("the string is over 3 KB", STRLEN > 3072, 1);
	readout(2);
	readout(5);
	readout(2);
	if (fails == 0) $display("PASSED: %0d checks, 0 failures", checks);
	else $display("FAILED: %0d checks, %0d failures", checks, fails);
	$finish;
end

endmodule

`default_nettype wire
