//============================================================================
//  blkoff_tb.sv -- hps_io with MISTER_DISABLE_SPI_BLK (arm Y of tb/run.sh)
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

// A real hps_io (WIDE, VDNUM 4, BLKSZ 2) with MISTER_LW and the OCRAM SECTOR and mount macros: slot 0 unused, the
// SECTOR slot 1, the mount slots 2 and 3, MISTER_OCRAM_MOUNT_FRAME 256. The HPS's link words go in on HPS_BUS's
// I/O bits and the core's requests are the bench's own. Built with MISTER_DISABLE_SPI_BLK (and
// MISTER_DISABLE_SPI_BLK_UNUSED=1) the link's block arms are dead: GET_SDSTAT (0x16) reads 0 with requests raised,
// 0x17 and 0x18 raise no ack and no buffer strobe, and the commit mark reads 0 on every cycle, present. Built without
// them (the control) the same words show the arms alive. BLKOFF_TB_VIOLATE = 1-4 breaks one of the core's
// obligations under MISTER_DISABLE_SPI_BLK and ends; tb/run.sh expects hps_io's own simulation check to name it:
// 1 an unused slot requests, 2 the SECTOR slot writes, 3 a mount request over the mount frame, 4 the same request
// seen only on the save mux's core rd bit (its mux hides it from hps_io).

`timescale 1ns/1ps
`default_nettype none

module blkoff_tb;

`ifdef MISTER_DISABLE_SPI_BLK
localparam OFF = 1;
`else
localparam OFF = 0;
`endif

integer checks = 0, fails = 0;

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

reg clk_sys = 1'b0;
always #14.762 clk_sys = ~clk_sys;   // 33.8688 MHz

// HPS_BUS: the bits no piece drives read 0, as synthesis ties them
wire [383:0] bus;
assign (weak0, weak1) bus[383:46] = 338'd0;
assign bus[45:38] = 8'd0;
reg         io_strobe = 1'b0, io_enable = 1'b0;
reg  [15:0] io_din = 16'd0;
assign bus[35:33] = {1'b0, io_enable, io_strobe};
assign bus[31:16] = io_din;
wire [35:0] ext;
assign ext[32] = 1'b0;
`ifdef BLKOFF_TB_VIOLATE
reg         mux_rd = 1'b0;   // the save mux for MOUNT0: its core rd bit
assign bus[169] = mux_rd;
`endif

// the core's side of the four slots
reg  [31:0] lba0 = 0, lba1 = 0, lba2 = 0, lba3 = 0;
reg   [5:0] cnt0 = 0, cnt1 = 0, cnt2 = 0, cnt3 = 0;
reg  [15:0] din0 = 0, din1 = 0, din2 = 0, din3 = 0;
reg   [3:0] rd = 4'd0, wr = 4'd0;
wire [31:0] lba [4];
wire  [5:0] cnt [4];
wire [15:0] din [4];
assign lba[0] = lba0, lba[1] = lba1, lba[2] = lba2, lba[3] = lba3;
assign cnt[0] = cnt0, cnt[1] = cnt1, cnt[2] = cnt2, cnt[3] = cnt3;
assign din[0] = din0, din[1] = din1, din[2] = din2, din[3] = din3;
wire  [3:0] ack;
wire [12:0] baddr;
wire [15:0] bdout;
wire        bwr;

hps_io #(.CONF_STR("BLK;;"), .WIDE(1), .VDNUM(4)) hps_io
(
	.clk_sys(clk_sys), .HPS_BUS(bus), .EXT_BUS(ext),
	.sd_lba(lba), .sd_blk_cnt(cnt), .sd_rd(rd), .sd_wr(wr), .sd_ack(ack),
	.sd_buff_addr(baddr), .sd_buff_dout(bdout), .sd_buff_din(din), .sd_buff_wr(bwr)
);

// every cycle once mon_on: acks, buffer strobes, the mark and its presence bit
reg     mon_on = 1'b0;
integer n_ack = 0, n_bwr = 0, n_mark = 0, n_nopres = 0;
always @(posedge clk_sys) if (mon_on) begin
	if (ack !== 4'd0) n_ack = n_ack + 1;
	if (bwr !== 1'b0) n_bwr = n_bwr + 1;
	if (bus[380] !== 1'b0) n_mark = n_mark + 1;
	if (bus[381] !== 1'b1) n_nopres = n_nopres + 1;
end

// one link word: the strobe for a cycle, then the reply hps_io holds until the next one
task word(input [15:0] w, output [15:0] r);
	begin
		@(negedge clk_sys); io_din = w; io_strobe = 1'b1;
		@(negedge clk_sys); io_strobe = 1'b0;
		repeat (2) @(negedge clk_sys); r = bus[15:0];
	end
endtask
task txn_begin; begin @(negedge clk_sys); io_enable = 1'b1; end endtask
task txn_end; begin @(negedge clk_sys); io_enable = 1'b0; repeat (4) @(negedge clk_sys); end endtask

reg  [15:0] r0, r1, r2, r3;
integer     a0, b0, m0;

initial begin
	repeat (8) @(negedge clk_sys);
`ifdef BLKOFF_TB_VIOLATE
	case (`BLKOFF_TB_VIOLATE)
		1: rd = 4'b0001;                          // slot 0, named unused
		2: wr = 4'b0010;                          // the SECTOR slot
		3: begin cnt2 = 6'd2; rd = 4'b0100; end   // 384 words on a 256-word mount frame
		4: begin cnt2 = 6'd2; mux_rd = 1'b1; end  // the same, hidden from hps_io by its mux
	endcase
	repeat (8) @(negedge clk_sys);
	$display("PASSED: 0 checks, 0 failures (violation %0d raised)", `BLKOFF_TB_VIOLATE);
	$finish;
`endif
	mon_on = 1'b1;

	$display("-- GET_SDSTAT (0x16) with the SECTOR slot's read raised");
	lba1 = 32'h1234_5678; rd = 4'b0010;
	repeat (4) @(negedge clk_sys);
	m0 = n_mark;
	txn_begin; word(16'h0016, r0); word(16'h0000, r1); word(16'h0000, r2); word(16'h0000, r3); txn_end;
	if (OFF) check("0x16: every reply word 0", {r0 | r1 | r2 | r3}, 0);
	else begin
		check("control: 0x16 shows {1, slot 1, rd}", {r0[15], r0[5:0]}, {1'b1, 4'd1, 1'b0, 1'b1});
		check("control: 0x16 gives the LBA", {r3, r2}, 32'h1234_5678);
		check("control: the commit mark holds the request shown", bus[380], 1);
	end

	$display("-- 0x17 to slot 1");
	a0 = n_ack; b0 = n_bwr;
	txn_begin; word(16'h0117, r0); word(16'hA5A5, r1); word(16'h5A5A, r1); word(16'h1111, r1); word(16'h2222, r1);
	if (OFF) begin
		check("0x17: no ack", n_ack - a0, 0);
		check("0x17: no buffer strobe", n_bwr - b0, 0);
	end
	else begin
		check("control: 0x17 raises slot 1's ack", ack, 4'b0010);
		check("control: four buffer strobes", n_bwr - b0, 4);
	end
	txn_end;
	if (!OFF) check("control: the transfer's end clears the mark", bus[380], 0);

	$display("-- 0x18 from slot 1");
	din1 = 16'hBEEF; a0 = n_ack;
	txn_begin; word(16'h0118, r0); word(16'h0000, r1); word(16'h0000, r2);
	if (OFF) begin
		check("0x18: no ack", n_ack - a0, 0);
		check("0x18: every reply word 0", {r0 | r1 | r2}, 0);
	end
	else begin
		check("control: 0x18 raises slot 1's ack", ack, 4'b0010);
		check("control: 0x18 reads the buffer word", {r1, r2}, 32'hBEEF_BEEF);
	end
	txn_end;

	$display("-- the mount slots' requests, within the frame: a read of 256 words on slot 2, a write on slot 3");
	cnt2 = 6'd1; lba2 = 32'h0000_0100; wr = 4'b1000; rd = 4'b0110;
	repeat (4) @(negedge clk_sys);
	txn_begin; word(16'h0016, r0); word(16'h0000, r1); word(16'h0000, r2); word(16'h0000, r3); txn_end;
	if (OFF) check("0x16 with three slots requesting: every reply word 0", {r0 | r1 | r2 | r3}, 0);
	else check("control: 0x16 shows a request", r0[15], 1);
	rd = 4'd0; wr = 4'd0;
	repeat (8) @(negedge clk_sys);

	$display("-- the commit mark over the run");
	check("the mark's presence bit read 1 on every cycle", n_nopres, 0);
	if (OFF) check("the mark read 0 on every cycle", n_mark, 0);
	else check("control: the mark read 1 on some cycle", n_mark - m0 > 0, 1);

	$display("%s: %0d checks, %0d failures", fails ? "FAILED" : "PASSED", checks, fails);
	$finish;
end

endmodule

`default_nettype wire
