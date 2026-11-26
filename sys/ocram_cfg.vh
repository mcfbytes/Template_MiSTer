//============================================================================
//  ocram_cfg.vh -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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

// The mailbox's .qsf macros as localparams, declared once.

/* verilator lint_off UNUSEDPARAM */   // each includer reads its own part

// The tiers.
`ifdef MISTER_OCRAM_OSD
localparam        OCRAM_OSD_EN    = 1;
`else
localparam        OCRAM_OSD_EN    = 0;
`endif
localparam        OCRAM_TS_EN     = 0;
localparam        OCRAM_FIO_EN    = 0;

// The download tier's declaration: the ring's log2 words (2-10 at 0x1000, or 13 at 0x7000) and the player's strobe
// gap (4-7); hps_ocram_rx stops the build on an unservable one.
localparam [31:0] OCRAM_FIO_RING  = 32'd10;
localparam [31:0] OCRAM_FIO_GAP   = 32'd4;
localparam  [3:0] OCRAM_FIO_SELS  = 4'b0010;

// The SECTOR tier's declaration: the hps_io slot, the frame in 32-bit words (1-4096), the strobe gap (2-15) and
// BLKSZ; hps_ocram_rx stops the build on an unservable geometry.
localparam        OCRAM_SEC_EN    = 0;
localparam [31:0] OCRAM_SEC_SLOT  = 32'd1;
localparam [31:0] OCRAM_SEC_FRAME = 32'd128;
localparam [31:0] OCRAM_SEC_GAP   = 32'd2;
localparam  [2:0] OCRAM_BLKSZ     = 3'd2;
localparam integer OCRAM_BLKSZ_DECL = 2;

// The mount slots' declaration: each slot's hps_io slot and the frame in 32-bit words (one block, 32 << BLKSZ,
// to 1,020); hps_ocram_master stops the build on an unservable one.
localparam        OCRAM_MNT0_EN   = 0;
localparam [31:0] OCRAM_MNT0_SLOT = 32'd2;
localparam        OCRAM_MNT1_EN   = 0;
localparam [31:0] OCRAM_MNT1_SLOT = 32'd3;
localparam [31:0] OCRAM_MNT_FRAME = 32'd128;
localparam  [1:0] OCRAM_MNT_EN    = {OCRAM_MNT1_EN != 0, OCRAM_MNT0_EN != 0};

// The OSD writer's rows: lw_osd's ROWS, 20 with MENU_CORE.
`ifdef MENU_CORE
localparam integer OCRAM_OSD_ROWS = 20;
`else
localparam integer OCRAM_OSD_ROWS = 16;
`endif

localparam [31:0] OCRAM_C_CAPS    = 32'h0000_0008
`ifdef MISTER_OCRAM_OSD
                                  | 32'h0000_0004
`endif
`ifdef MISTER_DISABLE_SPI_OSD
                                  | 32'h0002_0000
`endif
                                  ;

/* verilator lint_on UNUSEDPARAM */
