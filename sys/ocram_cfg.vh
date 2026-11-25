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

// The mailbox's .qsf macros as localparams, declared once. sys_top.v, hps_ocram_master.sv and hps_io.sv include
// it in their module body behind their own guard (MISTER_OCRAM; hps_io also MISTER_DISABLE_SPI_BLK), and each
// reads the names it needs. An enable is 1 when its tier macro is defined, whatever MISTER_OCRAM is (the
// includer tests that); a geometry value is its macro's, else the default below. Declarations only: it defines
// no macro and holds no logic.

/* verilator lint_off UNUSEDPARAM */   // each includer reads its own part

// The tiers.
`ifdef MISTER_OCRAM_OSD
localparam        OCRAM_OSD_EN    = 1;
`else
localparam        OCRAM_OSD_EN    = 0;
`endif
`ifdef MISTER_OCRAM_DIAG
localparam        OCRAM_TS_EN     = 1;
`else
localparam        OCRAM_TS_EN     = 0;
`endif
`ifdef MISTER_OCRAM_FIO
localparam        OCRAM_FIO_EN    = 1;
`else
localparam        OCRAM_FIO_EN    = 0;
`endif

// The download tier's declaration: the ring's log2 words (2-10 at 0x1000, or 13 at 0x7000) and the player's strobe
// gap (4-7); hps_ocram_rx stops the build on an unservable one.
`ifdef MISTER_OCRAM_FIO_RING
localparam [31:0] OCRAM_FIO_RING  = `MISTER_OCRAM_FIO_RING;
`else
localparam [31:0] OCRAM_FIO_RING  = 32'd10;
`endif
`ifdef MISTER_OCRAM_FIO_GAP
localparam [31:0] OCRAM_FIO_GAP   = `MISTER_OCRAM_FIO_GAP;
`else
localparam [31:0] OCRAM_FIO_GAP   = 32'd4;
`endif
// the selects an LFRAME may name, {osd_vga, osd_hdmi, fpga, uio}, for the walker and C_K[9]: the FPGA select alone;
// with MISTER_OCRAM_UIO also the UIO select and the OSD selects, HDMI only with an HDMI osd
`ifdef MISTER_OCRAM_UIO
`ifdef MISTER_DEBUG_NOHDMI
localparam  [3:0] OCRAM_FIO_SELS  = 4'b1011;
`else
localparam  [3:0] OCRAM_FIO_SELS  = 4'b1111;
`endif
`else
localparam  [3:0] OCRAM_FIO_SELS  = 4'b0010;
`endif

// The SECTOR tier's declaration: the hps_io slot, the frame in 32-bit words (1-4096), the strobe gap (2-15) and
// BLKSZ; hps_ocram_rx stops the build on an unservable geometry.
`ifdef MISTER_OCRAM_SECTOR_SLOT
localparam        OCRAM_SEC_EN    = 1;
localparam [31:0] OCRAM_SEC_SLOT  = `MISTER_OCRAM_SECTOR_SLOT;
`else
localparam        OCRAM_SEC_EN    = 0;
localparam [31:0] OCRAM_SEC_SLOT  = 32'd1;
`endif
`ifdef MISTER_OCRAM_SECTOR_FRAME
localparam [31:0] OCRAM_SEC_FRAME = `MISTER_OCRAM_SECTOR_FRAME;
`else
localparam [31:0] OCRAM_SEC_FRAME = 32'd128;
`endif
`ifdef MISTER_OCRAM_SECTOR_GAP
localparam [31:0] OCRAM_SEC_GAP   = `MISTER_OCRAM_SECTOR_GAP;
`else
localparam [31:0] OCRAM_SEC_GAP   = 32'd2;
`endif
`ifdef MISTER_OCRAM_BLKSZ
localparam  [2:0] OCRAM_BLKSZ     = `MISTER_OCRAM_BLKSZ;
localparam integer OCRAM_BLKSZ_DECL = `MISTER_OCRAM_BLKSZ;    // unsized for hps_io's check: [2:0] reads 10 as 2
`else
localparam  [2:0] OCRAM_BLKSZ     = 3'd2;
localparam integer OCRAM_BLKSZ_DECL = 2;
`endif

// The mount slots' declaration: each slot's hps_io slot and the frame in 32-bit words (1-1,020);
// hps_ocram_master stops the build on an unservable one.
`ifdef MISTER_OCRAM_MOUNT0_SLOT
localparam        OCRAM_MNT0_EN   = 1;
localparam [31:0] OCRAM_MNT0_SLOT = `MISTER_OCRAM_MOUNT0_SLOT;
`else
localparam        OCRAM_MNT0_EN   = 0;
localparam [31:0] OCRAM_MNT0_SLOT = 32'd2;
`endif
`ifdef MISTER_OCRAM_MOUNT1_SLOT
localparam        OCRAM_MNT1_EN   = 1;
localparam [31:0] OCRAM_MNT1_SLOT = `MISTER_OCRAM_MOUNT1_SLOT;
`else
localparam        OCRAM_MNT1_EN   = 0;
localparam [31:0] OCRAM_MNT1_SLOT = 32'd3;
`endif
`ifdef MISTER_OCRAM_MOUNT_FRAME
localparam [31:0] OCRAM_MNT_FRAME = `MISTER_OCRAM_MOUNT_FRAME;
`else
localparam [31:0] OCRAM_MNT_FRAME = 32'd128;
`endif
localparam  [1:0] OCRAM_MNT_EN    = {OCRAM_MNT1_EN != 0, OCRAM_MNT0_EN != 0};

// The OSD writer's rows: lw_osd's ROWS, 20 with MENU_CORE.
`ifdef MENU_CORE
localparam integer OCRAM_OSD_ROWS = 20;
`else
localparam integer OCRAM_OSD_ROWS = 16;
`endif

// C_CAPS: bit 3 the EVT ring, always; 2 the OSD ring, 4 ring 4 (the torture sink or the SECTOR ring), 10 the BIND
// record (SECTOR), 0 the mount ring, 1 the download ring, 11 the events tier, 12 the capture lane; the removals
// declared: 16 the link's block arms, 17 its OSD row writes, 18 its event replies.
localparam [31:0] OCRAM_C_CAPS    = 32'h0000_0008
`ifdef MISTER_OCRAM_OSD
                                  | 32'h0000_0004
`endif
`ifdef MISTER_OCRAM_DIAG
                                  | 32'h0000_0010
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
                                  | 32'h0000_0410
`endif
`ifdef MISTER_OCRAM_MOUNT0_SLOT
                                  | 32'h0000_0001
`endif
`ifdef MISTER_OCRAM_MOUNT1_SLOT
                                  | 32'h0000_0001
`endif
`ifdef MISTER_OCRAM_FIO
                                  | 32'h0000_0002
`endif
`ifdef MISTER_OCRAM_EVT
                                  | 32'h0000_0800
`endif
`ifdef MISTER_OCRAM_LANE
                                  | 32'h0000_1000
`endif
`ifdef MISTER_DISABLE_SPI_BLK
                                  | 32'h0001_0000
`endif
`ifdef MISTER_DISABLE_SPI_OSD
                                  | 32'h0002_0000
`endif
`ifdef MISTER_DISABLE_SPI_EVT
                                  | 32'h0004_0000
`endif
                                  ;

/* verilator lint_on UNUSEDPARAM */
