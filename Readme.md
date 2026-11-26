# Template core for MiSTer

## General description
This core contains the latest version of framework and will be updated when framework is updated. There will be no releases. This core is only for developers. Besides the framework, core demonstrates the basic usage. New or ported cores should use it as a template.

It's highly recommended to follow the notes to keep it standardized for easier maintenance and collaboration with other developers.

## LW bridge framework (branch `lw-framework`)
This branch adds an optional second path between the HPS and the core: a memory-mapped window on the HPS lightweight bridge, beside the SPI link that `hps_io` decodes. Pads, an OSD shadow, the status back-channel and (optionally) disk blocks travel as plain 32-bit stores and register reads, with two interrupt lines. `hps_io` keeps every SPI decode, so a host that ignores the window drives the core as before. It is opt-in: with none of `MISTER_LW`, `MISTER_OCRAM` and `MISTER_DISABLE_SPI_*` defined, `sys/` builds exactly as upstream. Adoption, register map and contracts: [docs/LW_BRIDGE.md](docs/LW_BRIDGE.md). Testbench: `tb/run.sh`.

## Source structure

### Legend:
* `<core_name>` - you have to use the same name where you see this in this manual. Basically it's your core name.

### Standard MiSTer core should have following folders:
* `sys` - the framework. Basically it's prohibited to change any files in this folder. Framework updates may erase any customization in this folder. All MiSTer cores have to include sys folder as is from this core.
* `rtl` - the actual source of core. It's up to the developer how to organize the inner structure of this folder. Exception is pll folder/files (see below).
* `releases` - the folder where rbf files should be placed. format of each rbf is: <core_name>_YYYYMMDD.rbf (YYYYMMDD is date code of release).

### Other standard files:
* `<core_name>.qpf`- quartus project file. Copy it as is and then modify the line `PROJECT_REVISION = "<core_name>"` according to your core name.
* `<core_name>.qsf` - quartus settings file. In most cases you don't need to modify anything inside (although you may wont to adjust some settings in quartus - this is fine, but keep changes minimal). You also need to watch this file before you make a commit. Quartus in some conditions may "spit" all settings from different files into this file so it will become large. If you see this, then simply revert it to original file.
* `<core_name>.srf` - optional file to disable some warnings which are safe to disable and make message list more clean, so you will have less chance to miss some important warnings. You are free to modify it.
* `<core_name>.sdc` - optional file for constraints in case if core require some special constraints. You are free to modify it.
* `<core_name>.sv` - glue logic between framework and core. This is where you adapt core specific signals to framework.
* `files.qip` - list of all core files. You need to edit it manually to add/remove files. Quartus will use this file but can't edit it. If you add files in Quartus IDE, then they will be added to `<core_name>.qsf` which is recommended manually move them to `files.qip`.
* `clean.bat` - windows batch file to clean the whole project from temporary files. In most cases you don't need to modify it.
* `.gitignore` - list of files should be ignored by git, so temporary files wont be included in commits.
* `jtag.cdf` - it will be produced when you compile the core. By clicking it in Quartus IDE, you will launch programmer where you can send the core to MiSTer over USB blaster cable (see manual for DE10-nano how to connect it). This file normally is not present on cleaned project and not included in commits.

### PLL:
Framework implies use of at least one PLL in the core. Framework doesn't contain this PLL but requires it to be placed in `rtl` folder, so `pll` folder and `pll.v`, `pll.qip` files must be present, however PLL settings are up to the core.

### Verilog Macros

The following macros can be defined and will affect the framework features:

Macro                    |   Effect
-------------------------|---------------------------------
MISTER_DEBUG_NOHDMI      | Disable HDMI-related modules. Speeds up compilation but only analogue/direct video is available
MISTER_DUAL_SDRAM        | Changes configuration of FPGA pins to work with dual SDRAM I/O boards
MISTER_FB                | Allows to use framebuffer from the core
MISTER_SMALL_VBUF        | Sets a smaller video buffer for the ASCAL
MISTER_DOWNSCALE_NN      | Ascal's downscale mode
MISTER_DISABLE_ADAPTIVE  | Disables adaptive scan lines
MISTER_FB_PALETTE        | Framebuffer palette
MISTER_DISABLE_DOWNSCALE | Removes Ascal's downscale path (see below)
MISTER_DISABLE_VGA_SCALER| Removes the scaled picture and the OSD on the analog output (see below)
MISTER_SMALL_HRES        | Ascal's output lines at most 2048 pixels wide (see below)
MISTER_LW                | LW bridge framework: the lightweight-bridge window beside hps_io (docs/LW_BRIDGE.md)
MISTER_LW_PAD            | LW PAD block and hps_io's LW write arm; needs MISTER_LW
MISTER_LW_KBM            | LW keys, mouse and triggers as PAD stores; needs MISTER_LW_PAD
MISTER_LW_EVT            | LW EVT block (status, block-request, info, upload flags); needs MISTER_LW
MISTER_LW_OSD            | LW OSDBUF block, a write port into each osd's osd_buffer; needs MISTER_LW
MISTER_LW_LANE           | LW frame lane with reply capture; needs MISTER_LW
LW_DIAG=0/1              | LW window diagnostics: 0 or absent shipping, 1 debug; needs a value (bare = error)
MISTER_LW_SECTOR_SLOT    | LW SECTOR block on this hps_io slot; with _FRAME, _GAP, MISTER_LW_BLKSZ (= hps_io's BLKSZ)
MISTER_LW_SAVE0_SLOT     | LW save slot 0 (likewise SAVE1); needs _WSRC, _EXTENT, the SECTOR slot
MISTER_DISABLE_SPI_PAD   | Removes the SPI link's pad commands; pads only over the LW PAD block (area-critical builds only, see below)
MISTER_DISABLE_SPI_KBM   | Removes the SPI key, mouse and trigger commands; needs MISTER_LW_KBM
MISTER_OCRAM             | OCRAM mailbox floor, test cores only (docs/OCRAM_MAILBOX.md); needs no other macro
MISTER_OCRAM_OSD         | OCRAM OSD tier; needs MISTER_OCRAM
MISTER_OCRAM_SECTOR_SLOT | OCRAM SECTOR tier on this slot; with _FRAME, _GAP, _BLKSZ; needs MISTER_OCRAM and MISTER_LW
MISTER_OCRAM_MOUNT0_SLOT | OCRAM mount slot 0 (likewise MOUNT1); _MOUNT_FRAME >= one block; needs OCRAM SECTOR
MISTER_OCRAM_FIO         | OCRAM file-download tier; with _FIO_GAP, _FIO_RING; needs MISTER_OCRAM
MISTER_OCRAM_UIO         | OCRAM link frames on the UIO and OSD selects; needs MISTER_OCRAM_FIO
MISTER_OCRAM_EVT         | OCRAM events tier (drives HPS_BUS[382]); needs MISTER_OCRAM and MISTER_LW, not MISTER_LW_EVT
MISTER_OCRAM_LANE        | OCRAM capture lane; needs MISTER_OCRAM_FIO
MISTER_OCRAM_DIAG        | OCRAM test-core diagnostics; needs MISTER_OCRAM
MISTER_OCRAM_POLL        | OCRAM idle poll period, a power of two of clk_100m cycles (default 12)
MISTER_DISABLE_SPI_OSD   | Removes osd.v's SPI row writes; needs MISTER_OCRAM_OSD or MISTER_LW_OSD
MISTER_DISABLE_SPI_BLK   | Removes hps_io's SPI block arms (_UNUSED = slot mask); needs a block tier per other slot
MISTER_DISABLE_SPI_EVT   | Removes hps_io's SPI event replies; needs MISTER_OCRAM_EVT

An LW or OCRAM tier macro without its floor stops the build, naming a guard module (`ocram_tier_needs_MISTER_OCRAM`,
`lw_blksz_differs_from_hps_io`, ...). Geometry, defaults and every guard: the macro tables in docs/LW_BRIDGE.md and
docs/OCRAM_MAILBOX.md.


#### Optional scaler features
Some macros remove a scaler feature to save FPGA resources. Each one that the HPS binary must know about sets a bit in the reply to the `UIO_SET_VIDEO` (0x20) command, so the HPS binary can tell the feature is missing. A clear bit means the feature is present.

Macro                     | 0x20 reply bit | Saves (Quartus 17.0.2, one fit)  | Lost
--------------------------|----------------|----------------------------------|---------------------------------
MISTER_DISABLE_DOWNSCALE  | 2              | about 270 ALMs, 5 M10K, 6 DSP    | any HDMI picture smaller than the core's output
MISTER_DISABLE_VGA_SCALER | 3              | about 1,000 ALMs, 0 M10K (one fit), and the analog osd's 470-496 ALMs, 6 M10K | `vga_scaler=1`, the framebuffer and the OSD on the analog output
MISTER_SMALL_HRES         | none           | about 112 ALUTs, 4 M10K (synthesis) | an image window more than 2048 pixels wide (only a custom modeline with 2049-2304 active pixels and no `pr`)

**Policy.** MISTER_SMALL_HRES may ship in a published core. MISTER_DISABLE_DOWNSCALE and MISTER_DISABLE_VGA_SCALER are for area-critical builds only: each is the core author's choice, never a default, and never enabled just because it fits.

**MISTER_DISABLE_DOWNSCALE is not safe with a stock HPS binary that does not read bit 2.** Without the downscale path the scaler cannot shrink a picture: if the HDMI output window is smaller than the core's picture (for example `video_mode` 640x480 with a 704-pixel-wide core, or a small `vscale_mode` result) the picture is broken. Stock Main does not read bit 2, so a published core that sets it breaks under stock Main whenever the window is smaller than the picture. Enable it only in a core whose output always fits the usual 720p/1080p windows, and say so in the core's release notes.

#### Optional link features
**MISTER_DISABLE_SPI_PAD** compiles out `hps_io`'s link arms for the pad commands (0x02, 0x03, 0x10-0x13, 0x1A, 0x3D), so pads reach the core only through the LW PAD block, which then reports it (PAD K9 bit 0). It saves about 85 ALMs (estimated). **Area-critical builds only, never a default: a stock HPS binary has no pad input.** It needs MISTER_LW_PAD, or the build stops naming `spi_pad_off_needs_MISTER_LW_PAD`.

**MISTER_DISABLE_VGA_SCALER works with any HPS binary, but the analog output has no OSD.** The analog output shows the core's native picture, as with `vga_scaler=0`, and a core's `VGA_SCALER` request has no effect. The analog path's osd is not built either: the menu and the info popups appear on HDMI only, so a setup with only an analog display (VGA, YPbPr, RGB or, without MISTER_DISABLE_YC, composite and S-Video) has no menu. The core's `OSD_STATUS` comes from the HDMI osd. With MISTER_DEBUG_NOHDMI the analog osd stays, as the only one. It has no effect with MISTER_DUAL_SDRAM: dual SDRAM boards use the analog pins for the second SDRAM, so that build never had the scaled analog output, keeps its osd as before, and bit 3 stays clear.

**MISTER_SMALL_HRES** builds Ascal with `OHRES` 2048 instead of 2304: its output line buffers end at 2048 pixels, while its counters still run to 4096, so an image window more than 2048 pixels wide shows a broken picture. No reply bit is needed, because stock Main never picks such a mode by itself: an EDID mode wider than 2048 falls back to the default mode, and a computed mode wider than 2048 uses pixel repetition (half the width, as `2560x1440` already does). Only a custom `video_mode` modeline with 2049-2304 active pixels and no `pr` flag loses its picture. Without MISTER_SMALL_HRES the Ascal generics are upstream's.

# Quartus version
Cores must be developed in **Quartus v17.0.x**. It's recommended to have updates, so it will be **v17.0.2**. Newer versions won't give any benefits to FPGA used in MiSTer, however they will introduce incompatibilities in project settings and it will make harder to maintain the core and collaborate with others. **So please stick to good old 17.0.x version.** You may use either Lite or Standard license.

