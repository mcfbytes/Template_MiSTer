# Template core for MiSTer

## General description
This core contains the latest version of framework and will be updated when framework is updated. There will be no releases. This core is only for developers. Besides the framework, core demonstrates the basic usage. New or ported cores should use it as a template.

It's highly recommended to follow the notes to keep it standardized for easier maintenance and collaboration with other developers.

## LW bridge framework
This branch adds an optional second path between the HPS and the core: a memory-mapped window on the HPS lightweight bridge, beside the SPI link that `hps_io` decodes. Pads, an OSD shadow, the status back-channel and (optionally) disk blocks travel as plain 32-bit stores and register reads, with two interrupt lines. `hps_io` keeps every SPI decode, so a host that ignores the window drives the core as before. Adoption, register map and contracts: [docs/LW_BRIDGE.md](docs/LW_BRIDGE.md).

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
MISTER_LW                | LW bridge framework: the lightweight-bridge window beside hps_io (docs/LW_BRIDGE.md)
MISTER_LW_PAD            | LW PAD block and hps_io's LW write arm; needs MISTER_LW
MISTER_LW_KBM            | LW keys, mouse and triggers as PAD stores; needs MISTER_LW_PAD
MISTER_LW_EVT            | LW EVT block (status, block-request, info, upload flags); needs MISTER_LW
MISTER_DISABLE_SPI_PAD   | Removes the SPI link's pad commands; pads only over the LW PAD block (area-critical builds only, see below)
MISTER_DISABLE_SPI_KBM   | Removes the SPI key, mouse and trigger commands; needs MISTER_LW_KBM

An LW or OCRAM tier macro without its floor stops the build, naming a guard module (`ocram_tier_needs_MISTER_OCRAM`,
`lw_blksz_differs_from_hps_io`, ...). Geometry, defaults and every guard: the macro tables in docs/LW_BRIDGE.md and
docs/OCRAM_MAILBOX.md.

#### Optional link features
**MISTER_DISABLE_SPI_PAD** compiles out `hps_io`'s link arms for the pad commands (0x02, 0x03, 0x10-0x13, 0x1A, 0x3D), so pads reach the core only through the LW PAD block, which then reports it (PAD K9 bit 0). It saves about 85 ALMs (estimated). **Area-critical builds only, never a default: a stock HPS binary has no pad input.** It needs MISTER_LW_PAD, or the build stops naming `spi_pad_off_needs_MISTER_LW_PAD`.

# Quartus version
Cores must be developed in **Quartus v17.0.x**. It's recommended to have updates, so it will be **v17.0.2**. Newer versions won't give any benefits to FPGA used in MiSTer, however they will introduce incompatibilities in project settings and it will make harder to maintain the core and collaborate with others. **So please stick to good old 17.0.x version.** You may use either Lite or Standard license.

