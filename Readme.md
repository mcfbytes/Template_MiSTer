# Template core for MiSTer

## General description
This core contains the latest version of framework and will be updated when framework is updated. There will be no releases. This core is only for developers. Besides the framework, core demonstrates the basic usage. New or ported cores should use it as a template.

It's highly recommended to follow the notes to keep it standardized for easier maintenance and collaboration with other developers.

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
MISTER_DISABLE_VGA_SCALER| Removes the scaled picture on the analog output (see below)


#### Optional scaler features
Some macros remove a scaler feature to save FPGA resources. Each one sets a bit in the reply to the `UIO_SET_VIDEO` (0x20) command, so the HPS binary can tell the feature is missing. A clear bit means the feature is present.

Macro                     | 0x20 reply bit | Saves (Quartus 17.0.2, one fit)  | Lost
--------------------------|----------------|----------------------------------|---------------------------------
MISTER_DISABLE_DOWNSCALE  | 2              | about 270 ALMs, 5 M10K, 6 DSP    | any HDMI picture smaller than the core's output
MISTER_DISABLE_VGA_SCALER | 3              | about 150 ALMs, 1 M10K (estimate) | `vga_scaler=1` and the framebuffer on the analog output

**MISTER_DISABLE_DOWNSCALE is not safe with a stock HPS binary that does not read bit 2.** Without the downscale path the scaler cannot shrink a picture: if the HDMI output window is smaller than the core's picture (for example `video_mode` 640x480 with a 704-pixel-wide core, or a small `vscale_mode` result) the picture is broken. Enable it only in a core whose output always fits the usual 720p/1080p windows, and say so in the core's release notes.

**MISTER_DISABLE_VGA_SCALER is safe with any HPS binary.** The analog output shows the core's native picture, as with `vga_scaler=0`, and a core's `VGA_SCALER` request has no effect. It has no effect with MISTER_DUAL_SDRAM: dual SDRAM boards use the analog pins for the second SDRAM, so that build never had the scaled analog output, and bit 3 stays clear.

# Quartus version
Cores must be developed in **Quartus v17.0.x**. It's recommended to have updates, so it will be **v17.0.2**. Newer versions won't give any benefits to FPGA used in MiSTer, however they will introduce incompatibilities in project settings and it will make harder to maintain the core and collaborate with others. **So please stick to good old 17.0.x version.** You may use either Lite or Standard license.

