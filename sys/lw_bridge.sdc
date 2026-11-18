# LW bridge clock crossings (docs/LW_BRIDGE.md, "Clock-domain contracts"). Inert without MISTER_LW.
set lw_s0 [get_keepers -nowarn {*|lws0_*}]
if {[get_collection_size $lw_s0] > 0} {
	set lw_x   [get_keepers -nowarn {*_lwx* *_lwgray* *|lwtg_* *|lwa1_*}]
	set lw_a   [get_keepers -nowarn {*|lwa?_*}]
	set lw_q   [get_keepers -nowarn {*|lwq_*}]
	set lw_src [get_keepers -nowarn {*|lwsrc_*}]
	# first stages have no enable and no logic on D: every path that ends at one is a crossing.
	# asynchronous whatever clock group the core's clk_sys is in (a second PLL is not in sys_top.sdc's groups)
	set_false_path -to $lw_s0
	set_net_delay -max 8.0 -from $lw_x -to $lw_s0
	if {[get_collection_size $lw_a] > 0} { set_false_path -to $lw_a }
	# bundles: only the foreign data path is cut. The capture's own enable stays a timed same-clock path.
	if {[get_collection_size $lw_q] > 0 && [get_collection_size $lw_src] > 0} {
		set_false_path -from $lw_src -to $lw_q
		set_net_delay -max 8.0 -from $lw_src -to $lw_q
	}
	# Gray buses: every bit of one step inside 5 ns (0.8 of a 150 MHz period), bus by bus
	foreach g [list sector_w sector_r lane_w lane_r lane_rw lane_rr osd_w osd_r] {
		set src [get_keepers -nowarn "*|${g}_lwgray*"]
		if {[get_collection_size $src] > 0} { set_max_skew -from $src -to $lw_s0 5.0 }
	}
}
