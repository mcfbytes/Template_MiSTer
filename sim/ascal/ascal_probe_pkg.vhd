-- Bench-only observation points inside ascal. run.sh copies sys/ascal.vhd into the
-- build directory and appends concurrent copies of a few internal signals into these
-- package signals (see probe.sed). The copies have no effect on the design's logic.
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

PACKAGE ascal_probe_pkg IS
	SIGNAL p_hfrac1 : unsigned(11 DOWNTO 0) := (OTHERS => '0'); -- o_hfrac(1), first phase stage
	SIGNAL p_hfrac9 : unsigned(11 DOWNTO 0) := (OTHERS => '0'); -- o_hfrac(9), last phase stage
	SIGNAL p_wr     : unsigned(3 DOWNTO 0)  := (OTHERS => '0'); -- line buffer write enables
	SIGNAL p_wadl   : natural := 0;                             -- line buffer write address
	SIGNAL p_ldw    : unsigned(23 DOWNTO 0) := (OTHERS => '0'); -- line buffer write data
	SIGNAL p_copy   : std_logic := '0';                         -- a line copy is in progress
END PACKAGE ascal_probe_pkg;
