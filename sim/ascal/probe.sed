/^ARCHITECTURE rtl OF ascal IS/i\
USE work.ascal_probe_pkg.ALL;
/^END ARCHITECTURE rtl;/i\
	p_hfrac1 <= o_hfrac(1);\
	p_hfrac9 <= o_hfrac(9);\
	p_wr     <= o_wr;\
	p_wadl   <= o_wadl;\
	p_ldw    <= o_ldw.r & o_ldw.g & o_ldw.b;\
	p_copy   <= '0' WHEN o_copy=sWAIT ELSE '1';
