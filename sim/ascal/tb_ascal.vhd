--------------------------------------------------------------------------------
-- ascal regression bench (GHDL, VHDL-2008)
--
-- Drives sys/ascal.vhd with a scripted core video input, a fixed output mode, an
-- Avalon memory model with deterministic back-pressure, and optionally the
-- framebuffer (LFB) path. It writes three logs to OUTDIR:
--   vid.txt : one record per output line (HS rising edge):
--             L <frame> <line> <o_clk cycle> <DE pixels> <non-black DE pixels>
--               <hash of every output pin, every cycle>
--               <line buffer writes> <hash of writes: address, enables, data>
--               <hash of o_hfrac(9) per write> <hash of o_hfrac(1), every cycle of a line copy>
--             and one F record per output frame (VS rising edge).
--   avl.txt : every Avalon burst: R <avl cycle> <word address> <burstcount>
--             W <avl cycle of first beat> <word address> <burstcount> <hash of the data>
--   pix.txt : DUMP only: every DE pixel, every line buffer write with its phase, and
--             every change of o_hfrac(1) after reset (A <cycle> <phase>).
-- Everything is a pure function of the generics: no randomness, no wall clock.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE std.textio.ALL;
USE work.ascal_probe_pkg.ALL;

ENTITY tb_ascal IS
	GENERIC (
		OUTDIR   : string  := ".";
		NFRAMES  : natural := 1;     -- full output frames after the first VS
		DUMP     : boolean := false;
		-- ascal generics
		G_DOWNSCALE    : boolean := true;
		G_DOWNSCALE_NN : boolean := false;
		G_ADAPTIVE     : boolean := true;
		G_NBURST       : natural := 256;
		-- clocks, ps
		I_CLK_PS : natural := 20000;
		I_CE_DIV : natural := 2;
		O_CLK_PS : natural := 6734;
		A_CLK_PS : natural := 10000;
		-- core video, per field
		IN_W     : natural := 256;
		IN_H     : natural := 224;
		IN_HTOT  : natural := 341;
		IN_VTOT  : natural := 262;
		IN_INTER : boolean := false;
		-- a mid-run switch: at output frame SW_FRAME (0 = never) the input becomes
		-- IN2_W x IN2_H at its next field and the output window SW_HMIN..SW_HMAX
		SW_FRAME : natural := 0;
		IN2_W    : natural := 0;
		IN2_H    : natural := 0;
		SW_HMIN  : natural := 0;
		SW_HMAX  : natural := 0;
		-- a mid-line window change: at output frame MID_FRAME (0 = never), MID_CYC o_clk cycles
		-- after the HS edge that starts its line MID_LINE, the window becomes MID_HMIN..MID_HMAX
		MID_FRAME : natural := 0;
		MID_LINE  : natural := 0;
		MID_CYC   : natural := 0;
		MID_HMIN  : natural := 0;
		MID_HMAX  : natural := 0;
		-- MID_SEED: the change must fall in a copy gap, at least 16 o_clk after a copy ends and 4 to 11
		-- before a line's first copy may start (seed.txt says MISSED otherwise, and run.sh fails)
		MID_SEED  : boolean := false;
		-- output mode
		HTOTAL : natural := 2200; HSSTART : natural := 2008; HSEND : natural := 2052;
		HDISP  : natural := 1920; HMIN : natural := 240; HMAX : natural := 1679;
		VTOTAL : natural := 1125; VSSTART : natural := 1084; VSEND : natural := 1089;
		VDISP  : natural := 1080; VMIN : natural := 0; VMAX : natural := 1079;
		MODE    : natural := 12;     -- ascal mode: bit 3 triple buffer, bits 2-0 filter
		BOB     : boolean := false;
		SWBLACK : boolean := false;
		-- coefficients: 0 none, 1 horizontal + vertical, 2 plus the adaptive pair
		POLY : natural := 1;
		-- framebuffer mode
		FB_ENA    : boolean := false;
		FB_W      : natural := 0;
		FB_H      : natural := 0;
		FB_FMT    : natural := 6;     -- o_fb_format: 3=8bpp 4=16bpp 5=24bpp 6=32bpp
		FB_STRIDE : natural := 0
		);
END ENTITY tb_ascal;

ARCHITECTURE sim OF tb_ascal IS

	CONSTANT FB_BASE : unsigned(31 DOWNTO 0) := x"21800000";
	CONSTANT MEM_AW  : natural := 21; -- 32 MB of 128-bit words, aliased

	SIGNAL i_clk, o_clk, a_clk : std_logic := '0';
	SIGNAL done     : boolean := false;
	SIGNAL reset_na : std_logic := '0';

	SIGNAL i_r, i_g, i_b : unsigned(7 DOWNTO 0) := x"00";
	SIGNAL i_hs, i_vs, i_fl, i_de, i_ce : std_logic := '0';
	SIGNAL o_r, o_g, o_b : unsigned(7 DOWNTO 0);
	SIGNAL o_hs, o_vs, o_de, o_vbl, o_brd : std_logic;
	SIGNAL o_lltune : unsigned(15 DOWNTO 0);

	SIGNAL hmin_s : natural RANGE 0 TO 4095 := HMIN;
	SIGNAL hmax_s : natural RANGE 0 TO 4095 := HMAX;
	SIGNAL sw_in  : boolean := false;

	SIGNAL poly_a  : unsigned(11 DOWNTO 0) := (OTHERS => '0');
	SIGNAL poly_dw : unsigned(9 DOWNTO 0)  := (OTHERS => '0');
	SIGNAL poly_wr : std_logic := '0';
	SIGNAL pal1_dw : unsigned(47 DOWNTO 0) := (OTHERS => '0');
	SIGNAL pal1_a  : unsigned(6 DOWNTO 0)  := (OTHERS => '0');
	SIGNAL pal1_wr : std_logic := '0';

	SIGNAL avl_waitrequest   : std_logic := '1';
	SIGNAL avl_readdata      : std_logic_vector(127 DOWNTO 0) := (OTHERS => '0');
	SIGNAL avl_readdatavalid : std_logic := '0';
	SIGNAL avl_burstcount    : std_logic_vector(7 DOWNTO 0);
	SIGNAL avl_writedata     : std_logic_vector(127 DOWNTO 0);
	SIGNAL avl_address       : std_logic_vector(27 DOWNTO 0);
	SIGNAL avl_write, avl_read : std_logic;
	SIGNAL avl_byteenable    : std_logic_vector(15 DOWNTO 0);

	SIGNAL mode_s : unsigned(4 DOWNTO 0);

	SUBTYPE u32 IS unsigned(31 DOWNTO 0);
	-- Hash step on one word: rotate, xor, add (cheap in simulation, unlike a multiply)
	FUNCTION mix(h : u32; v : unsigned) RETURN u32 IS
		VARIABLE x : u32 := (h(26 DOWNTO 0) & h(31 DOWNTO 27)) XOR resize(v, 32);
	BEGIN
		RETURN x + x"9E3779B9";
	END FUNCTION;
	CONSTANT FNV0 : u32 := x"811C9DC5";

	FUNCTION b2sl(b : boolean) RETURN std_logic IS
	BEGIN
		IF b THEN RETURN '1'; ELSE RETURN '0'; END IF;
	END FUNCTION;

	-- Rounded integer division, half up, for any sign of the numerator.
	FUNCTION rdiv(n, d : integer) RETURN integer IS
		VARIABLE a : integer := 2*n + d;
	BEGIN
		IF a >= 0 THEN RETURN a / (2*d); ELSE RETURN -((-a + 2*d - 1) / (2*d)); END IF;
	END FUNCTION;

	-- 4-tap coefficients at phase p/256, unity = 256, integer arithmetic only.
	-- set 0: Catmull-Rom; 1: linear; 2: sharp linear; 3: cubic B-spline.
	FUNCTION coef(set, p, tap : natural) RETURN integer IS
		TYPE ia IS ARRAY (0 TO 3) OF integer;
		VARIABLE c : ia;
		VARIABLE s : integer;
		CONSTANT K : integer := 256*256*256;
	BEGIN
		CASE set IS
			WHEN 0 =>
				c(0) := rdiv(-p*p*p + 512*p*p - 65536*p, 131072);
				c(2) := rdiv(-3*p*p*p + 1024*p*p + 65536*p, 131072);
				c(3) := rdiv(p*p*p - 256*p*p, 131072);
			WHEN 1 =>
				c := (0, 0, p, 0);
			WHEN 2 =>
				s := 2*p - 128;
				IF s < 0 THEN s := 0; END IF;
				IF s > 256 THEN s := 256; END IF;
				c := (0, 0, s, 0);
			WHEN OTHERS =>
				c(0) := rdiv((256-p)*(256-p)*(256-p), 393216);
				c(2) := rdiv(-3*p*p*p + 768*p*p + 196608*p + K, 393216);
				c(3) := rdiv(p*p*p, 393216);
		END CASE;
		c(1) := 256 - c(0) - c(2) - c(3);
		RETURN c(tap);
	END FUNCTION;

BEGIN
	mode_s <= to_unsigned(MODE, 5);

	-- Clocks
	PROCESS BEGIN
		WHILE NOT done LOOP
			i_clk <= '1'; WAIT FOR (I_CLK_PS / 2) * 1 ps;
			i_clk <= '0'; WAIT FOR (I_CLK_PS - I_CLK_PS / 2) * 1 ps;
		END LOOP; WAIT;
	END PROCESS;
	PROCESS BEGIN
		WHILE NOT done LOOP
			o_clk <= '1'; WAIT FOR (O_CLK_PS / 2) * 1 ps;
			o_clk <= '0'; WAIT FOR (O_CLK_PS - O_CLK_PS / 2) * 1 ps;
		END LOOP; WAIT;
	END PROCESS;
	PROCESS BEGIN
		WHILE NOT done LOOP
			a_clk <= '1'; WAIT FOR (A_CLK_PS / 2) * 1 ps;
			a_clk <= '0'; WAIT FOR (A_CLK_PS - A_CLK_PS / 2) * 1 ps;
		END LOOP; WAIT;
	END PROCESS;

	reset_na <= '0', '1' AFTER 2 us;

	-----------------------------------------------------------------------------
	-- Coefficients and the 8bpp palette, loaded over the poly/pal1 ports
	Loader:PROCESS
		VARIABLE nsets : natural;
		CONSTANT SETMAP : integer_vector(0 TO 3) := (0, 1, 2, 3); -- H:CR V:lin HA:sharp VA:bspline
	BEGIN
		WAIT UNTIL rising_edge(a_clk);
		IF POLY = 0 THEN nsets := 0; ELSIF POLY = 1 THEN nsets := 2; ELSE nsets := 4; END IF;
		FOR s IN 0 TO nsets - 1 LOOP
			FOR ph IN 0 TO 255 LOOP
				FOR tp IN 0 TO 3 LOOP
					poly_a  <= to_unsigned(s, 2) & to_unsigned(ph, 8) & to_unsigned(tp, 2);
					poly_dw <= unsigned(to_signed(coef(SETMAP(s), ph, tp), 10));
					poly_wr <= '1';
					WAIT UNTIL rising_edge(a_clk);
				END LOOP;
			END LOOP;
		END LOOP;
		poly_wr <= '0';
		FOR a IN 0 TO 127 LOOP
			pal1_a  <= to_unsigned(a, 7);
			pal1_dw <= to_unsigned((a*2+1)*5 MOD 256, 8) & to_unsigned(((a*2+1)*17+3) MOD 256, 8) &
			           to_unsigned(255-(a*2+1), 8) &
			           to_unsigned((a*2)*5 MOD 256, 8) & to_unsigned(((a*2)*17+3) MOD 256, 8) &
			           to_unsigned(255-(a*2), 8);
			pal1_wr <= '1';
			WAIT UNTIL rising_edge(a_clk);
		END LOOP;
		pal1_wr <= '0';
		WAIT;
	END PROCESS;

	-----------------------------------------------------------------------------
	-- Core video source
	Source:PROCESS(i_clk)
		VARIABLE hx, vy, fld : natural := 0;
		VARIABLE cediv : natural := 0;
		VARIABLE cw : natural := IN_W;
		VARIABLE ch : natural := IN_H;
		VARIABLE x, y : natural;
		VARIABLE de : boolean;
	BEGIN
		IF rising_edge(i_clk) THEN
			IF cediv = 0 AND reset_na = '1' THEN
				i_ce <= '1';
				de := hx < cw AND vy >= 8 AND vy < 8 + ch;
				i_de <= b2sl(de);
				i_hs <= b2sl(hx >= cw + 8 AND hx < cw + 24);
				i_vs <= b2sl(vy < 3);
				IF IN_INTER THEN i_fl <= b2sl(fld MOD 2 = 1); ELSE i_fl <= '0'; END IF;
				IF de THEN
					x := hx; y := vy - 8;
					i_r <= to_unsigned((x*37 + y*11 + fld*29) MOD 256, 8);
					i_g <= to_unsigned((x*x/4 + y*13 + fld*7) MOD 256, 8);
					i_b <= to_unsigned((x*y + x*3 + fld) MOD 256, 8);
				ELSE
					i_r <= x"00"; i_g <= x"00"; i_b <= x"00";
				END IF;
				hx := hx + 1;
				IF hx = IN_HTOT THEN
					hx := 0;
					vy := vy + 1;
					IF vy = IN_VTOT THEN
						vy := 0;
						fld := fld + 1;
						IF sw_in THEN cw := IN2_W; ch := IN2_H; END IF;
					END IF;
				END IF;
			ELSE
				i_ce <= '0';
			END IF;
			cediv := (cediv + 1) MOD I_CE_DIV;
		END IF;
	END PROCESS;

	-----------------------------------------------------------------------------
	DUT:ENTITY work.ascal
		GENERIC MAP (
			RAMBASE => x"20000000", RAMSIZE => x"00800000",
			PALETTE2 => false, ADAPTIVE => G_ADAPTIVE, DOWNSCALE => G_DOWNSCALE,
			DOWNSCALE_NN => G_DOWNSCALE_NN, FRAC => 8, N_BURST => G_NBURST,
			N_DW => 128, N_AW => 28)
		PORT MAP (
			i_r => i_r, i_g => i_g, i_b => i_b, i_hs => i_hs, i_vs => i_vs, i_fl => i_fl,
			i_de => i_de, i_ce => i_ce, i_clk => i_clk,
			o_r => o_r, o_g => o_g, o_b => o_b, o_hs => o_hs, o_vs => o_vs, o_de => o_de,
			o_vbl => o_vbl, o_brd => o_brd, o_ce => '1', o_clk => o_clk,
			o_fb_ena => b2sl(FB_ENA), o_fb_hsize => FB_W, o_fb_vsize => FB_H,
			o_fb_format => to_unsigned(FB_FMT, 6), o_fb_base => FB_BASE,
			o_fb_stride => to_unsigned(FB_STRIDE, 14),
			pal1_clk => a_clk, pal1_dw => pal1_dw, pal1_a => pal1_a, pal1_wr => pal1_wr,
			o_lltune => o_lltune,
			iauto => '1', run => '1', freeze => '0', mode => mode_s, bob_deint => b2sl(BOB),
			htotal => HTOTAL, hsstart => HSSTART, hsend => HSEND, hdisp => HDISP,
			hmin => hmin_s, hmax => hmax_s,
			vtotal => VTOTAL, vsstart => VSSTART, vsend => VSEND, vdisp => VDISP,
			vmin => VMIN, vmax => VMAX, vrr => '0', vrrmax => 0, swblack => b2sl(SWBLACK),
			format => "01",
			poly_clk => a_clk, poly_dw => poly_dw, poly_a => poly_a, poly_wr => poly_wr,
			avl_clk => a_clk, avl_waitrequest => avl_waitrequest, avl_readdata => avl_readdata,
			avl_readdatavalid => avl_readdatavalid, avl_burstcount => avl_burstcount,
			avl_writedata => avl_writedata, avl_address => avl_address, avl_write => avl_write,
			avl_read => avl_read, avl_byteenable => avl_byteenable,
			reset_na => reset_na);

	-----------------------------------------------------------------------------
	-- Output monitor
	Monitor:PROCESS(o_clk)
		FILE fv : text OPEN write_mode IS OUTDIR & "/vid.txt";
		FILE fp : text OPEN write_mode IS OUTDIR & "/pix.txt";
		FILE fs : text OPEN write_mode IS OUTDIR & "/seed.txt";
		VARIABLE pcp, psd : std_logic := '0';
		VARIABLE cfall, mat : natural := 0;
		VARIABLE mpend : boolean := false;
		VARIABLE l : line;
		VARIABLE cyc, nde, nnz, nlw, ln, fr, hc : natural := 0;
		VARIABLE hv, hl, hf, ha : u32 := FNV0;
		VARIABLE phs, pvs : std_logic := '0';
		VARIABLE hf9d, hf1d : unsigned(11 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF rising_edge(o_clk) THEN
			cyc := cyc + 1;
			hc := hc + 1;
			IF MID_FRAME > 0 AND fr = MID_FRAME AND ln = MID_LINE AND hc = MID_CYC THEN
				hmin_s <= MID_HMIN; hmax_s <= MID_HMAX;
				mat := cyc; mpend := MID_SEED;
				IF MID_SEED AND (p_copy = '1' OR cyc - cfall < 16) THEN
					write(l, string'("MISSED in a copy or under 16 after one")); writeline(fs, l);
				END IF;
			END IF;
			IF MID_FRAME > 0 THEN
				IF p_copy = '0' AND pcp = '1' THEN cfall := cyc; END IF;
				IF p_seed = '1' AND psd = '0' THEN
					write(l, string'("S ")); write(l, fr); write(l, string'(" ")); write(l, ln);
					write(l, string'(" ")); write(l, hc); writeline(fs, l);
					IF mpend THEN
						mpend := false;
						write(l, string'("M ")); write(l, mat - cfall); write(l, string'(" "));
						write(l, cyc - mat); writeline(fs, l);
						IF cyc - mat < 4 OR cyc - mat > 11 THEN
							write(l, string'("MISSED: the seed is not 4 to 11 after the change")); writeline(fs, l);
						END IF;
					END IF;
				END IF;
				pcp := p_copy; psd := p_seed;
			END IF;
			hv := mix(hv, o_r & o_g & o_b & o_hs & o_vs & o_de & o_vbl & o_brd);
			IF o_de = '1' THEN
				nde := nde + 1;
				IF (o_r OR o_g OR o_b) /= x"00" THEN nnz := nnz + 1; END IF;
				IF DUMP THEN
					write(l, string'("P ")); write(l, ln); write(l, string'(" "));
					write(l, to_hstring(o_r & o_g & o_b)); writeline(fp, l);
				END IF;
			END IF;
			IF p_wr /= "0000" THEN
				nlw := nlw + 1;
				hl := mix(hl, to_unsigned(p_wadl, 16) & p_wr);
				hl := mix(hl, p_ldw);
				hf := mix(hf, hf9d);
				IF DUMP THEN
					write(l, string'("W ")); write(l, ln); write(l, string'(" "));
					write(l, p_wadl); write(l, string'(" "));
					write(l, to_hstring(p_wr)); write(l, string'(" "));
					write(l, to_hstring(p_ldw)); write(l, string'(" "));
					write(l, to_hstring(hf9d)); writeline(fp, l);
				END IF;
			END IF;
			IF p_copy = '1' THEN -- the phase of every cycle of a line copy, written or not
				ha := mix(ha, p_hfrac1);
			END IF;
			IF reset_na = '1' THEN
				IF DUMP AND p_hfrac1 /= hf1d THEN
					write(l, string'("A ")); write(l, cyc); write(l, string'(" "));
					write(l, to_hstring(p_hfrac1)); writeline(fp, l);
				END IF;
			END IF;
			hf1d := p_hfrac1;
			hf9d := p_hfrac9;

			IF o_hs = '1' AND phs = '0' THEN
				write(l, string'("L ")); write(l, fr); write(l, string'(" "));
				write(l, ln); write(l, string'(" ")); write(l, cyc); write(l, string'(" "));
				write(l, nde); write(l, string'(" ")); write(l, nnz);
				write(l, string'(" ")); write(l, to_hstring(hv));
				write(l, string'(" ")); write(l, nlw); write(l, string'(" "));
				write(l, to_hstring(hl)); write(l, string'(" ")); write(l, to_hstring(hf));
				write(l, string'(" ")); write(l, to_hstring(ha));
				writeline(fv, l);
				hv := FNV0; hl := FNV0; hf := FNV0; ha := FNV0; nde := 0; nnz := 0; nlw := 0;
				ln := ln + 1; hc := 0;
			END IF;
			IF o_vs = '1' AND pvs = '0' THEN
				write(l, string'("F ")); write(l, fr); write(l, string'(" "));
				write(l, cyc); write(l, string'(" ")); write(l, ln);
				writeline(fv, l);
				fr := fr + 1; ln := 0;
				IF SW_FRAME > 0 AND fr = SW_FRAME THEN
					hmin_s <= SW_HMIN; hmax_s <= SW_HMAX; sw_in <= true;
				END IF;
				IF fr = NFRAMES + 1 THEN
					done <= true;
				END IF;
			END IF;
			phs := o_hs; pvs := o_vs;
		END IF;
	END PROCESS;

	-----------------------------------------------------------------------------
	-- Avalon memory: deterministic waitrequest and read-data gaps from an LFSR,
	-- fixed read latency, reads served in order.
	Memory:PROCESS(a_clk)
		TYPE word_t IS ARRAY (0 TO 3) OF integer;
		TYPE mem_t IS ARRAY (0 TO 2**MEM_AW-1) OF word_t;
		TYPE mem_p IS ACCESS mem_t;
		VARIABLE mem : mem_p := NULL;
		TYPE rq_t IS RECORD adr, bc, due : natural; END RECORD;
		TYPE rq_a IS ARRAY (0 TO 15) OF rq_t;
		VARIABLE rq : rq_a;
		VARIABLE rq_n, rbeat : natural := 0;
		VARIABLE wbeat, wadr, wbc, wst : natural := 0;
		VARIABLE wh : u32;
		VARIABLE lfsr : unsigned(15 DOWNTO 0) := x"ACE1";
		VARIABLE acyc : natural := 0;
		FILE fa : text OPEN write_mode IS OUTDIR & "/avl.txt";
		VARIABLE l : line;
		VARIABLE a, fw, nfw : natural;
		VARIABLE d : std_logic_vector(127 DOWNTO 0);
		VARIABLE hw : u32;
		VARIABLE fbb : unsigned(31 DOWNTO 0);
	BEGIN
		IF rising_edge(a_clk) THEN
			IF mem = NULL THEN
				mem := NEW mem_t;
				FOR n IN mem'RANGE LOOP
					mem(n) := (OTHERS => 0);
				END LOOP;
				IF FB_ENA THEN -- framebuffer contents: a hash of the word index
					fbb := FB_BASE; -- via a variable: GHDL 5.0.1/LLVM miscompiles to_integer of a constant's slice
					fw := to_integer(fbb(MEM_AW+3 DOWNTO 4));
					nfw := 2**(MEM_AW-2); -- 8 MB
					FOR n IN 0 TO nfw - 1 LOOP
						FOR k IN 0 TO 3 LOOP
							hw := mix(mix(FNV0, to_unsigned(n, 32)), to_unsigned(k, 8));
							mem((fw + n) MOD 2**MEM_AW)(k) := to_integer(signed(hw));
						END LOOP;
					END LOOP;
				END IF;
			END IF;
			acyc := acyc + 1;

			-- Commands accepted at this edge
			IF avl_read = '1' AND avl_waitrequest = '0' THEN
				a := to_integer(unsigned(avl_address(MEM_AW-1 DOWNTO 0)));
				rq(rq_n) := (adr => a, bc => to_integer(unsigned(avl_burstcount)), due => acyc + 12);
				rq_n := rq_n + 1;
				write(l, string'("R ")); write(l, acyc); write(l, string'(" "));
				write(l, to_hstring(unsigned(avl_address))); write(l, string'(" "));
				write(l, to_integer(unsigned(avl_burstcount)));
				writeline(fa, l);
			END IF;
			IF avl_write = '1' AND avl_waitrequest = '0' THEN
				IF wbeat = 0 THEN
					wadr := to_integer(unsigned(avl_address(MEM_AW-1 DOWNTO 0)));
					wbc := to_integer(unsigned(avl_burstcount));
					wst := acyc; wh := FNV0;
				END IF;
				FOR k IN 0 TO 3 LOOP
					mem((wadr + wbeat) MOD 2**MEM_AW)(k) := to_integer(signed(avl_writedata(32*k+31 DOWNTO 32*k)));
					wh := mix(wh, unsigned(avl_writedata(32*k+31 DOWNTO 32*k)));
				END LOOP;
				wbeat := wbeat + 1;
				IF wbeat = wbc THEN
					write(l, string'("W ")); write(l, wst); write(l, string'(" "));
					write(l, to_hstring(to_unsigned(wadr, 28))); write(l, string'(" "));
					write(l, wbc); write(l, string'(" ")); write(l, to_hstring(wh));
					writeline(fa, l);
					wbeat := 0;
				END IF;
			END IF;

			-- Back-pressure and read data for the next cycle
			lfsr := lfsr(14 DOWNTO 0) & (lfsr(15) XOR lfsr(13) XOR lfsr(12) XOR lfsr(10));
			avl_waitrequest <= b2sl(lfsr(3 DOWNTO 0) < 3 OR reset_na = '0');
			avl_readdatavalid <= '0';
			IF rq_n > 0 AND acyc >= rq(0).due AND lfsr(7 DOWNTO 5) /= "000" THEN
				a := (rq(0).adr + rbeat) MOD 2**MEM_AW;
				FOR k IN 0 TO 3 LOOP
					d(32*k+31 DOWNTO 32*k) := std_logic_vector(to_signed(mem(a)(k), 32));
				END LOOP;
				avl_readdata <= d;
				avl_readdatavalid <= '1';
				rbeat := rbeat + 1;
				IF rbeat = rq(0).bc THEN
					rbeat := 0;
					rq(0 TO 14) := rq(1 TO 15);
					rq_n := rq_n - 1;
				END IF;
			END IF;
		END IF;
	END PROCESS;

END ARCHITECTURE sim;
