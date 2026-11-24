# Simulation portability, applied identically to every ascal under test. Each rewrite makes
# an out-of-range integer result wrap the way the synthesized register does (modulo 2^bits),
# because GHDL stops on a range violation that the hardware simply truncates.
s/^\(\t*\)o_pshift<=o_off(0) -1;/\1o_pshift<=(o_off(0) + 15) MOD 16;/
s/^\(\t*\)o_ihsizem<=o_ihsize + o_off(0) - 2;/\1o_ihsizem<=(o_ihsize + o_off(0) + 4094) MOD 4096;/
