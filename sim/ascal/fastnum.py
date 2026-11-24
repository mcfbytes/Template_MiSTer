#!/usr/bin/env python3
"""Write a simulation-only copy of GHDL's VHDL-2008 numeric_std whose "+", "-", "*" and
TO_INTEGER take an integer fast path when every operand bit is '0'/'1'/'L'/'H' and the
result fits in 30 bits. Any metavalue, or a wider operand, falls through to the reference
code, so results are identical; only simulation speed changes (GHDL's numeric_std works
bit by bit and allocates a vector per call, which dominates the bench's run time).

usage: fastnum.py <ghdl ieee2008 source dir> <out dir>
"""
import os
import re
import sys

src, out = sys.argv[1], sys.argv[2]
body = open(os.path.join(src, "numeric_std-body.vhdl"), encoding="latin-1").read()

HELPERS = """
  -- fastnum.py: allocation-free conversions; -1 = not applicable (metavalue or too wide)
  function FAST_NAT (ARG : UNRESOLVED_UNSIGNED) return INTEGER is
    variable V : INTEGER := 0;
  begin
    if ARG'length > 30 then return -1; end if;
    for I in ARG'range loop
      case ARG(I) is
        when '0' | 'L' => V := V + V;
        when '1' | 'H' => V := V + V + 1;
        when others    => return -1;
      end case;
    end loop;
    return V;
  end function FAST_NAT;

  function FAST_SNAT (ARG : UNRESOLVED_SIGNED) return INTEGER is
    variable V : INTEGER := 0;
  begin
    if ARG'length > 30 then return -1; end if;
    for I in ARG'range loop
      case ARG(I) is
        when '0' | 'L' => V := V + V;
        when '1' | 'H' => V := V + V + 1;
        when others    => return -1;
      end case;
    end loop;
    return V;
  end function FAST_SNAT;

  function FAST_SIGNED (V, SIZE : INTEGER) return UNRESOLVED_SIGNED is
    variable W : INTEGER := V mod 2**SIZE;
  begin
    if W >= 2**(SIZE-1) then W := W - 2**SIZE; end if;
    return TO_SIGNED(W, SIZE);
  end function FAST_SIGNED;
"""


def patch_fn(sig, decls, fast):
    """Add declarations before the function's 'begin' and a fast path right after it."""
    global body
    i = body.index(sig)
    j = body.index("\n  begin\n", i)
    body = body[:j] + "\n" + decls + body[j:]
    j = body.index("\n  begin\n", i) + len("\n  begin\n")
    body = body[:j] + fast + body[j:]


m = re.search(r"package body NUMERIC_STD is\n", body, re.I)
body = body[:m.end()] + HELPERS + body[m.end():]

U = "UNRESOLVED_UNSIGNED"
S = "UNRESOLVED_SIGNED"
DU = "    variable FL, FR : INTEGER;"
for op in ("+", "-"):
    e = "FL %s FR" % op
    patch_fn('function "%s" (L, R : %s) return %s is' % (op, U, U), DU,
             "    if L'length >= 1 and R'length >= 1 and SIZE <= 30 then\n"
             "      FL := FAST_NAT(L); FR := FAST_NAT(R);\n"
             "      if FL >= 0 and FR >= 0 then\n"
             "        return TO_UNSIGNED((%s) mod 2**SIZE, SIZE);\n"
             "      end if;\n    end if;\n" % e)
    patch_fn('function "%s" (L, R : %s) return %s is' % (op, S, S), DU,
             "    if L'length >= 1 and R'length >= 1 and SIZE <= 29 then\n"
             "      FL := FAST_SNAT(RESIZE(L, SIZE)); FR := FAST_SNAT(RESIZE(R, SIZE));\n"
             "      if FL >= 0 and FR >= 0 then\n"
             "        if FL >= 2**(SIZE-1) then FL := FL - 2**SIZE; end if;\n"
             "        if FR >= 2**(SIZE-1) then FR := FR - 2**SIZE; end if;\n"
             "        return FAST_SIGNED(%s, SIZE);\n"
             "      end if;\n    end if;\n" % e)
patch_fn('function "*" (L, R : %s) return %s is' % (U, U), DU,
         "    if L'length >= 1 and R'length >= 1 and L'length + R'length <= 30 then\n"
         "      FL := FAST_NAT(L); FR := FAST_NAT(R);\n"
         "      if FL >= 0 and FR >= 0 then\n"
         "        return TO_UNSIGNED(FL * FR, L'length + R'length);\n"
         "      end if;\n    end if;\n")
patch_fn('function "*" (L, R : %s) return %s is' % (S, S), DU,
         "    if L'length >= 1 and R'length >= 1 and L'length + R'length <= 30 then\n"
         "      FL := FAST_SNAT(L); FR := FAST_SNAT(R);\n"
         "      if FL >= 0 and FR >= 0 then\n"
         "        if FL >= 2**(L'length-1) then FL := FL - 2**L'length; end if;\n"
         "        if FR >= 2**(R'length-1) then FR := FR - 2**R'length; end if;\n"
         "        return TO_SIGNED(FL * FR, L'length + R'length);\n"
         "      end if;\n    end if;\n")
patch_fn("function TO_INTEGER (ARG : %s) return NATURAL is" % U, DU,
         "    FL := FAST_NAT(ARG);\n    if FL >= 0 then return FL; end if;\n")
patch_fn("function TO_INTEGER (ARG : %s) return INTEGER is" % S, DU,
         "    FL := FAST_SNAT(ARG);\n"
         "    if FL >= 0 and ARG'length >= 1 then\n"
         "      if FL >= 2**(ARG'length-1) then return FL - 2**ARG'length; end if;\n"
         "      return FL;\n    end if;\n")

os.makedirs(out, exist_ok=True)
open(os.path.join(out, "numeric_std-body.vhdl"), "w", encoding="latin-1").write(body)
for f in ("std_logic_1164.vhdl", "std_logic_1164-body.vhdl", "numeric_std.vhdl"):
    with open(os.path.join(src, f), encoding="latin-1") as fi, \
         open(os.path.join(out, f), "w", encoding="latin-1") as fo:
        fo.write(fi.read())
