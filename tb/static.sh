#!/bin/sh
# Static checks on the preprocessed top level, arm N of tb/run.sh, and the net lists arm E checks.
#   tb/static.sh            the checks below, for every configuration of --configs; "-- " headings, a FAIL line
#                           per failed check, then PASSED/FAILED with the counts (tb/run.sh's verdict)
#   tb/static.sh --configs  one "name|macros" line per configuration arm E elaborates: off and each of the 32 LW
#                           tier sets of {PAD, EVT, SECTOR, LANE, OSD}, each with no upstream macro, with
#                           MISTER_DEBUG_NOHDMI and with MISTER_DUAL_SDRAM (99 configurations)
#   tb/static.sh --nets FILE MACRO...
#                           FILE gets one Verilog check call per LW net of that configuration, for tb/top_tb.sv
# An LW net is a net sys_top.v declares in this configuration whose declaration the off configuration of the
# same upstream variant does not have word for word (the LW wires, and every stock net the LW text re-declares,
# such as the link nets the lane takes over), and every port of osd, hps_io and emu that the configuration
# declares differently from off, at each of their instances. Arm E fails if one of them reads z.
# The checks, each in every configuration, on sys_top.v as Icarus preprocesses it:
#   1. The LW terms of GPI and the two LW instances come together or not at all: gp_in[25] is the net lw_present,
#      gp_in[24] the net lw_idle and gp_in[22] the net lw_held in exactly the configurations that instantiate both
#      hps_lw_bridge and lw_sys_window, and those are exactly the ones with MISTER_LW; each net is declared with no
#      value, assigned nowhere, and connected to its instance's output of its name (the firewall's present and
#      idle, the window's held). Every other bit of gp_in[27:21] is a constant 0 (all of them without MISTER_LW),
#      and gp_in is 32 bits wide. Once, in hps_lw_bridge.sv: present is assigned the constant 1 and nothing else,
#      and idle is lw_axi_regbus's idle_q, a register assigned only on posedge clk. Once, in lw_sys_window.sv
#      (check 8): held is en_q alone, a register assigned only on posedge clk.
#   2. f2h_irq: bits 1:0 are upstream's; bits 10:8 are the 3-bit net lw_sys_window's lines output drives, line
#      ordinal k at bit 7+k, declared with no value and assigned nowhere else; every other bit is a constant 0.
#   3. Every LW configuration has LW nets to check; the off configuration has none.
# Then, once, the crossing structure, on every LW-only file and the LW text of the stock files, with every tier:
#   4. The cell registers (lws0_, lws1_, lwa0_, lwa1_, lwq_, lwsrc_, lwtg_) are named only in sys/lw_cdc.sv, and
#      the synchroniser attribute is on its two-flop stages alone (SYNC_ALLOW lists no exception).
#      lw_qstat is built only inside lw_xfer, whose capture enable comes from no lwsrc_ register and whose D is
#      its hold register; an lw_arst input is hps_rst_n or gp_out[31].
#   5. The D of every lw_sync is a plain reference (names, selects, a concatenation of them) to registers of the
#      same module, clocked there and not on the destination clock, named *_lwx (GRAY 0) or *_lwgray (GRAY 1),
#      each declared with a preserve attribute (in lw_cdc.sv, also a cell's own toggles, and lw_arst's chain,
#      whose rise is asynchronous, into its own clock); every register so named feeds one. An lw_xfer's s_go and s_d are on its s_clk.
#      An lw_fence's hen, take, drain and fin are on its clk and its rest on its clk_sys.
#   6. Every *_lwgray register is one of the Gray buses, the list sys/lw_bridge.sdc's set_max_skew loop walks, and
#      is assigned once, unconditionally, as g <= b ^ (b >> 1); every bus on that list has its *_lwgray register.
#   7. No clocked block outside the cells reads, directly or through nets, a register or an input on another
#      clock (input and inout clocks from PORTDOM) other than through a cell's output, beside the listed reads
#      (XREAD_ALLOW) and the presence bits' nets (PRESENCE, each a constant select of its port), each of which must
#      still exist; and no register is assigned on two clocks.
# Then, once, the directory's lines, in sys/lw_sys_window.sv with every tier:
#   8. Every entry is one ent() call whose kind and LINE word are the localparams <B>_KIND and <B>_LINE of one
#      block B, each localparam in one call. The LINE words name each non-zero ordinal once, a framework kind's
#      ordinal (kind below 0x8000) in 1-15 and a private kind's in 16-31. line_d is one OR of terms
#      {3{<b>_lvl & line_en[e_<b>]}} & line_bit(<B>_LINE), one per non-zero ordinal, and line_bit(k) is bit k-1;
#      PENDING's terms put the same <b>_lvl on its entry's bit e_<b>. The lines and the host enable are registers:
#      lines is line_q alone, line_q is assigned once, on posedge clk, as {3{en_q}} & line_d, and held is
#      en_q alone, a register assigned only on posedge clk.
# Then, once, the wording:
#   9. No line of an LW-only file, of a file under tb/, of the LW text of a stock file (a branch that a MISTER_LW*
#      or LW_* macro opens) or of docs/LW_BRIDGE.md names a device of the reference core the framework was cut from
#      (the words are listed in the check, in pieces): the framework's names are its own ("slot", "save slot",
#      "image"). And the guide, docs/LW_BRIDGE.md, names no register, verb or field the host interface no longer
#      has (a lease and its TTL, timebase, renew, release, claim, owner and epoch), no planning label (a finding,
#      rule or step id, a section sign) and no IPv4 address. Its drain rules end on a level a drain brings to 0:
#      the lane's on RSTAT's replies field, never on RSTAT whole (its reply_drops field is never cleared). Its
#      enable-fall contract promises no queued frame, committed or not: only a frame already playing completes.
# Then, once, the constraints, sys/lw_bridge.sdc against the LW modules with every tier and LW_DIAG=1:
#  10. Every set_net_delay and set_max_skew carries -from and -to, each a collection the file builds; every
#      get_keepers name pattern (the Gray loop's expanded per bus) matches a register of an LW module, as a
#      keeper name <hierarchy>|<register>; the collection lw_x is exactly the registers that feed an lw_sync, and
#      lw_s0, lw_a, lw_q and lw_src exactly the cells' first stages, asynchronous chain, bundle captures and
#      hold registers. It prints the source bits into first stages and the bundle bits, per block at its
#      default parameters (one instance each, every generate branch), for the post-fit pair count to compare.
# Needs Icarus Verilog (for its preprocessor) and python3.
cd "$(dirname "$0")/.." || exit 2
O=${LW_TB_OUT:-tb/out}/static; mkdir -p "$O" || exit 2
command -v iverilog >/dev/null 2>&1 || { echo "FAILED: no iverilog on PATH"; exit 1; }
exec python3 -I - "$O" "$@" <<'EOF'
import fnmatch, os, re, subprocess, sys
O, args = sys.argv[1], sys.argv[2:]

# off, then each of the 32 tier sets (the floor MISTER_LW alone first, every tier last)
TIER_MACROS = [('pad', 'MISTER_LW_PAD'), ('evt', 'MISTER_LW_EVT'), ('sec', 'MISTER_LW_SECTOR_SLOT=1'),
               ('lane', 'MISTER_LW_LANE'), ('osd', 'MISTER_LW_OSD')]
TIERS = [('off', [])] + [('-'.join(['lw'] + [n for k, (n, _) in enumerate(TIER_MACROS) if m >> k & 1]),
                          ['MISTER_LW'] + [d for k, (_, d) in enumerate(TIER_MACROS) if m >> k & 1])
                         for m in range(32)]
VARIANTS = [('', []), ('.nohdmi', ['MISTER_DEBUG_NOHDMI']), ('.dual', ['MISTER_DUAL_SDRAM'])]
UPSTREAM = {'MISTER_DEBUG_NOHDMI', 'MISTER_DUAL_SDRAM'}
CONFIGS = [(t + v, td + vd) for v, vd in VARIANTS for t, td in TIERS]
# f2h_irq bits -> the lw_sys_window output that must drive them, its bit k-1 being line ordinal k at bit 7+k
LINES_LO, LINES_W, LINES_PORT = 8, 3, 'lines'
# gp_in bit -> (the net, the instance and its output that must drive it); the rest of gp_in[27:21] is 0
GPI = {25: ('lw_present', 'hps_lw_bridge', 'present'), 24: ('lw_idle', 'hps_lw_bridge', 'idle'),
       22: ('lw_held', 'lw_sys_window', 'held')}

if args[:1] == ['--configs']:
    for n, d in CONFIGS:
        print('%s|%s' % (n, ' '.join('-D' + x for x in d)))
    sys.exit(0)

_b = os.path.join(O, 'build_id.%d.v' % os.getpid())
open(_b, 'w').write('`define BUILD_DATE "000000"\n')
os.replace(_b, os.path.join(O, 'build_id.v'))   # whole, for the runs beside this one

def pp(path, defs):
    out = os.path.join(O, 'pp.%d.v' % os.getpid())   # one file per process: arm E runs several at once
    r = subprocess.run(['iverilog', '-E', '-I', 'sys', '-I', O, '-o', out] + ['-D' + d for d in defs] + [path],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if r.returncode:
        sys.exit('tb/static.sh: cannot preprocess %s with %s: %s' % (path, ' '.join(defs), r.stdout.strip()))
    t = open(out, errors='replace').read()
    os.remove(out)
    return t

TOK = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|[A-Za-z_$][\w$]*|\d+\s*\'\s*[sS]?[bodhBODH]\s*[\w?]+|'
                 r'\d[\w\'.]*|\'[sS]?[bodhBODH]\w*|\S', re.S)
def tokens(text):
    return [t for t in TOK.findall(text) if not t.startswith('//') and not t.startswith('/*')]

def module(toks, name):
    for i in range(len(toks) - 1):
        if toks[i] == 'module' and toks[i + 1] == name:
            return toks[i:toks.index('endmodule', i)]
    sys.exit('tb/static.sh: no module %s' % name)

OPEN = {'begin', 'fork', 'case', 'casez', 'casex', 'function', 'task', 'generate'}
CLOSE = {'end', 'join', 'endcase', 'endfunction', 'endtask', 'endgenerate'}
NETKW = {'wire', 'reg', 'logic', 'tri', 'tri0', 'tri1', 'wand', 'wor'}

def statements(body):
    """Module-level statements (outside begin/end, functions and the like), as token lists."""
    out, cur, depth, par = [], [], 0, 0
    for t in body:
        if t in OPEN:
            depth += 1; cur = []; continue
        if t in CLOSE:
            depth -= 1; cur = []; continue
        if depth:
            continue
        par += t in '([{'
        par -= t in ')]}'
        if t == ';' and par == 0:
            out.append(cur); cur = []
        else:
            cur.append(t)
    return out

def split_top(toks, sep=','):
    parts, cur, par = [], [], 0
    for t in toks:
        if t in '([{': par += 1
        if t in ')]}': par -= 1
        if t == sep and par == 0:
            parts.append(cur); cur = []
        else:
            cur.append(t)
    parts.append(cur)
    return parts

def decls(body):
    """name -> (declaration text, range tokens, initialiser tokens or None) for every module-level net."""
    r = {}
    for s in statements(body):
        if not s or s[0] not in NETKW:
            continue
        i = 1
        while i < len(s) and s[i] in {'signed', 'unsigned'} | NETKW:
            i += 1
        rng = []
        if i < len(s) and s[i] == '[':
            j = s.index(']', i); rng = s[i:j + 1]; i = j + 1
        for part in split_top(s[i:]):
            if not part:
                continue
            name = part[0]
            init = part[part.index('=') + 1:] if '=' in part else None
            r[name] = (' '.join([s[0]] + rng + part), rng, init)
    return r

def ports(body):
    """name -> declaration text for an ANSI port list."""
    i = body.index('(')
    if body[i - 1] == '#':
        d = 0
        for j in range(i, len(body)):
            d += body[j] == '('; d -= body[j] == ')'
            if d == 0:
                break
        i = body.index('(', j + 1)
    d, j = 0, i
    for j in range(i, len(body)):
        d += body[j] == '('; d -= body[j] == ')'
        if d == 0:
            break
    r, last = {}, []
    for part in split_top(body[i + 1:j]):
        if not part:
            continue
        if part[0] in ('input', 'output', 'inout'):
            last = [x for x in part[:-1]]
        r[part[-1]] = ' '.join(last + [part[-1]])
    return r

def instances(body, mod):
    """[(instance name, {port: connection tokens})] for every instance of mod."""
    r = []
    for i in range(len(body) - 2):
        if body[i] != mod or (i and body[i - 1] in ('module', '.')):
            continue
        j = i + 1
        if body[j] == '#':
            d = 0
            for j in range(j + 1, len(body)):
                d += body[j] == '('; d -= body[j] == ')'
                if d == 0:
                    break
            j += 1
        if not re.match(r'[A-Za-z_]\w*$', body[j]) or body[j + 1] != '(':
            continue
        name, k, d = body[j], j + 1, 0
        for e in range(k, len(body)):
            d += body[e] == '('; d -= body[e] == ')'
            if d == 0:
                break
        conn = {}
        for part in split_top(body[k + 1:e]):
            if len(part) >= 3 and part[0] == '.':
                conn[part[1]] = part[3:-1]
        r.append((name, conn))
    return r

def lit_width(t):
    m = re.match(r"(\d+)\s*'", t)
    return int(m.group(1)) if m else None

def lit_value(t):
    m = re.match(r"(\d+)\s*'\s*[sS]?([bodhBODH])\s*([\w?]+)$", t)
    if not m:
        return None
    try:
        return int(m.group(3).replace('_', ''), {'b': 2, 'o': 8, 'd': 10, 'h': 16}[m.group(2).lower()])
    except ValueError:
        return None

def rng_width(rng):
    try:
        a, b = ''.join(rng[1:-1]).split(':')
        return abs(int(a) - int(b)) + 1
    except ValueError:
        return None

def width(e, W):
    """Self-determined width of a simple expression (names, selects, sized literals, bitwise and ?:), or None."""
    for op in ('?', '|', '^', '&'):
        parts = split_top(e, op)
        if len(parts) > 1:
            if op == '?':
                alts = split_top(parts[1], ':') if len(parts) == 2 else []
                ws = [width(a, W) for a in alts]
            else:
                ws = [width(p, W) for p in parts]
            return None if not ws or None in ws else max(ws)
    if e[:1] == ['~']:
        return width(e[1:], W)
    if e[:1] == ['('] and e[-1:] == [')']:
        return width(e[1:-1], W)
    if len(e) == 1 and lit_width(e[0]):
        return lit_width(e[0])
    if e and re.match(r'[A-Za-z_]\w*$', e[0]):
        if len(e) == 1:
            return W.get(e[0])
        if e[1] == '[' and e[-1] == ']':
            s = e[2:-1]
            if ':' in s:
                return rng_width(['['] + s + [']'])
            return 1
    return None

def bitmap(init, W):
    """LSB-first list of (term tokens, bit within term) for a concatenation, or None if a term has no width."""
    if not init or init[0] != '{' or init[-1] != '}':
        return None
    bits = []
    for term in reversed(split_top(init[1:-1])):
        w = width(term, W)
        if w is None:
            return None
        bits += [(term, k) for k in range(w)]
    return bits

checks = fails = 0
def check(ok, what):
    global checks, fails
    checks += 1
    if not ok:
        fails += 1
        print('FAIL ' + what)
    return ok

def analyse(defs):
    lw = 'MISTER_LW' in defs
    off = [d for d in defs if d in UPSTREAM]
    top = module(tokens(pp('sys/sys_top.v', defs)), 'sys_top')
    top0 = module(tokens(pp('sys/sys_top.v', off)), 'sys_top')
    D, D0 = decls(top), decls(top0)
    nets = [n for n in D if D0.get(n, (None,))[0] != D[n][0]]
    lwports = []
    for mod, src, path in (('osd', 'sys/osd.v', None), ('emu', 'Template.sv', None),
                           ('hps_io', 'sys/hps_io.sv', 'emu')):
        P = ports(module(tokens(pp(src, defs)), mod))
        P0 = ports(module(tokens(pp(src, off)), mod))
        changed = [p for p in P if P0.get(p) != P[p]]
        if not changed:
            continue
        host = top if path is None else module(tokens(pp('Template.sv', defs)), 'emu')
        for inst, _ in instances(host, mod):
            lwports += [('.'.join(x for x in (path, inst, p) if x)) for p in changed]
    return lw, top, D, nets, lwports

def write_nets(path, nets, lwports):
    with open(path, 'w') as f:
        f.write('// generated by tb/static.sh --nets: the LW nets and ports of this configuration\n')
        for n in nets:
            f.write('lw_net("%s", zc(dut.%s), $bits(dut.%s));\n' % (n, n, n))
        for p in lwports:
            f.write('lw_port("%s", zc(dut.%s), $bits(dut.%s));\n' % (p, p, p))

if args[:1] == ['--nets']:
    defs = [a[2:] for a in args[2:] if a.startswith('-D')]
    _, _, _, nets, lwports = analyse(defs)
    write_nets(args[1], nets, lwports)
    sys.exit(0)

for name, defs in CONFIGS:
    lw, top, D, nets, lwports = analyse(defs)
    print('-- %s: the GPI terms, the lines and the LW nets (%s)' % (name, ' '.join(defs) or 'no macro'))
    W = {}
    for n, (_, rng, _) in D.items():
        W[n] = rng_width(rng) if rng else 1
    for p, txt in ports(top).items():
        t = txt.split()
        W[p] = rng_width(t[t.index('['):t.index(']') + 1]) if '[' in t else 1
    inst = {m: instances(top, m) for m in ('hps_lw_bridge', 'lw_sys_window')}
    fw, win = len(inst['hps_lw_bridge']) == 1, len(inst['lw_sys_window']) == 1
    gp = bitmap(D.get('gp_in', (0, 0, None))[2], W)
    if check(gp is not None and len(gp) == 32, '%s: gp_in is not a 32-bit concatenation of sized terms' % name):
        terms = []
        for b, (net, mod, port) in sorted(GPI.items(), reverse=True):
            t = gp[b][0]
            terms.append(t == [net])
            conn = inst[mod][0][1] if len(inst[mod]) == 1 else {}
            if t == [net]:
                ok = W.get(net) == 1 and D.get(net, (0, 0, 0))[2] is None and conn.get(port) == [net] and \
                     not any(s[:1] == ['assign'] and net in s[1:s.index('=')] for s in statements(top) if '=' in s)
                check(ok, '%s: gp_in[%d] is %s, not a 1-bit net driven by %s.%s alone'
                      % (name, b, net, mod, port))
            else:
                check(len(t) == 1 and lit_value(t[0]) == 0, '%s: gp_in[%d] is %s, neither %s nor 0'
                      % (name, b, ' '.join(t), net))
        both = all(terms)
        check(both == any(terms) and both == fw == win == lw,
              "%s: gp_in[25] lw_present %s, gp_in[24] lw_idle %s, gp_in[22] lw_held %s, hps_lw_bridge %s, "
              "lw_sys_window %s, MISTER_LW %s" % (name, terms[0], terms[1], terms[2], fw, win, lw))
        zero = all(lit_value(gp[b][0][0]) == 0 and len(gp[b][0]) == 1 for b in range(21, 28) if b not in GPI)
        check(zero, '%s: gp_in[27:26], gp_in[23] or gp_in[21] is not constant 0' % name)
    irq = bitmap(D.get('f2h_irq', (0, 0, None))[2], W)
    if check(irq is not None and len(irq) <= 64, '%s: f2h_irq is not a concatenation of sized terms' % name):
        irq += [(["1'b0"], 0)] * (64 - len(irq))
        check([' '.join(t) for t, _ in irq[:2]] == ['HDMI_TX_VS', 'video_sync'],
              '%s: f2h_irq[1:0] is not upstream\'s' % name)
        conn = inst['lw_sys_window'][0][1] if win else {}
        for b in range(2, 64):
            t, k = irq[b]
            if LINES_LO <= b < LINES_LO + LINES_W and lw:
                ok = len(t) == 1 and k == b - LINES_LO and conn.get(LINES_PORT) == t and W.get(t[0]) == LINES_W and \
                     D.get(t[0], (0, 0, 0))[2] is None and not any(s[:2] == ['assign', t[0]] for s in statements(top))
                check(ok, '%s: f2h_irq[%d] is bit %d of %s, not bit %d of a %d-bit net driven by lw_sys_window.%s alone'
                      % (name, b, k, ' '.join(t), b - LINES_LO, LINES_W, LINES_PORT))
            else:
                check(len(t) == 1 and lit_value(t[0]) == 0, '%s: f2h_irq[%d] is %s, not 0' % (name, b, ' '.join(t)))
    check(bool(nets) == lw, '%s: %d LW nets for arm E' % (name, len(nets)))
    print('%s: %d LW nets (%s), %d LW ports (%s)' % (name, len(nets), ' '.join(nets), len(lwports), ' '.join(lwports)))
    write_nets(os.path.join(O, name + '.nets.vh'), nets, lwports)

# ---- The crossing structure: every crossing register in sys/lw_cdc.sv's cells, every source named ----
LWALL = ['MISTER_LW', 'MISTER_LW_PAD', 'MISTER_LW_EVT', 'MISTER_LW_SECTOR_SLOT=1', 'MISTER_LW_LANE', 'MISTER_LW_OSD',
         'LW_DIAG=1']
CDC = 'sys/lw_cdc.sv'
CELLREG = ('lws0_', 'lws1_', 'lwa0_', 'lwa1_', 'lwq_', 'lwsrc_', 'lwtg_')
CELLSYNC = {'lws0_q', 'lws1_q', 'lwa0_q', 'lwa1_q'}
SYNATTR = 'SYNCHRONIZER_IDENTIFICATION'
# The stock files' LW text holds no synchroniser: the list is empty
SYNC_ALLOW = {}
SDC = 'sys/lw_bridge.sdc'
SDCTXT = open(SDC).read() if os.path.exists(SDC) else ''
# the Gray buses: the list the SDC's set_max_skew loop walks
_gl = re.findall(r'^\s*foreach g \[list ([\w ]+)\] \{$', SDCTXT, re.M)
GRAYBUS = _gl[0].split() if len(_gl) == 1 else []
# Reads across clocks outside the cells, as (module, register, source); each must still be there. None is left.
XREAD_ALLOW = set()
# The input and inout ports on a block's second clock. Every other one is on the block's port clk, or on the one clock
# of a block that has no clk port.
PORTDOM = {'lw_sector': ('clk_sys', 'c_reset c_req c_lba hps_ack c_commit c_wide x_rd x_wr x_lba x_hps_ack'),
           'lw_evt': ('clk_sys', 'st ev lvl'),
           'lw_lane': ('clk_sys', 'l_busy l_wait l_dout'),
           'lw_osd': ('clk_sys', 'sel view'),
           'lw_sys_window': ('clk_sys', 'lane_busy lane_wait lane_dout osd_sel osd_view hps_lw io_wide')}
# The presence bits, read on the window's clk as constants: each core-side piece ties its bit to 1, and an absent
# piece leaves it to the pull-down. Only these nets, each assigned exactly this select, are exempt; every other bit
# of hps_lw is on clk_sys and crosses through a cell.
PRESENCE = {'lw_sys_window': {'pres_pad': 'hps_lw [ 375 ]', 'pres_evt': 'hps_lw [ 376 ]',
                              'pres_sec': 'hps_lw [ 379 : 377 ]', 'pres_cmt': 'hps_lw [ 381 ]'}}

TOK2 = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|[A-Za-z_$][\w$]*|\d+\s*\'\s*[sS]?[bodhBODH]\s*[\w?]+|'
                  r'\d[\w\'.]*|\'[sS]?[bodhBODH]\w*|===|!==|<<<|>>>|<=|>=|==|!=|<<|>>|&&|\|\||\S', re.S)
IDENT = re.compile(r'[A-Za-z_]\w*$')
KW = set('module endmodule input output inout wire reg logic integer genvar parameter localparam assign always '
         'begin end if else case casez casex endcase default for while repeat forever posedge negedge or initial '
         'function endfunction task endtask generate endgenerate automatic signed unsigned string real'.split())
OPENER = {'(': ')', '[': ']', '{': '}'}

def toks2(text):
    return [t for t in TOK2.findall(text) if not t.startswith('//') and not t.startswith('/*')]

def idents(ts):
    r, i = [], 0
    while i < len(ts):
        t = ts[i]
        if t == '.' and i + 1 < len(ts):
            i += 2; continue
        if IDENT.match(t) and t not in KW:
            r.append(t)
        i += 1
    return r

def skip_group(t, i):
    o, c, d = t[i], OPENER[t[i]], 0
    for j in range(i, len(t)):
        if t[j] == o: d += 1
        elif t[j] == c:
            d -= 1
            if d == 0: return j + 1
    raise ValueError('unbalanced %s' % o)

def stmt_end(t, i):
    k = t[i]
    if k == 'begin':
        d = 0
        for j in range(i, len(t)):
            if t[j] in ('begin', 'fork'): d += 1
            elif t[j] in ('end', 'join'):
                d -= 1
                if d == 0: return j + 1
    if k == 'if':
        j = stmt_end(t, skip_group(t, i + 1))
        return stmt_end(t, j + 1) if j < len(t) and t[j] == 'else' else j
    if k in ('case', 'casez', 'casex'):
        d = 0
        for j in range(i, len(t)):
            if t[j] in ('case', 'casez', 'casex'): d += 1
            elif t[j] == 'endcase':
                d -= 1
                if d == 0: return j + 1
    if k in ('for', 'while', 'repeat'):
        return stmt_end(t, skip_group(t, i + 1))
    d = 0
    for j in range(i, len(t)):
        if t[j] in OPENER: d += 1
        elif t[j] in (')', ']', '}'): d -= 1
        elif t[j] == ';' and d == 0: return j + 1
    raise ValueError('no end of statement at %s' % ' '.join(t[i:i + 8]))

def walk(t, i, j, cond, unc, out):
    """Every assignment in t[i:j]: (lhs tokens, rhs tokens, condition identifiers, unconditional)."""
    while i < j:
        k = t[i]
        if k == ';':
            i += 1
        elif k == 'begin':
            e = stmt_end(t, i)
            s = i + 3 if t[i + 1] == ':' else i + 1
            walk(t, s, e - 1, cond, unc, out); i = e
        elif k == 'if':
            pe = skip_group(t, i + 1)
            c = cond + idents(t[i + 2:pe - 1])
            e = stmt_end(t, pe)
            walk(t, pe, e, c, False, out)
            if e < j and t[e] == 'else':
                e2 = stmt_end(t, e + 1); walk(t, e + 1, e2, c, False, out); e = e2
            i = e
        elif k in ('case', 'casez', 'casex'):
            pe = skip_group(t, i + 1)
            c = cond + idents(t[i + 2:pe - 1])
            e, m = stmt_end(t, i), pe
            while t[m] != 'endcase':
                n, d = m, 0
                while not (t[n] == ':' and d == 0) and not (n == m + 1 and t[m] == 'default'):
                    d += t[n] in OPENER; d -= t[n] in (')', ']', '}'); n += 1
                n -= t[n] != ':'
                se = stmt_end(t, n + 1)
                walk(t, n + 1, se, c + idents(t[m:n]), False, out); m = se
            i = e
        elif k in ('for', 'while', 'repeat'):
            pe = skip_group(t, i + 1); e = stmt_end(t, pe)
            walk(t, pe, e, cond + idents(t[i + 2:pe - 1]), False, out); i = e
        else:
            e = stmt_end(t, i); s, d = t[i:e - 1], 0
            for n, x in enumerate(s):
                d += x in OPENER; d -= x in (')', ']', '}')
                if d == 0 and x in ('<=', '='):
                    out.append((s[:n], s[n + 1:], cond, unc)); break
            i = e

def lhs_names(lhs):
    """(target names, identifiers read in their selects) of an assignment's left-hand side."""
    names, reads, i = [], [], 0
    while i < len(lhs):
        t = lhs[i]
        if t == '[':
            e = skip_group(lhs, i); reads += idents(lhs[i + 1:e - 1]); i = e; continue
        if IDENT.match(t) and t not in KW: names.append(t)
        i += 1
    return names, reads

def conns_of(t, i):
    """{port: tokens} of the connection list whose '(' is t[i]."""
    e, r = skip_group(t, i), {}
    for part in split_top(t[i + 1:e - 1]):
        if len(part) >= 4 and part[0] == '.' and part[2] == '(':
            r[part[1]] = part[3:-1]
    return r, e

def parse_module(t):
    """A module's items: ports, regs, nets, memories, constants, clocked and combinational blocks, instances."""
    name, i = t[1], 2
    if t[i] == '#': i = skip_group(t, i + 1)
    pe = skip_group(t, i)
    M = {'name': name, 'ports': {}, 'regs': {}, 'mems': set(), 'consts': set(), 'comb': {}, 'combrhs': {},
         'blocks': [], 'insts': []}
    for part in split_top(t[i + 1:pe - 1]):
        if part:
            if part[0] in ('input', 'output', 'inout'): last = part[0]
            M['ports'][part[-1]] = last
    i, attr = pe + 1, []
    while i < len(t):
        k = t[i]
        if k == '(' and t[i + 1] == '*':
            j = i + 2
            while not (t[j] == '*' and t[j + 1] == ')'): j += 1
            attr = t[i + 2:j]; i = j + 2; continue
        if k in ('function', 'task'):
            j, h = t.index('end' + k, i), t[i + 1:t.index(';', i)]
            M['consts'].add(h[h.index('(') - 1] if '(' in h else [x for x in h if IDENT.match(x)][-1])
            i = j + 1; continue
        if k in ('generate', 'endgenerate', 'else', 'end'):
            i += 1; continue
        if k == 'begin':
            i += 3 if t[i + 1] == ':' else 1; continue
        if k == 'if':
            i = skip_group(t, i + 1); continue
        if k in ('parameter', 'localparam', 'genvar'):
            e = stmt_end(t, i)
            for part in split_top(t[i + 1:e - 1]):
                head = part[:part.index('=')] if '=' in part else part
                ns = [x for x in head if IDENT.match(x) and x not in KW]
                if ns: M['consts'].add(ns[-1])
            i = e; continue
        if k in ('wire', 'reg', 'logic', 'integer', 'string', 'real', 'tri'):
            e = stmt_end(t, i); s = t[i + 1:e - 1]; n = 0
            while n < len(s) and s[n] in ('signed', 'unsigned', 'wire', 'reg', 'logic'): n += 1
            if n < len(s) and s[n] == '[': n = skip_group(s, n)
            for part in split_top(s[n:]):
                if not part: continue
                nm = part[0]
                if len(part) > 1 and part[1] == '[': M['mems'].add(nm)
                elif k == 'wire' and '=' in part:
                    M['comb'][nm] = idents(part[part.index('=') + 1:])
                    M['combrhs'][nm] = ' '.join(part[part.index('=') + 1:])
                if k in ('reg', 'logic', 'integer'):
                    M['regs'][nm] = {'attr': attr, 'kind': k}
                elif k == 'wire' and nm not in M['comb']:
                    M['comb'].setdefault(nm, [])
            attr = []; i = e; continue
        if k == 'assign':
            e = stmt_end(t, i)
            for part in split_top(t[i + 1:e - 1]):
                q = part.index('=')
                ln, lr = lhs_names(part[:q])
                for nm in ln: M['comb'][nm] = M['comb'].get(nm, []) + idents(part[q + 1:]) + lr
            i = e; continue
        if k == 'always':
            pe = skip_group(t, i + 2); sens = t[i + 3:pe - 1]
            e = stmt_end(t, pe); out = []
            walk(t, pe, e, [], True, out)
            clk = None
            for edge in ('posedge', 'negedge'):
                if clk is None and edge in sens: clk = sens[sens.index(edge) + 1]
            if clk is None:
                rd = idents(t[pe:e])
                for lhs, _, _, _ in out:
                    for nm in lhs_names(lhs)[0]: M['comb'][nm] = M['comb'].get(nm, []) + rd
            else:
                M['blocks'].append((clk, out))
            i = e; continue
        if k == 'initial':
            i = stmt_end(t, i + 1); continue
        if IDENT.match(k) and k not in KW and i + 1 < len(t) and (t[i + 1] == '#' or
                                                                   (IDENT.match(t[i + 1]) and t[i + 2] == '(')):
            j, par = i + 1, {}
            if t[j] == '#':
                par, j = conns_of(t, j + 1)
            con, e = conns_of(t, j + 1)
            M['insts'].append((k, t[j], par, con))
            i = e + 1; continue
        i = stmt_end(t, i)
    return M

def modules_of(path, defs):
    t = toks2(pp(path, defs))
    r, i = {}, 0
    while i < len(t):
        if t[i] == 'module':
            j = t.index('endmodule', i)
            r[t[i + 1]] = parse_module(t[i:j])
            i = j
        i += 1
    return r

def attr_regs(t):
    """Names of the registers declared under a synchroniser attribute."""
    r = set()
    for i, x in enumerate(t):
        if x.startswith('"') and SYNATTR in x:
            j = i
            while not (t[j] == '*' and t[j + 1] == ')'): j += 1
            j += 2
            e = stmt_end(t, j); s = t[j + 1:e - 1]; n = 0
            if s and s[0] == '[': n = skip_group(s, 0)
            r |= {p[0] for p in split_top(s[n:]) if p}
    return r

def domains(M):
    """Register -> clocks of the blocks assigning it; net -> its drivers; and each cell output's clock."""
    reg = {}
    for clk, out in M['blocks']:
        for lhs, _, _, _ in out:
            for nm in lhs_names(lhs)[0]:
                if nm not in M['mems']: reg.setdefault(nm, set()).add(clk)
    cell = {}
    for mod, _, _, con in M['insts']:
        outs = {'lw_sync': [('q', 'clk')], 'lw_xfer': [('d_q', 'd_clk'), ('d_v', 'd_clk'), ('s_busy', 's_clk')],
                'lw_arst': [('q', 'clk')],
                'lw_fence': [('hen_s', 'clk_sys'), ('open', 'clk'), ('owned', 'clk'), ('idle', 'clk'),
                             ('lvl', 'clk'), ('fall', 'clk')]}.get(mod, [])
        for o, c in outs:
            for nm in lhs_names(con.get(o, []))[0]: cell[nm] = ''.join(con.get(c, []))
    clocks = {c for c, _ in M['blocks']}
    deflt = 'clk' if 'clk' in M['ports'] else (sorted(clocks)[0] if len(clocks) == 1 else None)
    second = PORTDOM.get(M['name'], (None, ''))
    port = {p: (second[0] if p in second[1].split() else deflt) for p, d in M['ports'].items()
            if d in ('input', 'inout')}
    return reg, cell, port

def sources(M, reg, cell, port, n, seen=None):
    """(leaf, clock) pairs a read of n reaches through the module's nets; leaves of no known clock are left out."""
    seen = set() if seen is None else seen
    if n in seen or n in M['consts'] or n in M['mems']: return set()
    if PRESENCE.get(M['name'], {}).get(n, 0) == M['combrhs'].get(n) and len(M['comb'][n]) == 1: return set()
    seen.add(n)
    if n in reg: return {(n, c) for c in reg[n]}
    if n in cell: return {(n, cell[n])}
    if n in port: return {(n, port[n])} if port[n] else set()
    r = set()
    for m in M['comb'].get(n, []): r |= sources(M, reg, cell, port, m, seen)
    return r

def plain_ref(ts):
    """The names of a plain reference (a name, a select of one, or a concatenation of them), or None."""
    if ts[:1] == ['{'] and ts[-1:] == ['}']:
        terms = split_top(ts[1:-1])
    else:
        terms = [ts]
    r = []
    for term in terms:
        if not term or not IDENT.match(term[0]): return None
        if len(term) > 1 and (term[1] != '[' or skip_group(term, 1) != len(term)): return None
        r.append(term[0])
    return r

print('-- crossing structure: the cells hold every crossing register, and every source has its name')
lwfiles = sorted('sys/' + x for x in os.listdir('sys') if re.match(r'(hps_lw_|lw_)\w*\.s?v$', x))
stock = ['sys/sys_top.v', 'sys/hps_io.sv', 'sys/osd.v', 'sys/sysmem.sv']
check(CDC in lwfiles, '%s: no such file' % CDC)
# rule 1: cell register names and the synchroniser attribute
for f in sorted(os.listdir('sys')):
    p = 'sys/' + f
    if p == CDC or not re.search(r'\.(s?v|vh)$', f): continue
    bad = sorted({x for x in toks2(open(p, errors='replace').read()) if x.startswith(CELLREG)})
    check(not bad, '%s: declares or names a cell register outside %s: %s' % (p, CDC, ' '.join(bad)))
for p in lwfiles:
    a = attr_regs(toks2(pp(p, LWALL)))
    if p == CDC:
        check(a == CELLSYNC, '%s: the synchroniser attribute is on %s, not exactly %s' %
              (p, ' '.join(sorted(a)), ' '.join(sorted(CELLSYNC))))
    else:
        check(not a, '%s: a register with the synchroniser attribute outside the cells: %s' % (p, ' '.join(sorted(a))))
for p in stock:
    a = attr_regs(toks2(pp(p, LWALL))) - attr_regs(toks2(pp(p, [])))
    check(a == SYNC_ALLOW.get(p, set()), '%s: the LW text puts the synchroniser attribute on %s, the list says %s' %
          (p, ' '.join(sorted(a)) or 'nothing', ' '.join(sorted(SYNC_ALLOW.get(p, set()))) or 'nothing'))
mods = {}
for p in lwfiles:
    for n, M in modules_of(p, LWALL).items():
        M['file'] = p; mods[n] = M
for p in stock[:3]:
    for n, M in modules_of(p, LWALL).items():
        if n in ('sys_top', 'hps_io', 'osd'):
            M['file'] = p; mods[n] = M
xfer = mods.get('lw_xfer')
if check(xfer is not None, '%s: no lw_xfer' % CDC):
    q = [x for x in xfer['insts'] if x[0] == 'lw_qstat']
    if check(len(q) == 1, 'lw_xfer: %d lw_qstat instances, not one' % len(q)):
        reg, cell, port = domains(xfer)
        en = {s for n in idents(q[0][3].get('en', [])) for s, _ in sources(xfer, reg, cell, port, n)}
        en |= {n for n in idents(q[0][3].get('en', [])) if n in xfer['regs']}
        check(en and not any(s.startswith('lwsrc_') for s in en), 'lw_xfer: the capture enable comes from %s' %
              (' '.join(sorted(en)) or 'nothing'))
        d = plain_ref(q[0][3].get('d', []))
        check(d is not None and len(d) == 1 and d[0].startswith('lwsrc_') and d[0] in reg,
              'lw_xfer: the capture\'s D is %s, not a source hold register' % ' '.join(q[0][3].get('d', [])))
# rule 2: every lw_sync's D is a named source register of its own module; lw_qstat only in lw_xfer; lw_arst inputs
nsrc, nsync, nfence, fed = 0, 0, 0, set()
for n, M in sorted(mods.items()):
    reg, cell, port = domains(M)
    for mod, inst, par, con in M['insts']:
        where = '%s: %s %s' % (n, mod, inst)
        if mod == 'lw_qstat':
            check(n == 'lw_xfer', '%s: lw_qstat outside lw_xfer' % where)
        if mod == 'lw_arst':
            check(' '.join(con.get('rst_n', [])) in ('hps_rst_n', 'gp_out [ 31 ]'),
                  '%s: rst_n is %s, not hps_rst_n or gp_out[31]' % (where, ' '.join(con.get('rst_n', []))))
        if mod == 'lw_xfer':
            c = ''.join(con.get('s_clk', []))
            src = {s for x in ('s_go', 's_d') for m in idents(con.get(x, [])) for s in sources(M, reg, cell, port, m)}
            far = sorted('%s (%s)' % s for s in src if s[1] != c)
            check(not far, '%s: s_go and s_d read %s, not on s_clk %s' % (where, ' '.join(far), c))
        if mod == 'lw_fence':
            nfence += 1
            for side, ins in (('clk', ('hen', 'take', 'drain', 'fin')), ('clk_sys', ('rest',))):
                c = ''.join(con.get(side, []))
                src = {x for p_ in ins for m in idents(con.get(p_, [])) for x in sources(M, reg, cell, port, m)}
                far = sorted('%s (%s)' % x for x in src if x[1] != c)
                check(not far, '%s: the inputs on its %s side read %s, not on %s' % (where, side, ' '.join(far), c))
        if mod != 'lw_sync': continue
        nsync += 1
        d, dst = plain_ref(con.get('d', [])), ''.join(con.get('clk', []))
        if not check(d is not None, '%s: d is %s, not a plain reference' % (where, ' '.join(con.get('d', [])))):
            continue
        gray = ''.join(par.get('GRAY', ['0'])) not in ('0', "1'b0")
        for s in d:
            nsrc += 1; fed.add((n, s))
            # in the cells: a handshake's toggles, and the asynchronously set chain, whose rise has no clock
            chain = n == 'lw_arst' and s.startswith('lwa1_')
            okname = s.endswith('_lwx') or s.endswith('_lwgray') or chain or (n in ('lw_xfer', 'lw_fence') and
                                                                              s.startswith(('lwtg_', 'lwa1_')))
            check(okname, '%s: source %s is not named *_lwx or *_lwgray' % (where, s))
            check(s in M['regs'] and s in reg, '%s: source %s is not a register clocked in %s' % (where, s, n))
            check(chain or s not in reg or dst not in reg[s],
                  '%s: source %s is on the destination clock %s' % (where, s, dst))
            if s.endswith(('_lwx', '_lwgray')):
                check('preserve' in M['regs'].get(s, {}).get('attr', []),
                      '%s: source %s is declared without a preserve attribute' % (where, s))
            check(gray == s.endswith('_lwgray'), '%s: GRAY %d with source %s' % (where, gray, s))
for n, M in sorted(mods.items()):
    for r in M['regs']:
        if r.endswith(('_lwx', '_lwgray')):
            check((n, r) in fed, '%s: %s is named as a crossing source and feeds no lw_sync' % (n, r))
# rule 3: Gray sources, recomputed from their binary pointers in every cycle
for n, M in sorted(mods.items()):
    for r in sorted(x for x in M['regs'] if x.endswith('_lwgray')):
        bus = r[:-len('_lwgray')]
        check(bus in GRAYBUS, '%s: %s is not one of the Gray buses %s lists: %s' %
              (n, r, SDC, ' '.join(GRAYBUS)))
        asg = [(clk, rhs, unc) for clk, out in M['blocks'] for lhs, rhs, _, unc in out if lhs_names(lhs)[0] == [r]]
        asg += [(None, [], False)] * (r in M['comb'])
        ok = len(asg) == 1 and asg[0][2] and len(asg[0][1]) == 7 and IDENT.match(asg[0][1][0]) and \
             asg[0][1][1:] == ['^', '(', asg[0][1][0], '>>', '1', ')']
        check(ok, '%s: %s is not assigned once, unconditionally, as g <= b ^ (b >> 1): %s' %
              (n, r, ' / '.join(' '.join(a[1]) for a in asg) or 'never'))
check(GRAYBUS, '%s: no Gray-bus list (one line foreach g [list ...] {)' % SDC)
for bus in GRAYBUS:
    check(any(bus + '_lwgray' in M['regs'] for M in mods.values()),
          '%s: the Gray bus %s has no %s_lwgray register' % (SDC, bus, bus))
# every read across clocks goes through a cell: a clocked block reads nothing of another clock but a cell's output
seen_allow = set()
for n, M in sorted(mods.items()):
    if M['file'] == CDC or n in ('sys_top', 'hps_io', 'osd'): continue
    reg, cell, port = domains(M)
    for r, cs in sorted(reg.items()):
        check(len(cs) == 1, '%s: %s is assigned on %s' % (n, r, ' and '.join(sorted(cs))))
    found = set()
    for clk, out in M['blocks']:
        for lhs, rhs, cond, _ in out:
            names, sel = lhs_names(lhs)
            for m in idents(rhs) + cond + sel:
                for s, c in sources(M, reg, cell, port, m):
                    if c != clk:
                        for t in names: found.add((n, t, s, c, clk))
    for n_, t, s, c, clk in sorted(found):
        if (n_, t, s) in XREAD_ALLOW:
            seen_allow.add((n_, t, s)); continue
        check(False, '%s: %s on %s reads %s on %s with no cell between' % (n_, t, clk, s, c))
for a in sorted(XREAD_ALLOW - seen_allow):
    check(False, '%s: the listed read of %s by %s is gone; take it off the list' % (a[0], a[2], a[1]))
for n, nets in sorted(PRESENCE.items()):
    for net, sel in sorted(nets.items()):
        got = mods[n]['combrhs'].get(net) if n in mods and len(mods[n]['comb'].get(net, [])) == 1 else None
        check(got == sel, '%s: the presence net %s is %s, not %s' % (n, net, got or 'gone', sel))
print('crossing structure: %d lw_sync instances, %d source registers, %d enable links, %d listed reads across '
      'clocks' % (nsync, nsrc, nfence, len(seen_allow)))
check(nfence == 3, 'enable links: %d lw_fence instances, not one each in lw_sector, lw_osd and lw_lane' % nfence)

# ---- The firewall's two GPI outputs: present is the constant 1, idle a clk register ----
print('-- hps_lw_bridge: present is the constant 1 and idle the register idle_q')
fwt = tokens(pp('sys/hps_lw_bridge.sv', ['MISTER_LW']))
fwm, rbm = module(fwt, 'hps_lw_bridge'), module(fwt, 'lw_axi_regbus')
FWM = modules_of('sys/hps_lw_bridge.sv', ['MISTER_LW'])
def assigns(body, net):
    return [st[st.index('=') + 1:] for st in statements(body)
            if st[:1] == ['assign'] and '=' in st and st[1:st.index('=')] == [net]]
check(ports(fwm).get('present', '').split()[:2] == ['output', 'wire'], 'hps_lw_bridge: present is not an output wire')
check(assigns(fwm, 'present') == [["1'b1"]], 'hps_lw_bridge: present is %s, not the constant 1 alone'
      % assigns(fwm, 'present'))
check(not any('present' in idents(sum(con.values(), [])) for _, _, _, con in FWM['hps_lw_bridge']['insts']),
      'hps_lw_bridge: present is connected to an instance')
check([c.get('idle') for _, c in instances(fwm, 'lw_axi_regbus')] == [['idle']] and not assigns(fwm, 'idle'),
      'hps_lw_bridge: idle is not lw_axi_regbus.idle alone')
check(assigns(rbm, 'idle') == [['idle_q']], 'lw_axi_regbus: idle is %s, not idle_q' % assigns(rbm, 'idle'))
RB = FWM['lw_axi_regbus']
idq = [clk for clk, out in RB['blocks'] if any('idle_q' in lhs_names(lhs)[0] for lhs, _, _, _ in out)]
check('idle_q' in RB['regs'] and idq == ['clk'] and 'idle_q' not in RB['comb'],
      'lw_axi_regbus: idle_q is not a register assigned only on posedge clk (blocks on %s)' % idq)

# ---- The directory's lines: one ordinal per entry, each driving its own bit of lines ----
print('-- lw_sys_window: each LINE word an entry\'s own ordinal, and the same localparam picks its bit of lines')
wtxt = ' '.join(toks2(pp('sys/lw_sys_window.sv', LWALL)))
def lp(suffix, width):
    r = {}
    for m in re.finditer(r"localparam \[ %d : 0 \] (\w+)_%s = %d'd(\d+) ;" % (width - 1, suffix, width), wtxt):
        r[m.group(1)] = int(m.group(2))
    return r
KINDS, ORDS = lp('KIND', 16), lp('LINE', 8)
check(KINDS and set(KINDS) == set(ORDS), 'lw_sys_window: the blocks with a kind (%s) and with a LINE localparam (%s) '
      'differ' % (' '.join(sorted(KINDS)), ' '.join(sorted(ORDS))))
calls = re.findall(r'\bent \( \w+ , (\w+)_KIND , [^;]*?, (\w+)_LINE ,', wtxt)
check(len(calls) == len(KINDS) and all(a == b for a, b in calls) and sorted(a for a, _ in calls) == sorted(KINDS),
      'lw_sys_window: the entries are not one ent() call per block with its own _KIND and _LINE: %s' % calls)
nz = sorted(ORDS[b] for b in ORDS if ORDS[b])
check(len(nz) == len(set(nz)), 'lw_sys_window: an ordinal is named by two entries: %s' % ORDS)
for b, o in sorted(ORDS.items()):
    if o:
        lo, hi = (1, 15) if KINDS.get(b, 0) < 0x8000 else (16, 31)
        check(lo <= o <= hi, 'lw_sys_window: %s_LINE = %d, outside %d-%d for kind %s' % (b, o, lo, hi, KINDS.get(b)))
m = re.search(r'wire \[ 2 : 0 \] line_d = (.*?) ;', wtxt)
LT = r"\( \{ 3 \{ (\w+)_lvl & line_en \[ e_(\w+) \] \} \} & line_bit \( (\w+)_LINE \) \)"
terms3 = re.findall(LT, m.group(1)) if m else []
terms = [(l, b) for l, e, b in terms3]
rest = re.sub(LT, '', m.group(1)).replace('|', '').strip() if m else 'none'
check(m is not None and not rest and len(terms) == len(set(t for _, t in terms)) and
      all(l.upper() == e.upper() == b for l, e, b in terms3) and
      sorted(b for _, b in terms) == sorted(b for b in ORDS if ORDS[b]),
      'lw_sys_window: line_d is not one term {3{<b>_lvl & line_en[e_<b>]}} & line_bit(<B>_LINE) per non-zero '
      'ordinal: %s %s' % (terms3, rest))
WM = modules_of('sys/lw_sys_window.sv', LWALL)['lw_sys_window']
def asg(M, r):
    return [(clk, ' '.join(rhs)) for clk, out in M['blocks'] for lhs, rhs, _, _ in out if lhs_names(lhs)[0] == [r]]
check(re.search(r'assign lines = line_q ;', wtxt) is not None and len(re.findall(r'assign lines =', wtxt)) == 1,
      'lw_sys_window: lines is not line_q alone')
check(asg(WM, 'line_q') == [('clk', '{ 3 { en_q } } & line_d')] and 'line_q' in WM['regs'] and
      'line_q' not in WM['comb'], 'lw_sys_window: line_q is not assigned once, on posedge clk, as {3{en_q}} & line_d: '
      '%s' % asg(WM, 'line_q'))
check(re.search(r'assign held = en_q ;', wtxt) is not None and len(re.findall(r'assign held =', wtxt)) == 1,
      'lw_sys_window: held is not en_q alone')
lq = asg(WM, 'en_q')
check(lq and all(c == 'clk' for c, _ in lq) and 'en_q' in WM['regs'] and 'en_q' not in WM['comb'],
      'lw_sys_window: en_q is not a register assigned only on posedge clk: %s' % lq)
check(re.search(r"line_bit = \( ord >= 8'd1 && ord <= 8'd3 \) \? \( 3'd1 << \( ord - 8'd1 \) \) : 3'd0 ;",
                wtxt) is not None, 'lw_sys_window: line_bit(k) is not bit k-1 of the 3 lines')
pm = re.search(r'wire \[ 31 : 0 \] pending = (.*?) ;', wtxt)
pt = re.findall(r"\( \{ 31'd0 , (\w+)_lvl \} << e_(\w+) \)", pm.group(1)) if pm else []
check(pm is not None and
      sorted(l.upper() for l, _ in pt) == sorted(b.upper() for _, b in pt) == sorted(b for _, b in terms),
      'lw_sys_window: PENDING does not put each line\'s level on its own entry\'s bit: %s' % pt)
print('directory lines: %s' % ', '.join('%s ordinal %d' % (b, ORDS[b]) for b in sorted(ORDS) if ORDS[b]))

# ---- The wording: no word of the reference core's own devices in the framework's text ----
print('-- wording: no reference-core device word in the LW files, the LW text of the stock files, tb/ or the guide; the guide in its own terms')
# (spelled in pieces, so this file passes its own check)
RCW = re.compile(r'(?i)mem' + r'ca' + r'rd|(?<![a-z])c' + r'ard|ps' + r'x|play' + r'station')
def lw_lines(path):
    """(line number, text) of each line inside a branch that MISTER_LW* or LW_* opens."""
    out, stack = [], []
    for n, l in enumerate(open(path, errors='replace').read().split('\n'), 1):
        m = re.match(r'\s*`(ifdef|ifndef|elsif|else|endif)\b\s*(\w*)', l)
        if m:
            d, mac = m.groups()
            lw = mac.startswith('MISTER_LW') or mac.startswith('LW_')
            if d in ('ifdef', 'ifndef'): stack.append([d == 'ifdef' and lw, lw])
            elif d == 'elsif' and stack: stack[-1] = [lw, stack[-1][1] or lw]
            elif d == 'else' and stack: stack[-1][0] = stack[-1][1] and not stack[-1][0]
            elif d == 'endif' and stack: stack.pop()
            continue
        if any(b for b, _ in stack): out.append((n, l))
    return out
wfiles = [(p, None) for p in lwfiles + [SDC] * os.path.exists(SDC)] + [(p, 'lw') for p in stock + ['sys/emu_ports.vh']]
wfiles += [(d + '/' + x, None) for d in ('tb', 'tb/hps') if os.path.isdir(d)
           for x in sorted(os.listdir(d)) if re.search(r'\.(s?v|sh|c|h)$', x)]
GUIDE = 'docs/LW_BRIDGE.md'
wfiles += [(GUIDE, None)]
for p, how in wfiles:
    txt = lw_lines(p) if how else list(enumerate(open(p, errors='replace').read().split('\n'), 1))
    bad = [(n, l) for n, l in txt if RCW.search(l)]
    for n, l in bad[:8]: print('     %s:%d: %s' % (p, n, l.strip()[:100]))
    check(not bad, '%s: %d lines with a reference-core device word' % (p, len(bad)))
# the guide: no retired term of the host interface, no planning label, no address
GRET = re.compile(r'(?i:\blease|\bepoch\b|\bowner\b|\btimebase\b)|\b(?:TTL|RENEW|RELEASE|CLAIM)\b')
GLAB = re.compile(r'\b(?:F|D|O|Q|R[0-3]?|[CH]-W|T-[A-Z]+)-?\d{1,2}\b|\u00a7')
GIP = re.compile(r'(?<![\d.])\d{1,3}(?:\.\d{1,3}){3}(?![\d.])')
gtxt = list(enumerate(open(GUIDE, encoding='utf-8', errors='replace').read().split('\n'), 1))
for rx, what in ((GRET, 'a retired term of the host interface'), (GLAB, 'a planning label'), (GIP, 'an IPv4 address')):
    bad = [(n, l) for n, l in gtxt if rx.search(l)]
    for n, l in bad[:8]: print('     %s:%d: %s' % (GUIDE, n, l.strip()[:100]))
    check(not bad, '%s: %d lines with %s' % (GUIDE, len(bad), what))
# a drain ends on a level the drain brings to 0: RSTAT's reply_drops is never cleared, so RSTAT whole is no such level
GDRN = re.compile(r'\bRSTAT\s+(?:reads|is|==|equals)\s+0\b')
bad = [(n, l) for n, l in gtxt if GDRN.search(l)]
for n, l in bad[:8]: print('     %s:%d: %s' % (GUIDE, n, l.strip()[:100]))
check(not bad, '%s: %d lines that drain the lane until RSTAT reads 0, not its replies field' % (GUIDE, len(bad)))
check(any(re.search(r'RPOP until RSTAT', l) and re.search(r'\[15:0\]', l) for n, l in gtxt),
      '%s: no lane drain rule names RSTAT[15:0]' % GUIDE)
# at the enable's fall only a frame already playing completes; queued entries, committed or not, are discarded
GWHOLE = re.compile(r'(?i)delivered whole|plays out whole, whatever follows')
bad = [(n, l) for n, l in gtxt if GWHOLE.search(l)]
for n, l in bad[:8]: print('     %s:%d: %s' % (GUIDE, n, l.strip()[:100]))
check(not bad, '%s: %d lines that promise a queued frame plays at the enable\'s fall' % (GUIDE, len(bad)))
gall = '\n'.join(l for n, l in gtxt)
fall = re.search(r'\*\*When the enable falls\*\*(.*?)\n\n', gall, re.S)
check(fall is not None and re.search(r'committed\s+or\s+not', fall.group(1)) is not None and
      re.search(r'already\s+playing\s+completes', fall.group(1)) is not None,
      '%s: "When the enable falls" does not say that only a frame already playing completes and that queued '
      'entries, committed or not, are discarded' % GUIDE)

# ---- The constraints: every bound names both ends, every pattern matches, every source is covered ----
print('-- lw_bridge.sdc: -from and -to on every bound, every name pattern matched, every lw_sync source covered')
sdcl = [l.strip() for l in SDCTXT.split('\n') if l.strip() and not l.strip().startswith('#')]
check(sdcl, '%s: no such file, or no command in it' % SDC)
# the collections: name -> patterns; the Gray loop's query is expanded per bus
COLL, pats = {}, []
for l in sdcl:
    for v, q, a in re.findall(r'\bset (\w+) +\[get_keepers -nowarn (\{([^{}]*)\}|"[^"]*")\]', l):
        ps = a.split() if a else [q[1:-1].replace('${g}', b) for b in GRAYBUS]
        COLL.setdefault(v, []).extend(ps); pats += ps
    n = len(re.findall(r'\bget_(?:keepers|registers|pins|nets|cells)\b', l))
    check(n == len(re.findall(r'\bset \w+ +\[get_keepers -nowarn (?:\{[^{}]*\}|"[^"]*")\]', l)),
          '%s: a query this check cannot read: %s' % (SDC, l))
nbound = 0
for l in sdcl:
    for cmd in ('set_net_delay', 'set_max_skew'):
        if re.search(r'\b%s\b' % cmd, l):
            nbound += 1
            fr, to = re.search(r' -from \$(\w+)\b', l), re.search(r' -to \$(\w+)\b', l)
            check(fr is not None and to is not None, '%s: %s without both -from and -to: %s' % (SDC, cmd, l))
            for m in (fr, to):
                if m:
                    check(m.group(1) in COLL, '%s: %s names $%s, which no get_keepers builds' % (SDC, cmd, m.group(1)))
# every register of every LW module as a keeper name; a pattern's * matches across the hierarchy bars too
KEEP = {('%s|%s:inst|%s' % ('sys_top', n, r)): (n, r) for n, M in mods.items() for r in M['regs']}
def matched(ps):
    return {KEEP[k] for k in KEEP for p_ in ps if fnmatch.fnmatchcase(k, p_)}
for p_ in pats:
    check(matched([p_]), '%s: the pattern %s matches no register of an LW module' % (SDC, p_))
CELLS = {'lw_s0': {('lw_sync', 'lws0_q')}, 'lw_a': {('lw_arst', 'lwa0_q'), ('lw_arst', 'lwa1_q')},
         'lw_q': {('lw_qstat', 'lwq_d')}, 'lw_src': {('lw_xfer', 'lwsrc_d')}, 'lw_x': fed}
for v, want in sorted(CELLS.items()):
    got = matched(COLL.get(v, []))
    check(got == want, '%s: $%s also matches [%s] and misses [%s]' % (SDC, v,
          ' '.join(sorted('%s.%s' % x for x in got - want)), ' '.join(sorted('%s.%s' % x for x in want - got))))
# the bits, for the post-fit pair count: per block at its default parameters, every generate branch, one each
LIT = re.compile(r"(\d*)\s*'\s*[sS]?([bodhBODH])\s*([\w?]+)$")
def veval(ts, env):
    out = []
    for t in ts:
        m = LIT.match(t)
        if m: out.append(str(int(m.group(3).replace('_', ''), {'b': 2, 'o': 8, 'd': 10, 'h': 16}[m.group(2).lower()])))
        elif IDENT.match(t): out.append(str(env[t]))
        elif re.match(r'\d+$', t) or t in ('+', '-', '*', '<<', '>>', '(', ')'): out.append(t)
        else: raise ValueError(t)
    return eval(' '.join(out), {'__builtins__': {}})
def defaults(path, name):
    t, env = toks2(pp(path, LWALL)), {}
    b = t.index('module', next(i for i in range(len(t)) if t[i] == 'module' and t[i + 1] == name))
    t = t[b:t.index('endmodule', b)]
    for i, k in enumerate(t):
        if k in ('parameter', 'localparam'):
            j = i + 1
            while t[j] in ('signed', 'integer') or t[j] == '[':
                j = skip_group(t, j) if t[j] == '[' else j + 1
            if t[j + 1] != '=': continue
            e, d = j + 2, 0
            while d or t[e] not in (',', ';', ')'):
                d += t[e] in OPENER; d -= t[e] in (')', ']', '}'); e += 1
            try: env[t[j]] = veval(t[j + 2:e], env)
            except (ValueError, KeyError, SyntaxError): pass
    return env
CELLMOD = {'lw_sync', 'lw_xfer', 'lw_fence', 'lw_arst', 'lw_qstat'}
def bits(n, env):
    sb = qb = 0
    for mod, inst, par, con in mods[n]['insts']:
        if mod not in CELLMOD: continue
        ce = defaults(mods[mod]['file'], mod)
        ce.update({k: veval(v, env) for k, v in par.items()})
        if mod == 'lw_sync': sb += ce['W']
        elif mod == 'lw_qstat': qb += ce['W']
        else:
            a, b = bits(mod, ce); sb += a; qb += b
    return sb, qb
per, tot = [], [0, 0]
for n in sorted(mods):
    if n in CELLMOD: continue
    try: a, b = bits(n, defaults(mods[n]['file'], n))
    except (ValueError, KeyError, SyntaxError) as e:
        check(False, '%s: cannot count its crossing bits at its defaults (%s)' % (n, e)); continue
    if a or b:
        per.append('%s %d+%d' % (n, a, b)); tot[0] += a; tot[1] += b
print('constraints: %d bounds; %d source bits into first stages and %d bundle bits, one instance of each block at '
      'its defaults (%s)' % (nbound, tot[0], tot[1], ', '.join(per)))

print('%s: %d checks, %d failures' % ('FAILED' if fails else 'PASSED', checks, fails))
sys.exit(1 if fails else 0)
EOF
