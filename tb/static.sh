#!/bin/sh
# Static checks on the preprocessed top level, arm N of tb/run.sh, and the net lists arm E checks.
#   tb/static.sh            the checks below, for every configuration of --configs; "-- " headings, a FAIL line
#                           per failed check, then PASSED/FAILED with the counts (tb/run.sh's verdict)
#   tb/static.sh --configs  one "name|macros" line per configuration arm E elaborates: off and each LW tier
#                           set, each with no upstream macro, with MISTER_DEBUG_NOHDMI and with MISTER_DUAL_SDRAM
#   tb/static.sh --nets FILE MACRO...
#                           FILE gets one Verilog check call per LW net of that configuration, for tb/top_tb.sv
# An LW net is a net sys_top.v declares in this configuration whose declaration the off configuration of the
# same upstream variant does not have word for word (the LW wires, and every stock net the LW text re-declares,
# such as the link nets the lane takes over), and every port of osd, hps_io and emu that the configuration
# declares differently from off, at each of their instances. Arm E fails if one of them reads z.
# The checks, each in every configuration, on sys_top.v as Icarus preprocesses it:
#   1. The discovery bit and the two LW instances come together or not at all: gp_in[27] is the constant 1 in
#      exactly the configurations that instantiate both hps_lw_bridge and lw_sys_window, and those are exactly
#      the ones with MISTER_LW. gp_in[26:21] are constant 0, and gp_in is 32 bits wide.
#   2. f2h_irq: bits 1:0 are upstream's, bit 8 is the net lw_sys_window's doorbell output drives and bit 9 the
#      one its sys_irq output drives, each assigned nowhere else; every other bit is a constant 0.
#   3. Every LW configuration has LW nets to check; the off configuration has none.
# Then, once, the crossing structure, on every LW-only file and the LW text of the stock files, with every tier:
#   4. The cell registers (lws0_, lws1_, lwa0_, lwa1_, lwq_, lwsrc_, lwtg_) are named only in sys/lw_cdc.sv, and
#      the synchroniser attribute is on its two-flop stages alone, beside the listed exceptions (SYNC_ALLOW).
#      lw_qstat is built only inside lw_xfer, whose capture enable comes from no lwsrc_ register and whose D is
#      its hold register; an lw_arst input is hps_rst_n or gp_out[31].
#   5. The D of every lw_sync is a plain reference (names, selects, a concatenation of them) to registers of the
#      same module, clocked there and not on the destination clock, named *_lwx (GRAY 0) or *_lwgray (GRAY 1),
#      each declared with a preserve attribute (in lw_cdc.sv, also a cell's own toggles); every register so
#      named feeds one. An lw_xfer's s_go and s_d are on its s_clk.
#   6. Every *_lwgray register is one of the Gray buses and is assigned once, unconditionally, as g <= b ^ (b >> 1).
#   7. No clocked block outside the cells reads, directly or through nets, a register or an input on another
#      clock (input clocks from PORTDOM) other than through a cell's output, beside the listed reads
#      (XREAD_ALLOW), each of which must still exist; and no register is assigned on two clocks.
# Needs Icarus Verilog (for its preprocessor) and python3.
cd "$(dirname "$0")/.." || exit 2
O=${LW_TB_OUT:-tb/out}/static; mkdir -p "$O" || exit 2
command -v iverilog >/dev/null 2>&1 || { echo "FAILED: no iverilog on PATH"; exit 1; }
exec python3 -I - "$O" "$@" <<'EOF'
import os, re, subprocess, sys
O, args = sys.argv[1], sys.argv[2:]

TIERS = [('off', []), ('lw', ['MISTER_LW']), ('lw-sec', ['MISTER_LW', 'MISTER_LW_SECTOR_SLOT=1']),
         ('lw-lane', ['MISTER_LW', 'MISTER_LW_LANE']),
         ('lw-sec-lane', ['MISTER_LW', 'MISTER_LW_SECTOR_SLOT=1', 'MISTER_LW_LANE'])]
VARIANTS = [('', []), ('.nohdmi', ['MISTER_DEBUG_NOHDMI']), ('.dual', ['MISTER_DUAL_SDRAM'])]
UPSTREAM = {'MISTER_DEBUG_NOHDMI', 'MISTER_DUAL_SDRAM'}
CONFIGS = [(t + v, td + vd) for v, vd in VARIANTS for t, td in TIERS]
# f2h_irq bit -> the lw_sys_window output that must drive it
LINES = {8: 'doorbell', 9: 'sys_irq'}
DISC = 27

if args[:1] == ['--configs']:
    for n, d in CONFIGS:
        print('%s|%s' % (n, ' '.join('-D' + x for x in d)))
    sys.exit(0)

open(os.path.join(O, 'build_id.v'), 'w').write('`define BUILD_DATE "000000"\n')

def pp(path, defs):
    out = os.path.join(O, 'pp.v')
    r = subprocess.run(['iverilog', '-E', '-I', 'sys', '-I', O, '-o', out] + ['-D' + d for d in defs] + [path],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if r.returncode:
        sys.exit('tb/static.sh: cannot preprocess %s with %s: %s' % (path, ' '.join(defs), r.stdout.strip()))
    return open(out, errors='replace').read()

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
    print('-- %s: the discovery bit, the lines and the LW nets (%s)' % (name, ' '.join(defs) or 'no macro'))
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
        disc = gp[DISC][0] == ["1'b1"]
        check(disc == fw == win == lw, "%s: discovery bit %d %s, hps_lw_bridge %s, lw_sys_window %s, MISTER_LW %s"
              % (name, DISC, 'set' if disc else 'clear', fw, win, lw))
        zero = all(lit_value(gp[b][0][0]) == 0 and len(gp[b][0]) == 1 for b in range(21, DISC))
        check(zero, '%s: gp_in[26:21] is not constant 0' % name)
    irq = bitmap(D.get('f2h_irq', (0, 0, None))[2], W)
    if check(irq is not None and len(irq) <= 64, '%s: f2h_irq is not a concatenation of sized terms' % name):
        irq += [(["1'b0"], 0)] * (64 - len(irq))
        check([' '.join(t) for t, _ in irq[:2]] == ['HDMI_TX_VS', 'video_sync'],
              '%s: f2h_irq[1:0] is not upstream\'s' % name)
        conn = inst['lw_sys_window'][0][1] if win else {}
        for b in range(2, 64):
            t = irq[b][0]
            if b in LINES and lw:
                ok = len(t) == 1 and conn.get(LINES[b]) == t and D.get(t[0], (0, 0, 0))[2] is None and \
                     not any(s[:2] == ['assign', t[0]] for s in statements(top))
                check(ok, '%s: f2h_irq[%d] is %s, not a net driven by lw_sys_window.%s alone' % (name, b, ' '.join(t),
                                                                                                   LINES[b]))
            else:
                check(len(t) == 1 and lit_value(t[0]) == 0, '%s: f2h_irq[%d] is %s, not 0' % (name, b, ' '.join(t)))
    check(bool(nets) == lw, '%s: %d LW nets for arm E' % (name, len(nets)))
    print('%s: %d LW nets (%s), %d LW ports (%s)' % (name, len(nets), ' '.join(nets), len(lwports), ' '.join(lwports)))
    write_nets(os.path.join(O, name + '.nets.vh'), nets, lwports)

# ---- The crossing structure: every crossing register in sys/lw_cdc.sv's cells, every source named ----
LWALL = ['MISTER_LW', 'MISTER_LW_SECTOR_SLOT=1', 'MISTER_LW_LANE', 'LW_DIAG=1']
CDC = 'sys/lw_cdc.sv'
CELLREG = ('lws0_', 'lws1_', 'lwa0_', 'lwa1_', 'lwq_', 'lwsrc_', 'lwtg_')
CELLSYNC = {'lws0_q', 'lws1_q', 'lwa0_q', 'lwa1_q'}
SYNATTR = 'SYNCHRONIZER_IDENTIFICATION'
# osd.v's two synchronisers (CTRL's two bits into clk_sys and clk_video), until its LW stores write through
# osd_buffer and the shadow goes
SYNC_ALLOW = {'sys/osd.v': {'lwhr_s0', 'lwhr_s1', 'lwsel_v0', 'lwsel_v1'}}
GRAYBUS = ['sector_w', 'sector_r', 'lane_w', 'lane_r', 'lane_rw', 'lane_rr', 'osd_w', 'osd_r']
GRAY_ALLOW = {'stflg'}     # the status flag's Gray code, until the status snapshot crosses whole
# Reads across clocks outside the cells, as (module, register, source): the request LBAs, until requests cross as
# records; the status snapshot, a quasi-static capture until it crosses whole. Each must still be there.
XREAD_ALLOW = {('lw_sector', 'lba', 'c_lba'), ('lw_sector', 'xlba', 'x_lba'),
               ('lw_sys_status', 'snap', 'status_req')}
# The input ports on a block's second clock. Every other input is on the block's port clk, or on the one clock
# of a block that has no clk port.
PORTDOM = {'lw_sector': ('clk_sys', 'c_reset c_req c_lba hps_ack x_rd x_wr x_lba x_hps_ack'),
           'lw_sys_status': ('clk_sys', 'stflg status_req'),
           'lw_lane': ('clk_sys', 'l_busy l_wait l_dout'),
           'lw_sys_window': ('clk_sys', 'blk_up card_up st_up lane_busy lane_wait lane_dout')}

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
    M = {'name': name, 'ports': {}, 'regs': {}, 'mems': set(), 'consts': set(), 'comb': {}, 'blocks': [],
         'insts': []}
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
                'lw_arst': [('q', 'clk')]}.get(mod, [])
        for o, c in outs:
            for nm in lhs_names(con.get(o, []))[0]: cell[nm] = ''.join(con.get(c, []))
    clocks = {c for c, _ in M['blocks']}
    deflt = 'clk' if 'clk' in M['ports'] else (sorted(clocks)[0] if len(clocks) == 1 else None)
    second = PORTDOM.get(M['name'], (None, ''))
    port = {p: (second[0] if p in second[1].split() else deflt) for p, d in M['ports'].items() if d == 'input'}
    return reg, cell, port

def sources(M, reg, cell, port, n, seen=None):
    """(leaf, clock) pairs a read of n reaches through the module's nets; leaves of no known clock are left out."""
    seen = set() if seen is None else seen
    if n in seen or n in M['consts'] or n in M['mems']: return set()
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
nsrc, nsync, fed = 0, 0, set()
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
        if mod != 'lw_sync': continue
        nsync += 1
        d, dst = plain_ref(con.get('d', [])), ''.join(con.get('clk', []))
        if not check(d is not None, '%s: d is %s, not a plain reference' % (where, ' '.join(con.get('d', [])))):
            continue
        gray = ''.join(par.get('GRAY', ['0'])) not in ('0', "1'b0")
        for s in d:
            nsrc += 1; fed.add((n, s))
            okname = s.endswith('_lwx') or s.endswith('_lwgray') or (n in ('lw_xfer', 'lw_fence') and
                                                                     s.startswith(('lwtg_', 'lwa1_')))
            check(okname, '%s: source %s is not named *_lwx or *_lwgray' % (where, s))
            check(s in M['regs'] and s in reg, '%s: source %s is not a register clocked in %s' % (where, s, n))
            check(s not in reg or dst not in reg[s], '%s: source %s is on the destination clock %s' % (where, s, dst))
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
        check(bus in GRAYBUS or bus in GRAY_ALLOW, '%s: %s is not one of the Gray buses %s' %
              (n, r, ' '.join(GRAYBUS)))
        asg = [(clk, rhs, unc) for clk, out in M['blocks'] for lhs, rhs, _, unc in out if lhs_names(lhs)[0] == [r]]
        asg += [(None, [], False)] * (r in M['comb'])
        ok = len(asg) == 1 and asg[0][2] and len(asg[0][1]) == 7 and IDENT.match(asg[0][1][0]) and \
             asg[0][1][1:] == ['^', '(', asg[0][1][0], '>>', '1', ')']
        check(ok, '%s: %s is not assigned once, unconditionally, as g <= b ^ (b >> 1): %s' %
              (n, r, ' / '.join(' '.join(a[1]) for a in asg) or 'never'))
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
print('crossing structure: %d lw_sync instances, %d source registers, %d listed reads across clocks' %
      (nsync, nsrc, len(seen_allow)))

print('%s: %d checks, %d failures' % ('FAILED' if fails else 'PASSED', checks, fails))
sys.exit(1 if fails else 0)
EOF
