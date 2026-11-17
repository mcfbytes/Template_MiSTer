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

print('%s: %d checks, %d failures' % ('FAILED' if fails else 'PASSED', checks, fails))
sys.exit(1 if fails else 0)
EOF
