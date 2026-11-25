#!/bin/sh
# Off-state guard, arm S of tb/run.sh: a build without MISTER_LW reads upstream's text. Upstream is the
# framework revision this branch was cut from ($UPSTREAM below). The checks:
#   1. sys_top.v, hps_io.sv, osd.v, sysmem.sv and emu_ports.vh, preprocessed with MISTER_LW and every tier
#      macro undefined, give upstream's token stream (comments and whitespace aside) with no macro, with each
#      macro upstream's own text tests (read from those five files at upstream), alone, and with all of them.
#   1b. osd.v with MISTER_LW and MISTER_LW_OSD, under each of those macro sets, gives upstream's token stream
#      from its clk_video half (the pixel path, from ce_pix's declaration) to the end: the OSDBUF tier changes
#      only its clk_sys write side.
#   1c. hps_io.sv with MISTER_LW and MISTER_LW_SECTOR_SLOT, under each of those macro sets, gives upstream's token
#      stream for uio_block (the SPI command decoder): SECTOR's commit mark watches the commands and changes none.
#   1d. hps_io.sv with MISTER_LW, MISTER_LW_PAD and MISTER_DISABLE_SPI_PAD gives the uio_block of the same build
#      without it (1c ties that one to upstream's) less exactly the link's eight pad arms (0x02, 0x03, 0x10-0x13,
#      0x1A, 0x3D), under each of those macro sets.
#   2. Every framework-only file (sys/hps_lw_*.sv, sys/lw_*.sv, sys/hps_ocram_*.sv) preprocesses to no token at
#      all without its floor macro, under each of those macro sets; a tier's file also with MISTER_LW and
#      without its tier macro (hps_ocram_rx.sv and lw_blk_mux.sv: without both of theirs); sys/lw_cdc.sv, which serves either floor, is empty with neither and present
#      with MISTER_LW or MISTER_OCRAM.
#   3. Every framework-only file keeps its whole body in its guard: `ifdef MISTER_LW (then its tier macro), or
#      `ifdef MISTER_OCRAM, first; `default_nettype none first inside it, `default_nettype wire last. Each macro
#      the file defines begins with LW_ (or, in hps_ocram_master.sv, MISTER_OCRAM_) and is undefined before the
#      file ends; it undefines no macro it did not define; it defines no LW_DIAG (a build sets that one).
#   4. Every other file under sys/, and files.qip, is byte-identical to upstream; sys.qip adds only
#      SYSTEMVERILOG_FILE lines for LW-only files and one SDC_FILE line for lw_bridge.sdc, right after
#      sys_top.sdc's; Template.qsf's assignments (its comment and blank lines removed) are upstream's, and its
#      bytes are: every upstream line kept byte for byte and in order, the lines added only comment or blank
#      lines, and every line ending as upstream's lines end (CRLF), so a later upstream change merges.
#   5. sys/lw_bridge.sdc is inert in a build without MISTER_LW: its one command outside a guard collects the
#      first synchroniser stages (lws0_*, which only sys/lw_cdc.sv declares, and check 2 empties without
#      MISTER_LW), and every other command sits inside "if {[get_collection_size $lw_s0] > 0} {", which the file
#      ends by closing.
#   6. Template.sv, preprocessed without MISTER_LW, gives upstream's token stream under each of those macro sets
#      (the LW example and its CONF_STR text are all inside MISTER_LW). Its CONF_STR value, evaluated by Icarus
#      from emu's localparams: upstream's without MISTER_LW; with it, upstream's with field 1 "AXI:40000:80000:LW,"
#      and, before "T[0],Reset;", the example's J line, and with the SECTOR block also the block reader's S line
#      on that slot (slots 1 and 3 tried).
# Upstream's tree is read with git into $LW_TB_OUT/upstream. Where git cannot reach the repository (the
# container tb/run.sh starts), run "tb/offstate.sh --extract" outside first; tb/run.sh does.
# Output: "-- " headings, a FAIL line per failed check, then PASSED/FAILED with the counts (tb/run.sh's verdict).
UPSTREAM=e901a9f
cd "$(dirname "$0")/.." || exit 2
O=${LW_TB_OUT:-tb/out}; UP="$O/upstream"
if git rev-parse --verify -q "$UPSTREAM^{commit}" >/dev/null 2>&1; then
  if [ "$(cat "$UP/.rev" 2>/dev/null)" != "$UPSTREAM" ] || [ ! -f "$UP/Template.sv" ]; then
    rm -rf "$UP" && mkdir -p "$UP" &&
      git archive "$UPSTREAM" sys files.qip Template.qsf Template.sv | tar -x -C "$UP" &&
      echo "$UPSTREAM" > "$UP/.rev" || { echo "tb/offstate.sh: cannot extract $UPSTREAM" >&2; exit 2; }
  fi
fi
[ "$1" = --extract ] && exit 0
if [ "$(cat "$UP/.rev" 2>/dev/null)" != "$UPSTREAM" ] || [ ! -f "$UP/Template.sv" ]; then
  echo "FAILED: upstream tree $UPSTREAM not available in $UP (run tb/offstate.sh --extract where git works)"; exit 1
fi
command -v iverilog >/dev/null 2>&1 || { echo "FAILED: no iverilog on PATH"; exit 1; }
exec python3 -I - "$UP" "$O" <<'EOF'
import difflib, hashlib, os, re, subprocess, sys
UP, O = sys.argv[1], sys.argv[2]
# A stock file this branch carries changed for its own sake, not for LW: its sha256 is pinned, so any other edit
# still fails. sys/ascal.vhd is ascal-hphase-dda 91c0abe (E32, bit-exact against stock in sim/ascal's GHDL bench).
PINNED = {'sys/ascal.vhd': '835a007dde05d310ffd38801b804d332fe6212337df44c3d9db7ec732d93fdc6'}
STOCK = ['sys/sys_top.v', 'sys/hps_io.sv', 'sys/osd.v', 'sys/sysmem.sv', 'sys/emu_ports.vh']
TIER = {'lw_pad.sv': 'MISTER_LW_PAD', 'lw_evt.sv': 'MISTER_LW_EVT', 'lw_sector.sv': 'MISTER_LW_SECTOR_SLOT',
        'lw_blk_mux.sv': 'MISTER_LW_SECTOR_SLOT', 'lw_lane.sv': 'MISTER_LW_LANE', 'lw_osd.sv': 'MISTER_LW_OSD'}
# the OCRAM tiers: the floor's macro instead of MISTER_LW
OTIER = {}
# files built under either of two tier macros: (floor, [tier macros], local macro); empty without the floor or
# without both tiers, opened as the floor, then each tier defining the local macro, then the local macro's guard
EITHER = {'hps_ocram_rx.sv': ('MISTER_OCRAM', ['MISTER_OCRAM_OSD', 'MISTER_OCRAM_SECTOR_SLOT'], 'LW_OCRAM_RX_BUILT'),
          'lw_blk_mux.sv': ('MISTER_LW', ['MISTER_LW_SECTOR_SLOT', 'MISTER_OCRAM_SECTOR_SLOT'], 'LW_BLK_MUX_BUILT')}
TIER.pop('lw_blk_mux.sv')
checks = fails = 0

def check(ok, what):
    global checks, fails
    checks += 1
    if not ok:
        fails += 1
        print('FAIL ' + what)
    return ok

TOK = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|`?[A-Za-z_$][\w$]*|\d[\w\'.]*|\'[sS]?[bodhBODH]\w*|\S',
                 re.S)
def tokens(text):
    return [t for t in TOK.findall(text) if not t.startswith('//') and not t.startswith('/*')]

def pp(path, defs, cwd=None):
    # cwd: where the file's `include "sys/..." resolves (upstream's tree for upstream's Template.sv); build_id.v from O
    out = os.path.abspath(os.path.join(O, 'offstate.pp'))
    r = subprocess.run(['iverilog', '-E', '-I', os.path.abspath(O), '-o', out] + ['-D' + d for d in defs] + [path],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=cwd)
    if r.returncode != 0:
        return None, r.stdout.strip()
    return tokens(open(out, errors='replace').read()), ''

def first_diff(a, b):
    n = next((i for i in range(min(len(a), len(b))) if a[i] != b[i]), min(len(a), len(b)))
    return n, ' '.join(a[max(0, n - 6):n + 6]), ' '.join(b[max(0, n - 6):n + 6])

# the macro sets: every name upstream's five files test, none, each alone, all together
names = set()
for f in STOCK:
    names |= set(re.findall(r'`(?:ifdef|ifndef|elsif)\s+(\w+)', open(os.path.join(UP, f)).read()))
names = sorted(names)
sets = [('no macro', [])] + [(n, [n]) for n in names] + [('all %d' % len(names), names)]
print('macro sets: none, each of %s, all' % ' '.join(names))

print('-- stock files preprocess to upstream\'s token stream under every upstream macro set')
for f in STOCK:
    for label, defs in sets:
        mine, e1 = pp(f, defs)
        theirs, e2 = pp(os.path.join(UP, f), defs)
        if not check(mine is not None and theirs is not None, '%s [%s]: preprocessor error %s%s' % (f, label, e1, e2)):
            continue
        if mine != theirs:
            n, a, b = first_diff(mine, theirs)
            check(False, '%s [%s]: token %d differs: ... %s ... upstream: ... %s ...' % (f, label, n, a, b))
        else:
            check(True, '')

print('-- osd.v\'s clk_video half is upstream\'s token stream with MISTER_LW and MISTER_LW_OSD, under every macro set')
for label, defs in sets:
    mine, e1 = pp('sys/osd.v', ['MISTER_LW', 'MISTER_LW_OSD'] + defs)
    theirs, e2 = pp(os.path.join(UP, 'sys/osd.v'), defs)
    if not check(mine is not None and theirs is not None, 'sys/osd.v [OSD, %s]: preprocessor error %s%s' %
                 (label, e1, e2)):
        continue
    if not check('direct_enable' in mine and 'direct_enable' in theirs,
                 'sys/osd.v [OSD, %s]: no clk_video half' % label):
        continue
    a_, b_ = mine[mine.index('direct_enable') - 2:], theirs[theirs.index('direct_enable') - 2:]
    if a_ != b_:
        n, x, y = first_diff(a_, b_)
        check(False, 'sys/osd.v [OSD, %s]: the clk_video half differs at token %d: ... %s ... upstream: ... %s ...' %
              (label, n, x, y))
    else:
        check(True, '')

def uio_block(t):
    # the tokens of 'always ... begin : uio_block' through its matching end, or None
    if 'uio_block' not in t:
        return None
    i = t.index('uio_block')
    a = max(j for j in range(i) if t[j] == 'always')
    b = next(j for j in range(a, i) if t[j] == 'begin')
    depth, j = 0, b
    while j < len(t):
        if t[j] == 'begin':
            depth += 1
        elif t[j] == 'end':
            depth -= 1
            if depth == 0:
                return t[a:j + 1]
        j += 1
    return None

print('-- hps_io.sv\'s uio_block is upstream\'s token stream with MISTER_LW and MISTER_LW_SECTOR_SLOT, under every macro set')
for label, defs in sets:
    mine, e1 = pp('sys/hps_io.sv', ['MISTER_LW', 'MISTER_LW_SECTOR_SLOT=1'] + defs)
    theirs, e2 = pp(os.path.join(UP, 'sys/hps_io.sv'), defs)
    if not check(mine is not None and theirs is not None, 'sys/hps_io.sv [SECTOR, %s]: preprocessor error %s%s' %
                 (label, e1, e2)):
        continue
    a_, b_ = uio_block(mine), uio_block(theirs)
    if not check(a_ is not None and b_ is not None, 'sys/hps_io.sv [SECTOR, %s]: no uio_block' % label):
        continue
    if a_ != b_:
        n, x, y = first_diff(a_, b_)
        check(False, 'sys/hps_io.sv [SECTOR, %s]: uio_block differs at token %d: ... %s ... upstream: ... %s ...' %
              (label, n, x, y))
    else:
        check(True, '')

print('-- hps_io.sv with MISTER_DISABLE_SPI_PAD: the uio_block without it less exactly the link\'s eight pad arms')
PADARMS = ["'h02", "'h03", "'h10", "'h11", "'h12", "'h13", "'h1a", "'h3d"]
def labels(t):
    # the case labels of uio_block's casex: a label token followed by ':' after a ';', 'end', 'begin' or 'endcase'
    return [t[i] for i in range(1, len(t) - 1) if re.match(r"'h[0-9a-fA-FX]+$", t[i]) and t[i + 1] == ':' and
            t[i - 1] in (';', 'end', 'begin', 'endcase')]
for label, defs in sets:
    lwd = ['MISTER_LW', 'MISTER_LW_PAD']
    mine, e1 = pp('sys/hps_io.sv', lwd + ['MISTER_DISABLE_SPI_PAD'] + defs)
    base, e2 = pp('sys/hps_io.sv', lwd + defs)
    if not check(mine is not None and base is not None, 'sys/hps_io.sv [SPI_PAD, %s]: preprocessor error %s%s' %
                 (label, e1, e2)):
        continue
    a_, b_ = uio_block(mine), uio_block(base)
    if not check(a_ is not None and b_ is not None, 'sys/hps_io.sv [SPI_PAD, %s]: no uio_block' % label):
        continue
    ops = difflib.SequenceMatcher(None, b_, a_, autojunk=False).get_opcodes()
    gone = sum((b_[i1:i2] for tg, i1, i2, _, _ in ops if tg == 'delete'), [])
    other = [tg for tg, _, _, _, _ in ops if tg not in ('equal', 'delete')]
    check(not other and sorted(labels(['end'] + gone)) == sorted(PADARMS) and
          gone.count('begin') == gone.count('end'),
          'sys/hps_io.sv [SPI_PAD, %s]: uio_block is not the LW build\'s less the arms %s: removed %s, other edits %s'
          % (label, ' '.join(PADARMS), ' '.join(labels(['end'] + gone)), other))

lw = sorted(x for x in os.listdir('sys') if re.match(r'(hps_lw_|lw_|hps_ocram_)\w*\.s?v$', x))
OCM = 'hps_ocram_master.sv'
print('-- every framework-only file is empty without its floor macro (%s)' % ' '.join(lw))
for x in lw:
    f = 'sys/' + x
    if x == 'lw_cdc.sv':
        # the cells serve either floor: empty with neither, present with either
        for label, defs in sets:
            t, e = pp(f, defs)
            if check(t is not None, '%s [%s]: preprocessor error %s' % (f, label, e)):
                check(t == [], '%s [%s]: %d tokens with neither floor, from: %s'
                      % (f, label, len(t or []), ' '.join((t or [])[:12])))
        for label in ('MISTER_LW', 'MISTER_OCRAM'):
            t, e = pp(f, [label])
            check(t is not None and len(t) > 0, '%s [%s]: empty with a floor macro' % (f, label))
        continue
    if x == OCM:
        for label, defs in sets:
            t, e = pp(f, defs)
            if check(t is not None, '%s [%s]: preprocessor error %s' % (f, label, e)):
                check(t == [], '%s [%s]: %d tokens without MISTER_OCRAM, from: %s'
                      % (f, label, len(t or []), ' '.join((t or [])[:12])))
        t, e = pp(f, ['MISTER_OCRAM'])
        check(t is not None and len(t) > 0, '%s: empty with MISTER_OCRAM' % f)
        continue
    runs = [(label, defs) for label, defs in sets]
    if x in EITHER:
        fl, tm, _ = EITHER[x]
        runs += [(fl + ' without ' + ' or '.join(tm) + ', ' + label, [fl] + defs) for label, defs in sets]
        for m in tm:
            t, e = pp(f, [fl, m + '=1'])
            check(t is not None and len(t) > 0, '%s: empty with %s and %s' % (f, fl, m))
    if x in TIER:
        runs += [('MISTER_LW without ' + TIER[x] + ', ' + label, ['MISTER_LW'] + defs) for label, defs in sets]
    if x in OTIER:
        runs += [('MISTER_OCRAM without ' + OTIER[x] + ', ' + label, ['MISTER_OCRAM'] + defs)
                 for label, defs in sets]
    for label, defs in runs:
        t, e = pp(f, defs)
        if check(t is not None, '%s [%s]: preprocessor error %s' % (f, label, e)):
            check(t == [], '%s [%s]: %d tokens outside the guard, from: %s' % (f, label, len(t or []),
                                                                             ' '.join((t or [])[:12])))

print('-- every framework-only file keeps its body in its guard and leaves no macro behind')
for x in lw:
    f = 'sys/' + x
    t = tokens(open(f).read())
    if x == 'lw_cdc.sv':
        head = ['`ifdef', 'MISTER_LW', '`define', 'LW_CDC_BUILT', '`endif',
                '`ifdef', 'MISTER_OCRAM', '`define', 'LW_CDC_BUILT', '`endif',
                '`ifdef', 'LW_CDC_BUILT', '`default_nettype', 'none']
        tail = ['`default_nettype', 'wire', '`endif', '`undef', 'LW_CDC_BUILT']
        scan = False                       # the prologue's ifdef/endif pairs are not one guard
    elif x == OCM:
        head = ['`ifdef', 'MISTER_OCRAM', '`default_nettype', 'none']
        tail = ['`default_nettype', 'wire', '`endif', '`undef', 'MISTER_OCRAM_MARK',
                '`undef', 'MISTER_OCRAM_POLL']
        scan = True
    elif x in EITHER:
        fl, tm, lm = EITHER[x]
        head = ['`ifdef', fl] + sum([['`ifdef', m, '`define', lm, '`endif'] for m in tm], []) + \
               ['`ifdef', lm, '`default_nettype', 'none']
        tail = ['`default_nettype', 'wire', '`endif', '`undef', lm, '`endif']
        scan = False
    elif x in OTIER:
        head = ['`ifdef', 'MISTER_OCRAM', '`ifdef', OTIER[x], '`default_nettype', 'none']
        tail = ['`default_nettype', 'wire', '`endif', '`endif']
        scan = True
    else:
        head = ['`ifdef', 'MISTER_LW'] + (['`ifdef', TIER[x]] if x in TIER else []) + ['`default_nettype', 'none']
        tail = ['`default_nettype', 'wire'] + (['`endif'] if x in TIER else []) + ['`endif']
        scan = True
    check(t[:len(head)] == head, '%s: does not open with %s' % (f, ' '.join(head)))
    check(t[-len(tail):] == tail, '%s: does not close with %s' % (f, ' '.join(tail)))
    if scan:
        depth, closed_at = 0, None
        for i, k in enumerate(t):
            if k in ('`ifdef', '`ifndef'):
                depth += 1
            elif k == '`endif':
                depth -= 1
                if depth == 0 and closed_at is None:
                    closed_at = i
        close_at = len(t) - len(tail) + max(i for i, x in enumerate(tail) if x == '`endif')
        check(closed_at == close_at, '%s: the guard closes at token %s, not %d (of %d)' %
              (f, closed_at, close_at, len(t)))
    live = {}
    for i, k in enumerate(t):
        if k in ('`define', '`undef') and i + 1 < len(t):
            m = t[i + 1]
            if k == '`define':
                okname = m.startswith('LW_') or (x == OCM and m.startswith('MISTER_OCRAM_'))
                check(okname, '%s: defines %s, a name outside LW_' % (f, m))
                check(m != 'LW_DIAG', '%s: defines LW_DIAG, which only a build sets' % f)
                live[m] = True
            else:
                check(m in live, '%s: undefines %s, a macro it did not define' % (f, m))
                live.pop(m, None)
    check(not live, '%s: leaves %s defined' % (f, ' '.join(sorted(live))))

print('-- every other file is upstream\'s')
def tree(root):
    r = set()
    for d, _, fs in os.walk(os.path.join(root, 'sys')):
        for x in fs:
            r.add(os.path.relpath(os.path.join(d, x), root))
    return r
mine, theirs = tree('.'), tree(UP)
lwonly = lambda p: re.match(r'sys/(hps_lw_|lw_|hps_ocram_)[^/]*$', p) is not None
for p in sorted(mine - theirs):
    check(lwonly(p), '%s: not in upstream and not an LW-only file' % p)
for p in sorted(theirs - mine):
    check(False, '%s: in upstream, missing here' % p)
for p in sorted(mine & theirs) + ['files.qip']:
    if p in STOCK or p == 'sys/sys.qip':
        continue
    if p in PINNED:
        check(hashlib.sha256(open(p, 'rb').read()).hexdigest() == PINNED[p], '%s: differs from its pinned text' % p)
        continue
    check(open(p, 'rb').read() == open(os.path.join(UP, p), 'rb').read(), '%s: differs from upstream' % p)
qip = open('sys/sys.qip').read().splitlines()
add = re.compile(r'set_global_assignment -name SYSTEMVERILOG_FILE \[file join \$::quartus\(qip_path\) '
                 r'(hps_lw_|lw_|hps_ocram_)\w+\.sv \]$')
sdc = re.compile(r'set_global_assignment -name SDC_FILE +\[file join \$::quartus\(qip_path\) lw_bridge\.sdc \]$')
rest = [l for l in qip if not add.match(l) and not sdc.match(l)]
check(rest == open(os.path.join(UP, 'sys/sys.qip')).read().splitlines(),
      'sys/sys.qip: differs from upstream beyond the LW-only file lines')
at = [i for i, l in enumerate(qip) if sdc.match(l)]
check(len(at) == 1 and at[0] > 0 and re.search(r'\bsys_top\.sdc \]$', qip[at[0] - 1]) is not None,
      'sys/sys.qip: %d lw_bridge.sdc lines, not one right after sys_top.sdc\'s' % len(at))
print('-- lw_bridge.sdc constrains nothing without MISTER_LW')
GUARD_Q = 'set lw_s0 [get_keepers -nowarn {*|lws0_*}]'
GUARD_IF = 'if {[get_collection_size $lw_s0] > 0} {'
sd = [l.strip() for l in open('sys/lw_bridge.sdc').read().split('\n')] if os.path.exists('sys/lw_bridge.sdc') else []
sd = [l for l in sd if l and not l.startswith('#')]
depth, out = 0, []   # out: the commands after the guard's first close
for n, l in enumerate(sd):
    if n > 1 and depth == 0: out.append(l)
    depth += l.count('{') - l.count('}')
check(sd[:2] == [GUARD_Q, GUARD_IF] and not out and depth == 0 and sd[-1] == '}',
      'sys/lw_bridge.sdc: not one query of the first stages, then every other command inside %s: %s' %
      (GUARD_IF, ' / '.join(out) or ' / '.join(sd[:2]) or 'no file'))
check(re.search(r'lws0_', open('sys/lw_cdc.sv').read()) is not None and
      not any(re.search(r'lws0_', open(os.path.join('sys', x), errors='replace').read())
              for x in os.listdir('sys') if x != 'lw_cdc.sv' and re.search(r'\.(s?v|vh?|vhd)$', x)),
      'sys/lw_bridge.sdc\'s guard: lws0_ is named outside sys/lw_cdc.sv, or not in it')
asg = lambda p: [l for l in open(p).read().splitlines() if l.strip() and not l.lstrip().startswith('#')]
print('-- Template.qsf: upstream\'s assignments, its lines byte for byte and its line ends; only comment lines added')
check(asg('Template.qsf') == asg(os.path.join(UP, 'Template.qsf')), 'Template.qsf: an assignment differs from upstream')
qm = open('Template.qsf', 'rb').read().splitlines(keepends=True)
qu = open(os.path.join(UP, 'Template.qsf'), 'rb').read().splitlines(keepends=True)
ops = difflib.SequenceMatcher(None, qu, qm, autojunk=False).get_opcodes()
lost = sum(i2 - i1 for t, i1, i2, _, _ in ops if t in ('replace', 'delete'))
check(not lost, 'Template.qsf: %d of upstream\'s lines not kept byte for byte, in order' % lost)
added = [qm[j] for t, _, _, j1, j2 in ops if t == 'insert' for j in range(j1, j2)]
check(all(l.lstrip().startswith(b'#') or not l.strip() for l in added),
      'Template.qsf: an added line that is not a comment or blank: %s' %
      next((l.strip() for l in added if not (l.lstrip().startswith(b'#') or not l.strip())), b'').decode(errors='replace'))
eol = lambda ls: sorted({l[len(l.rstrip(b'\r\n')):] for l in ls})
check(eol(qm) == eol(qu), 'Template.qsf: lines end in %s, upstream\'s in %s' % (eol(qm), eol(qu)))

print('-- Template.sv is upstream\'s token stream without MISTER_LW; its CONF_STR value with and without it')
open(os.path.join(O, 'build_id.v'), 'w').write('`define BUILD_DATE "000000"\n')
for label, defs in sets:
    mine, e1 = pp('Template.sv', defs)
    theirs, e2 = pp('Template.sv', defs, cwd=UP)
    if not check(mine is not None and theirs is not None,
                 'Template.sv [%s]: preprocessor error %s%s' % (label, e1, e2)):
        continue
    if mine != theirs:
        n, a, b = first_diff(mine, theirs)
        check(False, 'Template.sv [%s]: token %d differs: ... %s ... upstream: ... %s ...' % (label, n, a, b))
    else:
        check(True, '')

def conf_str(defs, cwd=None):
    """CONF_STR as Icarus evaluates it from emu's localparams, or an error text."""
    t, e = pp('Template.sv', defs, cwd)
    if t is None:
        return None, e
    lp, i = [], 0
    while i < len(t):
        if t[i] == 'localparam':
            j, d = i, 0
            while j < len(t) and not (t[j] == ';' and d == 0):
                d += t[j] in '({['; d -= t[j] in ')}]'; j += 1
            lp.append(' '.join(t[i:j + 1])); i = j
        i += 1
    m = os.path.abspath(os.path.join(O, 'offstate_cs.v'))
    open(m, 'w').write('module offstate_cs;\n%s\ninitial $display("%%s", CONF_STR);\nendmodule\n' % '\n'.join(lp))
    r = subprocess.run(['iverilog', '-g2012', '-o', m + 'vp', m], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True)
    if r.returncode:
        return None, r.stdout.strip()
    r = subprocess.run(['vvp', '-n', m + 'vp'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return r.stdout.split('\n')[0], ''

up, e = conf_str([], cwd=UP)
if check(up is not None and up.startswith('Template;;') and up.count('T[0],Reset;') == 1,
         'upstream\'s Template.sv: no CONF_STR value, or not the expected shape: %s%s' % (up, e)):
    lw_up = up.replace('Template;;', 'Template;AXI:40000:80000:LW,;', 1)
    for label, defs, extra in [
            ('no macro', [], ''),
            ('MISTER_LW', ['MISTER_LW'], 'J,Colour;'),
            ('MISTER_LW, PAD, EVT', ['MISTER_LW', 'MISTER_LW_PAD', 'MISTER_LW_EVT'], 'J,Colour;'),
            ('every tier, SECTOR on slot 1', ['MISTER_LW', 'MISTER_LW_PAD', 'MISTER_LW_EVT', 'MISTER_LW_SECTOR_SLOT=1',
                                              'MISTER_LW_LANE', 'MISTER_LW_OSD'],
             'S1,IMG,Block image;J,Colour,Read block;'),
            ('MISTER_LW, SECTOR on slot 3', ['MISTER_LW', 'MISTER_LW_SECTOR_SLOT=3'],
             'S3,IMG,Block image;J,Colour,Read block;'),
            ('MISTER_LW, the OCRAM SECTOR tier on slot 2', ['MISTER_LW', 'MISTER_OCRAM', 'MISTER_OCRAM_SECTOR_SLOT=2'],
             'S2,IMG,Block image;J,Colour,Read block;')]:
        want = up if not extra else lw_up.replace('T[0],Reset;', extra + 'T[0],Reset;', 1)
        got, e = conf_str(defs)
        check(got == want, 'Template.sv [%s]: CONF_STR %s, expected %s %s' % (label, got, want, e))

print('%s: %d checks, %d failures' % ('FAILED' if fails else 'PASSED', checks, fails))
sys.exit(1 if fails else 0)
EOF
