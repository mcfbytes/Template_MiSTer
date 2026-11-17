#!/bin/sh
# Off-state guard, arm S of tb/run.sh: a build without MISTER_LW reads upstream's text. Upstream is the
# framework revision this branch was cut from ($UPSTREAM below). The checks:
#   1. sys_top.v, hps_io.sv, osd.v, sysmem.sv and emu_ports.vh, preprocessed with MISTER_LW and every tier
#      macro undefined, give upstream's token stream (comments and whitespace aside) with no macro, with each
#      macro upstream's own text tests (read from those five files at upstream), alone, and with all of them.
#   2. Every LW-only file (sys/hps_lw_*.sv, sys/lw_*.sv) preprocesses to no token at all without MISTER_LW,
#      under each of those macro sets; a tier's file also with MISTER_LW and without its tier macro.
#   3. Every LW-only file keeps its whole body in its guard: `ifdef MISTER_LW (then its tier macro) first,
#      `default_nettype none first inside it, `default_nettype wire last. Each macro the file defines begins
#      with LW_ and is undefined before the guard closes; it undefines no macro it did not define; it defines
#      no LW_DIAG (a build sets that one).
#   4. Every other file under sys/, and files.qip, is byte-identical to upstream; sys.qip adds only
#      SYSTEMVERILOG_FILE lines for LW-only files and at most one SDC_FILE line for lw_bridge.sdc;
#      Template.qsf's assignments (its comment and blank lines removed) are upstream's.
# Upstream's tree is read with git into $LW_TB_OUT/upstream. Where git cannot reach the repository (the
# container tb/run.sh starts), run "tb/offstate.sh --extract" outside first; tb/run.sh does.
# Output: "-- " headings, a FAIL line per failed check, then PASSED/FAILED with the counts (tb/run.sh's verdict).
UPSTREAM=e901a9f
cd "$(dirname "$0")/.." || exit 2
O=${LW_TB_OUT:-tb/out}; UP="$O/upstream"
if git rev-parse --verify -q "$UPSTREAM^{commit}" >/dev/null 2>&1; then
  if [ "$(cat "$UP/.rev" 2>/dev/null)" != "$UPSTREAM" ]; then
    rm -rf "$UP" && mkdir -p "$UP" && git archive "$UPSTREAM" sys files.qip Template.qsf | tar -x -C "$UP" &&
      echo "$UPSTREAM" > "$UP/.rev" || { echo "tb/offstate.sh: cannot extract $UPSTREAM" >&2; exit 2; }
  fi
fi
[ "$1" = --extract ] && exit 0
if [ "$(cat "$UP/.rev" 2>/dev/null)" != "$UPSTREAM" ]; then
  echo "FAILED: upstream tree $UPSTREAM not available in $UP (run tb/offstate.sh --extract where git works)"; exit 1
fi
command -v iverilog >/dev/null 2>&1 || { echo "FAILED: no iverilog on PATH"; exit 1; }
exec python3 -I - "$UP" "$O" <<'EOF'
import os, re, subprocess, sys
UP, O = sys.argv[1], sys.argv[2]
STOCK = ['sys/sys_top.v', 'sys/hps_io.sv', 'sys/osd.v', 'sys/sysmem.sv', 'sys/emu_ports.vh']
TIER = {'lw_sector.sv': 'MISTER_LW_SECTOR_SLOT', 'lw_blk_mux.sv': 'MISTER_LW_SECTOR_SLOT',
        'lw_lane.sv': 'MISTER_LW_LANE'}
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

def pp(path, defs):
    out = os.path.join(O, 'offstate.pp')
    r = subprocess.run(['iverilog', '-E', '-o', out] + ['-D' + d for d in defs] + [path],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
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

lw = sorted(x for x in os.listdir('sys') if re.match(r'(hps_lw_|lw_)\w*\.s?v$', x))
print('-- every LW-only file is empty without its macro (%s)' % ' '.join(lw))
for x in lw:
    f = 'sys/' + x
    runs = [(label, defs) for label, defs in sets]
    if x in TIER:
        runs += [('MISTER_LW without ' + TIER[x] + ', ' + label, ['MISTER_LW'] + defs) for label, defs in sets]
    for label, defs in runs:
        t, e = pp(f, defs)
        if check(t is not None, '%s [%s]: preprocessor error %s' % (f, label, e)):
            check(t == [], '%s [%s]: %d tokens outside the guard, from: %s' % (f, label, len(t or []),
                                                                             ' '.join((t or [])[:12])))

print('-- every LW-only file keeps its body in its guard and leaves no macro behind')
for x in lw:
    f = 'sys/' + x
    t = tokens(open(f).read())
    head = ['`ifdef', 'MISTER_LW'] + (['`ifdef', TIER[x]] if x in TIER else []) + ['`default_nettype', 'none']
    tail = ['`default_nettype', 'wire'] + (['`endif'] if x in TIER else []) + ['`endif']
    check(t[:len(head)] == head, '%s: does not open with %s' % (f, ' '.join(head)))
    check(t[-len(tail):] == tail, '%s: does not close with %s' % (f, ' '.join(tail)))
    depth, closed_at = 0, None
    for i, k in enumerate(t):
        if k in ('`ifdef', '`ifndef'):
            depth += 1
        elif k == '`endif':
            depth -= 1
            if depth == 0 and closed_at is None:
                closed_at = i
    check(closed_at == len(t) - 1, '%s: the guard closes before the end of the file (token %s of %d)' %
          (f, closed_at, len(t)))
    live = {}
    for i, k in enumerate(t):
        if k in ('`define', '`undef') and i + 1 < len(t):
            m = t[i + 1]
            if k == '`define':
                check(m.startswith('LW_'), '%s: defines %s, a name outside LW_' % (f, m))
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
lwonly = lambda p: re.match(r'sys/(hps_lw_|lw_)[^/]*$', p) is not None
for p in sorted(mine - theirs):
    check(lwonly(p), '%s: not in upstream and not an LW-only file' % p)
for p in sorted(theirs - mine):
    check(False, '%s: in upstream, missing here' % p)
for p in sorted(mine & theirs) + ['files.qip']:
    if p in STOCK or p == 'sys/sys.qip':
        continue
    check(open(p, 'rb').read() == open(os.path.join(UP, p), 'rb').read(), '%s: differs from upstream' % p)
qip = open('sys/sys.qip').read().splitlines()
add = re.compile(r'set_global_assignment -name SYSTEMVERILOG_FILE \[file join \$::quartus\(qip_path\) '
                 r'(hps_lw_|lw_)\w+\.sv \]$')
sdc = re.compile(r'set_global_assignment -name SDC_FILE +\[file join \$::quartus\(qip_path\) lw_bridge\.sdc \]$')
rest = [l for l in qip if not add.match(l) and not sdc.match(l)]
check(rest == open(os.path.join(UP, 'sys/sys.qip')).read().splitlines(),
      'sys/sys.qip: differs from upstream beyond the LW-only file lines')
check(sum(1 for l in qip if sdc.match(l)) <= 1, 'sys/sys.qip: more than one lw_bridge.sdc line')
asg = lambda p: [l for l in open(p).read().splitlines() if l.strip() and not l.lstrip().startswith('#')]
check(asg('Template.qsf') == asg(os.path.join(UP, 'Template.qsf')), 'Template.qsf: an assignment differs from upstream')

print('%s: %d checks, %d failures' % ('FAILED' if fails else 'PASSED', checks, fails))
sys.exit(1 if fails else 0)
EOF
