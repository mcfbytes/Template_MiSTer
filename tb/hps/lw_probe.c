/*
 * lw_probe: an HPS-side program that walks the LW bridge protocol of docs/LW_BRIDGE.md, step by step, and prints
 * each step. It is the rig's vehicle for the framework, independent of any particular host program, and a
 * reference a host author can read: it speaks only what the guide says.
 *
 *   lw_probe [--template] [--die]
 *
 * Run it as root on the DE10-Nano with a core loaded and NO other program on the link: it identifies the core,
 * which takes the link (and clears any host enable). Stop the host program first. No core may be loaded while
 * it runs. It needs no interrupt node: it polls PENDING.
 *
 *   1. The look. It reads the FPGA manager's GPO register itself, before any GPO store; with bit 31 set, one GPI
 *      sample: GPI[31] = 0, GPI[25] = 1 and GPI[22] = 1 mean another LW program has the window, and it stops
 *      there, having stored nothing.
 *   2. It identifies the core (the ID view), then discovers: one GPI sample in the I/O view with GPI[31] = 0,
 *      GPI[25] = 1 and GPI[24] = 1, the bridge out of reset, the CONF_STR token AXI:<base>:<size>:LW, then the
 *      directory's MAGIC, VERSION, WIN_BASE and WIN_SIZE. Any failure: it never touches the bridge.
 *   3. It walks the directory by its rules and prints every entry and RESETS.
 *   4. It reads ENABLE until quiet, stores the key, requires en, and reads OPEN until the bits of the linked
 *      entries it serves are set.
 *   5. One GET_SDSTAT over the link, and what it shows is served there (with --template, a read on the block
 *      reader's slot) before any SECTOR EN store; then it binds, re-stores every pad level and stores LINE_EN.
 *   6. The blocks: PAD's ID and drops; EVT drained and its status read in a bracket; an OSD image stored with
 *      the drops compared around it and the FIFO drained; two LANE capture frames (0x29 and 0x14) drained by
 *      the cause convention and their replies compared with EVT's status and with the CONF_STR read over the
 *      link.
 *   7. With --template (the Template core's LW example): the first pad button moves EVT's status; the second
 *      makes the block reader request blocks, which the probe serves over SECTOR with a pattern, and the
 *      reader's {LBA, sum} write-back through EVT must match the pattern's sum.
 *   8. The end: it identifies the core again, which clears the enable, and requires every block back at rest
 *      (quiet, OWNED 0, OPEN 0, LINE_EN 0, GPI[22] 0): the core is on stock SPI for whatever program comes next.
 *
 * --die (test only) replaces step 8: it begins a link command with the I/O select asserted through GPO and queues
 * lane capture frames behind it; then, as a host does to repaint the OSD, it raises the OSD select in place of the
 * I/O select (one GPO store, so the link is never quiet in between: both osd selects decode high) and stores OSD
 * bytes, which wait behind that select; and exits with the select asserted and the enable set, to stand in for a
 * host that died in the middle of a repaint. The next program's identification must find every block off, and
 * none of those lane frames or OSD bytes played or written.
 *
 * Every LW access is preceded by a GPI sample that must show GPI[31] = 0 and GPI[25] = 1; a failed sample ends
 * the run with no further LW access. Exit status: 0 every check passed; 1 a check failed; 2 the probe could not
 * run (usage, /dev/mem); 3 it stopped before touching the bridge (the look, or discovery found no window).
 *
 * tb/run.sh (arm H) compiles it with warnings as errors, and runs tb/hps/lw_probe_test.c, which includes it, on
 * the build host against a model of the window; the testbench never runs it against the RTL.
 */
#define _POSIX_C_SOURCE 200809L
#define _FILE_OFFSET_BITS 64   /* the physical offsets are above 2 GB: a 32-bit off_t cannot carry them */

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

/* ---- the HPS side (Cyclone V) ---- */
#define FPGAMGR_BASE 0xFF706000u   /* the FPGA manager: GPO at +0x10, GPI at +0x14 */
#define RSTMGR_BASE  0xFFD05000u   /* the reset manager: brgmodrst at +0x1C, bit 1 the lightweight bridge */
#define LWB_BASE     0xFF200000u   /* the lightweight HPS-to-FPGA bridge, 2 MB */
#define LWB_SIZE     0x00200000u
#define PAGE         0x1000u

#define GPO_IO_VIEW  (1u << 31)    /* 0 = the ID view: GPI shows the core magic */
#define GPO_IO_EN    (1u << 20)    /* the link selects: UIO, OSD, FPGA (FIO) */
#define GPO_OSD_EN   (1u << 19)
#define GPO_FPGA_EN  (1u << 18)
#define GPO_STROBE   (1u << 17)
#define GPI_UNCONF   (1u << 31)    /* an unconfigured FPGA reads all ones */
#define GPI_LW       (1u << 25)    /* the firewall is in this bitstream */
#define GPI_IDLE     (1u << 24)    /* the firewall holds no transaction */
#define GPI_EN       (1u << 22)    /* the window's host enable */
#define GPI_ACK      (1u << 17)
#define GPI_WIDE     (1u << 16)    /* hps_io's io_wide: 16-bit block transfers */
#define CORE_MAGIC   0x5CA623u

/* ---- the window: the directory header ---- */
#define H_MAGIC   0x00u
#define H_VERSION 0x04u
#define H_LAYOUT  0x08u
#define H_COUNT   0x0Cu
#define H_WBASE   0x10u
#define H_WSIZE   0x14u
#define H_ENABLE  0x18u
#define H_PENDING 0x20u
#define H_OWNED   0x24u
#define H_OPEN    0x2Cu
#define H_LINE_EN 0x30u
#define H_RESETS  0x34u
#define DIR_MAGIC 0x5244574Cu      /* "LWDR" */
#define DIR_MAJOR 2u
#define ENABLE_KEY 0x4C570001u
#define E_EN      (1u << 31)
#define E_QUIET   (1u << 30)
#define E_STALE   (1u << 29)
#define E_BURST   (1u << 28)
#define E_HRST    (1u << 15)
#define E_IDV     (1u << 11)

/* the framework kinds this probe knows, with the register-map major it speaks */
enum { K_PAD = 1, K_SECTOR = 2, K_OSD = 3, K_EVT = 4, K_LANE = 5, K_LAST = 5 };
static const uint32_t kind_major[K_LAST + 1] = { 0, 1, 2, 2, 2, 1 };
static const char *const kind_name[K_LAST + 1] = { "", "PAD", "SECTOR", "OSDBUF", "EVT", "LANE" };

/* block registers, block-relative */
#define PAD_STAT   0x0000u
#define PAD_ID     0x0004u
#define EVT_CAUSE  0x00u
#define EVT_STFLG  0x04u
#define EVT_STATUS 0x08u
#define EVT_PEND   0x18u
#define EVT_IDW    0x1Cu
#define OSD_STATUS 0x1800u
#define OSD_IDW    0x1804u
#define SEC_CAUSE  0x000u
#define SEC_REQ    0x004u
#define SEC_CTRL   0x008u
#define SEC_ACCEPT 0x010u
#define SEC_IDW    0x014u
#define SEC_RSTAT(k) (0x100u + 0x20u * (k))
#define SEC_LBA(k)   (0x104u + 0x20u * (k))
#define SEC_BEGIN(k) (0x108u + 0x20u * (k))
#define SEC_DATA   0x4000u
#define LN_BEGIN   0x00u
#define LN_WORD    0x04u
#define LN_END     0x08u
#define LN_RPOP    0x0Cu
#define LN_PLAYED  0x10u
#define LN_DROPS   0x14u
#define LN_IDW     0x18u
#define LN_RSTAT   0x1Cu
#define LN_RHEAD   0x20u
#define LN_CAUSE   0x24u
#define LN_CAPTURE (1u << 18)

/* link opcodes (UIO, sel 0) */
#define UIO_JOY0       0x02u
#define UIO_GET_STRING 0x14u
#define UIO_GET_SDSTAT 0x16u
#define UIO_SECTOR_RD  0x17u
#define UIO_GET_STATUS 0x29u

#define T_LINK_MS  100u    /* a link word's acknowledge */
#define T_QUIET_MS 1000u   /* the enable links' rest, OPEN, a FIFO's drain */
#define T_EVENT_MS 1000u   /* a cause after the event that raises it */

static volatile uint32_t *mgr, *rst, *win;
static uint32_t gpo;             /* the GPO value this program last stored */
static uint32_t win_base, win_size;
static unsigned fails;
static unsigned en_disagree;     /* reads of ENABLE whose en differed from GPI[22], running */
static char cfg[10240];

struct entry {
	int      present;
	unsigned idx;                  /* its PENDING, OPEN, OWNED and LINE_EN bit */
	uint32_t ver, off, size, cause, line, bits;
	uint32_t k[24];
};
static struct entry blk[K_LAST + 1];

/* ---- output and time ---- */

static void say(const char *fmt, ...)
{
	va_list a;
	va_start(a, fmt);
	vprintf(fmt, a);
	va_end(a);
	putchar('\n');
	fflush(stdout);
}

static int check(int ok, const char *fmt, ...)
{
	va_list a;
	fputs(ok ? "pass " : "FAIL ", stdout);
	va_start(a, fmt);
	vprintf(fmt, a);
	va_end(a);
	putchar('\n');
	fflush(stdout);
	if (!ok) fails++;
	return ok;
}

static void step(const char *what)
{
	say("-- %s", what);
}

static uint64_t now_us(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
}

static void sleep_ms(unsigned ms)
{
	struct timespec t = { (time_t)(ms / 1000u), (long)(ms % 1000u) * 1000000L };
	nanosleep(&t, NULL);
}

static void *map(int fd, uint32_t phys, uint32_t len)
{
	void *p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)phys);
	if (p == MAP_FAILED) {
		fprintf(stderr, "lw_probe: mmap of 0x%08x: %s\n", (unsigned)phys, strerror(errno));
		exit(2);
	}
	return p;
}

/* ---- GPO, GPI and the link ---- */

static uint32_t gpi(void)
{
	return mgr[0x14 / 4];
}

static void gpo_w(uint32_t v)
{
	gpo = v;
	mgr[0x10 / 4] = v;
}

/* one link word: strobe, wait for the acknowledge, drop the strobe, wait for it to fall; the reply is GPI[15:0] */
static uint16_t link_w(uint16_t w)
{
	uint32_t v = (gpo & ~(0xFFFFu | GPO_STROBE)) | w, g;
	uint64_t t;
	gpo_w(v);
	gpo_w(v | GPO_STROBE);
	for (t = now_us(); !((g = gpi()) & GPI_ACK); )
		if ((g & GPI_UNCONF) || now_us() - t > T_LINK_MS * 1000u) goto stuck;
	gpo_w(v);
	for (t = now_us(); (g = gpi()) & GPI_ACK; )
		if ((g & GPI_UNCONF) || now_us() - t > T_LINK_MS * 1000u) goto stuck;
	return (uint16_t)g;
stuck:
	check(0, "the link acknowledges word 0x%04x within %u ms (GPI 0x%08x): stopping", w, T_LINK_MS, (unsigned)g);
	exit(1);
}

static void sel(uint32_t mask, int on)
{
	gpo_w(on ? (gpo | GPO_IO_VIEW | mask) : ((gpo | GPO_IO_VIEW) & ~mask));
}

/* the ID view and back, as every program on this link does at its start; it clears the host enable */
static uint32_t identify(void)
{
	uint32_t g;
	gpo_w(gpo & ~GPO_IO_VIEW);
	g = gpi();
	gpo_w(gpo | GPO_IO_VIEW);
	return g;
}

/* ---- the window: every access after a GPI sample in the I/O view ---- */

static void lw_guard(void)
{
	uint32_t g = gpi();
	if ((g & GPI_UNCONF) || !(g & GPI_LW)) {
		check(0, "GPI 0x%08x before an LW access shows GPI[31] = 0 and GPI[25] = 1: no further LW access",
		      (unsigned)g);
		exit(1);
	}
}

static uint32_t rd(uint32_t off)
{
	lw_guard();
	return win[off >> 2];
}

/* a single-beat full-word store; the mapping is strongly ordered, so it is in effect before the next access */
static void wr(uint32_t off, uint32_t v)
{
	lw_guard();
	win[off >> 2] = v;
}

static uint32_t brd(int kind, uint32_t reg)
{
	return rd(blk[kind].off + reg);
}

static void bwr(int kind, uint32_t reg, uint32_t v)
{
	wr(blk[kind].off + reg, v);
}

/* ENABLE, with GPI[22] beside it: the two show one register, and may differ only inside a change */
static uint32_t read_enable(const char *when)
{
	uint32_t e = rd(H_ENABLE), g = gpi();
	unsigned en = e >> 31, held = (g >> 22) & 1u;
	say("ENABLE 0x%08x (en %u quiet %u stale %u burst %u hrst %u idv %u), GPI[22] %u, %s", (unsigned)e, en,
	    (unsigned)(e >> 30) & 1u, (unsigned)(e >> 29) & 1u, (unsigned)(e >> 28) & 1u, (unsigned)(e >> 15) & 1u,
	    (unsigned)(e >> 11) & 1u, held, when);
	en_disagree = en != held ? en_disagree + 1u : 0u;
	if (en_disagree >= 2u) check(0, "ENABLE's en and GPI[22] disagree twice running");
	return e;
}

static int wait_bits(uint32_t reg, uint32_t mask, uint32_t want, unsigned ms, uint32_t *last)
{
	uint64_t t = now_us();
	uint32_t v;
	while (((v = rd(reg)) & mask) != want)
		if (now_us() - t > ms * 1000u) { *last = v; return 0; }
	*last = v;
	return 1;
}

static int pending(int kind, unsigned ms)
{
	uint32_t v;
	return wait_bits(H_PENDING, 1u << blk[kind].idx, 1u << blk[kind].idx, ms, &v);
}

/* ---- the start ---- */

static void look(void)
{
	uint32_t g0 = mgr[0x10 / 4], g;
	step("the look: the GPO register itself, then GPI, before any GPO store");
	say("GPO register 0x%08x", (unsigned)g0);
	if (!(g0 & GPO_IO_VIEW)) {
		say("GPO bit 31 is 0: the ID view, so no host enable is held");
		return;
	}
	g = gpi();
	say("GPI 0x%08x: GPI[31] %u, GPI[25] %u, GPI[22] %u", (unsigned)g, (unsigned)(g >> 31), (unsigned)(g >> 25) & 1u,
	    (unsigned)(g >> 22) & 1u);
	if (!(g & GPI_UNCONF) && (g & GPI_LW) && (g & GPI_EN)) {
		say("another LW program has the window: this probe stores nothing and stops");
		exit(3);
	}
}

static void read_confstr(void)
{
	size_t n = 0;
	sel(GPO_IO_EN, 1);
	link_w(UIO_GET_STRING);
	while (n < sizeof cfg - 1) {
		char c = (char)(link_w(0) & 0xFFu);
		if (!c) break;
		cfg[n++] = c;
	}
	cfg[n] = 0;
	sel(GPO_IO_EN, 0);
}

/* field 1 of CONF_STR holds the window token, AXI:<base>:<size>:LW, in hex, among the field's comma items */
static int token(uint32_t *base, uint32_t *size)
{
	const char *f = strchr(cfg, ';'), *end, *p;
	if (!f) return 0;
	f++;
	end = strchr(f, ';');
	if (!end) end = f + strlen(f);
	for (p = f; p < end; ) {
		const char *q = memchr(p, ',', (size_t)(end - p));
		char item[64], *e1, *e2;
		size_t len;
		unsigned long b, s;
		if (!q) q = end;
		len = (size_t)(q - p);
		if (len < sizeof item) {
			memcpy(item, p, len);
			item[len] = 0;
			if (!strncmp(item, "AXI:", 4)) {
				errno = 0;
				b = strtoul(item + 4, &e1, 16);
				if (!errno && e1 != item + 4 && *e1 == ':') {
					s = strtoul(e1 + 1, &e2, 16);
					if (!errno && e2 != e1 + 1 && !strcmp(e2, ":LW")) {
						*base = (uint32_t)b;
						*size = (uint32_t)s;
						return b <= LWB_SIZE && s <= LWB_SIZE - b && !(b % PAGE) && !(s % PAGE) && s;
					}
				}
			}
		}
		p = q + 1;
	}
	return 0;
}

static void discover(int fd)
{
	uint32_t g, b, s, v;
	step("discovery: one GPI sample in the I/O view, the bridge out of reset, the token, the directory");
	g = gpi();
	say("GPI 0x%08x", (unsigned)g);
	if ((g & GPI_UNCONF) || !(g & GPI_LW) || !(g & GPI_IDLE)) {
		if (!(g & GPI_UNCONF) && (g & GPI_LW))
			say("GPI[24] is 0 with no access of this program in flight: the firewall holds a transaction nobody "
			    "will finish. Stay on SPI until the core is loaded again");
		else
			say("no LW firewall in this bitstream (or the FPGA is not configured): stay on SPI");
		exit(3);
	}
	v = rst[0x1C / 4];
	say("brgmodrst 0x%08x", (unsigned)v);
	if (v & 2u) {
		say("the lightweight bridge is held in reset: stay on SPI");
		exit(3);
	}
	read_confstr();
	say("CONF_STR (%u bytes): %.120s%s", (unsigned)strlen(cfg), cfg, strlen(cfg) > 120 ? "..." : "");
	if (!token(&b, &s)) {
		say("CONF_STR field 1 declares no usable AXI:<base>:<size>:LW window: stay on SPI");
		exit(3);
	}
	say("token: window at LW 0x%05x, 0x%05x bytes", (unsigned)b, (unsigned)s);
	win = map(fd, LWB_BASE + b, s);
	win_base = b;
	win_size = s;
	v = rd(H_MAGIC);
	if (v != DIR_MAGIC) { say("MAGIC 0x%08x is not LWDR: stay on SPI, no further LW access", (unsigned)v); exit(3); }
	v = rd(H_VERSION);
	say("VERSION 0x%08x", (unsigned)v);
	if (v >> 16 != DIR_MAJOR) { say("directory major %u is not %u: stay on SPI", (unsigned)(v >> 16), DIR_MAJOR); exit(3); }
	if (rd(H_WBASE) != b || rd(H_WSIZE) != s) {
		say("WIN_BASE 0x%08x / WIN_SIZE 0x%08x differ from the token: stay on SPI", (unsigned)rd(H_WBASE),
		    (unsigned)rd(H_WSIZE));
		exit(3);
	}
	check(1, "the directory is a layout this probe knows, and the window is the token's");
}

/* the walk: LAYOUT's lengths, never constants; skip what is not known; an ordinal that two entries name (either of
 * them skipped or not) or that lies outside 1-15 gets no line: neither entry has it */
static void walk(void)
{
	uint32_t lay = rd(H_LAYOUT), hl = lay & 0xFFFFu, st = lay >> 16, n = rd(H_COUNT), i, j, once = 0, dup = 0;
	step("the directory, walked by its rules");
	say("LAYOUT header %u bytes, stride %u; COUNT %u", (unsigned)hl, (unsigned)st, (unsigned)n);
	if (hl < 0x40u || st < 0x20u || hl % 4u || st % 4u || n > 31u || hl + n * st > PAGE) {
		check(0, "the table fits the directory's page with a header and entries this probe can read");
		return;
	}
	for (i = 0; i < n; i++) {
		uint32_t e = hl + i * st, line = rd(e + 20) & 0xFFu;
		if (!(rd(e) & 0xFFFFu) || !line || line > 31u) continue;
		if (once & (1u << line)) dup |= 1u << line;
		once |= 1u << line;
	}
	for (i = 0; i < n; i++) {
		uint32_t e = hl + i * st, kind = rd(e) & 0xFFFFu, ver = rd(e + 4), off = rd(e + 8), size = rd(e + 12);
		uint32_t cause = rd(e + 16), line = rd(e + 20) & 0xFFu, bits = rd(e + 24), tag = rd(e + 28);
		say("entry %u: kind %u ver 0x%08x offset 0x%05x size 0x%05x cause 0x%05x line %u bits 0x%x tag 0x%08x",
		    (unsigned)i, (unsigned)kind, (unsigned)ver, (unsigned)off, (unsigned)size, (unsigned)cause,
		    (unsigned)line, (unsigned)bits, (unsigned)tag);
		if (!kind) continue;
		if (kind > K_LAST) { say("  skipped: %s", kind >= 0x8000u ? "a core's own kind" : "a kind this probe does not know"); continue; }
		if (ver >> 16 != kind_major[kind]) { say("  skipped: %s major %u", kind_name[kind], (unsigned)(ver >> 16)); continue; }
		if (!size || off >= win_size || size > win_size - off || off % 4u ||
		    (cause && (cause < off || cause - off >= size || cause % 4u))) {
			say("  skipped: OFFSET, SIZE or CAUSE outside the window or the block");
			continue;
		}
		if (blk[kind].present) { say("  skipped: a second %s entry", kind_name[kind]); continue; }
		blk[kind].present = 1;
		blk[kind].idx = i;
		blk[kind].ver = ver;
		blk[kind].off = off;
		blk[kind].size = size;
		blk[kind].cause = cause;
		blk[kind].line = line;
		blk[kind].bits = bits;
		for (j = 0; j < 24; j++) blk[kind].k[j] = rd(e + 32 + 4 * j);
	}
	for (i = 1; i <= K_LAST; i++)
		if (blk[i].present && blk[i].line && (blk[i].line > 15u || (dup & (1u << blk[i].line)))) {
			say("%s: line %u is shared or out of range: no line", kind_name[i], (unsigned)blk[i].line);
			blk[i].line = 0;
		}
	for (i = 1; i <= K_LAST; i++)
		if (blk[i].present) say("%s at entry %u", kind_name[i], blk[i].idx);
	i = rd(H_RESETS);
	say("RESETS 0x%08x: HPS resets seen %u, state machines freed %u", (unsigned)i, (unsigned)(i & 0xFFu),
	    (unsigned)(i >> 8) & 0xFFu);
}

/* ---- the SECTOR pattern: word i of a frame at lba; the same bytes whichever path carries them ---- */

static uint32_t pattern(uint32_t lba, uint32_t i)
{
	return (lba * 0x01000193u) ^ (i * 0x00010001u + 0x00010000u);
}

static uint16_t pattern_sum(uint32_t lba, uint32_t words)
{
	uint32_t i, s = 0;
	for (i = 0; i < words; i++) s += (pattern(lba, i) & 0xFFFFu) + (pattern(lba, i) >> 16);
	return (uint16_t)s;
}

/* one GET_SDSTAT, and a read it shows served over the link when this probe knows the slot (reader_slot >= 0) */
static void sdstat(int reader_slot)
{
	uint16_t c, lo, hi;
	uint32_t slot, op, lba, sz, i;
	step("one GET_SDSTAT, served over the link before any SECTOR EN store");
	sel(GPO_IO_EN, 1);
	c = link_w(UIO_GET_SDSTAT);
	if (!(c & 0x8000u)) {
		sel(GPO_IO_EN, 0);
		say("GET_SDSTAT 0x%04x: not the form this probe reads; no block is served", c);
		return;
	}
	link_w(0);
	lo = link_w(0);
	hi = link_w(0);
	sel(GPO_IO_EN, 0);
	slot = (c >> 2) & 0xFu;
	op = c & 3u;
	lba = (uint32_t)lo | (uint32_t)hi << 16;
	say("GET_SDSTAT 0x%04x: slot %u, %s, LBA %u", c, (unsigned)slot, op & 1u ? "read" : op ? "write" : "no request",
	    (unsigned)lba);
	if (!op) return;
	if (!(op & 1u) || (int)slot != reader_slot) {
		say("not a read on a slot this probe serves: left standing (every SECTOR slot then waits for a GET_SDSTAT "
		    "that shows none)");
		return;
	}
	sz = (128u << ((c >> 6) & 7u)) * (((c >> 9) & 0x3Fu) + 1u);
	sel(GPO_IO_EN, 1);
	link_w((uint16_t)(UIO_SECTOR_RD | slot << 8));
	if (gpi() & GPI_WIDE)
		for (i = 0; i < sz / 2u; i++) link_w((uint16_t)(pattern(lba, i / 2u) >> (16u * (i & 1u))));
	else
		for (i = 0; i < sz; i++) link_w((uint16_t)((pattern(lba, i / 4u) >> (8u * (i & 3u))) & 0xFFu));
	sel(GPO_IO_EN, 0);
	say("served %u bytes over the link", (unsigned)sz);
}

static void enable(uint32_t serve)
{
	uint32_t e, v;
	uint64_t t;
	step("the host enable: quiet, the key, en; then OPEN for the linked entries served");
	e = read_enable("after the identification");
	check(!(e & (E_EN | E_IDV | E_HRST)), "no enable is held, the I/O view and no HPS reset after the identification");
	for (t = now_us(); !((e = rd(H_ENABLE)) & E_QUIET); )
		if (now_us() - t > T_QUIET_MS * 1000u) break;
	if (!(e & E_QUIET)) {
		v = rd(H_OWNED);
		check(0, "quiet within %u ms: OWNED 0x%08x names the block that cannot return; stay on SPI", T_QUIET_MS,
		      (unsigned)v);
		exit(1);
	}
	wr(H_ENABLE, ENABLE_KEY);
	e = read_enable("after the key");
	check((e & E_EN) && (gpi() & GPI_EN), "en and GPI[22] after the key");
	check(!(e & (E_STALE | E_BURST)), "stale and burst clear at the ENABLE");
	check(rd(H_LINE_EN) == 0, "LINE_EN reads 0 after the ENABLE");
	if (!wait_bits(H_OPEN, serve, serve, T_QUIET_MS, &v))
		say("OPEN 0x%08x: the bits 0x%08x not all set within %u ms; those entries stay on SPI", (unsigned)v,
		    (unsigned)(serve & ~v), T_QUIET_MS);
	else
		say("OPEN 0x%08x", (unsigned)v);
}

/* ---- the blocks ---- */

static int pad_takes(uint32_t op)
{
	return blk[K_PAD].present && ((blk[K_PAD].k[op >> 5] >> (op & 31u)) & 1u);
}

static void pad_store(uint32_t op, uint32_t idx, uint32_t v)
{
	bwr(K_PAD, (op << 10) + (idx << 2), v);
}

/* every one of PAD's 24 latest-wins levels: the six digital pads, both sticks of each player and each player's
 * paddle (0x1A at index {p, 15}); never a spinner, whose stores are events */
static void pad_levels(void)
{
	static const uint32_t joy[6] = { 0x02, 0x03, 0x10, 0x11, 0x12, 0x13 };
	uint32_t i;
	if (!blk[K_PAD].present) return;
	for (i = 0; i < 6; i++)
		if (pad_takes(joy[i])) pad_store(joy[i], 0, 0);
	for (i = 0; i < 6; i++) {
		if (pad_takes(0x1A)) {
			pad_store(0x1A, i, 0);
			pad_store(0x1A, i << 4 | 0xFu, 0);
		}
		if (pad_takes(0x3D)) pad_store(0x3D, i, 0);
	}
}

static void walk_pad(void)
{
	step("PAD");
	if (!blk[K_PAD].present) { say("not listed"); return; }
	check(brd(K_PAD, PAD_ID) == 0x31444150u, "PAD ID reads PAD1");
	say("PAD STAT: spinner drops %u; K8 0x%08x", (unsigned)brd(K_PAD, PAD_STAT) & 0xFFFFu, (unsigned)blk[K_PAD].k[8]);
}

/* the cause convention: read CAUSE, store it back, then the levels */
static uint32_t take_cause(int kind)
{
	uint32_t c = rd(blk[kind].cause);
	wr(blk[kind].cause, c);
	return c;
}

/* STFLG, the four words, STFLG: one transfer when the two STFLG words are equal */
static uint32_t evt_status(uint32_t w[4])
{
	uint32_t a, b, i, n = 0;
	do {
		a = brd(K_EVT, EVT_STFLG);
		for (i = 0; i < 4; i++) w[i] = brd(K_EVT, EVT_STATUS + 4 * i);
		b = brd(K_EVT, EVT_STFLG);
	} while (a != b && ++n < 16);
	check(a == b, "the status bracket closes (%u retries)", (unsigned)n);
	return a;
}

static void walk_evt(void)
{
	uint32_t c, f, w[4];
	step("EVT: drained, the status read in a bracket");
	if (!blk[K_EVT].present) { say("not listed"); return; }
	check(brd(K_EVT, EVT_IDW) == 0x32545645u, "EVT ID reads EVT2");
	c = take_cause(K_EVT);
	check(c & 1u, "EVT CAUSE 0x%x has STATUS, reloaded at the ENABLE", (unsigned)c);
	f = evt_status(w);
	say("STFLG 0x%08x (SEQ %u, head 0x%02x); STATUS 0x%08x 0x%08x 0x%08x 0x%08x; PEND 0x%x", (unsigned)f,
	    (unsigned)(f >> 16), (unsigned)f & 0xFFu, (unsigned)w[0], (unsigned)w[1], (unsigned)w[2], (unsigned)w[3],
	    (unsigned)brd(K_EVT, EVT_PEND));
}

static void walk_osd(int bound)
{
	uint32_t s0, s1, r, c, word, n = 0, tries;
	step("OSDBUF: an image stored, drops compared around it, the FIFO drained");
	if (!blk[K_OSD].present) { say("not listed"); return; }
	check(brd(K_OSD, OSD_IDW) == 0x3244534Fu, "OSDBUF ID reads OSD2");
	say("K0 0x%08x (rows, row bytes), K1 0x%08x", (unsigned)blk[K_OSD].k[0], (unsigned)blk[K_OSD].k[1]);
	if (!bound) { say("its OPEN bit is not set: not bound"); return; }
	for (tries = 0; tries < 3; tries++) {
		s0 = brd(K_OSD, OSD_STATUS);
		/* rows 0-7 only (a byte in rows 8-15 sets highres): 8x8-pixel squares */
		for (r = 0; r < 8; r++)
			for (c = 0; c < 256; c += 4) {
				uint32_t sq = ((r + (c >> 3)) & 1u) ? 0xFFu : 0x00u;
				word = sq * 0x01010101u;
				bwr(K_OSD, (r << 8) + c, word);
				n++;
			}
		s1 = brd(K_OSD, OSD_STATUS);
		if ((s1 & 0xFFFFu) == (s0 & 0xFFFFu)) break;
		say("drops moved %u -> %u: storing the image again", (unsigned)s0 & 0xFFFFu, (unsigned)s1 & 0xFFFFu);
	}
	check(tries < 3, "%u image words stored with no drop", (unsigned)n);
	check(wait_bits(blk[K_OSD].off + OSD_STATUS, 0x1FF0000u, 0, T_QUIET_MS, &s1), "OSD level 0 within %u ms (STATUS 0x%08x)",
	      T_QUIET_MS, (unsigned)s1);
}

/* a capture frame: BEGIN, n zero words, END */
static void lane_frame(uint32_t op, uint32_t n)
{
	uint32_t i;
	bwr(K_LANE, LN_BEGIN, LN_CAPTURE | op);
	for (i = 0; i < n; i++) bwr(K_LANE, LN_WORD, 0);
	bwr(K_LANE, LN_END, 0);
}

static void walk_lane(int bound)
{
	enum { NS = 8, NC = 32 };
	uint16_t rep[NS + 1 + NC + 1];
	uint32_t want, got = 0, p0, p, c, i, nc, f, w[4], wakes = 0;
	uint64_t t;
	step("LANE: two capture frames, drained by the cause convention");
	if (!blk[K_LANE].present) { say("not listed"); return; }
	check(brd(K_LANE, LN_IDW) == 0x314E414Cu, "LANE ID reads LAN1");
	say("K9 0x%08x (FIFO depth, strobe gap), K10 0x%08x (reply depth)", (unsigned)blk[K_LANE].k[9],
	    (unsigned)blk[K_LANE].k[10]);
	if (!bound) { say("its OPEN bit is not set: not bound"); return; }
	check(brd(K_LANE, LN_CAUSE) == 0, "LANE CAUSE reads 0 after the ENABLE");
	check((brd(K_LANE, LN_RSTAT) & 0xFFFFu) == 0, "no reply waiting after the ENABLE");
	nc = (uint32_t)strlen(cfg);
	if (nc > NC) nc = NC;
	want = NS + 1 + nc + 1;
	p0 = brd(K_LANE, LN_PLAYED) & 0xFFFFu;
	lane_frame(UIO_GET_STATUS, NS);
	lane_frame(UIO_GET_STRING, nc);
	for (t = now_us(); now_us() - t < T_EVENT_MS * 1000u; ) {
		if (!pending(K_LANE, 1)) continue;
		wakes++;
		take_cause(K_LANE);
		while ((brd(K_LANE, LN_RSTAT) & 0xFFFFu) && got < want) {
			rep[got++] = (uint16_t)brd(K_LANE, LN_RHEAD);
			bwr(K_LANE, LN_RPOP, 0);
		}
		p = brd(K_LANE, LN_PLAYED) & 0xFFFFu;
		if (((p - p0) & 0xFFFFu) == 2u && got == want && !(brd(K_LANE, LN_RSTAT) & 0xFFFFu)) break;
	}
	p = brd(K_LANE, LN_PLAYED) & 0xFFFFu;
	check(((p - p0) & 0xFFFFu) == 2u, "PLAYED moved by 2 (%u -> %u)", (unsigned)p0, (unsigned)p);
	check(got == want, "%u of %u replies drained over %u wakes; DROPS %u, RSTAT 0x%08x", (unsigned)got, (unsigned)want,
	      (unsigned)wakes, (unsigned)brd(K_LANE, LN_DROPS) & 0xFFFFu, (unsigned)brd(K_LANE, LN_RSTAT));
	if (got != want) return;
	if (blk[K_EVT].present) {
		f = evt_status(w);
		check((rep[0] & 0xFFu) == (f & 0xFFu), "0x29's first reply 0x%04x is STFLG's head byte 0x%02x", rep[0],
		      (unsigned)f & 0xFFu);
		for (i = 0, c = 0; i < NS; i++)
			c += rep[1 + i] != (uint16_t)(w[i / 2] >> (16u * (i & 1u)));
		check(!c, "0x29's eight status replies are EVT's STATUS words (%u differ)", (unsigned)c);
	}
	for (i = 0, c = 0; i < nc; i++) c += (rep[NS + 2 + i] & 0xFFu) != (uint8_t)cfg[i];
	check(!c, "0x14's %u replies are CONF_STR's first bytes as read over the link (%u differ)", (unsigned)nc,
	      (unsigned)c);
}

static void walk_sector(void)
{
	uint32_t i, n = 0;
	step("SECTOR");
	if (!blk[K_SECTOR].present) { say("not listed"); return; }
	check(brd(K_SECTOR, SEC_IDW) == 0x33434553u, "SECTOR ID reads SEC3");
	say("K0 0x%08x (FIFO entries, strobe gap), K1 0x%08x (BLKSZ, wide), K2 %u descriptors",
	    (unsigned)blk[K_SECTOR].k[0], (unsigned)blk[K_SECTOR].k[1], (unsigned)blk[K_SECTOR].k[2]);
	n = blk[K_SECTOR].k[2] > 3u ? 3u : blk[K_SECTOR].k[2];
	for (i = 0; i < n; i++) {
		uint32_t d = blk[K_SECTOR].k[4 + 4 * i];
		say("slot index %u: flags 0x%02x, hps_io slot %u, %u words a frame, %u LBA bits, image 0x%08x + 0x%x",
		    (unsigned)i, (unsigned)(d >> 24), (unsigned)(d >> 16) & 0xFu, (unsigned)d & 0x1FFFu,
		    (unsigned)blk[K_SECTOR].k[5 + 4 * i] & 0x3Fu, (unsigned)blk[K_SECTOR].k[6 + 4 * i],
		    (unsigned)blk[K_SECTOR].k[7 + 4 * i]);
	}
	say("CTRL 0x%08x, REQ 0x%08x, ACCEPT 0x%08x", (unsigned)brd(K_SECTOR, SEC_CTRL), (unsigned)brd(K_SECTOR, SEC_REQ),
	    (unsigned)brd(K_SECTOR, SEC_ACCEPT));
}

/* one pad press on joystick_0: the level, a hold, the level released */
static void press(uint32_t bit)
{
	pad_store(UIO_JOY0, 0, 1u << bit);
	sleep_ms(30);
	pad_store(UIO_JOY0, 0, 0);
}

/* the EVT status after an event: drained until STATUS is among the causes, then the bracket read */
static int evt_after(uint32_t w[4], uint32_t *f)
{
	uint64_t t = now_us();
	while (now_us() - t < T_EVENT_MS * 1000u)
		if (pending(K_EVT, 1) && (take_cause(K_EVT) & 1u)) {
			*f = evt_status(w);
			return 1;
		}
	return 0;
}

/* a primary frame: BEGIN_S[0] with the LBA asked, the words in order, ACCEPT; resent from next_index, or, with
 * ACCEPT's closed bit set (the BEGIN_S refused: no frame open), the BEGIN_S stored again and every word from 0 */
static int sector_frame(uint32_t lba, uint32_t words)
{
	uint32_t i = 0, a = 0, tries;
	bwr(K_SECTOR, SEC_BEGIN(0), lba);
	for (tries = 0; tries < 4; tries++) {
		for (; i < words; i++) bwr(K_SECTOR, SEC_DATA + 4 * i, pattern(lba, i));
		a = brd(K_SECTOR, SEC_ACCEPT);
		if (a >> 31) break;
		if ((a >> 30) & 1u) {
			say("ACCEPT 0x%08x: closed, the BEGIN_S refused; storing it again", (unsigned)a);
			sleep_ms(1);
			bwr(K_SECTOR, SEC_BEGIN(0), lba);
			i = 0;
			continue;
		}
		i = a & 0x1FFFu;
		say("ACCEPT 0x%08x: resending from word %u", (unsigned)a, (unsigned)i);
	}
	return check(a >> 31, "the frame for LBA %u: ACCEPT 0x%08x", (unsigned)lba, (unsigned)a);
}

static void walk_template(int sec_bound)
{
	uint32_t w[4], f0, f, i, last = 0;
	step("the Template's example: the first pad button, through EVT");
	if (!pad_takes(UIO_JOY0) || !blk[K_EVT].present) { say("needs PAD and EVT"); return; }
	f0 = evt_status(w);
	press(4);
	if (check(evt_after(w, &f), "a status transfer after the first button"))
		check(f >> 16 != f0 >> 16, "SEQ moved: %u -> %u", (unsigned)(f0 >> 16), (unsigned)(f >> 16));
	step("the Template's block reader: the second pad button, served over SECTOR");
	if (!sec_bound) { say("SECTOR is not bound"); return; }
	for (i = 0; i < 2; i++) {
		uint32_t d = blk[K_SECTOR].k[4], words = d & 0x1FFFu, c, q, lba;
		press(5);
		if (!check(pending(K_SECTOR, T_EVENT_MS), "a SECTOR cause after the second button")) return;
		c = take_cause(K_SECTOR);
		q = brd(K_SECTOR, SEC_REQ);
		lba = brd(K_SECTOR, SEC_LBA(0));
		say("CAUSE 0x%x, REQ 0x%08x, RSTAT_S[0] 0x%08x, LBA_S[0] %u", (unsigned)c, (unsigned)q,
		    (unsigned)brd(K_SECTOR, SEC_RSTAT(0)), (unsigned)lba);
		if (!check((c & 1u) && (q >> 31) && ((q >> 16) & 3u) == 0, "the primary's request is offered")) return;
		if (i) check(lba == last + 1u, "the reader asks for the next block (LBA %u after %u)", (unsigned)lba, (unsigned)last);
		last = lba;
		if (!sector_frame(lba, words)) return;
		if (!check(evt_after(w, &f), "the reader's write-back through EVT")) return;
		if (words == 128u && (blk[K_SECTOR].k[1] & 1u))
			check(w[1] == ((lba & 0xFFFFu) << 16 | pattern_sum(lba, words)),
			      "status[63:32] 0x%08x is {LBA, the pattern's sum 0x%04x}", (unsigned)w[1],
			      pattern_sum(lba, words));
		else
			say("status[63:32] 0x%08x (a %u-word frame: the reader's 256-word RAM sum is not compared)",
			    (unsigned)w[1], (unsigned)words);
	}
}

/* ---- the end ---- */

static void finish(void)
{
	uint32_t e, v;
	uint64_t t;
	step("the end: the identification clears the enable; every block back at rest");
	identify();
	e = read_enable("after the identification");
	check(!(e & E_EN) && !(gpi() & GPI_EN), "en and GPI[22] are 0");
	for (t = now_us(); !((e = rd(H_ENABLE)) & E_QUIET); )
		if (now_us() - t > T_QUIET_MS * 1000u) break;
	check(e & E_QUIET, "quiet within %u ms (OWNED 0x%08x)", T_QUIET_MS, (unsigned)rd(H_OWNED));
	check(rd(H_OWNED) == 0, "OWNED reads 0");
	v = rd(H_OPEN);
	check(v == 0, "OPEN reads 0 (0x%08x)", (unsigned)v);
	check(rd(H_LINE_EN) == 0, "LINE_EN reads 0");
	if (blk[K_SECTOR].present) check((brd(K_SECTOR, SEC_CTRL) & 7u) == 0, "SECTOR's EN bits are clear");
	say("the core is on stock SPI for whatever program comes next");
}

static void die(int osd_bound)
{
	uint32_t i, st = 0;
	step("--die: a link command begun, lane frames behind it; an OSD repaint begun, its bytes queued; the enable left");
	sel(GPO_IO_EN, 1);
	link_w(UIO_GET_STRING);
	if (blk[K_LANE].present)
		for (i = 0; i < 4; i++) lane_frame(UIO_GET_STATUS, 8);
	/* the OSD select alone: io_osd_hdmi = ss1 & ~ss0 and io_osd_vga = ss1 & ~ss2 both read 1 */
	gpo_w((gpo & ~(GPO_IO_EN | GPO_FPGA_EN)) | GPO_IO_VIEW | GPO_OSD_EN);
	if (osd_bound) {
		for (i = 0; i < 64; i++) bwr(K_OSD, i * 4u, 0xFFFFFFFFu);
		sleep_ms(10);
		st = brd(K_OSD, OSD_STATUS);
		check(((st >> 16) & 0x1FFu) == 64u, "the 64 OSD stores wait behind the osd select (STATUS 0x%08x)", (unsigned)st);
	}
	say("GPO 0x%08x; ENABLE 0x%08x; PLAYED %u; RSTAT 0x%08x; OSD STATUS 0x%08x", (unsigned)gpo,
	    (unsigned)rd(H_ENABLE), blk[K_LANE].present ? (unsigned)brd(K_LANE, LN_PLAYED) & 0xFFFFu : 0u,
	    blk[K_LANE].present ? (unsigned)brd(K_LANE, LN_RSTAT) : 0u, (unsigned)st);
	say("exiting with the OSD select asserted and the enable set");
}

static void usage(void)
{
	fputs("usage: lw_probe [--template] [--die]\n"
	      "  Walks the LW bridge protocol on a loaded core. Run it with no other program on the link.\n"
	      "  --template  drive the Template core's LW example (its pad buttons, its block reader)\n"
	      "  --die       test only: end mid-repaint, lane frames and OSD bytes queued behind the selects, the enable\n"
	      "              left set\n", stderr);
	exit(2);
}

int main(int argc, char **argv)
{
	int fd, i, tpl = 0, dying = 0, sec_bound = 0, reader = -1;
	uint32_t serve = 0, g, line_en = 0, opened;

	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--template")) tpl = 1;
		else if (!strcmp(argv[i], "--die")) dying = 1;
		else usage();
	}
	fd = open("/dev/mem", O_RDWR | O_SYNC);   /* O_SYNC: a strongly ordered mapping, every access completes in turn */
	if (fd < 0) { fprintf(stderr, "lw_probe: /dev/mem: %s\n", strerror(errno)); return 2; }
	mgr = map(fd, FPGAMGR_BASE, PAGE);
	rst = map(fd, RSTMGR_BASE, PAGE);

	look();
	step("the identification: the ID view and back, which clears any host enable");
	gpo = 0;                                    /* the ID view with every select, the strobe and the reset low */
	g = identify();
	say("GPI in the ID view 0x%08x", (unsigned)g);
	if (g >> 8 != CORE_MAGIC) { say("no core magic: the FPGA holds no MiSTer core; stay on SPI"); return 3; }
	say("core type 0x%02x", (unsigned)g & 0xFFu);
	discover(fd);
	walk();

	if (tpl && blk[K_SECTOR].present && blk[K_SECTOR].k[2] >= 1u && (blk[K_SECTOR].k[4] >> 24) & 1u) {
		serve |= 1u << blk[K_SECTOR].idx;
		reader = (int)(blk[K_SECTOR].k[4] >> 16) & 0xF;
	}
	if (blk[K_OSD].present) serve |= 1u << blk[K_OSD].idx;
	if (blk[K_LANE].present) serve |= 1u << blk[K_LANE].idx;
	enable(serve);
	opened = rd(H_OPEN);

	sdstat(reader);
	step("binding: SECTOR EN, every pad level, LINE_EN");
	if (reader >= 0 && (opened >> blk[K_SECTOR].idx) & 1u) {
		bwr(K_SECTOR, SEC_CTRL, 1u);
		sec_bound = check((brd(K_SECTOR, SEC_CTRL) & 7u) == 1u, "the primary's EN is set");
	}
	pad_levels();
	for (i = 1; i <= K_LAST; i++)
		if (blk[i].present && blk[i].line && blk[i].cause && ((opened >> blk[i].idx) & 1u) &&
		    (i != K_SECTOR || sec_bound))
			line_en |= 1u << blk[i].idx;
	wr(H_LINE_EN, line_en);
	check(rd(H_LINE_EN) == line_en, "LINE_EN 0x%08x (the probe polls PENDING)", (unsigned)line_en);
	read_enable("bound");

	walk_pad();
	walk_evt();
	walk_osd(blk[K_OSD].present && ((opened >> blk[K_OSD].idx) & 1u));
	walk_lane(blk[K_LANE].present && ((opened >> blk[K_LANE].idx) & 1u));
	walk_sector();
	if (tpl) walk_template(sec_bound);
	read_enable("after the walk");
	check(!(rd(H_ENABLE) & (E_STALE | E_BURST)), "no store was dropped as stale or as a burst");

	if (dying) die(blk[K_OSD].present && ((opened >> blk[K_OSD].idx) & 1u));
	else finish();
	say("%s: %u failures", fails ? "FAILED" : "PASSED", fails);
	return fails ? 1 : 0;
}
