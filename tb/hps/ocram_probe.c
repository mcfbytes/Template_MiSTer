/*
 * ocram_probe: the HPS-side probe of the OCRAM mailbox (docs/OCRAM_MAILBOX.md, layout 1: the OSD tier, the DIAG
 * host-to-fabric torture sink and the freeze knob), for the Template test core built with MISTER_OCRAM +
 * MISTER_OCRAM_DIAG + MISTER_OCRAM_OSD and NO LW bridge. It speaks only that guide and sys/hps_ocram_master.sv:
 * GPI 26 in the I/O view, the warmramgrp.enable clear, the ECC flag clear, the control page and
 * the OSD, EVT and torture rings zeroed with 64-bit first stores, the RUN edge (GPO 28: a 0 store, then a 1
 * store, through a copy of the GPO register), the armed read, the hello and its echo, the OSD ring (H_BIND bit
 * 11, ROW records, C_IDX[2]), the SPI OSD path (stock's OsdUpdate transport), the host-to-fabric torture ring
 * (TSEQ records, the VERDICT arithmetic), the freeze knob (GPO 25), and the quiesce (RUN 0, then the stopped
 * value).
 *
 *   ocram_probe look                       read-only: GPO, GPI, brgmodrst, warmramgrp.enable, eccgrp.ocram, C_HELLO
 *   ocram_probe run [opts]                 one session (M3)
 *   ocram_probe matrix [--seconds N]       M3's four arms
 *   ocram_probe quiesce                    RUN 0, then the stopped read
 *   ocram_probe torture --words N          M4, fabric to host (M3's torture; no --map)
 *   ocram_probe torture --map so|wc|wc-dmb|wc-dsb|wc-wmb [--words N] [--maxlen L]
 *                                       M4's other half, host to fabric through the strongly-ordered map or
 *                                       /dev/ocram_wc (load it first; the probe insmods nothing); the ring runs full
 *   ocram_probe torture --map ... --band [--words N] [--maxlen L (256)] [--rate W (6.3e6)] [--seed S]
 *                                       [--empty-words E (2^20)]
 *                                       E25, the ring PART-full: each cycle ONE read of C_IDX[4], then records back to
 *                                       back up to a random HI (50-90% of the ring) with no read of any mailbox word,
 *                                       then a wait by the clock (no mailbox access) down to a random LO (5-45%) from
 *                                       the fabric's rate W; every E words the ring runs empty and the VERDICT is
 *                                       checked. Ends with one PASS or FAIL line and the fill histograms
 *   ocram_probe osd [--rows N] [--repeat K] [--pattern rows|bars|inc] [--hold S] [--no-enable]
 *                                       rows through the OSD ring; times one row and a repaint; leaves the
 *                                       screen for the operator, then quiesces
 *   ocram_probe spi-osd [--rows N] [--pattern P] [--hold S]
 *   ocram_probe fallback [--rows N] [--pattern P] [--hold S]
 *                                       the stock SPI OSD path; no RUN edge, the control page untouched
 *   ocram_probe osd-off                    the SPI OSD disable (0x40)
 *   ocram_probe freeze                     the stopped-clk_sys case (test core only)
 *   ocram_probe gate --attended ...        M5 (M3)
 *   run options: --ns (GPO 27: AxPROT[1] = 1 for the run)   --flood (GPO 26: MARKs back to back)
 *                --line (H_LINE bit 3: the EVT line may rise; drained every --drain-ms, default 200)
 *                --seconds N (default 5)   --drain-ms M (default 50; with --flood the drain never sleeps)
 *
 * Run it as root on the DE10-Nano with the test core loaded and NO MiSTer process running (it refuses while one
 * is, because Main's next GPO store would clear RUN under it). It never maps an LW-bridge or h2f address: every
 * mapping goes through map_phys(), which accepts only the FPGA manager, reset manager, system manager and GIC
 * distributor pages and OCRAM; /dev/ocram_wc is opened only by --map wc*. Its only OCRAM stores are layout 1's
 * control page (0x0000-0x00FF, the host half after the zeroing) and the zeroing of the OSD, EVT and torture
 * rings (0x2000-0x3FFF, 0x4000-0x5FFF, 0x7000-0xEFFF); the ring payload stores of the OSD and torture modes
 * follow the same map. It reads OCRAM only after its own zeroing, except the read-only look and the SPI modes,
 * which read the control page without writing it. The GIC read (GICD_ISPENDR for f2h_irq[11], GIC ID 83) is a
 * side-effect-free witness of the EVT line; whether a disabled SPI shows its pending state there is UNVERIFIED
 * on this GIC, so it is reported, never failed.
 *
 * Exit status: 0 every check passed (a write-combined torture error is a RESULT and still exits 0); 1 a check
 * failed; 2 it could not run; 3 it stopped before any store.
 * Build: arm-buildroot-linux-gnueabihf-gcc -std=c11 -O2 -marm -Wall -Wextra -Werror -o ocram_probe ocram_probe.c
 * Desk build: cc -std=c11 -O2 -Wall -Wextra -Werror -DOCRAM_FAKE -pthread -o ocram_probe_fake ocram_probe.c
 *            (a thread plays the fabric's master over fake registers, ocram_probe_fake.h; no hardware is touched;
 *            the unit test of arm H, ocram_probe_test.c, drives it)
 */
#define _POSIX_C_SOURCE 200809L
#define _FILE_OFFSET_BITS 64

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>

/* ---- HPS addresses (Cyclone V) ---- */
#define FPGAMGR_BASE  0xFF706000u   /* GPO +0x10, GPI +0x14 */
#define RSTMGR_BASE   0xFFD05000u   /* brgmodrst +0x1C: bit 2 = fpga2hps */
#define SYSMGR_BASE   0xFFD08000u   /* warmramgrp.enable +0xE0, eccgrp.ocram +0x144 */
#define GICD_BASE     0xFFFED000u   /* GIC distributor: ISENABLER +0x100, ISPENDR +0x200 */
/* M5 (gate) only: the core reload, as stock fpga_io.cpp socfpga_load() + do_bridge() do it */
#define FPGADAT_BASE  0xFFB90000u   /* the FPGA manager data port */
#define L3REGS_BASE   0xFF800000u   /* NIC-301 remap +0x0 */
#define SDR_BASE      0xFFC25000u   /* SDRAM controller fpgaportrst +0x080 (0xFFC25080) */
#define OCRAM_BASE    0xFFFF0000u
#define OCRAM_SIZE    0x10000u
#define PAGE          0x1000u

#define GPO_IO_VIEW   (1u << 31)
#define GPO_RESET     (1u << 30)
#define GPO_RUN       (1u << 28)    /* RUN (design note §2.6) */
#define GPO_DIAG_NS   (1u << 27)    /* test core only: AxPROT[1] = 1 for the run */
#define GPO_DIAG_FL   (1u << 26)    /* test core only: MARK records back to back */
#define GPO_FREEZE    (1u << 25)    /* DIAG + OSD test core only: stops the OSD clk_sys half (GPO 25) */
#define GPO_KNOBS     (GPO_RUN | GPO_DIAG_NS | GPO_DIAG_FL | GPO_FREEZE)
#define GPI_UNCONF    (1u << 31)
#define GPI_OCRAM     (1u << 26)
#define GPI_LW        (1u << 25)
#define BRG_F2H       (1u << 2)
#define ECC_FLAGS     0x19u         /* EN | SERR | DERR: W1C of the two flags, EN kept */
#define GIC_EVT_ID    83u           /* f2h_irq[11] = GIC SPI 51 = ID 83 */

/* ---- the SPI OSD transport (fpga_io.cpp's fpga_spi, spi.cpp's EnableOsd; osd.v's decode) ---- */
#define SPI_STROBE    (1u << 17)    /* data in GPO [15:0], strobe GPO 17, ack GPI 17 */
#define SPI_ACK       SPI_STROBE
#define SPI_FPGA_SEL  (1u << 18)
#define SPI_OSD_SEL   (1u << 19)    /* OSD select: GPO 19 with GPO 31 set */
#define SPI_IO_SEL    (1u << 20)
#define SPI_OSD_WRITE   0x20u       /* OSD_CMD_WRITE: 0x20 | row, then 256 bytes */
#define SPI_OSD_DISABLE 0x40u       /* OSD_CMD_DISABLE */
#define SPI_OSD_ENABLE  0x41u       /* OSD_CMD_ENABLE */

/* ---- layout 1 (§2.3, §2.4) ---- */
#define H_HELLO   0x00u
#define H_BIND    0x04u
#define H_LINE    0x08u
#define H_SECTOR  0x0Cu
#define H_IDX(r)  (0x10u + 4u * (r))
#define C_HELLO   0x80u
#define C_VERSION 0x84u
#define C_CAPS    0x88u
#define C_STATE   0x8Cu
#define C_IDX(r)  (0x90u + 4u * (r))
#define C_BIND    0xA4u
#define C_RING(r) (0xB0u + 4u * (r))
#define C_K(n)    (0xD0u + 4u * (n))
#define R_OSD     2u
#define R_EVT     3u
#define R_TS      4u
#define OSD_OFF   0x2000u
#define OSD_WORDS 2048u
#define EVT_OFF   0x4000u
#define EVT_WORDS 2048u
#define TS_OFF    0x7000u
#define TS_WORDS  8192u
#define PAGE_OFF  0x0000u
#define PAGE_LEN  0x0100u

#define V_ARMED   0x4F430000u
#define V_WAITING 0x57540000u
#define V_STOPPED 0x51540000u
#define K_ROW     0x01u
#define ROW_LEN   64u               /* ROW: one header plus 64 payload words */
#define K_MARK    0x23u
#define MARK_LEN  40u
#define K_VERDICT 0x25u
#define VERDICT_ARG 4u
#define VERDICT_LEN 8u
#define K_TSEQ    0x50u
#ifdef OCRAM_FAKE
#define BOUND_NS  100000000L        /* the desk's thread ticks in tens of us, and a loaded host stalls it for ms */
#define BIND_NS   200000000L
#else
#define BOUND_NS  1000000L          /* every bounded read is 1 ms (docs/OCRAM_MAILBOX.md) */
#define BIND_NS   2000000L          /* H_BIND -> C_BIND: one poll period plus 1 ms */
#endif
#define OSD_WAIT_NS 2000000000LL    /* one row or a repaint consumed */
#define TORT_WAIT_NS 5000000000LL   /* the torture ring drained after the last record */
#define VERDICT_WAIT_NS 1000000000LL /* one more VERDICT after the drain */
#define MAX_ROWS  32u
#define MAX_MAXLEN 8191u

/* the hello's expected words (layout 1, with the OSD, EVT and DIAG rings) */
#define C_CAPS_M7  0x0000001Cu      /* bit 2 OSD, bit 3 EVT, bit 4 torture */
#define C_RING2_M7 0x182C0800u      /* {3, 0, 11, 0, 0x800}: OSD, no line, 2048 words at 0x2000 */
#define C_RING3_M7 0x212C1000u      /* {4, 4, 11, 0, 0x1000}: EVT, line 4, 2048 words at 0x4000 */
#define C_RING4_M7 0xF8341C00u      /* {31, 0, 13, 0, 0x1C00}: DIAG, no line, 8192 words at 0x7000 */

/* OSD patterns (--pattern) */
enum { PAT_ROWS, PAT_BARS, PAT_INC };

static volatile uint32_t *mgr, *rst, *sys, *gicd, *ocr, *fdat, *l3, *sdr;
static volatile uint32_t *wc;       /* /dev/ocram_wc's map (--map wc*), else NULL */
#ifndef OCRAM_FAKE
static int wc_fd = -1;
#endif
static int quiet;                   /* M5 cycles: say() prints nothing; check() prints only a failure */
static uint32_t gpo_base;           /* the GPO register as found, RUN and the knobs cleared */
static uint32_t gpo_shadow;         /* the GPO copy every store goes through */
static int io_err;                  /* a bounded GPI wait, a bad GPI read or a failed WC ioctl */
static unsigned n_fail;
static unsigned n_vfail;            /* verdict mismatches in the host-to-fabric torture */

#define OCRAM_WC_WMB _IO('o', 1)    /* kmod/ocram_wc.c: the kernel's wmb(), dsb + the PL310 sync */

static void say(const char *fmt, ...)
{
	va_list a;
	if (quiet) return;
	va_start(a, fmt);
	vprintf(fmt, a);
	va_end(a);
	fflush(stdout);
}

static void check(int ok, const char *fmt, ...)
{
	va_list a;
	if (!ok) n_fail++;
	if (ok && quiet) return;
	fputs(ok ? "pass " : "FAIL ", stdout);
	va_start(a, fmt);
	vprintf(fmt, a);
	va_end(a);
	fputc('\n', stdout);
	fflush(stdout);
}

static int64_t now_ns(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (int64_t)t.tv_sec * 1000000000 + t.tv_nsec;
}

static void sleep_ms(long ms)
{
	struct timespec t = { ms / 1000, (ms % 1000) * 1000000L };
	while (nanosleep(&t, &t) && errno == EINTR) {}
}

/* ---- mappings: an allowlist, so no LW or h2f address can ever be mapped ---- */
#ifndef OCRAM_FAKE
static int memfd = -1;
static volatile uint32_t *map_phys(uint32_t phys, uint32_t len)
{
	void *m;
	int ok = (phys == FPGAMGR_BASE || phys == RSTMGR_BASE || phys == SYSMGR_BASE || phys == GICD_BASE ||
	          phys == FPGADAT_BASE || phys == L3REGS_BASE || phys == SDR_BASE)
	         ? len == PAGE : (phys == OCRAM_BASE && len == OCRAM_SIZE);
	if (!ok) {
		fprintf(stderr, "ocram_probe: refusing to map 0x%08x (+0x%x): not on the allowlist\n", phys, len);
		exit(2);
	}
	if (memfd < 0 && (memfd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		perror("ocram_probe: /dev/mem");
		exit(2);
	}
	m = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, (off_t)phys);
	if (m == MAP_FAILED) {
		perror("ocram_probe: mmap");
		exit(2);
	}
	return (volatile uint32_t *)m;
}

/* first touch: one 64-bit store per pair of words, so no partial ECC word is ever read (V20, K7) */
static void zero64(volatile uint32_t *p, unsigned words)
{
	unsigned i;
	for (i = 0; i < words; i += 2)
#ifdef __arm__
		__asm__ volatile("mov r2, #0\n\tmov r3, #0\n\tstrd r2, r3, [%0]" ::"r"(p + i) : "r2", "r3", "memory");
#else
		p[i] = p[i + 1] = 0;   /* a host build only compiles this path; the probe runs on the HPS */
#endif
}

static int main_running(void)
{
	DIR *d = opendir("/proc");
	struct dirent *e;
	int found = 0;
	if (!d) return 1;
	while (!found && (e = readdir(d)) != NULL) {
		char p[300], c[32] = "";
		FILE *f;
		if (e->d_name[0] < '1' || e->d_name[0] > '9') continue;
		snprintf(p, sizeof p, "/proc/%s/comm", e->d_name);
		if (!(f = fopen(p, "r"))) continue;
		if (fgets(c, sizeof c, f) && strncmp(c, "MiSTer", 6) == 0) found = 1;
		fclose(f);
	}
	closedir(d);
	return found;
}
#else
#include "ocram_probe_fake.h"
#endif

#define REG(b, off) ((b)[(off) / 4])
#ifdef OCRAM_FAKE
#define OC(off)     (*f_oc(off))    /* the desk build sees which phase touches which word */
#define PHASE(p)    (f_phase = (p))
#define FAKE_BLOG(lo, hi, fill, peak) f_blog_add((lo), (hi), (fill), (peak))
#else
#define OC(off)     REG(ocr, off)
#define PHASE(p)    ((void)0)
#define FAKE_BLOG(lo, hi, fill, peak) ((void)0)
#endif

static uint32_t gpo_read(void)       { return REG(mgr, 0x10); }
static void gpo_store(uint32_t v)    { gpo_shadow = v; REG(mgr, 0x10) = v; }
static uint32_t gpi_read(void)       { return REG(mgr, 0x14); }

/* ---- the SPI OSD transport: stock's fpga_spi (fpga_io.cpp:698-733) over the GPO copy ---- */
static int gpi_wait(uint32_t bit, int want, int64_t bound_ns)
{
	int64_t t0 = now_ns();
	uint32_t v;
	for (;;) {
		v = gpi_read();
		if (v & GPI_UNCONF) { io_err = 1; return -1; }
		if (!!(v & bit) == want) return 0;
		if (now_ns() - t0 >= bound_ns) { io_err = 1; return -1; }
	}
}

static int spi_xfer(uint16_t word, uint16_t *out)
{
	uint32_t g = (gpo_shadow & ~(0xFFFFu | SPI_STROBE)) | (uint32_t)word;
	gpo_store(g);                 /* the data, strobe low */
	gpo_store(g | SPI_STROBE);    /* the strobe */
	if (gpi_wait(SPI_ACK, 1, BOUND_NS)) return -1;
	gpo_store(g);                 /* the strobe low */
	if (gpi_wait(SPI_ACK, 0, BOUND_NS)) return -1;
	if (out) *out = (uint16_t)gpi_read();
	return 0;
}

static int spi_byte(uint8_t b)
{
	return spi_xfer((uint16_t)b, NULL);
}

/* the desk's fabric thread samples GPO far slower than the bus; hold a select edge long enough to be seen */
static void spi_settle(void)
{
#ifdef OCRAM_FAKE
	int64_t w = now_ns() + 200000;
	while (now_ns() < w) {}
#endif
}

static void spi_select(int on)
{
	uint32_t g = gpo_shadow;
	/* EnableOsd in stock sets only GPO 19 (with bit 31); sys_top's io_osd also needs 18 and 20 low */
	if (on) g = (g | SPI_OSD_SEL | GPO_IO_VIEW) & ~(SPI_FPGA_SEL | SPI_IO_SEL);
	else g &= ~SPI_OSD_SEL;
	gpo_store(g);
	spi_settle();
}

static int spi_osd_cmd(uint8_t cmd)
{
	spi_select(1);
	if (spi_byte(cmd)) { spi_select(0); return -1; }
	spi_select(0);
	return 0;
}

static int spi_osd_row(unsigned row, const uint8_t *bytes)
{
	unsigned i;
	spi_select(1);
	if (spi_byte((uint8_t)(SPI_OSD_WRITE | row))) { spi_select(0); return -1; }
	for (i = 0; i < 256; i++) if (spi_byte(bytes[i])) { spi_select(0); return -1; }
	spi_select(0);
	return 0;
}

/* ---- the write-combined map: only --map wc* opens /dev/ocram_wc (load it first) ---- */
static void open_wc(void)
{
#ifdef OCRAM_FAKE
	wc = f_ocr;
#else
	void *m;
	wc_fd = open("/dev/ocram_wc", O_RDWR);
	if (wc_fd < 0) {
		fprintf(stderr, "ocram_probe: --map wc*: open /dev/ocram_wc: %s (load kmod/ocram_wc.ko)\n",
		        strerror(errno));
		exit(2);
	}
	m = mmap(NULL, OCRAM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, wc_fd, 0);
	if (m == MAP_FAILED) {
		perror("ocram_probe: mmap /dev/ocram_wc");
		exit(2);
	}
	wc = (volatile uint32_t *)m;
#endif
}

/* the publish barrier of a --map wc* run, before each head store (the sheet's four variants) */
static void wc_sync(int map)
{
	if (!wc) return;
	if (map == 2) {
#if defined(__arm__) && !defined(OCRAM_FAKE)
		__asm__ volatile("dmb st" ::: "memory");
#endif
	} else if (map == 3) {
#if defined(__arm__) && !defined(OCRAM_FAKE)
		__asm__ volatile("dsb st" ::: "memory");
#endif
	} else if (map == 4) {
#ifndef OCRAM_FAKE
		if (ioctl(wc_fd, OCRAM_WC_WMB) < 0) {
			fprintf(stderr, "ocram_probe: ioctl OCRAM_WC_WMB: %s\n", strerror(errno));
			io_err = 1;
		}
#endif
	}
}

static int gic_pending(void)
{
	return (int)((REG(gicd, 0x200 + 4 * (GIC_EVT_ID / 32)) >> (GIC_EVT_ID % 32)) & 1u);
}

/* reads C_HELLO until it equals want, for at most 1 ms; returns the ns taken, or -1 with *last the last value */
static int64_t await_c_hello(uint32_t want, uint32_t *last)
{
	int64_t t0 = now_ns(), t;
	uint32_t v;
	do {
		v = OC(C_HELLO);
		t = now_ns() - t0;
		if (v == want) { *last = v; return t; }
	} while (t < BOUND_NS);
	*last = v;
	return -1;
}

static void map_all(void)
{
	mgr  = map_phys(FPGAMGR_BASE, PAGE);
	rst  = map_phys(RSTMGR_BASE, PAGE);
	sys  = map_phys(SYSMGR_BASE, PAGE);
	gicd = map_phys(GICD_BASE, PAGE);
	ocr  = map_phys(OCRAM_BASE, OCRAM_SIZE);
}

static void look(void)
{
	uint32_t g = gpo_read();
	say("look gpo=0x%08x gpi=0x%08x brgmodrst=0x%08x warmram.enable=0x%08x ecc.ocram=0x%08x gic83.pend=%d en=%d\n",
	    g, gpi_read(), REG(rst, 0x1C), REG(sys, 0xE0), REG(sys, 0x144), gic_pending(),
	    (int)((REG(gicd, 0x100 + 4 * (GIC_EVT_ID / 32)) >> (GIC_EVT_ID % 32)) & 1u));
	say("look: ocram C_HELLO=0x%08x C_VERSION=0x%08x C_CAPS=0x%08x C_STATE=0x%08x C_BIND=0x%08x\n",
	    OC(C_HELLO), OC(C_VERSION), OC(C_CAPS), OC(C_STATE), OC(C_BIND));
	say("look: ocram H_HELLO=0x%08x H_BIND=0x%08x H_IDX[2]=0x%08x C_IDX[2]=0x%08x C_RING[2]=0x%08x\n",
	    OC(H_HELLO), OC(H_BIND), OC(H_IDX(R_OSD)), OC(C_IDX(R_OSD)), OC(C_RING(R_OSD)));
	if (!(g & GPO_IO_VIEW)) say("look: GPO bit 31 is 0 (the ID view): GPI shows the core magic, not bit 26\n");
}

/* §2.6: drop RUN, then read C_HELLO until the stopped value, bounded. Returns 1 if it came. */
static int quiesce(const char *tag)
{
	uint32_t last;
	int64_t t;
	gpo_store(gpo_base);
	t = await_c_hello(V_STOPPED, &last);
	if (t >= 0) say("%s: quiesce: stopped value after %lld ns\n", tag, (long long)t);
	else say("%s: quiesce: NO stopped value within 1 ms (C_HELLO 0x%08x)\n", tag, last);
	return t >= 0;
}

/* the preconditions, before any store; returns 0 to go on, 3 to stop having stored nothing */
static int preflight(void)
{
	uint32_t g = gpo_read(), i, b;
	if (main_running()) {
		say("preflight: a MiSTer process is running; stop it first (its next GPO store would clear RUN)\n");
		return 3;
	}
	if (!(g & GPO_IO_VIEW) || (g & GPO_RESET)) {
		say("preflight: GPO reads 0x%08x: not the I/O view out of core reset; load the core from Main first\n", g);
		return 3;
	}
	i = gpi_read();
	if (i & GPI_UNCONF) { say("preflight: GPI 0x%08x: the FPGA is not configured\n", i); return 3; }
	if (!(i & GPI_OCRAM)) {
		say("preflight: GPI 0x%08x: bit 26 is 0 in the I/O view: no OCRAM master in this bitstream; nothing stored\n", i);
		return 3;
	}
	if (i & GPI_LW) say("preflight: note: GPI bit 25 is 1 (an LW firewall is built); this probe never touches it\n");
	b = REG(rst, 0x1C);
	if (b & BRG_F2H) { say("preflight: brgmodrst 0x%08x: the fpga2hps bridge is held in reset; no RUN edge\n", b); return 3; }
	say("preflight: gpo=0x%08x gpi=0x%08x brgmodrst=0x%08x warmram.enable=0x%08x ecc.ocram=0x%08x\n",
	    g, i, b, REG(sys, 0xE0), REG(sys, 0x144));
	gpo_base = g & ~GPO_KNOBS;
	return 0;
}

/* the first stores: RUN 0 (this program's first GPO store carries it), the enable clear, the ECC clear, the
   zeroing. Returns 0 to go on. */
static int prepare(void)
{
	uint32_t g = gpo_read(), v;
	unsigned k, bad = 0;
	if (g & GPO_RUN) {
		say("prepare: GPO 28 was left set by an earlier program: taking the stop proof first\n");
		if (!quiesce("prepare")) {
			say("prepare: no stopped value: a master may hold a transaction. Make no RUN edge on this bitstream\n");
			return 1;
		}
	} else {
		gpo_store(gpo_base);
	}
	sleep_ms(2);   /* §2.6: zero the page no sooner than 1 ms after the first GPO store */
	REG(sys, 0xE0) = 0;
	v = REG(sys, 0xE0);
	check(v == 0, "warmramgrp.enable cleared (reads 0x%08x)", v);
	if (v != 0) return 1;
	REG(sys, 0x144) = ECC_FLAGS;
	say("prepare: eccgrp.ocram after the W1C clear: 0x%08x\n", REG(sys, 0x144));
	zero64(&OC(PAGE_OFF), PAGE_LEN / 4);
	zero64(&OC(OSD_OFF), OSD_WORDS);
	zero64(&OC(EVT_OFF), EVT_WORDS);
	zero64(&OC(TS_OFF), TS_WORDS);
	for (k = 0; k < PAGE_LEN / 4; k++) bad += OC(PAGE_OFF + 4 * k) != 0;
	for (k = 0; k < OSD_WORDS; k++) bad += OC(OSD_OFF + 4 * k) != 0;
	for (k = 0; k < EVT_WORDS; k++) bad += OC(EVT_OFF + 4 * k) != 0;
	for (k = 0; k < TS_WORDS; k++) bad += OC(TS_OFF + 4 * k) != 0;
	check(bad == 0, "control page and the OSD, EVT and torture rings zeroed (%u nonzero words)", bad);
	return bad ? 1 : 0;
}

struct mark {
	uint32_t w[MARK_LEN];
};

struct sess {
	int ns, flood, line, seconds, drain_ms;
	/* results */
	int armed, echoed, stopped;
	int64_t t_arm, t_echo;
	unsigned marks, records, faults, gic_hi_ok, gic_hi_n, gic_lo_ok, gic_lo_n, other;
	uint64_t words;
	struct mark first, last;
	uint32_t state;
	/* M4 (torture): the fabric-to-host word target, the invariant violations, the bad records printed */
	uint64_t t_words, t_err;
	unsigned t_shown;
	/* M7: the hello, C_K[8]'s rows, the last VERDICT */
	uint32_t hello;
	unsigned rows_max;
	uint32_t verdict[VERDICT_LEN];
	unsigned verdicts;
	/* M7: the host-to-fabric torture (--map) */
	int h2f, map, band;
	unsigned maxlen;
	double rate;                    /* --band: the fabric's words/s */
	uint64_t seed, empty_words;     /* --band: the PRNG seed, the words between two empty-ring checks */
	uint64_t words_target_h2f;
	uint64_t pub_records, pub_words;
	uint32_t pub_sum;               /* the RTL's 32-bit wrap-around sum */
	unsigned pub_maxfill;
};

static void print_mark(const char *tag, const struct mark *m)
{
	static const char *const edges[16] = { "0", "8", "16", "24", "32", "40", "48", "56", "64", "128", "256", "512",
	                                       "1024", "2048", "4096", "65536" };
	unsigned b;
	say("%s: fill=%u ns=%u flood=%u hellos=%u\n", tag, m->w[0] & 0xFFFF, (m->w[0] >> 16) & 1, (m->w[0] >> 17) & 1,
	    m->w[0] >> 24);
	say("%s: reads=%u min=%u max=%u sum=%u mean=%.2f errors=%u (clk_100m cycles)\n", tag, m->w[1], m->w[3] & 0xFFFF,
	    m->w[3] >> 16, m->w[5], m->w[1] ? (double)m->w[5] / m->w[1] : 0.0, m->w[7] >> 16);
	say("%s: writes=%u min=%u max=%u sum=%u mean=%.2f errors=%u\n", tag, m->w[2], m->w[4] & 0xFFFF, m->w[4] >> 16,
	    m->w[6], m->w[2] ? (double)m->w[6] / m->w[2] : 0.0, m->w[7] & 0xFFFF);
	say("%s: hist from-cycles:", tag);
	for (b = 0; b < 16; b++) say(" %s", edges[b]);
	say("\n%s: hist rd:", tag);
	for (b = 0; b < 16; b++) say(" %u", m->w[8 + b]);
	say("\n%s: hist wr:", tag);
	for (b = 0; b < 16; b++) say(" %u", m->w[24 + b]);
	say("\n");
}

/* The hello's words, as sys/hps_ocram_master.sv gives them. Sets s->state and s->rows_max. */
static void check_hello(struct sess *s, const char *tag)
{
	uint32_t v;
	unsigned k;
	v = OC(C_VERSION);
	check(v == 0x00010000u, "%s: C_VERSION 0x%08x (1.0)", tag, v);
	v = OC(C_CAPS);
	check(v == C_CAPS_M7, "%s: C_CAPS 0x%08x (OSD + EVT + torture, expected 0x1C)", tag, v);
	v = OC(C_BIND);
	check(v == 0, "%s: C_BIND 0x%08x (nothing bound at the hello)", tag, v);
	v = OC(C_IDX(R_OSD));
	check(v == 0, "%s: C_IDX[2] 0x%08x (the OSD ring's tail starts at 0)", tag, v);
	v = OC(C_IDX(R_EVT));
	/* Under flood the master posts MARKs (41 words) from the echo on, so the head may already have moved. */
	check(v == 0 || (s->flood && v % (MARK_LEN + 1u) == 0),
	      "%s: C_IDX[3] 0x%08x (the EVT ring's head starts at 0, or whole MARKs under flood)", tag, v);
	v = OC(C_IDX(R_TS));
	check(v == 0, "%s: C_IDX[4] 0x%08x (the torture ring's tail starts at 0)", tag, v);
	v = OC(C_RING(R_OSD));
	check(v == C_RING2_M7, "%s: C_RING[2] 0x%08x (OSD, no line, 2048 words at 0x2000)", tag, v);
	v = OC(C_RING(R_EVT));
	check(v == C_RING3_M7, "%s: C_RING[3] 0x%08x (EVT, line 4, 2048 words at 0x4000)", tag, v);
	v = OC(C_RING(R_TS));
	check(v == C_RING4_M7, "%s: C_RING[4] 0x%08x (DIAG, no line, 8192 words at 0x7000)", tag, v);
	for (k = 0; k < 5; k++) if (k != R_OSD && k != R_EVT && k != R_TS) {
		v = OC(C_RING(k));
		check(v == 0, "%s: C_RING[%u] 0x%08x (absent)", tag, k, v);
	}
	v = OC(C_K(8));
	s->rows_max = v >> 16;
	check((v & 0xFFFFu) == 256u, "%s: C_K[8] 0x%08x (row bytes 256)", tag, v);
	check(s->rows_max == 16u || s->rows_max == 20u, "%s: C_K[8] rows %u (16 or 20)", tag, s->rows_max);
	if (s->rows_max != 16u && s->rows_max != 20u) s->rows_max = 16u;
	s->state = OC(C_STATE);
	say("%s: C_STATE 0x%08x (hellos %u, faults 0x%02x, mid-record ends 0x%02x)\n", tag, s->state, s->state & 0xFF,
	    (s->state >> 8) & 0x1F, (s->state >> 16) & 0x1F);
	check((s->state & 0xFFE0E000u) == 0 && ((s->state >> 8) & 0x1F) == 0,
	      "%s: C_STATE has no fault and no unknown bit", tag);
}

/* M4: every word of a flood-mode MARK record is pinned by an invariant against its predecessor. The write count
   steps by exactly one (the previous record's head store, the only counted write between two records), so it is
   the sequence number; each histogram sums to its count (the checksum over 32 words); the counters never go
   backwards; the latency sums move by what the counts and extrema allow; nothing errs; the knobs echo. */
static void t_bad(struct sess *s, const struct mark *m, const char *why)
{
	s->t_err++;
	if (s->t_shown < 10) {
		s->t_shown++;
		say("torture: BAD record %u (%s): w0-7 %08x %08x %08x %08x %08x %08x %08x %08x\n", s->marks, why, m->w[0],
		    m->w[1], m->w[2], m->w[3], m->w[4], m->w[5], m->w[6], m->w[7]);
	}
}

static void t_check(struct sess *s, const struct mark *m)
{
	const struct mark *p = &s->last;
	uint32_t hr = 0, hw = 0, k, dr, dw, rmin = m->w[3] & 0xFFFF, rmax = m->w[3] >> 16, wmin = m->w[4] & 0xFFFF,
	         wmax = m->w[4] >> 16;
	uint64_t drs, dws;
	for (k = 0; k < 16; k++) { hr += m->w[8 + k]; hw += m->w[24 + k]; }
	if (hr != m->w[1] || hw != m->w[2]) t_bad(s, m, "histogram sum");
	if (m->w[7] != 0) t_bad(s, m, "error count");
	if (((m->w[0] >> 16) & 0xFFu) != (uint32_t)(s->ns | (s->flood << 1)) || (m->w[0] >> 24) != (s->state & 0xFF))
		t_bad(s, m, "word 0");
	if ((m->w[1] && rmin > rmax) || (m->w[2] && wmin > wmax)) t_bad(s, m, "min above max");
	if (s->marks == 0) return;
	if (m->w[2] != p->w[2] + 1u) t_bad(s, m, "write sequence");
	if (m->w[1] < p->w[1]) t_bad(s, m, "read count fell");
	if (rmin > (p->w[3] & 0xFFFF) || rmax < (p->w[3] >> 16) || wmin > (p->w[4] & 0xFFFF) || wmax < (p->w[4] >> 16))
		t_bad(s, m, "extrema moved inward");
	for (k = 0; k < 32; k++) if (m->w[8 + k] < p->w[8 + k]) { t_bad(s, m, "a bin fell"); break; }
	dr = m->w[1] - p->w[1];
	dw = m->w[2] - p->w[2];
	drs = (uint32_t)(m->w[5] - p->w[5]);
	dws = (uint32_t)(m->w[6] - p->w[6]);
	if (drs < (uint64_t)dr * rmin || drs > (uint64_t)dr * rmax) t_bad(s, m, "read sum");
	if (dws < (uint64_t)dw * wmin || dws > (uint64_t)dw * wmax) t_bad(s, m, "write sum");
}

/* one drain pass of the EVT ring; returns -1 on a protocol fault */
static int drain(struct sess *s, uint16_t *tail)
{
	uint16_t head = (uint16_t)OC(C_IDX(R_EVT));
	unsigned waiting = (uint16_t)(head - *tail), k;
	if (waiting > EVT_WORDS) {
		check(0, "EVT: head %u tail %u: %u words waiting, more than the ring", head, *tail, waiting);
		return -1;
	}
	while (waiting) {
		uint32_t hdr = OC(EVT_OFF + 4u * (*tail & (EVT_WORDS - 1)));
		unsigned kind = hdr >> 24, len = hdr & 0xFFFF;
		if (1u + len > waiting) {
			check(0, "EVT: record 0x%08x at tail %u runs past the head %u", hdr, *tail, head);
			return -1;
		}
		if (kind == K_MARK && len == MARK_LEN && ((hdr >> 16) & 0xFF) == R_EVT) {
			struct mark m;
			for (k = 0; k < MARK_LEN; k++) m.w[k] = OC(EVT_OFF + 4u * ((*tail + 1u + k) & (EVT_WORDS - 1)));
			if (s->t_words) t_check(s, &m);
			if (s->marks == 0) s->first = m;
			else {
				/* counters only grow within a session (the sums may wrap) */
				if (m.w[1] < s->last.w[1] || m.w[2] < s->last.w[2]) s->faults++;
			}
			s->last = m;
			s->marks++;
		} else if (kind == K_VERDICT && len == VERDICT_LEN && ((hdr >> 16) & 0xFF) == VERDICT_ARG) {
			for (k = 0; k < VERDICT_LEN; k++)
				s->verdict[k] = OC(EVT_OFF + 4u * ((*tail + 1u + k) & (EVT_WORDS - 1)));
			s->verdicts++;
		} else {
			s->other++;
			if (s->t_words) { s->t_err++; say("torture: BAD record header 0x%08x at tail %u\n", hdr, *tail); }
		}
		s->records++;
		s->words += 1u + len;
		*tail = (uint16_t)(*tail + 1u + len);
		waiting -= 1u + len;
	}
	OC(H_IDX(R_EVT)) = *tail;
	return 0;
}

static int session(struct sess *s, const char *tag)
{
	uint32_t edge0, last, hello, nonce, v;
	uint16_t tail = 0;
	int64_t t, t_end;
	unsigned k, hs, hr, hw;
	int rc = 0;

	say("== %s: ns=%d flood=%d line=%d seconds=%d drain_ms=%d\n", tag, s->ns, s->flood, s->line, s->seconds,
	    s->drain_ms);
	if (prepare()) return 1;
	/* the RUN edge (§2.5 step 2, fpga_ocram_run): a 0 store, then a 1 store; the knobs ride both stores */
	edge0 = gpo_base | (s->ns ? GPO_DIAG_NS : 0) | (s->flood ? GPO_DIAG_FL : 0);
	gpo_store(edge0);
	gpo_store(edge0 | GPO_RUN);
	t = await_c_hello(V_ARMED, &last);
	s->armed = t >= 0;
	s->t_arm = t;
	check(s->armed, "%s: armed value 0x4F430000 %s (C_HELLO 0x%08x, %lld ns after the edge)", tag,
	      s->armed ? "read" : "NOT read within 1 ms", last, (long long)t);
	if (!s->armed) {
		s->stopped = quiesce(tag);
		check(s->stopped, "%s: stopped value after the failed arm", tag);
		say("%s: eccgrp.ocram=0x%08x\n", tag, REG(sys, 0x144));
		return 1;
	}
	OC(H_LINE) = s->line ? (1u << R_EVT) : 0u;
	nonce = ((uint32_t)now_ns() ^ (uint32_t)getpid()) & 0xFFFFu;
	if (!nonce) nonce = 1;
	hello = 0x4F430000u | nonce;
	OC(H_HELLO) = hello;   /* stored last */
	t = await_c_hello(hello, &last);
	s->echoed = t >= 0;
	s->t_echo = t;
	check(s->echoed, "%s: the echo 0x%08x %s (C_HELLO 0x%08x, %lld ns)", tag, hello,
	      s->echoed ? "read" : "NOT read within 1 ms", last, (long long)t);
	if (!s->echoed) { rc = 1; goto stop; }
	check_hello(s, tag);

	t_end = now_ns() + (int64_t)s->seconds * 1000000000;
	{ int64_t t0 = now_ns(), t_note = t0 + 10000000000LL;
	while (now_ns() < t_end && (!s->t_words || s->words < s->t_words)) {
		if (s->t_words && now_ns() >= t_note) {
			t_note += 10000000000LL;
			say("%s: %llu words, %u records, %llu bad, %.2f M words/s\n", tag, (unsigned long long)s->words, s->marks,
			    (unsigned long long)s->t_err, s->words / ((now_ns() - t0) / 1e3));
		}
		if (s->line) {
			uint16_t head;
			sleep_ms(s->drain_ms);
			head = (uint16_t)OC(C_IDX(R_EVT));
			if (head != tail) { s->gic_hi_n++; s->gic_hi_ok += gic_pending(); }
			if (drain(s, &tail)) { rc = 1; break; }
			{ int64_t w = now_ns() + 50000; while (now_ns() < w) {} }   /* 50 us: the line falls in a few transactions */
			if ((uint16_t)OC(C_IDX(R_EVT)) == tail) { s->gic_lo_n++; s->gic_lo_ok += !gic_pending(); }
		} else {
			if (!s->flood) sleep_ms(s->drain_ms);
			if (drain(s, &tail)) { rc = 1; break; }
			if (gic_pending()) s->gic_hi_n++;   /* with H_LINE 0 the line must never rise */
		}
		v = OC(C_HELLO);
		if (v != hello) { check(0, "%s: C_HELLO changed under the session: 0x%08x", tag, v); rc = 1; break; }
	}
	if (s->t_words) {
		double sec = (now_ns() - t0) / 1e9;
		check(s->words >= s->t_words, "%s: torture: %llu of %llu words drained in %.1f s (%.2f M words/s)", tag,
		      (unsigned long long)s->words, (unsigned long long)s->t_words, sec, s->words / sec / 1e6);
		check(s->t_err == 0, "%s: torture: %llu invariant violations over %u records (sequence, checksums, monotonicity)",
		      tag, (unsigned long long)s->t_err, s->marks);
	}
	}
	v = OC(C_STATE);
	check(((v >> 8) & 0x1F) == 0, "%s: no ring stopped on a fault during the session (C_STATE 0x%08x)", tag, v);
stop:
	s->stopped = quiesce(tag);
	check(s->stopped, "%s: the stop proof (0x51540000 within 1 ms of RUN falling)", tag);
	if (!s->echoed) return 1;
	check(s->marks > 0, "%s: %u MARK records, %u records, %llu words drained, %u other", tag, s->marks, s->records,
	      (unsigned long long)s->words, s->other);
	check(s->faults == 0, "%s: counters never went backwards between MARKs (%u)", tag, s->faults);
	if (s->marks) {
		const struct mark *m = &s->last;
		hs = 0; hr = 0; hw = 0;
		for (k = 0; k < 16; k++) { hr += m->w[8 + k]; hw += m->w[24 + k]; }
		hs = (hr == m->w[1]) && (hw == m->w[2]);
		check(hs, "%s: last MARK: histogram sums equal the counts (rd %u/%u, wr %u/%u)", tag, hr, m->w[1], hw, m->w[2]);
		check(((m->w[0] >> 16) & 1u) == (unsigned)s->ns && ((m->w[0] >> 17) & 1u) == (unsigned)s->flood,
		      "%s: MARK echoes the knobs (ns %u flood %u)", tag, (m->w[0] >> 16) & 1, (m->w[0] >> 17) & 1);
		check((m->w[0] >> 24) == (s->state & 0xFF), "%s: MARK's hello count %u equals C_STATE's %u", tag, m->w[0] >> 24,
		      s->state & 0xFF);
		check((m->w[7] >> 16) == 0 && (m->w[7] & 0xFFFF) == 0, "%s: no error response (rd %u, wr %u)", tag, m->w[7] >> 16,
		      m->w[7] & 0xFFFF);
		print_mark(tag, m);
	}
	if (s->line) {
		say("%s: GIC 83 pending with records waiting: %u of %u samples; clear after the drain: %u of %u (UNVERIFIED witness)\n",
		    tag, s->gic_hi_ok, s->gic_hi_n, s->gic_lo_ok, s->gic_lo_n);
	} else {
		say("%s: GIC 83 pending with H_LINE 0: %u samples (expect 0; UNVERIFIED witness)\n", tag, s->gic_hi_n);
	}
	say("%s: eccgrp.ocram=0x%08x warmram.enable=0x%08x\n", tag, REG(sys, 0x144), REG(sys, 0xE0));
	say("%s: rate %.1f MARK/s, %.0f words/s\n", tag, s->marks / (double)s->seconds, s->words / (double)s->seconds);
	return rc || n_fail;
}

/* ================= M7: the OSD tier, the host-to-fabric torture and the freeze knob ================= */

/* a C_IDX read until it equals want, bounded; returns the ns taken or -1 with *last the last value */
static int64_t await_c_idx(unsigned ring, uint16_t want, int64_t bound_ns, uint16_t *last)
{
	int64_t t0 = now_ns(), t;
	uint16_t v;
	do {
		v = (uint16_t)OC(C_IDX(ring));
		t = now_ns() - t0;
		if (v == want) { *last = v; return t; }
	} while (t < bound_ns);
	*last = v;
	return -1;
}

/* a C_BIND read until every bit in mask reads 1, bounded (the bind takes one poll period plus the 1 ms bound) */
static int64_t await_c_bind(uint32_t mask, int64_t bound_ns, uint32_t *last)
{
	int64_t t0 = now_ns(), t;
	uint32_t v;
	do {
		v = OC(C_BIND);
		t = now_ns() - t0;
		if ((v & mask) == mask) { *last = v; return t; }
	} while (t < bound_ns);
	*last = v;
	return -1;
}

/* the hello of a new M7 mode: prepare, the RUN edge, ARMED, H_LINE, the hello, the echo and its words */
static int begin_m7(struct sess *s, const char *tag)
{
	uint32_t edge0, last, hello, nonce;
	int64_t t;
	if (prepare()) return 1;
	edge0 = gpo_base;
	gpo_store(edge0);
	gpo_store(edge0 | GPO_RUN);
	t = await_c_hello(V_ARMED, &last);
	s->armed = t >= 0;
	s->t_arm = t;
	check(s->armed, "%s: armed value 0x4F430000 %s (C_HELLO 0x%08x, %lld ns after the edge)", tag,
	      s->armed ? "read" : "NOT read within 1 ms", last, (long long)t);
	if (!s->armed) {
		s->stopped = quiesce(tag);
		check(s->stopped, "%s: stopped value after the failed arm", tag);
		return 1;
	}
	OC(H_LINE) = 1u << R_EVT;
	nonce = ((uint32_t)now_ns() ^ (uint32_t)getpid()) & 0xFFFFu;
	if (!nonce) nonce = 1;
	hello = 0x4F430000u | nonce;
	s->hello = hello;
	OC(H_HELLO) = hello;
	t = await_c_hello(hello, &last);
	s->echoed = t >= 0;
	s->t_echo = t;
	check(s->echoed, "%s: the echo 0x%08x %s (C_HELLO 0x%08x, %lld ns)", tag, hello,
	      s->echoed ? "read" : "NOT read within 1 ms", last, (long long)t);
	if (!s->echoed) {
		s->stopped = quiesce(tag);
		check(s->stopped, "%s: stopped value after the failed echo", tag);
		return 1;
	}
	check_hello(s, tag);
	return 0;
}

/* RUN 0 with FREEZE left as it is, then the stop proof (the freeze mode's intermediate stops) */
static int stop_run(const char *tag, int64_t *t_out)
{
	uint32_t last;
	int64_t t0 = now_ns(), t;
	gpo_store(gpo_shadow & ~GPO_RUN);
	t = await_c_hello(V_STOPPED, &last);
	if (t_out) *t_out = now_ns() - t0;
	say("%s: RUN 0 -> stopped value %s (C_HELLO 0x%08x, %lld ns)\n", tag, t >= 0 ? "read" : "NOT read within 1 ms",
	    last, (long long)t);
	return t >= 0;
}

/* the final quiesce of a new mode: FREEZE and RUN cleared, the stop proof checked, RECOVERY on a miss */
static void end_m7(struct sess *s, const char *tag)
{
	int froze = (gpo_shadow & GPO_FREEZE) != 0;
	if (froze) say("RECOVERY: %s: exiting with FREEZE set; clearing it (a word may be in flight to the OSD half)\n", tag);
	s->stopped = quiesce(tag);
	check(s->stopped, "%s: the stop proof (0x51540000 within 1 ms of RUN falling)", tag);
	if (!s->stopped)
		say("RECOVERY: %s: no stop proof: do NOT run Main, load a core or reset a bridge until the operator has read "
		    "the log; if the next 'look' hangs, power-cycle (attended)\n", tag);
}

static const char *pat_name(int pat)
{
	return pat == PAT_BARS ? "bars" : pat == PAT_INC ? "inc" : "rows";
}

static uint8_t pattern_byte(int pat, unsigned r, unsigned c)
{
	switch (pat) {
	case PAT_BARS: return (uint8_t)(((c >> 4) & 1u) ? 0xFFu : 0x00u);   /* 16-pixel vertical bars */
	case PAT_INC:  return (uint8_t)((r * 256u + c) & 0xFFu);
	default:       return (uint8_t)('A' + (r + c) % 26u);
	}
}

static void fill_row(uint32_t *w, int pat, unsigned r)
{
	unsigned k, b;
	for (k = 0; k < ROW_LEN; k++) {
		uint32_t v = 0;
		for (b = 0; b < 4; b++) v |= (uint32_t)pattern_byte(pat, r, 4 * k + b) << (8 * b);
		w[k] = v;
	}
}

/* stock moves a row in 8-word STM blocks; the head is a single STR (design note §3, "Copies") */
static void store8_stm(volatile uint32_t *dst, const uint32_t *src)
{
#if defined(__arm__) && !defined(OCRAM_FAKE)
	__asm__ volatile(
		"ldr r1, [%1, #0]\n\t"
		"ldr r2, [%1, #4]\n\t"
		"ldr r3, [%1, #8]\n\t"
		"ldr r4, [%1, #12]\n\t"
		"ldr r5, [%1, #16]\n\t"
		"ldr r6, [%1, #20]\n\t"
		"ldr r7, [%1, #24]\n\t"
		"ldr r8, [%1, #28]\n\t"
		"stm %0, {r1-r8}\n\t"
		:: "r"(dst), "r"(src)
		: "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "memory");
#else
	unsigned i;
	for (i = 0; i < 8; i++) dst[i] = src[i];
#endif
}

/* 64 payload words at ring index idx, split at the ring's end so every STM stays contiguous */
static void store_row(volatile uint32_t *base, uint16_t idx, const uint32_t *w)
{
	unsigned k = 0, pos = idx & (OSD_WORDS - 1);
	while (k < ROW_LEN) {
		if (pos + 8 <= OSD_WORDS) { store8_stm(base + pos, w + k); k += 8; pos += 8; }
		else { base[pos] = w[k]; k++; pos = (pos + 1) & (OSD_WORDS - 1); }
	}
}

/* publish nrows ROW records (one head store); returns the new head. head_t is the head store's start */
static uint16_t osd_put_rows(uint16_t head, unsigned nrows, int pat, uint64_t *row_ns, uint64_t *head_ns,
                             int64_t *head_t)
{
	uint32_t w[ROW_LEN];
	unsigned r;
	for (r = 0; r < nrows; r++) {
		int64_t t0;
		fill_row(w, pat, r);
		t0 = now_ns();
		OC(OSD_OFF + 4u * (head & (OSD_WORDS - 1))) = (K_ROW << 24) | (r << 16) | ROW_LEN;
		store_row(&OC(OSD_OFF), (uint16_t)(head + 1u), w);
		if (row_ns) *row_ns += (uint64_t)(now_ns() - t0);
		head = (uint16_t)(head + 1u + ROW_LEN);
	}
	{
		int64_t t0 = now_ns();
		if (head_t) *head_t = t0;
		OC(H_IDX(R_OSD)) = head;
		if (head_ns) *head_ns += (uint64_t)(now_ns() - t0);
	}
	return head;
}

static void print_us(const char *tag, const char *what, uint64_t min, uint64_t max, uint64_t sum, unsigned n)
{
	say("%s: %s: min %.2f us, max %.2f us, mean %.2f us (%u samples)\n", tag, what,
	    n ? min / 1000.0 : 0.0, n ? max / 1000.0 : 0.0, n ? sum / 1000.0 / n : 0.0, n);
}

#ifdef OCRAM_FAKE
static unsigned fake_osd_diff(int pat, unsigned rows)
{
	unsigned r, c;
	if (rows > 32) return 1;
	for (r = 0; r < rows; r++)
		for (c = 0; c < 256; c++)
			if (f_osd[r * 256 + c] != pattern_byte(pat, r, c)) return 1u + r * 256u + c;
	return 0;
}

static void fake_osd_say_diff(const char *tag, int pat, unsigned rows)
{
	unsigned d = fake_osd_diff(pat, rows);
	if (!d) { say("%s: no fake OSD difference\n", tag); return; }
	d--;
	say("%s: first fake OSD difference: row %u col %u got 0x%02x want 0x%02x\n", tag, d / 256u, d % 256u,
	    f_osd[d], pattern_byte(pat, d / 256u, d % 256u));
}
#endif

/* ---- osd: the OSD ring, a one-row latency and a whole-repaint latency, then READY-FOR-EYES ---- */
static int osd_mode(const char *tag, unsigned rows, unsigned repeat, int pat, int hold_s, int no_enable)
{
	struct sess s;
	uint32_t last;
	uint16_t head = 0, tail = 0, evt_tail = 0;
	uint64_t one_min = 0, one_max = 0, one_sum = 0, whole_min = 0, whole_max = 0, whole_sum = 0;
	uint64_t row_ns = 0, head_ns = 0;
	unsigned i, one_n = 0, whole_n = 0;
	int rc = 0;
	memset(&s, 0, sizeof s);
	say("== osd: rows=%u repeat=%u pattern=%s hold=%ds enable=%d\n", rows, repeat, pat_name(pat), hold_s, !no_enable);
	if (begin_m7(&s, tag)) return 1;
	if (rows < 1 || rows > s.rows_max) {
		check(0, "%s: --rows %u is outside C_K[8]'s 1..%u", tag, rows, s.rows_max);
		end_m7(&s, tag);
		return 1;
	}
	OC(H_BIND) = 1u << 11;
	{
		int64_t t = await_c_bind(1u << 11, BIND_NS, &last);
		check(t >= 0, "%s: C_BIND bit 11 %s (0x%08x, %lld ns after H_BIND's store)", tag,
		      t >= 0 ? "set" : "NOT set within 2 ms", last, (long long)t);
		if (t >= 0) check((last & ~(1u << 11)) == 0, "%s: C_BIND 0x%08x (bit 11 alone)", tag, last);
		if (t < 0) { end_m7(&s, tag); return 1; }
	}
	if (!no_enable) {
		if (spi_osd_cmd(SPI_OSD_ENABLE)) { check(0, "%s: the SPI OSD enable timed out", tag); end_m7(&s, tag); return 1; }
		say("%s: SPI OSD enable 0x41 sent (the ring is drained)\n", tag);
	}
	if ((uint16_t)OC(C_IDX(R_OSD)) != head) {
		check(0, "%s: C_IDX[2] %u is not the empty ring's 0", tag, (unsigned)(uint16_t)OC(C_IDX(R_OSD)));
		end_m7(&s, tag);
		return 1;
	}
	for (i = 0; i < repeat && !rc; i++) {
		int64_t t;
		/* (a) one row alone */
		head = osd_put_rows(head, 1, pat, &row_ns, &head_ns, NULL);
		t = await_c_idx(R_OSD, head, OSD_WAIT_NS, &tail);
		if (t < 0) {
			check(0, "%s: repaint %u: C_IDX[2] did not reach the one-row head %u within 2 s (tail %u)", tag, i,
			      (unsigned)head, (unsigned)tail);
			rc = 1;
			break;
		}
		if (!one_n || (uint64_t)t < one_min) one_min = (uint64_t)t;
		if (!one_n || (uint64_t)t > one_max) one_max = (uint64_t)t;
		one_sum += (uint64_t)t;
		one_n++;
		/* (b) the whole repaint in one head store */
		head = osd_put_rows(head, rows, pat, &row_ns, &head_ns, NULL);
		t = await_c_idx(R_OSD, head, OSD_WAIT_NS, &tail);
		if (t < 0) {
			check(0, "%s: repaint %u: C_IDX[2] did not reach the %u-row head %u within 2 s (tail %u)", tag, i, rows,
			      (unsigned)head, (unsigned)tail);
			rc = 1;
			break;
		}
		if (!whole_n || (uint64_t)t < whole_min) whole_min = (uint64_t)t;
		if (!whole_n || (uint64_t)t > whole_max) whole_max = (uint64_t)t;
		whole_sum += (uint64_t)t;
		whole_n++;
		if (drain(&s, &evt_tail)) { rc = 1; break; }
	}
	{
		unsigned nrows = repeat * (1u + rows);
		print_us(tag, "one row alone (head store -> C_IDX[2] = head)", one_min, one_max, one_sum, one_n);
		print_us(tag, "whole repaint (head store -> C_IDX[2] = head)", whole_min, whole_max, whole_sum, whole_n);
		say("%s: host store per row: %.3f us (%llu ns over %u rows); head store: %.3f us (%llu ns over %u)\n", tag,
		    nrows ? (double)row_ns / 1000.0 / nrows : 0.0, (unsigned long long)row_ns, nrows,
		    repeat ? (double)head_ns / 1000.0 / (2u * repeat) : 0.0, (unsigned long long)head_ns, 2u * repeat);
	}
	check((uint16_t)OC(C_IDX(R_OSD)) == head, "%s: C_IDX[2] ends at the head (%u)", tag, (unsigned)head);
#ifdef OCRAM_FAKE
	check(fake_osd_diff(pat, rows) == 0, "%s: the fake osd_buffer holds the last %u-row %s repaint", tag, rows,
	      pat_name(pat));
	if (fake_osd_diff(pat, rows)) fake_osd_say_diff(tag, pat, rows);
	say("%s: fake osd rows written %u (C_K rows %u)\n", tag, f_osd_rows, s.rows_max);
#endif
	say("%s: READY-FOR-EYES (the last repaint stays on the OSD; hold %d s)\n", tag, hold_s);
	if (hold_s > 0) sleep_ms((long)hold_s * 1000L);
	end_m7(&s, tag);
	return rc || n_fail ? 1 : 0;
}

/* ---- spi-osd / fallback / osd-off: the stock SPI OSD path, no RUN edge ---- */
static int spi_guard(const char *tag)
{
	uint32_t g = gpo_read(), i;
	if (main_running()) {
		say("%s: a MiSTer process is running: it owns GPO; nothing stored\n", tag);
		return 3;
	}
	if (!(g & GPO_IO_VIEW)) {
		say("%s: GPO 0x%08x: not the I/O view; nothing stored\n", tag, g);
		return 3;
	}
	if (g & GPO_RESET) {
		say("%s: GPO 0x%08x: the core is held in reset; nothing stored\n", tag, g);
		return 3;
	}
	if (g & GPO_RUN) {
		say("%s: GPO 28 is set: the fallback runs before any RUN edge; nothing stored\n", tag);
		return 3;
	}
	i = gpi_read();
	if (i & GPI_UNCONF) {
		say("%s: GPI 0x%08x: the FPGA is not configured; nothing stored\n", tag, i);
		return 3;
	}
	if (g & GPO_FREEZE) say("%s: note: GPO 25 was set; clearing it\n", tag);
	gpo_shadow = g & ~(GPO_RUN | GPO_FREEZE);
	say("%s: gpo=0x%08x gpi=0x%08x (no RUN edge)\n", tag, g, i);
	return 0;
}

static int spi_osd_mode(const char *tag, unsigned rows, int pat, int hold_s)
{
	uint32_t before[PAGE_LEN / 4], after[PAGE_LEN / 4], chello0, chello1;
	uint8_t bytes[256];
	unsigned r, k, differ = 0, first_diff = 0;
	int64_t t_rep, t0;
	int rc;
	if (rows < 1 || rows > MAX_ROWS) {
		say("%s: --rows %u is outside 1..%u\n", tag, rows, MAX_ROWS);
		return 2;
	}
	if ((rc = spi_guard(tag)) != 0) return rc;
	REG(sys, 0x144) = ECC_FLAGS;   /* the ECC flags before the first OCRAM read (V20) */
	say("%s: eccgrp.ocram after the W1C clear: 0x%08x\n", tag, REG(sys, 0x144));
	for (k = 0; k < PAGE_LEN / 4; k++) before[k] = OC(PAGE_OFF + 4 * k);
	chello0 = OC(C_HELLO);
	if (spi_osd_cmd(SPI_OSD_ENABLE)) { check(0, "%s: the SPI OSD enable timed out", tag); return 1; }
	say("%s: SPI OSD enable 0x41 sent\n", tag);
	t_rep = now_ns();
	for (r = 0; r < rows; r++) {
		for (k = 0; k < 256; k++) bytes[k] = pattern_byte(pat, r, k);
		t0 = now_ns();
		if (spi_osd_row(r, bytes)) { check(0, "%s: row %u: the SPI transfer timed out", tag, r); return 1; }
		say("%s: row %2u: %.3f ms (%s)\n", tag, r, (now_ns() - t0) / 1e6, pat_name(pat));
	}
	t_rep = now_ns() - t_rep;
	for (k = 0; k < PAGE_LEN / 4; k++) after[k] = OC(PAGE_OFF + 4 * k);
	chello1 = OC(C_HELLO);
	for (k = 0; k < PAGE_LEN / 4; k++) if (before[k] != after[k]) { if (!differ) first_diff = k; differ++; }
	check(differ == 0, "%s: the whole control page untouched by the master (%u words differ)", tag, differ);
	if (differ) say("%s: first difference at 0x%02x: 0x%08x -> 0x%08x\n", tag, 4u * first_diff, before[first_diff],
	                after[first_diff]);
	check(chello0 == chello1, "%s: C_HELLO before 0x%08x and after 0x%08x are equal", tag, chello0, chello1);
	say("%s: repaint of %u rows: %.3f ms (%.3f ms/row)\n", tag, rows, t_rep / 1e6, (double)t_rep / 1e6 / rows);
	check(!io_err, "%s: every SPI strobe was acknowledged", tag);
	say("%s: READY-FOR-EYES (the SPI rows stay on the OSD; hold %d s)\n", tag, hold_s);
	if (hold_s > 0) sleep_ms((long)hold_s * 1000L);
	look();
#ifdef OCRAM_FAKE
	sleep_ms(2);
	check(fake_osd_diff(pat, rows) == 0, "%s: the fake osd_buffer holds the %u-row %s repaint", tag, rows, pat_name(pat));
	if (fake_osd_diff(pat, rows)) fake_osd_say_diff(tag, pat, rows);
	check(f_osd_enable == 1, "%s: the fake OSD is enabled (0x41 took effect)", tag);
#endif
	return n_fail ? 1 : 0;
}

static int osd_off_mode(const char *tag)
{
	int rc = spi_guard(tag);
	if (rc) return rc;
	if (spi_osd_cmd(SPI_OSD_DISABLE)) { check(0, "%s: the SPI OSD disable timed out", tag); return 1; }
	say("%s: SPI OSD disable 0x40 sent\n", tag);
#ifdef OCRAM_FAKE
	sleep_ms(2);
	check(f_osd_enable == 0, "%s: the fake OSD is disabled (0x40 took effect)", tag);
#endif
	return n_fail ? 1 : 0;
}

/* ---- the host-to-fabric torture (M4's other half): TSEQ records, the RTL's VERDICT checks ---- */
static const char *map_name(int map)
{
	switch (map) {
	case 1: return "wc";
	case 2: return "wc-dmb";
	case 3: return "wc-dsb";
	case 4: return "wc-wmb";
	default: return "so";
	}
}

static uint32_t v_word(uint32_t g)
{
	return g ^ ((g << 16) | (g >> 16)) ^ 0x9E3779B9u;
}

static void vcheck(int ok, const char *fmt, ...)
{
	va_list a;
	if (!ok) n_vfail++;
	fputs(ok ? "pass " : "FAIL ", stdout);
	va_start(a, fmt);
	vprintf(fmt, a);
	va_end(a);
	fputc('\n', stdout);
	fflush(stdout);
}

static void ts_store(int wcmap, uint16_t idx, uint32_t v)
{
	unsigned off = TS_OFF + 4u * (idx & (TS_WORDS - 1));
	if (wcmap) wc[off / 4] = v;
	else OC(off) = v;
}

static int torture_h2f(struct sess *s, const char *tag)
{
	uint16_t head = 0, tail, evt_tail = 0;
	uint64_t t_start, t_print, t_drain, t_wait;
	uint32_t last_state;
	unsigned v0 = n_vfail, f0 = n_fail;
	int wcmap = s->map != 0, verdict_got = 0, ring_fault = 0, drained = 0;
	unsigned i;

	say("== torture (host to fabric): map=%s words=%llu maxlen=%u\n", map_name(s->map),
	    (unsigned long long)s->words_target_h2f, s->maxlen);
	if (wcmap) open_wc();
	if (begin_m7(s, tag)) return 1;
	if (s->maxlen < 1 || s->maxlen > MAX_MAXLEN) {
		check(0, "%s: --maxlen %u is outside 1..%u", tag, s->maxlen, MAX_MAXLEN);
		end_m7(s, tag);
		return 1;
	}
	t_start = now_ns();
	t_print = t_start;
	t_drain = t_start;
	while (s->pub_words < s->words_target_h2f) {
		uint32_t len;
		unsigned room;
		uint16_t used;
		int64_t now;
		tail = (uint16_t)OC(C_IDX(R_TS));
		used = (uint16_t)(head - tail);
		if (used > TS_WORDS) {
			check(0, "%s: C_IDX[4] %u is more than the ring behind the head %u", tag, (unsigned)tail, (unsigned)head);
			drained = -1;
			break;
		}
		room = TS_WORDS - used;
		len = 1u + (unsigned)(rand() % (int)s->maxlen);
		now = now_ns();
		if (now - t_drain >= 10000000LL) {
			if (drain(s, &evt_tail)) { drained = -1; break; }
			t_drain = now;
		}
		if (now - t_print >= 10000000000LL) {
			uint32_t *vv = s->verdict;
			double sec = (double)(now - t_start) / 1e9;
			say("%s: %.1f s: %llu records, %llu words, %.2f M words/s; last VERDICT records=%u words=%u errors=%u "
			    "first_g=%u sum=0x%08x (%u verdicts)\n", tag, sec, (unsigned long long)s->pub_records,
			    (unsigned long long)s->pub_words, sec ? s->pub_words / sec / 1e6 : 0.0, vv[0], vv[1], vv[2], vv[3],
			    vv[5], s->verdicts);
			t_print = now;
		}
		if (1u + len > room) continue;   /* wait for C_IDX[4] to leave room, draining on the way */
		ts_store(wcmap, head, (K_TSEQ << 24) | (((s->pub_records & 0xFFu) << 16)) | (len & 0xFFFFu));
		for (i = 0; i < len; i++) {
			uint32_t g = (uint32_t)s->pub_words;
			uint32_t v = v_word(g);
			ts_store(wcmap, (uint16_t)(head + 1u + i), v);
			s->pub_sum += v;
			s->pub_words++;
		}
		head = (uint16_t)(head + 1u + len);
		if (wcmap) { wc_sync(s->map); wc[H_IDX(R_TS) / 4] = head; }
		else OC(H_IDX(R_TS)) = head;
		{
			uint16_t fill = (uint16_t)(head - tail);
			if (fill > s->pub_maxfill) s->pub_maxfill = fill;
		}
		s->pub_records++;
	}
	if (drained != -1) {
		/* the sink consumes every published record, then C_IDX[4] equals the head */
		t_wait = now_ns();
		for (;;) {
			tail = (uint16_t)OC(C_IDX(R_TS));
			if (drain(s, &evt_tail)) { drained = -1; break; }
			if (tail == head) break;
			if (now_ns() - t_wait >= TORT_WAIT_NS) {
				say("%s: C_IDX[4] did not reach the head %u within 5 s (tail %u, %u words behind)\n", tag,
				    (unsigned)head, (unsigned)tail, (unsigned)(uint16_t)(head - tail));
				break;
			}
			sleep_ms(1);
		}
	}
	last_state = OC(C_STATE);
	if (last_state & (1u << (8 + R_TS))) {
		ring_fault = 1;
		say("%s: C_STATE 0x%08x: the torture ring stopped on a fault\n", tag, last_state);
	}
	if (drained != -1) {
		unsigned before = s->verdicts;
		int64_t tw = now_ns();
		while (s->verdicts == before && now_ns() - tw < VERDICT_WAIT_NS) {
			if (drain(s, &evt_tail)) { drained = -1; break; }
			if (s->verdicts == before) sleep_ms(1);
		}
		verdict_got = s->verdicts > before;
	}
	say("%s: %llu records, %llu payload words, host sum 0x%08x, deepest fill %u of %u; VERDICTs %u\n", tag,
	    (unsigned long long)s->pub_records, (unsigned long long)s->pub_words, s->pub_sum, s->pub_maxfill, TS_WORDS,
	    s->verdicts);
	if (!verdict_got) {
		check(0, "%s: no VERDICT after the ring drained", tag);
	} else {
		uint32_t *v = s->verdict;
		vcheck(v[0] == s->pub_records, "%s: VERDICT records %u == %llu published", tag, v[0],
		       (unsigned long long)s->pub_records);
		vcheck(v[1] == s->pub_words, "%s: VERDICT words %u == %llu published", tag, v[1],
		       (unsigned long long)s->pub_words);
		vcheck(v[2] == 0, "%s: VERDICT errors %u == 0", tag, v[2]);
		vcheck(v[3] == 0, "%s: VERDICT first_g %u == 0", tag, v[3]);
		vcheck(v[4] == 0, "%s: VERDICT first_got 0x%08x == 0", tag, v[4]);
		vcheck(v[5] == s->pub_sum, "%s: VERDICT sum 0x%08x == host sum 0x%08x", tag, v[5], s->pub_sum);
		say("%s: VERDICT skipped %u, deepest %u; osd_rows %u, osd_deepest %u\n", tag, v[6] >> 16, v[6] & 0xFFFFu,
		    v[7] >> 16, v[7] & 0xFFFFu);
	}
	{
		int verdict_bad = (n_vfail != v0) || ring_fault;
		unsigned probe_fail;
		int probe_bad, rc;
		end_m7(s, tag);
		probe_fail = n_fail - f0;
		probe_bad = probe_fail != 0 || !verdict_got || io_err || drained == -1;
		rc = probe_bad || (s->map == 0 && verdict_bad);
		if (probe_bad) say("%s: the probe itself misbehaved\n", tag);
		if (s->map == 0)
			say("RESULT torture (map so): %s (%u probe failures, %u verdict mismatches)\n",
			    rc ? "FAILED" : "PASSED", probe_fail, n_vfail - v0);
		else
			say("RESULT torture (map %s): %s (an error is a RESULT; %u probe failures, %u verdict mismatches%s)\n",
			    map_name(s->map), rc ? "FAILED (probe)" : (verdict_bad ? "RESULT: errors found" : "PASSED"),
			    probe_fail, n_vfail - v0, ring_fault ? ", ring fault" : "");
		return rc;
	}
}

/* ---- --band (E25): the torture ring held PART-full, so the host's next store is never behind a full ring ---- */
#define BAND_RATE_DEFAULT 6300000.0   /* the fabric's measured words/s (docs/OCRAM_MAILBOX.md, "Measured performance") */
#define BAND_MAXLEN_DEFAULT 256u
#define BAND_BINS 8u
#define BAND_BIN_WORDS (TS_WORDS / BAND_BINS)
#define BAND_EMPTY_WORDS_DEFAULT (1ULL << 20)
#define BAND_OUT_PCT 5u               /* more empty reads, or reads above BAND_HIGH_PCT, than this and the ring was not part-full */
#define BAND_HIGH_PCT 60u             /* LO tops out at 45% */
#define BAND_LOWER_BOUND_PCT 50.0     /* the guard assumes the fabric drains at least this share of its rate */

/* two streams, so the HI and LO of cycle n depend on the seed alone, not on how many records the clock allowed */
static uint64_t prng_len = 1, prng_band = 1;
static uint32_t prng(uint64_t *s)
{
	*s ^= *s >> 12;
	*s ^= *s << 25;
	*s ^= *s >> 27;
	return (uint32_t)((*s * 0x2545F4914F6CDD1DULL) >> 32);
}

/* sleeps to within 100 us of t, then spins; touches no mailbox word */
static void wait_until(int64_t t)
{
	for (;;) {
		int64_t d = t - now_ns();
		if (d <= 0) return;
		if (d > 200000) {
			struct timespec ts = { 0, (long)(d - 100000) };
			nanosleep(&ts, NULL);
		}
	}
}

static void band_hist(const char *what, const uint64_t *h)
{
	unsigned b;
	say("band: %s, 8 bins of %u words:", what, BAND_BIN_WORDS);
	for (b = 0; b < BAND_BINS; b++) say(" [%u-%u] %llu", b * BAND_BIN_WORDS, (b + 1) * BAND_BIN_WORDS - 1,
	                                    (unsigned long long)h[b]);
	say("\n");
}

/* the ring run empty, then the VERDICT that follows it checked against the host's own counts; 0 or -1 on a fault */
static int band_checkpoint(struct sess *s, uint16_t head, uint16_t *evt_tail, unsigned idx, int quietly)
{
	int64_t t0 = now_ns();
	uint16_t tail;
	const uint32_t *v = s->verdict;
	unsigned v0 = n_vfail;

	PHASE(PH_EMPTY);
	for (;;) {
		tail = (uint16_t)OC(C_IDX(R_TS));
		if (drain(s, evt_tail)) return -1;
		if (tail == head) break;
		if (now_ns() - t0 >= TORT_WAIT_NS) {
			check(0, "band: checkpoint %u: C_IDX[4] did not reach the head %u within 5 s (tail %u)", idx,
			      (unsigned)head, (unsigned)tail);
			return -1;
		}
		wait_until(now_ns() + 100000);
	}
	t0 = now_ns();
	for (;;) {
		if (drain(s, evt_tail)) return -1;
		if (s->verdicts && v[0] == s->pub_records && v[1] == s->pub_words) break;
		if (now_ns() - t0 >= VERDICT_WAIT_NS) break;
		wait_until(now_ns() + 200000);
	}
	if (!s->verdicts) { check(0, "band: checkpoint %u: no VERDICT after the ring ran empty", idx); return -1; }
	vcheck(v[0] == s->pub_records, "band: checkpoint %u: VERDICT records %u == %llu published", idx, v[0],
	       (unsigned long long)s->pub_records);
	vcheck(v[1] == s->pub_words, "band: checkpoint %u: VERDICT words %u == %llu published", idx, v[1],
	       (unsigned long long)s->pub_words);
	vcheck(v[2] == 0, "band: checkpoint %u: VERDICT errors %u == 0 (first_g %u, first_got 0x%08x)", idx, v[2], v[3], v[4]);
	vcheck(v[5] == s->pub_sum, "band: checkpoint %u: VERDICT sum 0x%08x == host sum 0x%08x", idx, v[5], s->pub_sum);
	vcheck((v[6] & 0xFFFFu) < TS_WORDS, "band: checkpoint %u: VERDICT deepest %u < %u", idx, v[6] & 0xFFFFu, TS_WORDS);
	if (!quietly || n_vfail != v0)
		say("band: checkpoint %u at %llu words: VERDICT records %u words %u errors %u deepest %u\n", idx,
		    (unsigned long long)s->pub_words, v[0], v[1], v[2], v[6] & 0xFFFFu);
	return 0;
}

static int torture_band(struct sess *s, const char *tag)
{
	uint16_t head = 0, tail = 0, tail_prev = 0, evt_tail = 0;
	uint64_t cycles = 0, reads = 0, empty_reads = 0, high_reads = 0, checkpoints = 0, next_cp = s->empty_words, hist_read[BAND_BINS] = { 0 },
	         hist_peak[BAND_BINS] = { 0 };
	unsigned min_read = TS_WORDS, max_peak = 0, v0 = n_vfail, f0 = n_fail;
	double rate = s->rate;
	int64_t t_start, t_print, win_t0 = 0;
	uint64_t win_words = 0;
	int wcmap = s->map != 0, fatal = 0, have_prev = 0, fresh = 1;

	say("== torture --band (host to fabric, ring part-full): map=%s words=%llu maxlen=%u rate=%.0f words/s seed=%llu "
	    "checkpoint every %llu words\n", map_name(s->map), (unsigned long long)s->words_target_h2f, s->maxlen,
	    s->rate, (unsigned long long)s->seed, (unsigned long long)s->empty_words);
	if (wcmap) open_wc();
	if (begin_m7(s, tag)) return 1;
	prng_len = s->seed * 0x9E3779B97F4A7C15ULL + 1;
	prng_band = s->seed * 0xD1B54A32D192ED03ULL + 3;
	t_start = now_ns();
	t_print = t_start;
	while (s->pub_words < s->words_target_h2f && !fatal) {
		int64_t t_read;
		unsigned fill, hi, lo, pub_since = 0, peak;

		/* the cycle's one mailbox read of the ring, then the EVT ring (another word) */
		PHASE(PH_READ);
		tail = (uint16_t)OC(C_IDX(R_TS));
		t_read = now_ns();
		fill = (uint16_t)(head - tail);
		if (fill >= TS_WORDS) {
			check(0, "%s: band: C_IDX[4] %u leaves the ring full or more behind the head %u (fill %u)", tag,
			      (unsigned)tail, (unsigned)head, fill);
			fatal = 1;
			break;
		}
		if (drain(s, &evt_tail)) { fatal = 1; break; }
		if (s->pub_words >= next_cp) {
			checkpoints++;
			if (band_checkpoint(s, head, &evt_tail, (unsigned)checkpoints, checkpoints % 64 != 1)) { fatal = 1; break; }
			next_cp += s->empty_words;
			have_prev = 0;
			fresh = 1;
			continue;
		}
		/* the rate: while every read finds words left, the ring never ran empty in between (it only falls while
		   the host waits), so the tail's travel over the window is the fabric's rate */
		if (have_prev && fill > 0) {
			double dt = (double)(t_read - win_t0) / 1e9;
			win_words += (uint16_t)(tail - tail_prev);
			if (dt >= 0.005) {
				rate = (double)win_words / dt;
				if (rate < 0.5 * s->rate) rate = 0.5 * s->rate;
				if (rate > 1.5 * s->rate) rate = 1.5 * s->rate;
			}
			if (dt >= 1.0) { win_t0 = t_read; win_words = 0; }
		} else {
			win_t0 = t_read;
			win_words = 0;
		}
		cycles++;
		if (fresh) {
			fresh = 0;   /* the first read after the ring was run empty finds it empty: not a band reading */
		} else {
			reads++;
			if (fill == 0) empty_reads++;
			if (fill * 100u > TS_WORDS * BAND_HIGH_PCT) high_reads++;
			if (fill < min_read) min_read = fill;
			hist_read[fill / BAND_BIN_WORDS]++;
		}
		hi = TS_WORDS * 50u / 100u + prng(&prng_band) % (TS_WORDS * 40u / 100u + 1u);
		lo = TS_WORDS * 5u / 100u + prng(&prng_band) % (TS_WORDS * 40u / 100u + 1u);

		/* publish back to back to HI by the clock's estimate: stores only, no read of any mailbox word */
		PHASE(PH_PUB);
		for (;;) {
			double el = (double)(now_ns() - t_read) / 1e9, est = (double)fill + pub_since - rate * el,
			       ub = (double)fill + pub_since - BAND_LOWER_BOUND_PCT / 100.0 * s->rate * el;
			unsigned room, len, i;
			if (est >= hi || s->pub_words >= s->words_target_h2f) break;
			room = ub <= 0 ? TS_WORDS - 1u : (TS_WORDS - 1u > (unsigned)ub ? TS_WORDS - 1u - (unsigned)ub : 0u);
			if (room < 2u) break;
			len = 1u + prng(&prng_len) % s->maxlen;
			if (len + 1u > room) len = room - 1u;
			ts_store(wcmap, head, (K_TSEQ << 24) | ((s->pub_records & 0xFFu) << 16) | (len & 0xFFFFu));
			for (i = 0; i < len; i++) {
				uint32_t g = (uint32_t)s->pub_words, v = v_word(g);
				ts_store(wcmap, (uint16_t)(head + 1u + i), v);
				s->pub_sum += v;
				s->pub_words++;
			}
			head = (uint16_t)(head + 1u + len);
			if (wcmap) { wc_sync(s->map); wc[H_IDX(R_TS) / 4] = head; }
			else OC(H_IDX(R_TS)) = head;
			pub_since += 1u + len;
			s->pub_records++;
		}
		{
			double el = (double)(now_ns() - t_read) / 1e9, est = (double)fill + pub_since - rate * el;
			peak = est <= 0 ? 0u : (est >= TS_WORDS ? TS_WORDS - 1u : (unsigned)est);
		}
		if (peak > max_peak) max_peak = peak;
		hist_peak[peak / BAND_BIN_WORDS]++;
		if (peak > s->pub_maxfill) s->pub_maxfill = peak;
		FAKE_BLOG(lo, hi, fill, peak);

		/* wait by the clock down to LO: no mailbox access at all */
		PHASE(PH_WAIT);
		if (fill + pub_since > lo) {
			wait_until(t_read + (int64_t)(((double)fill + pub_since - lo) / rate * 1e9));
		}
		PHASE(PH_NONE);
		tail_prev = tail;
		have_prev = 1;
		if (now_ns() - t_print >= 10000000000LL) {
			double sec = (double)(now_ns() - t_start) / 1e9;
			say("%s: band: %.1f s: %llu cycles, %llu records, %llu words, %.2f M words/s, rate estimate %.2f M, "
			    "lowest fill %u, empty reads %llu\n", tag, sec, (unsigned long long)cycles,
			    (unsigned long long)s->pub_records, (unsigned long long)s->pub_words, s->pub_words / sec / 1e6,
			    rate / 1e6, min_read, (unsigned long long)empty_reads);
			t_print = now_ns();
		}
	}
	if (!fatal) {
		/* the last checkpoint: the ring runs empty and the VERDICT must equal everything published */
		checkpoints++;
		if (band_checkpoint(s, head, &evt_tail, (unsigned)checkpoints, 0)) fatal = 1;
	}
	PHASE(PH_NONE);
	{
		unsigned probe_fail, vfail = n_vfail - v0;
		int probe_bad, rc, part_full, verdict_bad, c_fault = (OC(C_STATE) & (1u << (8 + R_TS))) != 0;
		uint32_t deepest = s->verdict[6] & 0xFFFFu;
		end_m7(s, tag);
		probe_fail = n_fail - f0;
		probe_bad = probe_fail != 0 || io_err || fatal;
		part_full = reads > 0 && empty_reads * 100u <= reads * BAND_OUT_PCT && high_reads * 100u <= reads * BAND_OUT_PCT;
		verdict_bad = vfail != 0 || c_fault;
		say("%s: band: %llu cycles, %llu records, %llu payload words, %llu checkpoints, host sum 0x%08x\n", tag,
		    (unsigned long long)cycles, (unsigned long long)s->pub_records, (unsigned long long)s->pub_words,
		    (unsigned long long)checkpoints, s->pub_sum);
		say("%s: band: fabric deepest %u of %u (VERDICT), host peak estimate %u, lowest fill at a read %u, "
		    "empty reads %llu and reads above %u%% %llu, of %llu, final rate estimate %.2f M words/s\n", tag, deepest,
		    TS_WORDS, max_peak, min_read == TS_WORDS ? 0u : min_read, (unsigned long long)empty_reads, BAND_HIGH_PCT,
		    (unsigned long long)high_reads, (unsigned long long)reads,
		    rate / 1e6);
		band_hist("fill at each cycle's read after the first (the LO side)", hist_read);
		band_hist("peak fill estimate after each cycle's publish (the HI side)", hist_peak);
		rc = probe_bad || verdict_bad || !part_full;
		say("RESULT torture --band (map %s): %s (%u probe failures, %u verdict mismatches%s%s; %llu cycles, %llu words, "
		    "deepest %u, lowest fill %u)\n", map_name(s->map), rc ? "FAIL" : "PASS", probe_fail, vfail,
		    c_fault ? ", ring fault" : "", part_full ? "" : ", the ring was NOT part-full (coverage)",
		    (unsigned long long)cycles, (unsigned long long)s->pub_words, deepest, min_read == TS_WORDS ? 0u : min_read);
		return rc;
	}
}

/* ---- freeze: the stopped-clk_sys case (GPO 25, test core only) ---- */
static int freeze_mode(const char *tag)
{
	struct sess s;
	uint32_t last, hello, nonce, v;
	uint16_t head = 0, tail0, tail1, evt_tail = 0;
	uint64_t row_ns = 0, head_ns = 0;
	int64_t t, t_frz, t_head = 0, t_arm2 = -1, t_echo2 = -1, t_rep = -1, t_bind = -1;
	unsigned marks0;
	int rc = 0;
	memset(&s, 0, sizeof s);
	say("== freeze: the stopped clk_sys case (GPO 25, DIAG + OSD)\n");
	if (begin_m7(&s, tag)) return 1;
	say("%s: hello armed in %lld ns, echoed in %lld ns\n", tag, (long long)s.t_arm, (long long)s.t_echo);
	OC(H_BIND) = 1u << 11;
	t_bind = await_c_bind(1u << 11, BIND_NS, &last);
	check(t_bind >= 0, "%s: C_BIND bit 11 %s (0x%08x, %lld ns)", tag, t_bind >= 0 ? "set" : "NOT set within 2 ms",
	      last, (long long)t_bind);
	if (t_bind < 0) { end_m7(&s, tag); return 1; }
	marks0 = s.marks;
	head = (uint16_t)OC(H_IDX(R_OSD));
	head = osd_put_rows(head, 16, PAT_ROWS, &row_ns, &head_ns, &t_head);
	{
		int64_t t0 = now_ns();
		gpo_store(gpo_shadow | GPO_FREEZE);
		t_frz = now_ns() - t0;
	}
	t = now_ns() - t_head;
	check(t <= 50000, "%s: FREEZE stored %lld ns after the repaint's head store (bound 50 us)", tag, (long long)t);
	say("%s: 16-row repaint published at head %u; FREEZE store %.3f us after the head store\n", tag, (unsigned)head,
	    t_frz / 1000.0);
	{
		int64_t tw = now_ns(), te = tw + 5000000;
		while (now_ns() < te) {
			if (drain(&s, &evt_tail)) { rc = 1; break; }
			if (now_ns() < te) sleep_ms(1);
		}
	}
	tail0 = (uint16_t)OC(C_IDX(R_OSD));
	sleep_ms(1);
	tail1 = (uint16_t)OC(C_IDX(R_OSD));
	check((uint16_t)(head - tail0) > 0, "%s: C_IDX[2] stopped short of the head: %u words (%u rows) behind", tag,
	      (unsigned)(uint16_t)(head - tail0), (unsigned)((uint16_t)(head - tail0) / (1u + ROW_LEN)));
	check(tail1 == tail0, "%s: C_IDX[2] did not move while frozen (%u then %u)", tag, (unsigned)tail0,
	      (unsigned)tail1);
	say("%s: during 5 ms frozen, %u MARK/VERDICT records arrived (polls continue: %s)\n", tag, s.marks - marks0,
	    s.marks - marks0 ? "yes" : "checked by the next ARMED read");
	if (!stop_run("freeze: first stop (FREEZE held)", &t)) {
		say("RECOVERY: %s: no first stop proof: no further RUN edge on this bitstream\n", tag);
		end_m7(&s, tag);
		return 1;
	}
	check(t <= BOUND_NS, "%s: the first stop proof landed in %lld ns (1 ms bound)", tag, (long long)t);
	zero64(&OC(PAGE_OFF), PAGE_LEN / 4);   /* H_HELLO = 0 before the next RUN edge (§2.5 step 2) */
	gpo_store(gpo_shadow & ~GPO_RUN);
	gpo_store((gpo_shadow & ~GPO_RUN) | GPO_RUN);
	t_arm2 = await_c_hello(V_ARMED, &last);
	check(t_arm2 >= 0, "%s: ARMED with FREEZE set %s (C_HELLO 0x%08x, %lld ns)", tag,
	      t_arm2 >= 0 ? "read" : "NOT read within 1 ms", last, (long long)t_arm2);
	nonce = ((uint32_t)now_ns() ^ (uint32_t)getpid()) & 0xFFFFu;
	if (!nonce) nonce = 1;
	hello = 0x4F430000u | nonce;
	OC(H_HELLO) = hello;
	{
		int64_t t0 = now_ns(), r = await_c_hello(hello, &last);
		t_echo2 = now_ns() - t0;
		check(r < 0, "%s: the echo did NOT come within 1 ms while FREEZE is set (C_HELLO 0x%08x)", tag, last);
	}
	say("%s: frozen hello: ARMED in %lld ns, then no echo in %lld ns\n", tag, (long long)t_arm2, (long long)t_echo2);
	if (!stop_run("freeze: second stop", &t)) {
		say("RECOVERY: %s: no second stop proof: no thaw RUN edge on this bitstream\n", tag);
		end_m7(&s, tag);
		return 1;
	}
	check(t <= BOUND_NS, "%s: the second stop proof landed in %lld ns (1 ms bound)", tag, (long long)t);
	gpo_store(gpo_shadow & ~GPO_FREEZE);
	say("%s: FREEZE cleared; waiting 1 ms for the half's in-flight word\n", tag);
	sleep_ms(1);
	zero64(&OC(PAGE_OFF), PAGE_LEN / 4);   /* the un-echoed hello goes; H_HELLO = 0 for the next edge */
	gpo_store(gpo_shadow & ~GPO_RUN);
	gpo_store((gpo_shadow & ~GPO_RUN) | GPO_RUN);
	t_arm2 = await_c_hello(V_ARMED, &last);
	check(t_arm2 >= 0, "%s: ARMED after the thaw %s (C_HELLO 0x%08x, %lld ns)", tag,
	      t_arm2 >= 0 ? "read" : "NOT read within 1 ms", last, (long long)t_arm2);
	if (t_arm2 < 0) { end_m7(&s, tag); return 1; }
	nonce = ((uint32_t)now_ns() ^ (uint32_t)getpid()) & 0xFFFFu;
	if (!nonce) nonce = 1;
	hello = 0x4F430000u | nonce;
	OC(H_HELLO) = hello;
	{
		int64_t t_echo3 = await_c_hello(hello, &last);
		check(t_echo3 >= 0, "%s: the echo after the thaw %s (C_HELLO 0x%08x, %lld ns)", tag,
		      t_echo3 >= 0 ? "read" : "NOT read within 1 ms", last, (long long)t_echo3);
		say("%s: thawed hello echoed in %lld ns\n", tag, (long long)t_echo3);
		if (t_echo3 < 0) { end_m7(&s, tag); return 1; }
	}
	check_hello(&s, tag);
	OC(H_BIND) = 1u << 11;
	t_bind = await_c_bind(1u << 11, BIND_NS, &last);
	check(t_bind >= 0, "%s: C_BIND bit 11 after the thaw %s (0x%08x, %lld ns)", tag,
	      t_bind >= 0 ? "set" : "NOT set within 2 ms", last, (long long)t_bind);
	if (t_bind < 0) { end_m7(&s, tag); return 1; }
	head = (uint16_t)OC(H_IDX(R_OSD));
	head = osd_put_rows(head, 16, PAT_ROWS, &row_ns, &head_ns, NULL);
	t_rep = await_c_idx(R_OSD, head, OSD_WAIT_NS, &tail0);
	check(t_rep >= 0, "%s: the fresh 16-row repaint %s (C_IDX[2] %u, head %u)", tag,
	      t_rep >= 0 ? "completed" : "did NOT complete within 2 s", (unsigned)tail0, (unsigned)head);
	say("%s: fresh 16-row repaint: %lld ns\n", tag, (long long)t_rep);
#ifdef OCRAM_FAKE
	check(fake_osd_diff(PAT_ROWS, 16) == 0, "%s: the fake osd_buffer holds the fresh 16-row repaint", tag);
	if (fake_osd_diff(PAT_ROWS, 16)) fake_osd_say_diff(tag, PAT_ROWS, 16);
#endif
	v = OC(C_STATE);
	say("%s: C_STATE 0x%08x before the final quiesce (mid-record ends 0x%02x)\n", tag, v, (v >> 16) & 0x1F);
	end_m7(&s, tag);
	return rc || n_fail ? 1 : 0;
}

/* ==== M5, the gate (ATTENDED ONLY: a hazard here wedges the L3 and needs a power cycle) ====
   Each cycle arms the master, puts traffic on the bridge, and reloads the core as stock Main does
   (fpga_core_reset(1), do_bridge(0), socfpga_load, do_bridge(1)), then proves the reloaded core arms again.
     --mode quiesce  RUN 0 and the stop proof first (design note §2.6); a missing proof ends the run, unreloaded
     --mode stock    stock's order with no wait: RUN and the I/O view fall in one GPO store, the bridges reset next
     --mode bridge   do_bridge(0) / do_bridge(1) with RUN still high and no reload: a bridge reset mid-traffic
   Traffic (--traffic poll|flood|mix, default mix): poll = H_LINE set and the ring left undrained, so the master
   re-reads the tail back to back; flood = MARKs back to back while the host drains, up to the reload's store. */
#define FM(off) REG(mgr, off)
#define FTO 0x1000000u

static int fm_mode(void) { return (int)(FM(0x00) & 0x7u); }

static int fm_dclk(uint32_t n)
{
	uint32_t i;
	if (FM(0x0C)) FM(0x0C) = 1;
	FM(0x08) = n;
	for (i = 0; i < FTO; i++) if (FM(0x0C)) { FM(0x0C) = 1; return 0; }
	return -1;
}

/* stock socfpga_load(), step for step; returns 0, or stock's negative step number */
static int fm_load(const uint32_t *w, size_t bytes)
{
	uint32_t msel = (FM(0x00) & 0xF8u) >> 3, ctrl = FM(0x04), cd, i;
	size_t k, n = (bytes + 3) / 4;
	ctrl = (msel & 8) ? (ctrl | 0x200u) : (ctrl & ~0x200u);
	cd = (msel & 3) == 0 ? 0 : (msel & 8) ? ((msel & 3) == 1 ? 2 : 3) : ((msel & 3) == 1 ? 1 : 2);
	if ((msel & 3) < 3) ctrl = (ctrl & ~(3u << 6)) | (cd << 6);
	FM(0x04) = ctrl;
	FM(0x04) &= ~0x2u;                                  /* nCE */
	FM(0x04) |= 0x1u;                                   /* EN */
	FM(0x04) |= 0x4u;                                   /* nCONFIG pull: reset phase */
	for (i = 0; i < FTO && fm_mode() != 1; i++) {}
	if (fm_mode() != 1) return -1;
	FM(0x04) &= ~0x4u;
	for (i = 0; i < FTO && fm_mode() != 2; i++) {}
	if (fm_mode() != 2) return -2;
	FM(0x84C) = 0xFFF;                                  /* gpio_porta_eoi */
	FM(0x04) |= 0x100u;                                 /* AXI cfg */
	for (k = 0; k < n; k++) fdat[0] = w[k];
	for (i = 0; i < FTO; i++) if (FM(0x850) & 0x3u) break;
	if (i == FTO) return -4;
	FM(0x04) &= ~0x100u;
	if (fm_dclk(0x4)) return -5;
	for (i = 0; i < FTO && fm_mode() != 3 && fm_mode() != 4; i++) {}
	if (i == FTO) return -6;
	if (fm_dclk(0x5000)) return -7;
	for (i = 0; i < FTO && fm_mode() != 4; i++) {}
	if (i == FTO) return -8;
	FM(0x04) &= ~0x1u;
	return 0;
}

static void bridges(int on)
{
	if (on) {
		REG(sdr, 0x080) = 0x3FFF;
		REG(rst, 0x1C) = 0;
		REG(l3, 0x0) = 0x19;
	} else {
		REG(sys, 0x28) = 0;
		REG(sdr, 0x080) = 0;
		REG(rst, 0x1C) = 7;
		REG(l3, 0x0) = 1;
	}
}

/* the arm, quietly: prepare, the RUN edge, the armed value, the hello and its echo. 0 = in session */
static int g_arm(int flood, int line, uint32_t *why)
{
	uint32_t last, hello;
	if (prepare()) { *why = 1; return 1; }
	gpo_store(gpo_base | (flood ? GPO_DIAG_FL : 0));
	gpo_store(gpo_base | (flood ? GPO_DIAG_FL : 0) | GPO_RUN);
	if (await_c_hello(V_ARMED, &last) < 0) { *why = last; return 2; }
	OC(H_LINE) = line ? (1u << R_EVT) : 0u;
	hello = 0x4F430000u | ((((uint32_t)now_ns() >> 4) & 0xFFFEu) | 1u);
	OC(H_HELLO) = hello;
	if (await_c_hello(hello, &last) < 0) { *why = last; return 3; }
	return 0;
}

static int gate(const char *mode, const char *traffic, const char *rbf, unsigned cycles)
{
	int m = !strcmp(mode, "quiesce") ? 0 : !strcmp(mode, "stock") ? 1 : !strcmp(mode, "bridge") ? 2 : -1;
	int tr = !strcmp(traffic, "poll") ? 0 : !strcmp(traffic, "flood") ? 1 : !strcmp(traffic, "mix") ? 2 : -1;
	uint32_t *img = NULL, why = 0, v;
	size_t bytes = 0;
	unsigned c, done = 0, nostop = 0, landed = 0, rd_poll = 0, faults = 0;
	int64_t stop_max = 0, load_max = 0, t;
	if (m < 0 || tr < 0 || !cycles) { say("gate: bad --mode or --traffic or --cycles\n"); return 2; }
	if (m != 2) {
		FILE *f = fopen(rbf ? rbf : "", "rb");
		long sz;
		if (!f) { say("gate: --rbf %s: cannot open\n", rbf ? rbf : "(none)"); return 2; }
		fseek(f, 0, SEEK_END); sz = ftell(f); fseek(f, 0, SEEK_SET);
		if (sz <= 16 || !(img = calloc(1, (size_t)sz + 4)) || fread(img, 1, (size_t)sz, f) != (size_t)sz) {
			say("gate: --rbf %s: cannot read\n", rbf); return 2;
		}
		fclose(f);
		bytes = (size_t)sz;
		if (!memcmp(img, "MiSTer", 6)) { say("gate: a MiSTer-headed rbf; give the raw one\n"); return 2; }
	}
	fdat = map_phys(FPGADAT_BASE, PAGE);
	l3 = map_phys(L3REGS_BASE, PAGE);
	sdr = map_phys(SDR_BASE, PAGE);
	say("gate: mode=%s traffic=%s cycles=%u rbf=%s (%zu bytes) fpga.stat=0x%08x sys.fpgaintf.module=0x%08x\n",
	    mode, traffic, cycles, rbf ? rbf : "-", bytes, FM(0x00), REG(sys, 0x28));
	for (c = 0; c < cycles; c++) {
		int fl = tr == 1 || (tr == 2 && (c & 1)), r;
		int64_t dwell = 5000000 + (int64_t)(rand() % 45) * 1000000, t0;
		uint16_t tail = 0;
		struct sess s;
		quiet = 1;
		r = g_arm(fl, !fl, &why);
		quiet = 0;
		if (r) {
			say("gate: cycle %u: the arm failed at step %d (C_HELLO 0x%08x): %s\n", c, r, why,
			    c ? "the reloaded core did not answer" : "the first arm");
			break;
		}
		memset(&s, 0, sizeof s);
		s.flood = fl;
		t0 = now_ns();
		if (fl) {
			quiet = 1;
			while (now_ns() - t0 < dwell) if (drain(&s, &tail)) { faults++; break; }
			quiet = 0;
		} else {
			while ((uint16_t)OC(C_IDX(R_EVT)) == 0 && now_ns() - t0 < 200000000) {}
			rd_poll += (uint16_t)OC(C_IDX(R_EVT)) != 0;
			while (now_ns() - t0 < dwell) {}
		}
		if (m == 0) {
			gpo_store(gpo_base | (fl ? GPO_DIAG_FL : 0));
			t = await_c_hello(V_STOPPED, &v);
			if (t < 0) {
				nostop++;
				say("gate: cycle %u: NO stopped value within 1 ms (C_HELLO 0x%08x): not reloaded\n", c, v);
				break;
			}
			if (t > stop_max) stop_max = t;
		}
		if (m == 2) {
			bridges(0);
			t0 = now_ns(); while (now_ns() - t0 < 1000000) {}
			bridges(1);
			t0 = now_ns(); while (now_ns() - t0 < 1000000) {}
			v = OC(C_HELLO);
			say("gate: cycle %u: after the bridge reset C_HELLO 0x%08x brgmodrst 0x%08x\n", c, v, REG(rst, 0x1C));
			if (!quiesce("gate")) { nostop++; break; }
			done++;
			continue;
		}
		/* stock fpga_core_reset(1): the I/O view falls and core reset rises; in stock mode RUN falls in this store */
		gpo_store((gpo_base & ~GPO_IO_VIEW) | GPO_RESET);
		bridges(0);
		landed += OC(C_HELLO) == V_STOPPED;
		t0 = now_ns();
		r = fm_load(img, bytes);
		t = now_ns() - t0;
		if (t > load_max) load_max = t;
		if (r) { say("gate: cycle %u: the load failed at stock step %d (fpga.stat 0x%08x)\n", c, r, FM(0x00)); break; }
		bridges(1);
		gpo_store(gpo_base);
		sleep_ms(20);
		v = gpi_read();
		if (!(v & GPI_OCRAM) || (v & GPI_UNCONF)) { say("gate: cycle %u: after the reload GPI 0x%08x\n", c, v); break; }
		v = REG(sys, 0x144);
		if (v & 0x18u) { say("gate: cycle %u: eccgrp.ocram 0x%08x: an ECC flag\n", c, v); break; }
		done++;
		if ((c + 1) % 50 == 0)
			say("gate: %u cycles, %u reloads clean, stop max %lld ns, load max %lld ms, terminal store landed %u\n",
			    c + 1, done, (long long)stop_max, (long long)(load_max / 1000000), landed);
	}
	check(done == cycles, "gate %s: %u of %u cycles clean (traffic %s; %u poll cycles saw the line's record)", mode,
	      done, cycles, traffic, rd_poll);
	check(faults == 0, "gate: no EVT protocol fault during the flood dwells (%u)", faults);
	if (m == 0) check(nostop == 0, "gate quiesce: the stopped value never missing (max %lld ns)", (long long)stop_max);
	if (m == 1) say("gate stock: the stopped value had landed before the bridge reset in %u of %u\n", landed, done);
	if (m != 2) say("gate: load max %lld ms; fpga.stat 0x%08x\n", (long long)(load_max / 1000000), FM(0x00));
	if (done < cycles || nostop)
		say("RECOVERY: the run ended early: do NOT run Main, load a core or reset a bridge until the operator has "
		    "read the log; if the next 'look' hangs or ssh stops, power-cycle (attended)\n");
	else if (gpo_read() & GPO_RUN) quiesce("gate");
	free(img);
	return done == cycles && !nostop ? 0 : 1;
}

static void usage(void)
{
	fputs("usage: ocram_probe look | quiesce | matrix [--seconds N] | run [--ns] [--flood] [--line] [--seconds N] [--drain-ms M]\n"
	      "       ocram_probe torture [--words N (10^9)] [--ns] [--seconds CAP (600)]     (M4, fabric to host)\n"
	      "       ocram_probe torture --map so|wc|wc-dmb|wc-dsb|wc-wmb [--words N (10^9)] [--maxlen L (1024)]\n"
	      "                                              (M4, host to fabric; wc* opens /dev/ocram_wc)\n"
	      "       ocram_probe torture --map so|wc|wc-dmb|wc-dsb|wc-wmb --band [--words N (10^9)] [--maxlen L (256)]\n"
	      "                           [--rate WORDS_PER_S (6.3e6)] [--seed S (1)] [--empty-words E (1048576)]\n"
	      "                                              (E25, ring held part-full; PASS or FAIL on the last line)\n"
	      "       ocram_probe osd [--rows N (16)] [--repeat K (3)] [--pattern rows|bars|inc] [--hold S (0)] [--no-enable]\n"
	      "       ocram_probe spi-osd|fallback [--rows N (16)] [--pattern rows|bars|inc] [--hold S (0)]\n"
	      "       ocram_probe osd-off | freeze\n"
	      "       ocram_probe gate --attended --mode quiesce|stock|bridge [--traffic poll|flood|mix] [--cycles N] [--rbf RAW.rbf]\n",
	      stderr);
}

int main(int argc, char **argv)
{
	struct sess s;
	int i, rc, seconds_set = 0, map_set = 0, no_enable = 0, hold_s = 0, pat = PAT_ROWS;
	unsigned rows = 16, repeat = 3, maxlen = 1024;
	int band = 0, maxlen_set = 0;
	double rate = BAND_RATE_DEFAULT;
	uint64_t seed = 1, empty_words = BAND_EMPTY_WORDS_DEFAULT;
	const char *g_mode = NULL, *g_traffic = "mix", *g_rbf = NULL;
	unsigned g_cycles = 1000;
	int g_attended = 0;
	if (argc < 2) { usage(); return 2; }
	memset(&s, 0, sizeof s);
	s.seconds = 5;
	s.drain_ms = -1;
	s.map = -1;
	for (i = 2; i < argc; i++) {
		if (!strcmp(argv[i], "--ns")) s.ns = 1;
		else if (!strcmp(argv[i], "--flood")) s.flood = 1;
		else if (!strcmp(argv[i], "--line")) s.line = 1;
		else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) { s.seconds = atoi(argv[++i]); seconds_set = 1; }
		else if (!strcmp(argv[i], "--words") && i + 1 < argc) s.t_words = strtoull(argv[++i], NULL, 10);
		else if (!strcmp(argv[i], "--drain-ms") && i + 1 < argc) s.drain_ms = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--map") && i + 1 < argc) {
			const char *m = argv[++i];
			map_set = 1;
			if (!strcmp(m, "so")) s.map = 0;
			else if (!strcmp(m, "wc")) s.map = 1;
			else if (!strcmp(m, "wc-dmb")) s.map = 2;
			else if (!strcmp(m, "wc-dsb")) s.map = 3;
			else if (!strcmp(m, "wc-wmb")) s.map = 4;
			else { usage(); return 2; }
		}
		else if (!strcmp(argv[i], "--maxlen") && i + 1 < argc) { maxlen = (unsigned)strtoul(argv[++i], NULL, 10); maxlen_set = 1; }
		else if (!strcmp(argv[i], "--band")) band = 1;
		else if (!strcmp(argv[i], "--rate") && i + 1 < argc) rate = strtod(argv[++i], NULL);
		else if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = strtoull(argv[++i], NULL, 10);
		else if (!strcmp(argv[i], "--empty-words") && i + 1 < argc) empty_words = strtoull(argv[++i], NULL, 10);
		else if (!strcmp(argv[i], "--rows") && i + 1 < argc) rows = (unsigned)strtoul(argv[++i], NULL, 10);
		else if (!strcmp(argv[i], "--repeat") && i + 1 < argc) repeat = (unsigned)strtoul(argv[++i], NULL, 10);
		else if (!strcmp(argv[i], "--hold") && i + 1 < argc) hold_s = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--no-enable")) no_enable = 1;
		else if (!strcmp(argv[i], "--pattern") && i + 1 < argc) {
			const char *p = argv[++i];
			if (!strcmp(p, "rows")) pat = PAT_ROWS;
			else if (!strcmp(p, "bars")) pat = PAT_BARS;
			else if (!strcmp(p, "inc")) pat = PAT_INC;
			else { usage(); return 2; }
		}
		else if (!strcmp(argv[i], "--attended")) g_attended = 1;
		else if (!strcmp(argv[i], "--mode") && i + 1 < argc) g_mode = argv[++i];
		else if (!strcmp(argv[i], "--traffic") && i + 1 < argc) g_traffic = argv[++i];
		else if (!strcmp(argv[i], "--rbf") && i + 1 < argc) g_rbf = argv[++i];
		else if (!strcmp(argv[i], "--cycles") && i + 1 < argc) g_cycles = (unsigned)strtoul(argv[++i], NULL, 10);
		else { usage(); return 2; }
	}
	if (s.seconds < 1 || s.seconds > 7200) { usage(); return 2; }
	if (rows < 1 || rows > MAX_ROWS) { usage(); return 2; }
	if (repeat < 1 || repeat > 10000) { usage(); return 2; }
	if (band && !map_set) { usage(); return 2; }
	if (band && !maxlen_set) maxlen = BAND_MAXLEN_DEFAULT;
	if (maxlen < 1 || maxlen > MAX_MAXLEN) { usage(); return 2; }
	if (rate < 1e5 || rate > 1e9 || empty_words < 1) { usage(); return 2; }
	if (hold_s < 0 || hold_s > 7200) { usage(); return 2; }
	if (s.drain_ms < 0) s.drain_ms = s.line ? 200 : 50;
	map_all();
	if (!strcmp(argv[1], "look")) { look(); return 0; }
	if (!strcmp(argv[1], "quiesce")) {
		uint32_t g = gpo_read();
		if (main_running()) { say("quiesce: a MiSTer process is running: it owns GPO; nothing stored\n"); return 3; }
		gpo_base = g & ~GPO_KNOBS;
		return quiesce("quiesce") ? 0 : 1;
	}
	/* the SPI modes need no OCRAM master: they are the stock fallback on the same bitstream */
	if (!strcmp(argv[1], "spi-osd") || !strcmp(argv[1], "fallback")) {
		rc = spi_osd_mode(argv[1], rows, pat, hold_s);
		if (rc > 1) return rc;
		say("RESULT %s: %s (%u failed checks)\n", argv[1], rc || n_fail ? "FAILED" : "PASSED", n_fail);
		return rc || n_fail ? 1 : 0;
	}
	if (!strcmp(argv[1], "osd-off")) {
		rc = osd_off_mode("osd-off");
		if (rc > 1) return rc;
		say("RESULT osd-off: %s (%u failed checks)\n", rc || n_fail ? "FAILED" : "PASSED", n_fail);
		return rc || n_fail ? 1 : 0;
	}
	if ((rc = preflight()) != 0) return rc;
	if (!strcmp(argv[1], "osd")) {
		rc = osd_mode("osd", rows, repeat, pat, hold_s, no_enable);
		say("RESULT osd: %s (%u failed checks)\n", rc || n_fail ? "FAILED" : "PASSED", n_fail);
		return rc || n_fail ? 1 : 0;
	}
	if (!strcmp(argv[1], "freeze")) {
		rc = freeze_mode("freeze");
		say("RESULT freeze: %s (%u failed checks)\n", rc || n_fail ? "FAILED" : "PASSED", n_fail);
		return rc || n_fail ? 1 : 0;
	}
	if (!strcmp(argv[1], "torture")) {
		if (map_set) {
			/* M4's other half, host to fabric: --map selects it; the bare mode stays M3's fabric-to-host one */
			s.h2f = 1;
			s.maxlen = maxlen;
			s.words_target_h2f = s.t_words ? s.t_words : 1000000000ULL;
			if (band) {
				s.band = 1;
				s.rate = rate;
				s.seed = seed;
				s.empty_words = empty_words;
				return torture_band(&s, "torture");
			}
			return torture_h2f(&s, "torture");
		}
		/* M4, the fabric-to-host half: flood MARK records until --words (default 10^9) are drained and checked;
		   --seconds is only the safety cap (default 600) */
		s.flood = 1; s.line = 0; s.drain_ms = 0;
		if (!s.t_words) s.t_words = 1000000000ULL;
		if (!seconds_set) s.seconds = 600;
		rc = session(&s, "torture");
		say("RESULT torture: %s (%u failed checks, %llu violations)\n", rc || n_fail ? "FAILED" : "PASSED", n_fail,
		    (unsigned long long)s.t_err);
		return rc || n_fail ? 1 : 0;
	}
	if (!strcmp(argv[1], "gate")) {
		if (!g_attended) {
			say("gate: refusing without --attended: a failed cycle can need a power cycle (M5 is attended only)\n");
			return 3;
		}
		rc = gate(g_mode ? g_mode : "", g_traffic, g_rbf, g_cycles);
		say("RESULT gate: %s (%u failed checks)\n", rc || n_fail ? "FAILED" : "PASSED", n_fail);
		return rc || n_fail ? 1 : 0;
	}
	if (!strcmp(argv[1], "run")) {
		rc = session(&s, "run");
		say("RESULT run: %s (%u failed checks)\n", rc || n_fail ? "FAILED" : "PASSED", n_fail);
		return rc || n_fail ? 1 : 0;
	}
	if (!strcmp(argv[1], "matrix")) {
		struct sess a[4];
		static const char *const tag[4] = { "A-secure-idle", "B-nonsecure-idle", "C-line-b2b-reads", "D-flood-b2b-writes" };
		unsigned f[4], k, use_ns;
		for (k = 0; k < 4; k++) { memset(&a[k], 0, sizeof a[k]); a[k].seconds = s.seconds; }
		a[0].drain_ms = 50;
		a[1].ns = 1; a[1].drain_ms = 50;
		for (k = 0; k < 2; k++) { unsigned f0 = n_fail; session(&a[k], tag[k]); f[k] = n_fail - f0; sleep_ms(10); }
		/* V21: secure is the design's value; the non-secure arm runs C and D only if secure did not arm */
		use_ns = !a[0].echoed && a[1].echoed;
		a[2].ns = a[3].ns = (int)use_ns;
		a[2].line = 1; a[2].drain_ms = 200;
		a[3].flood = 1; a[3].drain_ms = 0;
		if (a[0].echoed || a[1].echoed) {
			for (k = 2; k < 4; k++) { unsigned f0 = n_fail; session(&a[k], tag[k]); f[k] = n_fail - f0; sleep_ms(10); }
		} else {
			f[2] = f[3] = 0;
			say("matrix: neither AxPROT arm armed: C and D not run\n");
		}
		say("== matrix summary\n");
		for (k = 0; k < 4; k++) {
			int ran = k < 2 || a[0].echoed || a[1].echoed;
			say("%-20s ran=%d ns=%d armed=%d (%lld ns) echo=%d (%lld ns) stopped=%d marks=%u failed_checks=%u",
			    tag[k], ran, a[k].ns, a[k].armed, (long long)a[k].t_arm, a[k].echoed, (long long)a[k].t_echo,
			    a[k].stopped, a[k].marks, ran ? f[k] : 0);
			if (a[k].marks) {
				const uint32_t *w = a[k].last.w;
				say(" rd=%u [%u..%u] mean %.2f wr=%u [%u..%u] mean %.2f", w[1], w[3] & 0xFFFF, w[3] >> 16,
				    w[1] ? (double)w[5] / w[1] : 0.0, w[2], w[4] & 0xFFFF, w[4] >> 16, w[2] ? (double)w[6] / w[2] : 0.0);
			}
			say("\n");
		}
		/* arm B is expected to arm too (a non-secure slave admits both, §12); its failure alone is a finding */
		{
			int last_ran = (a[0].echoed || a[1].echoed) ? 3 : 1, k2, nostop = 0;
			for (k2 = 0; k2 <= last_ran; k2++) nostop += !a[k2].stopped;
			if (!a[last_ran].stopped)
				say("RECOVERY: the last arm has no stop proof: a master may hold a transaction. Do NOT load a core, "
				    "reset a bridge or run Main; power-cycle the board (attended)\n");
			else if (nostop)
				say("NOTE: %d arm(s) lacked a stop proof, but the last arm stopped cleanly: the master is idle\n", nostop);
		}
		say("RESULT matrix: %s (%u failed checks)\n", n_fail ? "FAILED" : "PASSED", n_fail);
		return n_fail ? 1 : 0;
	}
	usage();
	return 2;
}
