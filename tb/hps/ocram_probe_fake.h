/*
 * ocram_probe_fake.h: the desk build of ocram_probe (-DOCRAM_FAKE). The registers and OCRAM are static arrays, and a
 * thread plays the fabric's master after the design note's rules closely enough to walk every probe path: the
 * run latch on a RUN edge in the I/O view, ARMED on H_HELLO = 0, the hello's M7 stores (C_CAPS, the three ring
 * descriptors, C_K[8]) and its wait for rest, the echo, the OSD ring walker (one word per fake transaction, into
 * f_osd, stopping while GPO 25 is set), the torture sink (TSEQ records, the VERDICT arithmetic exactly as
 * M7-BRIEF §4), MARK/VERDICT records on a fake period, flow control on the host's tails, the SPI OSD transport
 * (GPO 19 select, GPO 17 strobe, GPI 17 ack, the osd.v 0x20|row / 0x40 / 0x41 decode), the line on GIC ID 83's
 * pending bit, and the stopped value. The WC maps of --map wc* are the same fake memory (write combining does
 * not exist on the desk).
 * The torture sink consumes whole records as fast as the thread ticks, or at f_ts_wps words/s (FAKE_TS_WPS in the
 * environment) when that is set: a record is taken when the budget is not in debt, and its words charged to it, so
 * the average rate holds and the tail moves in records. f_inj injects one fault into what the sink reads (a flipped
 * payload word, two adjacent words swapped, a record's seq skewed). f_oc() sees every access to OCRAM the probe
 * makes and the phase (f_phase) it makes it in, which is how the unit test shows --band reads nothing between a
 * record's payload and its head, and nothing while it waits. f_st is the sink's and the access counters' record
 * (shared memory when the test forks), f_blog the per-cycle log --band keeps.
 * FAKE_SECURE_REFUSED=1 makes secure runs never arm (AxPROT[1] = 0 refused); FAKE_BRIDGE_STICKS=1 makes a
 * bridge reset under RUN strand the master (no stopped value ever again).
 */
#include <pthread.h>

static uint32_t f_mgr[PAGE / 4], f_rst[PAGE / 4], f_sys[PAGE / 4], f_gicd[PAGE / 4], f_ocr[OCRAM_SIZE / 4];
static uint32_t f_dat[PAGE / 4], f_l3[PAGE / 4], f_sdr[PAGE / 4];
static uint8_t  f_osd[32 * 256];       /* the fake osd_buffer[32][256] */
static int      f_osd_enable;
static unsigned f_osd_rows;
static pthread_t f_thr;
static int f_started;

#define F_OSD_ROWS   16u               /* the fake core's C_K[8] rows */
#define F_MARK_TICKS 500u              /* 500 * 2 us = 1 ms, the fake MARK period */

static int main_running(void) { return 0; }

/* the probe's phases, the fake's counters, and the log (f_st and f_blog may point into shared memory) */
enum { PH_NONE, PH_READ, PH_PUB, PH_WAIT, PH_EMPTY };
enum { F_INJ_NONE, F_INJ_PAYLOAD, F_INJ_REORDER, F_INJ_SEQ };
struct f_stats {
	unsigned viol_pub;          /* PH_PUB accesses outside the ring's payload words and its head */
	unsigned viol_wait;         /* any access in PH_WAIT */
	unsigned idx4_reads;        /* accesses to C_IDX[4] in PH_READ */
	unsigned idx4_empty;        /* accesses to C_IDX[4] in PH_EMPTY */
	unsigned ts_deepest;        /* the sink's deepest fill, words */
	unsigned ts_fault;          /* the sink stopped on a ring fault */
	unsigned ts_records, ts_errors;
	unsigned blog_n, blog_cap;
	int inj;
	unsigned inj_at;            /* the word g (payload, reorder) or the record (seq) the fault lands on */
};
struct f_brec { uint16_t lo, hi, fill, peak; };
static struct f_stats f_st_local;
static struct f_stats *f_st = &f_st_local;
static struct f_brec *f_blog;
static int f_phase;
static double f_ts_wps;

static volatile uint32_t *f_oc(unsigned off);
static void f_blog_add(unsigned lo, unsigned hi, unsigned fill, unsigned peak)
{
	if (f_blog && f_st->blog_n < f_st->blog_cap) {
		struct f_brec *r = &f_blog[f_st->blog_n];
		r->lo = (uint16_t)lo; r->hi = (uint16_t)hi; r->fill = (uint16_t)fill; r->peak = (uint16_t)peak;
	}
	f_st->blog_n++;
}

static void zero64(volatile uint32_t *p, unsigned words)
{
	unsigned i;
	for (i = 0; i < words; i++) p[i] = 0;
}

static volatile uint32_t *f_oc(unsigned off)
{
	int ring = off >= TS_OFF && off < TS_OFF + 4u * TS_WORDS;
	if (f_phase == PH_PUB && !ring && off != H_IDX(R_TS)) f_st->viol_pub++;
	if (f_phase == PH_WAIT) f_st->viol_wait++;
	if (off == C_IDX(R_TS)) {
		if (f_phase == PH_READ) f_st->idx4_reads++;
		else if (f_phase == PH_EMPTY) f_st->idx4_empty++;
	}
	return &ocr[off / 4];
}

static uint32_t fld(uint32_t *b, unsigned off) { return __atomic_load_n(&b[off / 4], __ATOMIC_SEQ_CST); }
static void fst(uint32_t *b, unsigned off, uint32_t v) { __atomic_store_n(&b[off / 4], v, __ATOMIC_SEQ_CST); }

static void f_tick(void) { struct timespec t = { 0, 2000 }; nanosleep(&t, NULL); }

static uint32_t f_vword(uint32_t g)
{
	return g ^ ((g << 16) | (g >> 16)) ^ 0x9E3779B9u;
}

static uint32_t f_state(unsigned n, int osd_fault, int ts_fault, int osd_mid_next, int ts_mid_next)
{
	return (n & 0xFFu) | ((uint32_t)osd_fault << 10) | ((uint32_t)ts_fault << 12) |
	       ((uint32_t)osd_mid_next << 18) | ((uint32_t)ts_mid_next << 20);
}

/* the SPI OSD transport of fpga_io.cpp/spi.cpp decoded as osd.v does: GPO 19 select, GPO 17 strobe, GPI 17 ack */
static int f_spi_sel;
static int f_spi_have_cmd;
static uint8_t f_spi_cmd;
static unsigned f_spi_bcnt;

static void f_spi_tick(void)
{
	uint32_t g = fld(f_mgr, 0x10), gpi = fld(f_mgr, 0x14);
	int sel = (g & SPI_OSD_SEL) && (g & GPO_IO_VIEW) && !(g & (SPI_FPGA_SEL | SPI_IO_SEL));
	int strobe = !!(g & SPI_STROBE);
	int ack = !!(gpi & SPI_STROBE);
	if (sel && !f_spi_sel) { f_spi_have_cmd = 0; f_spi_cmd = 0; f_spi_bcnt = 0; }
	if (!sel && f_spi_sel) {
		/* osd.v applies the enable/disable command when io_osd falls */
		if (f_spi_have_cmd && (f_spi_cmd & 0xF0u) == 0x40u) f_osd_enable = f_spi_cmd & 1u;
		f_spi_have_cmd = 0;
		f_spi_cmd = 0;
		f_spi_bcnt = 0;
	}
	f_spi_sel = sel;
	if (sel && strobe && !ack) {
		uint8_t b = (uint8_t)(g & 0xFFu);
		if (!f_spi_have_cmd) {
			f_spi_have_cmd = 1;
			f_spi_cmd = b;
			if ((b & 0xF0u) == 0x20u) f_spi_bcnt = (unsigned)(b & 0x1Fu) * 256u;
			else f_spi_bcnt = 0;
		} else if ((f_spi_cmd & 0xF0u) == 0x20u) {
			if (f_spi_bcnt < sizeof f_osd) f_osd[f_spi_bcnt] = b;
			if (f_spi_bcnt < 8192u) f_spi_bcnt++;
		}
	}
	/* io_ack follows io_clk */
	if (strobe != ack) fst(f_mgr, 0x14, (gpi & ~SPI_STROBE) | (strobe ? SPI_STROBE : 0));
}

static void *f_master(void *arg)
{
	uint32_t prev_run = 0, echo = 0, pending_hello = 0;
	int latch = 0, armed = 0, insess = 0, waiting = 0, ns = 0, flood = 0, refused, stuck = 0, sticks;
	int bind_q = 0, osd_fault = 0, ts_fault = 0, osd_mid = 0, osd_mid_next = 0, ts_mid_next = 0;
	int osd_state = 0, osd_pending = 0, osd_abandon = 0;
	unsigned osd_arg = 0, osd_k = 0, osd_prow = 0, osd_pword = 0;
	uint32_t osd_pw = 0, mode = 4;
	uint16_t osd_tail = 0, ts_tail = 0, evt_head = 0;
	unsigned osd_rows = 0, osd_deepest = 0, tick = 0;
	uint32_t ts_records = 0, ts_words = 0, ts_errors = 0, ts_first_g = 0, ts_first_got = 0, ts_sum = 0;
	unsigned ts_skipped = 0, ts_deepest = 0, ts_have_bad = 0;
	uint32_t w[MARK_LEN];
	unsigned n = 0, k, lat;
	const char *e2 = getenv("FAKE_BRIDGE_STICKS");
	const char *e = getenv("FAKE_SECURE_REFUSED");
	const char *e3 = getenv("FAKE_TS_BAD");
	int ts_bad = e3 && *e3 == '1';
	double ts_budget = 0;
	int64_t ts_t_last = now_ns();
	int ts_more;
	refused = e && *e == '1';
	if (getenv("FAKE_TS_WPS")) f_ts_wps = atof(getenv("FAKE_TS_WPS"));
	sticks = e2 && *e2 == '1';
	(void)arg;
	f_sys[0xE0 / 4] = 0xAE9EFEBCu;
	f_sys[0x144 / 4] = 0x19u;
	f_mgr[0x14 / 4] = GPI_OCRAM | (1u << 18);
	memset(w, 0, sizeof w);
	w[3] = w[4] = 0xFFFF;
	for (;;) {
		uint32_t g, run, ctrl = fld(f_mgr, 0x04);
		int frz;
		/* the FPGA manager: nCONFIG pull -> reset phase -> config phase; DCLK counts -> init -> user mode */
		if (ctrl & 1u) {
			if (ctrl & 4u) mode = 1;
			else if (mode == 1) mode = 2;
		}
		if (fld(f_mgr, 0x08)) {
			fst(f_mgr, 0x08, 0);
			fst(f_mgr, 0x0C, 1);
			if (mode == 2) mode = 3; else if (mode == 3) mode = 4;
		}
		if (f_started > 1 && (fld(f_sys, 0x144) & 0x18u)) fst(f_sys, 0x144, 1u);   /* the W1C */
		fst(f_mgr, 0x00, mode | (0x0Au << 3));
		f_mgr[0x850 / 4] = mode == 2 ? 0x3u : 0x7u;
		f_spi_tick();
		if (mode != 4) {
			latch = 0; insess = 0; armed = 0; waiting = 0; prev_run = 0; stuck = 0; pending_hello = 0;
			osd_state = 0; osd_pending = 0;
			f_tick();
			continue;
		}
		if (fld(f_rst, 0x1C) & BRG_F2H) { if (latch && sticks) stuck = 1; f_tick(); continue; }
		if (stuck) { f_tick(); continue; }
		g = fld(f_mgr, 0x10);
		frz = !!(g & GPO_FREEZE);
		run = (g & GPO_RUN) && (g & GPO_IO_VIEW);
		if (!latch && run && !prev_run) {
			latch = 1; armed = 0; insess = 0; waiting = 0; pending_hello = 0;
			ns = !!(g & GPO_DIAG_NS); flood = !!(g & GPO_DIAG_FL);
		}
		prev_run = run;
		if (latch && !run) {
			latch = 0; insess = 0; pending_hello = 0;
			if (osd_mid) osd_mid_next = 1;
			osd_mid = 0;
			osd_abandon = osd_pending;
			f_gicd[(0x200 + 4 * (GIC_EVT_ID / 32)) / 4] = 0;
			fst(f_ocr, C_HELLO, V_STOPPED);
		}
		/* a word already handed to the clk_sys half is still written while the link drains */
		if (osd_pending && !frz) {
			if (!osd_abandon) {
				unsigned b;
				for (b = 0; b < 4; b++)
					f_osd[osd_prow * 256 + osd_pword * 4 + b] = (uint8_t)(osd_pw >> (8 * b));
			}
			osd_pending = 0;
			if (!osd_abandon) {
				osd_k++;
				if (osd_k == ROW_LEN) {
					osd_tail = (uint16_t)(osd_tail + 1u + ROW_LEN);
					fst(f_ocr, C_IDX(R_OSD), osd_tail);
					osd_rows++;
					f_osd_rows = osd_rows;
					osd_state = 0;
					osd_mid = 0;
				}
			} else {
				osd_abandon = 0;
			}
		}
		if (latch && !(refused && !ns)) {
			uint32_t h = fld(f_ocr, H_HELLO);
			if (!insess) {
				if (h == 0) {
					if (!armed) { armed = 1; fst(f_ocr, C_HELLO, V_ARMED); }
					pending_hello = 0;
				} else if (armed && (h >> 16) == 0x4F43 && (h & 0xFFFF)) {
					pending_hello = h;
				} else if (!waiting) {
					waiting = 1; armed = 0; fst(f_ocr, C_HELLO, V_WAITING);
				}
				if (pending_hello && !frz && !osd_pending) {
					echo = pending_hello; pending_hello = 0; insess = 1;
					osd_tail = 0; ts_tail = 0; evt_head = 0;
					osd_state = 0; osd_pending = 0; osd_abandon = 0; osd_mid = 0;
					osd_rows = 0; osd_deepest = 0; bind_q = 0; f_osd_rows = 0;
					ts_records = ts_words = ts_errors = ts_first_g = ts_first_got = ts_sum = 0;
					ts_skipped = ts_deepest = 0; ts_have_bad = 0; ts_fault = 0;
					ts_budget = 0; ts_t_last = now_ns();
					fst(f_ocr, C_VERSION, 0x00010000u);
					fst(f_ocr, C_CAPS, C_CAPS_M7);
					fst(f_ocr, C_STATE, f_state(++n, 0, 0, osd_mid_next, ts_mid_next));
					osd_fault = 0; ts_fault = 0;
					osd_mid_next = 0; ts_mid_next = 0;
					fst(f_ocr, C_IDX(R_OSD), 0);
					fst(f_ocr, C_IDX(R_EVT), 0);
					fst(f_ocr, C_IDX(R_TS), 0);
					fst(f_ocr, C_BIND, 0);
					fst(f_ocr, C_RING(R_OSD), C_RING2_M7);
					fst(f_ocr, C_RING(R_EVT), C_RING3_M7);
					fst(f_ocr, C_RING(R_TS), C_RING4_M7);
					fst(f_ocr, C_K(8), ((uint32_t)F_OSD_ROWS << 16) | 256u);
					fst(f_ocr, C_HELLO, echo);
					memset(w, 0, sizeof w);
					w[3] = w[4] = 0xFFFF;
				}
			} else if (h != echo) {
				insess = 0; bind_q = 0;
				if (osd_mid) osd_mid_next = 1;
				osd_mid = 0;
				osd_abandon = osd_pending;
				osd_state = 0;
				armed = (h == 0);
				if (armed) fst(f_ocr, C_HELLO, V_ARMED);
				else { waiting = 1; fst(f_ocr, C_HELLO, V_WAITING); }
			} else {
				uint16_t tail = (uint16_t)fld(f_ocr, H_IDX(R_EVT));
				uint16_t used = (uint16_t)(evt_head - tail);
				unsigned line;
				/* one fake transaction: a read of latency 20..40 cycles, bin 2..5 */
				lat = 20 + (unsigned)(rand() % 21);
				w[1]++; w[5] += lat; w[8 + lat / 8]++;
				if (lat < (w[3] & 0xFFFF)) w[3] = (w[3] & 0xFFFF0000u) | lat;
				if (lat > (w[3] >> 16)) w[3] = (w[3] & 0xFFFFu) | (lat << 16);
				/* the bind: H_BIND bit 11 read once, then held for the session */
				if (!bind_q && (fld(f_ocr, H_BIND) & (1u << 11))) {
					bind_q = 1;
					fst(f_ocr, C_BIND, 1u << 11);
				}
				/* the OSD walker: one record at a time, one word per two ticks, no progress while frozen */
				if (bind_q && !osd_fault && !osd_pending && !frz) {
					if (!osd_state) {
						uint16_t hd = (uint16_t)fld(f_ocr, H_IDX(R_OSD));
						unsigned depth = (uint16_t)(hd - osd_tail);
						if (depth > OSD_WORDS) {
							osd_fault = 1;
							fst(f_ocr, C_STATE, f_state(n, osd_fault, ts_fault, osd_mid_next, ts_mid_next));
						} else {
							if (depth > osd_deepest) osd_deepest = depth;
							if (depth >= 1) {
								uint32_t hdr = fld(f_ocr, OSD_OFF + 4u * (osd_tail & (OSD_WORDS - 1)));
								unsigned kind = hdr >> 24, len = hdr & 0xFFFFu;
								if (1u + len > depth || 1u + len > OSD_WORDS) {
									osd_fault = 1;
									fst(f_ocr, C_STATE, f_state(n, osd_fault, ts_fault, osd_mid_next, ts_mid_next));
								} else if (kind == K_ROW && len == ROW_LEN && ((hdr >> 16) & 0xFFu) < F_OSD_ROWS) {
									osd_arg = (hdr >> 16) & 0xFFu;
									osd_k = 0;
									osd_state = 1;
									osd_mid = 1;
								} else {
									osd_tail = (uint16_t)(osd_tail + 1u + len);
									fst(f_ocr, C_IDX(R_OSD), osd_tail);
								}
							}
						}
					} else {
						osd_pw = fld(f_ocr, OSD_OFF + 4u * ((osd_tail + 1u + osd_k) & (OSD_WORDS - 1)));
						osd_prow = osd_arg;
						osd_pword = osd_k;
						osd_pending = 1;
					}
				}
				/* the torture sink: one TSEQ record per tick (or as many as the rate allows), the RTL's counters */
				if (f_ts_wps > 0) {
					int64_t tn = now_ns();
					ts_budget += f_ts_wps * (double)(tn - ts_t_last) / 1e9;
					ts_t_last = tn;
				}
				do {
					ts_more = 0;
					if (!ts_fault) {
						uint16_t hd = (uint16_t)fld(f_ocr, H_IDX(R_TS));
						unsigned depth = (uint16_t)(hd - ts_tail);
						if (depth > TS_WORDS) {
							ts_fault = 1;
							f_st->ts_fault = 1;
							fst(f_ocr, C_STATE, f_state(n, osd_fault, ts_fault, osd_mid_next, ts_mid_next));
						} else {
							if (depth > ts_deepest) ts_deepest = depth;
							f_st->ts_deepest = ts_deepest;
							if (depth == 0 && f_ts_wps > 0) ts_budget = 0;
							if (depth >= 1 && (f_ts_wps <= 0 || ts_budget >= 0)) {
								uint32_t hdr = fld(f_ocr, TS_OFF + 4u * (ts_tail & (TS_WORDS - 1)));
								unsigned kind = hdr >> 24, len = hdr & 0xFFFFu;
								if (1u + len > depth || 1u + len > TS_WORDS) {
									ts_fault = 1;
									f_st->ts_fault = 1;
									fst(f_ocr, C_STATE, f_state(n, osd_fault, ts_fault, osd_mid_next, ts_mid_next));
								} else {
									if (kind == K_TSEQ) {
										uint32_t seq = (hdr >> 16) & 0xFFu;
										if (f_st->inj == F_INJ_SEQ && ts_records == f_st->inj_at) seq ^= 0x5u;
										if (seq != (ts_records & 0xFFu)) ts_errors++;
										for (k = 0; k < len; k++) {
											uint32_t gg = ts_words;
											uint32_t got = fld(f_ocr, TS_OFF + 4u * ((ts_tail + 1u + k) & (TS_WORDS - 1)));
											if (ts_bad && gg == 0) got ^= 1u;   /* desk test of the error path */
											if (f_st->inj == F_INJ_PAYLOAD && gg == f_st->inj_at) got ^= 1u;
											if (f_st->inj == F_INJ_REORDER && gg == f_st->inj_at) got = f_vword(gg + 1u);
											if (f_st->inj == F_INJ_REORDER && gg == f_st->inj_at + 1u) got = f_vword(gg - 1u);
											ts_sum += got; ts_words++;
											if (got != f_vword(gg)) {
												ts_errors++;
												if (!ts_have_bad) { ts_have_bad = 1; ts_first_g = gg; ts_first_got = got; }
											}
										}
										ts_records++;
									} else {
										ts_skipped++;
									}
									ts_tail = (uint16_t)(ts_tail + 1u + len);
									fst(f_ocr, C_IDX(R_TS), ts_tail);
									f_st->ts_records = ts_records;
									f_st->ts_errors = ts_errors;
									if (f_ts_wps > 0) {
										ts_budget -= 1.0 + len;
										ts_more = ts_budget >= 0;
									}
								}
							}
						}
					}
				} while (ts_more);
				/* MARK, then VERDICT on the same period while the EVT ring has room for both */
				{
					int post = flood || (tick % F_MARK_TICKS) == 0;
					unsigned need = 1u + MARK_LEN + (flood ? 0u : 1u + VERDICT_LEN);
					if (post && used <= EVT_WORDS - need) {
						w[0] = (used & 0xFFFFu) | ((uint32_t)ns << 16) | ((uint32_t)flood << 17) |
						       ((n & 0xFFu) << 24);
						fst(f_ocr, EVT_OFF + 4u * (evt_head & (EVT_WORDS - 1)),
						    (K_MARK << 24) | (R_EVT << 16) | MARK_LEN);
						for (k = 0; k < MARK_LEN; k++)
							fst(f_ocr, EVT_OFF + 4u * ((evt_head + 1u + k) & (EVT_WORDS - 1)), w[k]);
						evt_head = (uint16_t)(evt_head + 1u + MARK_LEN);
						fst(f_ocr, C_IDX(R_EVT), evt_head);
						lat = 25; w[2]++; w[6] += lat; w[24 + lat / 8]++;
						if (lat < (w[4] & 0xFFFF)) w[4] = (w[4] & 0xFFFF0000u) | lat;
						if (lat > (w[4] >> 16)) w[4] = (w[4] & 0xFFFFu) | (lat << 16);
						if (!flood) {
							uint32_t vv[VERDICT_LEN];
							vv[0] = ts_records;
							vv[1] = ts_words;
							vv[2] = ts_errors;
							vv[3] = ts_first_g;
							vv[4] = ts_first_got;
							vv[5] = ts_sum;
							vv[6] = ((uint32_t)(ts_skipped & 0xFFFFu) << 16) | (ts_deepest & 0xFFFFu);
							vv[7] = ((uint32_t)(osd_rows & 0xFFFFu) << 16) | (osd_deepest & 0xFFFFu);
							fst(f_ocr, EVT_OFF + 4u * (evt_head & (EVT_WORDS - 1)),
							    (K_VERDICT << 24) | (VERDICT_ARG << 16) | VERDICT_LEN);
							for (k = 0; k < VERDICT_LEN; k++)
								fst(f_ocr, EVT_OFF + 4u * ((evt_head + 1u + k) & (EVT_WORDS - 1)), vv[k]);
							evt_head = (uint16_t)(evt_head + 1u + VERDICT_LEN);
							fst(f_ocr, C_IDX(R_EVT), evt_head);
						}
					}
				}
				tail = (uint16_t)fld(f_ocr, H_IDX(R_EVT));
				line = ((fld(f_ocr, H_LINE) >> R_EVT) & 1u) && evt_head != tail;
				f_gicd[(0x200 + 4 * (GIC_EVT_ID / 32)) / 4] = line << (GIC_EVT_ID % 32);
			}
		}
		tick++;
		f_tick();
	}
	return NULL;
}

static volatile uint32_t *map_phys(uint32_t phys, uint32_t len)
{
	(void)len;
	if (!f_started) {
		f_started = 1;
		f_mgr[0x10 / 4] = GPO_IO_VIEW;
		f_mgr[0x00 / 4] = 4u | (0x0Au << 3);
		f_sys[0xE0 / 4] = 0xAE9EFEBCu;
		f_mgr[0x14 / 4] = GPI_OCRAM | (1u << 18);
		pthread_create(&f_thr, NULL, f_master, NULL);
	}
	if (phys == OCRAM_BASE) f_started = 2;
	switch (phys) {
	case FPGAMGR_BASE: return f_mgr;
	case RSTMGR_BASE:  return f_rst;
	case SYSMGR_BASE:  return f_sys;
	case GICD_BASE:    return f_gicd;
	case OCRAM_BASE:   return f_ocr;
	case FPGADAT_BASE: return f_dat;
	case L3REGS_BASE:  return f_l3;
	case SDR_BASE:     return f_sdr;
	default: fprintf(stderr, "fake: refusing 0x%08x\n", phys); exit(2);
	}
}
