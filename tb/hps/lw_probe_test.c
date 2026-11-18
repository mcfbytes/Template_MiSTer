/*
 * lw_probe_test: the probe's own logic, run on the build host against a model of the window (arm H of tb/run.sh).
 * It includes tb/hps/lw_probe.c whole, its main renamed, and points the probe's three mappings at memory it fills
 * itself, so each case runs the probe's own functions on a state it chose and checks what they did:
 *
 *   1. The walk: a directory where two known entries share an ordinal, a known and an unknown entry share
 *      another, and a framework entry names one above 15: none of those entries keeps a line; an entry whose
 *      ordinal no other entry names keeps it.
 *   2. The start's pad levels: with PAD's bitmap taking every level opcode, the probe stores each of the 24
 *      latest-wins levels once, the six paddles among them, and nothing else (no spinner, which is an event).
 *
 * Output: "-- " headings (the probe's own steps print theirs too), a pass or FAIL line per check, then the count:
 * "lw_probe_test: N checks, F failures". The probe's own FAIL lines count as failures.
 */
#define main lw_probe_main
int lw_probe_main(int argc, char **argv);
#include "lw_probe.c"
#undef main

#define WSIZE 0x80000u                       /* the model's window, 512 KB */
static volatile uint32_t m_mgr[PAGE / 4], m_rst[PAGE / 4], m_win[WSIZE / 4];
static unsigned t_checks, t_fails;

static void t_check(int ok, const char *fmt, ...)
{
	va_list a;
	t_checks++;
	if (!ok) t_fails++;
	fputs(ok ? "pass " : "FAIL ", stdout);
	va_start(a, fmt);
	vprintf(fmt, a);
	va_end(a);
	putchar('\n');
	fflush(stdout);
}

/* a fresh model: GPI shows the firewall present and idle, the window empty, no block known */
static void reset_model(void)
{
	uint32_t i;
	for (i = 0; i < PAGE / 4; i++) m_mgr[i] = m_rst[i] = 0;
	for (i = 0; i < WSIZE / 4; i++) m_win[i] = 0;
	m_mgr[0x14 / 4] = GPI_LW | GPI_IDLE;
	mgr = m_mgr;
	rst = m_rst;
	win = m_win;
	win_base = 0x40000u;
	win_size = WSIZE;
	gpo = GPO_IO_VIEW;
	memset(blk, 0, sizeof blk);
}

/* directory entry i, header 0x80 bytes, stride 0x100 */
static void entry(uint32_t i, uint32_t kind, uint32_t ver, uint32_t off, uint32_t size, uint32_t cause, uint32_t line)
{
	uint32_t e = (0x80u + i * 0x100u) / 4u;
	m_win[e] = kind;
	m_win[e + 1] = ver;
	m_win[e + 2] = off;
	m_win[e + 3] = size;
	m_win[e + 4] = cause;
	m_win[e + 5] = line;
}

static void case_walk(void)
{
	step("probe test 1: the walk gives no line to an ordinal two entries name, nor to one outside 1-15");
	reset_model();
	m_win[H_LAYOUT / 4] = 0x100u << 16 | 0x80u;
	m_win[H_COUNT / 4] = 6;
	entry(0, K_PAD, 1u << 16, 0x10000u, 0x10000u, 0, 4);              /* ordinal 4, its own */
	entry(1, K_EVT, 2u << 16, 0x06000u, 0x01000u, 0x06000u, 2);       /* ordinal 2, shared with the lane */
	entry(2, K_LANE, 1u << 16, 0x07000u, 0x01000u, 0x07024u, 2);
	entry(3, K_SECTOR, 2u << 16, 0x08000u, 0x08000u, 0x08000u, 1);    /* ordinal 1, shared with an unknown kind */
	entry(4, 0x0009u, 1u << 16, 0x20000u, 0x01000u, 0x20000u, 1);
	entry(5, K_OSD, 2u << 16, 0x30000u, 0x02000u, 0, 16);             /* a framework kind above 15 */
	walk();
	t_check(blk[K_PAD].present && blk[K_PAD].line == 4, "PAD keeps ordinal 4, which no other entry names (line %u)",
	        (unsigned)blk[K_PAD].line);
	t_check(blk[K_EVT].present && blk[K_EVT].line == 0, "EVT, the first of two entries naming ordinal 2, gets no line "
	        "(line %u)", (unsigned)blk[K_EVT].line);
	t_check(blk[K_LANE].present && blk[K_LANE].line == 0, "LANE, the second, gets no line (line %u)",
	        (unsigned)blk[K_LANE].line);
	t_check(blk[K_SECTOR].present && blk[K_SECTOR].line == 0, "SECTOR gets no line: a skipped entry names ordinal 1 "
	        "too (line %u)", (unsigned)blk[K_SECTOR].line);
	t_check(blk[K_OSD].present && blk[K_OSD].line == 0, "OSDBUF's ordinal 16 is outside 1-15: no line (line %u)",
	        (unsigned)blk[K_OSD].line);
}

static void case_pads(void)
{
	static const uint32_t ops[8] = { 0x02, 0x03, 0x10, 0x11, 0x12, 0x13, 0x1A, 0x3D };
	uint32_t i, a, n = 0, want = 0, other = 0, paddles = 0, sticks = 0, pads = 0;
	step("probe test 2: the start re-stores every pad level, the paddles included, and no spinner");
	reset_model();
	blk[K_PAD].present = 1;
	blk[K_PAD].off = 0x10000u;
	for (i = 0; i < 8; i++) blk[K_PAD].k[ops[i] >> 5] |= 1u << (ops[i] & 31u);
	for (a = 0; a < 0x10000u; a += 4) m_win[(0x10000u + a) / 4] = 0xA5A5A5A5u;
	pad_levels();
	for (a = 0; a < 0x10000u; a += 4) {
		uint32_t op = a >> 10, idx = (a >> 2) & 0xFFu;
		int level = (op != 0x1Au && op != 0x3Du) ? idx == 0 : idx <= 5u || (op == 0x1Au && (idx & 0xFu) == 0xFu &&
		            idx >> 4 <= 5u);
		int known = 0;
		for (i = 0; i < 8; i++) known |= ops[i] == op;
		if (m_win[(0x10000u + a) / 4] == 0xA5A5A5A5u) { want += known && level; continue; }
		n++;
		if (!known || !level) { other++; continue; }
		if (op == 0x1Au && (idx & 0xFu) == 0xFu) paddles++;
		else if (op == 0x1Au || op == 0x3Du) sticks++;
		else pads++;
	}
	t_check(n == 24 && !want, "the 24 levels stored, every one (%u stored, %u levels left unstored)", (unsigned)n,
	        (unsigned)want);
	t_check(pads == 6 && sticks == 12, "the six digital pads and both sticks of six players (%u, %u)", (unsigned)pads,
	        (unsigned)sticks);
	t_check(paddles == 6, "the six paddles, 0x1A at index {p, 15} for p = 0-5 (%u)", (unsigned)paddles);
	t_check(!other, "no other store: no spinner, no index outside the table (%u)", (unsigned)other);
}

int main(void)
{
	case_walk();
	case_pads();
	printf("lw_probe_test: %u checks, %u failures\n", t_checks, t_fails + fails);
	return t_fails + fails ? 1 : 0;
}
