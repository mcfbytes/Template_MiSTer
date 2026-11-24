/*
 * ocram_probe_test: the OCRAM probe's own logic, run on the build host against the desk model of the fabric
 * (arm H of tb/run.sh). It includes tb/hps/ocram_probe.c whole under OCRAM_FAKE, its main renamed; the model is
 * tb/hps/ocram_probe_fake.h (a thread playing the master, the torture sink consuming at a rate the test sets, and
 * a hook on every OCRAM access that knows which phase of --band made it). Each case forks, runs the probe's main
 * on an argv in the child, and the parent reads back its output, its exit status and the counters the model kept
 * in shared memory:
 *
 *   1. The existing modes still pass on the model: run, torture (fabric to host), torture --map so and wc-wmb
 *      (the ring full), osd, spi-osd, osd-off, freeze, look, quiesce.
 *   2. --band, strongly ordered, the fabric at the rate the probe assumes: PASS; every checkpoint's VERDICT equals
 *      the host's counts; exactly one C_IDX[4] read per cycle (and none in the publish or wait phases); no access
 *      to any mailbox word but the ring's payload and head between a record's first store and the cycle's end;
 *      the fabric's deepest fill below 8,192 and the lowest fill read above 0; HI drawn from 50-90% and LO from
 *      5-45% of the ring, the publish ending near HI and the next read finding about LO.
 *   3. --band on a write-combined variant: the same PASS and the same access counts.
 *   4. --band with the fabric slower than assumed: the ring never fills, no fault, PASS.
 *   5. --band with the fabric far slower than assumed: the ring is never full and never faulted, but the probe
 *      says FAIL, the ring was not part-full (coverage).
 *   6. --band against an injected fault, each a FAIL from the VERDICT: a flipped payload word; two adjacent words
 *      swapped (the host's sum is unchanged, the per-word check is not); a record's seq skewed.
 *   7. The seed fixes the HI and LO sequence; --band without --map, and a --rate outside 1e5-1e9, are usage errors.
 *
 * Output: "-- " headings, a pass or FAIL line per check, then the count: "ocram_probe_test: N checks, F failures".
 */
#define main ocram_probe_main
int ocram_probe_main(int argc, char **argv);
#include "ocram_probe.c"
#undef main
#include <sys/wait.h>

#define BLOG_CAP 65536u
#define OUT_MAX (1u << 22)

static unsigned t_checks, t_fails;
static struct f_stats *shared_st;
static struct f_brec *shared_log;

static void tt_check(int ok, const char *fmt, ...)
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

static void heading(const char *s)
{
	printf("-- %s\n", s);
	fflush(stdout);
}

/* one probe run in a child: its stdout into *out (NUL-terminated, from the heap), its exit status returned */
static int run_probe(char *out, const char *cmd, double wps, int inj, unsigned inj_at)
{
	char *argv[40];
	char line[512];
	int argc = 0, st = 0, fd;
	FILE *tf = tmpfile();
	pid_t pid;
	size_t n;

	snprintf(line, sizeof line, "ocram_probe %s", cmd);
	for (char *p = strtok(line, " "); p && argc < 39; p = strtok(NULL, " ")) argv[argc++] = p;
	argv[argc] = NULL;
	memset(shared_st, 0, sizeof *shared_st);
	shared_st->blog_cap = BLOG_CAP;
	shared_st->inj = inj;
	shared_st->inj_at = inj_at;
	f_ts_wps = wps;
	fflush(stdout);
	fd = fileno(tf);
	pid = fork();
	if (pid == 0) {
		int rc;
		dup2(fd, 1);
		dup2(fd, 2);
		rc = ocram_probe_main(argc, argv);
		fflush(stdout);
		_exit(rc);
	}
	waitpid(pid, &st, 0);
	rewind(tf);
	n = fread(out, 1, OUT_MAX - 1, tf);
	out[n] = 0;
	fclose(tf);
	return WIFEXITED(st) ? WEXITSTATUS(st) : 255;
}

static int has(const char *out, const char *s) { return strstr(out, s) != NULL; }

/* the "torture: band: N cycles, R records, W payload words, C checkpoints" line */
static int band_counts(const char *out, unsigned long long *cycles, unsigned long long *records,
                       unsigned long long *words, unsigned long long *cps)
{
	const char *p = strstr(out, "band: ");
	while (p) {
		if (sscanf(p, "band: %llu cycles, %llu records, %llu payload words, %llu checkpoints", cycles, records, words,
		           cps) == 4)
			return 1;
		p = strstr(p + 1, "band: ");
	}
	return 0;
}

static void existing(char *out, const char *name, const char *cmd, const char *want)
{
	int rc = run_probe(out, cmd, 20000000.0, 0, 0);
	tt_check(rc == 0 && has(out, want), "%s: exit %d, and its output has \"%s\"", name, rc, want);
}

static void case_existing(char *out)
{
	heading("probe test 1: the modes of the first probe still pass on the model");
	existing(out, "look", "look", "C_HELLO");
	existing(out, "run", "run --seconds 1", "RESULT run: PASSED");
	existing(out, "torture, fabric to host", "torture --words 20000 --seconds 20", "RESULT torture: PASSED");
	existing(out, "torture --map so", "torture --map so --words 300000", "RESULT torture (map so): PASSED");
	existing(out, "torture --map wc-wmb", "torture --map wc-wmb --words 200000", "RESULT torture (map wc-wmb): PASSED");
	existing(out, "osd", "osd --repeat 1 --rows 4", "RESULT osd: PASSED");
	existing(out, "spi-osd", "spi-osd --rows 4", "RESULT spi-osd: PASSED");
	existing(out, "osd-off", "osd-off", "RESULT osd-off: PASSED");
	existing(out, "freeze", "freeze", "RESULT freeze: PASSED");
	tt_check(run_probe(out, "quiesce", 0, 0, 0) == 1 && has(out, "quiesce: NO stopped value"),
	         "quiesce: nothing ran, so no stopped value, as on a freshly loaded core");
	tt_check(run_probe(out, "torture --map so --words 200000", 20000000.0, F_INJ_PAYLOAD, 5) == 1 && has(out, "FAILED"),
	        "an injected payload error fails the full-ring torture, as it did");
}

/* the checks every good band run shares; returns the number of log entries */
static unsigned band_good(const char *name, const char *out, int rc, unsigned maxlen)
{
	unsigned long long cycles = 0, records = 0, words = 0, cps = 0;
	const struct f_stats *st = shared_st;
	unsigned n = st->blog_n < BLOG_CAP ? st->blog_n : BLOG_CAP, i, near_hi = 0, near_lo = 0, empty = 0, in_hi = 0,
	         in_lo = 0, judged = 0, bad_draw = 0, hi_min = TS_WORDS, hi_max = 0, lo_min = TS_WORDS, lo_max = 0;

	tt_check(rc == 0 && has(out, "RESULT torture --band") && has(out, "): PASS ("), "%s: exit %d, a PASS line", name, rc);
	tt_check(band_counts(out, &cycles, &records, &words, &cps), "%s: the counts line is there", name);
	tt_check(cycles > 20 && cps >= 2, "%s: %llu cycles, %llu checkpoints (the ring ran empty more than once)", name, cycles,
	        cps);
	tt_check(st->blog_n == cycles, "%s: the model's log has %u cycles, the probe says %llu", name, st->blog_n, cycles);
	tt_check(st->idx4_reads == cycles + (cps - 1), "%s: C_IDX[4] read once per cycle: %u reads for %llu cycles + %llu "
	        "checkpoint reads", name, st->idx4_reads, cycles, cps - 1);
	tt_check(st->viol_pub == 0, "%s: %u accesses to a mailbox word other than the ring payload and head while publishing",
	        name, st->viol_pub);
	tt_check(st->viol_wait == 0, "%s: %u accesses to any mailbox word while waiting", name, st->viol_wait);
	tt_check(st->idx4_empty > 0, "%s: the empty-ring checks polled C_IDX[4] (%u)", name, st->idx4_empty);
	tt_check(st->ts_fault == 0 && st->ts_errors == 0, "%s: the sink saw no fault and no error (%u)", name, st->ts_errors);
	tt_check(st->ts_deepest > 0 && st->ts_deepest < TS_WORDS, "%s: the fabric's deepest fill %u is below %u", name,
	        st->ts_deepest, TS_WORDS);
	tt_check(!has(out, "FAIL "), "%s: no FAIL line in the output", name);
	tt_check(has(out, "[0-1023]") && has(out, "[7168-8191]"), "%s: both histograms are printed in 8 bins", name);
	for (i = 0; i < n; i++) {
		const struct f_brec *r = &shared_log[i];
		if (!(r->hi >= TS_WORDS * 50u / 100u && r->hi <= TS_WORDS * 90u / 100u && r->lo >= TS_WORDS * 5u / 100u &&
		      r->lo <= TS_WORDS * 45u / 100u))
			bad_draw++;
		if (r->hi > hi_max) hi_max = r->hi;
		if (r->hi < hi_min) hi_min = r->hi;
		if (r->lo > lo_max) lo_max = r->lo;
		if (r->lo < lo_min) lo_min = r->lo;
	}
	tt_check(bad_draw == 0, "%s: every cycle's HI is in 50-90%% and LO in 5-45%% of the ring (HI %u..%u, LO %u..%u)", name,
	        hi_min, hi_max, lo_min, lo_max);
	for (i = 0; i + 1 < n; i++) {
		const struct f_brec *r = &shared_log[i], *nx = &shared_log[i + 1];
		/* the publish ends within one record and the clock's slack of HI; the next read is about LO */
		if (r->peak + maxlen + 700u >= r->hi && r->peak <= r->hi + maxlen + 700u) in_hi++;
		if (r->peak >= r->hi) near_hi++;
		if (nx->fill + 600u >= r->lo && nx->fill <= r->lo + 1500u) in_lo++;
		if (nx->fill == 0) empty++;
		if (nx->fill > 0 && nx->fill < TS_WORDS) near_lo++;
		judged++;
	}
	tt_check(judged > 20 && in_hi * 100u >= judged * 95u, "%s: the publish ended near HI in %u of %u cycles", name, in_hi,
	        judged);
	tt_check(near_hi * 100u >= judged * 90u, "%s: the estimate reached HI in %u of %u cycles", name, near_hi, judged);
	tt_check(in_lo * 100u >= judged * 80u, "%s: the next read found about LO in %u of %u cycles", name, in_lo, judged);
	tt_check(near_lo * 100u >= judged * 95u, "%s: the lowest fill read was above 0 in %u of %u cycles (%u empty)", name,
	        near_lo, judged, empty);
	return n;
}

static void case_band(char *out)
{
	int rc;
	unsigned n;
	heading("probe test 2: --band, strongly ordered, the fabric at the rate the probe assumes");
	rc = run_probe(out, "torture --map so --band --words 1500000 --empty-words 400000", 6300000.0, 0, 0);
	n = band_good("band so", out, rc, 256);
	tt_check(n > 20, "band so: %u cycles logged", n);
	heading("probe test 3: --band on write-combined variants");
	rc = run_probe(out, "torture --map wc-dmb --band --words 800000 --empty-words 300000", 6300000.0, 0, 0);
	band_good("band wc-dmb", out, rc, 256);
	rc = run_probe(out, "torture --map wc --band --words 600000 --empty-words 250000 --maxlen 64 --seed 7", 6300000.0, 0, 0);
	band_good("band wc", out, rc, 64);
}

static void case_slow(char *out)
{
	int rc;
	heading("probe test 4: --band with the fabric 40% slower than assumed: never full, never faulted");
	rc = run_probe(out, "torture --map so --band --words 700000 --empty-words 300000 --seed 3", 3800000.0, 0, 0);
	tt_check(rc == 0 && has(out, "): PASS ("), "slower fabric: exit %d, a PASS line", rc);
	tt_check(shared_st->ts_fault == 0 && shared_st->ts_errors == 0 && shared_st->ts_deepest < TS_WORDS,
	        "slower fabric: no fault, no error, deepest %u below %u", shared_st->ts_deepest, TS_WORDS);
	tt_check(shared_st->viol_pub == 0 && shared_st->viol_wait == 0, "slower fabric: no stray mailbox access");
	heading("probe test 5: --band with the fabric far slower than assumed: never full, but FAIL, not part-full");
	rc = run_probe(out, "torture --map so --band --words 40000 --empty-words 1000000", 150000.0, 0, 0);
	tt_check(rc == 1 && has(out, "): FAIL (") && has(out, "NOT part-full"), "far slower fabric: exit %d, FAIL and the "
	        "coverage reason", rc);
	tt_check(shared_st->ts_fault == 0 && shared_st->ts_errors == 0 && shared_st->ts_deepest < TS_WORDS,
	        "far slower fabric: the ring was never full (deepest %u) nor faulted, and no word was lost", shared_st->ts_deepest);
}

static void case_inject(char *out)
{
	static const struct { const char *name; int inj; unsigned at; } k[3] = {
		{ "a flipped payload word", F_INJ_PAYLOAD, 150000 }, { "two adjacent words swapped", F_INJ_REORDER, 150000 },
		{ "a record's seq skewed", F_INJ_SEQ, 300 } };
	unsigned i;
	heading("probe test 6: --band against an injected fault: each a FAIL, named by the VERDICT");
	for (i = 0; i < 3; i++) {
		int rc = run_probe(out, "torture --map wc-dsb --band --words 500000 --empty-words 200000", 6300000.0, k[i].inj,
		                   k[i].at);
		tt_check(rc == 1 && has(out, "): FAIL ("), "%s: exit %d, a FAIL line", k[i].name, rc);
		tt_check(has(out, "FAIL band: checkpoint") && has(out, "VERDICT errors"), "%s: the VERDICT errors check names it",
		        k[i].name);
		tt_check(shared_st->ts_errors >= (k[i].inj == F_INJ_REORDER ? 2u : 1u), "%s: the sink counted %u errors", k[i].name,
		        shared_st->ts_errors);
		if (k[i].inj == F_INJ_REORDER)
			tt_check(!has(out, "FAIL band: checkpoint 1: VERDICT sum") && !has(out, "FAIL band: checkpoint 2: VERDICT sum") &&
			        !has(out, "FAIL band: checkpoint 3: VERDICT sum"), "%s: the sum alone does not see it, the per-word "
			        "check does", k[i].name);
	}
}

static void case_misc(char *out)
{
	struct f_brec first[24];
	unsigned i, same = 0;
	int rc;
	heading("probe test 7: the seed fixes the bands; bad options are usage errors");
	run_probe(out, "torture --map so --band --words 700000 --empty-words 10000000 --seed 11", 6300000.0, 0, 0);
	memcpy(first, shared_log, sizeof first);
	run_probe(out, "torture --map so --band --words 700000 --empty-words 10000000 --seed 11", 6300000.0, 0, 0);
	for (i = 0; i < 24; i++) same += first[i].lo == shared_log[i].lo && first[i].hi == shared_log[i].hi;
	tt_check(same == 24 && shared_st->blog_n >= 24, "the same seed, the same first 24 HI and LO (%u of 24)", same);
	run_probe(out, "torture --map so --band --words 700000 --empty-words 10000000 --seed 12", 6300000.0, 0, 0);
	same = 0;
	for (i = 0; i < 24; i++) same += first[i].lo == shared_log[i].lo && first[i].hi == shared_log[i].hi;
	tt_check(same < 24, "another seed, other bands (%u of 24 equal)", same);
	rc = run_probe(out, "torture --band --words 1000", 0, 0, 0);
	tt_check(rc == 2, "--band without --map: exit %d (usage)", rc);
	rc = run_probe(out, "torture --map so --band --rate 5 --words 1000", 0, 0, 0);
	tt_check(rc == 2, "--rate 5: exit %d (usage)", rc);
	rc = run_probe(out, "torture --map so --band --maxlen 9000 --words 1000", 0, 0, 0);
	tt_check(rc == 2, "--maxlen 9000: exit %d (usage)", rc);
}

int main(void)
{
	char *out = malloc(OUT_MAX);
	void *m = mmap(NULL, sizeof(struct f_stats) + BLOG_CAP * sizeof(struct f_brec), PROT_READ | PROT_WRITE,
	               MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (!out || m == MAP_FAILED) {
		perror("ocram_probe_test");
		return 2;
	}
	shared_st = f_st = m;
	shared_log = f_blog = (struct f_brec *)(shared_st + 1);
	case_existing(out);
	case_band(out);
	case_slow(out);
	case_inject(out);
	case_misc(out);
	printf("ocram_probe_test: %u checks, %u failures\n", t_checks, t_fails);
	free(out);
	return t_fails ? 1 : 0;
}
