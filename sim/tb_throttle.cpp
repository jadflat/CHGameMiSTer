// chg_throttle.sv and chg_speed.sv with a stand-in board (sim/throttle_top.sv) that can do 2 cycles
// in 3 clocks (64 M a second, 133% of the real 48 M) except where it stalls:
//  1. ff low, random stalls: run is the old throttle's, clock for clock (2 s)
//  2. normal: 100.0% of real time (24,000,000 cycles per 0.5 s); the speed display says 100
//  3. fast forward: faster (the stand-in's 133%), the display says 133
//  4. back to normal: the board does not catch up (the first 20 ms hold 100%), 100 again
//  5. pause (the OSD open): the board stands still (no cycle after the first few clocks), also with ff on and
//     with the board running flat out; on resuming it does not catch up (1/60 s of owed time would show as a
//     burst in the first 20 ms) and is at 100% again
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include "Vthrottle_top.h"
#include "verilated.h"
static Vthrottle_top *top;
static long clocks, cycles;
static int fails;
static uint32_t rng = 12345;
static int pat;
static void tick(bool random_stall)
{
    if (random_stall) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; top->stall = (rng % 7) < 3; }
    else top->stall = (++pat % 3) == 0;              // 2 clocks in 3
    top->clk = 0; top->eval();
    if (top->ce) cycles++;
    top->clk = 1; top->eval(); clocks++;
}
static void phase(const char *name, bool ff, double secs, double want_pct, int want_bcd)
{
    top->ff = ff;
    long c0 = cycles, n = (long)(secs * 96e6);
    long first = 0;
    for (long i = 0; i < n; i++) { tick(false); if (i == (long)(0.02 * 96e6)) first = cycles - c0; }
    double pct = (double)(cycles - c0) / (secs * 48e6) * 100, f20 = (double)first / (0.02 * 48e6) * 100;
    unsigned b = top->bcd;
    printf("%-12s %.1f s: %.2f%% of real time (first 20 ms %.1f%%), display %x%%\n", name, secs, pct, f20, b);
    if (want_pct > 0 && (pct < want_pct - 0.5 || pct > want_pct + 0.5)) { printf("  expected about %.0f%%\n", want_pct); fails++; }
    if (want_bcd >= 0 && b != (unsigned)want_bcd) { printf("  display expected %x\n", want_bcd); fails++; }
    if (name[0] == 'b' && f20 > 105) { printf("  caught up after fast forward\n"); fails++; }
}
// pause for secs (with ff as given); cycles after the first 8 clocks must be 0; then resume at normal
static void pause_phase(const char *name, bool ff, double secs)
{
    top->ff = ff; top->pause = 1;
    for (int i = 0; i < 8; i++) tick(false);
    long c0 = cycles, n = (long)(secs * 96e6);
    for (long i = 0; i < n; i++) tick(false);
    printf("%-12s %.1f s: %ld board cycles while paused\n", name, secs, cycles - c0);
    if (cycles != c0) { printf("  the board ran while paused\n"); fails++; }
    top->pause = 0; top->ff = 0;
    phase("back after pause", false, 0.5, 100, -1);
}
// Board clock options: rate, factor in percent. Run 0.1 s to settle, then 1 s (96 M clocks) measured:
// the long-run rate must be the factor of 48 MHz within 0.01% when the stand-in board (133.33%) keeps up,
// and its own 133.33% (saturated, no hiccup) when it does not.
static const double pcts[6] = { 100, 110, 125, 150, 175, 200 };
static void rate_phase(int rate)
{
    top->rate = rate;
    for (long i = 0; i < 9600000; i++) tick(false);
    long c0 = cycles;
    for (long i = 0; i < 96000000; i++) tick(false);
    double pct = (double)(cycles - c0) / 48e6 * 100, want = pcts[rate] < 133.3 ? pcts[rate] : 133.3333;
    double err = (pct - want) / want * 100;
    unsigned b = top->bcd;
    printf("rate %d (%3.0f%%): %.4f%% of real time, off by %.4f%% (%s); display %x\n", rate, pcts[rate], pct, err,
           pcts[rate] < 133.3 ? "keeps up" : "saturated", b);
    if (err > 0.01 || err < -0.01) { printf("  rate wrong\n"); fails++; }
}
// change the rate with owed built up (the board saturated at 200%): going down to 100% must not burst
static void change_phase(int from, int to)
{
    top->rate = from;
    for (long i = 0; i < 48000000; i++) tick(false);       // 0.5 s: owed is at its limit if the board lags
    top->rate = to;
    long c0 = cycles;
    for (long i = 0; i < 1920000; i++) tick(false);       // 20 ms
    double f20 = (double)(cycles - c0) / (0.02 * 48e6) * 100;
    printf("rate %d -> %d: first 20 ms %.1f%% of real time (want at most about %.0f%%)\n", from, to, f20, pcts[to] * 1.05);
    if (f20 > pcts[to] * 1.05 + 0.5) { printf("  burst after the change\n"); fails++; }
}
int main()
{
    top = new Vthrottle_top;
    top->ff = 0; top->pause = 0; top->rate = 0;
    long diff = 0;
    for (long i = 0; i < 192000000; i++) {            // 2 s
        tick(true);
        if (top->run != top->old_run && diff++ < 3) printf("run differs from the old throttle at clock %ld\n", clocks);
    }
    printf("ff low, random stalls, %ld clocks: %ld clocks where run differs from the old throttle\n", clocks, diff);
    fails += diff != 0;
    phase("normal", false, 2.2, 100, 0x100);
    phase("fast forward", true, 1.2, 133.3, 0x133);
    phase("back to normal", false, 1.2, 100, 0x100);
    pause_phase("pause", false, 0.3);                 // owed would have grown to its 1/60 s limit in 0.3 s
    pause_phase("pause + ff", true, 0.3);
    for (int r = 0; r < 6; r++) rate_phase(r);
    change_phase(5, 0);
    change_phase(3, 1);
    change_phase(4, 0);
    rate_phase(0);
    printf("%s: %d faults\n", fails ? "FAIL" : "PASS", fails);
    return fails;
}
