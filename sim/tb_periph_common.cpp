// The harness the tb_periph_*.cpp tests share (included by tb_periph.cpp).
//
// The RTL is chg_periph_sim (core/rtl/periph): every module of this
// directory behind one decoder. Beside it runs a C ChgMachine with no CPU:
// the testbench is the CPU. Board time 'now' is m->cycles. Each board cycle
// is one clock with ce high in the RTL, and in the C m->cycles = now and
// then bus_events while something is due (what chg_run does at the top of
// its loop before the CPU's next instruction; the loop goes on while
// several things are overdue, which only happens after a stall). Every
// cycle the irq lines, the pins and the buzzer level are compared.
//
// The C takes a peripheral's events (stk_event, tim_events) only at an
// instruction boundary, the top of chg_run's loop; the RTL at its
// 'boundary' strobe. Here every cycle is a boundary except inside windows
// (nb_lo, nb_hi) around accesses: the replay takes them from the trace (the
// instruction's boundary and the next one), the random tests make them up.
//
// An access at board time t: bus_read_slow / bus_write_slow on the C with
// m->cycles = t, then the same access on the RTL bus with ce low until the
// ack. Read values, the stall (board cycles the C added to m->cycles) and
// the SWIE pending pulse are compared; after a flash access the whole flash
// and the dirty-page set. A stall is then run as that many ce clocks.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <set>
#include <string>
#include "Vchg_periph_sim.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
#include "loader.h"
#include "sdcard.h"
}

static Vchg_periph_sim *top;
static ChgMachine *m;
static uint8_t rflash[CHG_FLASH_SIZE];      // the RTL's flash, behind port B
static uint64_t now;
static uint64_t nb_lo, nb_hi;               // no boundary for times in (nb_lo, nb_hi)
static long n_bnd;
static bool mack_next;
static uint32_t mrdata_next;
static std::set<int> rtl_pages;
static long sw_pulses;
static std::string serial_rtl;

struct Stats {
    long reads = 0, writes = 0, c_only = 0, cycles = 0, flash_checks = 0;
    long bad_read = 0, bad_stall = 0, bad_irq = 0, bad_pin = 0, bad_buzz = 0, bad_flash = 0,
         bad_pages = 0, bad_pend = 0, bad_trace = 0, timeouts = 0;
    long errors() const { return bad_read + bad_stall + bad_irq + bad_pin + bad_buzz + bad_flash + bad_pages +
                                 bad_pend + bad_trace + timeouts; }
};
static Stats st;
static int shown;
// coverage: cycles each irq line was high, edges of each pin, reads that were not 0
static long cov_irq[4], cov_pin[6], cov_nz_reads, cov_sw;
static bool cov_last[6];
#define SHOW(...) do { if (shown++ < 25) { printf("t=%llu: ", (unsigned long long)now); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rng = 1;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static uint32_t rn(uint32_t n) { return n ? rnd() % n : 0; }

static bool spi_nss_c() { return !((m->spi.ctlr2 & 0x04) && (m->spi.ctlr1 & 0x40)); }

static void clk1()
{
    top->mack = mack_next;
    top->mrdata = mrdata_next;
    mack_next = false;
    top->spi_nss = spi_nss_c();
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    if (top->mreq) {
        // port B: the flash, answered on the next clock
        const uint32_t a = top->maddr & 0xffff;
        if (top->mwe) {
            for (int i = 0; i < (int)top->msize && a + i < CHG_FLASH_SIZE; i++)
                rflash[a + i] = (uint8_t)(top->mwdata >> (8 * i));
            mrdata_next = 0;
        } else {
            uint32_t v = 0;
            for (int i = 0; i < 4 && a + i < CHG_FLASH_SIZE; i++) v |= (uint32_t)rflash[a + i] << (8 * i);
            mrdata_next = v;
        }
        mack_next = true;
    }
    if (top->fl_page_stb) rtl_pages.insert(top->fl_page);
    if (top->sw_pend) sw_pulses++;
    if (top->serial_stb) serial_rtl += (char)top->serial_byte;
}

static void c_events()
{
    for (int k = 0; k < 64 && m->cycles >= m->next_event; k++) {
        bus_events(m);
        chg_schedule(m);
    }
}

static bool buzz_c()
{
    const ChgBuzzState *b = &m->buzzer.cur;
    if (!b->pwm) return b->level;
    const uint64_t ph = (m->cycles - b->origin) % b->period;
    return ph < b->high ? b->level : !b->level;
}

static void compare()
{
    st.cycles++;
    const uint64_t l = m->pfic.line;
    const bool ci[4] = { (bool)(l >> CHG_IRQ_SYSTICK & 1), (bool)(l >> CHG_IRQ_TIM1_UP & 1),
                         (bool)(l >> CHG_IRQ_TIM2_UP & 1), (bool)(l >> CHG_IRQ_TIM3 & 1) };
    const bool ri[4] = { (bool)top->irq_systick, (bool)top->irq_tim1, (bool)top->irq_tim2, (bool)top->irq_tim3 };
    static const char *in[4] = { "SYSTICK", "TIM1_UP", "TIM2_UP", "TIM3" };
    for (int i = 0; i < 4; i++) {
        if (ci[i] != ri[i]) { st.bad_irq++; SHOW("irq %s c %d rtl %d", in[i], ci[i], ri[i]); }
        cov_irq[i] += ci[i];
    }
    const bool cp[6] = { gpio_pin(m, 0, 4), gpio_pin(m, 1, 0), gpio_pin(m, 1, 9), gpio_pin(m, 1, 10),
                         gpio_pin(m, 1, 11), gpio_pin(m, 1, 12) };
    const bool rp[6] = { (bool)top->pa4, (bool)top->pb0, (bool)top->pb9, (bool)top->pb10, (bool)top->pb11,
                         (bool)top->pb12 };
    static const char *pn[6] = { "PA4", "PB0", "PB9", "PB10", "PB11", "PB12" };
    for (int i = 0; i < 6; i++)
        if (cp[i] != rp[i]) { st.bad_pin++; SHOW("pin %s c %d rtl %d", pn[i], cp[i], rp[i]); }
    for (int i = 0; i < 6; i++) { cov_pin[i] += cp[i] != cov_last[i]; cov_last[i] = cp[i]; }
    const bool bz = buzz_c();
    if (bz != (bool)top->pb10) { st.bad_buzz++; SHOW("buzzer.cur c %d rtl pb10 %d (pwm %d)", bz, top->pb10, m->buzzer.cur.pwm); }
}

static bool bnd_at(uint64_t t) { return !(t > nb_lo && t < nb_hi); }

// an instruction boundary at 'now' on both
static void boundary()
{
    c_events();
    top->boundary = 1;
    clk1();
    top->boundary = 0;
    n_bnd++;
}

// one board cycle on both
static void step()
{
    top->ce = 1;
    clk1();
    top->ce = 0;
    now++;
    m->cycles = now;
    if (bnd_at(now)) boundary();
    compare();
}

// no boundary in (lo, hi) from now on
static void window(uint64_t lo, uint64_t hi) { nb_lo = lo; nb_hi = hi; }

static void advance_to(uint64_t t) { while (now < t) step(); }

static void check_flash()
{
    st.flash_checks++;
    if (memcmp(rflash, m->flash, CHG_FLASH_SIZE)) {
        st.bad_flash++;
        for (uint32_t a = 0; a < CHG_FLASH_SIZE; a++)
            if (rflash[a] != m->flash[a]) { SHOW("flash %04x c %02x rtl %02x", a, m->flash[a], rflash[a]); break; }
        memcpy(rflash, m->flash, CHG_FLASH_SIZE);       // carry on from the C's
    }
    std::set<int> cpages;
    for (int p = 0; p < (int)(CHG_FLASH_SIZE / CHG_PAGE_SIZE); p++) if (m->flash_dirty[p]) cpages.insert(p);
    if (cpages != rtl_pages) {
        st.bad_pages++;
        SHOW("dirty pages: c %zu rtl %zu", cpages.size(), rtl_pages.size());
        rtl_pages = cpages;
    }
}

static bool is_flash_addr(uint32_t a)
{
    return a < CHG_FLASH_SIZE || (a >= CHG_FLASH_ALIAS && a < CHG_FLASH_ALIAS + CHG_FLASH_SIZE);
}

// an access that is the C's alone (SPI, DMA, PFIC: modules not tested here)
static uint32_t acc_c(bool wr, uint32_t addr, uint32_t val, int size)
{
    int wait = 0;
    m->cycles = now;
    uint32_t v = 0;
    if (wr) bus_write_slow(m, addr, val, size, &wait);
    else v = bus_read_slow(m, addr, size, &wait);
    chg_schedule(m);
    st.c_only++;
    if (m->cycles != now) { now = m->cycles; }   // nothing of theirs stalls; kept honest
    return v;
}

// an access to both; returns the C's value
static uint32_t acc(bool wr, uint32_t addr, uint32_t val, int size)
{
    int wait = 0;
    m->cycles = now;
    const uint64_t pend0 = m->pfic.pending;
    uint32_t cv = 0;
    if (wr) bus_write_slow(m, addr, val, size, &wait);
    else cv = bus_read_slow(m, addr, size, &wait);
    const uint64_t cstall = m->cycles - now;
    m->cycles = now;
    chg_schedule(m);
    const long sw0 = sw_pulses;

    top->req = 1; top->we = wr; top->addr = addr; top->size = size; top->wdata = val;
    clk1();
    top->req = 0;
    long n = 0;
    while (!top->ack && n < 2000000) { clk1(); n++; }
    if (!top->ack) { st.timeouts++; SHOW("no ack: %s %08x size %d", wr ? "write" : "read", addr, size); }
    const uint32_t rv = top->rdata;
    const uint32_t rstall = top->stall;
    clk1();                                 // the clock after the ack (ack falls)
    if (wr) st.writes++; else st.reads++;
    if (!wr && cv) cov_nz_reads++;
    if (!wr && rv != cv) { st.bad_read++; SHOW("read %08x size %d: c %08x rtl %08x", addr, size, cv, rv); }
    if (rstall != cstall) { st.bad_stall++; SHOW("%s %08x: stall c %llu rtl %u", wr ? "write" : "read", addr, (unsigned long long)cstall, rstall); }
    const bool cpend = (m->pfic.pending & ~pend0) >> CHG_IRQ_SW & 1;
    const bool rpend = sw_pulses != sw0;
    cov_sw += cpend;
    if (cpend != rpend) { st.bad_pend++; SHOW("SW pending: c %d rtl %d", cpend, rpend); }
    m->pfic.pending &= ~(1ull << CHG_IRQ_SW);
    // the stall: board time runs on, within the instruction (no boundary)
    const uint64_t t0 = now;
    for (uint64_t i = 0; i < cstall; i++) { top->ce = 1; clk1(); top->ce = 0; }
    now += cstall;
    m->cycles = now;
    if (cstall) {
        if (nb_hi <= t0) nb_lo = t0;
        if (nb_hi < now + 1) nb_hi = now + 1;
        compare();
    }
    if ((addr >= 0x40022000u && addr < 0x40022400u) || (wr && is_flash_addr(addr))) check_flash();
    return cv;
}

static void set_buttons(uint8_t b) { chg_set_buttons(m, b); top->buttons = b; }

static void do_reset(bool power)
{
    chg_reset(m, power);
    chg_schedule(m);
    top->por = power; top->sreset = !power;
    clk1();
    top->por = 0; top->sreset = 0;
    clk1();
}

// a fresh machine and RTL at board time 0
static void start(uint32_t seed)
{
    rng = seed ? seed : 1;
    if (!m) m = (ChgMachine *)calloc(1, sizeof(ChgMachine));
    chg_init(m);
    memcpy(rflash, m->flash, CHG_FLASH_SIZE);
    if (!top) top = new Vchg_periph_sim;
    now = 0;
    nb_lo = nb_hi = 0;
    rtl_pages.clear();
    top->init = 1; top->por = 1; top->sreset = 0; top->ce = 0; top->req = 0; top->buttons = 0;
    clk1();
    top->init = 0; top->por = 0;
    clk1();
    chg_schedule(m);
}

static int report(const char *name)
{
    printf("%s: %ld reads, %ld writes, %ld C-only accesses, %llu board cycles, %ld cycles compared, %ld boundaries, %ld flash checks\n",
           name, st.reads, st.writes, st.c_only, (unsigned long long)now, st.cycles, n_bnd, st.flash_checks);
    printf("%s: differences: read %ld stall %ld irq %ld pin %ld buzzer %ld flash %ld pages %ld pending %ld trace %ld timeout %ld => %ld\n",
           name, st.bad_read, st.bad_stall, st.bad_irq, st.bad_pin, st.bad_buzz, st.bad_flash, st.bad_pages,
           st.bad_pend, st.bad_trace, st.timeouts, st.errors());
    printf("%s: coverage: irq-high cycles systick %ld tim1 %ld tim2 %ld tim3 %ld; pin edges PA4 %ld PB0 %ld PB9 %ld PB10 %ld PB11 %ld PB12 %ld; nonzero reads %ld; SW pends %ld\n",
           name, cov_irq[0], cov_irq[1], cov_irq[2], cov_irq[3], cov_pin[0], cov_pin[1], cov_pin[2], cov_pin[3],
           cov_pin[4], cov_pin[5], cov_nz_reads, cov_sw);
    return st.errors() ? 1 : 0;
}

// ---------------------------------------------------------------------------
// random access helpers
// ---------------------------------------------------------------------------

static int rsize() { int r = rn(10); return r < 6 ? 4 : r < 8 ? 2 : 1; }
// an address in the register at 'reg' for a 'size' access: mostly aligned, sometimes not
static uint32_t raddr(uint32_t reg, int size)
{
    if (rn(8) == 0) return reg + rn(4);
    return reg + (size == 4 ? 0 : size == 2 ? 2 * rn(2) : rn(4));
}
// time passes, then an access: half the time inside an instruction that began a few cycles
// before it and ends a few after (no boundary there)
static void gap()
{
    const uint32_t r = rn(100);
    const uint64_t t = now + (r < 50 ? rn(8) : r < 90 ? rn(200) : rn(5000));
    if (rn(2)) {
        uint64_t s = t - rn(5);
        if (s < now) s = now;
        advance_to(s);
        window(s, t + 1 + rn(4));
    }
    advance_to(t);
}
