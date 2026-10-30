// chg_systick against stk_* in bus.c: directed cases, then random register
// traffic at random board times (included by tb_periph.cpp).
//
// Kept out of the random traffic, as the C does not do what a counter does
// there (see chg_systick.sv): CMP high words of 2^20 and up (so CMP is never
// 2^64-1, where the C divides by zero with reload), and any write while the
// count is at 2^62 or more (a down count without reload that has passed 0):
// stk_plan's next_match would wrap past 2^64. Such a write is replaced by a
// CTLR write with INIT, which brings the count back.

static const uint32_t STK = 0xE000F000u;

static uint32_t stk_val(uint32_t off)
{
    switch (off) {
    case 0x00: {
        uint32_t v = rn(64);                        // STE STIE STCLK STRE MODE INIT
        if (rn(4) == 0) v &= ~0x20u;
        if (rn(10) == 0) v |= 0x80000000u;         // SWIE
        if (rn(10) == 0) v |= rnd() & 0x7fffff00u;  // other bits are kept as written
        if (rn(3)) v |= 1;                          // mostly running
        return v;
    }
    case 0x04: return rn(3) ? 0 : rnd();
    case 0x08: return rn(4) ? rn(3000) : rnd();
    case 0x0c: return rn(4) ? 0 : rn(1u << 20);
    case 0x10: return rn(4) ? rn(2000) : rn(3) ? rn(100000) : rnd();
    case 0x14: return rn(5) ? 0 : rn(1u << 20);
    }
    return rnd();
}

static void stk_write(uint32_t off, uint32_t v) { acc(true, STK + off, v, 4); }

static int test_systick(long n, uint32_t seed)
{
    start(seed);
    // directed: down with reload and CNT above CMP (the C's odd count)
    stk_write(0x10, 10); stk_write(0x14, 0);
    stk_write(0x08, 100);
    stk_write(0x00, 0x1f);                  // STE STIE STCLK STRE MODE
    for (int i = 0; i < 300; i++) { acc(false, STK + 0x08, 0, 4); acc(false, STK + 0x04, 0, 4); step(); }
    // the same with the divide by 8, and a CMP write while odd
    stk_write(0x00, 0x1a);
    stk_write(0x08, 1000); stk_write(0x00, 0x1b);
    for (int i = 0; i < 400; i++) { acc(false, STK + 0x08, 0, 4); advance_to(now + 3); if (i == 100) stk_write(0x10, 7); }
    // up with reload, the SDK's way: CMP, INIT, STIE, clear SR in the "handler"
    stk_write(0x00, 0); stk_write(0x10, 47); stk_write(0x14, 0); stk_write(0x04, 0);
    stk_write(0x00, 0x2f);
    for (int i = 0; i < 2000; i++) {
        step();
        if (top->irq_systick) { acc(false, STK + 0x04, 0, 4); stk_write(0x04, 0); }
    }
    // SWIE, INIT in both directions
    stk_write(0x00, 0x80000000u); stk_write(0x00, 0x31); stk_write(0x00, 0x21);
    for (int i = 0; i < 100; i++) step();
    // byte and halfword accesses into each register
    for (uint32_t off = 0; off < 0x18; off++) { acc(false, STK + off, 0, 1); if (!(off & 1)) acc(false, STK + off, 0, 2); }
    acc(true, STK + 0x09, 0x55, 1); acc(true, STK + 0x12, 0x1234, 2); acc(false, STK + 0x20, 0, 4);
    // matches inside an instruction: the access issues at or just after the match, in an
    // instruction that began before it (the C has not run stk_event yet)
    stk_write(0x00, 0); stk_write(0x10, 20); stk_write(0x14, 0); stk_write(0x04, 0);
    for (int k = 0; k < 3000; k++) {
        if (k % 500 == 0) stk_write(0x00, (k / 500 & 1 ? 0x0b : 0x2f) | (k / 1000 == 1 ? 0x10 : 0));
        const uint64_t e = m->systick.next_match;
        if (e == UINT64_MAX || e < now + 4) { advance_to(now + 1); stk_write(0x04, 0); continue; }
        const uint64_t t = e + rn(3);
        const uint64_t s = e - 1 - rn(3);
        advance_to(s);
        window(s, t + 1 + rn(3));
        advance_to(t);
        switch (rn(5)) {
        case 0: case 1: acc(false, STK + 0x04, 0, 4); break;
        case 2: stk_write(0x04, 0); break;
        case 3: stk_write(0x00, m->systick.ctlr); break;     // the same CTLR: drops the match in the C
        case 4: acc(false, STK + 0x08, 0, 4); break;
        }
        if (rn(3) == 0) { advance_to(now + 2); acc(false, STK + 0x04, 0, 4); stk_write(0x04, 0); }
    }
    printf("systick directed: %ld differences\n", st.errors());

    for (long i = 0; i < n; i++) {
        gap();
        const uint32_t off = rn(20) == 0 ? (rn(64) * 4) : rn(6) * 4;
        int size = rn(6) ? 4 : rsize();
        const uint32_t a = raddr(STK + off, size);
        if (rn(2)) { acc(false, a, 0, size); continue; }
        // the guard (see the top)
        int w = 0;
        m->cycles = now;
        const uint32_t hi = bus_read_slow(m, STK + 0x0c, 4, &w);
        if (hi >= 0x40000000u) { stk_write(0x00, stk_val(0) | 0x20 | (rn(2) ? 0x10 : 0)); continue; }
        if (size != 4 || (a & 3)) {
            // narrow writes replace the whole register with the value in its lane: not into
            // the high words (the guard), and mostly small
            if (((a & 0xfc) == 0x0c || (a & 0xfc) == 0x14)) { acc(false, a, 0, size); continue; }
            acc(true, a, rn(4) ? rn(256) : rnd(), size);
            continue;
        }
        stk_write(off, stk_val(off));
    }
    return report("systick");
}
