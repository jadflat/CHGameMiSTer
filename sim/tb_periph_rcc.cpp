// chg_rcc against the RCC cases of bus_read_slow / bus_write_slow and the
// reset flags of chg_reset: random register traffic with power-on and
// software resets among it (included by tb_periph.cpp).

static const uint32_t RCC = 0x40021000u;

static int test_rcc(long n, uint32_t seed)
{
    start(seed);
    for (long i = 0; i < n; i++) {
        gap();
        const int r = rn(100);
        if (r < 3) { do_reset(false); continue; }
        if (r < 4) { do_reset(true); continue; }
        const uint32_t off = rn(10) ? rn(12) * 4 : rn(256) * 4;
        const int size = rsize();
        const uint32_t a = raddr(RCC + off, size);
        if (rn(2)) acc(false, a, 0, size);
        else acc(true, a, off == 0x24 && rn(3) == 0 ? (rnd() | 0x01000000u) : rnd(), size);
    }
    return report("rcc");
}
