// chg_adc against adc_read / adc_write in bus.c: conversions at random
// board times (the noise takes in the cycle count), the EOC flag, the plain
// registers (included by tb_periph.cpp).

static const uint32_t ADC = 0x40012400u;

static int test_adc(long n, uint32_t seed)
{
    start(seed);
    for (long i = 0; i < n; i++) {
        gap();
        const int r = rn(100);
        if (r < 1) { do_reset(false); continue; }
        if (r < 30) { acc(true, ADC + 0x08, (rnd() & 0xff80ffff) | (rn(3) ? 0x00400000u : 0), 4); continue; }
        if (r < 50) { acc(false, ADC + 0x4c + (rn(5) == 0 ? rn(4) : 0), 0, rn(4) ? 4 : rsize()); continue; }
        const uint32_t off = rn(10) ? rn(24) * 4 : rn(256) * 4;
        const int size = rsize();
        const uint32_t a = raddr(ADC + off, size);
        if (rn(2)) acc(false, a, 0, size);
        else acc(true, a, rn(2) ? rnd() : rnd() | 0x00400000u, size);
    }
    return report("adc");
}
