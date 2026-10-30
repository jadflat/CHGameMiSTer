// chg_misc against sys_read, the USART, USB and io[] cases and the default
// of bus_read_slow / bus_write_slow (included by tb_periph.cpp).
//
// io[] traffic stays within 8 word addresses (16 with the words misaligned
// stores run on into), which chg_misc's 16-word backing holds; USB stores
// do not run past the end of m->usb (the C would write into the PFIC).

static const uint32_t IO_POOL[8] = { 0x40010400u, 0x40010404u, 0x40007000u, 0x40003000u,
                                     0x40002C08u, 0x40026000u, 0x40027010u, 0x4002FFF0u };

static int test_misc(long n, uint32_t seed)
{
    start(seed);
    // directed: the system words, a byte at a time and misaligned
    static const uint32_t sys[] = { 0x1FFFF7E0u, 0x1FFFF7E8u, 0x1FFFF7ECu, 0x1FFFF7F0u, 0x1FFFF800u,
                                    0x1FFFF83Cu, 0x1FFFF840u, 0x1FFFF704u, 0x1FFF0000u, 0x1FFFF7E4u };
    for (uint32_t a : sys)
        for (int o = 0; o < 4; o++) { acc(false, a + o, 0, 1); acc(false, a + o, 0, 2); acc(false, a + o, 0, 4); }
    printf("misc directed: %ld differences\n", st.errors());

    for (long i = 0; i < n; i++) {
        gap();
        const int r = rn(100);
        const int size = rsize();
        if (r < 2) { do_reset(false); continue; }
        if (r < 20) {
            // the system area and other places below 0x4000_0000 that are not memory
            uint32_t a = rn(4) ? 0x1FFFF700u + rn(0x160) : rn(2) ? 0x1FFF0000u + rn(0xFFF0) : 0x10000000u + rn(1 << 20);
            if (rn(2)) acc(false, a, 0, size); else acc(true, a, rnd(), size);
            continue;
        }
        if (r < 30) {
            static const uint32_t us[] = { 0x40013800u, 0x40004400u, 0x40004800u, 0x40004C00u };
            const uint32_t a = raddr(us[rn(4)] + rn(4) * 4, size);
            if (rn(2)) acc(false, a, 0, size); else acc(true, a, 'a' + rn(26), size);
            continue;
        }
        if (r < 55) {
            uint32_t a = 0x40023400u + rn(0x400);
            if (rn(3)) a &= ~(uint32_t)(size - 1);
            if ((a & 0x3ff) + size > 0x400) a -= 4;
            if (rn(2)) acc(false, a, 0, size); else acc(true, a, rnd(), size);
            continue;
        }
        if (r < 85) {
            uint32_t a = IO_POOL[rn(8)] + rn(4);
            if (rn(3)) a &= ~(uint32_t)(size - 1);
            if (rn(2)) acc(false, a, 0, size); else acc(true, a, rnd(), size);
            continue;
        }
        // nothing there
        static const uint32_t no[] = { 0x40030000u, 0x50000000u, 0xE000F100u, 0xE0010000u, 0x60001234u, 0xFFFFFFFCu };
        const uint32_t a = no[rn(6)] + rn(4);
        if (rn(2)) acc(false, a, 0, size); else acc(true, a, rnd(), size);
    }
    printf("misc: serial from the RTL: %zu bytes\n", serial_rtl.size());
    return report("misc");
}
