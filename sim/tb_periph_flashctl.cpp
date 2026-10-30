// chg_flashctl against flash_ctl_*, flash_mem_write, flash_erase in bus.c:
// the SDK's fast page erase and program, then random controller and flash
// store traffic over a randomly filled flash. The flash and the dirty-page
// set are compared after every access to either (included by tb_periph.cpp).

static const uint32_t FLC = 0x40022000u;

static void fl_unlock(bool fast)
{
    acc(true, FLC + 0x04, 0x45670123u, 4); acc(true, FLC + 0x04, 0xCDEF89ABu, 4);
    if (fast) { acc(true, FLC + 0x24, 0x45670123u, 4); acc(true, FLC + 0x24, 0xCDEF89ABu, 4); }
}

static uint32_t fl_target()
{
    const int r = rn(20);
    uint32_t a = r == 0 ? rnd() & 0x1ffff : r == 1 ? 0xF700 + rn(0x200) : 0x3000 + rn(0xC800);
    if (rn(2)) a |= CHG_FLASH_ALIAS;
    return a;
}

static int test_flash(long n, uint32_t seed)
{
    start(seed);
    for (uint32_t a = 0; a < CHG_FLASH_SIZE; a++) m->flash[a] = (uint8_t)rnd();
    memcpy(rflash, m->flash, CHG_FLASH_SIZE);
    // directed: the save path (fast erase, buffer, program), as STARLIFT does it
    fl_unlock(true);
    acc(true, FLC + 0x10, 1u << 17, 4); acc(true, FLC + 0x14, 0x0800F600u, 4);
    acc(true, FLC + 0x10, (1u << 17) | 0x40, 4); acc(false, FLC + 0x0c, 0, 4); acc(true, FLC + 0x0c, 0x20, 4);
    acc(true, FLC + 0x10, 1u << 16, 4); acc(true, FLC + 0x10, (1u << 16) | (1u << 19), 4);
    for (uint32_t w = 0; w < 64; w++) acc(true, 0x0800F600u + 4 * w, rnd(), 4);
    acc(true, FLC + 0x10, (1u << 16) | 0x40, 4); acc(false, FLC + 0x10, 0, 4);
    acc(true, FLC + 0x10, 0x80, 4); acc(false, FLC + 0x10, 0, 4);
    // half words with PG, a 1 KB erase, a mass erase
    fl_unlock(false);
    acc(true, FLC + 0x10, 1, 4);
    for (int i = 0; i < 20; i++) acc(true, 0x5000 + 2 * i + (i & 1 ? 0 : 1), rnd(), 2);
    acc(true, FLC + 0x10, 2, 4); acc(true, FLC + 0x14, 0x4321, 4); acc(true, FLC + 0x10, 0x42, 4);
    acc(true, FLC + 0x10, 4, 4); acc(true, FLC + 0x10, 0x44, 4);
    printf("flash directed: %ld differences\n", st.errors());

    for (uint32_t a = 0; a < CHG_FLASH_USER; a++) m->flash[a] = (uint8_t)rnd();
    memcpy(rflash, m->flash, CHG_FLASH_SIZE);
    for (long i = 0; i < n; i++) {
        gap();
        const int r = rn(100);
        if (r < 1) { do_reset(false); continue; }
        if (r < 6) { fl_unlock(rn(2)); continue; }
        if (r < 9) { acc(true, FLC + (rn(2) ? 0x04 : 0x24), rn(2) ? 0x45670123u : rnd(), 4); continue; }
        if (r < 20) {
            // CTLR: PG PER MER STRT LOCK FLOCK PAGE_PG PAGE_ER BUF_LOAD BUF_RST
            static const uint32_t bits[] = { 1, 2, 4, 0x40, 0x80, 0x8000, 1u << 16, 1u << 17, 1u << 18, 1u << 19 };
            uint32_t v = 0;
            for (int b = 0; b < 10; b++) if (rn(b == 2 ? 40 : b == 4 || b == 5 ? 8 : 3) == 0) v |= bits[b];
            if (rn(10) == 0) v |= rnd();
            if ((v & 0x40) && rn(3)) v &= ~4u;                  // mass erase is slow: seldom
            acc(true, FLC + 0x10, v, 4);
            continue;
        }
        if (r < 26) { acc(true, FLC + 0x14, fl_target(), 4); continue; }
        if (r < 30) { acc(true, FLC + 0x0c, rnd(), 4); continue; }
        if (r < 40) {
            static const uint32_t ro[] = { 0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28, 0x300 };
            const int size = rsize();
            acc(false, raddr(FLC + ro[rn(12)], size), 0, size);
            continue;
        }
        if (r < 43) { acc(true, FLC + rn(12) * 4, rnd(), 4); continue; }
        // a store into the flash
        const int size = rsize();
        uint32_t a = fl_target();
        if (rn(6)) a &= ~(uint32_t)(size - 1);
        acc(true, a, rnd(), size);
    }
    return report("flash");
}
