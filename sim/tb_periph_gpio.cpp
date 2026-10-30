// chg_gpio against gpio_*, af_level, chg_set_buttons and buzzer_update in
// bus.c: random port, AFIO and button traffic, with SPI1's NSS (C side
// only: SPI is not ours) and TIM1 channel 2 moving under PB10 (included by
// tb_periph.cpp).

static const uint32_t GPIO_BASE[3] = { 0x40010800u, 0x40010C00u, 0x40011000u };
static const uint32_t AFIO = 0x40010000u;
static const uint32_t TIM1 = 0x40012C00u;

// a configuration word: mostly the modes the board uses
static uint32_t cfg_word()
{
    static const uint8_t modes[] = { 0x4, 0x8, 0x3, 0x1, 0x2, 0xB, 0x9, 0xA, 0x7, 0xF, 0x0, 0xC };
    uint32_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint32_t)modes[rn(sizeof modes)] << (4 * i);
    return rn(10) ? v : rnd();
}

// TIM1 set up for channel 2 output, in a random way (shared with the timer test)
static void tim1_buzzer_op()
{
    switch (rn(8)) {
    case 0: acc(true, TIM1 + 0x18, (uint32_t)(rn(8) << 12) | (rn(4) ? 0 : rnd() & 0x8fff), 2); break;   // CHCTLR1 OC2M
    case 1: acc(true, TIM1 + 0x20, rn(4) ? (0x10 | (rn(2) << 5)) : rnd() & 0xffff, 2); break;           // CCER CC2E CC2P
    case 2: acc(true, TIM1 + 0x44, rn(4) ? 0x8000 : rnd() & 0xffff, 2); break;                          // BDTR MOE
    case 3: acc(true, TIM1 + 0x38, rn(3) ? rn(40) : rnd() & 0xffff, 2); break;                          // CCR2
    case 4: acc(true, TIM1 + 0x2c, rn(3) ? rn(40) : rnd() & 0xffff, 2); break;                          // ATRLR
    case 5: acc(true, TIM1 + 0x28, rn(3) ? rn(4) : rn(300), 2); break;                                  // PSC
    case 6: acc(true, TIM1 + 0x00, rn(4) ? 1 : rnd() & 0xffff, 2); break;                               // CTLR1 CEN
    case 7: acc(true, TIM1 + 0x14, 1, 2); break;                                                        // UG
    }
}

static int test_gpio(long n, uint32_t seed)
{
    start(seed);
    // directed: PB10 PWM through the remap, then the remap taken away mid-wave
    acc(true, GPIO_BASE[1] + 0x04, 0x44444B34u, 4);   // PB9 output, PB10 AF push-pull
    acc(true, AFIO + 0x04, 1u << 15, 4);
    acc(true, TIM1 + 0x2c, 9, 2); acc(true, TIM1 + 0x38, 3, 2); acc(true, TIM1 + 0x18, 0x6000, 2);
    acc(true, TIM1 + 0x20, 0x10, 2); acc(true, TIM1 + 0x44, 0x8000, 2); acc(true, TIM1 + 0x00, 1, 2);
    for (int k = 0; k < 40; k++) {
        advance_to(now + 1 + rn(15));
        acc(true, AFIO + 0x04, rn(2) ? (1u << 15) : 0, 4);
        if (rn(3) == 0) acc(true, GPIO_BASE[1] + 0x10, 1u << 9, 4);
    }
    printf("gpio directed: %ld differences\n", st.errors());

    for (long i = 0; i < n; i++) {
        gap();
        const int r = rn(100);
        if (r < 8) { set_buttons((uint8_t)rnd()); continue; }
        if (r < 12) {
            // SPI1 NSS: SSOE (CTLR2 bit 2) and SPE (CTLR1 bit 6), on the C side
            if (rn(2)) acc_c(true, 0x40013004u, rn(2) << 2, 2);
            else acc_c(true, 0x40013000u, (rn(2) << 6) | 0x4, 2);
            continue;
        }
        if (r < 30) { tim1_buzzer_op(); continue; }
        if (r < 38) {
            // AFIO
            static const uint32_t ao[] = { 0x04, 0x08, 0x0c, 0x18, 0x00, 0x10, 0x14, 0x1c };
            const uint32_t off = ao[rn(8)];
            if (rn(2)) acc(false, AFIO + off, 0, 4);
            else acc(true, AFIO + off, off == 0x04 && rn(2) ? (rn(8) << 15) | (rnd() & 0xff) : rnd(), 4);
            continue;
        }
        const int p = rn(3);
        static const uint32_t go[] = { 0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x3fc };
        const uint32_t off = go[rn(sizeof go / 4)];
        const int size = rsize();
        const uint32_t a = raddr(GPIO_BASE[p] + off, size);
        if (rn(2)) { acc(false, a, 0, size); continue; }
        uint32_t v;
        switch (off) {
        case 0x00: case 0x04: case 0x1c: v = cfg_word(); break;
        default: v = rn(3) ? (rnd() & rnd()) : rnd(); break;
        }
        acc(true, a, v, size);
    }
    return report("gpio");
}
