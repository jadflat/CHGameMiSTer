// chg_tim (TIM1, TIM2, TIM3) against tim_* and buzzer_update in bus.c:
// directed cases, then random register traffic on all three with PB10 on
// TIM1 channel 2 part of the time (included by tb_periph.cpp).

static const uint32_t TIMS[3] = { 0x40012C00u, 0x40000000u, 0x40000400u };

static uint32_t tim_val(uint32_t off)
{
    switch (off) {
    case 0x00: return rn(4) ? (rn(2) | (rn(2) << 2) | (rn(4) == 0 ? rnd() & 0xfffa : 0)) : rnd() & 0xffff;
    case 0x0c: return rn(3) ? rn(2) : rnd() & 0xffff;                  // UIE
    case 0x10: return rn(3) ? 0 : rnd() & 0xffff;
    case 0x14: return rn(3) ? 1 : rnd() & 0xffff;
    case 0x24: return rn(2) ? rn(300) : rnd() & 0xffff;               // CNT, often above ATRLR
    case 0x28: return rn(3) ? rn(5) : rn(4) ? rn(200) : rnd() & 0xffff;
    case 0x2c: return rn(3) ? rn(100) : rn(4) ? rn(2000) : rnd() & 0xffff;
    case 0x18: return (rn(8) << 12) | (rn(3) ? 0 : rnd() & 0x8fff);
    case 0x20: return rn(2) ? (rn(4) << 4) : rnd() & 0xffff;
    case 0x38: return rn(2) ? rn(100) : rnd() & 0xffff;
    case 0x44: return rn(2) ? 0x8000 : rnd() & 0xffff;
    }
    return rnd();
}

static int test_tim(long n, uint32_t seed)
{
    start(seed);
    // directed: UG stopped and running, CNT above ATRLR, URS, an update interrupt at 1 kHz
    for (int t = 0; t < 3; t++) {
        const uint32_t b = TIMS[t];
        acc(true, b + 0x24, 250, 2); acc(true, b + 0x14, 1, 2); acc(false, b + 0x24, 0, 2);
        acc(true, b + 0x2c, 99, 2); acc(false, b + 0x24, 0, 2);       // 250 % 100
        acc(true, b + 0x28, 47, 2); acc(true, b + 0x0c, 1, 2); acc(true, b + 0x00, 1, 2);
        for (int i = 0; i < 30000; i++) {
            step();
            if ((i % 777) == 0) acc(false, b + 0x24, 0, 2);
            if (t == 0 ? top->irq_tim1 : t == 1 ? top->irq_tim2 : top->irq_tim3) acc(true, b + 0x10, 0, 2);
        }
        acc(true, b + 0x00, 5, 2); acc(true, b + 0x14, 1, 2); acc(false, b + 0x10, 0, 2);
        acc(true, b + 0x00, 0, 2);
    }
    // updates inside an instruction (the C has not run tim_events yet)
    for (int t = 0; t < 3; t++) {
        const uint32_t b = TIMS[t];
        acc(true, b + 0x2c, 15 + t, 2); acc(true, b + 0x28, t, 2); acc(true, b + 0x0c, 1, 2); acc(true, b + 0x00, 1, 2);
        for (int k = 0; k < 1500; k++) {
            const uint64_t e = m->tim[t].next_update;
            if (e == UINT64_MAX || e < now + 4) { advance_to(now + 1); acc(true, b + 0x10, 0, 2); continue; }
            const uint64_t at = e + rn(3);
            const uint64_t s = e - 1 - rn(3);
            advance_to(s);
            window(s, at + 1 + rn(3));
            advance_to(at);
            switch (rn(5)) {
            case 0: case 1: acc(false, b + 0x10, 0, 2); break;
            case 2: acc(true, b + 0x10, 0, 2); break;
            case 3: acc(true, b + 0x38, rn(10), 2); break;    // any write drops the update in the C
            case 4: acc(false, b + 0x24, 0, 2); break;
            }
            if (rn(3) == 0) { advance_to(now + 2); acc(false, b + 0x10, 0, 2); acc(true, b + 0x10, 0, 2); }
        }
        acc(true, b + 0x00, 0, 2);
    }
    printf("tim directed: %ld differences\n", st.errors());

    // the buzzer: whole PWM set-ups with random parameters, then changed under
    // way as the games' audio does (CCR2 every sample), with the pin's mode and
    // the remap taken away and given back now and then (where the C freezes it)
    acc(true, GPIO_BASE[1] + 0x04, 0x44444B34u, 4);
    acc(true, AFIO + 0x04, 1u << 15, 4);
    const uint32_t T1 = TIMS[0];
    for (long k = 0; k < n / 10 + 50; k++) {
        acc(true, T1 + 0x00, 0, 2);
        acc(true, T1 + 0x28, rn(3) ? rn(3) : rn(50), 2);
        acc(true, T1 + 0x2c, rn(4) ? 2 + rn(60) : rnd() & 0xffff, 2);
        acc(true, T1 + 0x38, rn(40), 2);
        acc(true, T1 + 0x18, (uint32_t)(rn(4) ? 6 + rn(2) : rn(8)) << 12, 2);
        acc(true, T1 + 0x20, 0x10 | (rn(2) << 5) | (rn(8) == 0 ? 0 : 0), 2);
        acc(true, T1 + 0x44, rn(8) ? 0x8000 : 0, 2);
        if (rn(2)) acc(true, T1 + 0x14, 1, 2);
        acc(true, T1 + 0x00, rn(8) ? 1 : 5, 2);
        for (int j = 0; j < 20; j++) {
            advance_to(now + 1 + rn(rn(4) ? 30 : 400));
            switch (rn(12)) {
            case 0: case 1: case 2: acc(true, T1 + 0x38, rn(60), 2); break;
            case 3: acc(false, T1 + 0x24, 0, 2); break;
            case 4: acc(true, T1 + 0x24, rn(80), 2); break;
            case 5: acc(true, T1 + 0x14, 1, 2); break;
            case 6: acc(true, AFIO + 0x04, rn(3) ? (1u << 15) : 0, 4); break;
            case 7: acc(true, GPIO_BASE[1] + 0x04, rn(3) ? 0x44444B34u : rn(2) ? 0x44444334u : 0x44444F34u, 4); break;
            case 8: acc(true, T1 + 0x00, rn(4) ? 1 : 0, 2); break;
            case 9: acc(true, GPIO_BASE[1] + (rn(2) ? 0x10 : 0x14), 1u << 10, 4); break;
            case 10: acc(true, T1 + 0x2c, 2 + rn(60), 2); break;
            case 11: acc(true, T1 + 0x18, (uint32_t)rn(8) << 12, 2); break;
            }
        }
    }
    printf("tim buzzer: %ld differences, %ld PB10 edges\n", st.errors(), cov_pin[3]);

    // PB10 to TIM1 CH2 through the remap
    acc(true, GPIO_BASE[1] + 0x04, 0x44444B34u, 4);
    acc(true, AFIO + 0x04, 1u << 15, 4);
    for (long i = 0; i < n; i++) {
        gap();
        const int r = rn(100);
        if (r < 2) { acc(true, GPIO_BASE[1] + 0x04, rn(2) ? 0x44444B34u : cfg_word(), 4); continue; }
        if (r < 4) { acc(true, AFIO + 0x04, rn(3) ? (1u << 15) : rn(8) << 15, 4); continue; }
        if (r < 5) { do_reset(false); continue; }
        const int t = rn(4) ? 0 : 1 + rn(2);
        static const uint32_t to[] = { 0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28, 0x2c,
                                       0x30, 0x34, 0x38, 0x3c, 0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x100 };
        const uint32_t off = rn(3) ? to[rn(sizeof to / 4)] : (rn(3) ? 0x24 : 0x10);
        const int size = rn(4) ? 2 : rsize();
        const uint32_t a = raddr(TIMS[t] + off, size);
        if (rn(2)) acc(false, a, 0, size);
        else acc(true, a, tim_val(off), size);
    }
    return report("tim");
}
