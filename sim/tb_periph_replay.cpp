// A game's recorded peripheral traffic (tb_periph_log) played into the RTL
// and into a fresh C machine with the same program loaded and no CPU
// (included by tb_periph.cpp). Accesses to SPI1, DMA1 and the PFIC go to
// the C only (other people's modules; the SPI's NSS level reaches the RTL
// as an input). Every read is checked three ways: RTL against the replayed
// C, and both against the value the C returned when the game ran.
#include "tb_periph_trace.cpp"

static bool c_only_addr(uint32_t a)
{
    if (a >= 0xE000E000u && a < 0xE000F000u) return true;          // PFIC
    const uint32_t w = a & ~0x3ffu;
    return w == 0x40013000u || w == 0x40020000u;                     // SPI1, DMA1
}

static int test_replay(const char *game, const char *trace, const char *sd, long max_recs)
{
    start(1);
    char err[256];
    if (sd) {
        m->sd = sdcard_open(sd, 0, err, sizeof err);
        if (!m->sd) { fprintf(stderr, "sd: %s\n", err); return 1; }
    }
    if (!chg_load_program(m, game, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    chg_reset(m, true);
    chg_schedule(m);
    memcpy(rflash, m->flash, CHG_FLASH_SIZE);
    FILE *f = fopen(trace, "rb");
    if (!f) { perror(trace); return 1; }
    TraceRec r;
    long nrec = 0, mine = 0;
    while (fread(&r, sizeof r, 1, f) == 1) {
        if (max_recs && nrec >= max_recs) break;
        nrec++;
        if (r.t < now) { st.bad_trace++; SHOW("record at %llu is behind board time", (unsigned long long)r.t); }
        if (r.kind <= TR_WRITE) {
            // the instruction's own window: no boundary after the one it began at
            advance_to(r.t - r.dt);
            window(r.t - r.dt, r.t + r.dn);
        }
        advance_to(r.t);
        switch (r.kind) {
        case TR_BUTTONS: set_buttons((uint8_t)r.val); break;
        case TR_RESET: do_reset(false); break;
        case TR_READ:
        case TR_WRITE: {
            const bool wr = r.kind == TR_WRITE;
            uint32_t v;
            if (c_only_addr(r.addr)) v = acc_c(wr, r.addr, r.val, r.size);
            else { v = acc(wr, r.addr, r.val, r.size); mine++; }
            if (!wr && v != r.val && !c_only_addr(r.addr)) {
                st.bad_trace++;
                SHOW("read %08x: replayed C %08x, recorded %08x", r.addr, v, r.val);
            }
            break;
        }
        default: break;
        }
        if ((nrec & 0xfffff) == 0) { printf("  %ld records, t=%llu, %ld differences\n", nrec, (unsigned long long)now, st.errors()); fflush(stdout); }
    }
    fclose(f);
    check_flash();
    printf("replay %s: %ld records, %ld to the RTL\n", game, nrec, mine);
    return report("replay");
}
