// The whole board (chg_soc.sv) against the emulator, in lockstep: the C is
// stepped one cycle at a time (chg_run(m, m->cycles + 1), the reference) and
// the RTL is run to its next instruction boundary each time. At every boundary
// board time, instructions retired and all 32 registers must match; every 1/60 s
// the 128x128 picture too. Everything is the RTL's own: CPU, peripherals,
// SPI/DMA, card, display. The card is served from the same folder as the C's.
//   sim/out/soc/soc.exe game.chg seconds [sd folder]
// Buttons: START at 3 s and 5 s, then A held from 6 s.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include "Vchg_soc.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
#include "loader.h"
#include "sdcard.h"
}

static Vchg_soc *top;
static ChgMachine m;
static SdCard *rcard;               // the RTL's card: a second copy of the same folder
static uint64_t now;                // the RTL's board time: ce counted
static uint8_t blk[512];

// one fabric clock; MiSTer's virtual disk is served alongside, a byte every
// other clock, so that no clock of the board goes unwatched (hold_mode = 1)
static int sd_state, sd_i;          // 0 idle, 1 reading, 2 writing
static uint32_t sd_lba_cur;
static void edge()
{
    top->clk = 0; top->eval();
    if (top->ce) now++;             // the ce this rising edge acts on
    top->clk = 1; top->eval();
    top->sd_buff_wr = 0;
    switch (sd_state) {
    case 0:
        if (top->sd_rd || top->sd_wr) {
            sd_lba_cur = top->sd_lba;
            sd_state = top->sd_rd ? 1 : 2;
            sd_i = 0;
            top->sd_ack = 1;
            top->sd_buff_addr = 0;
            if (sd_state == 1 && (!rcard || !sdcard_read(rcard, sd_lba_cur, blk, 1))) memset(blk, 0xff, 512);
        }
        break;
    case 1:                         // a byte in, every other clock
        if (sd_i & 1) { if (sd_i / 2 + 1 < 512) top->sd_buff_addr = sd_i / 2 + 1; }
        else { top->sd_buff_dout = blk[sd_i / 2]; top->sd_buff_wr = 1; }
        if (++sd_i == 1024) { top->sd_ack = 0; sd_state = 0; }
        break;
    case 2:                         // a byte out: address, then read two clocks later
        if (sd_i % 3 == 2) blk[sd_i / 3] = top->sd_buff_din;
        if (++sd_i % 3 == 0 && sd_i / 3 < 512) top->sd_buff_addr = sd_i / 3;
        if (sd_i == 1536) {
            if (rcard) sdcard_write(rcard, sd_lba_cur, blk, 1);
            top->sd_ack = 0; sd_state = 0;
        }
        break;
    }
}

static bool to_boundary()
{
    for (long i = 0; i < 20000000; i++) {
        edge();
        if (top->dbg_boundary) return true;
    }
    return false;
}

static int compare_picture(int frame)
{
    static uint32_t want[128 * 128];
    st7735_render(&m.lcd, want);
    top->run = 0;
    int bad = 0;
    for (int y = 0; y < 128; y++) {
        top->vy = y; top->vline = 1; top->clk = 0; top->eval(); top->clk = 1; top->eval(); top->vline = 0;
        for (int k = 0; k < 6; k++) { top->clk = 0; top->eval(); top->clk = 1; top->eval(); }
        for (int x = 0; x < 128; x++) {
            top->vx = x;
            for (int k = 0; k < 3; k++) { top->clk = 0; top->eval(); top->clk = 1; top->eval(); }
            if (top->vrgb != want[y * 128 + x] && bad++ < 3)
                printf("frame %d (%d,%d): rtl %06x c %06x\n", frame, x, y, top->vrgb, want[y * 128 + x]);
        }
    }
    top->run = 1;
    return bad;
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: soc game.chg seconds [sd folder]\n"); return 2; }
    char err[256];
    chg_init(&m);
    if (argc > 3) {
        if (!(m.sd = sdcard_open(argv[3], 0, err, sizeof err))) { fprintf(stderr, "%s\n", err); return 1; }
        rcard = sdcard_open(argv[3], 0, err, sizeof err);
    }
    if (!chg_load_program(&m, argv[1], err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    chg_reset(&m, true);

    top = new Vchg_soc;
    top->hold_mode = 1;
    top->run = 0;
    top->sd_mounted = rcard != nullptr;
    top->sd_size = rcard ? sdcard_sectors(rcard) * 512 : 0;
    // the flash as the loader leaves it, written while the board is in reset
    top->reset = 1;
    for (int a = 0; a < 0x10000; a++) {
        top->fl_we = 1; top->fl_addr = a; top->fl_data = m.flash[a]; edge();
    }
    top->fl_we = 0;
    for (int i = 0; i < 4; i++) edge();
    top->reset = 0;
    now = 0;
    top->run = 1;
    if (!to_boundary()) { printf("no first boundary\n"); return 1; }

    const uint64_t end = (uint64_t)(atof(argv[2]) * CHG_HCLK);
    uint64_t steps = 0, next_frame = CHG_HCLK / 60;
    int frame = 0, rc = 0;
    for (;;) {
        // the same buttons at the same board time
        const double s = (double)m.cycles / CHG_HCLK;
        uint8_t b = ((s >= 3 && s < 3.1) || (s >= 5 && s < 5.1)) ? CHG_BTN_START : (s >= 6) ? CHG_BTN_A : 0;
        chg_set_buttons(&m, b);
        top->buttons = b;

        if (now != m.cycles || top->retire_instret != m.instret) {
            printf("DIFFERENCE after %llu steps: C cycle %llu instret %llu pc %08x | RTL cycle %llu instret %llu (last retired pc %08x)\n",
                   (unsigned long long)steps, (unsigned long long)m.cycles, (unsigned long long)m.instret, m.cpu.pc,
                   (unsigned long long)now, (unsigned long long)top->retire_instret, top->retire_pc);
            rc = 1; break;
        }
        for (int r = 1; r < 32; r++)
            if (top->dbg_x[r] != m.cpu.x[r]) {
                printf("DIFFERENCE at cycle %llu: x%d C %08x RTL %08x (C pc %08x, RTL last retired %08x)\n",
                       (unsigned long long)m.cycles, r, m.cpu.x[r], top->dbg_x[r], m.cpu.pc, top->retire_pc);
                rc = 1; break;
            }
        if (rc) break;
        if (m.cycles >= next_frame) {
            frame++;
            next_frame += CHG_HCLK / 60;
            if (compare_picture(frame)) { rc = 1; break; }
            if (frame % 60 == 0) {
                printf("%d s: %llu instructions, pictures match\n", frame / 60, (unsigned long long)m.instret);
                fflush(stdout);
            }
        }
        if (m.cycles >= end) break;

        chg_run(&m, m.cycles + 1);
        if (!to_boundary()) { printf("RTL stuck at cycle %llu\n", (unsigned long long)now); rc = 1; break; }
        steps++;
    }
    printf("%s: %.2f s of board time, %llu instructions, %d pictures compared: %s\n", argv[1],
           (double)m.cycles / CHG_HCLK, (unsigned long long)m.instret, frame, rc ? "DIFFERENT" : "identical");
    return rc;
}
