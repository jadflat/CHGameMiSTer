/*
 * chgtrace: the reference trace an FPGA CHGame is checked against.
 *
 *   chgtrace game.chg instructions [--sd folder] [--every n] > trace.txt
 *
 * Runs the game through joyrider3774's emulator a step at a time and prints,
 * for every n-th retired instruction (default every one):
 *
 *   <cycle> <instret> <pc> <hash of x1..x31>
 *
 * all in hex, plus a line "F <cycle> <hash of the 128x128 picture>" whenever
 * the panel starts a new frame. A Verilator model of the core prints the same
 * lines; the first line that differs is the first bug. Buttons are not
 * pressed, so the trace is the boot, the bootloader and the title screen.
 *
 * Part of CHGameMiSTer. The emulator: https://github.com/joyrider3774/chgame_emulator (MIT).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "machine.h"
#include "loader.h"
#include "sdcard.h"

static ChgMachine m;
static uint32_t pixels[128 * 128];

/* FNV-1a over 32-bit words: cheap to do the same in a testbench */
static uint32_t fnv(const uint32_t *w, int n)
{
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++)
        for (int b = 0; b < 32; b += 8) { h ^= (w[i] >> b) & 0xff; h *= 16777619u; }
    return h;
}

int main(int argc, char **argv)
{
    const char *game = NULL, *sd = NULL;
    unsigned long long count = 0, every = 1;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--sd") && i + 1 < argc) sd = argv[++i];
        else if (!strcmp(argv[i], "--every") && i + 1 < argc) every = strtoull(argv[++i], 0, 0);
        else if (!game) game = argv[i];
        else count = strtoull(argv[i], 0, 0);
    }
    if (!game || !count || !every) {
        fprintf(stderr, "chgtrace 0.1\nusage: chgtrace game.chg instructions [--sd folder] [--every n]\n");
        return 2;
    }
    char err[512];
    chg_init(&m);
    if (sd && !(m.sd = sdcard_open(sd, 0, err, sizeof err))) { fprintf(stderr, "%s\n", err); return 1; }
    if (!chg_load_program(&m, game, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    chg_reset(&m, true);

    uint32_t frames = m.lcd.frame_starts;
    while (m.instret < count) {
        uint64_t before = m.instret;
        chg_run(&m, m.cycles + 1);      /* at least one cycle: one instruction, or a sleeping cycle */
        if (m.instret / every != before / every)
            printf("%llx %llx %08x %08x\n", (unsigned long long)m.cycles, (unsigned long long)m.instret,
                   m.cpu.pc, fnv(m.cpu.x + 1, 31));
        if (m.lcd.frame_starts != frames) {
            frames = m.lcd.frame_starts;
            st7735_render(&m.lcd, pixels);
            printf("F %llx %08x\n", (unsigned long long)m.cycles, fnv(pixels, 128 * 128));
        }
        if (m.cpu.faulted) { fprintf(stderr, "cpu fault at %08x\n", m.cpu.fault_pc); return 1; }
    }
    if (m.sd) sdcard_close(m.sd);
    return 0;
}
