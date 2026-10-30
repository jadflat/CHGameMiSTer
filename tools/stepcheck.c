/* Does the emulator give the same machine run in 1/60 s chunks (as a front
   end runs it) as run one cycle at a time (as chgtrace and the RTL
   lockstep run it)? Compares state at every 1/60 s mark. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "machine.h"
#include "loader.h"
#include "sdcard.h"
static ChgMachine a, b;
static void load(ChgMachine *m, const char *g, const char *sd)
{
    char err[256];
    chg_init(m);
    if (sd) m->sd = sdcard_open(sd, 0, err, sizeof err);
    if (!chg_load_program(m, g, err, sizeof err)) { fprintf(stderr, "%s\n", err); exit(1); }
    chg_reset(m, true);
}
int main(int argc, char **argv)
{
    int frames = atoi(argv[2]);
    load(&a, argv[1], argc > 3 ? argv[3] : NULL);
    load(&b, argv[1], argc > 3 ? argv[3] : NULL);
    for (int f = 1; f <= frames; f++) {
        uint8_t btn = (f / 60) % 3 == 1 ? (CHG_BTN_START | CHG_BTN_A) : 0;
        chg_set_buttons(&a, btn); chg_set_buttons(&b, btn);
        uint64_t until = (uint64_t)f * (CHG_HCLK / 60);
        chg_run(&a, until);
        while (b.cycles < until) chg_run(&b, b.cycles + 1);
        if (a.cycles != b.cycles || a.instret != b.instret || a.cpu.pc != b.cpu.pc ||
            memcmp(a.cpu.x, b.cpu.x, sizeof a.cpu.x) || memcmp(a.ram, b.ram, sizeof a.ram)) {
            printf("frame %d: chunked cyc %llu ins %llu pc %08x | stepped cyc %llu ins %llu pc %08x\n", f,
                   (unsigned long long)a.cycles, (unsigned long long)a.instret, a.cpu.pc,
                   (unsigned long long)b.cycles, (unsigned long long)b.instret, b.cpu.pc);
            return 1;
        }
    }
    printf("%d frames identical\n", frames);
    return 0;
}
