// chg_loader.sv against chg_load_program (emu/src/loader.c): a .CHG sent byte
// by byte as MiSTer's hps_io sends it, the 64 KB flash compared with the C's.
//   sim/out/loader/loader.exe game.chg [game2.chg ...]
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include "Vchg_loader.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
#include "loader.h"
}

static Vchg_loader *top;
static uint8_t flash[0x10000];
static ChgMachine m;

static void tick()
{
    top->clk = 0; top->eval(); top->clk = 1; top->eval();
    if (top->fl_we) flash[top->fl_addr] = top->fl_data;
}

static bool load_init()
{
    FILE *f = fopen("core/rtl/flash_init.hex", "r");
    if (!f) return false;
    for (int i = 0; i < 0x4000; i++) {
        unsigned w;
        if (fscanf(f, "%x", &w) != 1) { fclose(f); return false; }
        memcpy(flash + 4 * i, &w, 4);
    }
    fclose(f);
    return true;
}

int main(int argc, char **argv)
{
    top = new Vchg_loader;
    if (!load_init()) { fprintf(stderr, "core/rtl/flash_init.hex missing: python -I tools/gen_rom.py\n"); return 1; }
    top->reset = 1; tick(); tick(); top->reset = 0; tick();
    int fails = 0;
    for (int a = 1; a < argc; a++) {
        FILE *f = fopen(argv[a], "rb");
        if (!f) { perror(argv[a]); return 1; }
        std::vector<uint8_t> d;
        int c;
        while ((c = fgetc(f)) != EOF) d.push_back((uint8_t)c);
        fclose(f);

        // the RTL: download on, the bytes with gaps between them, download off
        top->download_in = 1; tick();
        int waits = 0;
        for (size_t i = 0; i < d.size(); i++) {
            while (top->wait_req) { tick(); waits++; }
            top->wr_in = 1; top->addr_in = (uint32_t)i; top->din_in = d[i]; tick();
            top->wr_in = 0; tick(); tick();
        }
        top->download_in = 0;
        for (int i = 0; i < 100; i++) tick();

        char err[256];
        chg_init(&m);
        if (!chg_load_program(&m, argv[a], err, sizeof err)) { fprintf(stderr, "%s: %s\n", argv[a], err); return 1; }
        int diff = 0, first = -1;
        for (int i = 0; i < 0x10000; i++)
            if (flash[i] != m.flash[i]) { if (first < 0) first = i; diff++; }
        printf("%s: %zu bytes, erase %d clocks, busy %d bad %d: %d flash bytes differ",
               argv[a], d.size(), waits, top->busy, top->bad, diff);
        if (first >= 0) printf(" (first at %04x: rtl %02x c %02x)", first, flash[first], m.flash[first]);
        printf("\n");
        if (diff || top->busy || top->bad) fails++;
    }
    return fails ? 1 : 0;
}
