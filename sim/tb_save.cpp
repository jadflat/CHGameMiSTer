// chg_save.sv against the emulator's saves (emu/src/loader.c chg_write_save /
// chg_load_save), with a model of hps_io's virtual disk.
//   sim/out/save/save.exe game.chg outdir
// 1. the game is loaded (its flash writes go through the shadow), then random
//    "game" writes hit random pages, the same in a C ChgMachine (flash and
//    flash_dirty); after a second of quiet board time the autosave writes the
//    file; it must hold exactly the C's dirty pages and their contents, and
//    convert (tools/savconv.py to-emu) to the byte-identical file chg_write_save
//    writes (checked by the caller; this writes both into outdir)
// 2. the game is loaded again and the save restored; the flash must equal the
//    C's after chg_load_program + chg_load_save.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include "Vchg_save.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
#include "loader.h"
}

static Vchg_save *top;
static std::vector<uint8_t> disk(125 * 512, 0);
static uint8_t flash[0x10000];      // the RTL side's flash, written by restore
static uint32_t rng = 7;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static void tick()
{
    top->clk = 0; top->eval(); top->clk = 1; top->eval();
    if (top->fl_we) {
        flash[top->fl_addr] = top->fl_data;
    }
}

// hps_io's side of one block transfer, roughly at its pace
static void serve()
{
    if (top->sd_rd) {
        uint32_t lba = top->sd_lba;
        top->sd_ack = 1; tick();
        for (int i = 0; i < 512; i++) {
            top->sd_buff_addr = i; top->sd_buff_dout = disk[lba * 512 + i]; top->sd_buff_wr = 1; tick();
            top->sd_buff_wr = 0;
            // what the restore writes into the flash goes back into the shadow, as chg_soc wires it
            top->fw_we = top->fl_we; top->fw_addr = top->fl_addr; top->fw_data = top->fl_data; top->fw_size = 1; top->fw_game = 0;
            tick(); top->fw_we = 0; tick();
        }
        top->sd_ack = 0; tick(); tick();
    }
    else if (top->sd_wr) {
        uint32_t lba = top->sd_lba;
        top->sd_ack = 1; top->sd_buff_addr = 0; tick();
        for (int i = 0; i < 512; i++) {
            top->sd_buff_addr = i; tick(); tick(); tick();
            disk[lba * 512 + i] = top->sd_buff_din;
        }
        top->sd_ack = 0; tick(); tick();
    }
}

static void mount()
{
    top->img_size = disk.size(); top->img_readonly = 0; top->img_mounted = 1; tick(); top->img_mounted = 0; tick();
}

static void fw(uint16_t a, uint8_t d, bool game)
{
    top->fw_we = 1; top->fw_addr = a; top->fw_data = d; top->fw_size = 1; top->fw_game = game; tick(); top->fw_we = 0;
}

static void fwn(uint16_t a, uint32_t d, int size)
{
    top->fw_we = 1; top->fw_addr = a; top->fw_data = d; top->fw_size = size; top->fw_game = 1; tick(); top->fw_we = 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: save game.chg outdir\n"); return 2; }
    std::string out = argv[2];
    char err[256];
    static ChgMachine m;
    chg_init(&m);
    if (!chg_load_program(&m, argv[1], err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }

    top = new Vchg_save;
    top->autosave = 1;
    top->reset = 1; tick(); tick(); top->reset = 0; tick();
    mount();

    // 1. load: the loader's writes (the whole flash) pass through, then 'loaded'; the disk is empty
    for (int a = 0; a < 0xF800; a++) fw(a, m.flash[a], false);
    top->loaded = 1; tick(); top->loaded = 0;
    for (int i = 0; i < 2000 && (top->busy || top->sd_rd); i++) { serve(); tick(); }

    // the game writes: whole pages and single bytes, as page programming and half-word writes do
    for (int k = 0; k < 40; k++) {
        int page = 0x30 + rnd() % (0xF7 - 0x30);
        int kind = rnd() % 3;
        if (kind == 0)          // a whole page in words, as fast page programming writes it
            for (int i = 0; i < 256; i += 4) { uint32_t v = rnd(); fwn(page * 256 + i, v, 4); memcpy(m.flash + page * 256 + i, &v, 4); }
        else if (kind == 1) {   // a half-word
            int a = page * 256 + (rnd() % 128) * 2; uint16_t v = rnd(); fwn(a, v, 2); memcpy(m.flash + a, &v, 2);
        }
        else { int a = page * 256 + rnd() % 256; uint8_t v = rnd(); fw(a, v, true); m.flash[a] = v; }
        m.flash_dirty[page] = 1;
        for (int i = 0; i < 1000; i++) { top->ce = 1; tick(); top->ce = 0; }
    }
    // a second of board time: the autosave
    int saved = 0;
    for (long i = 0; i < 48100000 && !saved; i++) {
        top->ce = 1; tick(); top->ce = 0;
        if (top->sd_wr) { while (top->sd_wr || top->sd_ack) { serve(); tick(); } saved = 1; }
    }
    for (int i = 0; i < 100000; i++) { tick(); if (top->sd_wr) serve(); }
    printf("autosave %s, pending %d\n", saved ? "written" : "NOT written", top->pending);

    FILE *f = fopen((out + "/rtl.sav").c_str(), "wb"); fwrite(disk.data(), 1, disk.size(), f); fclose(f);
    chg_write_save(&m, (out + "/c.sav").c_str());

    // 2. load again, restore into a fresh flash, compare with the C's own restore
    static ChgMachine m2;
    chg_init(&m2);
    chg_load_program(&m2, argv[1], err, sizeof err);
    memcpy(flash, m2.flash, sizeof flash);
    for (int a = 0; a < 0xF800; a++) fw(a, m2.flash[a], false);
    top->loaded = 1; tick(); top->loaded = 0;
    for (int i = 0; i < 200000 && (top->busy || top->sd_rd); i++) { if (top->sd_rd) serve(); else tick(); }
    chg_load_save(&m2, (out + "/c.sav").c_str());
    int diff = 0;
    for (int a = 0; a < 0x10000; a++) if (flash[a] != m2.flash[a]) diff++;
    int dd = 0;
    for (int p = 0; p < 248; p++) if (((disk[16 + p / 8] >> (p % 8)) & 1) != m2.flash_dirty[p]) dd++;
    printf("restore: %d flash bytes differ from chg_load_save, %d dirty bits differ, busy %d\n", diff, dd, top->busy);
    return (!saved || diff || dd || top->busy) ? 1 : 0;
}
