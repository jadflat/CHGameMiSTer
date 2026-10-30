// The core as the MiSTer runs it (sim/mister_top.sv: sys_top's HPS I/O glue,
// the real hps_io, CHGame.sv's loader, save, card and board) against the
// emulator. The testbench is the ARM, doing what Main_MiSTer does over
// gp_out/gp_in (fpga_spi: the word, the strobe, wait for io_ack, drop it,
// wait again), and the DDR3:
//   - the game goes in through FIO (FILE_INDEX 1, FILE_TX, FILE_TX_DAT bytes,
//     FILE_TX 0), held off by ioctl_wait as sys_top holds io_ack; the save
//     (a blank 64,000-byte .sav) is mounted as virtual disk 0 first
//     (UIO_SET_SDINFO 0x1D, UIO_SET_SDSTAT 0x1C)
//   - the card, one of:
//       image  the .img (chgsdimg) mounted as virtual disk 1 before the game
//       dat    the .DAT loaded through FIO index 2 ("Load data file") before
//              the game
//       late   the game first, with no card; the .DAT loaded at 2 s of board
//              time: the board must start again, with the card
//   - the ARM polls UIO_GET_SDSTAT (0x16) every so often; a request gets its
//     LBA, then after a file delay (ARM_LAT clocks at most, now and then
//     ARM_SLOW) the block goes in with UIO_SECTOR_RD (0x17) or comes out with
//     UIO_SECTOR_WR (0x18), as hps_io.sv expects
//   - DDR3: waitrequest at random, reads answered 8-60 clocks later
// The C is stepped a cycle at a time as in tb_soc; at every boundary board
// time, instructions and registers must match, and the glass every 1/60 s.
// For the image, hold_mode is forced to 1 (the core's own choice for the
// image is 0, the card answering 'not yet', where the game sees the ARM's time and cannot be in
// lockstep): "image0" runs it so, without the C, and checks that the game
// reads its blocks and does not hang.
//       pause  (no C) the real throttle with the .DAT's game: OSD open with "Pause when OSD open" Off (the
//              default) changes nothing; On, the board stands still for the whole time the OSD is open;
//              closed again, no catch-up
//   sim/out/mister/mister.exe mode game.chg seconds dat_or_img [folder]
//     dat_or_img: for the dat modes several .DATs, loaded in order, separated by commas ("clear" among them is
//       the OSD's Clear data files); the folder then holds the files that are left
//     folder: the C's card (a folder with the .DAT); for image mode the .img
// Env: ARM_LAT (clocks, default 200000 = 2 ms), ARM_SLOW (default 3000000),
// ARM_POLL (default 20000).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <deque>
#include <string>
#include <functional>
#include "Vmister_top.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
#include "loader.h"
#include "sdcard.h"
}

static Vmister_top *top;
static ChgMachine m;
static uint64_t now, clocks;
static uint32_t rng = 777;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static long blocks_rd[2], blocks_wr[2], ddr_rds, ddr_wrs;
static int fails;

// ---------------------------------------------------------------------------
// DDR3
// ---------------------------------------------------------------------------
static std::vector<uint64_t> ddr(1u << 24);        // the card's 64 MB, then the staging area's (0x3800_0000)
static uint32_t dwleft, dwaddr;                      // a write burst in progress
static uint32_t didx(uint32_t a) { return ((a >> 24) & 1) << 23 | (a & ((1u << 23) - 1)); }
struct DRd { uint64_t at; uint32_t a; };
static std::deque<DRd> drq;

// ---------------------------------------------------------------------------
// the ARM: a queue of small steps
// ---------------------------------------------------------------------------
enum { A_SS, A_WORD, A_DELAY, A_CALL };
struct Op { int k; uint32_t v; std::function<void(uint16_t)> cb; };
static std::deque<Op> q;
static int a_ph;            // A_WORD: 0 set the word, 1 strobe up, 2 wait ack, 3 strobe down, 4 wait ack low
static long a_wait;
static uint32_t ss;         // gp_out[20:18]: ss2 ss1 ss0

static void push(std::vector<Op> &v, int k, uint32_t val = 0, std::function<void(uint16_t)> cb = nullptr)
{
    v.push_back({k, val, cb});
}
static void run_front(std::vector<Op> &v) { q.insert(q.begin(), v.begin(), v.end()); }
static void run_back(std::vector<Op> &v) { q.insert(q.end(), v.begin(), v.end()); }

static void en_io(std::vector<Op> &v) { push(v, A_SS, 4); }     // EnableIO: ss2
static void en_fpga(std::vector<Op> &v) { push(v, A_SS, 1); }   // EnableFpga: ss0
static void dis(std::vector<Op> &v) { push(v, A_SS, 0); }

// the disks
struct Disk { std::vector<uint8_t> d; bool mounted; bool ro; };
static bool grow;           // savefirst: writes past the end of disk 0 extend it, as a Linux file does
static Disk disk[2];
static long arm_lat = 200000, arm_slow = 3000000, arm_poll = 20000;

static void mount(std::vector<Op> &v, int n)
{
    const uint64_t size = disk[n].d.size();
    en_io(v); push(v, A_WORD, 0x1D);
    for (int i = 0; i < 4; i++) push(v, A_WORD, (uint32_t)(size >> (16 * i)) & 0xffff);
    dis(v);
    en_io(v); push(v, A_WORD, 0x1C); push(v, A_WORD, (1u << n) | (disk[n].ro ? 0x80u : 0u)); dis(v);
    disk[n].mounted = true;
}

static void file_tx(std::vector<Op> &v, int index, const std::vector<uint8_t> &data)
{
    en_fpga(v); push(v, A_WORD, 0x55); push(v, A_WORD, (uint32_t)index); dis(v);
    if (index == 2) {
        // Main with a load address in CONF_STR ("F2,DAT,Load data file,38000000"): user_io_set_download(1, size), the file
        // copied into DDR3 at 0x3800_0000 through shared memory, user_io_set_download(0). No FILE_TX_DAT.
        const uint32_t n = (uint32_t)data.size();
        en_fpga(v); push(v, A_WORD, 0x53); push(v, A_WORD, 0xFF);
        if (n) { push(v, A_WORD, n & 0xffff); push(v, A_WORD, n >> 16); }
        dis(v);
        push(v, A_CALL, 0, [data](uint16_t) {
            for (size_t i = 0; i < data.size(); i++) {
                uint64_t &w = ddr[(1u << 23) + i / 8];
                w = (w & ~(0xffull << (8 * (i % 8)))) | ((uint64_t)data[i] << (8 * (i % 8)));
            }
        });
        push(v, A_DELAY, 200 + rnd() % 2000);
        en_fpga(v); push(v, A_WORD, 0x53); push(v, A_WORD, 0x00); dis(v);
        return;
    }
    en_fpga(v); push(v, A_WORD, 0x53); push(v, A_WORD, 0xFF); dis(v);
    for (size_t at = 0; at < data.size(); at += 4096) {
        en_fpga(v); push(v, A_WORD, 0x54);
        for (size_t i = at; i < data.size() && i < at + 4096; i++) push(v, A_WORD, data[i]);
        dis(v);
        push(v, A_DELAY, 200 + rnd() % 2000);
    }
    en_fpga(v); push(v, A_WORD, 0x53); push(v, A_WORD, 0x00); dis(v);
}

// one poll of the virtual disks (user_io_poll's part), and the transfer it finds
static uint16_t st_c, st_lo;
static uint8_t blk[512];
static uint16_t stw[8];             // the ARM's copy of the OSD status (Main's status[]), sent with command 0x1E
static bool model_status_set;       // posn mode: poll Main's way for a status_set from the core (command 0x29)
static uint16_t stflg_seen;
static void send_status(std::vector<Op> &w)
{
    en_io(w); push(w, A_WORD, 0x1E);
    for (int k = 0; k < 8; k++) push(w, A_WORD, stw[k]);
    dis(w);
}
static void poll()
{
    std::vector<Op> v;
    if (model_status_set) {
        // Main: flag word {A, counter}; when the counter moves, read the 8 words and write them back as the status
        static uint16_t flg, rd[8];
        en_io(v);
        push(v, A_WORD, 0x29, [](uint16_t f) { flg = f; });
        for (int k = 0; k < 8; k++)
            push(v, A_WORD, 0, [k](uint16_t x) {
                rd[k] = x;
                if (k == 7 && (flg & 0xF0) == 0xA0 && (flg & 15) != stflg_seen) {
                    stflg_seen = flg & 15;
                    memcpy(stw, rd, sizeof stw);
                    std::vector<Op> w;
                    dis(w);
                    send_status(w);
                    run_front(w);
                }
            });
        dis(v);
    }
    en_io(v);
    push(v, A_WORD, 0x16, [](uint16_t c) { st_c = c; });
    push(v, A_WORD, 0);
    push(v, A_WORD, 0, [](uint16_t lo) { st_lo = lo; });
    push(v, A_WORD, 0, [](uint16_t hi) {
        const uint16_t c = st_c;
        const uint32_t lba = st_lo | (uint32_t)hi << 16;
        std::vector<Op> w;
        dis(w);
        if ((c & 0x8000) && (c & 3)) {
            const int n = (c >> 2) & 15, op = c & 3, blksz = 128 << ((c >> 6) & 7), blks = ((c >> 9) & 63) + 1;
            if (n > 1 || blksz != 512 || blks != 1) { printf("ARM: odd request %04x\n", c); fails++; }
            const long lat = (rnd() % 16 == 0) ? arm_slow : (long)(rnd() % (uint32_t)(arm_lat + 1));
            if (op & 1) {
                const uint64_t off = (uint64_t)lba * 512;
                for (int i = 0; i < 512; i++) blk[i] = off + i < disk[n].d.size() ? disk[n].d[off + i] : 0;
                blocks_rd[n]++;
                push(w, A_DELAY, (uint32_t)lat);
                en_io(w); push(w, A_WORD, 0x17 | n << 8);
                for (int i = 0; i < 512; i++) push(w, A_WORD, blk[i]);
                dis(w);
            } else {
                en_io(w); push(w, A_WORD, 0x18 | n << 8);
                for (int i = 0; i < 512; i++)
                    push(w, A_WORD, 0, [n, lba, i](uint16_t b) {
                        const uint64_t off = (uint64_t)lba * 512 + i;
                        if (off >= disk[n].d.size() && n == 0 && grow) disk[n].d.resize(off + 1, 0);
                        if (off < disk[n].d.size()) disk[n].d[off] = (uint8_t)b;
                    });
                dis(w);
                blocks_wr[n]++;
                push(w, A_DELAY, (uint32_t)lat);
            }
        }
        run_front(w);
    });
    run_back(v);
}

static void arm_clock()
{
    if (q.empty()) {
        std::vector<Op> v;
        push(v, A_DELAY, (uint32_t)(arm_poll / 2 + rnd() % (uint32_t)(arm_poll + 1)));
        push(v, A_CALL, 0, [](uint16_t) { poll(); });
        run_back(v);
    }
    Op &o = q.front();
    switch (o.k) {
    case A_SS:
        ss = o.v;
        top->gp_ss0 = ss & 1; top->gp_ss1 = (ss >> 1) & 1; top->gp_ss2 = (ss >> 2) & 1;
        a_wait = 3 + rnd() % 8;
        q.front().k = A_DELAY; q.front().v = (uint32_t)a_wait;
        break;
    case A_DELAY:
        if (o.v) o.v--;
        else q.pop_front();
        break;
    case A_CALL: {
        auto cb = o.cb;
        q.pop_front();
        cb(0);
        break;
    }
    case A_WORD:
        switch (a_ph) {
        case 0: top->gp_din = o.v & 0xffff; top->gp_clk = 0; a_wait = 2 + rnd() % 6; a_ph = 1; break;
        case 1: if (a_wait-- <= 0) { top->gp_clk = 1; a_ph = 2; } break;
        case 2: if (top->gp_ack) { a_wait = 2 + rnd() % 6; a_ph = 3; } break;
        case 3: if (a_wait-- <= 0) { top->gp_clk = 0; a_ph = 4; } break;
        case 4:
            if (!top->gp_ack) {
                const uint16_t r = top->gp_dout;
                auto cb = o.cb;
                q.pop_front();
                a_ph = 0;
                if (cb) cb(r);
            }
            break;
        }
        break;
    }
}

// ---------------------------------------------------------------------------
// one fabric clock
// ---------------------------------------------------------------------------
static void edge()
{
    // DDR3, as the core sees it before this edge
    top->DDRAM_BUSY = (rnd() % 5) == 0;
    const bool acc_rd = top->DDRAM_RD && !top->DDRAM_BUSY, acc_we = top->DDRAM_WE && !top->DDRAM_BUSY;
    const uint32_t a = top->DDRAM_ADDR;
    const uint64_t din = top->DDRAM_DIN;
    const uint8_t be = top->DDRAM_BE;
    top->DDRAM_DOUT_READY = 0;
    if (!drq.empty() && drq.front().at <= clocks) {
        top->DDRAM_DOUT = ddr[drq.front().a];
        top->DDRAM_DOUT_READY = 1;
        drq.pop_front();
    }
    top->clk = 0; top->eval();
    if (top->ce) now++;
    top->clk = 1; top->eval();
    clocks++;
    if (acc_rd || acc_we) {
        if ((a >> 25) != 0x3 || (a >> 23 & 1) || top->DDRAM_BURSTCNT < 1 || top->DDRAM_BURSTCNT > 8) {
            printf("DDR3: address %08x burst %d\n", a, top->DDRAM_BURSTCNT); fails++;
        }
        if (acc_rd) {
            for (uint32_t i = 0; i < top->DDRAM_BURSTCNT; i++) drq.push_back({clocks + 8 + rnd() % 52 + i, didx(a) + i});
            ddr_rds += top->DDRAM_BURSTCNT;
        }
        if (acc_we) {
            if (dwleft == 0) { dwleft = top->DDRAM_BURSTCNT; dwaddr = a; }
            const uint32_t w = didx(dwaddr);
            dwaddr++; dwleft--;
            uint64_t x = ddr[w];
            for (int i = 0; i < 8; i++) if (be >> i & 1) x = (x & ~(0xffull << (8 * i))) | (din & (0xffull << (8 * i)));
            ddr[w] = x;
            ddr_wrs++;
        }
    }
    arm_clock();
}

static bool to_boundary()
{
    for (long i = 0; i < 400000000; i++) {
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
        top->vy = y; top->vline = 1; edge(); top->vline = 0;
        for (int k = 0; k < 6; k++) edge();
        for (int x = 0; x < 128; x++) {
            top->vx = x;
            for (int k = 0; k < 3; k++) edge();
            if (top->vrgb != want[y * 128 + x] && bad++ < 3)
                printf("frame %d (%d,%d): rtl %06x c %06x\n", frame, x, y, top->vrgb, want[y * 128 + x]);
        }
    }
    top->run = 1;
    return bad;
}

static std::vector<uint8_t> read_file(const char *p)
{
    std::vector<uint8_t> v;
    FILE *f = fopen(p, "rb");
    if (!f) { perror(p); exit(2); }
    fseek(f, 0, SEEK_END);
    v.resize((size_t)ftell(f));
    fseek(f, 0, SEEK_SET);
    if (fread(v.data(), 1, v.size(), f) != v.size()) { perror(p); exit(2); }
    fclose(f);
    return v;
}

// runs the ARM's script to its end, then the board until it is out of reset
// (the last time) and at its first boundary
static bool script_done, reset_seen;
// put before the last file's end (FILE_TX 0): the board is held off by that download until then
static void script_end(std::vector<Op> &v)
{
    v.insert(v.end() - 4, Op{A_CALL, 0, [](uint16_t) { script_done = true; reset_seen = top->board_reset; }});
}
static bool start_board(long limit)
{
    for (long i = 0; i < limit; i++) {
        edge();
        if (top->board_reset) { now = 0; if (script_done) reset_seen = true; }
        else if (script_done && !reset_seen) { printf("the board was running before the ARM was done\n"); return false; }
        else if (script_done && top->dbg_boundary) { script_done = false; return true; }
    }
    return false;
}

// savefirst: the save fix (chg_save.sv: any writable mount is written, restore only from a whole file), no C.
//   1 game loaded with an empty (size 0) .sav, played, saved with "Save now" (status bit 8): 125 blocks written
//   2 reloaded with an empty .sav again (control: no restore, the saved pages are not in the flash)
//   3 reloaded with the 64,000-byte file: 125 blocks read, the flash's saved pages equal what was saved
//   4 read-only mounts (size 0 and 64,000): nothing written
//   sim/out/mister_sv/mister_sv.exe savefirst game.chg seconds_of_play dats_or_-
static uint32_t flash_w(int w) { top->dbg_fa = w; top->eval(); return top->dbg_fq; }
static void run_board(double secs, bool press)
{
    const uint64_t t0 = now, n = (uint64_t)(secs * CHG_HCLK);
    while (now - t0 < n) {
        const double s = (double)(now - t0) / CHG_HCLK;
        top->buttons_in = press && ((s >= 1 && s < 1.1) || (s >= 2 && s < 2.1)) ? CHG_BTN_START : 0;
        for (int i = 0; i < 2000; i++) edge();
    }
}
static void set_status8(int on)
{
    stw[0] = on ? 0x100 : 0;
    std::vector<Op> w;
    send_status(w);
    run_back(w);
    for (long i = 0; i < 400000 && !q.empty(); i++) edge();
    for (int i = 0; i < 100; i++) edge();
}
static void wait_blocks(long &ctr, long want, long limit_clocks)
{
    for (long i = 0; i < limit_clocks && ctr < want; i++) edge();
    for (long i = 0; i < 3000000; i++) edge();      // let a stray one show up
}
static int savefirst(const std::vector<uint8_t> &chg, const std::vector<std::string> &dats, double secs)
{
    int bad = 0;
    auto check = [&](bool ok, const char *what) { printf("  %-70s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) bad++; };
    grow = true;
    for (auto &d : dats) { std::vector<Op> v; file_tx(v, 2, read_file(d.c_str())); run_back(v); }
    auto load = [&](size_t size, bool ro, const std::vector<uint8_t> *content) {
        disk[0].d.assign(size, 0);
        if (content) disk[0].d = *content;
        disk[0].ro = ro;
        std::vector<Op> v;
        mount(v, 0);
        file_tx(v, 1, chg);
        script_end(v);
        run_back(v);
        script_done = false;
        return start_board(2000000000L);
    };
    long r0 = blocks_rd[0], w0 = blocks_wr[0];
    if (!load(0, false, nullptr)) { printf("no first boundary\n"); return 1; }
    printf("phase 1: empty .sav (size 0), writable; blocks read during load %ld\n", blocks_rd[0] - r0);
    check(blocks_rd[0] == r0, "size 0: no restore (no blocks read), the board starts right away");
    run_board(secs, true);
    printf("  pending after %.1f s of play: %d, blocks written so far %ld\n", secs, (int)top->sv_pending_o, blocks_wr[0] - w0);
    set_status8(1); set_status8(0);
    const long w1 = blocks_wr[0];
    wait_blocks(blocks_wr[0], w0 + 125, 600000000L);
    printf("  Save now: %ld blocks written (%ld before it), file %zu bytes, magic %.4s\n", blocks_wr[0] - w0, w1 - w0,
           disk[0].d.size(), disk[0].d.size() >= 4 ? (const char *)disk[0].d.data() : "");
    check(blocks_wr[0] - w0 == 125, "125 blocks written to disk 0");
    check(disk[0].d.size() == 64000, "the file grew to 64,000 bytes");
    check(disk[0].d.size() >= 4 && !memcmp(disk[0].d.data(), "CHGS", 4), "block 0 starts CHGS");
    const std::vector<uint8_t> sav = disk[0].d;
    std::vector<uint32_t> snap(16384);
    for (int w = 0; w < 16384; w++) snap[w] = flash_w(w);
    auto dirty = [&](int p) { return sav.size() >= 64000 && (sav[16 + p / 8] >> (p % 8) & 1); };
    int npages = 0;
    for (int p = 0; p < 248; p++) npages += dirty(p);
    printf("  pages saved in the bitmap: %d\n", npages);
    check(npages > 0, "the game wrote flash pages (the save is not empty)");
    int fbad = 0;
    for (int p = 0; p < 248; p++) if (dirty(p))
        for (int b = 0; b < 256; b++)
            if (sav[512 + p * 256 + b] != (uint8_t)(snap[p * 64 + b / 4] >> (8 * (b & 3)))) fbad++;
    check(fbad == 0, "the file's pages equal the flash they were saved from");
    auto pagediff = [&]() {
        int d = 0;
        for (int p = 0; p < 248; p++) if (dirty(p))
            for (int b = 0; b < 256; b++)
                if ((uint8_t)(flash_w(p * 64 + b / 4) >> (8 * (b & 3))) != (uint8_t)(snap[p * 64 + b / 4] >> (8 * (b & 3)))) d++;
        return d;
    };
    r0 = blocks_rd[0];
    if (!load(0, false, nullptr)) { printf("no boundary\n"); return 1; }
    int diff = pagediff();
    printf("phase 2: reload, size 0 again: %ld blocks read, %d saved-page bytes differ from the saved state\n", blocks_rd[0] - r0, diff);
    check(blocks_rd[0] == r0, "size 0: no restore, no blocks read");
    check(diff > 0, "control: the saved pages are not in the flash without a restore");
    r0 = blocks_rd[0]; w0 = blocks_wr[0];
    if (!load(64000, false, &sav)) { printf("no boundary\n"); return 1; }
    diff = pagediff();
    printf("phase 3: reload with the 64,000-byte file: %ld blocks read, %ld written, %d saved-page bytes differ\n", blocks_rd[0] - r0, blocks_wr[0] - w0, diff);
    check(blocks_rd[0] - r0 == 125, "restore reads all 125 blocks");
    check(diff == 0, "the flash's saved pages equal what was saved");
    for (int sz = 0; sz < 2; sz++) {
        r0 = blocks_rd[0]; w0 = blocks_wr[0];
        const size_t size = sz ? 64000 : 0;
        const std::vector<uint8_t> c = sz ? sav : std::vector<uint8_t>();
        if (!load(size, true, &c)) { printf("no boundary\n"); return 1; }
        set_status8(1); set_status8(0);
        for (long i = 0; i < 30000000; i++) edge();
        printf("phase 4: read-only, size %zu: %ld blocks read, %ld written (Save now pressed)\n", size, blocks_rd[0] - r0, blocks_wr[0] - w0);
        check(blocks_wr[0] == w0, sz ? "read-only (64,000): nothing written" : "read-only (size 0): nothing written");
        check(disk[0].d == c, "the file is unchanged");
    }
    printf("savefirst: %s\n", bad || fails ? "FAILED" : "PASSED");
    return bad || fails;
}

// posn: per-game picture position (chg_save.sv bytes 48-50, status 89 = Per-game position Off)
//   sim/out/mister_pn/mister_pn.exe posn game.chg seconds_of_play -
static uint32_t st_bits(int lo, int n)
{
    uint32_t r = 0;
    for (int i = 0; i < n; i++) r |= (uint32_t)((top->status_o[(lo + i) / 32] >> ((lo + i) % 32)) & 1) << i;
    return r;
}
static void set_pos(int v, int h, int off)
{
    stw[1] = (uint16_t)((stw[1] & ~0x7FF) | (v & 31) | ((h & 63) << 5));        // status 16-20 = V, 21-26 = H
    stw[5] = (stw[5] & ~0x200) | (off ? 0x200 : 0);
    std::vector<Op> w;
    send_status(w);
    run_back(w);
    for (long i = 0; i < 400000 && !q.empty(); i++) edge();
    for (int i = 0; i < 100; i++) edge();
}
static int posn(const std::vector<uint8_t> &chg, double secs)
{
    int bad = 0;
    auto check = [&](bool ok, const char *what) { printf("  %-70s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) bad++; };
    setvbuf(stdout, nullptr, _IONBF, 0);
    model_status_set = !getenv("NOSTAT");
    grow = true;
    auto mk = [&](int marker, int v, int h) {
        std::vector<uint8_t> s(64000, 0);
        memcpy(s.data(), "CHGS", 4); s[4] = 1; s[8] = 248;
        s[48] = marker ? 0xA5 : 0; s[49] = v; s[50] = h;
        return s;
    };
    auto load = [&](const std::vector<uint8_t> &content) {
        disk[0].d = content; disk[0].ro = false;
        std::vector<Op> v;
        mount(v, 0);
        file_tx(v, 1, chg);
        script_end(v);
        run_back(v);
        script_done = false;
        return start_board(2000000000L);
    };
    auto pos = [&]() { return std::make_pair(st_bits(16, 5), st_bits(21, 6)); };
    // A: option On, V +5 H -3 (38) stored
    set_pos(0, 0, 0);
    if (!load(mk(1, 5, 38))) { printf("no boundary\n"); return 1; }
    run_board(0.3, false);
    printf("A: after load V %u H %u\n", pos().first, pos().second);
    check(pos().first == 5 && pos().second == 38, "On: the stored V 5 and H 38 are in status[20:16], [26:21]");
    set_pos(5, 7, 0);
    long w0 = blocks_wr[0];
    run_board(secs, false);
    printf("  after the change to H 7 and %.1f s: %ld blocks written; block 0 marker %02x V %u H %u\n", secs, blocks_wr[0] - w0,
           disk[0].d[48], disk[0].d[49], disk[0].d[50]);
    check(blocks_wr[0] - w0 >= 125, "an autosave wrote the save");
    check(disk[0].d[48] == 0xA5 && disk[0].d[49] == 5 && disk[0].d[50] == 7, "block 0 holds marker, V 5, H 7");
    check(!memcmp(disk[0].d.data(), "CHGS", 4) && disk[0].d.size() == 64000, "header intact, 64,000 bytes");
    // B: option Off: the load leaves the status alone, the save has no marker
    set_pos(0, 0, 1);
    if (!load(mk(1, 5, 38))) { printf("no boundary\n"); return 1; }
    run_board(0.3, false);
    printf("B: Off, after load V %u H %u\n", pos().first, pos().second);
    check(pos().first == 0 && pos().second == 0, "Off: status unchanged by the load");
    set_pos(0, 7, 1);
    w0 = blocks_wr[0];
    run_board(0.5, false);
    set_status8(1); set_status8(0);
    run_board(0.5, false);
    printf("  Save now: %ld blocks written; block 0 byte 48 %02x\n", blocks_wr[0] - w0, disk[0].d[48]);
    check(blocks_wr[0] - w0 >= 125 && disk[0].d[48] == 0, "Off: a save writes marker 0");
    // C: On, a save without the marker keeps the OSD's values
    set_pos(3, 4, 0);
    if (!load(mk(0, 9, 9))) { printf("no boundary\n"); return 1; }
    run_board(0.3, false);
    printf("C: no marker, after load V %u H %u\n", pos().first, pos().second);
    check(pos().first == 3 && pos().second == 4, "no stored position: the OSD values stay");
    printf("posn: %s\n", bad || fails ? "FAILED" : "PASSED");
    return bad || fails;
}

// mountorder: Main_MiSTer mounts the game's .sav after the .CHG download, not before
//   M1 mount right after the download, M2 mount 50 ms after it, M3 no mount (the board starts after about 0.5 s),
//   M4 Reload save (OSD status 90) restores the mounted file and restarts the board
//   sim/out/mister_mo/mister_mo.exe mountorder game.chg seconds -
static int mountorder(const std::vector<uint8_t> &chg)
{
    int bad = 0;
    setvbuf(stdout, nullptr, _IONBF, 0);
    auto check = [&](bool ok, const char *what) { printf("  %-70s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) bad++; };
    grow = true;
    std::vector<uint8_t> sav(64000, 0);
    memcpy(sav.data(), "CHGS", 4); sav[4] = 1; sav[8] = 248;
    const int pg[2] = {200, 201};
    for (int p : pg) {
        sav[16 + p / 8] |= 1 << (p % 8);
        for (int b = 0; b < 256; b++) sav[512 + p * 256 + b] = (uint8_t)((p * 7 + b) ^ 0x5A);
    }
    auto diff = [&]() {
        int d = 0;
        for (int p : pg) for (int b = 0; b < 256; b++)
            if ((uint8_t)(flash_w(p * 64 + b / 4) >> (8 * (b & 3))) != sav[512 + p * 256 + b]) d++;
        return d;
    };
    auto load = [&](bool mnt, uint32_t delay) {
        disk[0].d = sav; disk[0].ro = false;
        std::vector<Op> v;
        file_tx(v, 1, chg);
        if (delay) push(v, A_DELAY, delay);
        if (mnt) mount(v, 0);
        script_end(v);
        run_back(v);
        script_done = false;
        return start_board(2000000000L);
    };
    long r0 = blocks_rd[0];
    if (!load(true, 0)) { printf("no boundary\n"); return 1; }
    printf("M1: mount right after the download: %ld blocks read, %d restored-page bytes differ at the first boundary\n", blocks_rd[0] - r0, diff());
    check(blocks_rd[0] - r0 == 125 && diff() == 0, "restored before the board ran, flash pages equal the save");
    r0 = blocks_rd[0];
    if (!load(true, 4800000)) { printf("no boundary\n"); return 1; }
    printf("M2: mount 50 ms after the download: %ld blocks read, %d bytes differ\n", blocks_rd[0] - r0, diff());
    check(blocks_rd[0] - r0 == 125 && diff() == 0, "restored before the board ran, flash pages equal the save");
    r0 = blocks_rd[0];
    if (!load(false, 0)) { printf("no boundary\n"); return 1; }
    printf("M3: no mount: %ld blocks read, %d bytes differ, board started\n", blocks_rd[0] - r0, diff());
    check(blocks_rd[0] == r0 && diff() > 0, "no save: the board starts after the timeout, nothing restored");
    // M4: the game is running without the save; the file is mounted (mid-game: no restore), then Reload save
    r0 = blocks_rd[0];
    { std::vector<Op> v; mount(v, 0); run_back(v); }
    run_board(0.2, false);
    check(blocks_rd[0] == r0 && diff() > 0, "a mount while the game runs does not restore");
    stw[5] |= 0x400; { std::vector<Op> v; send_status(v); run_back(v); }
    for (long i = 0; i < 400000 && !q.empty(); i++) edge();
    stw[5] &= ~0x400; { std::vector<Op> v; send_status(v); run_back(v); }
    for (long i = 0; i < 400000 && !q.empty(); i++) edge();
    run_board(0.3, false);
    printf("M4: Reload save: %ld blocks read, %d bytes differ\n", blocks_rd[0] - r0, diff());
    check(blocks_rd[0] - r0 == 125 && diff() == 0, "Reload save restored the file, game restarted and running");
    printf("mountorder: %s\n", bad || fails ? "FAILED" : "PASSED");
    return bad || fails;
}

int main(int argc, char **argv)
{
    if (argc < 5) { fprintf(stderr, "usage: mister mode game.chg seconds dat_or_img [folder]\n"); return 2; }
    const std::string mode = argv[1];
    if (getenv("ARM_LAT")) arm_lat = atol(getenv("ARM_LAT"));
    if (getenv("ARM_SLOW")) arm_slow = atol(getenv("ARM_SLOW"));
    if (getenv("ARM_POLL")) arm_poll = atol(getenv("ARM_POLL"));
    const std::vector<uint8_t> chg = read_file(argv[2]);
    const double seconds = atof(argv[3]);
    char err[256];
    SdCard *ccard = nullptr;
    if (argc > 5 && !(ccard = sdcard_open(argv[5], 0, err, sizeof err))) { printf("%s\n", err); return 2; }

    top = new Vmister_top;
    top->run = 1;
    top->force_hold = mode == "image";
    top->RESET = 1;
    for (auto &w : ddr) w = ((uint64_t)rnd() << 32) | rnd();      // DDR3 holds junk
    for (int i = 0; i < 50; i++) edge();
    top->RESET = 0;
    for (int i = 0; i < 50; i++) edge();

    if (mode == "posn") return posn(chg, seconds);
    if (mode == "mountorder") return mountorder(chg);
    if (mode == "savefirst") {
        // argv[4]: the .DATs, comma separated, or "-"
        std::vector<std::string> dats;
        const std::string list = argv[4];
        for (size_t at = 0; list != "-" && at <= list.size();) {
            size_t c = list.find(',', at);
            if (c == std::string::npos) c = list.size();
            dats.push_back(list.substr(at, c - at));
            at = c + 1;
        }
        return savefirst(chg, dats, seconds);
    }
    // the ARM's script up to the game's start
    std::vector<Op> v;
    disk[0].d.assign(64000, 0);
    if (mode == "image" || mode == "image0") {
        disk[1].d = read_file(argv[4]);
        mount(v, 1);
    }
    std::vector<uint8_t> dat;
    if (mode == "late") dat = read_file(argv[4]);
    if (mode == "dat" || mode == "ffdat" || mode == "pause") {
        // the .DATs: each goes in as FILE_TX index 2; "clear" sets and drops status bit 65 (hps_io's 128-bit status)
        std::string list = argv[4];
        size_t at = 0;
        while (at <= list.size()) {
            size_t c = list.find(',', at);
            if (c == std::string::npos) c = list.size();
            const std::string tok = list.substr(at, c - at);
            at = c + 1;
            if (tok == "clear") {
                for (int bit = 1; bit >= 0; bit--) {
                    en_io(v); push(v, A_WORD, 0x1E);
                    for (int w = 1; w <= 8; w++) push(v, A_WORD, w == 5 && bit ? 2 : 0);
                    dis(v);
                    push(v, A_DELAY, 5000);
                }
            } else file_tx(v, 2, read_file(tok.c_str()));
        }
    }
    mount(v, 0);
    file_tx(v, 1, chg);
    script_end(v);
    run_back(v);

    chg_init(&m);
    if (!chg_load_program(&m, argv[2], err, sizeof err)) { printf("%s\n", err); return 1; }
    chg_reset(&m, true);
    m.sd = mode == "late" ? nullptr : ccard;

    if (!start_board(2000000000L)) { printf("no first boundary\n"); return 1; }
    printf("board started after %llu clocks (card: %s)\n", (unsigned long long)clocks,
           top->card_sel_dat ? "the .DAT" : disk[1].mounted ? "the image" : "none");

    const uint64_t end = (uint64_t)(seconds * CHG_HCLK);
    uint64_t steps = 0, next_frame = CHG_HCLK / 60, t0 = 0;    // t0: board time of the last start
    int frame = 0, rc = 0;
    bool late_done = false;
    uint64_t cbase = 0, ibase = 0;
    if (mode == "ffdat") {
        // the real throttle (chg_throttle.sv) and speed counter, with the .DAT's game, no C: 1.1 s of wall
        // time each of normal, fast forward (the pad's FF), normal. Board cycles done per phase, the
        // speed display's number at its end, and board cycles in the first 0.1 s after fast forward
        // ends (owed was cleared: no catching up, it must not be more than normal)
        top->thr_sel = 1; top->ff = 0;
        top->rate = getenv("CHG_RATE") ? atoi(getenv("CHG_RATE")) : 0;     // the OSD's Board clock setting
        const char *names[3] = { "normal", "fast forward", "normal again" };
        for (int ph = 0; ph < 3; ph++) {
            top->ff = ph == 1;
            const uint64_t c0 = clocks, n0 = now;
            uint64_t n01 = 0;
            const uint64_t span = (uint64_t)(1.1 * 96e6);
            for (uint64_t i = 0; i < span; i++) {
                edge();
                if (i == 9600000) n01 = now - n0;
            }
            const unsigned b = top->speed_bcd;
            printf("%-12s: %.3f s of wall time, %llu board cycles = %.1f%% of real time; display %x%%; first 0.1 s: %llu cycles = %.1f%%\n",
                   names[ph], (double)(clocks - c0) / 96e6, (unsigned long long)(now - n0), (double)(now - n0) / span * 200.0,
                   b, (unsigned long long)n01, (double)n01 / 9600000 * 200.0);
            fflush(stdout);
        }
        return fails;
    }
    if (mode == "pause") {
        // the real throttle, no C. The OSD flag comes from outside (OSD_STATUS); the option is status bit 64.
        top->thr_sel = 1; top->ff = 0; top->OSD_STATUS = 0;
        double base_rate = 0;
        auto span = [&](const char *name, double secs, bool want_still) {
            const uint64_t c0 = clocks, n0 = now, n = (uint64_t)(secs * 96e6);
            uint64_t n_late = 0, n20 = 0;
            for (uint64_t i = 0; i < n; i++) {
                if (i == 1000) n_late = now;
                edge();
                if (i == 1920000) n20 = now - n0;
            }
            const uint64_t ran = now - n_late;
            printf("%-34s %.2f s: %llu board cycles (%.1f%% of real time), %llu in the last %.2f s, first 20 ms: %.1f%%, paused=%d\n",
                   name, (double)(clocks - c0) / 96e6, (unsigned long long)(now - n0), (double)(now - n0) / n * 200.0,
                   (unsigned long long)ran, (double)(n - 1000) / 96e6, (double)n20 / 960000 * 100.0, (int)top->paused);
            if (want_still && ran != 0) { printf("  the board ran while paused\n"); fails++; }
            const double rate = (double)(now - n0) / n * 200.0;
            if (base_rate == 0) base_rate = rate;
            if (!want_still && (rate < base_rate * 0.85 || rate > base_rate * 1.15)) { printf("  the board did not run as at first (%.1f%%)\n", base_rate); fails++; }
            if (!want_still && n20 > 960000 * 1.05) { printf("  caught up\n"); fails++; }
        };
        auto set_opt = [&](int on) {
            std::vector<Op> w;
            en_io(w); push(w, A_WORD, 0x1E);
            for (int k = 1; k <= 8; k++) push(w, A_WORD, k == 5 ? on : 0);
            dis(w);
            run_back(w);
            for (long i = 0; i < 400000 && !q.empty(); i++) edge();
            for (int i = 0; i < 100; i++) edge();
        };
        for (uint64_t i = 0; i < 14400000; i++) edge();      // 0.15 s: the throttle's catch-up after the reset is over
        span("normal", 0.3, false);
        top->OSD_STATUS = 1;
        span("OSD open, option Off (default)", 0.3, false);
        set_opt(1);
        span("OSD open, option On", 0.3, true);
        top->OSD_STATUS = 0;
        span("OSD closed again", 0.3, false);
        top->OSD_STATUS = 1;
        top->ff = 1;
        span("OSD open, On, fast forward", 0.1, true);
        top->OSD_STATUS = 0; top->ff = 0;
        span("OSD closed, no fast forward", 0.3, false);
        set_opt(0);
        printf("pause: %s\n", fails ? "FAILED" : "ok");
        return fails;
    }
    const bool lockstep = mode != "image0";
    if (!lockstep) {
        // the image path: no C; the game must keep going and read its blocks
        top->buttons_in = 0;
        while (now < end) {
            const double s = (double)now / CHG_HCLK;
            top->buttons_in = ((s >= 3 && s < 3.1) || (s >= 5 && s < 5.1)) ? CHG_BTN_START : (s >= 6) ? CHG_BTN_A : 0;
            if (!to_boundary()) { printf("RTL stuck at %llu\n", (unsigned long long)now); rc = 1; break; }
        }
        printf("image0: %.2f s of board time, %llu instructions, %ld blocks read from the image, %ld+%ld written: %s\n",
               (double)now / CHG_HCLK, (unsigned long long)top->retire_instret, blocks_rd[1], blocks_wr[0], blocks_wr[1],
               rc || fails ? "FAILED" : "ran");
        return rc || fails;
    }
    for (;;) {
        const uint64_t mc = m.cycles - cbase, mi = m.instret - ibase;   // since the last power-on (chg_reset keeps the counts going)
        const double s = (double)(mc) / CHG_HCLK;
        uint8_t b = ((s >= 3 && s < 3.1) || (s >= 5 && s < 5.1)) ? CHG_BTN_START : (s >= 6) ? CHG_BTN_A : 0;
        chg_set_buttons(&m, b);
        top->buttons_in = b;

        // (retire_instret is the last retired one's: stale at the first boundary after a reset)
        if (now != mc || (mi && top->retire_instret != mi)) {
            printf("DIFFERENCE after %llu steps: C cycle %llu instret %llu pc %08x | RTL cycle %llu instret %llu\n",
                   (unsigned long long)steps, (unsigned long long)mc, (unsigned long long)mi, m.cpu.pc,
                   (unsigned long long)now, (unsigned long long)top->retire_instret);
            rc = 1; break;
        }
        for (int r = 1; r < 32; r++)
            if (top->dbg_x[r] != m.cpu.x[r]) {
                printf("DIFFERENCE at cycle %llu: x%d C %08x RTL %08x (C pc %08x)\n",
                       (unsigned long long)mc, r, m.cpu.x[r], top->dbg_x[r], m.cpu.pc);
                rc = 1; break;
            }
        if (rc) break;
        if (mc >= next_frame) {
            frame++;
            next_frame += CHG_HCLK / 60;
            if (compare_picture(frame)) { rc = 1; break; }
            if (frame % 60 == 0) {
                printf("%d s: %llu instructions, pictures match, blocks read: image %ld, DDR3 words %ld\n", frame / 60,
                       (unsigned long long)mi, blocks_rd[1], ddr_rds);
                fflush(stdout);
            }
        }
        if (mc >= end) break;
        if (mode == "late" && !late_done && mc >= 2 * CHG_HCLK) {
            // the .DAT loaded while the game runs: the board goes off, then starts again with the card
            late_done = true;
            std::vector<Op> w;
            const uint64_t t_off = now;
            file_tx(w, 2, dat);
            script_end(w);
            run_back(w);
            if (!start_board(2000000000L)) { printf("no boundary after the .DAT\n"); return 1; }
            // the C: power on again with the card; the glass keeps its picture (the RTL's st7735 keeps its memory)
            static uint32_t g[ST7735_ROWS][ST7735_COLS];
            memcpy(g, m.lcd.gram, sizeof g);
            chg_reset(&m, true);
            memcpy(m.lcd.gram, g, sizeof g);
            m.sd = ccard;
            cbase = m.cycles; ibase = m.instret;
            printf("the .DAT went in at %.3f s of board time; the board started again\n", (double)t_off / CHG_HCLK);
            t0 = t_off;
            next_frame = CHG_HCLK / 60;
            continue;
        }
        chg_run(&m, m.cycles + 1);
        if (!to_boundary()) { printf("RTL stuck at cycle %llu\n", (unsigned long long)now); rc = 1; break; }
        steps++;
    }
    (void)t0;
    printf("%s %s: %.2f s of board time, %llu instructions, %d pictures compared, blocks read: image %ld save %ld, "
           "written: save %ld image %ld, DDR3 reads %ld writes %ld, %llu clocks: %s\n",
           mode.c_str(), argv[2], (double)m.cycles / CHG_HCLK, (unsigned long long)m.instret, frame, blocks_rd[1],
           blocks_rd[0], blocks_wr[0], blocks_wr[1], ddr_rds, ddr_wrs, (unsigned long long)clocks,
           rc || fails ? "DIFFERENT" : "identical");
    return rc || fails;
}
