// chg_datcard.sv against the emulator's folder card (emu/src/sdcard.c): .DAT
// files are loaded as Main_MiSTer loads a file with a load address in CONF_STR
// ("F2,DAT,Load data file,38000000"): ioctl_download high, ioctl_addr = the
// size, the file copied into the DDR3 model at 0x3800_0000 meanwhile, no
// ioctl_wr, then ioctl_download low. The DDR3 model has random waitrequest and
// read latency and takes bursts; then every sector of the card (or
// a sample, with a step) is read through the virtual-disk side and compared
// with sdcard_read on a folder holding the same files, each under the name
// the file gives itself (a name already on the card is replaced: the folder
// holds the last one loaded). A few writes to the files' sectors are made and
// read back.
//   sim/out/datcard/datcard.exe step op...
//     op: "clear" (Clear data files) or "file.DAT NAME.DAT" (a load)
// e.g. datcard.exe 1 STARLIFT.DAT STARLIFT.DAT DANKTOMB.DAT DANKTOMB.DAT clear RASTAN.DAT RASTAN.DAT
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <deque>
#include <string>
#include <map>
#include <sys/stat.h>
#include "Vchg_datcard.h"
#include "verilated.h"
extern "C" {
#include "sdcard.h"
}
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#endif

static Vchg_datcard *top;
static std::vector<uint64_t> ddr(1u << 24);              // the card's 64 MB, then the staging area's
static uint32_t wleft, waddr;                             // a write burst in progress
static long maxburst;
static uint32_t rng = 12345;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static uint64_t ticks;
struct Rd { uint64_t at; uint32_t a; };
static uint32_t didx(uint32_t a) { return ((a >> 24) & 1) << 23 | (a & ((1u << 23) - 1)); }
static std::deque<Rd> rq;
static long bad;

static void tick()
{
    // DDR3: waitrequest at random; a read's word some clocks later, in order
    top->ddr_busy = (rnd() % 4) == 0;
    const bool acc_rd = top->ddr_rd && !top->ddr_busy, acc_we = top->ddr_we && !top->ddr_busy;
    const uint32_t a = top->ddr_addr;
    top->ddr_dout_ready = 0;
    if (!rq.empty() && rq.front().at <= ticks) {
        top->ddr_dout = ddr[rq.front().a];
        top->ddr_dout_ready = 1;
        rq.pop_front();
    }
    const uint64_t din = top->ddr_din;
    const uint8_t be = top->ddr_be;
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    ticks++;
    if (acc_rd || acc_we) {
        if ((a >> 25) != 0x3 || (a >> 23 & 1)) { printf("DDR3 address %08x outside the card or the staging area\n", a); bad++; }
        if (top->ddr_burstcnt < 1 || top->ddr_burstcnt > 8) { printf("burst %d\n", top->ddr_burstcnt); bad++; }
        if (top->ddr_burstcnt > maxburst) maxburst = top->ddr_burstcnt;
    }
    if (acc_rd)
        for (uint32_t i = 0; i < top->ddr_burstcnt; i++) rq.push_back({ticks + 8 + rnd() % 40 + i, didx(a) + i});
    if (acc_we) {
        if (wleft == 0) { wleft = top->ddr_burstcnt; waddr = a; }
        const uint32_t w = didx(waddr);
        waddr++; wleft--;
        uint64_t v = ddr[w];
        for (int i = 0; i < 8; i++)
            if (be >> i & 1) v = (v & ~(0xffull << (8 * i))) | (din & (0xffull << (8 * i)));
        ddr[w] = v;
    }
}

// the virtual-disk side, as hps_io's client: sd_rd/sd_wr until ack, ack's fall ends it
static uint8_t buf[512];
static bool block(uint32_t lba, bool wr)
{
    std::vector<int> seen(512, 0);
    top->sd_lba = lba;
    if (wr) top->sd_wr = 1; else top->sd_rd = 1;
    long k = 0;
    while (!top->sd_ack) { tick(); if (++k > 100000) { printf("no ack for %u\n", lba); return false; } }
    top->sd_rd = 0; top->sd_wr = 0;
    uint32_t prev = 0;
    while (top->sd_ack) {
        // the buffer: a block RAM a clock behind the address (chg_card's registers make it more)
        if (wr) top->sd_buff_din = buf[prev];
        prev = top->sd_buff_addr;
        if (!wr && top->sd_buff_wr) { buf[top->sd_buff_addr] = top->sd_buff_dout; seen[top->sd_buff_addr]++; }
        tick();
        if (++k > 1000000) { printf("ack stuck for %u\n", lba); return false; }
    }
    if (!wr)
        for (int i = 0; i < 512; i++)
            if (seen[i] != 1) { printf("lba %u: byte %d written %d times\n", lba, i, seen[i]); bad++; return false; }
    return true;
}

static void load(const std::vector<uint8_t> &dat)
{
    // hps_io: ioctl_addr = the size when the download starts, size + 1 when it ends
    top->dl_addr = (uint32_t)dat.size();
    top->dl = 1;
    // Main copies the file to 0x3800_0000 (the bytes past its end stay as they were)
    for (size_t i = 0; i < dat.size(); i++) {
        uint64_t &w = ddr[(1u << 23) + i / 8];
        w = (w & ~(0xffull << (8 * (i % 8)))) | ((uint64_t)dat[i] << (8 * (i % 8)));
        if (i % 4096 == 0) tick();
    }
    for (int i = 0; i < 20; i++) tick();
    top->dl_addr = (uint32_t)dat.size() + 1;
    top->dl = 0;
    long k = 0;
    // the card is ready when loading falls (loaded pulses then)
    bool pulse = false;
    while (top->loading || !pulse) {
        tick();
        pulse |= top->loaded;
        if (!top->loading && !pulse && k > 2000) break;       // nothing to add and nothing held
        if (++k > 200000000) { printf("never loaded\n"); exit(1); }
    }
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: datcard step (clear | file.DAT NAME.DAT)...\n"); return 2; }
    const uint32_t step = (uint32_t)atoi(argv[1]);
    top = new Vchg_datcard;
    for (int i = 0; i < 10; i++) tick();
    // junk in DDR3 first, so that bytes past a file's end must be made zero
    for (auto &w : ddr) w = ((uint64_t)rnd() << 32) | rnd();
    std::map<std::string, std::vector<uint8_t>> files;      // the folder: name -> contents (strcmp order)
    for (int a = 2; a < argc;) {
        if (!strcmp(argv[a], "clear")) {
            top->clr = 1; tick(); top->clr = 0;
            long k = 0;
            while (!top->cleared) { tick(); if (++k > 100000) { printf("never cleared\n"); return 1; } }
            for (int i = 0; i < 20; i++) tick();
            files.clear();
            printf("cleared (valid %d)\n", top->valid);
            if (top->valid) bad++;
            a++;
            continue;
        }
        if (a + 1 >= argc) { fprintf(stderr, "file without a name\n"); return 2; }
        FILE *f = fopen(argv[a], "rb");
        if (!f) { perror(argv[a]); return 2; }
        std::vector<uint8_t> dat;
        for (int c; (c = fgetc(f)) != EOF;) dat.push_back((uint8_t)c);
        fclose(f);
        const uint64_t t0 = ticks;
        load(dat);
        printf("loaded %s (%zu bytes) as %s in %llu clocks; valid %d\n", argv[a], dat.size(), argv[a + 1],
               (unsigned long long)(ticks - t0), top->valid);
        if (files.size() < 8 || files.count(argv[a + 1])) files[argv[a + 1]] = dat;
        else printf("  (a ninth file: the card is as it was)\n");
        a += 2;
    }

    // the reference: a folder with the same files
    std::string dir = std::string(getenv("TEMP") ? getenv("TEMP") : ".") + "/chg_datcard_ref" + std::to_string(_getpid());
#ifdef _WIN32
    _mkdir(dir.c_str());
#endif
    for (auto &kv : files) {
        const std::string path = dir + "/" + kv.first;
        FILE *f = fopen(path.c_str(), "wb");
        fwrite(kv.second.data(), 1, kv.second.size(), f);
        fclose(f);
    }
    char err[256];
    SdCard *ref = sdcard_open(dir.c_str(), 0, err, sizeof err);
    for (auto &kv : files) remove((dir + "/" + kv.first).c_str());
    if (!ref) { printf("%s\n", err); return 2; }
    printf("reference card: %zu files, %llu sectors\n", files.size(), (unsigned long long)sdcard_sectors(ref));
    if (sdcard_sectors(ref) != 262144) { printf("the reference card is not 128 MB\n"); bad++; }

    // where the files are on the card (the folder's order, one run each after the root)
    std::vector<std::pair<uint32_t, uint32_t>> ext;          // first sector, one past the last
    std::vector<size_t> esize;                                // the file's bytes
    uint32_t at = 6144 + (files.size() >= 8 ? 2 : 1);
    for (auto &kv : files) {
        const uint32_t n = (uint32_t)((kv.second.size() + 511) / 512);
        ext.push_back({at, at + n});
        esize.push_back(kv.second.size());
        at += n;
    }
    auto close_to = [&](uint32_t lba) {
        if (lba < 6200) return true;
        for (auto &e : ext)
            if ((lba + 3 >= e.first && lba < e.first + 3) || (lba + 3 >= e.second && lba < e.second + 3)) return true;
        return false;
    };

    uint8_t want[512];
    long n = 0;
    for (uint32_t lba = 0; lba < 262144; lba += close_to(lba) ? 1 : step) {
        if (!block(lba, false)) { bad++; break; }
        sdcard_read(ref, lba, want, 1);
        if (memcmp(buf, want, 512)) {
            int i = 0;
            while (buf[i] == want[i]) i++;
            if (bad++ < 20) printf("lba %u differs from byte %d: rtl %02x c %02x\n", lba, i, buf[i], want[i]);
        }
        n++;
    }
    // writes: a file sector changed and read back (in each file), and a FAT sector (dropped)
    for (size_t fi = 0; fi < ext.size(); fi++) {
        const auto &e = ext[fi];
        for (int t = 0; t < 2; t++) {
            const uint32_t lba = e.first + rnd() % (e.second - e.first);
            for (int i = 0; i < 512; i++) buf[i] = (uint8_t)rnd();
            uint8_t w[512];
            memcpy(w, buf, 512);
            const size_t left = esize[fi] - (size_t)(lba - e.first) * 512;      // the last sector reads back zero past the file
            if (left < 512) memset(w + left, 0, 512 - left);
            block(lba, true);
            memset(buf, 0, 512);
            block(lba, false);
            if (memcmp(buf, w, 512)) { printf("write to lba %u not read back\n", lba); bad++; }
        }
    }
    block(2080, true);
    block(2080, false);
    sdcard_read(ref, 2080, want, 1);
    if (memcmp(buf, want, 512)) { printf("a write to the FAT changed it\n"); bad++; }
    printf("longest burst %ld\n", maxburst);
    printf("%zu files: %ld sectors compared, %ld differences, %llu clocks: %s\n", files.size(), n, bad,
           (unsigned long long)ticks, bad ? "FAIL" : "identical");
    return bad != 0;
}
