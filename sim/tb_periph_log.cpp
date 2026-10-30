// Records the peripheral traffic a game makes in the C model, for
// tb_periph_replay.cpp to play into the RTL.
//
//   tb_periph_log <game.CHG> <frames> <out.trace> [sd folder]
//
// rv32.c is compiled for this program with bus_read_slow / bus_write_slow
// renamed to tap_bus_read_slow / tap_bus_write_slow (a -D on the zig command
// line, emu/src untouched), so every load and store the CPU makes outside
// flash and SRAM comes through here first, at the cycle rv32.c makes it.
// The machine is run one cycle at a time (chg_run(m, m->cycles + 1)), as the
// contract defines matching, so a software reset is seen at the cycle it is
// done. Each such call is one instruction boundary (the top of chg_run's
// loop, where bus_events runs) and one instruction: every access records
// the boundary before it and the one after it, which is when the C lets a
// peripheral's events show. It also counts which offsets of the generic
// io[] backing are used.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <map>
#include <set>
extern "C" {
#include "machine.h"
#include "loader.h"
#include "sdcard.h"
uint32_t tap_bus_read_slow(ChgMachine *m, uint32_t addr, int size, int *wait);
void tap_bus_write_slow(ChgMachine *m, uint32_t addr, uint32_t value, int size, int *wait);
}
#include "tb_periph_trace.cpp"

static FILE *out;
static uint64_t nrec, nio;
// the records of the instruction in hand, written when its next boundary is known
static TraceRec held[8];
static int nheld;

static void put(uint64_t t, uint8_t kind, uint32_t addr, uint32_t val, uint8_t size)
{
    TraceRec r;
    memset(&r, 0, sizeof r);
    r.t = t; r.kind = kind; r.addr = addr; r.val = val; r.size = size;
    if (kind <= TR_WRITE && nheld < 8) { held[nheld++] = r; return; }
    fwrite(&r, sizeof r, 1, out);
    nrec++;
}

static void flush_held(uint64_t s, uint64_t n)
{
    for (int i = 0; i < nheld; i++) {
        held[i].dt = (uint32_t)(held[i].t - s);
        held[i].dn = (uint32_t)(n - held[i].t);
        fwrite(&held[i], sizeof held[i], 1, out);
        nrec++;
    }
    nheld = 0;
}

// is it an access the trace wants: peripherals, the system area, flash stores
static bool wanted(uint32_t addr, bool store)
{
    if (addr >= 0x40000000u) return true;
    if (addr >= 0x1FFF0000u && addr < 0x20000000u) return true;
    if (store && (addr < CHG_FLASH_SIZE || (addr >= CHG_FLASH_ALIAS && addr < CHG_FLASH_ALIAS + CHG_FLASH_SIZE)))
        return true;
    return false;
}

// io[]: an offset in 0x4000_0000..0x4002_FFFF that bus.c has no case for
static bool is_io(uint32_t addr)
{
    const uint32_t word = addr & ~3u;
    if (word < 0x40000000u || word >= 0x40030000u) return false;
    switch ((word - 0x40000000u) & ~0x3ffu) {
    case 0x10800: case 0x10C00: case 0x11000: case 0x10000: case 0x13000: case 0x20000:
    case 0x12C00: case 0x00000: case 0x00400: case 0x22000: case 0x12400: case 0x21000:
    case 0x13800: case 0x04400: case 0x04800: case 0x04C00: case 0x23400:
        return false;
    }
    return true;
}

struct IoUse { uint64_t reads = 0, writes = 0; std::set<int> sizes; std::set<uint32_t> wvals, rvals; };
static std::map<uint32_t, IoUse> io_use;
static std::map<uint32_t, uint64_t> usb_use;

extern "C" uint32_t tap_bus_read_slow(ChgMachine *m, uint32_t addr, int size, int *wait)
{
    const uint32_t v = bus_read_slow(m, addr, size, wait);
    if (wanted(addr, false)) put(m->cycles, TR_READ, addr, v, (uint8_t)size);
    if (is_io(addr)) {
        IoUse &u = io_use[addr];
        u.reads++; u.sizes.insert(size); if (u.rvals.size() < 8) u.rvals.insert(v);
        nio++;
    }
    if ((addr & ~0x3ffu) == 0x40023400u) usb_use[addr]++;
    return v;
}

extern "C" void tap_bus_write_slow(ChgMachine *m, uint32_t addr, uint32_t value, int size, int *wait)
{
    const uint64_t t = m->cycles;
    if (wanted(addr, true)) put(t, TR_WRITE, addr, value, (uint8_t)size);
    if (is_io(addr)) {
        IoUse &u = io_use[addr];
        u.writes++; u.sizes.insert(size); if (u.wvals.size() < 8) u.wvals.insert(value);
        nio++;
    }
    if ((addr & ~0x3ffu) == 0x40023400u) usb_use[addr]++;
    bus_write_slow(m, addr, value, size, wait);
}

static ChgMachine mach;

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "usage: tb_periph_log game.CHG frames out.trace [sd folder]\n"); return 2; }
    const int frames = atoi(argv[2]);
    out = fopen(argv[3], "wb");
    if (!out) { perror(argv[3]); return 1; }
    ChgMachine *m = &mach;
    char err[256];
    chg_init(m);
    if (argc > 4) {
        m->sd = sdcard_open(argv[4], 0, err, sizeof err);
        if (!m->sd) { fprintf(stderr, "sd: %s\n", err); return 1; }
    }
    if (!chg_load_program(m, argv[1], err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    chg_reset(m, true);
    uint8_t btn = 0xff;
    for (int f = 1; f <= frames; f++) {
        // a varied pattern of presses, changing every half second
        static const uint8_t pat[8] = { 0, CHG_BTN_START, CHG_BTN_A, CHG_BTN_LEFT | CHG_BTN_A,
                                        CHG_BTN_RIGHT, CHG_BTN_UP, CHG_BTN_B, CHG_BTN_DOWN };
        const uint8_t b = pat[(f / 30) % 8];
        if (b != btn) {
            chg_set_buttons(m, b);
            put(m->cycles, TR_BUTTONS, 0, b, 0);
            btn = b;
        }
        const uint64_t until = (uint64_t)f * (CHG_HCLK / 60);
        while (m->cycles < until) {
            if (m->reset_request) put(m->cycles, TR_RESET, 0, 0, 0);
            const uint64_t s = m->cycles;
            chg_run(m, s + 1);
            flush_held(s, m->cycles);
        }
    }
    put(m->cycles, TR_END, 0, 0, 0);
    fclose(out);
    if (m->sd) sdcard_close(m->sd);
    printf("%llu records, %llu cycles, %llu instructions, %llu io[] accesses\n",
           (unsigned long long)nrec, (unsigned long long)m->cycles, (unsigned long long)m->instret,
           (unsigned long long)nio);
    for (auto &e : io_use) {
        printf("io %08x  r %llu w %llu  sizes", e.first, (unsigned long long)e.second.reads,
               (unsigned long long)e.second.writes);
        for (int s : e.second.sizes) printf(" %d", s);
        printf("  wvals");
        for (uint32_t v : e.second.wvals) printf(" %x", v);
        printf("  rvals");
        for (uint32_t v : e.second.rvals) printf(" %x", v);
        printf("\n");
    }
    for (auto &e : usb_use) printf("usb %08x %llu\n", e.first, (unsigned long long)e.second);
    return 0;
}
