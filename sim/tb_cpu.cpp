// chg_cpu.sv in lockstep with emu/src/rv32.c.
//
//   sim/out/cpu/cpu.exe game.chg [seconds] [--sd folder] [--press btn@sec[:dur]]...
//                       [--noref] [--full n] [--every s]
//   sim/out/cpu/cpu.exe --random seed [seconds]       (tb_cpu_random.cpp)
//
// A .CHG goes into a C ChgMachine (chg_init, chg_load_program, chg_reset(m, true):
// the real bootloader at 0, the game at 0x3000). Its flash and SRAM are copied into
// the testbench's memories, which play chg_soc's block RAMs for the RTL (port A,
// one clock read latency, read-first). Then, step by step:
//
//   the C   chg_run(m, m->cycles + 1): one instruction (with an interrupt entry or
//           a reset before it), or one cycle of WFI sleep. That is the reference
//           call (CONTRACT.md 1), with one addition: when an event is due, the
//           testbench calls bus_events + chg_schedule itself first, which is what
//           chg_run does first anyway, so that it can see what the events did (the
//           irq lines, DMA writes to SRAM) before the RTL needs it. A second
//           machine run with plain chg_run checks that this changes nothing
//           (--noref to skip it).
//   the RTL is clocked from one boundary pulse (where chg_run runs bus_events and looks at
//           interrupts) to the next, with ce = !hold on every clock.
//
// After every step: cycles, instret, pc, x1..x31, mstatus, mepc, mcause, the trap
// depth, wfi, and the timing state (fetch_word, fetch_time - cycles, div_ready -
// cycles, div_rd, frac8, pair_open, recent_data) must be equal. The SRAM bytes the
// RTL wrote, and the bytes a store wrote in the C, must be equal; the whole SRAM
// and flash every --full steps and at the end. The first difference stops the run,
// printing both sides and the last instructions.
//
// The oracle for what the RTL's peripheral bus reaches (SysTick, GPIO, SPI, ... are
// not in the RTL here):
//   reads    answered with what the C machine's instruction loaded: x[rd] after
//            the C step (x[32] when rd is x0), shifted by the byte the access is
//            at within the instruction's access (for a misaligned access split
//            into bytes) and masked to the access size. AMO and LR alike.
//   writes   checked against what the instruction stores (x[rs2] before the step,
//            shifted likewise), and acked; the C has done the write itself.
//   bus_wait the board cycles the C's peripheral adds to m->cycles for a write:
//            ADC SWSTART 50, a flash halfword 1,440, page program 48,000, erase
//            96,000, worked out from the flash controller as it was before the step.
//   sw_pend  with the ack of a SysTick CTLR write with SWIE set.
//   irq_line m->pfic.line after the step's events (the levels at the boundary).
//   DMA      SRAM bytes that changed in the C without the CPU (SPI RX DMA, run by
//            bus_events or by an SPI/DMA register access) are copied into the RTL's
//            SRAM, and flash the flash controller wrote into its flash, with a
//            mem_b_wr pulse so the RTL fetches again.
// The PFIC is the RTL's own (chg_pfic), checked through all of the above.
//

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <vector>
#include "Vchg_cpu.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
#include "loader.h"
#include "sdcard.h"
}

static Vchg_cpu *top;
static ChgMachine *m, *ref;
static uint32_t flash[0x4000], ram[0x1400];
static uint64_t clocks;
static bool prof_on;
static uint64_t prof[67];
static bool busy_test;
static uint32_t busy_rng = 12345;

// what the RTL did on port A and the bus during the step
static int nwr;
static uint32_t wr_idx[64], wr_mask[64];
static bool bus_spi_dma, bus_flash, bus_bad;
static bool fault_seen;
static int thr_noted;
static uint64_t mie_steps, ready_steps, pfic_stores, pend_stores, alias_steps, flash_writes, waits;
static uint64_t op_count[128], ram_steps, excs, bus_reads, bus_writes, hpe_ints;
static int ret_op;                  // the retired instruction (captured at its retire pulse)
static uint32_t ret_ea;
static char bus_msg[256];

// the C state before the step
static uint32_t x_before[33];
static ChgFlashCtl fc_before;

// the bus answer for the next edge
static bool ack_next;
static uint32_t rdata_next, wait_next;
static bool swp_next;

static struct { uint8_t bit; double at, dur; } press[32];
static int npress;

static const char *op_name(int op) { return cpu_op_name(op); }

static bool is_store(int op) { return op == 16 || op == 17 || op == 18 || op == 47 || (op >= 48 && op <= 56); }
static bool is_load(int op) { return (op >= 11 && op <= 15) || op == 46 || (op >= 48 && op <= 56); }

static uint32_t mask_of(int size) { return size == 1 ? 0xffu : size == 2 ? 0xffffu : 0xffffffffu; }

// the board cycles a write costs in the C on top of the instruction (bus.c)
static uint32_t write_wait(uint32_t addr, uint32_t value, int size)
{
    if (addr < CHG_FLASH_SIZE || (addr >= CHG_FLASH_ALIAS && addr < CHG_FLASH_ALIAS + CHG_FLASH_SIZE)) {
        const uint32_t a = addr & 0x00ffffffu;              // flash_mem_write
        if (fc_before.locked || a >= CHG_FLASH_USER) return 0;
        if ((fc_before.ctlr & 0x00010000u) && !fc_before.fast_locked) return 0;
        if (fc_before.ctlr & 1u) return 48 * 30;
        return 0;
    }
    const uint32_t word = addr & ~3u;
    const uint32_t v = value << ((addr & 3) * 8);
    if (word == 0x40022010u) {                              // flash_ctl_write CTLR
        const bool locked = fc_before.locked || (v & 0x80u);
        if (!locked && (v & 0x40u)) return (v & (0x00020000u | 2u | 4u)) ? CHG_HCLK / 1000 * 2 : CHG_HCLK / 1000;
        return 0;
    }
    if (word == 0x40012408u && (v & 0x00400000u)) return 50;   // adc_write r == 2, SWSTART
    (void)size;
    return 0;
}

static void settle() { top->clk = 0; top->eval(); }

static void edge()
{
    // --busy: SPI/DMA's busy at random, with ce held low meanwhile (CONTRACT.md 2); the C has
    // no such thing, so nothing may change
    if (busy_test) { busy_rng ^= busy_rng << 13; busy_rng ^= busy_rng >> 17; busy_rng ^= busy_rng << 5; }
    top->busy = busy_test && (busy_rng % 3 == 0 || (top->boundary && busy_rng % 2 == 0));
    // the inputs set since the last edge reach the comb outputs (port A follows the boundary decision)
    settle();
    // ce first: port A's write follows the decision, which may wait for ce (chg_soc: ce = !hold && !busy)
    top->ce = !top->hold && !top->busy;
    settle();
    const uint32_t fa = top->fl_addr & 0x3fff, ra = top->ram_addr, we = top->ram_we, wd = top->ram_wdata;
    const bool acked = ack_next;
    top->bus_ack = ack_next;
    top->bus_rdata = rdata_next;
    top->stall = wait_next;
    top->sw_pend = swp_next;
    top->ce = !top->hold && !top->busy;
    if (getenv("TBTRACE") && clocks < 120)
        printf("clk %llu state %u pc %08x hold %u busy %u ce %u bnd %u ret %u cyc %llu ins %llu" "%c", (unsigned long long)clocks,
               top->dbg_state, top->dbg_pc, top->hold, top->busy, top->ce, top->boundary, top->retire,
               (unsigned long long)top->cycles, (unsigned long long)top->instret, 10);
    // --prof: fabric clocks by what the RTL was doing (state 0 split by ce / boundary / waiting)
    if (prof_on) prof[top->dbg_state == 0 ? (top->ce ? 64 : top->boundary ? 65 : 66) : top->dbg_state & 63]++;
    top->clk = 1;
    top->eval();
    clocks++;
    top->fl_rdata = flash[fa];
    uint32_t old = ra < 0x1400 ? ram[ra] : 0;
    if (we && ra < 0x1400) {
        uint32_t mk = 0;
        for (int b = 0; b < 4; b++) if (we >> b & 1) mk |= 0xffu << (8 * b);
        ram[ra] = (old & ~mk) | (wd & mk);
        if (nwr < 64) { wr_idx[nwr] = ra; wr_mask[nwr] = mk; nwr++; }
    }
    top->ram_rdata = old;
    if (top->retire) { ret_op = top->dbg_op; ret_ea = top->dbg_ea; }
    if (acked) { ack_next = false; swp_next = false; rdata_next = 0; wait_next = 0; }
    if (top->bus_req) {
        const uint32_t addr = top->bus_addr, size = top->bus_size, op = top->dbg_op;
        const uint32_t ea = top->dbg_ea;
        const uint32_t sh = (addr - ea) * 8;
        ack_next = true;
        if (top->bus_we) bus_writes++; else bus_reads++;
        if ((addr >= 0x40013000u && addr < 0x40013400u) || (addr >= 0x40020000u && addr < 0x40020400u))
            bus_spi_dma = true;
        if (!top->bus_we) {
            // a fetch (or vector load) from where nothing is mapped: bus_read_slow gives 0
            const bool unmapped = (addr >= 0x40030000u && addr < 0xE000E000u) || addr >= 0xE000F100u;
            if (!is_load((int)op) && unmapped) { rdata_next = 0; }
            else if (!is_load((int)op) || sh >= 32) {
                bus_bad = true;
                snprintf(bus_msg, sizeof bus_msg, "bus read %08x size %u not from a load (op %s, ea %08x)",
                         addr, size, op_name((int)op), ea);
            }
            const unsigned rd = top->dbg_rd;
            if (is_load((int)op) || !unmapped) rdata_next = (sh < 32 ? m->cpu.x[rd] >> sh : 0) & mask_of((int)size);
        } else {
            const uint32_t v = top->bus_wdata;
            if (op == 16 || op == 17 || op == 18) {
                const uint32_t want = (sh < 32 ? x_before[top->dbg_rs2] >> sh : 0) & mask_of((int)size);
                if ((v & mask_of((int)size)) != want) {
                    bus_bad = true;
                    snprintf(bus_msg, sizeof bus_msg, "bus write %08x size %u value %08x, the C stored %08x",
                             addr, size, v, want);
                }
            }
            wait_next = write_wait(addr, v, (int)size);
            if (wait_next) waits++;
            if (addr < CHG_FLASH_SIZE || (addr >= CHG_FLASH_ALIAS && addr < CHG_FLASH_ALIAS + CHG_FLASH_SIZE)) flash_writes++;
            if ((addr & ~3u) == 0xE000F000u && ((v << ((addr & 3) * 8)) & 0x80000000u)) swp_next = true;
            if (addr < CHG_FLASH_SIZE || (addr >= CHG_FLASH_ALIAS && addr < CHG_FLASH_ALIAS + CHG_FLASH_SIZE) ||
                (addr >= 0x40022000u && addr < 0x40022400u) || (addr >= 0x40020000u && addr < 0x40020400u))
                bus_flash = true;
        }
    }
}

// SRAM / flash that changed in the C without the CPU: into the RTL's memories
static bool sync_ram()
{
    if (!memcmp(ram, m->ram, sizeof ram)) return false;
    memcpy(ram, m->ram, sizeof ram);
    return true;
}
static bool sync_flash()
{
    if (!memcmp(flash, m->flash, sizeof flash)) return false;
    memcpy(flash, m->flash, sizeof flash);
    return true;
}

struct Hist { uint64_t cyc, ins; uint32_t pc; int op; };
static Hist hist[16];
static int nhist;

static void dump_history()
{
    printf("last steps (C side, oldest first):\n");
    for (int k = 0; k < 16; k++) {
        const Hist &h = hist[(nhist + k) % 16];
        if (!h.cyc && !h.ins) continue;
        printf("  cycle %llu instret %llu pc %08x %s\n", (unsigned long long)h.cyc,
               (unsigned long long)h.ins, h.pc, op_name(h.op));
    }
}

static uint32_t xr(int i) { return top->dbg_x[i]; }

static int compare(const char *when)
{
    char buf[4096];
    int n = 0;
    auto chk = [&](const char *what, long long c, long long r) {
        if (c != r) n += snprintf(buf + n, sizeof buf - n, "  %s: C %llx RTL %llx\n", what, c, r);
    };
    const ChgCpu *c = &m->cpu;
    chk("cycles", (long long)m->cycles, (long long)top->cycles);
    chk("instret", (long long)m->instret, (long long)top->instret);
    chk("pc", c->pc, top->dbg_pc);
    for (int i = 1; i < 32; i++) {
        char nm[8];
        snprintf(nm, sizeof nm, "x%d", i);
        chk(nm, c->x[i], xr(i));
    }
    chk("mstatus", c->mstatus, top->dbg_mstatus);
    chk("mepc", c->mepc, top->dbg_mepc);
    chk("mcause", c->mcause, top->dbg_mcause);
    chk("trap_depth", c->trap_depth, top->dbg_depth);
    chk("wfi", c->wfi, top->dbg_wfi);
    chk("fetch_word", (int32_t)c->fetch_word, ((int32_t)(top->dbg_fw << 14)) >> 14);
    const long long cfd = (long long)(c->fetch_time - m->cycles);
    const long long rfd = ((int32_t)(top->dbg_fd << 8)) >> 8;
    if (cfd > -900 || rfd > -880) chk("fetch_time-cycles", cfd, rfd);
    chk("div_ready-cycles", c->div_ready > m->cycles ? (long long)(c->div_ready - m->cycles) : 0, top->dbg_div_rem);
    chk("div_rd", c->div_rd, top->dbg_div_rd);
    chk("frac8", c->frac8, top->dbg_frac);
    chk("pair_open", c->pair_open, top->dbg_pair);
    chk("recent_data", c->recent_data, top->dbg_recent);
    chk("faulted", c->faulted, top->faulted);
    if (!n) return 0;
    printf("DIFFERENCE %s, after the step at cycle %llu (instret %llu):\n%s", when,
           (unsigned long long)m->cycles, (unsigned long long)m->instret, buf);
    return 1;
}

static int compare_mem(bool full, uint32_t store_ea, int store_size)
{
    for (int k = 0; k < nwr; k++) {
        const uint32_t i = wr_idx[k];
        uint32_t cw;
        memcpy(&cw, m->ram + 4 * i, 4);
        if ((cw & wr_mask[k]) != (ram[i] & wr_mask[k])) {
            printf("DIFFERENCE: SRAM word %05x (RTL wrote mask %08x): C %08x RTL %08x\n",
                   0x20000000u + 4 * i, wr_mask[k], cw, ram[i]);
            return 1;
        }
    }
    if (store_size) {
        const uint32_t off = store_ea - CHG_RAM_BASE;
        if (off <= CHG_RAM_SIZE - (uint32_t)store_size)
            for (int b = 0; b < store_size; b++)
                if (m->ram[off + b] != ((const uint8_t *)ram)[off + b]) {
                    printf("DIFFERENCE: the store to %08x: C %02x RTL %02x at +%d\n", store_ea,
                           m->ram[off + b], ((const uint8_t *)ram)[off + b], b);
                    return 1;
                }
    }
    if (full) {
        for (uint32_t i = 0; i < 0x1400; i++) {
            uint32_t cw;
            memcpy(&cw, m->ram + 4 * i, 4);
            if (cw != ram[i]) {
                printf("DIFFERENCE: SRAM word %08x: C %08x RTL %08x\n", 0x20000000u + 4 * i, cw, ram[i]);
                return 1;
            }
        }
        if (memcmp(flash, m->flash, sizeof flash)) { printf("DIFFERENCE: flash\n"); return 1; }
    }
    return 0;
}

static int compare_ref(bool full)
{
    if (!ref) return 0;
    bool bad = ref->cycles != m->cycles || ref->instret != m->instret || ref->cpu.pc != m->cpu.pc ||
               memcmp(ref->cpu.x, m->cpu.x, sizeof m->cpu.x) ||
               (full && (memcmp(ref->ram, m->ram, sizeof m->ram) || memcmp(ref->flash, m->flash, sizeof m->flash)));
    if (bad) {
        printf("REFERENCE DIFFERS from the testbench's machine at cycle %llu / %llu: the early bus_events changed something\n",
               (unsigned long long)ref->cycles, (unsigned long long)m->cycles);
        return 1;
    }
    return 0;
}

#include "tb_cpu_random.cpp"

static ChgMachine *new_machine(const char *game, const char *sd)
{
    char err[512];
    ChgMachine *mm = (ChgMachine *)calloc(1, sizeof(ChgMachine));
    chg_init(mm);
    if (sd && !(mm->sd = sdcard_open(sd, 0, err, sizeof err))) { fprintf(stderr, "%s\n", err); exit(1); }
    if (!chg_load_program(mm, game, err, sizeof err)) { fprintf(stderr, "%s\n", err); exit(1); }
    chg_reset(mm, true);
    return mm;
}

int main(int argc, char **argv)
{
    const char *game = NULL, *sd = NULL, *sdref = NULL;
    double seconds = 1, every = 1;
    bool want_ref = true;
    uint64_t full_every = 2000000;
    long rseed = -1;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--sd") && i + 1 < argc) sd = argv[++i];
        else if (!strcmp(argv[i], "--sdref") && i + 1 < argc) sdref = argv[++i];
        else if (!strcmp(argv[i], "--noref")) want_ref = false;
        else if (!strcmp(argv[i], "--busy")) busy_test = true;
        else if (!strcmp(argv[i], "--prof")) prof_on = true;
        else if (!strcmp(argv[i], "--random") && i + 1 < argc) rseed = strtol(argv[++i], 0, 0);
        else if (!strcmp(argv[i], "--full") && i + 1 < argc) full_every = strtoull(argv[++i], 0, 0);
        else if (!strcmp(argv[i], "--every") && i + 1 < argc) every = atof(argv[++i]);
        else if (!strcmp(argv[i], "--press") && i + 1 < argc && npress < 32) {
            const char *s = argv[++i];
            static const struct { const char *n; uint8_t b; } names[] = {
                { "up", CHG_BTN_UP }, { "down", CHG_BTN_DOWN }, { "left", CHG_BTN_LEFT }, { "right", CHG_BTN_RIGHT },
                { "a", CHG_BTN_A }, { "b", CHG_BTN_B }, { "select", CHG_BTN_SELECT }, { "start", CHG_BTN_START } };
            const char *at = strchr(s, '@');
            uint8_t bit = 0;
            for (auto &nm : names) if (at && (size_t)(at - s) == strlen(nm.n) && !strncmp(s, nm.n, at - s)) bit = nm.b;
            if (!bit) { fprintf(stderr, "bad --press %s\n", s); return 2; }
            press[npress].bit = bit;
            press[npress].at = atof(at + 1);
            const char *colon = strchr(at, ':');
            press[npress].dur = colon ? atof(colon + 1) : 0.1;
            npress++;
        } else if (!game) game = argv[i];
        else seconds = atof(argv[i]);
    }
    if (rseed >= 0 && game) { seconds = atof(game); game = NULL; }
    if (!game && rseed < 0) {
        fprintf(stderr, "usage: cpu.exe game.chg [seconds] [--sd folder --sdref folder2] [--press btn@sec[:dur]]... [--noref]\n");
        return 2;
    }
    if (rseed >= 0) {
        m = random_machine((uint32_t)rseed);
        if (want_ref) ref = random_machine((uint32_t)rseed);
    } else {
        m = new_machine(game, sd);
        if (want_ref) ref = new_machine(game, sdref ? sdref : sd);
    }

    memcpy(flash, m->flash, sizeof flash);
    memcpy(ram, m->ram, sizeof ram);

    top = new Vchg_cpu;
    top->entry = m->entry;
    top->reset = 1;
    settle(); edge(); edge();
    top->reset = 0;
    settle();
    uint64_t guard = 0;
    while (!top->boundary) { edge(); if (++guard > 1000) { printf("no boundary after reset\n"); return 1; } }

    const uint64_t end = (uint64_t)(seconds * CHG_HCLK);
    const uint64_t report = (uint64_t)(every * CHG_HCLK);
    uint64_t next_report = report, steps = 0, ints = 0, sleeps = 0, syncs = 0;
    bool post_ram = false, post_flash = false;
    uint64_t asleep = 0;
    auto t0 = std::chrono::steady_clock::now();
    int rc = 0;

    if (compare("at the start")) return 1;
    while (m->cycles < end) {
        // the step's inputs
        uint8_t held = 0;
        const double t = (double)m->cycles / CHG_HCLK;
        for (int i = 0; i < npress; i++)
            if (t >= press[i].at && t < press[i].at + press[i].dur) held |= press[i].bit;
        if (held != m->buttons) { chg_set_buttons(m, held); if (ref) chg_set_buttons(ref, held); }

        bool copy = false;
        if (post_ram) copy |= sync_ram();
        if (post_flash) copy |= sync_flash();
        if (!m->reset_request && m->cycles >= m->next_event) {
            bus_events(m);
            chg_schedule(m);
            copy |= sync_ram();
        }
        if (copy) syncs++;
        // the lines change in the boundary clock here (in chg_soc at its end): irq_ev, so that
        // the CPU does not decide in that clock on the lines of the clock before
        top->irq_ev = top->irq_line != m->pfic.line;
        top->irq_line = m->pfic.line;
        memcpy(x_before, m->cpu.x, sizeof x_before);
        fc_before = m->flashctl;
        const uint64_t ins0 = m->instret, cyc0 = m->cycles;
        const int depth0 = m->cpu.trap_depth;
        const uint32_t thr0 = m->pfic.ithresdr;

        chg_run(m, m->cycles + 1);
        if (ref) chg_run(ref, ref->cycles + 1);
        if (m->instret == ins0) { sleeps++; asleep++; } else asleep = 0;
        if (rseed >= 0 && asleep > 200000) { printf("asleep for good at cycle %llu: stopping" "%c", (unsigned long long)m->cycles, 10); break; }
        if (m->cpu.trap_depth > depth0 && m->cpu.mcause >> 31) {
            ints++;
            if (m->cpu.intsyscr & 1) hpe_ints++;
        }

        // the RTL, to its next boundary
        nwr = 0;
        bus_spi_dma = bus_flash = bus_bad = false;
        top->mem_b_wr = copy;
        uint64_t c0 = clocks;
        edge();
        top->mem_b_wr = 0;
        while (!top->boundary) {
            edge();
            if (clocks - c0 > 400000 + 4 * (m->cycles - cyc0)) {
                printf("RTL stuck: no boundary for %llu clocks (state %u, pc %08x)\n",
                       (unsigned long long)(clocks - c0), top->dbg_state, top->dbg_pc);
                rc = 1;
                break;
            }
        }
        steps++;
        if (m->instret == ins0) ret_op = 61;    // asleep: shown as wfi
        hist[nhist % 16] = { m->cycles, m->instret, top->retire_pc, ret_op };
        nhist++;
        post_ram = bus_spi_dma;
        post_flash = bus_flash;

        if (rc) break;
        if (bus_bad) { printf("DIFFERENCE: %s\n", bus_msg); rc = 1; break; }
        const int op = ret_op;
        const bool st = m->instret != ins0 && is_store(op);
        const int ssize = (op == 16) ? 1 : (op == 17) ? 2 : st ? 4 : 0;
        if (compare("") || compare_mem(full_every && steps % full_every == 0 && !post_ram && !post_flash,
                                       ret_ea, ssize) || compare_ref(full_every && steps % full_every == 0)) {
            rc = 1;
            break;
        }
        if (m->cpu.faulted && !fault_seen) {
            fault_seen = true;
            printf("the C machine noted a HardFault at %08x (cause %u)\n", m->cpu.fault_pc, m->cpu.fault_cause);
        }
        if (m->cpu.mstatus & 8) mie_steps++;
        if (m->pfic.enabled & (m->pfic.pending | m->pfic.line)) ready_steps++;
        if (m->pfic.ithresdr != thr0 && is_store(ret_op) && ret_ea >= 0xE000E4FDu && ret_ea <= 0xE000E4FFu && !thr_noted++)
            printf("NOTE: the C's priority store at %08x ran past prio[255] into ITHRESDR (%u -> %u) at cycle %llu" "%c",
                   ret_ea, thr0, m->pfic.ithresdr, (unsigned long long)m->cycles, 10);
        if (m->instret != ins0) {
            op_count[ret_op & 127]++;
            if (is_store(ret_op) && ret_ea >= 0xE000E000u && ret_ea < 0xE000F000u) pfic_stores++;
            if (is_store(ret_op) && ret_ea >= 0xE000E200u && ret_ea < 0xE000E208u) { pend_stores++; if (pend_stores < 6) printf("IPSR store %s at %08x x_before[rs2]=%08x pending now %016llx enabled %016llx" "%c", op_name(ret_op), ret_ea, x_before[top->dbg_rs2], (unsigned long long)m->pfic.pending, (unsigned long long)m->pfic.enabled, 10); }
            if (top->retire_pc >= CHG_RAM_BASE) ram_steps++;
            if (top->retire_pc >= CHG_FLASH_ALIAS && top->retire_pc < CHG_FLASH_ALIAS + CHG_FLASH_SIZE) alias_steps++;
            if (m->cpu.trap_depth > depth0 && !(m->cpu.mcause >> 31)) excs++;
        }
        if (m->cycles >= next_report) {
            next_report += report;
            double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            printf("t=%.2fs cycles %llu instret %llu clocks %llu (%.3f clocks/cycle) ints %llu sleeps %llu syncs %llu  %.0f s wall\n",
                   (double)m->cycles / CHG_HCLK, (unsigned long long)m->cycles, (unsigned long long)m->instret,
                   (unsigned long long)clocks, (double)clocks / (double)m->cycles,
                   (unsigned long long)ints, (unsigned long long)sleeps, (unsigned long long)syncs, secs);
            fflush(stdout);
        }
    }
    if (!rc) {
        if (post_ram) sync_ram();
        if (post_flash) sync_flash();
        rc = compare_mem(true, 0, 0);
    }
    if (rc) {
        dump_history();
        printf("RTL: state %u pc %08x op %s ea %08x\n", top->dbg_state, top->dbg_pc, op_name(top->dbg_op), top->dbg_ea);
        // the code around the last instruction, as the C decodes it
        const uint32_t pc0 = (hist[(nhist + 15) % 16].pc & ~1u) - 24;
        for (uint32_t a = pc0; a < pc0 + 48; a += 2) {
            uint32_t raw = 0;
            if (a < CHG_FLASH_SIZE - 4) memcpy(&raw, m->flash + a, 4);
            else if (a - CHG_RAM_BASE < CHG_RAM_SIZE - 4) memcpy(&raw, m->ram + (a - CHG_RAM_BASE), 4);
            ChgInsn d;
            cpu_decode(a, raw, &d, true);
            printf("  %08x: %08x %-10s rd %u rs1 %u rs2 %u imm %d\n", a, raw, op_name(d.op), d.rd, d.rs1, d.rs2, d.imm);
        }
    }
    printf("coverage: interrupts %llu (%llu with HPE), exceptions %llu, steps in RAM code %llu, bus reads %llu, writes %llu\n",
           (unsigned long long)ints, (unsigned long long)hpe_ints, (unsigned long long)excs,
           (unsigned long long)ram_steps, (unsigned long long)bus_reads, (unsigned long long)bus_writes);
    printf("steps with MIE %llu, with an interrupt ready %llu; PFIC at the end: enabled %016llx pending %016llx line %016llx thresh %u intsyscr %x\n",
           (unsigned long long)mie_steps, (unsigned long long)ready_steps, (unsigned long long)m->pfic.enabled,
           (unsigned long long)m->pfic.pending, (unsigned long long)m->pfic.line, m->pfic.ithresdr, m->cpu.intsyscr);
    printf("stores to the PFIC %llu, to IPSR %llu; steps in the 0x08000000 alias %llu; flash writes %llu; bus_wait given %llu; flash written %d" "%c",
           (unsigned long long)pfic_stores, (unsigned long long)pend_stores, (unsigned long long)alias_steps,
           (unsigned long long)flash_writes, (unsigned long long)waits, (int)m->flash_written, 10);
    printf("ops retired:");
    for (int k = 1; k < 69; k++) if (op_count[k]) printf(" %s %llu", op_name(k), (unsigned long long)op_count[k]);
    printf("\nops never retired:");
    for (int k = 1; k < 69; k++) if (!op_count[k]) printf(" %s", op_name(k));
    printf("\n");
    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("%s: %llu steps, cycles %llu (%.3f s of board time), instret %llu, clocks %llu, %.1f s wall\n",
           rc ? "FAILED" : "LOCKSTEP OK", (unsigned long long)steps, (unsigned long long)m->cycles,
           (double)m->cycles / CHG_HCLK, (unsigned long long)m->instret, (unsigned long long)clocks, secs);
    if (prof_on) {
        printf("clocks by state (per board cycle):");
        for (int k = 0; k < 67; k++)
            if (prof[k]) printf(" %s%d %.3f", k == 64 ? "ce" : k == 65 ? "bnd" : k == 66 ? "wait" : "s", k, (double)prof[k] / (double)m->cycles);
        printf("%c", 10);
    }
    delete top;
    return rc;
}
