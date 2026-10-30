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
#include "Vchg_soc___024root.h"
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
static int ntr; static uint32_t tr_pc[300], tr_w[300]; static uint64_t tr_clk[300], tr_ce[300];
static uint64_t nblk_rd, nblk_wr, pend, opc_n[32], opc_clk[32], c_fl, c_adc, c_hbus, c_bl;
static uint64_t nclk, st_cnt[128], opc_ce[32], opc_hold[32], pc_hold[1<<15], pc_ce[1<<15];
static void edge()
{
    top->clk = 0; top->eval();
    nclk++;
    {
        auto *r = top->rootp;
        const int st = r->chg_soc__DOT__cpu__DOT__state;
        const uint32_t pc = r->chg_soc__DOT__cpu__DOT__pc;
        int k;
        if (top->ce) k = 100;
        else if (r->chg_soc__DOT__spi__DOT__busy_q) k = 101;
        else if (!top->run) k = 105;
        else if (st == 0) k = r->chg_soc__DOT__cpu__DOT__stale ? 102 : top->dbg_boundary ? 103 : 104;
        else k = st & 63;
        st_cnt[k]++;
        if (top->retire && now > 240000000ull + 9000000 && ntr < 300) { uint32_t rp = top->retire_pc; uint32_t w = 0;
            if ((rp >> 28) == 2) { uint32_t wi = (rp & 0xffff) >> 2; if (wi < 5120) w = r->chg_soc__DOT__sram__DOT__mem[wi]; if (rp & 2) w = w >> 16; }
            tr_pc[ntr] = rp; tr_clk[ntr] = nclk; tr_w[ntr] = w; tr_ce[ntr] = now; ntr++; }
        if (top->retire) {
            uint32_t rp = top->retire_pc; int oc = 31; uint32_t w = 0;
            if ((rp >> 28) != 2) { uint32_t i = rp & 0xffff; w = m.flash[i] | m.flash[i+1] << 8; oc = ((w & 3) != 3) ? 30 : (w >> 2) & 31; pc_hold[i >> 2] += pend; pc_ce[i >> 2]++; }
            opc_n[oc]++; opc_clk[oc] += pend; pend = 0;
        }
        pend++;
        if (k != 100) { if (r->chg_soc__DOT__periph__DOT__u_fl__DOT__stall) c_fl++; if (r->chg_soc__DOT__periph__DOT__u_adc__DOT__stall) c_adc++; if (r->chg_soc__DOT__hold_bus) c_hbus++; if (r->chg_soc__DOT__cpu__DOT__bl_stall) c_bl++; }
    }
    if (top->ce) now++;             // the ce this rising edge acts on
    top->clk = 1; top->eval();
    top->sd_buff_wr = 0;
    switch (sd_state) {
    case 0:
        if (top->sd_rd || top->sd_wr) {
            sd_lba_cur = top->sd_lba; if (top->sd_rd) nblk_rd++; else nblk_wr++;
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

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: prof game.chg seconds [sd folder]\n"); return 2; }
    char err[256];
    setvbuf(stdout, NULL, _IONBF, 0); chg_init(&m);
    if (argc > 3) {
        if (!(m.sd = sdcard_open(argv[3], 0, err, sizeof err))) { fprintf(stderr, "%s\n", err); return 1; }
        rcard = sdcard_open(argv[3], 0, err, sizeof err);
    }
    if (!chg_load_program(&m, argv[1], err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    chg_reset(&m, true);
    top = new Vchg_soc;
    top->hold_mode = 1; top->run = 0;
    top->sd_mounted = rcard != nullptr;
    top->sd_size = rcard ? sdcard_sectors(rcard) * 512 : 0;
    top->reset = 1;
    for (int a = 0; a < 0x10000; a++) { top->fl_we = 1; top->fl_addr = a; top->fl_data = m.flash[a]; edge(); }
    top->fl_we = 0;
    for (int i = 0; i < 4; i++) edge();
    top->reset = 0; now = 0; nclk = 0; memset(st_cnt, 0, sizeof st_cnt);
    memset(opc_n, 0, sizeof opc_n); memset(opc_clk, 0, sizeof opc_clk); c_fl = c_adc = c_hbus = c_bl = 0; memset(opc_ce, 0, sizeof opc_ce); memset(opc_hold, 0, sizeof opc_hold); memset(pc_hold, 0, sizeof pc_hold); memset(pc_ce, 0, sizeof pc_ce);
    top->run = 1;
    const uint64_t end = (uint64_t)(atof(argv[2]) * CHG_HCLK);
    uint64_t wc = 0, wn = 0, worst_c = 0, worst_n = 1; int worst_s = 0;
    uint64_t sc[128]; memset(sc, 0, sizeof sc);
    while (now < end) {
        const double s = (double)now / CHG_HCLK;
        uint8_t b = ((s >= 3 && s < 3.1) || (s >= 5 && s < 5.1)) ? CHG_BTN_START : (s >= 6) ? CHG_BTN_A : 0;
        top->buttons = b;
        for (int i = 0; i < 20000; i++) edge();
        if (now - wn >= CHG_HCLK) {
            uint64_t dc = nclk - wc, dn = now - wn;
            printf("  s%llu: %.3f\n", (unsigned long long)(now / CHG_HCLK), (double)dc / dn);
            if (dc * worst_n > worst_c * dn) { worst_c = dc; worst_n = dn; worst_s = (int)(now / CHG_HCLK); }
            wc = nclk; wn = now;
        }
    }
    printf("%s: clk %llu cycles %llu : %.3f clk/cycle, worst 1 s (ending s%d) %.3f\n", argv[1], (unsigned long long)nclk, (unsigned long long)now,
           (double)nclk / now, worst_s, (double)worst_c / worst_n);
    printf("card blocks read %llu written %llu\n", (unsigned long long)nblk_rd, (unsigned long long)nblk_wr);
    for (int i = 1; i < ntr; i++) printf("T pc %08x w %08x dclk %llu dcyc %llu\n", tr_pc[i-1], tr_w[i-1], (unsigned long long)(tr_clk[i] - tr_clk[i-1]), (unsigned long long)(tr_ce[i] - tr_ce[i-1]));
    printf("share of fabric clocks (clk per board cycle):\n");
    for (int k = 0; k < 128; k++) if (st_cnt[k]) printf("  %s%d %.4f\n", k==100?"CE ":k==101?"busy_q ":k==102?"stale ":k==103?"bnd ":k==104?"run_wait ":k==105?"norun ":"state", k<100?k:0, (double)st_cnt[k] / now);
    printf("stall flags in non-ce clocks per cycle: flash %.4f adc %.4f hold_bus %.4f bl %.4f\n", (double)c_fl/now, (double)c_adc/now, (double)c_hbus/now, (double)c_bl/now);
    printf("by opcode[6:2] (31=sram/other 30=RVC):\n");
    for (int k = 0; k < 32; k++) if (opc_n[k]) printf("  op%02x n/cyc %.4f clk/insn %.2f clk/cyc %.4f\n", k, (double)opc_n[k] / now, (double)opc_clk[k] / opc_n[k], (double)opc_clk[k] / now);
    // top hold pcs
    for (int n = 0; n < 12; n++) {
        int bi = 0; for (int i = 0; i < (1 << 14); i++) if (pc_hold[i] > pc_hold[bi]) bi = i;
        uint32_t w = m.flash[bi*4] | m.flash[bi*4+1] << 8 | m.flash[bi*4+2] << 16 | (uint32_t)m.flash[bi*4+3] << 24;
        printf("  pc %05x insn %08x hold %.4f ce %.4f\n", bi * 4, w, (double)pc_hold[bi] / now, (double)pc_ce[bi] / now);
        pc_hold[bi] = 0;
    }
    return 0;
}
