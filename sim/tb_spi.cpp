// chg_spi.sv + chg_dma.sv + chg_sdspi.sv + st7735.sv against emu/src (bus.c's
// SPI1/DMA1, sdspi.c, st7735.c).
//
//   tb_spi random <ops> <seed> <cardfolder>
//       A C ChgMachine with its CPU kept out of the way and the RTL get the
//       same register accesses, GPIO writes, CPU stores to SRAM and spi_sync
//       calls at the same board times (m->cycles set, bus_write_slow /
//       bus_read_slow / spi_sync); LCD, SD and DMA traffic in random scenes.
//   tb_spi game <program.CHG> <seconds> [cardfolder]
//       The C machine runs the game one instruction at a time, exactly as
//       chg_run(m, m->cycles + 1) does (step() below is chg_run with a hook
//       before cpu_exec). Each instruction's load or store is decoded before
//       it runs, with the cycle it is made at (its issue cycle, worked out as
//       cpu_exec does), and made on the RTL at that board time; the end of
//       the instruction is the RTL's sync. RTL and C are compared after every
//       instruction (SPI/DMA state, interrupt lines, SRAM the DMA wrote), at
//       every load (the value the C CPU got against the RTL's register or
//       SRAM), and every 1/60 s (all of SRAM and the 128x128 picture).
//   tb_spi latency <cardfolder>
//       hold_mode 0 (the MiSTer): blocks arrive thousands of cycles late; a
//       host driving the RTL as SdSpi.cpp does must still read and write the
//       right blocks.
//
// Every difference is printed (the first few of each kind) and counted; the
// exit code is 1 if there were any.
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <string>
#include "Vchg_spi_tb_top.h"
#include "Vchg_spi_tb_top___024root.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
#include "loader.h"
#include "sdcard.h"
}

static Vchg_spi_tb_top *top;
static ChgMachine M;
static ChgMachine *m = &M;
static uint8_t ram_rtl[CHG_RAM_SIZE];
static uint8_t ram_dirty[CHG_RAM_SIZE];
static std::vector<uint32_t> dirty_list;
static SdCard *card;
static uint64_t nticks;
static long errors;
static int err_shown;
static bool hps_late;           // latency test: hps_io answers late
static int hps_delay;
static long blocks_read, blocks_written;

static void fail(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void fail(const char *fmt, ...)
{
    errors++;
    if (err_shown++ < 30) {
        va_list ap;
        va_start(ap, fmt);
        printf("DIFF @%llu: ", (unsigned long long)m->cycles);
        vprintf(fmt, ap);
        printf("\n");
        va_end(ap);
    }
    if (getenv("TBSTOP")) { printf("stopping at the first difference\n"); exit(1); }
}

// ---------------------------------------------------------------------------
// the clock, port B and hps_io
// ---------------------------------------------------------------------------

static int hps_state, hps_k, hps_wait;
// random mode: the RTL's writes go to a card of its own, compared with the C's at the end
static SdCard *card_rtl;
static uint8_t c_snap[512];
static bool track_c_writes;
static uint8_t hps_blk[512];
static uint32_t hps_lba;
static void tick()
{
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    nticks++;

    // port B: SRAM and flash, one clock later
    top->mack = 0;
    if (top->mreq) {
        const uint32_t a = top->maddr, n = top->msize;
        uint32_t v = 0;
        if (n != 1 && n != 2 && n != 4) fail("port B size %u", n);
        if (a & (n - 1)) fail("port B misaligned %08x size %u", a, n);
        if (a - CHG_RAM_BASE < CHG_RAM_SIZE) {
            const uint32_t off = a - CHG_RAM_BASE;
            if (top->mwe) {
                const uint32_t w = top->mwdata;
                memcpy(ram_rtl + off, &w, n);
                for (uint32_t i = 0; i < n; i++)
                    if (!ram_dirty[off + i]) { ram_dirty[off + i] = 1; dirty_list.push_back(off + i); }
            } else memcpy(&v, ram_rtl + off, n);
        } else if (a + n <= CHG_FLASH_SIZE) {
            if (top->mwe) fail("port B write to flash %08x", a);
            else memcpy(&v, m->flash + a, n);
        } else fail("port B outside memory %08x", a);
        top->mrdata = v;
        top->mack = 1;
    }

    // hps_io
    top->sd_buff_wr = 0;
    switch (hps_state) {
    case 0:
        if (top->sd_rd || top->sd_wr) {
            if (hps_late && hps_wait < hps_delay) { hps_wait++; break; }
            hps_wait = 0;
            hps_lba = top->sd_lba;
            top->sd_ack = 1;
            if (top->sd_rd) {
                if (!card || !sdcard_read(card, hps_lba, hps_blk, 1)) fail("hps: read of block %u fails", hps_lba);
                blocks_read++;
                hps_state = 1;
                hps_k = 0;
            } else {
                top->sd_buff_addr = 0;
                hps_k = 1;
                hps_state = 2;
                // the C card has made this write already (the C runs ahead): what it holds now
                if (track_c_writes && !sdcard_read(card, hps_lba, c_snap, 1)) fail("hps: block %u not on the card", hps_lba);
            }
        }
        break;
    case 1:     // the block into the core
        top->sd_buff_addr = hps_k;
        top->sd_buff_dout = hps_blk[hps_k];
        top->sd_buff_wr = 1;
        if (++hps_k == 512) hps_state = 3;
        break;
    case 2:     // the block out of the core: sd_buff_din is a clock behind the address
        hps_blk[hps_k - 1] = top->sd_buff_din;
        if (hps_k == 512) {
            uint8_t want[512];
            blocks_written++; if (getenv("TBW")) printf("RTL write lba %u @%llu\n", hps_lba, (unsigned long long)top->dbg_now);
            if (hps_late) {
                if (!sdcard_write(card, hps_lba, hps_blk, 1)) fail("hps: write of block %u fails", hps_lba);
            } else if (track_c_writes) {
                if (memcmp(c_snap, hps_blk, 512)) fail("hps: written block %u differs from the C card's", hps_lba);
                sdcard_write(card_rtl, hps_lba, hps_blk, 1);
            } else if (!card || !sdcard_read(card, hps_lba, want, 1) || memcmp(want, hps_blk, 512)) {
                fail("hps: written block %u differs from the C card's", hps_lba);
            }
            hps_state = 3;
        } else {
            top->sd_buff_addr = hps_k;
            hps_k++;
        }
        break;
    case 3:
        top->sd_ack = 0;
        hps_state = 0;
        break;
    }
}

static void wait_idle()
{
    long k = 0;
    while (top->busy) {
        tick();
        if (++k > 2000000) { printf("FATAL: busy stuck\n"); exit(2); }
    }
}

static void advance_to(uint64_t t)
{
    while (top->dbg_now < t) {
        wait_idle();
        top->ce = 1; tick(); top->ce = 0;
    }
    if (top->dbg_now != t) fail("RTL time %llu past %llu", (unsigned long long)top->dbg_now, (unsigned long long)t);
}

static uint32_t rtl_access(bool we, uint32_t addr, int size, uint32_t wdata)
{
    wait_idle();
    top->req = 1; top->we = we; top->addr = addr; top->size = size; top->wdata = wdata;
    tick();
    top->req = 0;
    long k = 0;
    while (!top->ack) {
        tick();
        if (++k > 2000000) { printf("FATAL: no ack for %08x\n", addr); exit(2); }
    }
    const uint32_t r = top->rdata;
    wait_idle();
    return r;
}

static void rtl_sync() { top->boundary = 1; tick(); top->boundary = 0; wait_idle(); }
static void rtl_gpio_wr() { top->gpio_wr = 1; tick(); top->gpio_wr = 0; wait_idle(); }
static void rtl_reset(bool por)
{
    wait_idle();
    top->reset = 1; top->por = por; tick(); tick(); top->reset = 0; top->por = 0; tick();
}
static void lcd_reset() { top->lcd_reset = 1; tick(); top->lcd_reset = 0; tick(); }

static bool last_rst = true;
// the GPIO stand-in follows the C's pins; PB12 falling resets the display
static void set_pins()
{
    const unsigned cfg4 = (m->gpio[0].cfg[0] >> 16) & 15;
    top->pa4_af = (cfg4 & 3) && (cfg4 & 8);
    top->pa4_gpio = gpio_pin(m, 0, 4);
    top->pb0 = gpio_pin(m, 1, 0);
    top->pb11 = gpio_pin(m, 1, 11);
    top->lcd_rst_level = m->lcd.rst_level;
    if (last_rst && !m->lcd.rst_level) lcd_reset();
    last_rst = m->lcd.rst_level;
}

// ---------------------------------------------------------------------------
// comparisons
// ---------------------------------------------------------------------------

static long state_checks, glass_checks, glass_content, load_checks, reg_reads;
static uint32_t last_hash, picture_changes;

static void compare_state(const char *where)
{
    const ChgSpi *s = &m->spi;
    state_checks++;
    const unsigned f = top->dbg_flags;
    const unsigned cf = (s->shifting ? 1 : 0) | (s->tx_full ? 2 : 0) | (s->rx_full ? 4 : 0) |
                        (m->sd_selected ? 8 : 0) | ((s->statr_sticky & 0x40) ? 16 : 0);
    if (top->dbg_now != m->cycles) fail("%s: now %llu c %llu", where, (unsigned long long)top->dbg_now, (unsigned long long)m->cycles);
    if (f != cf) fail("%s: flags rtl %02x c %02x (shifting tx_full rx_full sd_sel ovr)", where, f, cf);
    if (top->dbg_rx_data != s->rx_data) fail("%s: rx_data rtl %04x c %04x", where, top->dbg_rx_data, s->rx_data);
    if (top->dbg_tx_data != s->tx_data) fail("%s: tx_data rtl %04x c %04x", where, top->dbg_tx_data, s->tx_data);
    if (top->dbg_shift_data != s->shift_data) fail("%s: shift_data rtl %04x c %04x", where, top->dbg_shift_data, s->shift_data);
    if (top->dbg_t != s->t) fail("%s: t rtl %llu c %llu", where, (unsigned long long)top->dbg_t, (unsigned long long)s->t);
    if (top->dbg_shift_end != s->shift_end)
        fail("%s: shift_end rtl %llu c %llu", where, (unsigned long long)top->dbg_shift_end, (unsigned long long)s->shift_end);
    if (top->dbg_intfr != m->dma.intfr) fail("%s: intfr rtl %08x c %08x", where, top->dbg_intfr, m->dma.intfr);
    if (top->dbg_cntr1 != m->dma.ch[1].cntr || top->dbg_cntr2 != m->dma.ch[2].cntr)
        fail("%s: cntr rtl %u %u c %u %u", where, top->dbg_cntr1, top->dbg_cntr2, m->dma.ch[1].cntr, m->dma.ch[2].cntr);
    if (top->dbg_pos1 != (m->dma.ch[1].pos & 0xffff) || top->dbg_pos2 != (m->dma.ch[2].pos & 0xffff))
        fail("%s: pos rtl %u %u c %u %u", where, top->dbg_pos1, top->dbg_pos2, m->dma.ch[1].pos, m->dma.ch[2].pos);
    const unsigned irq_c = (unsigned)((m->pfic.line >> CHG_IRQ_DMA1_CH1) & 0x7f);
    if (top->irq_dma != irq_c) fail("%s: DMA irq lines rtl %02x c %02x", where, top->irq_dma, irq_c);
    const unsigned spi_c = (unsigned)((m->pfic.line >> CHG_IRQ_SPI1) & 1);
    if (top->irq_spi != spi_c) fail("%s: SPI1 irq line rtl %u c %u", where, top->irq_spi, spi_c);
    { const auto *rp = top->rootp;  // the card's own state against sdspi.c's
      if (rp->chg_spi_tb_top__DOT__sd__DOT__wstate != m->sdspi.wstate || rp->chg_spi_tb_top__DOT__sd__DOT__nblk != m->sdspi.nblk ||
          rp->chg_spi_tb_top__DOT__sd__DOT__ncmd != m->sdspi.ncmd || rp->chg_spi_tb_top__DOT__sd__DOT__pout != m->sdspi.pout)
          fail("%s: card wstate/nblk/ncmd/pout rtl %d %d %d %d c %d %d %d %d", where, rp->chg_spi_tb_top__DOT__sd__DOT__wstate, rp->chg_spi_tb_top__DOT__sd__DOT__nblk, rp->chg_spi_tb_top__DOT__sd__DOT__ncmd, rp->chg_spi_tb_top__DOT__sd__DOT__pout, m->sdspi.wstate, m->sdspi.nblk, m->sdspi.ncmd, m->sdspi.pout); }
}

// the SRAM bytes the RTL's DMA wrote since the last check
static void compare_dirty(const char *where)
{
    for (uint32_t off : dirty_list) {
        if (ram_rtl[off] != m->ram[off])
            fail("%s: SRAM %08x rtl %02x c %02x", where, CHG_RAM_BASE + off, ram_rtl[off], m->ram[off]);
        ram_dirty[off] = 0;
    }
    dirty_list.clear();
}

static void compare_ram(const char *where)
{
    compare_dirty(where);
    if (memcmp(ram_rtl, m->ram, CHG_RAM_SIZE)) {
        int n = 0;
        for (uint32_t i = 0; i < CHG_RAM_SIZE; i++)
            if (ram_rtl[i] != m->ram[i] && n++ < 4)
                fail("%s: SRAM %08x rtl %02x c %02x", where, CHG_RAM_BASE + i, ram_rtl[i], m->ram[i]);
        memcpy(ram_rtl, m->ram, CHG_RAM_SIZE);      // carry on from the C's
    }
}

static int compare_glass(const char *where)
{
    static uint32_t want[128 * 128];
    glass_checks++;
    st7735_render(&m->lcd, want);
    int bad = 0;
    for (int y = 0; y < 128; y++) {
        top->vy = y; top->vline = 1; tick(); top->vline = 0;
        for (int k = 0; k < 6; k++) tick();     // the line is fetched before its first pixel (as tb_soc)
        for (int x = 0; x < 128; x++) {
            top->vx = x;
            tick(); tick(); tick();
            const uint32_t got = top->vrgb;
            if (got != want[y * 128 + x]) {
                if (!bad) fail("%s: picture (%d,%d) rtl %06x c %06x", where, x, y, got, want[y * 128 + x]);
                bad++;
            }
        }
    }
    if (bad) printf("  %s: %d pixels differ (C display_on %d sleeping %d rst_level %d; RTL rst_level %d)\n", where, bad,
                    m->lcd.display_on, m->lcd.sleeping, m->lcd.rst_level, top->lcd_rst_level);
    // how much there was to compare: pictures not all one colour, and how often it changed
    uint32_t h = 2166136261u;
    bool varied = false;
    for (int i = 0; i < 128 * 128; i++) { h = (h ^ want[i]) * 16777619u; if (want[i] != want[0]) varied = true; }
    if (varied) glass_content++;
    if (h != last_hash) picture_changes++;
    last_hash = h;
    return bad;
}

// ---------------------------------------------------------------------------
// start
// ---------------------------------------------------------------------------

static void start_rtl(bool hold)
{
    top = new Vchg_spi_tb_top;
    top->hold_mode = hold;
    top->lcd_rst_level = 1;
    top->pa4_gpio = 1; top->pb11 = 1;
    rtl_reset(true);
    lcd_reset();
    if (card) {
        top->img_size = sdcard_sectors(card) * 512ull;
        top->img_mounted = 1; tick(); top->img_mounted = 0; tick();
    }
}

static SdCard *open_card(const char *folder)
{
    char err[256];
    SdCard *c = sdcard_open(folder, 0, err, sizeof err);
    if (!c) { printf("card %s: %s\n", folder, err); exit(2); }
    return c;
}

// ---------------------------------------------------------------------------
// random: C without its CPU, and the RTL, given the same things to do
// ---------------------------------------------------------------------------

static uint32_t rng = 1;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static uint32_t rn(uint32_t n) { return n ? rnd() % n : 0; }
static uint64_t T;
static long ops_done;

static const uint32_t SPI = 0x40013000, DMA = 0x40020000, GPIOA = 0x40010800, GPIOB = 0x40010C00;

static void at_T()
{
    advance_to(T);
    m->cycles = T;
}

static void c_store_ram(uint32_t off, uint32_t v, int n)
{
    memcpy(m->ram + off, &v, n);
    memcpy(ram_rtl + off, &v, n);
}

static void w(uint32_t addr, uint32_t v, int size = 4)
{
    at_T();
    if (getenv("TBV")) printf("  w %08x %08x /%d @%llu\n", addr, v, size, (unsigned long long)T);
    int wait = 0;
    if ((addr & ~0x7ffu) == 0x40010800u || (addr & ~0x3ffu) == 0x40011000u) {
        spi_sync(m, T);         // what gpio_write does first, before the RTL catches up
        rtl_gpio_wr();
        bus_write_slow(m, addr, v, size, &wait);
        set_pins();
    } else {
        bus_write_slow(m, addr, v, size, &wait);
        rtl_access(true, addr, size, v);
        if (getenv("TBV")) printf("    c ch1 cntr %u pos %u cfgr %x  rtl cntr %u pos %u\n", m->dma.ch[1].cntr, m->dma.ch[1].pos, m->dma.ch[1].cfgr, top->dbg_cntr1, top->dbg_pos1);
        set_pins();
    }
}

static uint32_t r(uint32_t addr, int size = 4)
{
    at_T();
    int wait = 0;
    const uint32_t c = bus_read_slow(m, addr, size, &wait);
    const uint32_t v = rtl_access(false, addr, size, 0);
    reg_reads++;
    if (c != v) fail("read %08x/%d rtl %08x c %08x", addr, size, v, c);
    return c;
}

static void sync_both()
{
    at_T();
    spi_sync(m, T);
    rtl_sync();
}

static void check(const char *where)
{
    compare_state(where);
    compare_dirty(where);
}

// a run of time with the SPI left alone, sync points as instructions would make them
static void run(uint32_t cycles)
{
    const uint64_t end = T + cycles;
    while (T < end) {
        T += 1 + rn(rn(6) ? 4 : 40);
        if (T > end) T = end;
        sync_both();
        check("run");
    }
}

static std::vector<uint8_t> q;      // bytes for the card
static void sd_cmd(int c, uint32_t arg)
{
    q.push_back(0xFF);
    q.push_back(0x40 | c);
    for (int sh = 24; sh >= 0; sh -= 8) q.push_back((uint8_t)(arg >> sh));
    q.push_back(c == 8 ? 0x87 : 0x95);
    for (int i = 0; i < 9; i++) q.push_back(0xFF);
}

static void sd_script()
{
    const uint32_t sectors = card ? (uint32_t)sdcard_sectors(card) : 0;
    const uint32_t lba = rn(4) ? rn(4000) : sectors - 3 + rn(6);
    switch (rn(12)) {
    case 0: sd_cmd(0, 0); sd_cmd(8, 0x1AA); sd_cmd(55, 0); sd_cmd(41, 1u << 30); sd_cmd(58, 0); break;
    case 1: sd_cmd(9, 0); for (int i = 0; i < 24; i++) q.push_back(0xFF); sd_cmd(10, 0); for (int i = 0; i < 24; i++) q.push_back(0xFF); break;
    case 2: sd_cmd(55, 0); sd_cmd(13, 0); for (int i = 0; i < 72; i++) q.push_back(0xFF);
            sd_cmd(55, 0); sd_cmd(51, 0); for (int i = 0; i < 16; i++) q.push_back(0xFF); break;
    case 3: case 4: sd_cmd(17, lba); for (int i = 0; i < 530; i++) q.push_back(0xFF); break;
    case 5: {
        sd_cmd(18, lba);
        const int n = 1 + rn(3);
        for (int i = 0; i < 520 * n + (int)rn(300); i++) q.push_back(0xFF);
        q.push_back(0x4C); for (int i = 0; i < 4; i++) q.push_back(0); q.push_back(0x61);
        for (int i = 0; i < 12; i++) q.push_back(0xFF);
        break;
    }
    case 6: {
        sd_cmd(24, 100 + rn(50));
        q.push_back(0xFF); q.push_back(0xFE);
        for (int i = 0; i < 514; i++) q.push_back((uint8_t)rnd());
        for (int i = 0; i < 8; i++) q.push_back(0xFF);
        break;
    }
    case 7: {
        sd_cmd(25, 100 + rn(50));
        const int n = 1 + rn(3);
        for (int b = 0; b < n; b++) {
            q.push_back(0xFF); q.push_back(0xFC);
            for (int i = 0; i < 514; i++) q.push_back((uint8_t)rnd());
            for (int i = 0; i < 4; i++) q.push_back(0xFF);
        }
        q.push_back(0xFD);
        for (int i = 0; i < 6; i++) q.push_back(0xFF);
        break;
    }
    case 8: sd_cmd(rn(64), rnd()); break;
    case 9: for (int i = rn(20); i; i--) q.push_back((uint8_t)rnd()); break;
    case 10: sd_cmd(13, 0); sd_cmd(16, 512); sd_cmd(59, 0); sd_cmd(12, 0); break;
    default: sd_cmd(55, 0); sd_cmd(rn(64), 0); break;
    }
}

static void spi_config(bool sd)
{
    uint32_t c1 = 0x0004 | 0x0040 | 0x0300;     // MSTR SPE SSI SSM
    if (sd) c1 |= (rn(3) ? 0 : rn(4)) << 3;
    else {
        c1 |= (rn(3) ? rn(2) : rn(8)) << 3;
        if (rn(2)) c1 |= 0x0800;                // 16 bit
        if (!rn(4)) c1 |= 0x0080;               // LSB first
    }
    if (!rn(3)) w(SPI + 0, c1 & ~0x40u);        // as spiSet does: SPE off, then on
    w(SPI + 0, c1);
}

// a DMA channel: n the C's channel index
static void dma_setup(int n, uint32_t cfgr, uint32_t paddr, uint32_t maddr, uint32_t cnt)
{
    w(DMA + 0x08 + n * 0x14, 0);
    w(DMA + 0x10 + n * 0x14, paddr);
    w(DMA + 0x14 + n * 0x14, maddr);
    w(DMA + 0x0C + n * 0x14, cnt);
    w(DMA + 0x08 + n * 0x14, cfgr);
}

// PSIZE/MSIZE 11 (reserved) makes the C copy 8 bytes into a 4-byte variable
// (dma_mem_read, memcpy(&v, ..., 8)): undefined behaviour, so never used here.
// Random CFGR values never set MEM2MEM: their PADDR is SPI1 DATAR, and a DMA
// reading a peripheral register is not modelled (port B is memory only).
static uint32_t no_size8(uint32_t v)
{
    if (((v >> 8) & 3) == 3) v &= ~0x100u;
    if (((v >> 10) & 3) == 3) v &= ~0x400u;
    return v;
}

static uint32_t rand_mem(uint32_t n)
{
    switch (rn(4)) {
    case 0: return 0x00003000 + rn(0x1000);                     // flash
    case 1: return 0x08003000 + rn(0x1000);                     // its alias
    default: return CHG_RAM_BASE + 0x1000 + rn(0x3000 - n);     // SRAM
    }
}

static void random_test(long nops, uint32_t seed)
{
    rng = seed ? seed : 1;
    track_c_writes = true;
    chg_init(m);
    m->sd = card;
    chg_reset(m, true);
    // the frame memory: power-on junk in both (gram_init.hex is st7735_power_on's)
    for (uint32_t i = 0x3000; i < 0x5000; i++) m->flash[i] = (uint8_t)rnd();
    memcpy(ram_rtl, m->ram, CHG_RAM_SIZE);
    start_rtl(true);
    T = 10;
    set_pins();

    // pins: PA4 push-pull, PB0, PB11, PB12 outputs; display out of reset, nothing selected
    w(GPIOA + 0x00, (m->gpio[0].cfg[0] & ~0x000F0000u) | 0x00030000u);
    w(GPIOA + 0x10, 1u << 4);
    w(GPIOB + 0x00, (m->gpio[1].cfg[0] & ~0x0000000Fu) | 0x00000003u);
    w(GPIOB + 0x04, (m->gpio[1].cfg[1] & ~0x000FF000u) | 0x00033000u);
    w(GPIOB + 0x10, (1u << 11) | (1u << 12));
    check("setup");

    long scene_left = 0;
    int scene = 0;
    while (ops_done < nops) {
        if (scene_left-- <= 0) {
            scene = rn(10) < 4 ? 0 : rn(10) < 7 ? 1 : 2;
            scene_left = 200 + rn(1500);
            q.clear();
            // stop any DMA, then set the scene up
            w(SPI + 0x04, 0);
            dma_setup(1, 0, 0, 0, 0);
            dma_setup(2, 0, 0, 0, 0);
            w(DMA + 0x04, 0xFFFFFFFF);
            if (scene == 0) {           // the display
                w(GPIOB + 0x10, 1u << 11);
                if (!rn(3)) w(GPIOA + 0x00, (m->gpio[0].cfg[0] & ~0x000F0000u) | (rn(2) ? 0x000B0000u : 0x00030000u));
                spi_config(false);
                if (!rn(2)) w(SPI + 0x04, 0x04);    // SSOE: PA4 as NSS when in AF mode
                w(GPIOA + 0x14, 1u << 4);
            } else if (scene == 1) {    // the card
                w(GPIOA + 0x00, (m->gpio[0].cfg[0] & ~0x000F0000u) | 0x00030000u);
                w(GPIOA + 0x10, 1u << 4);
                spi_config(true);
                if (!rn(4)) {           // power-up clocks with CS high
                    w(GPIOB + 0x10, 1u << 11);
                    for (int i = 0; i < 3; i++) { w(SPI + 0x0C, 0xFF); run(2000); }
                }
                w(GPIOB + 0x14, 1u << 11);
            } else {
                spi_config(rn(2));
                w(GPIOA + 0x00, (m->gpio[0].cfg[0] & ~0x000F0000u) | (rn(2) ? 0x000B0000u : 0x00030000u));
            }
        }
        ops_done++;
        T += rn(8) ? rn(24) : rn(8) ? rn(400) : rn(20000);
        const uint32_t k = rn(100);
        const bool sd = scene == 1;
        if (getenv("TBFROM") && ops_done >= atol(getenv("TBFROM"))) printf("start op %ld k %u q %zu\n", ops_done, k, q.size());

        if (k < 25) {
            sync_both();
        } else if (k < 50 && sd && rn(2)) {
            // a burst of bytes for the card, polled as SdSpi.cpp's xfer() does
            const uint32_t frame = 16u << ((m->spi.ctlr1 >> 3) & 7);
            for (int n = 1 + rn(200); n; n--) {
                at_T();
                spi_sync(m, T);
                if (!m->spi.tx_full) {
                    while (q.empty()) sd_script();
                    w(SPI + 0x0C, q.front(), 1);
                    q.erase(q.begin());
                    if (rn(2)) r(SPI + 0x0C, 1);
                } else rtl_sync();
                check("burst");
                T += 1 + rn(frame + 8);
            }
        } else if (k < 50) {
            // the program writes DATAR, as a polled driver would (mostly when TXE)
            at_T();
            spi_sync(m, T);
            const bool txe = !m->spi.tx_full;
            if (txe || !rn(4)) {
                uint32_t v;
                if (sd) {
                    while (q.empty()) sd_script();
                    v = q.front();
                    if (txe) q.erase(q.begin());
                } else v = rnd() & 0xffff;
                const int sz = rn(4) ? ((m->spi.ctlr1 & 0x800) ? 2 : 1) : 4;
                w(SPI + 0x0C, v, sz);
            } else rtl_sync();
        } else if (k < 60) {
            static const uint32_t offs[] = { 0x00, 0x04, 0x08, 0x08, 0x08, 0x0C, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20 };
            const uint32_t o = offs[rn(12)];
            const int sz = 1 << rn(3);
            r(SPI + o + (rn(4) ? 0 : rn(4) & ~(sz - 1)), sz);
        } else if (k < 64) {
            static const uint32_t offs[] = { 0x00, 0x04, 0x08, 0x0C, 0x1C, 0x1C, 0x30, 0x44, 0x80, 0xA4, 0x100 };
            const int sz = 1 << rn(3);
            r(DMA + offs[rn(11)] + (rn(4) ? 0 : rn(4) & ~(sz - 1)), sz);
        } else if (k < 68) {
            // GPIO: D/C, CS lines, now and then the display's reset or the pin modes
            const uint32_t which = rn(20);
            if (which < 8) w(GPIOB + (rn(2) ? 0x10 : 0x14), 1u << 0);
            else if (which < 10) w(GPIOA + (rn(2) ? 0x10 : 0x14), 1u << 4);
            else if (which < 12 && !sd) w(GPIOB + (rn(2) ? 0x10 : 0x14), 1u << 11);
            else if (which < 13) w(GPIOB + (rn(3) ? 0x10 : 0x14), 1u << 12);
            else if (which < 14) w(GPIOA + 0x00, (m->gpio[0].cfg[0] & ~0x000F0000u) | (rn(2) ? 0x000B0000u : 0x00030000u));
            else if (which < 15) w(GPIOB + 0x0C, m->gpio[1].outdr ^ (1u << 0));
            else w(GPIOA + 0x18, rnd());                        // LCKR: no pin changes
        } else if (k < 74) {
            // a CPU store into SRAM that the DMA may be reading
            at_T();
            const uint32_t off = 0x1000 + rn(0x3000);
            c_store_ram(off & ~3u, rnd(), 4);
        } else if (k < 82) {
            // DMA for SPI1
            at_T();
            const uint32_t sz = rn(3);
            const uint32_t cnt = rn(5) ? 1 + rn(sd ? 600 : 300) : rn(3);
            uint32_t cf = 0x0001 | (rn(2) ? 0x80 : 0) | (rn(4) ? 0 : 0x20) | (sz << 10) | (rn(3) << 8) |
                          (rn(2) ? 0x02 : 0) | (rn(3) ? 0 : 0x04) | (rn(4) << 12);
            if (rn(2)) {
                const uint32_t ma = sd && rn(2) ? 0x00003000u : rand_mem(cnt << sz);
                dma_setup(2, cf | 0x10, SPI + 0x0C, rn(8) ? ma & ~((1u << sz) - 1) : ma, cnt);
            } else {
                const uint32_t ma = CHG_RAM_BASE + 0x1000 + rn(0x3000 - (cnt << sz));
                dma_setup(1, cf, SPI + 0x0C, rn(8) ? ma & ~((1u << sz) - 1) : ma, cnt);
            }
            if (rn(2)) w(SPI + 0x04, (m->spi.ctlr2 & ~3u) | 1 | 2 | (rn(5) ? 0 : 0xC0));
            else w(SPI + 0x04, (m->spi.ctlr2 & ~3u) | rn(4) | (rn(5) ? 0 : (rn(2) ? 0x80 : 0x40)));
        } else if (k < 86) {
            // flags cleared, channels stopped, counts changed
            switch (rn(5)) {
            case 0: w(DMA + 0x04, rnd() & (rn(2) ? 0xFF0 : 0xFFFFFFFF)); break;
            case 1: w(DMA + 0x08 + (1 + rn(2)) * 0x14, rn(2) ? 0 : no_size8(rnd() & 0x3FFF)); break;
            case 2: w(DMA + 0x0C + (1 + rn(2)) * 0x14, rn(600)); break;
            case 3: w(DMA + 0x04, 0, 1 << rn(3)); break;
            default: w(DMA + 0x08 + rn(3) * 0x14 + 1, no_size8((rnd() & 0x3F) << 8) >> 8, 1); break;
            }
        } else if (k < 89) {
            // memory to memory, on any channel
            const int n = rn(8) == 0 ? 7 : rn(7);
            const uint32_t ps = rn(3), ms = rn(3), cnt = rn(20) ? 1 + rn(64) : 0;
            const uint32_t cf = 0x4001 | (ps << 8) | (ms << 10) | (rn(2) ? 0x40 : 0) | (rn(2) ? 0x80 : 0) |
                                (rn(2) ? 0x10 : 0) | (rn(2) ? 0x02 : 0);
            const bool dir = cf & 0x10;
            uint32_t src = rand_mem(cnt * 4), dst = CHG_RAM_BASE + 0x1000 + rn(0x3000 - cnt * 4);
            if (rn(6)) { src &= ~3u; dst &= ~3u; }
            if (dir) dma_setup(n, cf, dst, src, cnt);
            else dma_setup(n, cf, src, dst, cnt);
        } else if (k < 92) {
            // SPI1 itself reconfigured, sometimes in mid frame
            switch (rn(6)) {
            case 0: spi_config(sd); break;
            case 1: w(SPI + 0x00, m->spi.ctlr1 & ~0x40u); break;
            case 2: w(SPI + 0x00, m->spi.ctlr1 | 0x40u); break;
            case 3: w(SPI + 0x08, rnd() & 0xFFFF); break;
            case 4: w(SPI + 0x04, rnd() & 0xC7); break;
            default: w(SPI + 0x10 + rn(4) * 4, rnd(), 1 << rn(3)); break;
            }
        } else if (k < 95) {
            // long stretches of time
            run(rn(3000));
        } else if (k < 97 && scene == 1) {
            // CHSd's stream(): CMD18 then the card's blocks by DMA, 0xFF from flash
            w(SPI + 0x04, 0);                       // no circular DMA keeping TX full
            q.clear();
            sd_cmd(18, rn(4) ? rn(3000) : (uint32_t)sdcard_sectors(card) - 1);
            while (!q.empty()) {
                at_T(); spi_sync(m, T);
                if (!m->spi.tx_full) { w(SPI + 0x0C, q.front()); q.erase(q.begin()); }
                else rtl_sync();
                T += 1 + rn(30);
                check("stream cmd");
            }
            m->flash[0x3000] = 0xFF;
            const uint32_t buf = CHG_RAM_BASE + 0x1000 + rn(0x2000);
            for (int b = 0; b < 1 + (int)rn(3); b++) {
                // the token, polled
                for (int i = 0; i < 20; i++) { w(SPI + 0x0C, 0xFF); run(60); if (r(SPI + 0x0C) == 0xFE) break; }
                dma_setup(1, 0x3081, SPI + 0x0C, buf, 512);
                dma_setup(2, 0x2011, SPI + 0x0C, 0x00003000, 512);
                w(SPI + 0x04, 3);
                for (int i = 0; i < 400 && !(m->dma.intfr & 0x20); i++) { run(40); r(DMA + 0x00); }
                w(SPI + 0x04, 0);
                r(SPI + 0x0C);
            }
            q.clear();
            sd_cmd(12, 0);
        } else {
            sync_both();
        }
        check("op");
        if (getenv("TBV") || ops_done % 10000 == 0 || (getenv("TBFROM") && ops_done >= atol(getenv("TBFROM")))) printf("op %ld k %u scene %d T %llu ticks %llu\n", ops_done, k, scene, (unsigned long long)T, (unsigned long long)nticks);
        if (ops_done % 1000 == 0) compare_ram("op");
        if (ops_done % 5000 == 0) compare_glass("random");
    }
    run(5000);
    compare_ram("end");
    compare_glass("end");
    for (int i = 0; i < 5000 && hps_state; i++) tick();
    // every block of the two cards: the C's writes and the RTL's
    uint8_t a[512], b[512];
    long diff = 0;
    for (uint64_t l = 0; l < sdcard_sectors(card); l++) {
        sdcard_read(card, l, a, 1);
        sdcard_read(card_rtl, l, b, 1);
        if (memcmp(a, b, 512) && diff++ < 4) fail("card block %llu: the RTL's writes and the C's differ", (unsigned long long)l);
    }
    printf("cards compared: %llu blocks, %ld differ\n", (unsigned long long)sdcard_sectors(card), diff);
}

// ---------------------------------------------------------------------------
// game: the C runs the program one instruction at a time; the RTL follows
// ---------------------------------------------------------------------------

enum { K_NONE, K_LOAD, K_STORE, K_AMO, K_LR, K_SC };
static int opkind[128], opsize[128];
static bool opsigned[128];
static int op_amo[128];         // which AMO: 1 swap 2 add 3 xor 4 and 5 or 6 min 7 max 8 minu 9 maxu

static void init_ops()
{
    for (int op = 0; op < 128; op++) {
        const std::string n = cpu_op_name(op);
        if (n == "lb") { opkind[op] = K_LOAD; opsize[op] = 1; opsigned[op] = true; }
        else if (n == "lh") { opkind[op] = K_LOAD; opsize[op] = 2; opsigned[op] = true; }
        else if (n == "lw") { opkind[op] = K_LOAD; opsize[op] = 4; }
        else if (n == "lbu") { opkind[op] = K_LOAD; opsize[op] = 1; }
        else if (n == "lhu") { opkind[op] = K_LOAD; opsize[op] = 2; }
        else if (n == "sb") { opkind[op] = K_STORE; opsize[op] = 1; }
        else if (n == "sh") { opkind[op] = K_STORE; opsize[op] = 2; }
        else if (n == "sw") { opkind[op] = K_STORE; opsize[op] = 4; }
        else if (n == "lr.w") { opkind[op] = K_LR; opsize[op] = 4; }
        else if (n == "sc.w") { opkind[op] = K_SC; opsize[op] = 4; }
        else if (n.rfind("amo", 0) == 0) {
            static const char *names[] = { "amoswap.w", "amoadd.w", "amoxor.w", "amoand.w", "amoor.w",
                                           "amomin.w", "amomax.w", "amominu.w", "amomaxu.w" };
            for (int i = 0; i < 9; i++) if (n == names[i]) op_amo[op] = i + 1;
            opkind[op] = K_AMO; opsize[op] = 4;
        }
    }
}

struct Ev {
    int type;                   // 0 access (load() and store() never trap), 1 software reset
    uint64_t when;
    int kind, size;
    bool sgn;
    uint32_t pc, len, addr, val, rd;
    uint32_t old;               // AMO: the value it read
    bool sc_ok;
    int amo;
};
static std::vector<Ev> evs;

// the cycle cpu_exec will issue the instruction at pc at (its prefetch and divider waits)
static void predict(ChgMachine *mm)
{
    ChgCpu *c = &mm->cpu;
    const uint32_t pc = c->pc;
    ChgInsn d;
    uint32_t raw;
    const uint32_t off = pc - CHG_RAM_BASE;
    if (pc < CHG_FLASH_SIZE - 2) { memcpy(&raw, mm->flash + pc, 4); cpu_decode(pc, raw, &d, true); }
    else if (off < CHG_RAM_SIZE - 2) { memcpy(&raw, mm->ram + off, 4); cpu_decode(pc, raw, &d, false); }
    else {
        int wt = 0;
        raw = bus_read_slow(mm, pc, 2, &wt);
        if ((raw & 3) == 3) raw |= bus_read_slow(mm, pc + 2, 2, &wt) << 16;
        cpu_decode(pc, raw, &d, false);
    }
    const int kind = opkind[d.op];
    if (kind == K_NONE) return;
    const bool in_flash = pc < CHG_FLASH_SIZE;
    const uint32_t word = in_flash ? 4 : 1;
    uint32_t fw = c->fetch_word;
    uint64_t ft = c->fetch_time;
    uint64_t now = mm->cycles;
    while ((int32_t)(d.wend - fw) > 0) {
        fw++;
        ft += word;
        if (ft + word * (2 - 1) < now) ft = now - word * (2 - 1);
    }
    if (ft > now) now = ft;
    if (c->div_ready > now && (d.rs1 == c->div_rd || d.rs2 == c->div_rd)) now = c->div_ready;

    Ev e{};
    e.type = 0;
    e.when = now;
    e.kind = kind;
    e.size = opsize[d.op];
    e.sgn = opsigned[d.op];
    e.pc = pc;
    e.len = d.len;
    e.rd = d.rd;
    const uint32_t a = c->x[d.rs1], b = c->x[d.rs2];
    e.addr = (kind == K_LOAD || kind == K_STORE) ? a + (uint32_t)d.imm : a;
    e.val = b;
    e.amo = op_amo[d.op];
    e.sc_ok = kind == K_SC && c->lr_valid && c->lr_addr == a;
    if (kind == K_AMO && e.addr - CHG_RAM_BASE <= CHG_RAM_SIZE - 4) memcpy(&e.old, mm->ram + (e.addr - CHG_RAM_BASE), 4);
    evs.push_back(e);
}

// chg_run(m, m->cycles + 1), line for line, with predict() before cpu_exec
static void step(ChgMachine *mm)
{
    const uint64_t until = mm->cycles + 1;
    while (mm->cycles < until) {
        if (mm->reset_request) {
            chg_reset(mm, false);
            Ev e{};
            e.type = 1;
            e.when = mm->cycles;
            evs.push_back(e);
            continue;
        }
        if (mm->cycles >= mm->next_event)
            bus_events(mm);
        cpu_take_interrupt(mm);
        chg_schedule(mm);

        uint64_t stop = until < mm->next_event ? until : mm->next_event;
        if (stop <= mm->cycles)
            stop = mm->cycles + 1;
        if (mm->cpu.wfi) {
            if (cpu_irq_waiting(mm) && (mm->cpu.mstatus & 8)) {
                mm->cpu.wfi = false;
                continue;
            }
            mm->cycles = stop > mm->cycles ? stop : mm->cycles;
            if (cpu_irq_waiting(mm))
                mm->cpu.wfi = false;
            continue;
        }
        mm->stop_at = stop;
        predict(mm);
        cpu_exec(mm);
    }
    spi_sync(mm, mm->cycles);
}

static long accesses_spi, accesses_dma, accesses_gpio, ram_loads, ram_stores, steps;

static bool is_spi(uint32_t a) { return (a & ~0x3ffu) == SPI; }
static bool is_dma(uint32_t a) { return (a & ~0x3ffu) == DMA; }
static bool is_gpio_port(uint32_t a) { return a >= 0x40010800u && a < 0x40011400u; }
static bool is_ram(uint32_t a, int n) { return a - CHG_RAM_BASE <= CHG_RAM_SIZE - (uint32_t)n; }

static uint32_t mask(int n) { return n == 4 ? 0xffffffffu : (1u << (8 * n)) - 1; }

// what the instruction did, at its issue cycle, on the RTL side
static void replay(const Ev &e, uint32_t got)
{
    advance_to(e.when);
    if (e.type == 1) {          // chg_reset(m, false)
        rtl_reset(false);
        lcd_reset();
        set_pins();
        return;
    }
    const uint32_t a = e.addr;
    const int n = e.size;
    const bool loads = e.kind == K_LOAD || e.kind == K_LR || e.kind == K_AMO;
    bool stores = e.kind == K_STORE || e.kind == K_AMO || (e.kind == K_SC && e.sc_ok);

    if (is_ram(a, n)) {
        const uint32_t off = a - CHG_RAM_BASE;
        if (loads) {
            uint32_t v = 0;
            memcpy(&v, ram_rtl + off, n);
            ram_loads++;
            if ((v & mask(n)) != (got & mask(n)))
                fail("load %08x/%d at pc %08x: rtl SRAM %08x, the C CPU got %08x", a, n, e.pc, v, got);
        }
        if (stores) {
            uint32_t v = e.val;
            if (e.kind == K_AMO) {
                const uint32_t o = e.old, b = e.val;
                switch (e.amo) {
                case 2: v = o + b; break;
                case 3: v = o ^ b; break;
                case 4: v = o & b; break;
                case 5: v = o | b; break;
                case 6: v = (int32_t)o < (int32_t)b ? o : b; break;
                case 7: v = (int32_t)o > (int32_t)b ? o : b; break;
                case 8: v = o < b ? o : b; break;
                case 9: v = o > b ? o : b; break;
                default: v = b; break;
                }
            }
            memcpy(ram_rtl + off, &v, n);
            ram_stores++;
        }
        return;
    }
    if (is_spi(a) || is_dma(a)) {
        if (is_spi(a)) accesses_spi++; else accesses_dma++;
        if (loads) {
            const uint32_t v = rtl_access(false, a, n, 0);
            reg_reads++;
            if ((v & mask(n)) != (got & mask(n)))
                fail("load %08x/%d at pc %08x: rtl %08x, the C CPU got %08x", a, n, e.pc, v, got);
        }
        if (stores) rtl_access(true, a, n, e.val);
        set_pins();
        return;
    }
    if (stores && is_gpio_port(a)) {
        accesses_gpio++;
        rtl_gpio_wr();
    }
    set_pins();
}

static void game(const char *chg, double seconds, const char *folder)
{
    char err[256];
    init_ops();
    if (folder) card = open_card(folder);
    chg_init(m);
    m->sd = card;
    if (!chg_load_program(m, chg, err, sizeof err)) { printf("%s\n", err); exit(2); }
    chg_reset(m, true);
    // the frame memory as power-on leaves it: the C fills it (st7735_power_on), st7735.sv starts from gram_init.hex, the same
    memcpy(ram_rtl, m->ram, CHG_RAM_SIZE);
    start_rtl(true);
    set_pins();
    advance_to(m->cycles);

    const int frames = (int)(seconds * 60);
    int bad_frames = 0;
    for (int f = 1; f <= frames; f++) {
        const uint8_t btn = (f / 60) % 3 == 1 ? (CHG_BTN_START | CHG_BTN_A) : (f / 60) % 3 == 2 ? CHG_BTN_RIGHT : 0;
        chg_set_buttons(m, btn);
        const uint64_t until = (uint64_t)f * (CHG_HCLK / 60);
        while (m->cycles < until) {
            evs.clear();
            step(m);
            steps++;
            for (Ev &e : evs) {
                uint32_t got = 0;
                if (e.type == 0 && (e.kind == K_LOAD || e.kind == K_LR || e.kind == K_AMO)) {
                    const uint32_t rd = e.rd;
                    got = m->cpu.x[rd];
                }
                replay(e, got);
            }
            advance_to(m->cycles);
            rtl_sync();
            compare_state("step");
            compare_dirty("step");
            if (errors && err_shown > 30) break;
        }
        compare_ram("frame");
        if (compare_glass("frame")) bad_frames++;
        if (f % 60 == 0)
            printf("  %s: %d s, %llu cycles, %ld steps, %ld differences\n", chg, f / 60,
                   (unsigned long long)m->cycles, steps, errors);
        if (errors && err_shown > 30) break;
    }
    printf("game %s: %d frames (%.1f s board time), %ld instructions, %ld SPI and %ld DMA register accesses,"
           " %ld GPIO port writes, %ld SRAM loads and %ld stores mirrored, %ld state checks, %ld pictures,"
           " %d pictures different (%ld not one colour, %u changes), %ld blocks read, %ld written, %ld differences\n",
           chg, frames, frames / 60.0, steps, accesses_spi, accesses_dma, accesses_gpio, ram_loads, ram_stores,
           state_checks, glass_checks, bad_frames, glass_content, picture_changes, blocks_read, blocks_written, errors);
}

// ---------------------------------------------------------------------------
// latency: hold_mode 0, hps_io late; the RTL alone, driven like SdSpi.cpp
// ---------------------------------------------------------------------------

static uint8_t xf(uint8_t b)
{
    rtl_access(true, SPI + 0x0C, 1, b);
    for (int i = 0; i < 100; i++) {
        if (rtl_access(false, SPI + 0x08, 4, 0) & 1) break;
        top->ce = 1; tick(); top->ce = 0;
    }
    return (uint8_t)rtl_access(false, SPI + 0x0C, 4, 0);
}
static uint8_t lcmd(uint8_t c, uint32_t arg)
{
    xf(0xFF); xf(0x40 | c);
    for (int sh = 24; sh >= 0; sh -= 8) xf((uint8_t)(arg >> sh));
    xf(0x95);
    uint8_t r1;
    int k = 9;
    do r1 = xf(0xFF); while ((r1 & 0x80) && --k);
    return r1;
}
static long ff_waits;
static uint8_t ltoken()
{
    uint8_t b;
    int k = 0;
    while ((b = xf(0xFF)) == 0xFF && k++ < 200000) ff_waits++;
    return b;
}

static void latency_test(const char *folder)
{
    card = open_card(folder);
    chg_init(m);
    hps_late = true;
    hps_delay = 20000;
    start_rtl(false);
    top->pa4_af = 0; top->pa4_gpio = 1; top->pb11 = 0; top->lcd_rst_level = 1;
    rtl_access(true, SPI + 0x00, 2, 0x0344);
    uint8_t buf[512], want[512];
    if (lcmd(0, 0) != 1) fail("latency: CMD0");
    lcmd(55, 0);
    if (lcmd(41, 1u << 30) != 0) fail("latency: ACMD41");
    rng = 99;
    int checked = 0;
    for (int i = 0; i < 12; i++) {
        const uint32_t lba = rn(3000);
        if (lcmd(17, lba) != 0) { fail("latency: CMD17 R1"); continue; }
        if (ltoken() != 0xFE) { fail("latency: no token for %u", lba); continue; }
        for (int j = 0; j < 512; j++) buf[j] = xf(0xFF);
        xf(0xFF); xf(0xFF);
        sdcard_read(card, lba, want, 1);
        if (memcmp(buf, want, 512)) fail("latency: CMD17 block %u differs", lba); else checked++;
    }
    // CMD18, 4 blocks
    const uint32_t l0 = 500;
    lcmd(18, l0);
    for (int b = 0; b < 4; b++) {
        if (ltoken() != 0xFE) { fail("latency: CMD18 token %d", b); break; }
        for (int j = 0; j < 512; j++) buf[j] = xf(0xFF);
        xf(0xFF); xf(0xFF);
        sdcard_read(card, l0 + b, want, 1);
        if (memcmp(buf, want, 512)) fail("latency: CMD18 block %u differs", l0 + b); else checked++;
    }
    xf(0x4C); for (int i = 0; i < 4; i++) xf(0); xf(0x61);
    for (int i = 0; i < 12; i++) xf(0xFF);
    // CMD24 then read it back
    for (int j = 0; j < 512; j++) want[j] = (uint8_t)rnd();
    if (lcmd(24, 120) != 0) fail("latency: CMD24 R1");
    xf(0xFF); xf(0xFE);
    for (int j = 0; j < 512; j++) xf(want[j]);
    xf(0xFF); xf(0xFF);
    const uint8_t resp = xf(0xFF);
    if ((resp & 0x1F) != 0x05) fail("latency: data response %02x", resp);
    long busy = 0;
    while (xf(0xFF) == 0x00) busy++;
    lcmd(17, 120);
    if (ltoken() != 0xFE) fail("latency: no token after write");
    for (int j = 0; j < 512; j++) buf[j] = xf(0xFF);
    if (memcmp(buf, want, 512)) fail("latency: block written then read differs"); else checked++;
    printf("latency: hps_io %d clocks late, %d blocks right, %ld 0xFF bytes before tokens, %ld busy bytes after"
           " the write, %ld differences\n", hps_delay, checked, ff_waits, busy, errors);
}

int main(int argc, char **argv)
{
    if (argc < 2) { printf("tb_spi random <ops> <seed> <card> | game <chg> <seconds> [card] | latency <card>\n"); return 2; }
    setvbuf(stdout, nullptr, _IONBF, 0);
    const std::string mode = argv[1];
    if (mode == "random") {
        card = open_card(argv[4]);
        card_rtl = open_card(argv[4]);
        random_test(atol(argv[2]), strtoul(argv[3], 0, 0));
        printf("random: %ld ops (seed %s), %llu cycles, %ld register reads compared, %ld state checks,"
               " %ld pictures, %ld blocks read and %ld written by hps_io, %ld differences\n",
               ops_done, argv[3], (unsigned long long)T, reg_reads, state_checks, glass_checks,
               blocks_read, blocks_written, errors);
    } else if (mode == "game") {
        game(argv[2], atof(argv[3]), argc > 4 ? argv[4] : nullptr);
    } else if (mode == "speed") {
        start_rtl(true);
        for (int i = 0; i < 1000000; i++) { top->ce = i & 1; tick(); }
    } else if (mode == "latency") {
        latency_test(argv[2]);
    }
    printf("ticks %llu\n", (unsigned long long)nticks);
    delete top;
    return errors ? 1 : 0;
}
