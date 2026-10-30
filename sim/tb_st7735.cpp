// st7735.sv against emu/src/st7735.c: the same random command streams into
// both, the 128x128 glass compared after every burst.
//   sim/out/st7735/st7735.exe [bursts] [seed]
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include "Vst7735.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
}

static Vst7735 *top;
static ChgSt7735 lcd;
static uint32_t rng = 1;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static void tick() { top->clk = 0; top->eval(); top->clk = 1; top->eval(); }

static void send(bool dc, uint8_t b)
{
    st7735_byte(&lcd, dc, b);
    top->byte_stb = 1; top->dc = dc; top->byte_in = b;
    tick();
    top->byte_stb = 0;
    tick();
}
static void cmd(uint8_t c) { send(false, c); }
static void par16(uint16_t v) { send(true, v >> 8); send(true, v & 0xff); }

static int compare(int burst)
{
    static uint32_t want[128 * 128];
    st7735_render(&lcd, want);
    int bad = 0;
    for (int y = 0; y < 128; y++) {
        top->vy = y; top->vline = 1; tick(); top->vline = 0; for (int k = 0; k < 6; k++) tick();
        for (int x = 0; x < 128; x++) {
            top->vx = x;
            tick(); tick(); tick();
            uint32_t got = top->vrgb;
            if (got != want[y * 128 + x] && bad++ < 5)
                printf("burst %d: (%d,%d) rtl %06x c %06x  madctl %02x colmod %02x part %d scr %d inv %d idle %d\n",
                       burst, x, y, got, want[y * 128 + x], lcd.madctl, lcd.colmod, lcd.partial,
                       lcd.scrolling, lcd.inverted, lcd.idle);
        }
    }
    return bad;
}

int main(int argc, char **argv)
{
    int bursts = argc > 1 ? atoi(argv[1]) : 300;
    rng = argc > 2 ? strtoul(argv[2], 0, 0) : 12345;
    top = new Vst7735;
    // both start with st7735_power_on's pattern
    st7735_power_on(&lcd);
    lcd.rst_level = true;
    top->rst_level = 1;
    top->reset = 1; tick(); tick(); top->reset = 0; tick();

    int fails = 0;
    for (int b = 0; b < bursts; b++) {
        int n = 1 + rnd() % 12;
        for (int i = 0; i < n; i++) {
            switch (rnd() % 16) {
            case 0: cmd(0x11); break;
            case 1: cmd(0x29); break;
            case 2: cmd(0x36); send(true, (rnd() & 1) ? 0xC8 : rnd() & 0xF8); break;
            case 3: { static const uint8_t m[] = { 3, 5, 6, 5, 5 }; cmd(0x3A); send(true, m[rnd() % 5]); break; }
            case 4: case 5: case 6: {           // a window and pixels into it
                uint16_t x0 = rnd() % 140, y0 = rnd() % 170;
                cmd(0x2A); par16(x0); par16(x0 + rnd() % 40);
                cmd(0x2B); par16(y0); par16(y0 + rnd() % 40);
                cmd(0x2C);
                int k = rnd() % 3000;
                for (int j = 0; j < k; j++) send(true, rnd());
                break;
            }
            case 7: cmd(rnd() & 1 ? 0x21 : 0x20); break;
            case 8: cmd(rnd() % 4 ? 0x38 : 0x39); break;
            case 9: cmd(0x30); par16(rnd() % 140); par16(rnd() % 140); cmd(0x12); break;
            case 10: cmd(0x13); break;
            case 11: cmd(0x33); par16(rnd() % 40); par16(rnd() % 170); par16(rnd() % 40);
                     cmd(0x37); par16(rnd() % 140); break;
            case 12: if (rnd() % 8 == 0) cmd(0x01); break;
            case 13: if (rnd() % 8 == 0) cmd(0x10); else cmd(0x11); break;
            case 14: if (rnd() % 8 == 0) cmd(0x28); else cmd(0x29); break;
            case 15: {                          // the games' own full-screen frame
                cmd(0x36); send(true, 0xC8); cmd(0x3A); send(true, 5);
                cmd(0x2A); par16(2); par16(129); cmd(0x2B); par16(3); par16(130); cmd(0x2C);
                for (int j = 0; j < 128 * 128 * 2; j++) send(true, rnd());
                break;
            }
            }
        }
        if (compare(b)) fails++;
    }
    printf("st7735: %d bursts, %d with differences\n", bursts, fails);
    delete top;
    return fails ? 1 : 0;
}
