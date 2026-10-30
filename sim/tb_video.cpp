// chg_video.sv with st7735.sv and chg_hfine (a wire in modes 0-2): a picture
// drawn the way the games draw it, then in each Analogue mode (1x, 2x, 3x, the
// fine modes 1.75x and 240x240 with each Scale filter, and 1x again after the
// others) and at V positions -15, 0 and +15, two whole frames captured from
// the video output. The timing must be what chg_video.sv says (clocks a
// pixel, pixels a line, lines a frame, sync widths and places, the visible
// size), and every visible pixel must be st7735_render's glass pixel at
// (x / S, row0 + y / S), or in the fine modes the pixel the rows/lines table
// and the filter give (computed here, in integers, from the glass frame).
// Pictures of the fine modes are written to the directory given as argv[1].
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <string>
#include "Vvideo_top.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
}
static Vvideo_top *top;
static ChgSt7735 lcd;
static long clocks;
static void tick() { top->clk = 0; top->eval(); top->clk = 1; top->eval(); clocks++; }
static void send(bool dc, uint8_t b)
{
    st7735_byte(&lcd, dc, b);
    top->byte_stb = 1; top->dc = dc; top->byte_in = b; tick(); top->byte_stb = 0; tick();
}

struct Mode {
    const char *name;
    int clk, htotal, vtotal, hsync0, hsync1, vsync_lines, vis_w, vis_h, vstart, scale, row0;
    int T, g, gl;          // fine modes: ticks a glass pixel; glass rows and lines a group (the group's last row has one line)
    int ph;                // lines of picture
};
// hsync0/hsync1: the sync's first pixel and the one after it, counted from the
// line's first visible pixel; vstart: lines from the start of vsync to the
// first visible line
static const Mode modes[] = {
    { "15 kHz 1x", 16, 384, 262, 231, 259, 3, 128, 128, 75, 1, 0, 0, 0, 0, 128 },
    { "15 kHz 2x", 16, 384, 262, 294, 322, 3, 256, 240, 19, 2, 4, 0, 0, 0, 256 },
    { "31 kHz 3x",  5, 610, 525, 448, 521, 2, 384, 384, 83, 3, 0, 0, 0, 0, 384 },
    // the fine modes, in 48 MHz ticks: 1.75x (mode 3) and 240x240 (mode 4)
    { "15 kHz 1.75x",   2, 3072, 262, 2224, 2448, 3, 1792, 224, 27, 1, 0, 14, 4, 7, 224 },
    { "15 kHz 240x240", 2, 3072, 262, 2288, 2512, 3, 1920, 240, 19, 1, 0, 15, 8, 15, 240 },
};
static int filt_sel;         // the Scale filter being tested (fine modes)

static uint32_t want[128 * 128];
static int hidx;      // hoff_in; hsh: the pixels the sync moves earlier (right shift) or later (negative)
static int hshift(int m, int h) { int s = m == 2 || h > 40 ? 0 : h > 20 ? -(41 - h) : h; return m >= 3 ? s * 8 : s; }

// the speed display, written out again here (not copied from the RTL): glass
// columns 111-127, rows 13-19, black, with the text's 3x5 glyphs in white from
// column 112 and row 5, four columns a character; ovl_text is what is shown
static bool ovl_on;
static const char *ovl_text = "100%";
static const char *font[][5] = {
    {"###", "# #", "# #", "# #", "###"}, {" # ", "## ", " # ", " # ", "###"}, {"###", "  #", "###", "#  ", "###"},
    {"###", "  #", "###", "  #", "###"}, {"# #", "# #", "###", "  #", "  #"}, {"###", "#  ", "###", "  #", "###"},
    {"###", "#  ", "###", "# #", "###"}, {"###", "  #", "  #", "  #", "  #"}, {"###", "# #", "###", "# #", "###"},
    {"###", "# #", "###", "  #", "###"}, {"# #", "  #", " # ", "#  ", "# #"} };
// the overlay's colour at glass (gx, gy), or -1 if there is none
static int64_t ovl_at(int gx, int gy)
{
    if (!ovl_on || gx < 111 || gx > 127 || gy < 13 || gy > 19) return -1;
    int tx = gx - 112, ty = gy - 14;
    if (tx < 0 || ty < 0 || ty > 4 || tx % 4 == 3) return 0;
    char c = ovl_text[tx / 4];
    if (c == ' ') return 0;
    int idx = c == '%' ? 10 : c - '0';
    return font[idx][ty][tx % 4] == '#' ? 0xffffff : 0;
}

// the spectrum analyser, written out again here: seven bars of 3 columns and a gap, from column 50
// bottom at glass row 105, 15 rows at most, on a black box of columns 49..77, rows 89-106;
// green from the bottom, yellow from row 8 up, red from 12 up, and a white mark at the peak when it is above the bar
static int spec_mode_v;
static int sbar[7], spk[7];
static int64_t spec_at(int gx, int gy)
{
    if (!spec_mode_v) return -1;
    int x0 = 50;
    if (gx < x0 - 1 || gx > x0 + 27 || gy < 89 || gy > 106) return -1;
    int dx = gx - x0, r = 105 - gy;
    if (dx < 0 || dx >= 27 || dx % 4 == 3 || r < 0 || r > 15) return 0;
    int b = dx / 4;
    if (spec_mode_v == 2 && spk[b] > sbar[b] && r == spk[b] - 1) return 0xffffff;
    if (r < sbar[b]) return r >= 12 ? 0xff3020 : r >= 8 ? 0xf0d020 : 0x20d030;
    return 0;
}

// the glass frame's pixel mixed: (8 - w) / 8 of a and w / 8 of b, rounded half up, per channel
static uint32_t mix(uint32_t a, uint32_t b, int w)
{
    uint32_t o = 0;
    for (int sh = 16; sh >= 0; sh -= 8) {
        int ca = a >> sh & 255, cb = b >> sh & 255;
        o |= (uint32_t)(((8 - w) * ca + w * cb + 4) >> 3) << sh;
    }
    return o;
}
// 3 : 1 mix (25% of the neighbour n), rounded half up
static uint32_t quarter(uint32_t c, uint32_t n)
{
    uint32_t o = 0;
    for (int sh = 16; sh >= 0; sh -= 8) o |= (uint32_t)((3 * (c >> sh & 255) + (n >> sh & 255) + 2) >> 2) << sh;
    return o;
}
// the fine modes' picture at glass column gx on line y: the row the table gives, mixed with the one
// before or after as the filter says; the overlays on top unfiltered; -1 for black (no picture)
static int64_t vexp(const Mode &M, int off, int filt, int gx, int y)
{
    int j = y - (M.vstart + off);
    if (j < 0 || j >= M.ph) return -1;
    int q = j / M.gl, rm = j % M.gl;
    bool last = rm == M.gl - 1;
    int row = q * M.g + (last ? M.g - 1 : rm / 2), rep = last ? 0 : rm & 1, k = row % M.g;
    int64_t o = ovl_at(gx, row);
    if (o < 0) o = spec_at(gx, row);
    if (o >= 0) return o;
    uint32_t c = want[row * 128 + gx];
    if (filt == 1 && k == M.g - 1) return mix(want[(row - 1) * 128 + gx], c, 4);
    if (filt >= 2 && rep == 1) return mix(c, want[(row + 1) * 128 + gx], (k + 1) * (8 / M.g));
    return c;
}
static int64_t fexp(const Mode &M, int off, int filt, int x, int y)
{
    int gx = x / M.T, jx = x % M.T;
    int64_t c = vexp(M, off, filt, gx, y);
    if (c < 0 || filt != 3) return c;
    if (jx < 4 && gx > 0) return quarter((uint32_t)c, (uint32_t)vexp(M, off, filt, gx - 1, y));
    if (jx >= M.T - 4 && gx < 127) return quarter((uint32_t)c, (uint32_t)vexp(M, off, filt, gx + 1, y));
    return c;
}

// what a frame should show at line y (from vsync), pixel x of the window:
// the glass pixel, or -1 for black (picture moved off), with off lines down.
// In 1x the window moves with the picture; in 2x it stays and the picture
// (all 256 lines, rows 0-3 above the window at offset 0) moves inside it.
static int64_t expect(const Mode &M, int off, int x, int y)
{
    if (M.T) return fexp(M, off, filt_sel, x, y);
    int pst = M.vstart - M.row0 * M.scale + off;     // the line of glass row 0
    int gy = y - pst;
    if (gy < 0 || gy >= 128 * M.scale) return -1;
    int64_t o = ovl_at(x / M.scale, gy / M.scale);
    if (o < 0) o = spec_at(x / M.scale, gy / M.scale);
    return o >= 0 ? o : want[(gy / M.scale) * 128 + x / M.scale];
}

// the visible window of the last frame checked at each mode and offset index
enum { SW = 1920, SH = 528 };
static std::vector<uint32_t> shot[5][31];
static uint32_t *shotp(int m, int idx) { auto &v = shot[m][idx]; if (v.empty()) v.resize(SW * SH); return v.data(); }

// one mode and offset index (0-15 down, 16-30 up 15-1): wait for them to
// start, then check two frames; returns the faults. A frame starts at the
// rising edge of vsync, which is also a line's first pixel, so pixel p of
// the frame is at (p % htotal, p / htotal), x counted from the line's first
// visible pixel and y from vsync.
static int check(int m, int idx)
{
    Mode M = modes[m];
    int hsh = hshift(m, hidx);
    M.hsync0 -= hsh; M.hsync1 -= hsh;
    int sel = idx < 16 ? idx : idx - 31;
    int off = m == 2 ? 0 : sel;                       // 3x has no offset
    int wtop = M.vstart + (m == 0 ? off : 0);         // the window's first line
    char name[64];
    if (M.T) snprintf(name, sizeof name, "%s f%d %+d h%d", M.name, filt_sel, sel, hidx);
    else snprintf(name, sizeof name, "%s %+d h%d", M.name, sel, hidx);
    top->mode_in = m; top->filt_in = filt_sel; top->voff_in = idx; top->hoff_in = hidx;
    top->ovl_en = ovl_on;
    long guard = 0;
    while (top->mode != m) { tick(); if (++guard > 20000000) { printf("%s: never started\n", name); return 1; } }
    // two vsyncs: the frame that takes the new values, then the start of one shown with them
    int last_vs = top->vs;
    for (int n = 0; n < 2; ) { tick(); if (top->ce_pix) { if (top->vs && !last_vs) n++; last_vs = top->vs; } }

    uint32_t *img = shotp(m, idx);
    memset(img, 0, SW * SH * 4);
    int faults = 0, bad = 0, frames = 0;
    std::vector<int> badline(M.vtotal + 1, 0);
    long p = 0, last_ce = clocks, clk0 = clocks;
    // measured from the signals alone
    int hs_run = 0, hs_w = -1, vs_run = 0, vs_w = -1, de_run = 0, de_w = -1, de_lines = 0, last_hs = 0, hs_rises = 0;
    int first_de_line = -1, first_de_x = -1;
    last_vs = 0;
    for (;; tick()) {
        if (!top->ce_pix) continue;
        if (p > 0 && clocks - last_ce != M.clk && faults++ < 5) printf("%s: a pixel of %ld clocks\n", name, clocks - last_ce);
        last_ce = clocks;
        if (top->vs && !last_vs && p > 0) {
            long fclk = clocks - clk0;
            printf("%s frame %d: %ld pixels, %d lines (hsyncs), %ld clocks = %.3f kHz x %.2f Hz;"
                   " hsync %d px, vsync %.1f lines; window %d x %d from (%d, %d)\n",
                   name, frames, p, hs_rises, fclk, 96e6 / ((double)M.clk * M.htotal) / 1000, 96e6 / fclk,
                   hs_w, (double)vs_w / M.htotal, de_w, de_lines, first_de_x, first_de_line);
            if (p != (long)M.htotal * M.vtotal || hs_rises != M.vtotal || hs_w != M.hsync1 - M.hsync0
                || vs_w != M.vsync_lines * M.htotal || de_w != M.vis_w || de_lines != M.vis_h
                || first_de_line != wtop || first_de_x != 0) { printf("%s: not the table's timing\n", name); faults++; }
            if (++frames == 2) break;
            p = 0; clk0 = clocks; hs_rises = 0; de_lines = 0; first_de_line = -1; vs_run = 0;
        }
        int x = (int)(p % M.htotal), y = (int)(p / M.htotal);
        int de = !(top->hblank || top->vblank);
        // against the table, pixel by pixel
        int hs_t = x >= M.hsync0 && x < M.hsync1, vs_t = y < M.vsync_lines;
        int de_t = x < M.vis_w && y >= wtop && y < wtop + M.vis_h;
        if ((top->hs != hs_t || top->vs != vs_t || de != de_t) && faults++ < 5)
            printf("%s (%d,%d): hs %d vs %d de %d, the table says %d %d %d\n", name, x, y, top->hs, top->vs, de, hs_t, vs_t, de_t);
        // from the signals
        if (top->hs) hs_run++; else { if (hs_run) hs_w = hs_run; hs_run = 0; }
        if (top->hs && !last_hs) hs_rises++;
        last_hs = top->hs;
        if (top->vs) vs_run++; else { if (vs_run) vs_w = vs_run; vs_run = 0; }
        last_vs = top->vs;
        uint32_t got = (uint32_t)top->r << 16 | top->g << 8 | top->b;
        if (de) {
            if (first_de_line < 0) { first_de_line = y; first_de_x = x; }
            de_run++;
            int64_t w = expect(M, off, x, y);
            if (got != (w < 0 ? 0u : (uint32_t)w)) badline[y]++;
            if (got != (w < 0 ? 0u : (uint32_t)w) && bad++ < 5)
                printf("%s (%d,%d): video %06x, expected %06llx\n", name, x, y, got, (long long)w);
            img[y * SW + x] = got;
        }
        else {
            if (got && faults++ < 5) printf("%s (%d,%d): %06x outside the window\n", name, x, y, got);
            if (de_run) { de_w = de_run; de_run = 0; de_lines++; }
        }
        p++;
    }
    if (bad) { printf("%s: lines with wrong pixels (line from vsync: count):", name); int n = 0; for (int y = 0; y < M.vtotal && n < 40; y++) if (badline[y]) { printf(" %d:%d", y, badline[y]); n++; } printf("\n"); }
    if (M.T) {
        printf("%s: window %d x %d ticks (%d glass pixels of %d), picture on lines %d-%d of the window; %d pixels differ\n\n",
               name, M.vis_w, M.vis_h, 128, M.T, off > 0 ? off : 0, M.ph - 1 + (off > 0 ? off : 0) > M.vis_h - 1 ? M.vis_h - 1 : M.ph - 1 + (off > 0 ? off : 0), bad);
        return faults + bad;
    }
    // the picture's first and last lines in the window, and the glass rows they show
    int pst = M.vstart - M.row0 * M.scale + off;
    int a = pst > wtop ? pst : wtop;
    int b = pst + 128 * M.scale < wtop + M.vis_h ? pst + 128 * M.scale : wtop + M.vis_h;
    int ra = (a - pst) / M.scale, rb = (b - 1 - pst) / M.scale;
    uint32_t f = img[a * SW], l = img[(b - 1) * SW + M.vis_w - 1];
    uint32_t wf = want[ra * 128], wl = want[rb * 128 + 127];
    printf("%s: picture on lines %d-%d (glass rows %d-%d), black lines in the window %d above it, %d below;"
           " first pixel %06x (glass %06x), last %06x (glass %06x); %d pixels differ\n\n",
           name, a, b - 1, ra, rb, a - wtop, wtop + M.vis_h - b, f, wf, l, wl, bad);
    if (f != wf || l != wl) faults++;
    return faults + bad;
}

// the frame at offset index idx against the one at 0: every window line y is
// line y - off of that frame, or black where that is not picture
static int shifted(int m, int idx)
{
    const Mode &M = modes[m];
    int off = idx < 16 ? idx : idx - 31, wtop = M.vstart + (m == 0 ? off : 0), wtop0 = M.vstart;
    int pst0 = M.T ? M.vstart : M.vstart - M.row0 * M.scale, ph = M.T ? M.ph : 128 * M.scale, bad = 0, black = 0, cut0 = 0;
    const uint32_t *s1 = shotp(m, idx), *s0 = shotp(m, 0);
    for (int y = wtop; y < wtop + M.vis_h; y++) {
        int y0 = y - off;
        bool pic = y0 - pst0 >= 0 && y0 - pst0 < ph;              // picture there at offset 0
        bool in0 = y0 >= wtop0 && y0 < wtop0 + M.vis_h;             // and inside its window then
        if (!pic) black++;
        else if (!in0) cut0++;
        for (int x = 0; x < M.vis_w; x++) {
            uint32_t g = s1[y * SW + x];
            if (!pic) { if (g) bad++; }
            else if (in0) { if (g != s0[y0 * SW + x]) bad++; }
            else if (g != (uint32_t)expect(M, 0, x, y0)) bad++;     // cut off at 0, shown now
        }
    }
    printf("%s %+d against 0: every line moved by exactly %+d: %s (%d black lines, %d lines cut off at 0 now shown,"
           " %d pixels differ)\n", M.name, off, off, bad ? "NO" : "yes", black, cut0, bad);
    return bad;
}

static std::vector<uint32_t> base;

// a picture of the last frame checked at (m, idx): the window with square pixels, 4 columns and 4 rows
// to a 6 MHz pixel (a fine mode's tick is 1/8 of one: every second tick; 1x/2x: each pixel 4 times), as a PPM
static void ppm(const std::string &dir, const std::string &name, int m, int idx)
{
    const Mode &M = modes[m];
    const uint32_t *img = shotp(m, idx);
    int y0 = M.vstart, w = M.T ? M.vis_w / 2 : M.vis_w * 4, h = M.vis_h * 4;
    std::string path = dir + "/" + name + ".ppm";
    FILE *fp = fopen(path.c_str(), "wb");
    if (!fp) { printf("cannot write %s\n", path.c_str()); return; }
    fprintf(fp, "P6\n%d %d\n255\n", w, h);
    for (int y = 0; y < M.vis_h; y++)
        for (int rep = 0; rep < 4; rep++)
            for (int x = 0; x < w; x++) {
                uint32_t p = img[(y0 + y) * SW + (M.T ? 2 * x : x / 4)];
                fputc(p >> 16 & 255, fp); fputc(p >> 8 & 255, fp); fputc(p & 255, fp);
            }
    fclose(fp);
}

// hoff_in changed in the middle of a frame (from 0 to hv): the rest of that
// frame keeps the sync where it was, and the next frame has it moved. Hsync
// start x is counted from vsync's rising edge as in check().
static int midframe(int m, int hv)
{
    const Mode &M = modes[m];
    int sh = hshift(m, hv), faults = 0;
    hidx = 0; top->hoff_in = 0; top->mode_in = m; top->voff_in = 0;
    long guard = 0;
    while (top->mode != m) { tick(); if (++guard > 20000000) return 1; }
    int last_vs = top->vs;
    for (int n = 0; n < 2; ) { tick(); if (top->ce_pix) { if (top->vs && !last_vs) n++; last_vs = top->vs; } }
    long p = 0; int last_hs = 0, frame = 0, old_n = 0, new_n = 0, bad = 0, lines_new = 0, set = 0;
    last_vs = 1;
    for (;; tick()) {
        if (!top->ce_pix) continue;
        if (top->vs && !last_vs) {                       // a frame starts
            if (++frame == 3) break;
            p = 0;
        }
        last_vs = top->vs;
        if (frame == 1 && !set && p == (long)M.htotal * 100) { top->hoff_in = hv; set = 1; }
        if (frame >= 1 && top->hs && !last_hs) {
            int x = (int)(p % M.htotal);
            int want_x = M.hsync0 - (frame == 2 ? sh : 0);
            if (frame == 1) old_n++; else new_n++;
            if (x != want_x) { bad++; }
            if (frame == 2) lines_new++;
        }
        last_hs = top->hs;
        p++;
    }
    printf("%s mid-frame hoff 0 -> %d: rest of the frame keeps the sync (%d lines), next frame has it %+d (%d lines), %d wrong\n",
           M.name, hv, old_n, -sh, new_n, bad);
    if (bad || old_n < M.vtotal - 100 || new_n != M.vtotal) faults++;
    hidx = hv;                                              // leave it set; the caller resets
    return faults;
}

int main(int argc, char **argv)
{
    std::string outdir = argc > 1 ? argv[1] : ".";
    top = new Vvideo_top;
    top->mode_in = 0; top->filt_in = 0; top->voff_in = 0; top->hoff_in = 0; top->ovl_en = 0; top->ovl_bcd = 0; top->spec_opt = 0; top->spec_pk = 0; top->osd_open = 0; top->spec_bars = 0; top->spec_peaks = 0;
    st7735_power_on(&lcd); lcd.rst_level = true;
    top->reset = 1; tick(); tick(); top->reset = 0; tick();
    const uint8_t init[] = { 0x11, 0x29 };
    for (uint8_t c : init) send(false, c);
    send(false, 0x36); send(true, 0xC8); send(false, 0x3A); send(true, 5);
    send(false, 0x2A); send(true, 0); send(true, 2); send(true, 0); send(true, 129);
    send(false, 0x2B); send(true, 0); send(true, 3); send(true, 0); send(true, 130);
    send(false, 0x2C);
    for (int y = 0; y < 128; y++)
        for (int x = 0; x < 128; x++) {
            uint16_t c = (uint16_t)((x * 31 / 127) << 11 | ((x ^ y) & 63) << 5 | (y * 31 / 127));
            send(true, c >> 8); send(true, c & 0xff);
        }
    st7735_render(&lcd, want);

    int faults = 0;
    // (mode, offset index): 1x and 2x at 0, -15 and +15; 3x (no offset, the
    // index ignored); 1x again
    const int order[][2] = { {0, 0}, {0, 16}, {0, 15}, {1, 0}, {1, 16}, {1, 15}, {2, 0}, {2, 15}, {0, 0} };
    const bool quick = getenv("QUICK") != nullptr;      // only the fine modes (for a quicker run while working)
    if (!quick) {
        for (auto &t : order) faults += check(t[0], t[1]);
        for (int m = 0; m < 2; m++) faults += shifted(m, 16) + shifted(m, 15);
    }

    // ---- the fine modes: 1.75x (3) and 240x240 (4) with each Scale filter ----
    // 1x and 2x must not care about the filter (the option is for the new modes only)
    printf("--- filters in 1x and 2x: no effect\n");
    for (int f = 1; f < 4; f++) { filt_sel = f; faults += check(0, 0) + check(1, 0); }
    filt_sel = 0;
    printf("--- fine modes\n");
    int only_m = -1, only_f = -1;
    if (getenv("ONLY")) sscanf(getenv("ONLY"), "%d %d", &only_m, &only_f);       // ONLY="3 1": just that mode and filter
    for (int m = 3; m <= 4; m++)
        for (int f = 0; f < 4; f++) {
            if (only_m >= 0 && (m != only_m || f != only_f)) continue;
            filt_sel = f;
            for (int idx : {0, 15, 16}) faults += check(m, idx);
            if (f == 0 || f == 2) faults += shifted(m, 16) + shifted(m, 15);
        }
    if (only_m >= 0) { printf("only: %d faults\n", faults); return faults ? 1 : 0; }
    // every way the V position can cut the picture's top (the pattern's place under the window's top)
    printf("--- fine modes: the V position cutting into the pattern\n");
    for (int m = 3; m <= 4; m++)
        for (int f : {1, 2}) {
            filt_sel = f;
            int n = m == 3 ? 9 : 16;
            for (int k = 1; k <= n; k++) faults += check(m, 31 - k);
            for (int k = 1; k <= 3; k++) faults += check(m, k);
        }
    // the H position: picture and timing, 20 steps of 8 ticks
    printf("--- fine modes: the H position\n");
    for (int m = 3; m <= 4; m++) {
        filt_sel = 3;
        hidx = 0; faults += check(m, 0);
        base = shot[m][0];
        for (int h : {1, 20, 21, 40, 41}) {
            hidx = h; faults += check(m, 0);
            int d = 0;
            for (int i = 0; i < SW * SH; i++) d += base[i] != shot[m][0][i];
            printf("%s hoff %d: sync %+d ticks, %d pixels differ from hoff 0\n\n", modes[m].name, h, -hshift(m, h), d);
            faults += d;
        }
        faults += midframe(m, 20) + midframe(m, 21);
        hidx = 0;
    }
    // a picture with detail, for the eye: single-row lines, 2-row bands, text, diagonals
    {
        printf("--- fine modes: pictures\n");
        send(false, 0x2A); send(true, 0); send(true, 2); send(true, 0); send(true, 129);
        send(false, 0x2B); send(true, 0); send(true, 3); send(true, 0); send(true, 130);
        send(false, 0x2C);
        static const char *msg = "CHELESTE 1.75";
        for (int y = 0; y < 128; y++)
            for (int x = 0; x < 128; x++) {
                int r = 8, g = 8, b = 24;
                if (y < 32) { if (y & 1) r = g = b = 255; else r = g = b = 0; }                        // single-row lines
                else if (y < 56) { if (y % 4 < 2) { r = 255; g = 160; b = 0; } else { r = 0; g = 40; b = 90; } }  // bands of 2 rows
                else if (y < 64) { if (y == 60) r = g = b = 255; else r = g = b = 20; }                  // one thin line
                else if (y < 96) {                                                                       // text, 5 rows high, three times
                    int ty = (y - 66) % 10, tx = x - 6;
                    if (ty >= 0 && ty < 5 && tx >= 0 && tx < 4 * 13) {
                        char ch = msg[tx / 4];
                        if (tx % 4 != 3 && ch != ' ') {
                            int idx = ch == '.' ? 10 : ch >= '0' && ch <= '9' ? ch - '0' : -1;
                            bool on = false;
                            if (idx >= 0) on = idx < 10 ? font[idx][ty][tx % 4] == '#' : (ty == 4 && tx % 4 == 1);
                            else on = ((ch * 7 + ty * 3 + tx % 4 * 5) % 5) < 3;       // letters: a stand-in pattern
                            if (on) r = g = b = 255;
                        }
                    }
                }
                else if (x % 16 == 0 || y % 16 == 0) { r = 0; g = 255; b = 0; }                          // a green grid, 1 pixel lines
                else if (x == y || x == 127 - y) { r = 255; g = 0; b = 0; }
                else if ((x / 4 + y / 4) & 1) { r = 40; g = 40; b = 160; }
                uint16_t c = (uint16_t)((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3));
                send(true, c >> 8); send(true, c & 0xff);
            }
        st7735_render(&lcd, want);
        for (int m = 3; m <= 4; m++)
            for (int f = 0; f < 4; f++) {
                filt_sel = f;
                faults += check(m, 0);
                char nm[64]; snprintf(nm, sizeof nm, "%s_filter%d", m == 3 ? "175x" : "240x240", f);
                ppm(outdir, nm, m, 0);
            }
        filt_sel = 0;
        faults += check(1, 0); ppm(outdir, "2x_reference", 1, 0);     // not a fine mode; ppm handles it with 2 ticks to the column
        // back to the first picture
        send(false, 0x2A); send(true, 0); send(true, 2); send(true, 0); send(true, 129);
        send(false, 0x2B); send(true, 0); send(true, 3); send(true, 0); send(true, 130);
        send(false, 0x2C);
        for (int y = 0; y < 128; y++)
            for (int x = 0; x < 128; x++) {
                uint16_t c = (uint16_t)((x * 31 / 127) << 11 | ((x ^ y) & 63) << 5 | (y * 31 / 127));
                send(true, c >> 8); send(true, c & 0xff);
            }
        st7735_render(&lcd, want);
    }
    if (quick) { printf("%s (quick): %d faults\n", faults ? "FAIL" : "PASS", faults); return faults ? 1 : 0; }
    // the H position: pictures identical, sync moved, timing unchanged (check() tests the sync
    // against the table moved by hsh), 3x ignoring it
    {
        const int hv[] = { 0, 1, 20, 21, 40, 41, 63 };
        for (int m = 0; m < 2; m++) {
            hidx = 0; faults += check(m, 0);
            base = shot[m][0];
            for (int h : hv) {
                hidx = h; faults += check(m, 0);
                int d = 0;
                for (int i = 0; i < SW * SH; i++) d += base[i] != shot[m][0][i];
                printf("%s hoff %d: sync %+d px, %d pixels differ from hoff 0\n\n", modes[m].name, h, -hshift(m, h), d);
                faults += d;
            }
            faults += midframe(m, 20) + midframe(m, 21);
            hidx = 0; faults += check(m, 0);              // back to 0 (also settles the mode)
        }
        for (int h : {20, 21, 40}) { hidx = h; faults += check(2, 0); }
        hidx = 0; faults += check(2, 0);
        faults += check(0, 0);
    }
    // the speed display on: "100%", then " 87%" and "  5%" (leading zeros blank), in every mode
    // (each checked pixel by pixel as the others, the box and the digits included)
    const struct { uint16_t bcd; const char *text; } ovls[] = { {0x100, "100%"}, {0x087, " 87%"}, {0x005, "  5%"}, {0x999, "999%"} };
    ovl_on = true;
    for (auto &o : ovls) {
        ovl_text = o.text; top->ovl_bcd = o.bcd;
        printf("--- speed display on: \"%s\"\n", o.text);
        const int k = o.bcd == 0x100 ? 3 : 2;           // 100%: all three modes; the others 1x and 2x
        for (int m = 0; m < k; m++) faults += check(m, 0);
        int white = 0;
        for (int y = 0; y < 528; y++) for (int x = 0; x < 640; x++) white += shot[0][0][y * SW + x] == 0xffffff;
        if (o.bcd == 0x100) {                       // and in the fine modes, unfiltered (the box and digits), under two filters
            for (int f : {2, 3}) { filt_sel = f; faults += check(3, 0) + check(4, 15); }
            filt_sel = 0;
            int w34 = 0;
            for (int y = 0; y < 528; y++) for (int x = 0; x < SW; x++) w34 += shotp(3, 0)[y * SW + x] == 0xffffff;
            printf("white pixels of the overlay in 1.75x: %d\n", w34);
            if (!w34) { printf("no overlay in 1.75x\n"); faults++; }
        }
        printf("white pixels of the overlay in 1x: %d\n", white);
        if (!white) { printf("no overlay\n"); faults++; }
    }
    ovl_on = false;
    // the spectrum analyser: Off / While in menu / Always, with and without peaks, the OSD open or not,
    // in every mode, against spec_at (which draws nothing when it should not be there: so the picture
    // must be the plain one then)
    {
        const int hb[7] = { 0, 3, 7, 9, 12, 15, 1 }, hp[7] = { 2, 3, 10, 9, 15, 15, 4 };
        for (int i = 0; i < 7; i++) { sbar[i] = hb[i]; spk[i] = hp[i]; }
        uint32_t bars = 0, pks = 0;
        for (int i = 0; i < 7; i++) { bars |= (uint32_t)hb[i] << (4 * i); pks |= (uint32_t)hp[i] << (4 * i); }
        top->spec_bars = bars; top->spec_peaks = pks;
        const struct { int opt, pk, osd; } cs[] = {
            {0,0,0}, {0,1,1},                 // Off: never
            {1,0,0}, {1,1,0},                 // While in menu, menu closed: not drawn
            {1,0,1}, {1,1,1},                 // menu open: drawn
            {2,0,0}, {2,1,0}, {2,0,1}, {2,1,1} };   // Always
        for (auto &c : cs) {
            int vis = c.opt == 2 || (c.opt == 1 && c.osd);
            spec_mode_v = vis ? (c.pk ? 2 : 1) : 0;
            top->spec_opt = c.opt; top->spec_pk = c.pk; top->osd_open = c.osd;
            printf("--- spectrum analyser: option %d, peaks %d, OSD %s: %s\n", c.opt, c.pk, c.osd ? "open" : "closed", vis ? "drawn" : "not drawn");
            for (int m = 0; m < 3; m++) faults += check(m, 0);
            int lit = 0;
            for (int y = 0; y < 528; y++) for (int x = 0; x < 640; x++) lit += shot[0][0][y * SW + x] != 0 && y >= 75 + 89 && y <= 75 + 106 && x >= 49 && x <= 77 && vis;
            printf("coloured pixels in the analyser's box in 1x: %d\n", lit);
            if (vis && !lit) { printf("no analyser\n"); faults++; }
        }
        // 2x with the picture moved, with the speed display on as well
        spec_mode_v = 2; top->spec_opt = 2; top->spec_pk = 1; top->osd_open = 0;
        ovl_on = true; ovl_text = "100%"; top->ovl_bcd = 0x100;
        faults += check(1, 16) + check(0, 15);
        for (int f : {0, 1, 3}) { filt_sel = f; faults += check(3, 16) + check(4, 15) + check(3, 0); }
        filt_sel = 0;
        ovl_on = false;
        spec_mode_v = 0; top->spec_opt = 0;
        faults += check(0, 0);                            // off again: the plain picture
    }
    printf("%s: %d faults\n", faults ? "FAIL" : "PASS", faults);
    return faults ? 1 : 0;
}
