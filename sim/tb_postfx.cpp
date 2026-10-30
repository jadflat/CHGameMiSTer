// chg_postfx.sv between chg_video and the video path (sim/postfx_top.sv): with
// every option off the picture is chg_video's, exactly, delayed by 2 pixels
// (all signals together); the grid, the LCD colours, the ghosting and the LED
// dot each against a model written here, per pixel, from chg_video's own frames
// (chg_video itself is checked against the emulator by tb_video). Pictures are
// written to a directory given as argv[1] (PPM).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cstdarg>
#include <vector>
#include <string>
#include "Vpostfx_top.h"
#include "verilated.h"

static Vpostfx_top *top;
static bool logging;
static std::vector<uint32_t> rawv, outv;     // one entry a ce_pix: rgb, hs<<24, vs<<25, hblank<<26, vblank<<27
static std::string outdir;

static uint32_t pack(int r, int g, int b, int hs, int vs, int hb, int vb)
{ return (uint32_t)r << 16 | g << 8 | b | (uint32_t)hs << 24 | (uint32_t)vs << 25 | (uint32_t)hb << 26 | (uint32_t)vb << 27; }
static void tick()
{
    if (logging && top->ce_pix) {
        rawv.push_back(pack(top->v_r, top->v_g, top->v_b, top->v_hs, top->v_vs, top->v_hblank, top->v_vblank));
        outv.push_back(pack(top->r, top->g, top->b, top->hs, top->vs, top->hblank, top->vblank));
    }
    top->clk = 0; top->eval(); top->clk = 1; top->eval();
}
static void send(bool dc, uint8_t b)
{
    top->byte_stb = 1; top->dc = dc; top->byte_in = b; tick(); top->byte_stb = 0; tick();
}
// glass pictures: A is tb_video's; B a different one
static void picture(int which)
{
    send(false, 0x2A); send(true, 0); send(true, 2); send(true, 0); send(true, 129);
    send(false, 0x2B); send(true, 0); send(true, 3); send(true, 0); send(true, 130);
    send(false, 0x2C);
    for (int y = 0; y < 128; y++)
        for (int x = 0; x < 128; x++) {
            uint16_t c = which == 0 ? (uint16_t)((x * 31 / 127) << 11 | ((x ^ y) & 63) << 5 | (y * 31 / 127))
                                    : (uint16_t)((((x + y) & 31)) << 11 | ((127 - x) & 63) << 5 | ((127 - y) * 31 / 127));
            if (which == 3) c = 0xffff;
            if (which == 4) c = 0;
            if (which == 2) c = ((x / 16 + y / 16) & 1) ? 0xffff : (uint16_t)(((x / 16) * 4 + 3) << 11 | ((y / 16) * 8 + 7) << 5 | 0x0f);   // colour blocks
            send(true, c >> 8); send(true, c & 0xff);
        }
}

// T, g, gl, ph: the fine modes (3: 1.75x, 4: 240x240; a glass pixel is T ticks, a group g glass rows in gl lines,
// ph lines of picture); delay: the ticks from chg_video's picture to the whole chain's (chg_hfine's) output
struct Mode { const char *name; int htotal, vtotal, vis_w, vis_h, vstart, scale, row0, T, g, gl, ph, delay; };
static const Mode modes[] = {
    { "15 kHz 1x", 384, 262, 128, 128, 75, 1, 0, 0, 0, 0, 0, 2 },
    { "15 kHz 2x", 384, 262, 256, 240, 19, 2, 4, 0, 0, 0, 0, 2 },
    { "31 kHz 3x", 610, 525, 384, 384, 83, 3, 0, 0, 0, 0, 0, 2 },
    { "15 kHz 1.75x", 3072, 262, 1792, 224, 27, 1, 0, 14, 4, 7, 224, 57 },
    { "15 kHz 240x240", 3072, 262, 1920, 240, 19, 1, 0, 15, 8, 15, 240, 61 },
};
// the glass row on line j of a fine mode's picture
static int fine_row(const Mode &M, int j)
{
    int q = j / M.gl, rm = j % M.gl;
    return q * M.g + (rm == M.gl - 1 ? M.g - 1 : rm / 2);
}
typedef std::vector<uint32_t> Frame;

static int faults;
static void fail(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void fail(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap); faults++;
}

// the vs rises in a log
static std::vector<long> rises(const std::vector<uint32_t> &v)
{
    std::vector<long> r; uint32_t last = 1u << 25;
    for (size_t i = 0; i < v.size(); i++) { if ((v[i] & (1u << 25)) && !(last & (1u << 25))) r.push_back((long)i); last = v[i]; }
    return r;
}
static std::vector<Frame> split(const std::vector<uint32_t> &v, const std::vector<long> &rs, int n)
{
    std::vector<Frame> f;
    for (size_t k = 0; k + 1 < rs.size(); k++) {
        if (rs[k + 1] - rs[k] != n) { fail("a frame of %ld pixels, not %d\n", rs[k + 1] - rs[k], n); continue; }
        f.emplace_back(v.begin() + rs[k], v.begin() + rs[k + 1]);
    }
    return f;
}

// configuration
struct Cfg { int m, idx, look, col, led_en, led, sty; };   // look = grid strength 0-4, sty = grid style 0-3
static Cfg cur;
static void apply(const Cfg &c)
{
    cur = c;
    top->mode_in = c.m; top->voff_in = c.idx; top->hoff_in = 0;
    top->grid_str = c.look; top->grid_sty = c.sty; top->lcd_col = c.col; top->led_en = c.led_en; top->led = c.led;
}
static void run_vs(int n)         // n raw vsync rises
{
    int last = top->v_vs;
    for (int k = 0; k < n; ) { tick(); if (top->ce_pix) { if (top->v_vs && !last) k++; last = top->v_vs; } }
}
static void skip() { for (int i = 0; i < 60000; i++) tick(); }   // past the vsync, so a log starts with vs low
static void settle(const Cfg &c)  // the new mode, two frames with it
{
    apply(c);
    long g = 0;
    while (top->mode != c.m) { tick(); if (++g > 20000000) { printf("mode never started\n"); exit(1); } }
    run_vs(3);
    skip();
}
static int offset_of(const Cfg &c)
{
    int sel = c.idx < 16 ? c.idx : c.idx - 31;
    return c.m == 2 ? 0 : sel;
}
static int ytop(const Mode &M, int m, int off) { return M.T ? M.vstart + off : M.vstart - M.row0 * M.scale + off; }

// the model
static int quant_t(int c) { return c; }
static uint32_t lcd_colour(uint32_t p)
{
    int r = p >> 16 & 255, g = p >> 8 & 255, b = p & 255;
    int y = (r + 2 * g + b) / 4;
    int c[3] = { r, g, b };
    for (int &v : c) {
        v = (3 * v + y) / 4;               // 75% of the colour, the rest grey
        v = 12 + v - v / 8;                // black lifted to 12, 7/8 contrast
    }
    c[0] = c[0] - c[0] / 16;               // cooler: less red, more blue
    c[2] = c[2] + c[2] / 32; if (c[2] > 255) c[2] = 255;
    return (uint32_t)c[0] << 16 | c[1] << 8 | c[2];
}
static int dim1(int x, int str) { return str == 1 ? (x * 29) >> 5 : str == 2 ? x - x / 4 : str == 3 ? x / 2 : 0; }
static uint32_t dim(uint32_t p, int str)
{
    int r = p >> 16 & 255, g = p >> 8 & 255, b = p & 255;
    return (uint32_t)dim1(r, str) << 16 | dim1(g, str) << 8 | dim1(b, str);
}
struct State {};

// what chg_postfx should output for a frame of chg_video's. st (if not null) is the
// ghosting memory, updated. cfgs from cur. Returns the number of pixels the options changed.
static Frame model(const Frame &raw, const Cfg &c, State *st)
{
    const Mode &M = modes[c.m];
    const int S = M.scale, off = offset_of(c);
    const int pst = ytop(M, c.m, off);
    Frame out(raw.size());
    for (int y = 0; y < M.vtotal; y++)
        for (int x = 0; x < M.htotal; x++) {
            uint32_t p = raw[y * M.htotal + x];
            bool blank = p >> 26 & 3;
            int gl = y - pst;
            bool pic = !blank && gl >= 0 && gl < (M.T ? M.ph : 128 * S);
            uint32_t v = p & 0xffffff, flags = p & 0xff000000;
            int gxx = M.T ? x / M.T : x / S, gyy = M.T ? (pic ? fine_row(M, gl) : 0) : gl / S;
            if (pic) {
                if (c.col) v = lcd_colour(v);
                if (c.look && S > 1) {
                    bool xl = x % S == S - 1, yl = gl % S == S - 1;
                    if (c.sty == 3) { if (xl && yl) v = dim(v, c.look); }
                    else {
                        if (c.sty != 1 && xl) v = dim(v, c.look);
                        if (c.sty != 2 && yl) v = dim(v, c.look);
                    }
                }
                if (c.led_en && gxx >= 1 && gxx <= 5 && gyy >= 110 && gyy <= 114) {
                    bool dot = gxx >= 2 && gxx <= 4 && gyy >= 111 && gyy <= 113;
                    v = dot ? (c.led ? 0xf02020 : 0x383838) : 0;
                }
            }
            out[y * M.htotal + x] = flags | v;
        }
    return out;
}

static int diff(const char *what, const Frame &got, const Frame &want, const Mode &M)
{
    int bad = 0;
    for (size_t i = 0; i < got.size(); i++)
        if (got[i] != want[i]) {
            if (bad++ < 4) printf("  %s (%d,%d): got %08x want %08x\n", what, (int)(i % M.htotal), (int)(i / M.htotal), got[i], want[i]);
        }
    return bad;
}
static int count_diff(const Frame &a, const Frame &b)
{
    int n = 0; for (size_t i = 0; i < a.size(); i++) n += a[i] != b[i]; return n;
}

static void ppm(const char *name, const Frame &f, const Mode &M)
{
    // the visible window: the lines and columns not blanked
    int y0 = -1, y1 = -1;
    for (int y = 0; y < M.vtotal; y++) {
        uint32_t p = f[y * M.htotal];
        if (!(p >> 26 & 3)) { if (y0 < 0) y0 = y; y1 = y; }
    }
    std::string path = outdir + "/" + name + ".ppm";
    FILE *fp = fopen(path.c_str(), "wb");
    if (!fp) { printf("cannot write %s\n", path.c_str()); return; }
    int w = M.T ? M.vis_w / 2 : M.vis_w, h = y1 - y0 + 1, rr = M.T ? 4 : 1;      // fine modes: every second tick, 4 rows a line
    h *= rr;
    fprintf(fp, "P6\n%d %d\n255\n", w, h);
    for (int y = y0; y <= y1; y++)
        for (int k = 0; k < rr; k++)
            for (int x = 0; x < w; x++) {
                uint32_t p = f[y * M.htotal + x * (M.T ? 2 : 1)];
                fputc(p >> 16 & 255, fp); fputc(p >> 8 & 255, fp); fputc(p & 255, fp);
            }
    fclose(fp);
}

// capture n whole frames of both signal sets; checks the delay (outputs 2 pixels behind) on the way
static void grab(int n, std::vector<Frame> &raw, std::vector<Frame> &out, const Mode &M)
{
    skip();
    rawv.clear(); outv.clear(); logging = true;
    run_vs(n + 2);
    logging = false;
    auto rr = rises(rawv), ro = rises(outv);
    raw = split(rawv, rr, M.htotal * M.vtotal);
    out = split(outv, ro, M.htotal * M.vtotal);
    size_t k = raw.size() < out.size() ? raw.size() : out.size();
    raw.resize(k); out.resize(k);
    for (size_t i = 0; i < k; i++)
        if (ro[i] - rr[i] != M.delay) fail("delay of frame %zu: %ld pixels, not %d\n", i, ro[i] - rr[i], M.delay);
}

static void report(const char *what, const Cfg &c, int bad, int changed)
{
    printf("%-34s %s idx %2d: %s (%d pixels differ from the model, %d changed from chg_video's)\n", what, modes[c.m].name,
           c.idx, bad ? "FAIL" : "ok", bad, changed);
    if (bad) faults++;
}

int main(int argc, char **argv)
{
    outdir = argc > 1 ? argv[1] : ".";
    top = new Vpostfx_top;
    top->mode_in = 0; top->filt_in = 0; top->voff_in = 0; top->hoff_in = 0; top->grid_str = 0; top->grid_sty = 0; top->lcd_col = 0; top->led_en = 0; top->led = 0;
    top->reset = 1; tick(); tick(); top->reset = 0; tick();
    send(false, 0x11); send(false, 0x29);
    send(false, 0x36); send(true, 0xC8); send(false, 0x3A); send(true, 5);
    picture(0);

    const bool full = argc > 2;      // argv[2]: also the colour, LED and combined checks
    const Cfg cfgs[] = { {0, 0}, {1, 0}, {1, 16}, {1, 15}, {2, 0}, {3, 0}, {3, 15}, {4, 0}, {4, 16} };
    std::vector<Frame> raw, out;

    // 1. everything off: chg_video's frames, every signal, exactly
    printf("--- options off: the picture is chg_video's, 2 pixels late\n");
    for (auto c : cfgs) {
        if (!full && c.m > 1) continue;
        settle(c);
        grab(3, raw, out, modes[c.m]);
        int bad = 0;
        for (size_t i = 0; i < out.size(); i++) bad += count_diff(raw[i], out[i]);
        printf("%-34s %s idx %2d: %zu frames of %zu pixels, %d pixels (rgb, hs, vs, hblank, vblank) differ\n", "off = bypass",
               modes[c.m].name, c.idx, out.size(), raw[0].size(), bad);
        if (bad || out.size() < 2) faults++;
    }

    // 2. the grid: every strength and style in 2x; strength 2 / lines elsewhere (1x, 3x, fine: no change)
    printf("--- grid\n");
    static const char *sn[] = { "off", "light", "medium", "dark", "black" }, *yn[] = { "lines", "rows", "columns", "dots" };
    for (auto c : cfgs) {
        if (c.m == 1 && c.idx == 15) continue;
        for (int str = 1; str <= 4; str++) for (int sty = 0; sty < 4; sty++) {
            if (!(c.m == 1 && c.idx == 0) && (str != 2 || sty != 0)) { if (!(c.m == 1 && c.idx == 16 && sty != 0 && str == 2)) continue; }
            c.look = str; c.sty = sty; settle(c);
            grab(2, raw, out, modes[c.m]);
            int bad = 0, ch = 0;
            for (size_t i = 0; i < out.size(); i++) { bad += diff("grid", out[i], model(raw[i], c, nullptr), modes[c.m]); ch += count_diff(raw[i], out[i]); }
            printf("%s/%s: ", sn[str], yn[sty]);
            report("grid", c, bad, ch / (int)out.size());
            if ((c.m == 0 || c.m >= 3) && ch) fail("the grid changed the %s picture (it has no grid)\n", modes[c.m].name);
            if (c.m == 1 && c.idx == 0 && str == 2 && sty == 0) ppm("grid_2x", out[0], modes[1]);
            if (c.m == 1 && c.idx == 0 && str == 3 && sty == 3) ppm("grid_2x_dark_dots", out[0], modes[1]);
            if (c.m == 0) ppm("grid_1x_no_effect", out[0], modes[0]);
        }
    }

    if (full) {
    // 3. the LCD colours
    printf("--- LCD colours\n");
    for (auto c : cfgs) {
        c.col = 1; settle(c);
        grab(2, raw, out, modes[c.m]);
        int bad = 0, ch = 0;
        for (size_t i = 0; i < out.size(); i++) { bad += diff("colours", out[i], model(raw[i], c, nullptr), modes[c.m]); ch += count_diff(raw[i], out[i]); }
        report("colours", c, bad, ch / (int)out.size());
        // the blanking and the black around a moved picture stay 0
        int stray = 0;
        const Mode &M = modes[c.m];
        for (int y = 0; y < M.vtotal; y++)
            for (int x = 0; x < M.htotal; x++) {
                uint32_t p = out[0][y * M.htotal + x];
                if ((p >> 26 & 3) && (p & 0xffffff)) stray++;
            }
        if (stray) fail("  colours: %d blanked pixels not black\n", stray);
        if (c.m == 0 && c.idx == 0) ppm("colours_1x", out[0], M);
        if (c.m == 1 && c.idx == 0) { ppm("colours_2x", out[0], M); ppm("colours_2x_input", raw[0], M); }
        if (c.m >= 3 && c.idx == 0) ppm(c.m == 3 ? "colours_175x" : "colours_240x240", out[0], M);
    }

    // 4. the LED dot
    printf("--- LED dot\n");
    for (auto c : cfgs) {
        const Mode &M = modes[c.m];
        for (int l = 0; l < 3; l++) {
            c.led_en = l != 2; c.led = l == 0; settle(c);
            grab(2, raw, out, M);
            int bad = 0, ch = 0;
            for (size_t i = 0; i < out.size(); i++) { bad += diff("led", out[i], model(raw[i], c, nullptr), M); ch += count_diff(raw[i], out[i]); }
            const char *w = l == 0 ? "LED on, lit" : l == 1 ? "LED on, unlit" : "LED off";
            report(w, c, bad, ch / (int)out.size());
            if (l == 2 && ch) fail("  LED off still changes the picture\n");
            if (c.idx == 0 && c.m >= 3 && l < 2) { char n[64]; snprintf(n, sizeof n, "led_%s_%s", c.m == 3 ? "175x" : "240x240", l ? "off" : "lit"); ppm(n, out[0], M); }
            if (c.idx == 0 && (c.m == 0 || c.m == 1) && l < 2) { char n[64]; snprintf(n, sizeof n, "led_%s_%s", c.m ? "2x" : "1x", l ? "off" : "lit"); ppm(n, out[0], M); }
        }
        // the LED changing in the middle of a frame shows from the next one
        c.led_en = 1; c.led = 0; settle(c);
        rawv.clear(); outv.clear(); logging = true;
        run_vs(1); for (int i = 0; i < 100000; i++) tick(); top->led = 1; c.led = 1;
        run_vs(3); logging = false;
        auto rr = rises(rawv), ro = rises(outv);
        auto fr = split(rawv, rr, M.htotal * M.vtotal), fo = split(outv, ro, M.htotal * M.vtotal);
        Cfg c0 = c; c0.led = 0;
        int bad = 0;
        // frame 0 begins with led 0 (the change comes 100000 clocks into it, before the LED's pixels
        // in 1x/2x but they are sampled per pixel: so only frames 1+ are unambiguous)
        for (size_t i = 1; i < fo.size() && i < fr.size(); i++) bad += diff("led change", fo[i], model(fr[i], c, nullptr), M);
        report("LED lit mid-frame, next frames", c, bad, 0);
    }

    }

    if (full) {
    // 6. everything at once, 2x: for the picture
    {
        Cfg c = {1, 0, 2, 1, 1, 1};
        picture(2);
        settle(c);
        grab(2, raw, out, modes[1]);
        run_vs(8);
        grab(2, raw, out, modes[1]);
        Cfg c0 = c; c0.look = 2;
        int bad = diff("all", out[1], model(raw[1], c0, nullptr), modes[1]);
        report("grid + colours + LED, settled ghosting", c, bad, count_diff(raw[1], out[1]));
        ppm("all_2x_blocks", out[1], modes[1]);
        c = {2, 0, 2, 1, 1, 1};
        settle(c); run_vs(8);
        grab(2, raw, out, modes[2]);
        c0 = c; c0.look = 2;
        bad = diff("all", out[1], model(raw[1], c0, nullptr), modes[2]);
        report("grid + colours + LED, settled ghosting", c, bad, count_diff(raw[1], out[1]));
        ppm("all_3x_blocks", out[1], modes[2]);
        c = {0, 0, 2, 1, 1, 0}; picture(0);
        settle(c); run_vs(8);
        grab(2, raw, out, modes[0]);
        c0 = c; c0.look = 0;
        bad = diff("all", out[1], model(raw[1], c0, nullptr), modes[0]);
        report("colours + LED, settled ghosting", c, bad, count_diff(raw[1], out[1]));
        ppm("all_1x", out[1], modes[0]);
        ppm("plain_1x", raw[1], modes[0]);
    }

    }

    printf("%s: %d faults\n", faults ? "FAIL" : "PASS", faults);
    return faults ? 1 : 0;
}
