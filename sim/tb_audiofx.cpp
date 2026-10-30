// chg_audio_fx.sv (via sim/audiofx_top.sv, with chg_audio.sv):
//  1. bypass: the defaults give chg_audio's stream unchanged (also with the analyser on)
//  2. the engine against a double precision cascade of the same coefficients (any config)
//  3. each EQ band's gain, with sine tones at the band centres, against its target
//  4. full scale square waves through every model and preset: no overflow (matches the reference)
//  5. the analyser's bars for a tone in each band
//  6. the shortest sample spacing the engine keeps up with
//  7. WAVs of a square wave melody through each model and preset, with peak and RMS
// Run from CHGameMiSTer/ . Usage: tb_audiofx [wav output folder]
#define _USE_MATH_DEFINES
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include "Vaudiofx_top.h"
#include "verilated.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static Vaudiofx_top *t;
static long clocks;
static int faults;
static void tick() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); clocks++; }
static int SP = 800;                       // clocks a sample

static void cfg(int model, int preset, uint32_t bands, int spec)
{
    t->model = model; t->preset = preset; t->bands = bands; t->spec = spec;
    for (int i = 0; i < 400; i++) tick();     // the states are cleared meanwhile
}
static int step(int x, int sp = 0)
{
    if (!sp) sp = SP;
    t->inj_en = 1; t->inj_sample = (int16_t)x; t->inj_strobe = 1; tick(); t->inj_strobe = 0;
    for (int i = 1; i < sp; i++) tick();
    return (int16_t)t->audio;
}

// run zeros through until every state has died away (a changed setting starts from the old states)
static void flush(int n = 100) { for (int i = 0; i < n; i++) step(0, 800); }

// ---- the reference: the same coefficients in doubles
static double coefs[512][1];
static int32_t rom[512];
static void load_rom()
{
    FILE *f = fopen("core/rtl/audiofx_coefs.hex", "r");
    if (!f) { printf("no coefficient file\n"); exit(2); }
    for (int i = 0; i < 512; i++) {
        unsigned v; if (fscanf(f, "%x", &v) != 1) break;
        rom[i] = (v & 0x4000000) ? (int32_t)(v | 0xF8000000u) : (int32_t)v;
    }
    fclose(f);
}
struct Biq {
    double c[5], x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    void set(int set) { for (int i = 0; i < 5; i++) c[i] = rom[set * 5 + i] / 16777216.0; }
    double run(double x)
    {
        double y = c[0] * x + c[1] * x1 + c[2] * x2 + c[3] * y1 + c[4] * y2;
        x2 = x1; x1 = x; y2 = y1; y1 = y; return y;
    }
};
static int gidx(int v) { return v <= 4 ? 4 + v : v <= 8 ? v - 5 : 4; }
static int pgi(int p, int b)
{
    static const int tab[5][7] = { {4,4,4,4,4,4,4}, {7,6,5,4,4,4,4}, {4,4,4,4,5,6,7}, {7,6,4,4,4,5,6}, {2,3,4,5,6,5,3} };
    return tab[p - 0 < 5 ? p : 0][b];
}
struct Ref {
    std::vector<Biq> chain;      // pre, model, bands, in order
    Ref(int model, int preset, uint32_t bands)
    {
        int g[7], maxb = 0;
        bool eq = preset >= 1 && preset <= 5;
        if (eq) for (int b = 0; b < 7; b++) {
            g[b] = preset == 5 ? gidx((bands >> (4 * b)) & 15) : pgi(preset, b);
            if (g[b] - 4 > maxb) maxb = g[b] - 4;
        }
        Biq q;
        if (eq && maxb) { q.set(maxb - 1); chain.push_back(q); }
        if (model >= 2) { q.set(67 + 2 * (model - 2)); chain.push_back(q); q.set(67 + 2 * (model - 2) + 1); chain.push_back(q); }
        if (eq) for (int b = 0; b < 7; b++) if (g[b] != 4) { q.set(4 + 9 * b + g[b]); chain.push_back(q); }
    }
    int run(int x)
    {
        double v = x;
        for (auto &q : chain) v = q.run(v);
        v = std::floor(v + 0.5);
        return v > 32767 ? 32767 : v < -32768 ? -32768 : (int)v;
    }
};

static double centre[7] = { 63, 160, 400, 1000, 2500, 6300, 16000 };
static const char *mname[8] = { "Bare pin", "Piezo filter", "Small piezo", "Large buzzer", "Speaker", "Warm", "Handheld", "Bright" };
static const char *pname[6] = { "Off", "Bass boost", "Treble boost", "Loudness", "Vocal", "Custom" };

// gain in dB of the path at f (a multiple of 10 Hz), by correlation over 0.1 s
static double gain_at(double f, double amp, int settle = 3000)
{
    const int N = 2400;
    double ph = 0, dph = 2 * M_PI * f / 48000, si = 0, sq = 0;
    for (int n = 0; n < settle + N; n++) {
        int y = step((int)std::lround(amp * std::sin(ph)));
        ph += dph;
        if (n >= settle) { double p = ph - dph; si += y * std::sin(p); sq += y * std::cos(p); }
    }
    return 20 * std::log10(2 * std::sqrt(si * si + sq * sq) / N / amp);
}

static void wav(const std::string &path, const std::vector<int16_t> &d)
{
    FILE *f = fopen(path.c_str(), "wb");
    if (!f) { printf("cannot write %s\n", path.c_str()); return; }
    uint32_t n = d.size() * 2, v;
    auto w32 = [&](uint32_t x) { fwrite(&x, 4, 1, f); };
    auto w16 = [&](uint16_t x) { fwrite(&x, 2, 1, f); };
    fwrite("RIFF", 1, 4, f); w32(36 + n); fwrite("WAVEfmt ", 1, 8, f); w32(16); w16(1); w16(1); w32(48000); w32(96000); w16(2); w16(16);
    fwrite("data", 1, 4, f); w32(n); fwrite(d.data(), 2, d.size(), f); (void)v;
    fclose(f);
}
static void levels(const char *what, const std::vector<int16_t> &d)
{
    double pk = 0, ss = 0;
    for (int16_t s : d) { pk = std::fmax(pk, std::fabs((double)s)); ss += (double)s * s; }
    printf("  %-28s peak %6.0f (%6.1f dBFS)  RMS %7.0f (%6.1f dBFS)\n", what, pk, 20 * std::log10(pk / 32768 + 1e-9),
           std::sqrt(ss / d.size()), 10 * std::log10(ss / d.size() / 1073741824.0 + 1e-12));
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    std::string outdir = argc > 1 ? argv[1] : "";
    load_rom();
    t = new Vaudiofx_top;
    t->reset = 1; t->ce = 0; t->pin = 0; t->inj_en = 0; t->inj_strobe = 0; t->inj_sample = 0;
    t->model = 0; t->preset = 0; t->bands = 0; t->spec = 0;
    for (int i = 0; i < 10; i++) tick();
    t->reset = 0;

    // ---- 1. bypass: chg_audio's stream, bare and piezo, with the analyser on or off
    {
        printf("1. bypass\n");
        for (int model = 0; model < 2; model++) for (int spec = 0; spec < 3; spec += 2) {
            cfg(model, 0, 0x4444444, spec);     // bands set but the EQ is off
            t->inj_en = 0;
            long n = 0, bad = 0, ph = 0, nstr = 0;
            int16_t prev = 0; bool have = false;
            for (long c = 0; c < 2000L * 300; c++) {
                t->ce = c & 1;
                if (t->ce) { ph++; t->pin = ((ph / 3000) & 1) ^ (((ph / 7) & 3) == 0); }   // some square wave
                // the sample is compared one clock after it is new: out lags it by one clock
                bool st = t->raw_strobe;
                int16_t before = (int16_t)t->raw;           // out is `sample` one clock later
                tick();
                if (st) nstr++;
                if (c > 2000 && (int16_t)t->audio != before) bad++;
                n++; (void)prev; (void)have;
            }
            printf("  model %d, analyser %d: %ld samples, %ld clocks with audio != chg_audio's sample -> %s\n", model, spec, nstr, bad, bad ? "FAIL" : "ok");
            faults += bad != 0;
        }
        t->ce = 0;
    }

    // ---- 2./4. the engine against the reference, for random noise and full scale squares
    printf("2. engine against a double precision reference (random noise and full scale squares)\n");
    {
        struct C { int model, preset; uint32_t bands; };
        std::vector<C> cs;
        for (int m = 2; m < 8; m++) cs.push_back({ m, 0, 0 });
        for (int p = 1; p <= 4; p++) cs.push_back({ 0, p, 0 });
        cs.push_back({ 0, 5, 0x4444444 });     // every band +12 dB
        cs.push_back({ 0, 5, 0x8888888 });     // every band -3 dB
        cs.push_back({ 0, 5, 0x5155154 });     // mixed
        cs.push_back({ 2, 5, 0x4444444 });     // the worst: a model and every band +12
        cs.push_back({ 3, 3, 0 });
        srand(7);
        for (auto &c : cs) {
            cfg(c.model, c.preset, c.bands, 0);
            flush();
            Ref ref(c.model, c.preset, c.bands);
            int maxd = 0, peak = 0, nclip = 0;
            auto run = [&](int x) {
                int y = step(x), r = ref.run(x);
                maxd = std::max(maxd, std::abs(y - r));
                peak = std::max(peak, std::abs(y));
                if (std::abs(y) >= 32767) nclip++;
            };
            for (int i = 0; i < 2500; i++) run((rand() % 20001) - 10000);
            for (double f : { 100.0, 440.0, 1000.0, 3500.0, 9000.0 }) {
                int half = (int)(48000 / f / 2);
                for (int i = 0; i < 1500; i++) run(((i / half) & 1) ? 32767 : -32768);
            }
            bool ok = maxd <= 3;
            printf("  model %d (%-12s) preset %d (%-12s) bands %07x: max |RTL - ref| %d LSB, peak %d, %d clipped samples -> %s\n",
                   c.model, mname[c.model], c.preset, pname[c.preset], c.bands, maxd, peak, nclip, ok ? "ok" : "FAIL");
            faults += !ok;
        }
    }

    // ---- 3. the bands' gains
    printf("3. EQ band gain at the band centre (custom preset, one band moved; target = band gain - pre-gain)\n");
    {
        double worst = 0;
        for (int b = 0; b < 7; b++) {
            double fc = std::round(centre[b] / 20) * 20;
            for (int v : { 4, 5 }) {           // +12, +6, -12, -6 dB
                cfg(0, 5, (uint32_t)v << (4 * b), 0);
                int db = v <= 4 ? 3 * v : 3 * (v - 9);       // 5 -> -12 ... 8 -> -3
                double pre = db > 0 ? db : 0;
                double want = db - pre;
                double g = gain_at(fc, 2000);
                double fr = gain_at(b < 3 ? 16000 : 100, 2000);   // far from the band: only the pre-gain
                double e1 = std::fabs(g - want), e2 = std::fabs(fr + pre);
                worst = std::fmax(worst, std::fmax(e1, e2));
                if (e1 > 1 || e2 > 1) { faults++; printf("  FAIL "); } else printf("  ");
                printf("band %d (%5.0f Hz) %+3d dB: %+6.2f dB at the centre (want %+.1f), %+6.2f dB far away (want %+.1f)\n", b, fc, db, g, want, fr, -pre);
            }
        }
        printf("  worst error %.2f dB\n", worst);
    }

    // ---- 3b. flat EQ is a pass through
    {
        cfg(0, 5, 0, 0);
        int bad = 0;
        for (int i = 0; i < 3000; i++) { int x = (rand() % 65536) - 32768; if (step(x) != x) bad++; }
        printf("3b. Custom preset with all bands 0 dB: %d of 3000 samples changed -> %s\n", bad, bad ? "FAIL" : "ok");
        faults += bad != 0;
    }

    // ---- 5. the analyser
    printf("5. analyser bars (tone of 8000 at each centre, 7000 samples; bars / peaks, 0-15)\n");
    {
        for (int b = 0; b < 7; b++) {
            cfg(0, 0, 0, 1);
            double fc = std::round(centre[b] / 20) * 20, ph = 0;
            for (int n = 0; n < 7000; n++) { step((int)std::lround(8000 * std::sin(ph))); ph += 2 * M_PI * fc / 48000; }
            int h[7], mx = 0, am = 0;
            for (int i = 0; i < 7; i++) { h[i] = (t->bars >> (4 * i)) & 15; if (h[i] > mx) { mx = h[i]; am = i; } }
            int nb = 0;
            for (int i = 0; i < 7; i++) if (i != b) nb = std::max(nb, h[i]);
            bool ok = am == b && h[b] - nb >= 2 && h[b] >= 9;
            printf("  tone %5.0f Hz:", fc);
            for (int i = 0; i < 7; i++) printf(" %2d", h[i]);
            printf("   -> %s\n", ok ? "ok" : "FAIL");
            faults += !ok;
        }
        // silence, then peaks hold and fall
        cfg(0, 0, 0, 2);
        double ph = 0;
        for (int n = 0; n < 7000; n++) { step((int)std::lround(20000 * std::sin(ph))); ph += 2 * M_PI * 1000 / 48000; }
        int b0 = (t->bars >> 12) & 15, p0 = (t->peaks >> 12) & 15;
        for (int n = 0; n < 8000; n++) step(0);
        int b1 = (t->bars >> 12) & 15, p1 = (t->peaks >> 12) & 15;
        for (int n = 0; n < 40000; n++) step(0);
        int b2 = (t->bars >> 12) & 15, p2 = (t->peaks >> 12) & 15;
        bool ok = b0 >= 12 && p0 >= b0 && b1 < b0 && p1 >= b1 && p1 > b1 - 1 && p2 == 0 && b2 == 0;
        printf("  1 kHz at 20000, then silence: bar %d peak %d; after 8000 samples bar %d peak %d; after 48000 bar %d peak %d -> %s\n", b0, p0, b1, p1, b2, p2, ok ? "ok" : "FAIL");
        faults += !ok;
        cfg(0, 0, 0, 0);
        bool z = t->bars == 0 && t->peaks == 0;
        printf("  analyser off: bars %07x peaks %07x -> %s\n", t->bars, t->peaks, z ? "ok" : "FAIL");
        faults += !z;
    }

    // ---- 6. how fast a sample may follow the last
    printf("6. the engine's speed (every slot running: model 2, custom +12 dB, analyser on)\n");
    {
        int ok_sp = 0; bool broke = false;
        for (int sp : { 1000, 900, 850, 800, 750, 700, 650 }) {
            cfg(2, 5, 0x4444444, 2);
            flush();
            Ref ref(2, 5, 0x4444444);
            int maxd = 0;
            for (int i = 0; i < 1500; i++) { int x = (rand() % 20001) - 10000; int y = step(x, sp), r = ref.run(x); maxd = std::max(maxd, std::abs(y - r)); }
            printf("  %d clocks a sample: max |RTL - ref| %d\n", sp, maxd);
            if (maxd <= 3 && !broke) ok_sp = sp; else broke = true;
        }
        printf("  keeps up down to %d clocks a sample (a sample comes every 1000 clocks at the fastest)\n", ok_sp);
        if (ok_sp > 1000 || ok_sp == 0) faults++;
    }

    // ---- 7. the melody
    if (!outdir.empty()) {
        printf("7. WAVs in %s\n", outdir.c_str());
        // the buzzer pin: a square wave melody with a bass line and a chirp, 0.7 s
        struct N { double f; double sec; };
        std::vector<N> notes = { {220, .07}, {523.25, .07}, {659.25, .07}, {783.99, .07}, {1046.5, .07}, {1568, .07}, {3136, .07} };
        auto melody = [&](int model, std::vector<int16_t> &d) {
            t->inj_en = 0; cfg(model, 0, 0, 0);
            double bc = 0;
            for (auto &n : notes) {
                long board = (long)(n.sec * 48e6);
                double half = 48e6 / n.f / 2;
                for (long c = 0; c < board * 2; c++) {
                    t->ce = c & 1;
                    if (t->ce) { bc++; t->pin = (long)(bc / half) & 1; }
                    bool st = t->raw_strobe;
                    tick();
                    if (st) d.push_back(model == 1 ? t->audio : t->raw);
                }
            }
            t->ce = 0;
            for (int i = 0; i < 400; i++) tick();
        };
        std::vector<int16_t> bare, piezo;
        melody(0, bare);
        melody(1, piezo);
        wav(outdir + "/00_bare_pin.wav", bare); levels("Bare pin", bare);
        wav(outdir + "/01_piezo_filter.wav", piezo); levels("Piezo filter", piezo);
        auto render = [&](const std::string &name, const char *label, int model, int preset, uint32_t bands) {
            cfg(model, preset, bands, 2);
            std::vector<int16_t> o;
            for (int16_t s : bare) o.push_back((int16_t)step(s, 800));
            wav(outdir + "/" + name + ".wav", o); levels(label, o);
        };
        for (int m = 2; m < 8; m++) { char nm[64]; snprintf(nm, sizeof nm, "%02d_%s", m, mname[m]); for (char *p = nm; *p; p++) if (*p == ' ') *p = '_'; render(nm, mname[m], m, 0, 0); }
        for (int p = 1; p < 5; p++) { char nm[64]; snprintf(nm, sizeof nm, "1%d_EQ_%s", p, pname[p]); for (char *q = nm; *q; q++) if (*q == ' ') *q = '_'; render(nm, pname[p], 0, p, 0); }
        render("15_EQ_custom_V", "Custom: 63 +12, 6.3k -12", 0, 5, 0x00500004);
        render("16_Speaker_plus_Bass_boost", "Speaker + Bass boost", 4, 1, 0);
    }

    printf("%s\n", faults ? "FAIL" : "PASS");
    delete t;
    return faults != 0;
}
