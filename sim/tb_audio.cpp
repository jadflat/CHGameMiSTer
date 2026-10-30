// chg_audio.sv against emu/src/audio.c: a game runs in the C model, the
// buzzer pin is rebuilt cycle by cycle from the C's log of what drives it,
// and both turn it into 48 kHz samples. Prints the largest difference, bare
// pin and piezo filter.
//   sim/out/audio/audio.exe game.chg seconds [sd folder]
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>
#include "Vchg_audio.h"
#include "verilated.h"
extern "C" {
#include "machine.h"
#include "loader.h"
#include "sdcard.h"
#include "audio.h"
}

static ChgMachine m;
static ChgAudio abare, apiezo;
static Vchg_audio *bare, *piezo;

static void tick(Vchg_audio *t) { t->clk = 0; t->eval(); t->clk = 1; t->eval(); }

// the pin at cycle c under state s, as high_time integrates it
static int pin_at(const ChgBuzzState &s, uint64_t c)
{
    if (!s.pwm) return s.level;
    const uint64_t ph = (c - s.origin) % s.period;
    return ph < s.high ? s.level : !s.level;
}

static int16_t to16(float v)
{
    v *= 32767.0f;
    v = v > 32767.0f ? 32767.0f : v < -32768.0f ? -32768.0f : v;
    return (int16_t)v;
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: audio game.chg seconds [sd]\n"); return 2; }
    char err[256];
    chg_init(&m);
    if (argc > 3 && !(m.sd = sdcard_open(argv[3], 0, err, sizeof err))) { fprintf(stderr, "%s\n", err); return 1; }
    if (!chg_load_program(&m, argv[1], err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    chg_reset(&m, true);
    chg_audio_init(&abare, 48000, m.cycles);
    chg_audio_init(&apiezo, 48000, m.cycles);
    apiezo.piezo = true;

    bare = new Vchg_audio; piezo = new Vchg_audio;
    for (Vchg_audio *t : { bare, piezo }) {
        t->vol = 512;               // 0.5, the emulator's default
        t->piezo = t == piezo;
        t->reset = 1; tick(t); tick(t); t->reset = 0;
    }

    ChgBuzzState cur = m.buzzer.cur;
    cur.cycle = m.cycles;
    std::vector<ChgBuzzState> pending;
    std::vector<int16_t> want_b, want_p, got_b, got_p;
    static float buf[48000];
    const int frames = atoi(argv[2]) * 60;
    uint64_t t = m.cycles;          // next cycle to feed the RTL

    for (int f = 1; f <= frames; f++) {
        // buttons: through the title screen and into play, then some movement
        int s = f / 60;
        uint8_t btn = (s == 2 || s == 4) ? CHG_BTN_START : (s >= 6 && s % 2) ? (CHG_BTN_A | CHG_BTN_LEFT) : 0;
        chg_set_buttons(&m, btn);
        chg_run(&m, (uint64_t)f * (CHG_HCLK / 60));

        // the log, read before the C renders (rendering consumes it)
        for (unsigned i = m.buzzer.tail; i != m.buzzer.head; i = (i + 1) % CHG_BUZZ_LOG)
            pending.push_back(m.buzzer.log[i]);
        const unsigned tail = m.buzzer.tail;
        int n = chg_audio_render(&abare, &m, buf, 48000);
        for (int i = 0; i < n; i++) want_b.push_back(to16(buf[i]));
        m.buzzer.tail = tail;
        n = chg_audio_render(&apiezo, &m, buf, 48000);
        for (int i = 0; i < n; i++) want_p.push_back(to16(buf[i]));

        // the same cycles through the RTL, the pin rebuilt from the log
        const uint64_t end = abare.pos > 0 ? (uint64_t)llround(abare.pos) : m.cycles;
        size_t k = 0;
        for (; t < end; t++) {
            while (k < pending.size() && pending[k].cycle <= t) cur = pending[k++];
            const int p = pin_at(cur, t);
            for (Vchg_audio *v : { bare, piezo }) {
                v->ce = 1; v->pin = p; tick(v); v->ce = 0;
                if (v->strobe) (v == bare ? got_b : got_p).push_back((int16_t)v->sample);
            }
        }
        pending.erase(pending.begin(), pending.begin() + k);
        // let the FIR finish (about 260 clocks) without board time
        for (int i = 0; i < 300; i++)
            for (Vchg_audio *v : { bare, piezo }) {
                tick(v);
                if (v->strobe) (v == bare ? got_b : got_p).push_back((int16_t)v->sample);
            }
    }

    int rc = 0;
    for (int which = 0; which < 2; which++) {
        auto &w = which ? want_p : want_b;
        auto &g = which ? got_p : got_b;
        size_t n = w.size() < g.size() ? w.size() : g.size();
        int worst = 0; size_t at = 0, over1 = 0; double peak = 0;
        for (size_t i = 0; i < n; i++) {
            int d = abs(w[i] - g[i]);
            if (d > worst) { worst = d; at = i; }
            if (d > 1) over1++;
            if (abs(w[i]) > peak) peak = abs(w[i]);
        }
        printf("%s: C %zu samples, RTL %zu; largest difference %d LSB (sample %zu: C %d RTL %d), %zu over 1 LSB; C peak %.0f\n",
               which ? "piezo" : "bare ", w.size(), g.size(), worst, at, n ? w[at] : 0, n ? g[at] : 0, over1, peak);
        if (worst > (which ? 8 : 2)) rc = 1;
    }
    return rc;
}
