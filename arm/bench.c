/*
 * chgbench: how fast joyrider3774's CHGame emulator runs on this machine.
 *
 *   chgbench game.chg [seconds] [--sd folder] [--ppm out.ppm] [--press btn@sec[:dur]]...
 *
 * Runs the game for <seconds> of board time (default 20) the way a front end
 * would: 800,000 cycles (1/60 s) a step, the 128x128 picture rendered and the
 * buzzer turned into 48 kHz samples every step, both thrown away. Prints the
 * speed as a multiple of real time; 1.00x or more means full speed is possible.
 * btn is one of up down left right a b select start.
 *
 * Part of CHGameMiSTer. The emulator: https://github.com/joyrider3774/chgame_emulator (MIT).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "machine.h"
#include "loader.h"
#include "sdcard.h"
#include "audio.h"

#define VERSION "0.1"
#define FRAME_CYCLES (CHG_HCLK / 60)

static ChgMachine m;
static ChgAudio audio;
static uint32_t pixels[128 * 128];
static float samples[48000];

static double now_s(void)
{
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static struct { uint8_t bit; double at, dur; } press[32];
static int npress;

static int button_bit(const char *s, size_t n)
{
    static const char *names[] = { "up", "down", "left", "right", "a", "b", "select", "start" };
    for (int i = 0; i < 8; i++)
        if (strlen(names[i]) == n && !strncmp(names[i], s, n)) return 1 << i;
    return 0;
}

static void write_ppm(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return; }
    fprintf(f, "P6\n128 128\n255\n");
    for (int i = 0; i < 128 * 128; i++) {
        uint8_t rgb[3] = { pixels[i] >> 16, pixels[i] >> 8, pixels[i] };
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
}

int main(int argc, char **argv)
{
    const char *game = NULL, *sd = NULL, *ppm = NULL;
    double seconds = 20;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--sd") && i + 1 < argc) sd = argv[++i];
        else if (!strcmp(argv[i], "--ppm") && i + 1 < argc) ppm = argv[++i];
        else if (!strcmp(argv[i], "--press") && i + 1 < argc && npress < 32) {
            const char *s = argv[++i], *at = strchr(s, '@');
            int bit = at ? button_bit(s, (size_t)(at - s)) : 0;
            if (!bit) { fprintf(stderr, "bad --press %s\n", s); return 2; }
            press[npress].bit = (uint8_t)bit;
            press[npress].at = atof(at + 1);
            const char *colon = strchr(at, ':');
            press[npress].dur = colon ? atof(colon + 1) : 0.1;
            npress++;
        }
        else if (!game) game = argv[i];
        else seconds = atof(argv[i]);
    }
    printf("chgbench %s\n", VERSION);
    if (!game) {
        fprintf(stderr, "usage: chgbench game.chg [seconds] [--sd folder] [--ppm out.ppm] [--press btn@sec[:dur]]...\n");
        return 2;
    }

    char err[512];
    chg_init(&m);
    if (sd && !(m.sd = sdcard_open(sd, 0, err, sizeof err))) {
        fprintf(stderr, "SD card %s: %s\n", sd, err);
        return 1;
    }
    if (!chg_load_program(&m, game, err, sizeof err)) {
        fprintf(stderr, "%s: %s\n", game, err);
        return 1;
    }
    chg_reset(&m, true);
    chg_audio_init(&audio, 48000, m.cycles);

    long frames = (long)(seconds * 60);
    double t0 = now_s(), worst = 0;
    for (long f = 0; f < frames; f++) {
        double t = f / 60.0, ft = now_s();
        uint8_t held = 0;
        for (int i = 0; i < npress; i++)
            if (t >= press[i].at && t < press[i].at + press[i].dur) held |= press[i].bit;
        chg_set_buttons(&m, held);
        chg_run(&m, m.cycles + FRAME_CYCLES);
        chg_audio_render(&audio, &m, samples, 48000);
        st7735_render(&m.lcd, pixels);
        ft = now_s() - ft;
        if (ft > worst) worst = ft;
    }
    double wall = now_s() - t0;
    printf("%s: %.1f s of board time in %.2f s: %.2fx real time\n", game, seconds, wall, seconds / wall);
    printf("average %.2f ms a 1/60 s frame (16.67 ms is real time), worst %.2f ms\n",
           wall * 1000 / frames, worst * 1000);
    printf("instructions %llu, LCD frames %u, cpu %s\n", (unsigned long long)m.instret,
           m.lcd.frame_starts, m.cpu.faulted ? "FAULTED" : "ok");
    if (ppm) write_ppm(ppm);
    if (m.sd) sdcard_close(m.sd);
    return 0;
}
