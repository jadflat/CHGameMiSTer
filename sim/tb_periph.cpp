// The peripheral testbenches (core/rtl/periph against emu/src/bus.c):
//
//   tb_periph <test> [accesses] [seed]
//     systick gpio rcc tim flash adc misc   random and directed tests of one module
//     replay <game.CHG> <trace> [sd folder] a game's recorded traffic (tb_periph_log)
//
// Each test is its own file, tb_periph_<name>.cpp; the harness is
// tb_periph_common.cpp.
#include "tb_periph_common.cpp"
#include "tb_periph_systick.cpp"
#include "tb_periph_gpio.cpp"
#include "tb_periph_rcc.cpp"
#include "tb_periph_tim.cpp"
#include "tb_periph_flashctl.cpp"
#include "tb_periph_adc.cpp"
#include "tb_periph_misc.cpp"
#include "tb_periph_replay.cpp"

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    if (argc < 2) { fprintf(stderr, "usage: tb_periph <test> [n] [seed]\n"); return 2; }
    const std::string t = argv[1];
    if (t == "replay") {
        if (argc < 4) { fprintf(stderr, "usage: tb_periph replay game.CHG trace [sd]\n"); return 2; }
        return test_replay(argv[2], argv[3], argc > 4 && argv[4][0] ? argv[4] : nullptr, argc > 5 ? atol(argv[5]) : 0);
    }
    const long n = argc > 2 ? atol(argv[2]) : 20000;
    const uint32_t seed = argc > 3 ? strtoul(argv[3], 0, 0) : 1;
    if (t == "systick") return test_systick(n, seed);
    if (t == "gpio") return test_gpio(n, seed);
    if (t == "rcc") return test_rcc(n, seed);
    if (t == "tim") return test_tim(n, seed);
    if (t == "flash") return test_flash(n, seed);
    if (t == "adc") return test_adc(n, seed);
    if (t == "misc") return test_misc(n, seed);
    fprintf(stderr, "no test %s\n", t.c_str());
    return 2;
}
