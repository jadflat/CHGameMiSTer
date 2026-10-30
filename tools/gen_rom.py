"""Generates the RTL's ROM files from the emulator's C sources.

  core/rtl/piezo_fir.hex   256 x 18-bit signed, Q17: emu/src/piezo_filter.c
  core/rtl/flash_init.hex  16384 x 32-bit words: the 64 KB flash as a CHGame
                           leaves the factory with the bootloader installed:
                           emu/src/bootloader_image.c at 0x0000, 0xFF after

Run from CHGameMiSTer/: python -I tools/gen_rom.py
"""
import re

src = open('emu/src/piezo_filter.c').read()
body = src.split('chg_piezo_fir[256] = {')[1].split('};')[0]
fir = [float(x.rstrip('f')) for x in re.findall(r'-?[0-9.]+e[-+]?[0-9]+f', body)]
assert len(fir) == 256, len(fir)
q = [max(-(1 << 17), min((1 << 17) - 1, round(c * (1 << 17)))) for c in fir]
with open('core/rtl/piezo_fir.hex', 'w', newline='') as f:
    for v in q:
        f.write('%05x\n' % (v & 0x3ffff))
print('piezo_fir.hex: max |coef| %.4f' % max(abs(c) for c in fir))

src = open('emu/src/bootloader_image.c').read()
body = src.split('chg_bootloader[] = {')[1].split('};')[0]
boot = bytes(int(x, 16) for x in re.findall(r'0x([0-9a-fA-F]{2})', body))
assert 0 < len(boot) <= 0x3000, len(boot)
flash = boot + b'\xff' * (0x10000 - len(boot))
with open('core/rtl/flash_init.hex', 'w', newline='') as f:
    for i in range(0, 0x10000, 4):
        f.write('%08x\n' % int.from_bytes(flash[i:i + 4], 'little'))
print('flash_init.hex: bootloader %d bytes' % len(boot))


# the ST7735's frame memory at power-on, as st7735_power_on fills it (an LCG), 162 x 132
r = 0x12345678
with open('core/rtl/gram_init.hex', 'w', newline='') as f:
    for i in range(162 * 132):
        r = (r * 1664525 + 1013904223) & 0xffffffff
        f.write('%05x\n' % (r >> 14))
print('gram_init.hex: %d pixels' % (162 * 132))
