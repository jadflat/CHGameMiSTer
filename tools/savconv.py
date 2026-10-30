"""Converts CHGame saves between the emulator's .sav and the MiSTer core's.

  python -I tools/savconv.py to-mister  game.sav  game_mister.sav
  python -I tools/savconv.py to-emu     game_mister.sav game.sav

The emulator's (and the libretro core's) .sav is a list of records, each a
4-byte little-endian flash address and the 256-byte page there. The MiSTer
core's is fixed size, 125 blocks of 512 bytes: block 0 holds "CHGS", version
1, the page count 248 and at byte 16 a bitmap of the saved pages; blocks
1-124 hold pages 0-247. Pages not in the bitmap are 0xFF here.
CHGameMiSTer 0.1.
"""
import sys

PAGES, PAGE = 248, 256

def to_mister(data):
    img = bytearray(b'\xff' * (PAGES * PAGE))
    bitmap = bytearray(31)
    for o in range(0, len(data) - 4 - PAGE + 1, 4 + PAGE):
        addr = int.from_bytes(data[o:o + 4], 'little')
        if addr % PAGE or addr + PAGE > PAGES * PAGE:
            continue
        p = addr // PAGE
        img[addr:addr + PAGE] = data[o + 4:o + 4 + PAGE]
        bitmap[p // 8] |= 1 << (p % 8)
    hdr = bytearray(512)
    hdr[0:4] = b'CHGS'
    hdr[4] = 1
    hdr[8:10] = PAGES.to_bytes(2, 'little')
    hdr[16:16 + 31] = bitmap
    return bytes(hdr) + bytes(img)

def to_emu(data):
    if data[0:4] != b'CHGS' or data[4] != 1:
        sys.exit('not a CHGame MiSTer save')
    out = bytearray()
    for p in range(PAGES):
        if data[16 + p // 8] >> (p % 8) & 1:
            addr = p * PAGE
            out += addr.to_bytes(4, 'little') + data[512 + addr:512 + addr + PAGE]
    return bytes(out)

if __name__ == '__main__':
    if len(sys.argv) != 4 or sys.argv[1] not in ('to-mister', 'to-emu'):
        sys.exit(__doc__)
    data = open(sys.argv[2], 'rb').read()
    out = to_mister(data) if sys.argv[1] == 'to-mister' else to_emu(data)
    open(sys.argv[3], 'wb').write(out)
    print('%s: %d bytes' % (sys.argv[3], len(out)))
