/*
 * chgsdimg: a microSD card image for the CHGame MiSTer core, from a folder.
 *
 *   chgsdimg folder card.img [size_mb]
 *
 * Writes a FAT32 card image (MBR and all, as a card reader would dump it)
 * holding the folder's files, with the volume label CHGAME, using the
 * emulator's own sdcard_make_image, so the core sees the card the emulator
 * sees. Put the .DAT files games need in the folder (STARLIFT.DAT, ...).
 * Without size_mb the image is just big enough (at least 64 MB).
 *
 * Part of CHGameMiSTer. The emulator: https://github.com/joyrider3774/chgame_emulator (MIT).
 */
#include <stdio.h>
#include <stdlib.h>
#include "sdcard.h"

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "chgsdimg 0.1\nusage: chgsdimg folder card.img [size_mb]\n");
        return 2;
    }
    char err[512];
    uint32_t mb = argc > 3 ? (uint32_t)atoi(argv[3]) : 0;
    if (!sdcard_make_image(argv[2], argv[1], mb, err, sizeof err)) {
        fprintf(stderr, "%s\n", err);
        return 1;
    }
    printf("%s written from %s\n", argv[2], argv[1]);
    return 0;
}
