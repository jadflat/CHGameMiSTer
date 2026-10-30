# CHGameMiSTer

A MiSTer FPGA core for the CHGame handheld (WCH CH32X035 RISC-V chip, 128 x 128 ST7735 screen,
piezo buzzer, microSD card). It reproduces the board cycle for cycle in the FPGA and plays
CHGame `.CHG` games.

**Status: public beta (v0.5).** Every game tested runs at 100% speed on a DE10-Nano, over HDMI and
on a 15 kHz analogue monitor. Expect rough edges; please report them.

This repository contains no games. Bring your own `.CHG` and `.DAT` files.

## Install

**With Downloader / update_all:** follow `docs/DOWNLOADER-USERS.md` (add the two-line
`downloader_chgame.ini` to the top of the SD card, then run update_all).

**By hand:** copy `CHGame_YYYYMMDD.rbf` to `_Console` (or `_Other`) on the MiSTer's SD card, keeping only
one CHGame `.rbf` there. Put your games in `/media/fat/games/CHGame/`. Details: [INSTALL.md](INSTALL.md).

## Playing

- **Games:** *Load game* in the OSD lists the `.CHG` files in `games/CHGame`.
- **Data files:** games that read the microSD card need their `.DAT` file: use *Load data file*
  (repeat for each file, up to 8; *Clear data files* forgets them). A card image can also be mounted.
- **Saves:** kept automatically as `.sav` files in `saves/CHGame/` (Autosave or *Save now*).
  `tools/savconv.py` converts saves to and from the PC emulator's.

OSD pages:

- **Main:** load game / data file, mount image, speed (fast forward, speed display), autosave,
  pause when OSD is open, button mapping, and *Fast card loading* (Auto, Always on, Off): Auto speeds
  up a game while it loads from the card and returns to exact timing when it draws again.
- **Video:** analogue mode (15 kHz 1x, 1.75x, 240x240, 2x), V and H position, *Per-game position*
  (kept in each game's save), aspect ratio, scale, scandoubler effects, scale filter, LCD grid
  (2x only), LCD colours, LED dot.
- **Audio:** sound (bare pin or piezo models), volume, EQ preset, spectrum display.
- **Equaliser:** seven bands from 63 Hz to 6.3 kHz, used by the Custom preset.

15 kHz analogue output is 240p timing (15.625 kHz, 60 Hz) with the 128 x 128 picture square on a
4:3 tube; HDMI goes through MiSTer's scaler.

## How it is checked

The Verilog in `core/rtl` is checked in Verilator against joyrider3774's C emulator (`emu/`),
instruction by instruction and picture by picture (`sim/`, built with `sim/vbuild.sh`; set `OSSCAD` to
the OSS CAD Suite folder, default `$HOME/tools/oss-cad-suite`). Those tests need your own `.CHG` and
`.DAT` files, which are not included. The `.rbf` is built with Quartus 17.0.2 Lite from `core/`.
Version history: [CHANGELOG.md](CHANGELOG.md).

## Credits

- The emulator in `emu/` is joyrider3774's [chgame_emulator](https://github.com/joyrider3774/chgame_emulator) (MIT).
- CHGame is by Kevin Bates ([bateske/CHGame](https://github.com/bateske/CHGame), Apache-2.0); its bootloader (MIT) is included as the factory flash image.
- The MiSTer framework (`core/sys/`) is MiSTer-devel's Template_MiSTer (GPL).

See [NOTICE](NOTICE) for the full list.

## Licence

GPL-3.0-or-later; see [LICENSE](LICENSE). Third-party parts keep their own licences, see [NOTICE](NOTICE).
