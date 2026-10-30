# Changelog

All notable changes to CHGameMiSTer, newest first.

## 0.5 (2026-10-30)

First public beta. Every game tested runs at full speed on a DE10-Nano, over HDMI and on a
15 kHz analogue monitor.

### Added

- `LICENSE`: GPL-3.0-or-later.
- The author's games CHix, STARLIFT and CHarth (MIT) in the release zip and through the
  optional `downloader_chgame_games.ini`.

### Changed

- Documentation rewritten for public release.

### Removed

- `tools/make_mgl.py`.

## 0.4.5 (2026-10-30)

### Added

- Automatic fast loading screens. When the card has delivered at least 32 blocks and the game has
  sent fewer than 4 frames to the display in the last quarter second of board time, card frames
  take 1 board cycle and the board runs unthrottled while the card is selected (Ethan's Critters'
  loading screen: about 28 s down to about 14 s). It ends by itself when the game draws again,
  with no catch-up. Timing is not cycle exact while it runs (sound pitch may rise); the lockstep
  testbenches switch it off.
- OSD option *Fast card loading*: Auto (default), Always on (fast whenever the card is selected,
  as in 0.4.4) or Off (exact).

### Notes

- Meets 96 MHz timing.

## 0.4.4 (2026-10-30)

### Added

- *Fast card loading* (main page, on by default): card frames take 1 board cycle and the board
  runs unthrottled while the card is selected. Ethan's Critters' loading screen takes about 14 s
  (was 28 s). Off is cycle exact.

### Changed

- Ethan's Critters' title screen needs about 1.83 fabric clocks per board cycle in simulation
  (was 2.24): a DMA write into SRAM no longer makes the CPU refetch unless SRAM code is in the
  pipeline; the card, DMA and SPI register paths are quicker; the divider skips leading zeros.
- *Load data file* copies the `.DAT` straight into DDR3 (the OSD entry carries a load address),
  then the core moves it into place with bursts. Ethan's Critters (44 MB) loads in a second or two
  (was 39 s).
- LED dot moved down 3 pixels; speed display moved up 3 pixels.

### Known issues

- The core clock meets 96 MHz; the HDMI scaler clock from the MiSTer template misses by 0.123 ns
  in the slow 85 C corner.

## 0.4.3 (2026-10-30)

### Changed

- Faster CPU: a load and its user, taken branches and JALR cost fewer clocks. New Blocks on the Kid
  needs about 1.9 fabric clocks per board cycle in simulation (was 2.3), about 105% of real time;
  STARLIFT 1.69 (was 1.81), CHix 1.63 (was 1.73). Still cycle exact with the emulator.

### Notes

- Meets 96 MHz timing.

## 0.4.2 (2026-10-29)

### Added

- Saves: the game waits for its `.sav` (mounted after the `.CHG` download) and is restored before
  it runs. OSD entry *Reload save*.
- Video page: *Per-game position* (on by default). V and H position are kept in the save (block 0,
  bytes 48-50, marker 0xA5) and restored on load.
- LCD grid strength (Off, Light, Medium, Dark, Black) and style (Lines, Rows only, Columns only,
  Dots); these replace the on/off grid setting.

### Changed

- Card reads about 3.5 times faster (10 fabric clocks a byte, was about 36). Ethan's Critters'
  title screen needs about 2.26 clocks per board cycle in simulation (was about 2.74).
- OSD organised into pages: Main (files, save, speed), Video, Audio, Equaliser.
- Speed display moved down 12 pixels; the LED dot moved up.

### Notes

- Meets 96 MHz timing.

## 0.4.1 (2026-10-29)

### Changed

- LCD look: the ghosting option, which showed only in 2x, is removed; the option is now Off / Grid.

### Fixed

- A game's first save is now written. MiSTer creates a new, empty `.sav` for a game that was never
  saved, and the core only wrote to a file that was already 64,000 bytes.
- The picture is black until the first game loads (the panel showed white before it was set up).

### Known issues

- Misses 96 MHz by 0.128 ns in the slow 85 C corner.

## 0.4 (2026-10-29)

### Added

- Board clock: 100% (real), 110%, 125%, 150%, 175%, 200%. Heavy games top out at about 110%; the
  speed display shows the real figure. Pitch rises with the clock.
- Sound: a 7-band equaliser (63 Hz to 16 kHz, -12 to +12 dB) on its own OSD page, with presets
  (Bass boost, Treble boost, Loudness, Vocal, Custom).
- Spectrum analyser: Off, While in menu, Always, with optional peaks, drawn under the OSD menu.
  *While in menu* keeps the game running with the menu open.
- *Pause when OSD open* (default Off).
- LCD look (Grid, in 2x), LCD colours (an approximation of the panel) and an LED dot.

### Changed

- Faster CPU: STARLIFT needs about 1.73 fabric clocks per board cycle (was 2.05), 100% of real
  time in simulation (was 86-96%), up to 110% with fast forward or a higher board clock. Still
  cycle exact with the emulator.
- More timing margin: the flash-to-SRAM address path lost its adder.

### Notes

- Meets 96 MHz timing.

## 0.3.5 (2026-10-29)

### Added

- Analogue modes 15 kHz 1.75x (224 x 224) and 240 x 240 (1.875x), alongside 1x and 2x. The mode
  is now a two-bit setting, so it starts at 1x once.
- Scale filter for the 1.75x and 240x240 modes: Off, Even rows, Sharp bilinear, Soft.
- Sound: eight types (Bare pin, Piezo filter, Small piezo, Large buzzer, Speaker, Warm, Handheld,
  Bright).
- Several `.DAT` files on the card (up to 8, 64 MB) and *Clear data files*.
- An extra pad button for Up.

### Fixed

- Display: the ST7735 scroll arithmetic was too narrow for out-of-range values.

### Notes

- Meets 96 MHz timing.

## 0.3.1 (2026-10-29)

### Changed

- Same as 0.3, rebuilt to meet 96 MHz timing. The OSD still shows v0.3.

## 0.3 (2026-10-29)

### Added

- H position: 20 left to 20 right, in the OSD.

### Known issues

- Misses 96 MHz by 0.049 ns in the slow 85 C corner.

## 0.2 (2026-10-29)

### Added

- Analogue modes 15 kHz 1x and 15 kHz 2x (both 60 Hz).
- V position: 15 up to 15 down.
- *Load data file*: a `.DAT` from the games folder is loaded into DDR3 and shown to the game as a
  FAT32 card.
- Fast forward (OSD and the FF button). The gain is small in heavy games, which are at the CPU's
  limit.
- Speed display: the board's speed as a percentage of real time.

### Notes

- All 38 games checked are identical to the emulator in whole-board simulation.

## 0.1 (2026-10-28)

First core. Meets 96 MHz timing.

### Added

- `.CHG` loading, autosave to `.sav`, microSD image mounting (*Mount microSD image*), and NES-style
  pad mapping with A/B and Select/Start swapped.
- HDMI output and 15 kHz analogue output.

## Possible later

- Update All database entry.
- Save states and rewind (design notes in `docs/SAVESTATES.md`).
