# Installing and using the CHGame core

CHGame is a handheld with a CH32 RISC-V chip, a 128 x 128 screen, a piezo buzzer and a microSD
card. This core plays its `.CHG` games in an FPGA on a MiSTer board (any DE10-Nano based system,
such as the SuperStation One). It does not need the MiSTer SDRAM add-on board.

Credits: the emulator this core is checked against is joyrider3774's
[chgame_emulator](https://github.com/joyrider3774/chgame_emulator) (MIT). CHGame itself is by
Kevin Bates (Apache-2.0). The MiSTer framework is MiSTer-devel's Template_MiSTer (GPL).

## What you need

- A MiSTer with a working SD card and network access (or the card's reader).
- The core file: `CHGame_YYYYMMDD.rbf` (from the release zip, or built as described in README.md).
- Your `.CHG` games. Some games also need a `.DAT` data file (for example STARLIFT needs
  `STARLIFT.DAT`).
- For a PC: `tools/savconv.py` (Python 3) and, to make a card image, `chgsdimg` (see below).

## 1. Copy the core

Put the `.rbf` in the `_Console` folder on the MiSTer's SD card.

```
/media/fat/_Console/CHGame_20261028.rbf
```

Keep only one CHGame `.rbf` in that folder. The dated name tells you which version it is; to
change versions, replace the file rather than adding a second one.

## 2. Copy the games

Put the `.CHG` files in the core's games folder:

```
/media/fat/games/CHGame/
```

This follows the MiSTer convention `/media/fat/games/<core name>/`. The OSD's *Load game* entry
lists the `.CHG` files in that folder.

The core loads a game by streaming the `.CHG` into its flash, and it drops the 512-byte header
of the package. A game that is not a CHGame package will not start.

## 3. Games that need a `.DAT` file

Some games read a `.DAT` file from a microSD card (STARLIFT needs `STARLIFT.DAT`, DANKTOMB
`DANKTOMB.DAT`, ANOTHERW `ANOTHERW.DAT`, RASTAN `RASTAN.DAT`, Ethan's Critters `CRITTERS.DAT`).
There are two ways to give the core its card. The first is the simple one.

### The usual way: Load data file

Put the `.DAT` files in the games folder, `/media/fat/games/CHGame/`, beside the `.CHG` games.
In the OSD choose *Load data file* and pick the game's `.DAT`, then *Load game* and pick the game.
The order does not matter. A game that needs several `.DAT` files (or a card shared by several
games) gets them all: *Load data file* once for each.

- The core copies the `.DAT` into the MiSTer's memory (DDR3) and shows the game a microSD card
  that holds those files: a 128 MB FAT32 card labelled `CHGAME`, laid out as the PC
  emulator's card for a folder holding the same files (directory entries in name order, the
  files one after another). During play nothing is read from the
  MiSTer's SD card or the network, so a slow share does not matter.
- The load is quick: Main_MiSTer copies the file straight into memory at the speed of the SD card
  (the OSD entry carries a load address), then the core moves it into place. Ethan's Critters
  (44 MB) takes a second or two, not half a minute. The OSD may show "Loading" for that time.
- Loading a `.DAT` while a game runs starts the game again, as putting a card into a real
  CHGame would need.
- **Several files.** Each *Load data file* adds its file to the card: up to 8 files, 64 MB in
  all. Load `STARLIFT.DAT`, then `DANKTOMB.DAT`, and the card holds both. A file whose name (see
  below) is already on the card replaces that file; if it was the last one loaded it goes into
  the same place (loading the same big file again always fits), otherwise the old copy's room
  in memory stays used until *Clear data files*. A ninth name, or a file with no room left, is
  ignored (the board still restarts). A file too long for what is left is cut off at the end of
  memory.
- **Clear data files** (a button right under *Load data file*) forgets all the files: the card
  is taken out of the game's slot and the board restarts. Load the ones you want again.
- The files stay loaded when you load another game or reset, until you clear them. Loading the
  core again forgets them too.
- The file's name on the card comes from the file itself, because the MiSTer tells the core
  only a file's contents, not its name. CHGame `.DAT` files begin with their own name
  (`STARLIFT...`, `DANKTOMB...`, `ANOTHERW...`, `RASTAN` and zeros), so the card shows
  `STARLIFT.DAT` and so on, whatever the file is called on the MiSTer. Ethan's Critters' file
  begins `ECRTCARD`; the core knows that one and calls it `CRITTERS.DAT`. A file that begins
  with neither is shown as `CARD.DAT`, which no game opens.
- A game that writes to the card (none of the games above do: CHGame's SD driver only reads)
  writes into the copy in memory. Nothing is written back to the `.DAT` file.

### The other way: a card image

The core can also mount one FAT32 image file through the OSD's *Mount microSD image* entry (the
`S1` line). This is the way to give a game a card it keeps what it writes on, or one with more than 8 files. Make the image on the PC from a folder that holds the files:

```
chgsdimg folder CHGAME.img [size_mb]
```

- `folder` is the folder holding the `.DAT` files, such as `STARLIFT.DAT`.
- `CHGAME.img` is the image written. The volume label is `CHGAME`.
- `size_mb` is optional. Without it the image is big enough for the files (at least 128 MB).
- Build the tool from a Windows PC with `./arm/build.sh pc` (zig is needed). It writes
  `build/chgsdimg.exe`. The release zip carries `build/chgsdimg.exe` only if that file exists in
  the `build` folder when the zip is made.

Copy the `.img` to `/media/fat/games/CHGame/` and mount it before you start a game that needs a
`.DAT`. The core reads and writes the image, so keep a copy on the PC. The ARM reads the image
for the core block by block while the game runs; a game that reads its card a lot
may slow down if the image is on a slow network share. The emulator's own
`sdcard_make_image` builds the same kind of image, so a game sees the same card here as on the
PC.

Whichever went in last is the card in the slot. Mounting an empty image slot (ejecting the
image) brings back the loaded `.DAT` files, if there are any.

## 4. Saves

- The core keeps a game's flash pages in a save file. The game saves with its own in-game
  save; the core writes the save out.
- The MiSTer convention for a core's save is `/media/fat/saves/CHGame/` with the game's name and
  `.sav` (for example `STARLIFT.sav`). The save file is always 64,000 bytes (125 blocks of 512
  bytes), with a `CHGS` header.
- The save is mounted automatically. The game entry is `FS1` in the OSD menu
  (`core/CHGame.sv`, `CONF_STR`), and the `S` in it tells the MiSTer to mount
  `saves/CHGame/<game>.sav` as virtual disk 0 when a game is loaded. No menu entry is needed for
  it. After the first save, `saves/CHGame/` holds the file. Use the OSD's *Save now* before you
  turn off.
- The OSD options:
  - **Autosave** (On or Off): when On, the core saves the game's pages as they change.
  - **Save now**: writes the save at once.
  - **Reload save**: reads the mounted save again, with the game held in reset, and restarts the game.
    A save mounted while the game runs is not read by itself; use this.
  - **Per-game position** (Video page, On or Off): see below.
- The save is read before the game starts: the core holds the game until the .sav MiSTer mounts
  after the download has been read (up to half a second of waiting; with no save, the game starts then).
- With *Per-game position* On (the default), the save also keeps the game's V and H position (block 0, bytes
  48-50), and loading the game sets the OSD's V and H position to them. Changing either position makes the
  next autosave (or *Save now*) write the new values. A game with no stored position keeps the current ones.
  Off: the save is read and written without positions.

### Moving saves between the PC emulator and the MiSTer

The PC emulator (chgame_emulator) and the core keep saves in different formats. Convert them with
`tools/savconv.py` (Python 3):

```
python -I tools/savconv.py to-mister  STARLIFT.sav  STARLIFT_mister.sav
python -I tools/savconv.py to-emu     STARLIFT_mister.sav  STARLIFT.sav
```

- `to-mister` turns an emulator save into the 64,000-byte core save. Copy the result to
  `/media/fat/saves/CHGame/` with the name the core uses for that game.
- `to-emu` does the reverse, for a save the core wrote, so you can carry it back to the PC.
- Keep a copy of the save before you convert. The script does not check that the game matches.

## 5. Video

The core makes a native 15 kHz RGB picture: 240p timing, 6 MHz pixels, so the 128 x 128 picture
is square on a 4:3 tube, centred with a black border. A CRT takes this through the analogue
output. For HDMI, MiSTer's scaler takes the picture.

The OSD's *Video* page has these options (from `core/CHGame.sv`):

| Option | Choices | What it does |
|---|---|---|
| Analogue mode | 15kHz 1x, 15kHz 1.75x, 15kHz 240x240, 15kHz 2x | All four are 15.625 kHz, 60 Hz. 1x: the 128 x 128 picture at its own size. 1.75x: 224 x 224, each pixel 1.75 wide and 1.75 tall (rows 2, 2, 2, 1 lines), nothing cut. 240x240: each pixel 1.875, 240 x 240, nothing cut, fills the 240 lines. 2x: each pixel doubled (256 x 240, the top and bottom 4 rows are cut), much bigger on a CRT. V and H position work in all four |
| Scale filter | Off, Even rows, Sharp bilinear, Soft | For 1.75x and 240x240 only. Off: plain repeated rows. Even rows: the single-line row of each group gets a 50/50 mix with the row above, so it looks as tall as the others. Sharp bilinear: the second line of each two-line row is mixed with the next row by the exact share of the line it covers. Soft: Sharp bilinear plus a light blend (25% of the neighbour) on the first and last quarter-pixel of each glass pixel. The speed display and the spectrum are drawn unfiltered |
| V position | 0, 1-15 down, 15-1 up | Moves the picture on a CRT up or down by up to 15 lines |
| H position | 0, 1-20 right, 20-1 left | Moves the picture on a CRT left or right by up to 20 pixels |
| Per-game position | On, Off | On: V and H position are kept in each game's save (with the save, see Saves) and restored when the game loads. Off: one position for all games |
| Aspect ratio | Original, Full Screen, [ARC1], [ARC2] | Original is 1:1 (square pixels) |
| Scale | Normal, V-Integer, Narrower HV-Integer, Wider HV-Integer | Integer scaling for HDMI |
| Scandoubler Fx | None, HQ2x, CRT 25%, CRT 50%, CRT 75% | Scanline and filter effects; they apply when the scandoubler is in use (a 31 kHz VGA monitor, or MiSTer.ini forces it). Not in 1.75x and 240x240 (their lines are 3072 samples long: HDMI gets 1792 or 1920 samples a line, the scaler does the rest) |

Gamma, the scaler's filters, and shadow masks come from MiSTer's own menu and `MiSTer.ini`.

### LCD grid, colours and LED dot

More options on the *Video* page (all start off; with all of them off the picture is exactly
what it was before, only two pixel periods later with its sync, which nothing can see):

| Option | Choices | What it does |
|---|---|---|
| LCD grid (2x only) | Off, Light, Medium, Dark, Black | The dark lines between the handheld panel's pixels: the last column and/or line of each 2x2 block is dimmed to 90% (Light), 75% (Medium), 50% (Dark) or 0% (Black). It needs a whole number of output pixels per game pixel, so it works in *15kHz 2x* only |
| Grid style | Lines, Rows only, Columns only, Dots | *Lines*: rows and columns (a block's corner is dimmed twice). *Rows only*: the horizontal lines. *Columns only*: the vertical lines. *Dots*: only the corner pixel of each block |
| LCD colours | Off, On | An approximation of the ST7735 panel's colours: a little less saturated and contrasty, black lifted a little, whites slightly cool. An approximation, not a measurement. It is applied inside the picture only, never to the black border |
| LED dot | Off, On | A small dot in the picture's bottom left corner for the board's LED: red when the game lights it, dark grey when it does not (the board's PB9). In 2x it is cut off if V position moves the picture far enough up or down |


### Fast forward and the speed display

Three options on the main OSD page (none is saved in the game; they start at Normal, Off and 100%):

- **Speed**: *Normal* or *Fast forward*. Fast forward lets the board run as fast as the FPGA's
  CPU can; the game runs faster and the sound is muted. Heavy games (STARLIFT) are already
  at the CPU's limit, so they gain little; light ones gain more. Back to
  *Normal* it carries on at real speed straight away (it does not race to catch up).
  The pad's **FF** button (the 9th button; map it in the OSD's button mapping) fast-forwards
  while it is held.
- **Board clock**: *100% (real)*, *110%*, *125%*, *150%*, *175%*, *200% (max)* (starts at 100%,
  which is the board as it always was). The board's clock runs at 48 MHz times this, like an
  overclocked handheld. The throttle is exact (no drift) as long as the FPGA's CPU keeps up; it
  needs about 1.6 to 1.8 FPGA clocks per board cycle, so about 110% is the most heavy games can
  reach and light ones get more. Where it cannot keep up the board just runs as fast as it can
  (like fast forward) and the speed display shows the real figure. Games time themselves with the
  board's own cycles (SysTick), so they simply run faster. The sound follows board time too: the
  buzzer pin is driven by the board, so the pitch (and the tempo) rise with the clock; that is
  expected, as on an overclocked handheld. The sound path itself is untouched. Changing the
  setting does not make the board race to catch up. *Speed* (fast forward) and Pause win over it.
- **Speed display**: *Off* or *On*. *On* draws a small white-on-black percentage in the
  picture's top right corner, the board's speed as a percent of real time (100% is real time),
  refreshed twice a second. It also shows how fast fast forward is.

### Loading screens

When a game is loading from the card with a loading screen (few frames drawn, many card blocks read),
the core speeds the load up by itself and goes back to exact real time the moment the game draws
again. **Fast card loading** (main OSD page): *Auto* (default) is this; *Always on* runs the board
unthrottled with short card frames whenever the card is selected (it can slow games that stream from
the card while playing); *Off* keeps everything exact. Sound may sound higher while it runs.

### Pause when OSD open

**Pause when OSD open**: *Off* or *On* (main OSD page, starts *Off*, so the core behaves as it did
before this option; most MiSTer cores that have it are best left *On*, so set it once and the
MiSTer remembers it in its config). *On* stops the board while the OSD menu is open: game time
stands still, the sound is muted, the picture stays. Closing the menu carries on at real speed
at once (the time spent in the menu is not made up). The pad's fast forward does nothing while paused.
The pause lasts only while the menu itself is open, not for the small info boxes the MiSTer
shows. Downloads still go on, and loading a game or `.DAT` from the menu restarts the board
as before.

## 6. Sound

- **Sound** (the speaker model): the first two are as before, the rest are filters on the bare pin.
  - *Bare pin*: the buzzer pin as the emulator plays it (the default; with the Equaliser off and the
    Spectrum off the sound is exactly what it was before these options).
  - *Piezo filter*: a filter fitted to a recording of the board's piezo.
  - *Small piezo*: weak bass (high pass at 700 Hz) and a sharp resonance near 4 kHz.
  - *Large buzzer*: high pass at 400 Hz and a softer resonance near 2.5 kHz.
  - *Speaker*: fuller, a bass shelf and a gentle low pass at 6 kHz.
  - *Warm*: a low pass at 3 kHz with a little bass.
  - *Handheld*: a band pass, 500 Hz to 5 kHz, like a small console speaker.
  - *Bright*: bass cut below 100 Hz and treble up 6 dB above 3 kHz.
  The models are scaled so that none of them rises more than 6 dB, and at the default 50% volume the
  buzzer's square wave does not clip.
- **Volume**: 50%, 75%, 100%, 150%, 25%, or Off.
- **Spectrum**: Off, While in menu, or Always. Seven bars (63, 160, 400 Hz, 1, 2.5, 6.3, 16 kHz) of
  what you hear, 3 dB a step from about -48 dB (green, yellow, red), drawn over the picture just
  below the MiSTer menu, centred, so you can watch them while you set the equaliser. *While in
  menu* shows them only while the OSD is open (and the game keeps running then, even with *Pause
  when OSD open*, so there is sound to see). **Spectrum peaks** adds white marks that fall slowly.
  The bars are at glass rows 90-105 of 128 and scale with the Analogue mode; the menu's bottom edge
  is at about row 80 (1x, 2x, HDMI at 720p and 1080p) or 85 (3x), so there is room below it. If
  MiSTer's high resolution OSD (twice as tall) is on, the menu will cover the top of the bars.
- **EQ preset** (on the *Audio* page, with Sound, Volume and the spectrum options) and the **Equaliser** page: the preset Off, Bass boost, Treble boost, Loudness, Vocal, or
  Custom. Custom uses the seven band settings on the *Equaliser* page: 63 Hz, 160 Hz, 400 Hz, 1 kHz, 2.5 kHz, 6.3 kHz
  and 16 kHz, each 0 dB (the default) or -12 to +12 dB in 3 dB steps. The presets work whatever the
  bands show. The equaliser lowers the whole sound by its biggest boost (so a +12 dB band gives a
  sound 12 dB quieter everywhere else) to keep it from clipping; turn the volume up to make up.

The sound goes out through HDMI or the analogue audio, as the MiSTer does for any core.

## 7. Controls

The pad works like an NES pad with each pair of buttons the other way round:

| Pad | CHGame |
|---|---|
| D-pad | D-pad |
| A | B |
| B | A |
| Select | START |
| Start | SELECT |
| Up (an extra button, unmapped on an NES pad: X by default; map it in the OSD) | Up, as well as the d-pad's Up |

The OSD's button names (`J1` in `core/CHGame.sv`) are the CHGame's, in that order.

Reset is in the OSD: *Reset* restarts the game; *Reset and close OSD* does the same and closes
the menu.

## 8. Troubleshooting

- The core is missing from the menu: check the `.rbf` is in an `_` folder on the card and that
  the file name ends in `.rbf`.
- *Load game* lists no games: check the `.CHG` files are in `/media/fat/games/CHGame/`.
- A game does not start, or stops at a card read: load its `.DAT` with *Load data file* (or
  check that its `.DAT` is in the `.img` and the image is mounted), then load the game again.
- The save does not come back: see section 4. Compare `saves/CHGame/` after a save.
- A picture is not square on a CRT: check the core's 15 kHz output and the aspect ratio option.

## Getting files onto the MiSTer

- **Network (SMB)**: enable Samba from the MiSTer's Scripts menu (`samba_on`), then open
  `\\MiSTer\sdcard\` in Windows Explorer. The MiSTer's wiki names the share as `sdcard`. The
  default MiSTer login is user `root` with password `1`, unless you have changed it.
- **FTP and SSH**: the MiSTer runs both out of the box; see the MiSTer's Network Access
  documentation for the connection details.
- **SD card**: take the card out and copy on a PC. The card's partition is exFAT, labelled
  `MiSTer`, which most systems read.

Copy the `.rbf`, the `.CHG` games, the `.DAT` files (or the `.img`) and any save. Power the MiSTer off
properly before you take the card out.
