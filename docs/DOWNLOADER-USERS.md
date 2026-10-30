# Getting CHGame updates through Downloader

The MiSTer's Downloader (which *update_all* runs) can fetch the CHGame core along with
everything else. It isn't in update_all's menu, so you add it yourself, once, with a small
text file.

## Add it (once)

1. Take `downloader_chgame.ini` from the release zip (or make a text file with that name)
   holding these two lines:

   ```ini
   [jadflat/CHGameMiSTer]
   db_url = https://raw.githubusercontent.com/jadflat/CHGameMiSTer/db/db.json.zip
   ```

2. Put it in the top folder of the MiSTer's SD card, next to `downloader.ini`:
   `/media/fat/downloader_chgame.ini`. Use the network share (`\\MiSTer\sdcard\`), FTP,
   or take the card out. Save it with plain line endings if your editor asks (Notepad's
   default is fine).

3. On the MiSTer, open *Scripts* and run **update_all** (or **downloader**). It now fetches
   the newest CHGame core into `_Console/`.

Each later update_all run fetches a new CHGame core when there is one.

**Other way:** instead of the separate file, add the same two lines at the bottom of
`/media/fat/downloader.ini`. Keep the sections already in it.

## What it installs

- `_Console/CHGame_<date>.rbf`, the core. Only one CHGame core should be in `_Console`;
  update_all's tidy-up setting normally removes the old one (if two appear, delete the older
  date).
- The folder `games/CHGame/`, empty until you add games.

It installs no games, `.DAT` files or saves. Add your own `.CHG` and `.DAT` files to
`games/CHGame/` as before, or add the optional games below.

## Optional: the author's games

The core's author's free games can be installed the same way: at present CHix, STARLIFT
(with its `STARLIFT.DAT`) and CHarth. Each is downloaded from its own repository's release,
under its own licence. The list is rebuilt every day, so new games and new versions arrive
with the next update_all run. Add `downloader_chgame_games.ini` (in the release zip) to the top folder of the SD
card, or these two lines:

```ini
[jadflat/CHGameMiSTer_games]
db_url = https://raw.githubusercontent.com/jadflat/CHGameMiSTer/db/games.json.zip
```

The next update_all run puts the games (`CHix.CHG`, `STARLIFT.CHG`, `STARLIFT.DAT`,
`CHarth.CHG`, and any added later) in `games/CHGame/`, and updates them when new versions
are released. Remove the file to stop.
A game you already have under the same name is replaced.

## Remove it

Delete `/media/fat/downloader_chgame.ini` (or the two lines from `downloader.ini`). The
core already installed stays; delete `_Console/CHGame_<date>.rbf` too if you don't want it.

## If it doesn't work

- update_all says it can't fetch the database: check the two lines are copied exactly and
  that the MiSTer is online.
- No CHGame core appears: run update_all again and check `_Console/`. Its log names each
  database it read.
