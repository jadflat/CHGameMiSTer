#!/usr/bin/env python
"""Make the optional games database for MiSTer Downloader / Update All.

  python -I tools/make_games_db.py GAMES_FOLDER --out db_out/

GAMES_FOLDER holds the release assets listed in GAMES below (download them
with `gh release download`). Writes <out>/games.json and <out>/games.json.zip;
push both to the `db` branch next to db.json.zip. Each file is downloaded by
the MiSTer from its own game's GitHub Release, so every game keeps its own
licence and repository. Schema: MiSTer-devel/Downloader_MiSTer
docs/custom-databases.md (v = 1).
"""
import argparse, hashlib, json, os, sys, time, zipfile

DB_ID = "jadflat/CHGameMiSTer_games"

# installed name: (repository, release tag, asset name)
GAMES = {
    "CHix.CHG":     ("jadflat/CHix",     "v1.2.3", "CHix.CHG"),
    "STARLIFT.CHG": ("jadflat/Starlift", "v0.24",  "Starlift.CHG"),
    "STARLIFT.DAT": ("jadflat/Starlift", "v0.24",  "STARLIFT.DAT"),
    "CHarth.CHG":   ("jadflat/CHarth",   "v1.0",   "CHarth.CHG"),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("games")
    ap.add_argument("--out", default="db_out")
    ap.add_argument("--timestamp", type=int, default=None)
    a = ap.parse_args()

    files = {}
    for name, (repo, tag, asset) in GAMES.items():
        path = os.path.join(a.games, asset)
        if not os.path.isfile(path):
            sys.exit(f"missing {path} (gh release download {tag} -R {repo} -p {asset})")
        data = open(path, "rb").read()
        files["games/CHGame/" + name] = {
            "hash": hashlib.md5(data).hexdigest(),
            "size": len(data),
            "url": f"https://github.com/{repo}/releases/download/{tag}/{asset}",
        }

    db = {
        "v": 1,
        "db_id": DB_ID,
        "timestamp": a.timestamp or int(time.time()),
        "files": files,
        "folders": {"games/": {}, "games/CHGame/": {}},
    }
    os.makedirs(a.out, exist_ok=True)
    js = os.path.join(a.out, "games.json")
    with open(js, "w", newline="") as f:
        json.dump(db, f, indent=2)
        f.write("\n")
    with zipfile.ZipFile(js + ".zip", "w", zipfile.ZIP_DEFLATED) as z:
        z.write(js, "games.json")
    print(f"wrote {js} and {js}.zip ({len(files)} files)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
