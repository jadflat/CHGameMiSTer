#!/usr/bin/env python
"""Make the optional games database for MiSTer Downloader / Update All.

  python -I tools/make_games_db.py --out db_out/ [--download DIR] [--previous games.json]

Finds the games itself: every public, non-fork, non-archived repository of
OWNER whose latest (non-pre-)release has a .CHG asset and whose repository has
a LICENSE file. A repository with the topic `no-mister`, or listed in EXCLUDE,
is left out. Each .CHG is installed under its cart name (the package header's
title) and each .DAT under its asset name, in games/CHGame/. The files are
downloaded by the MiSTer from each game's own release, so every game keeps its
own licence and repository.

Writes <out>/games.json and <out>/games.json.zip (push both to the `db`
branch). With --download, the files (under their installed names) and each
game's licence as LICENSE-<repo>.txt are also kept in DIR, for the release
zip. With --previous, the old timestamp is kept when nothing changed.

Uses the GitHub CLI (`gh`), logged in or with GH_TOKEN set.
Schema: MiSTer-devel/Downloader_MiSTer docs/custom-databases.md (v = 1).
"""
import argparse, hashlib, json, os, subprocess, sys, tempfile, time, zipfile

OWNER = "jadflat"
DB_ID = "jadflat/CHGameMiSTer_games"
EXCLUDE = {"CHGameMiSTer"}
BAD = '<>:"/\\|?*'


def gh(*args):
    r = subprocess.run(["gh", *args], capture_output=True)
    if r.returncode:
        return None
    return r.stdout


def gh_json(path):
    out = gh("api", path)
    return json.loads(out) if out else None


def cart_name(data):
    if len(data) < 64 or data[:4] != b"CHG1":
        return None
    t = data[32:64].split(b"\0")[0].decode("ascii", "replace")
    t = "".join(c for c in t if c >= " " and c not in BAD).strip().rstrip(".")
    return t or None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="db_out")
    ap.add_argument("--download", help="also keep the files and licences here")
    ap.add_argument("--previous", help="the current games.json, to keep its timestamp")
    a = ap.parse_args()

    repos = gh_json(f"users/{OWNER}/repos?type=owner&per_page=100") or sys.exit("GitHub API failed")
    files, notes = {}, []
    tmp = tempfile.mkdtemp()
    for repo in sorted(repos, key=lambda r: r["name"].lower()):
        name = repo["name"]
        if repo["private"] or repo["fork"] or repo["archived"] or name in EXCLUDE:
            continue
        if "no-mister" in repo.get("topics", []):
            notes.append(f"{name}: topic no-mister, left out")
            continue
        rel = gh_json(f"repos/{OWNER}/{name}/releases/latest")
        if not rel:
            continue
        assets = [x for x in rel["assets"] if x["name"].upper().endswith((".CHG", ".DAT"))]
        if not any(x["name"].upper().endswith(".CHG") for x in assets):
            continue
        lic = gh_json(f"repos/{OWNER}/{name}/license")
        if not lic:
            notes.append(f"{name}: no LICENSE file, left out")
            continue
        for x in assets:
            path = os.path.join(tmp, name + "_" + x["name"])
            if gh("release", "download", rel["tag_name"], "-R", f"{OWNER}/{name}",
                  "-p", x["name"], "-O", path, "--clobber") is None:
                sys.exit(f"download failed: {name} {x['name']}")
            data = open(path, "rb").read()
            inst = x["name"]
            if inst.upper().endswith(".CHG"):
                inst = (cart_name(data) or os.path.splitext(inst)[0]) + ".CHG"
            key = "games/CHGame/" + inst
            if key in files:
                key = f"games/CHGame/{os.path.splitext(inst)[0]} ({name}){os.path.splitext(inst)[1]}"
            files[key] = {
                "hash": hashlib.md5(data).hexdigest(),
                "size": len(data),
                "url": x["browser_download_url"],
            }
            notes.append(f"{name} {rel['tag_name']}: {x['name']} -> {key}")
            if a.download:
                os.makedirs(a.download, exist_ok=True)
                with open(os.path.join(a.download, os.path.basename(key)), "wb") as f:
                    f.write(data)
        if a.download:
            text = gh("api", f"repos/{OWNER}/{name}/contents/LICENSE",
                      "-H", "Accept: application/vnd.github.raw")
            if text:
                with open(os.path.join(a.download, f"LICENSE-{name}.txt"), "wb") as f:
                    f.write(text)

    stamp = int(time.time())
    if a.previous and os.path.isfile(a.previous):
        old = json.load(open(a.previous))
        if old.get("files") == files:
            stamp = old.get("timestamp", stamp)
    db = {
        "v": 1,
        "db_id": DB_ID,
        "timestamp": stamp,
        "files": files,
        "folders": {"games/": {}, "games/CHGame/": {}},
    }
    os.makedirs(a.out, exist_ok=True)
    js = os.path.join(a.out, "games.json")
    with open(js, "w", newline="") as f:
        json.dump(db, f, indent=2)
        f.write("\n")
    with zipfile.ZipFile(js + ".zip", "w", zipfile.ZIP_DEFLATED) as z:
        info = zipfile.ZipInfo("games.json", time.gmtime(stamp)[:6])
        z.writestr(info, open(js, "rb").read(), zipfile.ZIP_DEFLATED)
    print("\n".join(notes))
    print(f"wrote {js} and {js}.zip ({len(files)} files)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
