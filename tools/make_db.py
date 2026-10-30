#!/usr/bin/env python
"""Make the MiSTer Downloader / Update All database for the CHGame core.

  python -I tools/make_db.py dist/CHGame_0.5_YYYYMMDD.rbf --out db_out/
  options: --repo jadflat/CHGameMiSTer   GitHub owner/repo (also the db_id)
           --tag v0.5                    release tag (default v<version>)
           --db-id ID                    default: the repo

Writes <out>/db.json and <out>/db.json.zip (the zip holds db.json). Push both
to the `db` branch; users point downloader.ini at db.json.zip there.

The core is installed as _Console/CHGame_<yyyymmdd>.rbf (the MiSTer menu and
the downloader's old-core clean-up both expect Name_YYYYMMDD.rbf), so each
release replaces the previous dated file. The .rbf itself is downloaded from
the GitHub Release asset (its dist name keeps the version).
Schema: MiSTer-devel/Downloader_MiSTer docs/custom-databases.md (v = 1).
"""
import argparse, hashlib, json, os, re, sys, time, zipfile


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rbf")
    ap.add_argument("--out", default="db_out")
    ap.add_argument("--repo", default="jadflat/CHGameMiSTer")
    ap.add_argument("--db-id", default=None)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--timestamp", type=int, default=None)
    a = ap.parse_args()

    name = os.path.basename(a.rbf)
    m = re.fullmatch(r"CHGame_(.+)_(\d{8})\.rbf", name)
    if not m:
        sys.exit("error: expected a name like CHGame_0.5_YYYYMMDD.rbf, got " + name)
    version, date = m.groups()
    tag = a.tag or "v" + version
    with open(a.rbf, "rb") as f:
        data = f.read()
    if not data:
        sys.exit("error: empty .rbf")

    installed = "_Console/CHGame_%s.rbf" % date
    db = {
        "v": 1,
        "db_id": a.db_id or a.repo,
        "timestamp": a.timestamp or int(time.time()),
        "files": {
            installed: {
                "hash": hashlib.md5(data).hexdigest(),
                "size": len(data),
                "url": "https://github.com/%s/releases/download/%s/%s" % (a.repo, tag, name),
            }
        },
        "folders": {"_Console/": {}, "games/CHGame/": {}},
    }
    os.makedirs(a.out, exist_ok=True)
    text = json.dumps(db, indent=2) + "\n"
    with open(os.path.join(a.out, "db.json"), "w", newline="") as f:
        f.write(text)
    with zipfile.ZipFile(os.path.join(a.out, "db.json.zip"), "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("db.json", text)
    print("wrote %s/db.json and db.json.zip (%s -> %s, v%s)" % (a.out, name, installed, version))


main()
