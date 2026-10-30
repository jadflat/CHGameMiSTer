#!/usr/bin/env python3
"""Renames .CHG games to the cart name in their header, so MiSTer's file list
shows "Another World" rather than ANOTHERW.CHG.

  python -I tools/name_games.py GAMES_FOLDER [--saves SAVES_FOLDER] [--go]

The name is the package header's title (32 bytes at offset 32). Without --go it
only prints what it would do. A game's save is named after its file, so with
--saves (e.g. the MiSTer's saves/CHGame) a matching .sav is renamed with it.
Characters a file name can't hold are dropped; two carts with the same name get
the old file name added in brackets. .DAT files are left alone: the core names
them on the card from their contents.
"""
import argparse
import os
import sys

BAD = '<>:"/\\|?*'


def title(path):
    with open(path, "rb") as f:
        h = f.read(512)
    if len(h) < 64 or h[:4] != b"CHG1":
        return None
    t = h[32:64].split(b"\0")[0].decode("ascii", "replace")
    t = "".join(c for c in t if c >= " " and c not in BAD).strip().rstrip(".")
    return t or None


def rename(old, new):
    # through a temporary name, so a change of case only also works on Windows shares
    tmp = old + ".renaming"
    os.rename(old, tmp)
    os.rename(tmp, new)


def main():
    ap = argparse.ArgumentParser(description="Rename .CHG files to their cart names.")
    ap.add_argument("games")
    ap.add_argument("--saves", help="the saves folder, to rename each game's .sav with it")
    ap.add_argument("--go", action="store_true", help="rename (without it, only show)")
    a = ap.parse_args()

    files = sorted(f for f in os.listdir(a.games) if f.upper().endswith(".CHG"))
    names = {f: title(os.path.join(a.games, f)) for f in files}
    count = {}
    for t in names.values():
        if t:
            count[t.lower()] = count.get(t.lower(), 0) + 1

    plan = []
    for f, t in names.items():
        stem = os.path.splitext(f)[0]
        if not t:
            print(f"{f}: no cart name in its header, left alone")
            continue
        new = t if count[t.lower()] == 1 else f"{t} ({stem})"
        if new == stem:
            continue
        if new.lower() != stem.lower() and os.path.exists(os.path.join(a.games, new + ".CHG")):
            print(f"{f}: {new}.CHG already exists, left alone")
            continue
        plan.append((stem, new))

    for stem, new in plan:
        print(f"{stem}.CHG -> {new}.CHG")
        if a.saves and os.path.exists(os.path.join(a.saves, stem + ".sav")):
            print(f"    save: {stem}.sav -> {new}.sav")
    if not a.go:
        print(f"\n{len(plan)} to rename. Nothing changed: add --go to rename.")
        return 0

    for stem, new in plan:
        rename(os.path.join(a.games, stem + ".CHG"), os.path.join(a.games, new + ".CHG"))
        if a.saves:
            s = os.path.join(a.saves, stem + ".sav")
            if os.path.exists(s):
                rename(s, os.path.join(a.saves, new + ".sav"))
    print(f"\n{len(plan)} renamed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
