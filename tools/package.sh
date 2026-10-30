#!/bin/bash
# Packages the MiSTer core for release.
#
#   tools/package.sh VERSION            e.g. tools/package.sh 0.1
#
# Copies core/output_files/CHGame.rbf to dist/CHGame_<version>_<yyyymmdd>.rbf and makes
# dist/CHGameMiSTer_<version>.zip holding that .rbf, README.md, INSTALL.md, NOTICE, LICENSE, CHANGELOG.md, build/chgsdimg.exe (if present)
# and tools/savconv.py, at the same relative paths. It does not run Quartus: the .rbf must
# already be built.
#
# RBF overrides the .rbf path (for testing). Zips with `zip` if it is installed, else with
# Python's zipfile module (python3 or python).

set -u

usage() { echo "usage: $0 VERSION   (e.g. $0 0.1)" >&2; exit 2; }

[ $# -eq 1 ] || usage
VERSION=$1
case "$VERSION" in
    ''|*[!0-9A-Za-z._-]*) echo "error: VERSION must be letters, digits, dots, dashes or underscores" >&2; exit 2 ;;
esac

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT" || exit 1

RBF=${RBF:-core/output_files/CHGame.rbf}
if [ ! -f "$RBF" ]; then
    echo "error: $RBF is missing." >&2
    echo "The core has not been built. Build it in Quartus 17.0.2 Lite (core/CHGame.qpf);" >&2
    echo "the .rbf is written to core/output_files/. Nothing was packaged." >&2
    exit 1
fi

for f in README.md INSTALL.md NOTICE LICENSE CHANGELOG.md tools/savconv.py; do
    if [ ! -f "$f" ]; then
        echo "error: $f is missing; nothing was packaged." >&2
        exit 1
    fi
done

DATE=$(date +%Y%m%d)
DIST=dist
RBF_OUT="$DIST/CHGame_${VERSION}_${DATE}.rbf"
ZIP_OUT="$DIST/CHGameMiSTer_${VERSION}.zip"

mkdir -p "$DIST" || exit 1
cp "$RBF" "$RBF_OUT" || { echo "error: could not copy $RBF" >&2; exit 1; }

# Stage the zip's contents in a scratch folder, at the paths INSTALL.md names.
STAGE=$(mktemp -d) || exit 1
trap 'rm -rf "$STAGE"' EXIT
mkdir -p "$STAGE/tools" "$STAGE/arm"
cp "$RBF_OUT" "$STAGE/$(basename "$RBF_OUT")"
for f in README.md INSTALL.md NOTICE LICENSE CHANGELOG.md; do cp "$f" "$STAGE/$f"; done
cp tools/savconv.py "$STAGE/tools/savconv.py"
cp docs/downloader_chgame.ini docs/downloader_chgame_games.ini "$STAGE/"
# GAMES=folder: the author's games (tools/make_games_db.py lists them) into games/CHGame/
if [ -n "${GAMES:-}" ]; then
    mkdir -p "$STAGE/games/CHGame"
    cp "$GAMES/CHix.CHG" "$STAGE/games/CHGame/CHix.CHG"
    cp "$GAMES/Starlift.CHG" "$STAGE/games/CHGame/STARLIFT.CHG"
    cp "$GAMES/STARLIFT.DAT" "$STAGE/games/CHGame/STARLIFT.DAT"
    cp "$GAMES/CHarth.CHG" "$STAGE/games/CHGame/CHarth.CHG"
    for g in CHix Starlift CHarth; do cp "$GAMES/LICENSE-$g.txt" "$STAGE/games/CHGame/"; done
fi
if [ -f build/chgsdimg.exe ]; then
    mkdir -p "$STAGE/build"; cp build/chgsdimg.exe "$STAGE/build/chgsdimg.exe"
    HAVE_SD=1
else
    echo "note: build/chgsdimg.exe not present; the zip does not include it (build it with arm/build.sh pc)."
    HAVE_SD=0
fi

rm -f "$ZIP_OUT"
if command -v zip >/dev/null 2>&1; then
    (cd "$STAGE" && zip -q -r "$ROOT/$ZIP_OUT" .) || { echo "error: zip failed" >&2; exit 1; }
else
    PY=""
    for p in python3 python; do
        if "$p" -c 'import zipfile' >/dev/null 2>&1; then PY=$p; break; fi
    done
    if [ -z "$PY" ]; then
        echo "error: neither zip nor Python 3 is available to make $ZIP_OUT" >&2
        exit 1
    fi
    "$PY" - "$STAGE" "$ROOT/$ZIP_OUT" <<'EOF' || { echo "error: zipping failed" >&2; exit 1; }
import os, sys, zipfile
stage, out = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for d, _, files in sorted(os.walk(stage)):
        for name in sorted(files):
            full = os.path.join(d, name)
            z.write(full, os.path.relpath(full, stage).replace(os.sep, "/"))
EOF
fi

echo "$RBF_OUT"
echo "$ZIP_OUT"
if [ "$HAVE_SD" -eq 1 ]; then
    echo "zip contents: $(basename "$RBF_OUT"), README.md, INSTALL.md, NOTICE, LICENSE, CHANGELOG.md, build/chgsdimg.exe, tools/savconv.py"
else
    echo "zip contents: $(basename "$RBF_OUT"), README.md, INSTALL.md, NOTICE, LICENSE, CHANGELOG.md, tools/savconv.py"
fi
exit 0
