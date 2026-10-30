"""Save-state audit: every reg/logic/bit declaration of every module under core/rtl.

  python -I tools/ss_audit.py [--all]

Lists, per module, each declared register or array with its size in bits and a
class from the table below: STATE (saved), MEM (a memory saved through a second
port), SCRATCH (holds nothing at a safe point), NOSAVE (not part of the state,
docs/SAVESTATES.md 1.2) or ROM.  A name the table does not cover is listed as
UNCLASSIFIED: the manual pass is to make that list empty.  Output is written
with newline="" (LF).
"""
import os, re, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RTL = os.path.join(ROOT, "core", "rtl")

S, SC, M, N, ROM = "STATE", "SCRATCH", "MEM", "NOSAVE", "ROM"

# ---------------------------------------------------------------------------
# The manual pass.  Whole modules that are not state, then per module regexes
# on the register name, first match wins.  (The reasons are in
# docs/SAVESTATES.md section 1.)
# ---------------------------------------------------------------------------
MODULE_NOSAVE = {
    "chg_throttle", "chg_speed", "chg_loader", "chg_save", "chg_video", "chg_card",
    "chg_periph_sim", "chg_spi_tb_top", "chg_dpram", "chg_cpu_div",
}
RULES = {}

def R(mod, *rules): RULES[mod] = rules

RULES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ss_classes.txt")

def load_rules():
    """tools/ss_classes.txt: 'module  CLASS  regex' per line (# comments)."""
    if not os.path.exists(RULES_FILE): return
    with open(RULES_FILE, encoding="utf-8") as f:
        for ln in f:
            ln = ln.split("#", 1)[0].strip()
            if not ln: continue
            mod, cls, pat = ln.split(None, 2)
            RULES.setdefault(mod, [])
            RULES[mod] = list(RULES[mod]) + [(pat.strip(), cls)]

# memories made by instantiating chg_dpram or by a macro: not reg declarations,
# so listed by hand (module, name, width, depth, class)
EXTRA = [
    ("chg_soc", "sram (chg_dpram, WORDS=5120)", 32, 5120, M),
    ("chg_sdspi", "buff (chg_dpram AW=10 BYTES=1) x 2 blocks", 8, 1024, M),
]

# instances of a module (chg_periph_sim builds TIM1/2/3 from chg_tim); bits a table row over-counts
INSTANCES = {"chg_tim": 3}
ADJUST = {("chg_cpu", "xr"): -32}       # xr[0] stays 0
DROP = {"chg_dpram"}                    # the memory primitive: its instances are listed by hand (EXTRA)

def classify(mod, name):
    if mod in MODULE_NOSAVE: return N
    for pat, cls in RULES.get(mod, ()):
        if re.match(pat + r"$", name): return cls
    return None

# ---------------------------------------------------------------------------
def strip_comments(t):
    t = re.sub(r"/\*.*?\*/", " ", t, flags=re.S)
    return re.sub(r"//[^\n]*", "", t)

def split_top(s, sep=","):
    out, d, cur = [], 0, ""
    for ch in s:
        if ch in "[({": d += 1
        if ch in "])}": d -= 1
        if ch == sep and d == 0: out.append(cur); cur = ""
        else: cur += ch
    if cur.strip(): out.append(cur)
    return out

def clog2(x):
    n = 0
    while (1 << n) < x: n += 1
    return n

def ev(expr, env):
    e = expr.strip()
    e = re.sub(r"\$clog2\s*\(", "clog2(", e)
    e = re.sub(r"\d*'[sS]?[dD]\s*(\d+)", r"\1", e)
    e = re.sub(r"\d*'[hH]\s*([0-9a-fA-F_]+)", lambda m: str(int(m.group(1).replace("_", ""), 16)), e)
    e = re.sub(r"\d*'[bB]\s*([01_]+)", lambda m: str(int(m.group(1).replace("_", ""), 2)), e)
    try:
        return int(eval(e, {"clog2": clog2}, dict(env)))
    except Exception:
        return None

def width(rng, env):
    if not rng: return 1
    if not re.match(r"\[\s*.+?\s*:\s*.+?\s*\]$", rng.strip()):
        return None
    m = re.match(r"\[\s*(.+?)\s*:\s*(.+?)\s*\]$", rng.strip())
    if not m: return None
    a, b = ev(m.group(1), env), ev(m.group(2), env)
    return None if a is None or b is None else abs(a - b) + 1

def depth_of(dims, env):
    d = 1
    for r in re.findall(r"\[[^\]]*\]", dims):
        m = re.match(r"\[\s*(.+?)\s*:\s*(.+?)\s*\]$", r)
        if m:
            a, b = ev(m.group(1), env), ev(m.group(2), env)
            if a is None or b is None: return None
            d *= abs(a - b) + 1
        else:
            n = ev(r[1:-1], env)
            if n is None: return None
            d *= n
    return d

def scan(path):
    txt = strip_comments(open(path, encoding="utf-8", errors="replace").read())
    mods = []
    for mm in re.finditer(r"\bmodule\s+(\w+)(.*?)\bendmodule\b", txt, flags=re.S):
        name, body = mm.group(1), mm.group(2)
        env = {}
        for pm in re.finditer(r"\b(?:parameter|localparam)\b\s*(?:\[[^\]]*\]\s*)?([^;]*?)(?=;|\)\s*\()", body, flags=re.S):
            for part in split_top(pm.group(1)):
                m = re.match(r"\s*(?:\w+\s+)?(\w+)\s*=\s*(.+)$", part, flags=re.S)
                if m:
                    v = ev(m.group(2), env)
                    if v is not None: env[m.group(1)] = v
        b2 = re.sub(r"\bfunction\b.*?\bendfunction\b", " ", body, flags=re.S)
        b2 = re.sub(r"\btask\b.*?\bendtask\b", " ", b2, flags=re.S)
        decls, seen = [], set()
        pats = [r"(?m)^\s*(?:reg|logic|bit)\b([^;]*);",
                r"(?m)^\s*(?:output\s+|input\s+)?(?:output\s+)(?:reg|logic)\b([^,;\)\n]*)"]
        for pat in pats:
            for dm in re.finditer(pat, b2):
                rm = re.match(r"\s*(?:signed\s+)?((?:\[[^\]]*\]\s*)*)(.*)$", dm.group(1), flags=re.S)
                rng, names = rm.group(1), rm.group(2)
                w = 1
                for one in re.findall(r"\[[^\]]*\]", rng):
                    x = width(one, env)
                    w = None if (x is None or w is None) else w * x
                for nm in split_top(names):
                    nm = re.sub(r"=.*$", "", nm, flags=re.S).strip()
                    m = re.match(r"(\w+)\s*(.*)$", nm, flags=re.S)
                    if not m: continue
                    n, dims = m.group(1), m.group(2)
                    d = depth_of(dims, env) if dims.strip() else 1
                    if n in seen: continue
                    seen.add(n)
                    decls.append((n, w, d))
        mods.append((name, decls))
    return mods

def main():
    show_all = "--all" in sys.argv
    load_rules()
    mod_tot, unclassified, unresolved = {}, [], []
    for dp, dn, fn in os.walk(RTL):
        for f in sorted(fn):
            if not (f.endswith(".sv") or f.endswith(".v")) or f.startswith("pll"): continue
            for name, decls in scan(os.path.join(dp, f)):
                rows = []
                for n, w, d in decls:
                    if w is None or d is None: unresolved.append("%s.%s" % (name, n)); continue
                    cls = classify(name, n)
                    if cls is None:
                        cls = "UNCLASSIFIED"
                        unclassified.append("%s.%s(%d)" % (name, n, w * d))
                    bits = (w * d + ADJUST.get((name, n), 0)) * INSTANCES.get(name, 1)
                    rows.append((n, w, d, bits, cls))
                mod_tot[name] = (os.path.relpath(os.path.join(dp, f), ROOT).replace("\\", "/"), rows)
    for m in DROP: mod_tot.pop(m, None)
    for mod, nm, w, d, cls in EXTRA:
        mod_tot[mod][1].append((nm, w, d, w * d, cls))
    cn = [S, M, SC, N, ROM, "UNCLASSIFIED"]
    out = ["%-16s %-28s" % ("module", "file") + "".join("%13s" % c for c in cn)]
    grand = {c: 0 for c in cn}
    for name in sorted(mod_tot):
        path, rows = mod_tot[name]
        t = {c: 0 for c in cn}
        for r in rows: t[r[4]] += r[3]
        for c in cn: grand[c] += t[c]
        out.append("%-16s %-28s" % (name, path.replace("core/rtl/", "")) + "".join("%13d" % t[c] for c in cn))
    out.append("%-16s %-28s" % ("TOTAL", "") + "".join("%13d" % grand[c] for c in cn))
    out.append("")
    out.append("saved: STATE %d + MEM %d = %d bits = %.1f KB" % (
        grand[S], grand[M], grand[S] + grand[M], (grand[S] + grand[M]) / 8192.0))
    for name in sorted(mod_tot):
        path, rows = mod_tot[name]
        sel = [r for r in rows if show_all or r[4] in (S, M, "UNCLASSIFIED")]
        if not sel: continue
        out.append("")
        out.append("== %s (%s)" % (name, path))
        for n, w, d, bits, cls in sel:
            out.append("   %-12s %-16s %6d x %-6d = %8d" % (cls, n, w, d, bits) + (" (x%d instances)" % INSTANCES[name] if name in INSTANCES else ""))
    if unresolved:
        out.append("")
        out.append("width not resolved (count by hand): " + ", ".join(unresolved))
    if unclassified:
        out.append("")
        out.append("UNCLASSIFIED (add to tools/ss_classes.txt): " + ", ".join(unclassified))
    with open(sys.stdout.fileno(), "w", newline="", closefd=False, encoding="utf-8") as o:
        o.write("\n".join(out) + "\n")

if __name__ == "__main__":
    main()
