"""Coefficient ROM for core/rtl/chg_audio_fx.sv -> core/rtl/audiofx_coefs.hex.

Run: python -I tools/eq_coefs.py

Biquads at 48 kHz from the RBJ cookbook, stored as the five numbers the
engine multiplies, b0 b1 b2 (-a1) (-a2), each divided by a0, in Q3.24
(27 bit two's complement, 7 hex digits). 512 entries, in sets of five:

  sets  0-3    pre-gain for the EQ: -3, -6, -9, -12 dB (b0 only)
  sets  4-66   EQ band b (0-6, 63 160 400 1k 2.5k 6.3k 16k Hz), gain g
               (0-8 = -12..+12 dB in 3 dB steps): set 4 + 9 b + g
  sets 67-78   sound models 2-7, two biquads each: 67 + 2 (m - 2) + stage
  sets 79-85   the analyser's band pass for each band
"""
import math
import os

FS = 48000.0
FREQS = [63, 160, 400, 1000, 2500, 6300, 16000]
EQ_Q = 1.4
BPF_Q = 1.4
ONE = 1 << 24


def norm(b0, b1, b2, a0, a1, a2):
    return [b0 / a0, b1 / a0, b2 / a0, -a1 / a0, -a2 / a0]


def peaking(f, db, q):
    A = 10 ** (db / 40)
    w = 2 * math.pi * f / FS
    al = math.sin(w) / (2 * q)
    c = math.cos(w)
    return norm(1 + al * A, -2 * c, 1 - al * A, 1 + al / A, -2 * c, 1 - al / A)


def lowpass(f, q):
    w = 2 * math.pi * f / FS
    al = math.sin(w) / (2 * q)
    c = math.cos(w)
    return norm((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al)


def highpass(f, q):
    w = 2 * math.pi * f / FS
    al = math.sin(w) / (2 * q)
    c = math.cos(w)
    return norm((1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al)


def bandpass(f, q):
    w = 2 * math.pi * f / FS
    al = math.sin(w) / (2 * q)
    c = math.cos(w)
    return norm(al, 0, -al, 1 + al, -2 * c, 1 - al)


def lowshelf(f, db, s=1.0):
    A = 10 ** (db / 40)
    w = 2 * math.pi * f / FS
    c, sn = math.cos(w), math.sin(w)
    al = sn / 2 * math.sqrt((A + 1 / A) * (1 / s - 1) + 2)
    t = 2 * math.sqrt(A) * al
    return norm(A * ((A + 1) - (A - 1) * c + t), 2 * A * ((A - 1) - (A + 1) * c),
                A * ((A + 1) - (A - 1) * c - t), (A + 1) + (A - 1) * c + t,
                -2 * ((A - 1) + (A + 1) * c), (A + 1) + (A - 1) * c - t)


def highshelf(f, db, s=1.0):
    A = 10 ** (db / 40)
    w = 2 * math.pi * f / FS
    c, sn = math.cos(w), math.sin(w)
    al = sn / 2 * math.sqrt((A + 1 / A) * (1 / s - 1) + 2)
    t = 2 * math.sqrt(A) * al
    return norm(A * ((A + 1) + (A - 1) * c + t), -2 * A * ((A - 1) + (A + 1) * c),
                A * ((A + 1) + (A - 1) * c - t), (A + 1) - (A - 1) * c + t,
                2 * ((A - 1) - (A + 1) * c), (A + 1) - (A - 1) * c - t)


def response(co, f):
    b0, b1, b2, na1, na2 = co
    z = complex(math.cos(2 * math.pi * f / FS), -math.sin(2 * math.pi * f / FS))   # z^-1
    return (b0 + b1 * z + b2 * z * z) / (1 - na1 * z - na2 * z * z)


def cascade_peak_db(stages):
    m = 0.0
    f = 20.0
    while f < 20000:
        h = 1.0
        for s in stages:
            h *= abs(response(s, f))
        m = max(m, h)
        f *= 1.02
    return 20 * math.log10(m)


# the sound models: two biquads each, as the bare pin through a small
# transducer; scaled so the response never rises above +6 dB (the bare pin at
# the default 50% volume cannot then clip)
MODELS = {
    2: ("Small piezo",  [highpass(700, 0.7), peaking(4000, 9, 2.5)]),
    3: ("Large buzzer", [highpass(400, 0.7), peaking(2500, 8, 2.0)]),
    4: ("Speaker",      [lowshelf(250, 4), lowpass(6000, 0.6)]),
    5: ("Warm",         [lowpass(3000, 0.707), lowshelf(200, 2)]),
    6: ("Handheld",     [highpass(500, 0.7), lowpass(5000, 0.7)]),
    7: ("Bright",       [highpass(100, 0.7), highshelf(3000, 6)]),
}


def q(v):
    n = int(round(v * ONE))
    assert -(1 << 26) <= n < (1 << 26), v
    return n & 0x7FFFFFF


def build():
    sets = []
    for k in range(1, 5):
        sets.append([10 ** (-3 * k / 20), 0, 0, 0, 0])
    for f in FREQS:
        for g in range(9):
            db = (g - 4) * 3
            sets.append([1, 0, 0, 0, 0] if db == 0 else peaking(f, db, EQ_Q))
    info = []
    for m in range(2, 8):
        name, st = MODELS[m]
        pk = cascade_peak_db(st)
        sc = 10 ** (min(0.0, 6.0 - pk) / 20)
        st = [list(s) for s in st]
        for i in range(3):
            st[0][i] *= sc
        sets += st
        info.append((m, name, pk, 20 * math.log10(sc)))
    for f in FREQS:
        sets.append(bandpass(f, BPF_Q))
    return sets, info


if __name__ == "__main__":
    sets, info = build()
    words = [q(v) for s in sets for v in s]
    assert len(words) <= 512
    words += [0] * (512 - len(words))
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "core", "rtl", "audiofx_coefs.hex")
    with open(path, "w", newline="") as f:
        for w in words:
            f.write("%07x\n" % w)
    print("%d sets, %d words -> %s" % (len(sets), len(sets) * 5, os.path.normpath(path)))
    for m, name, pk, sc in info:
        print("model %d %-12s peak %+.1f dB, scaled %+.1f dB" % (m, name, pk, sc))
