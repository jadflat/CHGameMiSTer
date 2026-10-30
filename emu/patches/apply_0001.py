"""0001: keep the store-pairing and refetch-bubble state across cpu_exec calls.

rv32.c keeps pair_open and recent_data in locals of cpu_exec, so they reset
whenever cpu_exec returns (an event, a kick, the end of a front end's chunk).
Timing then depends on where the caller stops. The chip does not stop
between instructions, so the state is kept in ChgCpu instead.
Run from CHGameMiSTer/: python -I emu/patches/apply_0001.py
"""
import sys

def edit(path, pairs):
    s = open(path, newline='').read()
    nl = '\r\n' if '\r\n' in s else '\n'
    for old, new in pairs:
        old = old.replace('\n', nl)
        new = new.replace('\n', nl)
        if s.count(old) != 1:
            sys.exit(f'{path}: expected one match for {old!r}')
        s = s.replace(old, new)
    open(path, 'w', newline='').write(s)

edit('emu/src/machine.h', [(
    '    uint8_t  frac8;                 /* eighths of a cycle carried over */\n',
    '    uint8_t  frac8;                 /* eighths of a cycle carried over */\n'
    '    bool     pair_open;             /* CHGameMiSTer patch 0001: kept across cpu_exec calls */\n'
    '    uint8_t  recent_data;           /* CHGameMiSTer patch 0001 */\n')])
edit('emu/src/rv32.c', [
    ('    bool pair_open = false;     /* the last instruction was a RAM store that opened a pair */\n'
     '    unsigned recent_data = 0;   /* bit n: the instruction n back touched SRAM, from RAM code */\n',
     '    bool pair_open = c->pair_open;              /* the last instruction was a RAM store that opened a pair */\n'
     '    unsigned recent_data = c->recent_data;      /* bit n: the instruction n back touched SRAM, from RAM code */\n'),
    ('    c->frac8 = (uint8_t)frac;\n',
     '    c->frac8 = (uint8_t)frac;\n'
     '    c->pair_open = pair_open;               /* CHGameMiSTer patch 0001 */\n'
     '    c->recent_data = (uint8_t)recent_data;\n'),
])
print('0001 applied')
