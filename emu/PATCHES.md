# Patches to the emulator copy in emu/src

`emu/src` is joyrider3774's chgame_emulator at bb9baf5 (MIT) with the patches below. They make it
a fixed target for checking the FPGA core. Each is applied by a script in `emu/patches/`.

## 0001: timing state kept across cpu_exec calls

`cpu_exec` kept `pair_open` (back-to-back SRAM stores pay 2 port cycles, then 1) and
`recent_data` (an SRAM access just before a branch hides a refetch cycle) in locals. They reset
whenever `cpu_exec` returned: at every event, every `chg_kick`, and at the end of every chunk
a front end asks for. So the timing depended on how the emulator was called. CHix run in 1/60 s
chunks and run one cycle at a time diverged at frame 48: a memset loop from RAM paid 2-2-3-2-3
cycles for its stores one way and 3-3-3-3 the other. STARLIFT diverged at frame 47.

The patch keeps both in `ChgCpu`. With it, `tools/stepcheck.c` finds CHix and STARLIFT identical
over 1,800 frames (30 s, with buttons pressed), chunked against stepped. On the chip the state
does not reset, so the chunked figures were the right ones; the patch makes every way of calling
the emulator give them.
