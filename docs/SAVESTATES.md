# Save states and rewind: design notes

Status: not implemented. These notes describe a possible design, based on `core/` and
`core/rtl/CONTRACT.md`. Sizes in section 1 are counted by `tools/ss_audit.py`. Points marked
**(to verify)** depend on other MiSTer cores or on Main_MiSTer and must be checked before use.

## 0. Summary

- Total state about **1.11 Mbit (139 KB)**, audited by `tools/ss_audit.py`. 99% is memory: st7735
  frame memory 385 Kb, flash 512 Kb, SRAM 160 Kb. All registers together are 9.0 Kb.
- Approach: **a central save controller (`chg_ss.sv`) on a save bus.** Memories are copied through
  their spare ports; registers go out as a 64-bit word stream from a pack/unpack block in each
  module. The board is frozen (`ce` low) at a safe instruction boundary, streamed to DDR3, then
  released. A slot is 139 KB, 256 KB reserved. Save or load takes about 0.26 ms.
- Rewind is the same capture on a timer into a 64 MB ring in DDR3 (25 s at 0.1 s steps).
- Estimate: 1,000-1,600 ALMs (2.5-4%), 1-2 M10K. Effort about 8-10 days for save/load, 3-4 more
  for rewind, most of it testing. The CPU, SPI and DMA blocks come last (section 7).

## 1. State, module by module (bits)

Audited by `tools/ss_audit.py` (`python -I tools/ss_audit.py [--all]`): it reads every
`reg`/`logic` declaration of every module under `core/rtl`, evaluates the widths and array depths,
and classes each name from the manual pass in `tools/ss_classes.txt` (STATE saved as flops, MEM saved
through a second port, SCRATCH, NOSAVE, ROM). It lists any name the table does not cover, so the
table stays complete when the RTL changes (**UNCLASSIFIED must be empty**). Memories
made by `chg_dpram` instances (SRAM, the SD buffers) are listed by hand in the script. "Scratch"
(`*_q`, `n_*`, `w_*`, `t_*`, divider and FSM registers, read data, registered outputs, the CPU
pipeline) holds nothing at a safe point or is a function of the state.
The CPU, SPI and DMA rows should be re-counted whenever those modules change.

| Module | State (flops) | Memory | What |
|---|---:|---:|---|
| Flash (`chg_soc`, `flash[16384]` x 32) | | 524,288 | 64 KB |
| SRAM (`chg_soc`, `chg_dpram` 5120 x 32) | | 163,840 | 20 KB |
| st7735 | 253 | 384,912 | `gram` 132 x 162 x 18 (do not trim to 128 x 128); registers: cmd, nparam, p0/2/4, xs/xe/ys/ye/cx/cy, writing, pix0/1, npix, madctl, colmod, 6 flags, psl/pel/tfa/vsa/ssa. The read-out (`row`, `rstep`, `rs`, `rg`, `rend`, `rr`, `vrgb` ...: 180) is free-running |
| `chg_video`, `chg_throttle`, `chg_speed`, `chg_loader`, `chg_save`, `chg_card` | 0 | 0 | not saved (1.2): 1,500 bits of pacing, scan, HPS side |
| `chg_audio` | 78 | 4,608 | `cyc`, `high`, `level`, `dc` (40), `hpos`; `hist` 256 x 18. `coef` (4,608) is a ROM. The FIR pipeline (908) is not saved: a sample in flight at the freeze is lost (one 48 kHz sample) |
| SysTick | 294 | | ctlr, sr, cmp, cmp_m1, cnt, pre, weird, wrap_left, pend (the n_/w_/t_ copies and divider, 657, are scratch) |
| TIM1/2/3 | 3 x 307 = 921 | | 17 config registers of 16 bits, uif, cnt, pcnt, pend, oc2_active |
| GPIO A/B/C + AFIO | 585 | | cfg0/1/2, lckr (3 x 32 each), outdr (3 x 24), pcfr1, exticr0/1, actlr, bz_hold (`indr` is the pin input) |
| RCC | 384 | | rcc[0:10], flags |
| ADC | 833 | | adc[0:23] 768, noise, t, conv |
| Flash controller | 173 | 2,048 | actlr, ctlr, faddr, eop, key flags, locked, fast_locked, pdef (64), pdval; `pbuf` 64 x 32 |
| `chg_misc` | 528 | 8,704 | `usb` 256 x 32 (8,192) + `usb_ok` (256 valid bits); `io_tag` 16 x 16 (256), `io_dat` 16 x 32 (512), `io_ok` 16 (IO_WORDS=16) |
| PFIC | 357 | 2,048 | enabled, pending, ithresdr, sctlr, vtf_addr0..3, vtf_id, depth_r, nest_r; priority RAM `pr0..3` 4 x 64 x 8. The ready/best/scan registers (673) are a cache it rebuilds |
| SPI1 | 393 | | ctlr1/2, crcr, hscr, statr_sticky, tx/rx regs and flags, shift_data, shift_end (64), t (64), now (64), sd_selected, pin-edge flops |
| DMA1 | 1,051 | | per channel x 8: cfgr 15, cntr 16, paddr 32, cmaddr 32, reload 16, pos 16 (= 1,016); intfr 32; txp, rxp, swait. `m_*`, `mc_*`, the FSM (534) are in flight, not saved |
| microSD SPI side (`chg_sdspi`) | 350 | 8,192 | present, size (64), bstate/blba/bdrop x 2, wr_pend/busy, wbuf, wlba, idle, app, cmd0..4, ncmd, pre[6], npre, dk, pout, busy_resp, cur, reading, wstate, multi_write, nblk, lba, xs/xmosi/xout, xfrom_buf; `buff` 2 x 512 B |
| `chg_datcard` | 0 | | option 1 of 1.3: idle at a safe point; the file identity (len, nsec, name: 253) goes in the header check |
| chg_soc glue | 21 | | rst_level, lcd_reset, last_sd, stall_q (17), hold_bus |
| CPU (1.1) | 2,822 | 4,096 | see 1.1 |
| **Total** | **9,043** | **1,102,736** | |

Register flops are **9.0 Kb** , memories **1,102,736 bits (1.10 Mbit, 99% of the state)**. Packed in 64-bit words: memories
137,842 B, registers about 1.2 KB with per-block padding, 64 B header: **139.2 KB**, giving the 139 KB total. Slot content: 64 B header, the register blocks,
then SRAM, flash, gram, small RAMs. The `.DAT` in DDR3 is not saved (but see 1.3).

**Safe point.** Easiest and recommended: an instruction boundary with SPI/DMA idle:
`boundary && !busy_q && !hold && stall == 0`, flash controller idle (an erase or program stalls
up to 96,000 cycles), no divider running, no SD block loading. The frame state of SPI (`shift_end`,
`now`) is a function of board time, so a mid-frame save is exact if those are saved, but an
in-flight DMA memory access (`m_*`, `mack` pending) is not worth saving: wait for `busy_q` low.
The wait should be under 1 us. Step 8 settles it.

### 1.1 CPU

`chg_cpu.sv` audited: 2,822 flop bits and a 4,096-bit memory.
x1..x31 (992), pc, mstatus, mtvec, mscratch, mepc, mcause, mtval, intsyscr, corecfgr (8 x 32), `misc`
16 x 32 (512), the trap stack of 8 entries (`t_mepc`, `t_mcause`, `t_mst` 3 x 256, `t_irq` 48, `t_exc`,
`t_hpe`, `depth`: 1,081), wfi, lr_valid, lr_addr, rst_req, board cycle `cyc` and `icount` (64 each),
and the timing history `owe` (24), `frac`, `pair`, `recent0..2`, which must be confirmed as needed at a boundary. **The HPE stack is in the CPU**: `hpe_mem` 128 x 32 (4,096 bits), a memory with
its own write port, saved like the others . The other ~3,700 bits
(fetch-ahead, decode, B/A stage packets, load/store/CSR/AMO scratch) are dropped. At a safe point the
fetch-ahead and decode registers are dropped and the pc is re-fetched; that costs fabric clocks
only (CONTRACT 6a says the fetch-ahead is invisible to board time). Build the CPU block as one `ss_regs` list of (name, width).

### 1.2 What is not state

- The video scan and the glass read-out: free-running 15.625 kHz, tied to board time only through
  `gram`. After a load the next frame is complete and right. Compare pictures (4.5), not raster phase.
- Throttle/speed (real time; its `owed` counter absorbs the 0.26 ms freeze), the loader, the HPS
  side of `chg_save`. After a load, mark all save pages dirty once (one flag in `chg_save.sv`).

### 1.3 The card

`chg_datcard.sv` writes a game's writes to the `.DAT` sectors **back to DDR3**, so the `.DAT`
region does change. To verify: what happens to writes outside the file and what its `K_*`, `C_*`,
`cw_*`, `cr_*` registers hold. Options:

1. **State covers the SPI side only** (chg_sdspi plus the datcard's control registers, ~1 Kb). A
   game's card writes survive a load, as a real card would. Exact only if the game does not
   write the card. **Recommended for v1.**
2. Add copy-on-write: before each overwrite, log the old sector to DDR3; a state stores the log
   length; a load undoes back to it. ~150 ALMs, one extra DDR3 read per card write. Exact rewind.
3. Copy the whole file with each state: up to 64 MB. No.

A mounted card image (hps_io disk 1) is served by the ARM and cannot be rewound. The header holds
a hash of the flash at game load, the `.DAT` length and the mount state; a load is refused if they differ.

## 2. Getting the state out and back

**Chosen: central save bus. Memories through second ports, registers through a stream.**

```
ss_sel[k], ss_addr[15:0] (64-bit word in the block), ss_we, ss_wdata[63:0], ss_rdata[63:0]
```

Rejected: a **scan chain** through every flop (a mux on every D and the memories, 94%, cannot go on
it; worst for timing) and **reusing the CPU's peripheral bus** (reads have side effects, such as SPI
DATAR clearing flags; CONTRACT 3). The `ss` port has no side effects.

- Each module gets an `ss` port and a pack/unpack: `{a, b, c...}` read out as N words, written
  back in the same order. The load is the last priority in each flop's `always` (after reset).
- Memories are already dual-port (`chg_dpram`; chg_soc port B for flash and SRAM). The
  controller becomes one more owner of port B while frozen. gram's read port belongs to chg_video:
  lend it (blank or hold the picture for ~0.3 ms). PFIC priority RAM and audio `hist` have a port
  free while frozen.
- A fixed block table in `chg_ss.sv` (about 22 blocks), with a version in the header; a mismatch
  refuses the load.

DDR3 (64-bit words; byte address = `ddr_addr * 8`; chg_datcard forms `{4'b0011, 2'b00, daddr}`):

| Byte range | Use |
|---|---|
| 0x3000_0000-0x33FF_FFFF | `.DAT` (existing, up to 64 MB). **Not touched** |
| 0x3400_0000-0x35FF_FFFF | free, reserved |
| 0x3600_0000-0x3600_FFFF | control block: slot valid bits, ring head/tail, version |
| 0x3610_0000 + n x 0x4_0000, n = 0..3 | four user slots, 256 KB each (to 0x361F_FFFF) |
| 0x3800_0000-0x3BFF_FFFF | rewind ring, 64 MB = 256 entries x 256 KB |

Other cores keep their state area near 0x3C00_0000 and up **(to verify)**, and MiSTer's scaler
frame buffers live in the DDR3 window too **(to verify)**: confirm 0x3000_0000-0x3BFF_FFFF is
free in the Main_MiSTer release in use.

Cost: 139 KB is ~17,800 64-bit beats; with bursts and `DDRAM_BUSY` stalls about 25,000 clocks at
96 MHz = **0.26 ms**, a load the same. Board time does not advance meanwhile (`ce` low).

## 3. The OSD and MiSTer side

`core/sys/hps_io.sv` has `ioctl_upload`/`ioctl_upload_req`/`ioctl_din` (core to HPS
file upload), the SD block interface, `status`, `EXT_BUS`, and **nothing named `ss_` or savestate**.
`sys/` must stay unchanged, so:

- **A. DDR3 slots only (v1).** `CONF_STR` entries `O[..],Slot,1,2,3,4;` `R[..],Save state;`
  `R[..],Load state;` (status bits, like the existing `R[8],Save now`). `chg_ss` takes the edge and
  acts at the next safe point. States last while the core stays loaded (DDR3 may be cleared on core
  load **(to verify)**), not across power. A 1 s overlay `S2`/`L2` reuses chg_video's speed-display
  characters. About 20 lines in `CHGame.sv`.
- **B. Files (v2).** Main_MiSTer supports a `CONF_STR` line
  `SS<base>:<size>;` declaring a DDR3 window it copies to and from `savestates/CHGame/<game>_N.ss`,
  with a handshake through status bits or `EXT_BUS` **(to verify)**. Cores that do it (NES, SNES, Genesis) use a
  save bus `ss_adr/ss_din/ss_dout/ss_we/ss_req/ss_ack` into that window, halt the CPU at an
  instruction boundary, and give each register block a fixed address range. Section 2 follows that
  pattern so B is a wrapper later. Verify in `NES_MiSTer` `rtl/savestates.sv` and the Main_MiSTer
  source before choosing. If `EXT_BUS` is the route it is a `CHGame.sv` change only.
- A game or `.DAT` load clears the slot valid bits. OSD Reset does not.

## 4. Testing in Verilator

Use `sim/tb_mister.cpp` (+ `mister_top.sv`: real `hps_io`, loader, save, card, DDR3 model); the
lockstep against the C is `tb_soc.cpp`.

1. **Round trip.** Run game G, request a save at time T (many T: random, in a frame, during SPI
   bursts, in WFI, in a flash write). The controller stops at the next safe boundary B >= T. Keep
   running the unbroken core to B + N; log per boundary: board time, pc, instruction count, x1..x31
   (what tb_soc compares), and hashes of SRAM, flash and gram every 1000 boundaries.
2. **Load into a dirty core.** A second run goes to T2 far from T, loads, runs N: the trace from B
   on must equal 1, cycle for cycle. Also load into a freshly reset core.
3. **Against the C.** At B copy the C model's `Chg` struct and step it one cycle at a time from
   there (as tb_soc does): proves the state is complete, not just self-consistent. CHix and STARLIFT
   with its card.
4. **Mutation.** For each of the ~22 blocks, flip bits in the slot after the save; the trace must
   diverge. A bit the test cannot see is a bit it does not cover.
5. **Pictures.** The first complete 128 x 128 frame after the load equals the unbroken run's at the same board time.
6. **Per-module.** Extend `tb_periph_*` and `tb_spi` first: save a module, run, load into a second
   instance, compare outputs. Catches mistakes before the whole board exists.
7. **Sweep.** 1,000 random T per game, including SPI/DMA busy, a stalled flash erase and a `ce`-low
   window: the controller must wait, never save mid-instruction. Count the retries.
8. **Rewind.** 200 entries in the ring; step back k at random; compare with a run to that time.
9. Card test: snapshot the DDR3 `.DAT` region in the DDR3 model for option 2 of 1.3; otherwise
   use games that do not write the card during the test.

## 5. Rewind

- **Ring** in DDR3 at 0x3800_0000, 256 entries x 256 KB. Capture every N video frames of *board*
  time (800,000 board cycles = 1/60 s): N = 6 gives 10 captures/s, **25 s of rewind** at 0.1 s steps;
  N = 3 gives 12.8 s at 0.05 s.
- **Cost.** 0.26 ms per capture = 2.6 ms/s (0.26%), invisible; DDR3 write traffic 26 MB/s, small
  against the scaler's use. Board time is held, so the throttle sees the board late by 0.26 ms each
  time; `chg_throttle`'s `owed` counter pays it back (built for stalls), so pace does not drift.
- **Action.** A pad combo or MiSTer key held: every 6 frames load the previous entry (0.26 ms),
  show its picture, mute audio. On release run on from the entry, drop newer ones. The audio filter
  history is in the state, so sound restarts clean. Which input to use needs a check of what
  `joystick_0` carries **(to verify)**.
- **Cheap saving, v2.** A `flash_dirty` bit in `chg_flashctl` (set on any erase/program): if clear
  since the last capture, skip writing the flash (47% of the state), ring entries drop to ~75 KB
  and the ring holds 3.4x more. A frame-delta/RLE compression would reach minutes at ~400 ALMs: not
  before the plain version works.
- Card writes during a rewind: option 1 of 1.3 keeps them (the card is a real card); option 2 undoes them.

## 6. Cost and timing

### 6.1 Area (Cyclone V 5CSEBA6: 41,910 ALMs, 553 M10K = 5.5 Mbit; estimates)

| Piece | ALMs | M10K |
|---|---|---|
| chg_ss (FSM, DDR3 master, block table, header, hash) | 300-400 | 0-1 |
| pack/unpack and a load mux on ~9 Kb of flops (~1 ALM per 2-3 flops) | 400-600 | 0 |
| port-B owner for ss, gram read mux, PFIC/audio memory muxes | 100-200 | 0 |
| Rewind (ring pointers, timer, input, overlay) | 150-250 | 0 |
| DDR3 write FIFO 64 x 32 if the busy stalls need it | 20 | 1 |
| **Total** | **~1,000-1,600 (2.5-4%)** | **~1-2** |

No BRAM for the state itself: it is copied out of the RAMs that exist.

### 6.2 Timing at 96 MHz (slack is small; find the critical path in `output_files/*.sta.rpt`)

1. **The load mux on every saved flop** adds a LUT input to each D. Mitigations: the load comes
   last in the priority chain; do peripherals first (the CPU is the likely critical one); a
   multicycle constraint from the registered `ss_wdata` (stable for 4+ clocks, removes the source
   side only); if one path still fails, give that module a shadow register and one `ss_commit` clock.
2. **Fan-out of `ss_load`/`ss_sel`** to 9 Kb of flops: one registered strobe per module.
3. **DDR3 port** is shared with chg_datcard: arbiter with the card first (never at the same time:
   the card holds board time, the controller freezes it).
4. Place-and-route noise: build after every step. The area and timing figures here are estimates; the first real numbers come with the first build (step 7).

## 7. Step order

| # | Step | Size | Depends on CPU changes? |
|---|---|---|---|
| 1 | Audit: exact list of saved flops per module (script over `reg` + manual), scratch vs state; correct section 1 | 0.5 d | no |
| 2 | `chg_ss` skeleton, DDR3 slot writer/reader, tested against a fake 1 KB module in `tb_mister` | 1 d | no |
| 3 | Memory blocks: SRAM, flash, gram, usb, pbuf, PFIC priority RAM, audio hist, sdspi buff (new files first; the `chg_soc.sv` port-B wiring last, as a small diff) | 1.5 d | wiring only |
| 4 | Peripheral register blocks: SysTick, TIM x3, GPIO, RCC, ADC, flashctl, misc, audio, st7735, PFIC ; per-module test (4.6) | 2 d | no |
| 5 | Card, option 1 (control registers); option 2 later | 0.5-1 d | no |
| 6 | OSD entries, header, hash, slot table, refusal rules | 0.5 d | no |
| 7 | First Quartus build with a dummy CPU block: area and timing cost | 0.5 d + fixes | no |
| 8 | SPI, DMA, safe-point logic (`busy_q`, `gpio_wr`, flash stall, sdspi idle) | 1.5 d | yes |
| 9 | CPU block, `ss_halt/ss_ok` handshake, fetch-ahead flush | 2 d | yes |
| 10 | Whole-board tests 4.1-4.8, fix what the mutation test finds | 2-3 d | yes |
| 11 | Rewind: ring, timer, input, overlay, throttle | 2 d | yes |
| 12 | Optional: flash-dirty skip, delta compression, route B (files) | 2-4 d | yes |

Steps 1-7 do not depend on the CPU, SPI and DMA blocks; steps 8-11 do. Write new files
(`chg_ss.sv`, the `ss` wrappers) first and wire them into `core/rtl/cpu/`, `chg_spi.sv`,
`chg_dma.sv` and `chg_soc.sv` at the end, as a small diff.

## 8. Open questions

- Whether Main_MiSTer's savestate route (3B) needs more than `CONF_STR` and a DDR3 window.
- `chg_datcard.sv` write behaviour outside the file (1.3).
- `chg_misc` `io[]` backing: IO_WORDS=16 here (768 bits counted); the size grows if it does.
- Whether 0x3000_0000-0x3BFF_FFFF is really free of Main_MiSTer's own DDR3 users.
