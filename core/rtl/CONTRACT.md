# How the pieces of chg_soc fit together

Every RTL module of the board follows these rules, so that modules developed separately work
together and match the reference emulator (`emu/src`) cycle for cycle.

## 1. The reference

The specification is the C emulator in `emu/src`. That is joyrider3774's chgame_emulator at
bb9baf5 with the patches listed in `emu/PATCHES.md`. Where this file and the C disagree, the C
wins. `emu/src` is never edited except through the patches in `emu/PATCHES.md`; an apparent bug in
the C is reported with a reproducing case instead.

"Matches" means the same values, at the same board cycle, as the C model run **one cycle at a
time**: `chg_run(m, m->cycles + 1)` in a loop, as `arm/trace.c` does.

## 2. Time

- `clk` is the fabric clock (96 MHz on the MiSTer; in Verilator it is just the step).
- `ce` is one **board cycle** (48 MHz on the board). Board time `t` is the number of `ce`s since
  reset; the C model calls it `m->cycles`.
- **State at time t** means the state after exactly `t` clocks with `ce` high. Peripherals change
  state only on clocks with `ce` high, or in response to a bus access (below).
- The CPU holds `ce` low whenever it needs fabric clocks that are not board time: decoding, a
  BRAM read, waiting for a bus answer. Board time stops; nothing else may change.
- An event the C model schedules for cycle E (a timer update, a SysTick match, the end of an SPI
  frame) becomes visible **only at the first instruction boundary at or after E**: the C runs
  `bus_events` at the top of each `chg_run` loop, and `rv32.c` moves `m->cycles` on to the
  issue cycle (fetch and divider waits) before a load. So a load in an instruction that began
  before E does not see the flag, and a write in that window replans it. The games hit this about
  ten times a second.
- So the CPU outputs **`boundary`**: one clock, with `ce` low (or high: a merged decision, 6b), at every point where `chg_run`
  would run `bus_events`: the top of each instruction, before that instruction's access, and
  every cycle while asleep (WFI). Peripherals with events (SysTick, TIM1/2/3, SPI/DMA if their
  C works the same way) take it as an input and commit due events only on it. Tied high, it gives
  "visible at E", which does not match the C.
- **Where exactly `boundary` falls:** at the cycle of the `chg_run` loop top, which is the end of
  the previous instruction. That is before `cpu_take_interrupt`'s 1+FLASH_WORD and before the
  next instruction's fetch and divider waits, not at its issue cycle (SPI/DMA's SRAM writes
  would then be visible too early).
- **`busy`** (SPI/DMA): high, in the same clock as `boundary` or a GPIO port write, when SPI/DMA
  has work due (frame ends, DMA memory traffic through port B). While it is high, chg_soc holds
  `ce` low and the CPU neither makes its next access nor samples interrupts; lines settle when it
  falls. A `boundary` with nothing due costs no clock.
- **`gpio_wr`**: chg_soc gives SPI/DMA a one-clock strobe in the clock GPIO takes a port write
  (0x4001_0800-0x4001_13FF), with the pins still at their old levels in that clock: the C's
  `gpio_write` runs `spi_sync` first.
- **`stall[16:0]`**, valid with a peripheral's `ack`: board cycles the C adds to `m->cycles`
  inside the access (the flash controller: 96,000 for an erase, 48,000 for another STRT, 1,440
  per programmed store; the ADC: 50 per conversion). The CPU runs that many `ce`s right after the
  access, with no `boundary` in between, before the instruction goes on, as `m->cycles +=`
  inside the store does.

## 3. The peripheral bus (CPU to a peripheral)

One access at a time, with board time frozen at the access's cycle: the instruction's issue
cycle, which is `m->cycles` when `rv32.c` calls `load`/`store`. A read returns the state at that
time. A write takes effect at that time, before the next `ce`.

```
input         req      // one clock; the module is selected (address decoded by chg_soc)
input         we
input  [31:0] addr     // the full byte address, as bus_read_slow / bus_write_slow get it
input   [2:0] size     // 1, 2 or 4 bytes, as in the C
input  [31:0] wdata    // the value, right aligned, as bus_write_slow gets it
output [31:0] rdata    // the value, right aligned, as bus_read_slow returns it
output        ack      // one clock, 1 or more clocks after req; rdata valid with it
```

Peripherals decode their own offsets from `addr`, and do what `bus.c` does for that offset,
size and value: byte and halfword shifts, read-only bits, side effects of reads. Accesses below
0x4000_0000 (flash, SRAM) never reach a peripheral; chg_soc serves them.

## 4. Interrupt lines

Every peripheral drives the PFIC with **levels**, one bit per interrupt number as `machine.h`
numbers them (`CHG_IRQ_*`). A level is what `pfic_set_line` would be given at that time.
SysTick also gives `sw_pend`, a one-clock pulse with the ack of a CTLR write with SWIE set: the
C does `pfic.pending |= 1 << 14` there, so the PFIC ORs it into pending bit 14. The
PFIC (the CPU's part) keeps the enabled, pending and priority state, and takes interrupts only at
instruction boundaries, as `chg_run` does.

## 5. Pins

GPIO owns the pin levels. Peripherals that need a pin get it from chg_soc as a wire:
`gpio_pin(m, port, pin)` in the C. The ones used:

| Pin | What | Who reads it |
|---|---|---|
| PA4 | LCD CS (low = selected) | SPI/LCD |
| PB0 | LCD D/C | SPI/LCD |
| PB12 | LCD RESET (low = in reset) | st7735 |
| PB11 | SD CS (low = selected) | SPI/SD |
| PB9 | LED | chg_soc |
| PB10 | buzzer (GPIO, or TIM1 CH2 via AFIO) | audio |

Buttons come in as `buttons[7:0]` in `CHG_BTN_*` order, 1 = held, and GPIO turns them into
`ext_low`, as `chg_set_buttons` does.

## 6. Memory ports

chg_soc owns the flash (64 KB) and the SRAM (20 KB) as dual-port block RAM.

- **Port A** belongs to the CPU: instruction fetch and data accesses below 0x4000_0000.
- **Port B** is shared by the DMA, the flash controller and the loader, and the testbench can
  use it to peek.

Port B:

```
output        mreq     // one clock
output        mwe
output [31:0] maddr    // a flash (0x0000_0000 / 0x0800_0000 alias) or SRAM address
output  [2:0] msize    // 1, 2 or 4
output [31:0] mwdata
input  [31:0] mrdata
input         mack     // one clock; holds board time (chg_soc keeps ce low until it comes)
```

When a DMA write to SRAM becomes visible to the CPU is part of matching the C. Find out from
`spi_sync` and `dma_*` in `bus.c` when the C does it, and do the same.

## 6a. Registers added for 96 MHz

None of these changes board time; they only cost fabric clocks while `ce` is low.

- **`busy_q`**: SPI/DMA's `busy` without its combinational part. `ce` and the CPU use `busy_q`;
  the combinational part (`boundary` or `gpio_wr` with work due) is only high in a clock where
  `hold` is high anyway, and in the clock after it the registered part has taken over.
- **`irq_ev`**: SysTick/TIM (`p_irq_ev`) and SPI/DMA (`s_irq_ev`) say, with `boundary`, that an
  event is due there, so an interrupt line may change. The CPU registers the PFIC's lines
  (`irq_q`) and makes no interrupt decision in the clock after such a boundary, or after
  `busy_q` was high, since its copy of the lines may be a clock old then.
- **The peripheral bus** is registered once on the way out and once on the way back: `ack`
  reaches the CPU two clocks after the peripheral gives it. Board time is held meanwhile.
- **Port B** is registered on its way to the memories; `mrdata`/`mack` come a clock later.
  Board time is held until `mack`, as before.
- The CPU fetches ahead and releases `hold` early; that is invisible to board time, and
  `tb_cpu`, `tb_soc` and `tb_mister` check it cycle for cycle against the C.

## 6b. Fewer fabric clocks per board cycle

Again none of these changes board time or anything the C can see; they let `ce` come in more
clocks (STARLIFT, whole board, 0-3 s: 2.05 clocks a cycle before, 1.73 after without the T1
bypass; tb_mister's fast forward: 86% of real time before, 110% after).

- **Merged decision**: the boundary clock may also be the decision clock and a board cycle
  (`ce` high with `boundary`), when the CPU knows a clock ahead that the next clock is a boundary
  with a fast instruction ready in A (`mspec`), no peripheral event is due there (`irq_ev` low:
  the lines are still those of the clock before, `irq_q`), SPI/DMA have no sync work due there
  (`swork`) and the lines are settled (`stale` low). Whatever sees `boundary` must then take
  `ce` in the same clock as well: SysTick and the timers take only a match already pending (one
  found on that `ce` is the next boundary's), SPI's sync with nothing to do stamps `t` with
  `now` before the `ce` moves it.
- **`swork`** (chg_spi `work_q`): spi_sync's "something to do", registered a clock ahead (with
  `now + 1` when `ce`); valid in any clock after one in which SPI/DMA did nothing but count `ce`.
  SPI's `irq_ev` is registered too (it is only read with `boundary`, never the clock after a
  sync or an access).
- **SPI's common sync in five clocks**: one frame left, the next in the buffer, the transmit DMA
  refilling it, the receive DMA off, the card not selected: S_F1 .. S_FW in chg_spi, with the
  DMA's read asked for at once; the DMA sends an aligned SRAM/flash item straight to port B and
  does dma_take's work with the answer. Everything else goes through S_A .. S_END as before.
- **Bus loads** (LB .. LHU at 0x4000_0000 and up, not the PFIC: SysTick, SPI status and so on)
  go out from the instruction (S_BL0) and the slow path's timing sums run while the bus answers
  (S_BL1); a load with a stall goes on through AX_SLN and S_FIN as before.
- **T1's bypass**: when B moves to A as D1 moves to B, the first stage of T1 for D1 is made from
  the packet A is getting, so that the next instruction can follow a clock sooner.

## 6c. Fast card loading

Not part of board time: with the OSD's *Fast card loading* (status[92:91]: 0 Auto, 1 Always on,
2/3 Off) a card frame takes 1-2 board cycles instead of 16/32 (`chg_spi`, `fast_card`), and the
board runs unthrottled. A DMA write into SRAM makes the CPU refetch only when SRAM code is in the
pipeline. Auto uses the detector in `chg_soc`: six windows of 2^21 board cycles; fast while
the card delivered >= 32 blocks and the game sent < 4 RAMWR (0x2C) commands. The lockstep
testbenches leave `fast_card` 0 (cycle exact).

## 7. Files by area

| Area | Files |
|---|---|
| Integration | `core/rtl/chg_soc.sv`, `core/CHGame.sv`, `sim/tb_soc.cpp`, `sim/vbuild.sh`, this file |
| CPU | `core/rtl/cpu/*.sv`, `sim/tb_cpu*.cpp` |
| timers, GPIO, flash | `core/rtl/periph/*.sv` except SPI/DMA/SD, `sim/tb_periph*.cpp` |
| SPI, DMA, LCD, SD | `core/rtl/periph/chg_spi*.sv`, `chg_dma*.sv`, `chg_sd*.sv`, `core/rtl/st7735.sv`, `sim/tb_spi*.cpp` |

## 8. Style

- SystemVerilog, synthesisable by Quartus 17.0 (no interfaces, no classes, no `unique case` with
  side effects). Verilator `-Wall` clean apart from reviewed width warnings.
- `ce` qualifies every state change of board time. Reset is synchronous, active high.
- A comment at the top of each file says which C function it translates. LF line endings.
- No `float`/`real`. Counters that the C keeps as 64-bit cycle stamps become running counters
  in the RTL. That is the main translation work: the C computes lazily from stamps, the RTL counts.
