// The QingKe V4C core of the CH32X035: RV32IMAC, WCH's XW compressed byte
// and halfword loads and stores, the hardware prologue/epilogue (HPE), and
// (in chg_pfic) the interrupt controller.
//
// Translates cpu_exec, fetch, load, store, trap_enter, exception, mret,
// cpu_take_interrupt, csr_read, csr_write and cpu_reset of emu/src/rv32.c,
// the CPU's part of chg_run in emu/src/machine.c (interrupts taken at
// instruction boundaries, WFI sleep, the reset request), and the part of
// bus_read_slow/bus_write_slow that is not a peripheral (misaligned accesses
// split into bytes, flash and SRAM, the PFIC's address range).
//
// The design (built for 96 MHz on the Cyclone V).
//
// Board time runs in 'owe': every ce takes one off it and adds one to the
// cycle counter (m->cycles). Each instruction is worked out completely while
// board time still pays for the ones before, so that its own decision clock
// only looks at the interrupt lines and copies registered results:
//
//   front end   fpc, a word buffer and port A: one instruction into IR, in
//               program order. It follows a JAL or a backward branch as soon
//               as RR sees it (static guess); anything else it gets wrong is
//               put right when the instruction moves into A.
//   DEC         IR -> D1: chg_cpu_decode.
//   RR          D1 -> B: the operands, from the register file, the pending
//               write (wr_*), or the results of B or A (b_res, a_res: one-
//               cycle operations and multiplies). It waits for a load or a
//               slow instruction it depends on.
//   B           EX (ALU, branch, address; a multiply over two clocks, in the
//               DSPs) and T1, the timing to issue (cpu_exec's prefetcher loop
//               and divider wait), in two stages, from the state A leaves
//               (A's packet) or, with A empty, the committed state.
//   B -> A, T2  when A is empty (the instruction before has committed): A's
//               packet, what cpu_exec's loop body leaves (cost, the flash/
//               SRAM flags, the branch refetch) as registers: k_E, k_fw,
//               k_fd, k_div, k_frac, k_pair, k_rec, and whether A is "fast"
//               (ALU, LUI/AUIPC, MUL*, jumps, branches, FENCE, WFI, an
//               aligned load from SRAM or flash, an aligned store to SRAM).
//               Every sum is made for each case, the case picked last.
//   decision    the boundary clock (boundary out, ce low), then the decision
//               of chg_run's loop: reset request, an interrupt (chg_pfic's
//               scan, skipped when it would turn down the same interrupts
//               again), WFI sleep, or execute. A fast instruction commits in
//               that clock: registers copied, a load's read made on port A,
//               and owe = k_E. The register write follows a clock later
//               (wr_*), a load's three clocks later (RR forwards it), and a
//               store's write a clock later (st_*: its cycles are still
//               running, so nothing can read the word before).
//
// The decision clock is also the instruction's first board cycle when A is
// fast and ready at the boundary ("spec": hold is let go a clock early, so
// ce comes in the decision clock). Everything the decision can choose then
// costs at least one cycle and does not depend on which of its cycles the
// board is in: executing A (owe = k_E - 1), or taking an interrupt (its
// 1 + FLASH_WORD less one, 'dce'); a pick that finds nothing executes A.
// Spec is not used with WFI, a reset request, a refetch pending or after an
// interrupt entry (nochk), nor for slow instructions (their access must be
// made at the issue cycle). So a fast instruction of k_E cycles takes k_E + 1
// fabric clocks when the pipeline keeps up (2 to 3 when k_E is 1).
//
// The boundary clock itself can be the decision clock and a board
// cycle (mgo, CONTRACT.md 6b): when the clock before knew that A would be
// fast and ready (mspec), and no peripheral event or SPI/DMA work is due at
// the boundary. And T1 for the instruction moving into B is started from the
// packet A is getting (the bypass), so B can move a clock sooner.
//
// Everything else (DIV, AMO, LR/SC, CSRs, ECALL/EBREAK/illegal, MRET, bus
// and misaligned accesses) goes through the slow path, S_EX0 .. S_FIN2: the
// multi-cycle core with its long sums split over more clocks. A fetch
// outside flash and SRAM (the 0x08000000 alias, the last halfword of a
// region) is made after the decision, through AX_*, as fetch() does with
// bus_read_slow, and the instruction goes down the pipeline again with no
// second boundary (nochk). A sleeping cycle (WFI) throws the pipeline away.
//
// Between any two registers there is at most one adder or compare and a few
// LUTs. The interrupt lines are registered on the way in (irq_q, and the
// PFIC's masked ORs of them): a decision waits a clock after a boundary at
// which a peripheral had an event due (irq_ev) and after SPI/DMA was busy,
// so that they are the lines of now. T1 is two clocks; the PFIC's scan
// reduces a word in two clocks and folds it in a third.
//
// The C keeps fetch_time and div_ready as 64-bit cycle stamps; here they are
// kept relative to the next instruction boundary, which keeps them small
// (see 'fd' below).
//
// What chg_soc must give it:
//   - port A of the flash and SRAM BRAMs, one clock read latency, byte
//     enables on SRAM writes. Flash writes go out on the bus (to the flash
//     controller), as bus_write_slow sends them to flash_mem_write.
//   - mem_b_wr: a pulse whenever port B writes flash or SRAM (only while
//     busy is high or inside an access: never with ce in a decision clock).
//     Everything fetched is fetched again before the next instruction runs.
//   - stall with the ack: board cycles the access took on top (the C's
//     peripherals add to m->cycles: an ADC conversion 50, a flash halfword
//     1,440, a page program 48,000, an erase 96,000). Board time runs them
//     off (owe) right after the access, before the instruction goes on.
//   - boundary is given to the peripherals: one clock with ce low at the top
//     of every instruction (and every cycle asleep), where chg_run runs
//     bus_events; the decision is made the clock after (or later), so that
//     it sees the irq lines as the events left them. None is given between
//     an interrupt entry and the handler's first instruction, nor after a
//     software reset, as chg_run gives none.
//   - sw_pend from SysTick, irq_line levels from everyone (changing only at
//     a boundary or with an access).
//   - busy from SPI/DMA, raised with boundary (or with a GPIO write's ack)
//     while its spi_sync work is due. While it is high the core waits: no
//     decision of chg_run's loop and no access is started.
//   - sys_reset: a pulse, chg_reset(m, false), for the peripherals.

module chg_cpu
(
	input              clk,
	input              reset,           // power-on: chg_reset(m, true) for the CPU and PFIC
	input       [31:0] entry,           // m->entry: where a reset starts

	output             hold,            // high: board time must not advance
	input              ce,              // one board cycle (only while hold is low)
	input              busy,            // SPI/DMA has work due (CONTRACT.md 2): no access, no interrupt decision
	input              irq_ev,          // an irq_line may change at this boundary (a peripheral has an event due)
	input              swork,           // SPI/DMA would be busy at a boundary in this clock (registered; 0: never)

	output reg  [13:0] fl_addr,         // flash port A, word address
	input       [31:0] fl_rdata,
	output reg  [12:0] ram_addr,        // SRAM port A, word address
	output reg   [3:0] ram_we,
	output reg  [31:0] ram_wdata,
	input       [31:0] ram_rdata,
	input              mem_b_wr,
	input              mem_b_ram,       // with mem_b_wr: the write was to SRAM (code is fetched from SRAM rarely: see code_ram)

	output reg         bus_req,
	output reg         bus_we,
	output reg  [31:0] bus_addr,
	output reg   [2:0] bus_size,
	output reg  [31:0] bus_wdata,
	input       [31:0] bus_rdata,
	input              bus_ack,
	input       [16:0] stall,           // with bus_ack: board cycles the access took on top (CONTRACT.md 2)

	input       [63:0] irq_line,
	input              sw_pend,
	output reg         sys_reset,       // one clock: m->reset_request taken

	output             boundary,        // one clock, ce low: where chg_run runs bus_events (CONTRACT.md 2)
	output reg         retire,
	output reg  [31:0] retire_pc,
	output reg  [63:0] retire_instret,
	output reg  [63:0] retire_cycles,
	output      [63:0] cycles,          // m->cycles
	output      [63:0] instret,

	output reg         faulted,         // the first HardFault (ChgCpu.faulted and friends)
	output reg  [31:0] fault_cause,
	output reg  [31:0] fault_pc,
	output reg  [31:0] fault_tval,

	// for the testbench
	output    [1023:0] dbg_x,
	output      [31:0] dbg_pc,
	output      [31:0] dbg_mstatus,
	output      [31:0] dbg_mepc,
	output      [31:0] dbg_mcause,
	output       [3:0] dbg_depth,
	output             dbg_wfi,
	output      [17:0] dbg_fw,
	output      [23:0] dbg_fd,
	output       [4:0] dbg_div_rem,
	output       [5:0] dbg_div_rd,
	output       [2:0] dbg_frac,
	output             dbg_pair,
	output       [2:0] dbg_recent,
	output       [6:0] dbg_op,          // the instruction being executed: op, rd, rs2
	output       [5:0] dbg_rd,
	output       [4:0] dbg_rs2,
	output       [5:0] dbg_state,
	output      [31:0] dbg_ea           // ... and its access address (rs1 + imm)
);

`include "chg_cpu_ops.svh"

localparam [31:0] MIE = 32'h8, MPIE = 32'h80, MPP = 32'h1800;

// ---- states ------------------------------------------------------------------

localparam [5:0]
	S_RUN = 6'd0, S_RSTW = 6'd1, S_PICK = 6'd2, S_PICKW = 6'd3, S_ITRAP = 6'd4,
	S_SFETCH = 6'd5, S_SFETCH2 = 6'd6, S_SFETCH3 = 6'd7, S_EX0 = 6'd8, S_EX1 = 6'd9,
	S_MEM0 = 6'd10, S_MEM_LD = 6'd11, S_MEM_AMO = 6'd12, S_MEM_AMO2 = 6'd13,
	S_MUL = 6'd14, S_MUL1 = 6'd15, S_MUL2 = 6'd16, S_DIVW = 6'd17, S_MRET = 6'd18, S_MR_HPE = 6'd19,
	S_TE0 = 6'd20, S_TE_HPE = 6'd21, S_TE_VEC = 6'd22, S_TE_VL = 6'd23, S_TE_DONE = 6'd24,
	S_FIN = 6'd25, S_FIN1 = 6'd26, S_FIN2 = 6'd27, S_FIN3 = 6'd28, S_EXC = 6'd29,
	AX_START = 6'd32, AX_RD1 = 6'd33, AX_RD2 = 6'd34, AX_WR2 = 6'd35, AX_SL0 = 6'd36, AX_SL = 6'd37,
	AX_SLF = 6'd38, AX_SLR = 6'd39, AX_SLP = 6'd40, AX_BUSW = 6'd41, AX_SLN = 6'd42, AX_SLP2 = 6'd43, AX_SLP3 = 6'd44, S_MEM_AMO3 = 6'd45, S_CSRW = 6'd46, S_MEM_AMO4 = 6'd47, AX_PRE = 6'd48,
	S_BL0 = 6'd49, S_BL1 = 6'd50;

reg [5:0] state, ax_ret;

// ---- architectural state (ChgCpu) ------------------------------------------------

reg [31:0] xr [0:31];               // xr[0] stays 0; the C's sink x[32] is not kept
reg [31:0] pc;
reg [31:0] mstatus, mtvec, mscratch, mepc, mcause, mtval, intsyscr, corecfgr;
reg [31:0] misc [0:15];             // csr_misc
reg  [3:0] depth;                   // trap_depth, 0..8
reg  [7:0] t_exc;                   // trap_irq[d] == -1
reg  [5:0] t_irq [0:7];
reg  [7:0] t_hpe;
reg [31:0] t_mepc [0:7];
reg [31:0] t_mcause [0:7];
reg [31:0] t_mst [0:7];
reg        wfi, lr_valid;
reg [31:0] lr_addr;
reg        rst_req;                 // m->reset_request

// the register file's write port: written the clock after it is set
reg        wr_en;
reg  [4:0] wr_rd;
reg [31:0] wr_val;

// the HPE stack: hpe[depth][16] of the C, a RAM, saved and restored a
// register a clock while board time is stopped
reg [31:0] hpe_mem [0:127];
reg        hpe_we;
reg  [6:0] hpe_wa, hpe_ra;
reg [31:0] hpe_wd, hpe_q;
always @(posedge clk) begin
	if (hpe_we) hpe_mem[hpe_wa] <= hpe_wd;
	hpe_q <= hpe_mem[hpe_ra];
end

// hpe_regs of rv32.c
function [4:0] hpe_reg(input [3:0] i);
	case (i)
	4'd0: hpe_reg = 5'd1;   4'd1: hpe_reg = 5'd5;   4'd2: hpe_reg = 5'd6;   4'd3: hpe_reg = 5'd7;
	4'd4: hpe_reg = 5'd10;  4'd5: hpe_reg = 5'd11;  4'd6: hpe_reg = 5'd12;  4'd7: hpe_reg = 5'd13;
	4'd8: hpe_reg = 5'd14;  4'd9: hpe_reg = 5'd15;  4'd10: hpe_reg = 5'd16; 4'd11: hpe_reg = 5'd17;
	4'd12: hpe_reg = 5'd28; 4'd13: hpe_reg = 5'd29; 4'd14: hpe_reg = 5'd30; default: hpe_reg = 5'd31;
	endcase
endfunction

// ---- timing state ------------------------------------------------------------------
//
// fw is fetch_word. fd is fetch_time minus the cycle of the next boundary,
// kept no lower than -1024: below that it makes no difference to anything
// cpu_exec computes (a prefetch step clamps it to -word, a branch takes the
// later of issue+1-hidden and fd+word-hidden), so the clamp is exact.
// div_rem is div_ready minus that cycle, no lower than 0, likewise exact.

reg signed [17:0] fw;
reg signed [23:0] fd;
reg  [4:0] div_rem;
reg  [5:0] div_rd;
reg  [2:0] frac;
reg        pair, recent0, recent1, recent2;
wire [2:0] recent = {recent2, recent1, recent0};

reg [23:0] owe;
reg        owe_z;                   // owe == 0
reg [63:0] cyc, icount;
reg        spec;                    // this decision clock is also a board cycle
reg        dce;                     // ... and it went to the interrupt scan
reg        bseen, nochk, refetch, rst_hard;
reg        stale;                   // see irq_q
reg        mspec;                   // this clock is a boundary that may also be the decision (see mgo)
wire       mgo;
assign hold = owe_z && !(spec && !stale) && !mgo;
assign cycles = cyc;
assign instret = icount;
assign boundary = state == S_RUN && owe_z && !bseen && !nochk;

// ---- the PFIC ------------------------------------------------------------------

reg         pfic_clr;
wire        pfic_busy;
wire        pf_rd, pf_wr;
wire [31:0] pf_rdata;
wire        ready_any, ready_fast, pick_want;
wire        pick_done, pick_valid;
wire  [5:0] pick_irq;
wire        pfic_reset_req;
reg  [63:0] active;
reg         exc_in_stack;
reg  [31:0] sa_r;
wire  [2:0] ss;
wire [31:0] sdata;

integer j;
always @* begin
	active = 0;
	exc_in_stack = 0;
	for (j = 0; j < 8; j = j + 1)
		if (j < depth) begin
			if (t_exc[j]) exc_in_stack = 1;
			else active[t_irq[j]] = 1'b1;
		end
end

// the trap stack's interrupts for the PFIC, registered (depth changes long
// before the next scan or IACTR read)
reg  [63:0] active_r;
always @(posedge clk) active_r <= active;

// The interrupt lines, registered here (96 MHz: they come from all over the
// board into the decision). They are the lines of the clock before; a
// decision is not made while that could differ from now: in the clock after
// a boundary at which a peripheral had an event due (irq_ev), and in the
// clock after SPI/DMA was busy ('stale'; hold stays high then).
reg  [63:0] irq_q;
always @(posedge clk) begin
	irq_q <= irq_line;
	stale <= (boundary && irq_ev) || busy;
end

chg_pfic u_pfic (
	.clk(clk), .clr(pfic_clr), .busy(pfic_busy),
	.irq_line(irq_q), .irq_raw(irq_line), .sw_pend(sw_pend),
	.rd_req(pf_rd), .wr_req(pf_wr), .off(sa_r[11:0]), .size(ss), .wdata(sdata), .rdata(pf_rdata),
	.depth(depth), .active(active_r), .ready_any(ready_any), .ready_fast(ready_fast), .pick_want(pick_want),
	.pick_start(state == S_PICK), .in_trap(depth != 0), .nest(intsyscr[1]),
	.pick_done(pick_done), .pick_valid(pick_valid), .pick_irq(pick_irq),
	.take(state == S_ITRAP), .take_irq(pick_irq),
	.reset_req(pfic_reset_req)
);

// MIE and nothing in the way of pick_irq. Registered: the decision that
// reads it never comes in the clock after mstatus, depth or INTSYSCR change.
reg irq_ok;
always @(posedge clk) irq_ok <= mstatus[3] && !exc_in_stack && !(depth != 0 && !intsyscr[1]);

// =====================================================================================
// The pipeline
// =====================================================================================

// ---- front end -------------------------------------------------------------------

localparam [2:0] FE_ADDR = 3'd0, FE_DATA = 3'd1, FE_HELD = 3'd2, FE_HIADDR = 3'd3,
                 FE_HIDATA = 3'd4, FE_HIHELD = 3'd5, FE_STOP = 3'd6;
reg  [2:0] fe_st;
reg [31:0] fpc;                     // the instruction the front end is on
reg        fe_ram;                  // its word is in SRAM (else flash)
reg [31:0] curd;                    // the word at fpc, once read
reg [15:0] hold16;                  // the first half of a 32-bit instruction across two words

// IR: an instruction's bits
reg        ir_v, ir_mk, ir_fs;      // mk: fpc is outside flash and SRAM (fetched after the decision); fs: so fetched
reg [31:0] ir, ir_pc;

// D1: decoded
reg        d1_v, d1_mk, d1_fs, d1_len4, d1_pd;
reg  [6:0] d1_op;
reg  [5:0] d1_rd;
reg  [4:0] d1_rs1, d1_rs2;
reg [31:0] d1_imm, d1_pc;
reg  [2:0] d1_cost;
reg [14:0] d1_wend;

// B: with operands
reg        b_v, b_mk, b_fs, b_fl, b_fwd, b_simple, b_isld, b_isst, b_pred;
reg        b_wl1, b_wl2, b_wr1, b_wr2;  // B takes an operand this clock: from ld_ext (wl) or wr_val (wr)
reg        b_wa1, b_wa2;                // ... with the next ld_p, from port A (ld_x0)
reg        b_exv, b_e1;             // b_res .. b_ea, b_ramf .. b_n2 hold B's EX (a multiply's after two clocks)
reg [63:0] b_prod;
wire       b_ismul = b_op >= OP_MUL && b_op <= OP_MULHU;
wire signed [32:0] b_ma = {(b_op == OP_MULH || b_op == OP_MULHSU) & b_opa[31], b_opa};
wire signed [32:0] b_mb = {(b_op == OP_MULH) & b_opb[31], b_opb};
wire signed [65:0] b_mp = b_ma * b_mb;
reg [31:0] b_res, b_npc, b_ea;
reg        b_tk, b_ramf, b_flf, b_n2;
reg  [2:0] b_sz;
reg  [6:0] b_op;
reg  [5:0] b_rd;
reg  [4:0] b_rs1, b_rs2;
reg [31:0] b_imm, b_pc, b_opa, b_opb, b_seq, b_pci;
reg  [2:0] b_cost;
reg [14:0] b_wend;

// A: the next instruction to commit
reg        a_acc;                   // a load, store, LR/SC or AMO
reg        a_v, a_rdy, a_mk, a_fs, a_fl, a_fwd, a_simple, a_isld, a_isst, a_pred;
reg  [2:0] a_sz;
reg  [6:0] a_op;
reg  [5:0] a_rd;
reg  [4:0] a_rs1, a_rs2;
reg [31:0] a_imm, a_pc, a_opa, a_opb, a_seq, a_pci;
reg  [2:0] a_cost;
reg [14:0] a_wend;
reg [31:0] a_res, a_npc, a_ea;
reg        a_taken;
// T1, for A (from B's q_* when it moved)
reg signed [23:0] p_t1, p_fdpf;
reg signed [17:0] p_fwpf;
// T1 for B, in two stages (s1_*, q_*), from the state A leaves (its packet)
// or, with A empty, the committed state
reg        s1v, q_v;
reg signed [17:0] s1_kk, s1_fwpf, q_fwpf;
reg               s1_kpos, s1_rsm, s1_fl;
reg signed [23:0] s1_mfd, s1_fd, q_t1, q_fdpf, q_u;
reg  [4:0] s1_dv;
// T2: the packet
reg        k_fast, k_ld, k_st, k_ram, k_wb, k_wfi, k_E1;
reg [23:0] k_E, k_Em1;
reg signed [17:0] k_fw;
reg signed [23:0] k_fd;
reg  [4:0] k_div;
reg  [2:0] k_frac, k_rec;
reg        k_pair;
reg  [3:0] k_be;
reg [31:0] k_wd;
reg  [2:0] k_sz;

// a store committed last clock: written now
reg        st_p;
reg [12:0] st_a;
reg  [3:0] st_be;
reg [31:0] st_wd;

// a load committed last clock: its value comes from port A now
reg        ld_p, ld_ram, ld_sx;
// ... and the clock after: the word registered, extended now (96 MHz)
reg        ld_p2, ld_sx2;
reg  [5:0] ld_rd2;
reg  [1:0] ld_b2;
reg  [2:0] ld_sz2;
reg [31:0] ld_w;
reg  [5:0] ld_rd;
reg  [1:0] ld_b;
reg  [2:0] ld_sz;

// fetch(): pc < 0xFFFE, and pc - RAM < 0x4FFE. The upper bits' halves are
// registers (fe_hfl: fpc[31:16] == 0, fe_hram: fpc[31:15] == 0x4000), set
// with fpc: a redirect works them out, a step can only leave flash (0xFFFC + 4)
reg  fe_hfl, fe_hram;
wire fe_okfl  = fe_hfl && fpc[15:1] != 15'h7FFF;
wire fe_okram = fe_hram && fpc[14:0] < 15'h4FFE;

wire [31:0] fe_rd   = fe_ram ? ram_rdata : fl_rdata;
wire        fe_live = fe_st == FE_DATA || fe_st == FE_HIDATA;
wire [31:0] fe_w    = fe_live ? fe_rd : curd;
wire        fe_lo   = fe_st == FE_DATA || fe_st == FE_HELD;
wire        fe_hi   = fe_st == FE_HIDATA || fe_st == FE_HIHELD;
wire [15:0] fe_h    = fpc[1] ? fe_w[31:16] : fe_w[15:0];
wire        fe_32   = fe_h[1:0] == 2'b11;
wire        fe_strad = fe_lo && fpc[1] && fe_32;
wire        fe_cmp  = (fe_lo && !fe_strad) || fe_hi;
wire [31:0] fe_ir   = fe_hi ? {fe_w[15:0], hold16} : fpc[1] ? {16'd0, fe_w[31:16]} : fe_w;
wire        fe_len4 = fe_hi || fe_32;
wire        fe_same = fe_lo && !fpc[1] && !fe_32;      // the next one is in this word too
wire        fe_mark = (fe_st == FE_ADDR || fe_st == FE_HELD) && !fe_okfl && !fe_okram;

// port A for the front end: in S_RUN when the instruction at the decision does not need it
wire        dacc = state == S_RUN && owe_z && (bseen || nochk || mspec) && a_v && a_rdy && k_fast && (k_ld || k_st);
wire        fe_pfree = state == S_RUN && !dacc && !st_p;
wire        fe_rqlo = fe_st == FE_ADDR && !fe_mark;
// (fe_strad, written out: only bits 17:16 of the word just read reach it)
wire        fe_rqhi = fe_st == FE_HIADDR ||
                      (fpc[1] && ((fe_st == FE_DATA && fe_rd[17:16] == 2'b11) || (fe_st == FE_HELD && curd[17:16] == 2'b11)));
wire        fe_issue = (fe_rqlo || fe_rqhi) && fe_pfree;
// (the word after fpc's is summed from fpc alone; fe_rqhi, which comes from
// the word just read when an instruction straddles two, only picks: 96 MHz)
wire [13:0] fe_fl1 = fpc[15:2] + 14'd1;
wire [12:0] fe_ra1 = fpc[14:2] + 13'd1;
wire [13:0] fe_fla = fe_rqhi ? fe_fl1 : fpc[15:2];
wire [12:0] fe_rama = fe_rqhi ? fe_ra1 : fpc[14:2];
// a branch in B the front end did not follow (or followed wrongly): port A
// reads its target's word in the clock the front end is sent there (rd_iss)
wire        rd_b = b_v && b_exv && b_tk != b_pred;
wire        rd_fl = b_npc[31:16] == 16'd0 && b_npc[15:1] != 15'h7FFF;
wire        rd_ram = b_npc[31:15] == 17'h04000 && b_npc[14:0] < 15'h4FFE;
wire        rd_iss = rd_b && fe_pfree && (rd_fl || rd_ram);

// ---- DEC ----
wire  [6:0] dc_op;
wire  [5:0] dc_rd;
wire  [4:0] dc_rs1, dc_rs2;
wire [31:0] dc_imm;
wire        dc_len4;
wire  [2:0] dc_cost;
wire [14:0] dc_wend;
chg_cpu_decode u_dec (
	.pc(ir_pc[15:0]), .raw(ir), .op(dc_op), .rd(dc_rd), .rs1(dc_rs1), .rs2(dc_rs2),
	.imm(dc_imm), .len4(dc_len4), .cost(dc_cost), .wend(dc_wend)
);

// ---- RR ----
wire [31:0] xr1 = xr[d1_rs1];
wire [31:0] xr2 = xr[d1_rs2];
wire        rr_fa1 = a_v && a_fwd && a_rd == {1'b0, d1_rs1};
wire        rr_fa2 = a_v && a_fwd && a_rd == {1'b0, d1_rs2};
wire        rr_fw1 = wr_en && wr_rd == d1_rs1;
wire        rr_fw2 = wr_en && wr_rd == d1_rs2;
wire        rr_fb1 = b_v && b_exv && b_fwd && b_rd == {1'b0, d1_rs1};
wire        rr_fb2 = b_v && b_exv && b_fwd && b_rd == {1'b0, d1_rs2};
wire [31:0] rr_a = rr_fb1 ? b_res : rr_fa1 ? a_res : rr_fw1 ? wr_val : xr1;
wire [31:0] rr_b = rr_fb2 ? b_res : rr_fa2 ? a_res : rr_fw2 ? wr_val : xr2;
wire        rr_m1 = (b_v && !(b_exv && b_fwd) && b_rd == {1'b0, d1_rs1}) || (a_v && !a_fwd && !k_ld && a_rd == {1'b0, d1_rs1});
wire        rr_m2 = (b_v && !(b_exv && b_fwd) && b_rd == {1'b0, d1_rs2}) || (a_v && !a_fwd && !k_ld && a_rd == {1'b0, d1_rs2});
// a fast load in A: B takes its value straight from port A with ld_p (rr_a*)
wire        rr_a1 = a_v && k_ld && a_rd == {1'b0, d1_rs1} && !rr_fb1;
wire        rr_a2 = a_v && k_ld && a_rd == {1'b0, d1_rs2} && !rr_fb2;
// a fast load committed, its value not yet in the register file: D1 goes on
// into B, which takes the value the clock after (ld_ext with ld_p2, or
// wr_val: rr_l*, rr_r*), unless a younger result in A or B is forwarded
wire        rr_l1 = ld_p && ld_rd == {1'b0, d1_rs1} && !rr_fb1 && !rr_fa1;
wire        rr_l2 = ld_p && ld_rd == {1'b0, d1_rs2} && !rr_fb2 && !rr_fa2;
wire        rr_r1 = ld_p2 && ld_rd2 == {1'b0, d1_rs1} && !rr_fb1 && !rr_fa1 && !rr_l1;
wire        rr_r2 = ld_p2 && ld_rd2 == {1'b0, d1_rs2} && !rr_fb2 && !rr_fa2 && !rr_l2;
wire        rr_hz = !d1_mk && (rr_m1 || rr_m2);
// the front end follows a JAL, or a branch backwards, from here (its target
// is d1_pc + d1_imm); T2 puts it right when the guess was wrong
wire        rr_pred = !d1_mk && (d1_op == OP_JAL || (d1_op >= OP_BEQ && d1_op <= OP_BGEU && d1_imm[31]));
wire [31:0] rr_pci = d1_pc + d1_imm;
// written through a_res: the one-cycle operations
wire        d1_alu = (d1_op >= OP_LUI && d1_op <= OP_JALR) || (d1_op >= OP_ADDI && d1_op <= OP_AND) ||
                     (d1_op >= OP_MUL && d1_op <= OP_MULHU);

// ---- the pipeline's moves ----
wire b_wt   = b_wl1 || b_wl2 || b_wr1 || b_wr2 || b_wa1 || b_wa2;                // B's operands are not all in yet
wire xfer   = b_v && !a_v && b_exv && q_v;                      // B -> A, with T2
wire rr_go  = d1_v && (!b_v || xfer) && !rr_hz;                 // D1 -> B
wire dec_go = ir_v && (!d1_v || rr_go);                         // IR -> D1
wire ir_free = !ir_v || !d1_v;                                  // (not || dec_go: a shorter path)

// ---- EX: the operation (cpu_exec's switch) for the one-cycle instructions --------------

reg [31:0] alu;
reg        b_taken;
always @* begin
	alu = 0;
	b_taken = 0;
	case (b_op)
	OP_LUI:   alu = b_imm;
	OP_AUIPC: alu = b_pci;
	OP_JAL, OP_JALR: alu = b_seq;
	OP_BEQ:   b_taken = b_opa == b_opb;
	OP_BNE:   b_taken = b_opa != b_opb;
	OP_BLT:   b_taken = $signed(b_opa) < $signed(b_opb);
	OP_BGE:   b_taken = $signed(b_opa) >= $signed(b_opb);
	OP_BLTU:  b_taken = b_opa < b_opb;
	OP_BGEU:  b_taken = b_opa >= b_opb;
	OP_ADDI:  alu = b_opa + b_imm;
	OP_SLTI:  alu = {31'd0, $signed(b_opa) < $signed(b_imm)};
	OP_SLTIU: alu = {31'd0, b_opa < b_imm};
	OP_XORI:  alu = b_opa ^ b_imm;
	OP_ORI:   alu = b_opa | b_imm;
	OP_ANDI:  alu = b_opa & b_imm;
	OP_SLLI:  alu = b_opa << b_imm[4:0];
	OP_SRLI:  alu = b_opa >> b_imm[4:0];
	OP_SRAI:  alu = $signed(b_opa) >>> b_imm[4:0];
	OP_ADD:   alu = b_opa + b_opb;
	OP_SUB:   alu = b_opa - b_opb;
	OP_SLL:   alu = b_opa << b_opb[4:0];
	OP_SLT:   alu = {31'd0, $signed(b_opa) < $signed(b_opb)};
	OP_SLTU:  alu = {31'd0, b_opa < b_opb};
	OP_XOR:   alu = b_opa ^ b_opb;
	OP_SRL:   alu = b_opa >> b_opb[4:0];
	OP_SRA:   alu = $signed(b_opa) >>> b_opb[4:0];
	OP_OR:    alu = b_opa | b_opb;
	OP_AND:   alu = b_opa & b_opb;
	default: ;
	endcase
end
wire [31:0] ex_ea = b_opa + b_imm;
wire        ex_isbr = b_op >= OP_BEQ && b_op <= OP_BGEU;
wire        ex_taken = !b_mk && (b_op == OP_JAL || b_op == OP_JALR || (ex_isbr && b_taken));
wire [31:0] ex_npc = (b_op == OP_JALR) ? (ex_ea & ~32'd1) : ex_taken ? b_pci : b_seq;

// ---- T1: the timing unit to issue, the prefetcher loop of cpu_exec ------------------
//
// while (wend - fw > 0) { fw++; ft += word; if (ft + word < cycles) ft = cycles - word; }
// in closed form: after k > 0 steps, fd = k*word + max(fd, -2*word).
// Worked out for B while A waits: from A's packet (the state after A, valid
// when A is fast and ready), or from the committed state when A is empty. Two
// stages; recomputed every clock, valid when nothing moved for two clocks.

wire               src_pk = a_v;
wire               src_ok = !a_v || (a_rdy && k_fast);
wire signed [17:0] fw_s = src_pk ? k_fw : fw;
wire signed [23:0] fd_s = src_pk ? k_fd : fd;
wire  [4:0]        dr_s = src_pk ? k_div : div_rem;
// stage 1
wire signed [17:0] t1_kk = $signed({3'b000, b_wend}) - fw_s;
wire signed [23:0] t1_mfd = b_fl ? ((fd_s < -24'sd8) ? -24'sd8 : fd_s) : ((fd_s < -24'sd2) ? -24'sd2 : fd_s);
// stage 2
wire signed [23:0] t1_k24 = {{6{s1_kk[17]}}, s1_kk};
wire signed [23:0] t1_sm = (s1_fl ? (t1_k24 <<< 2) : t1_k24) + s1_mfd;
wire signed [23:0] t1_fdpf = s1_kpos ? t1_sm : s1_fd;
wire signed [23:0] t1_t1a = t1_fdpf[23] ? 24'sd0 : t1_fdpf;
wire signed [23:0] t1_dv = $signed({19'd0, s1_dv});
// reading a register the divider has not delivered yet: div_rem > max(fd_pf, 0)
wire               t1_dh = s1_rsm && s1_dv != 0 && (s1_kpos ? (t1_sm < t1_dv) : (s1_fd < t1_dv));
wire signed [23:0] t1_t1 = t1_dh ? t1_dv : t1_t1a;
wire signed [23:0] t1_u = t1_dh ? (t1_fdpf - t1_dv) : (t1_fdpf[23] ? t1_fdpf : 24'sd0);   // fd_pf - t1

// ---- T2: the rest of cpu_exec's loop body, for the fast instructions ---------------

wire  [2:0] t2_sz = b_sz;
wire        t2_ramf = b_ramf;
wire        t2_flf = b_flf;
wire        t2_n2 = b_n2;
wire        t2_ldr = b_isld && t2_ramf && !t2_n2;                  // RAM load
wire        t2_ldf = b_isld && !t2_ramf && t2_flf && !t2_n2;       // flash load
wire        t2_st = b_isst && t2_ramf && !t2_n2;                    // RAM store
wire        t2_fast = !b_mk && (b_simple || t2_ldr || t2_ldf || t2_st);
wire  [5:0] t2_fs = {3'd0, frac} + 6'd21;                          // FLASH_LOAD_RAM8 - 8 * LOAD_CYCLES eighths
// cycles after issue (cE = E - t1): 1, 2, 4 or 5; what fd gains less cE: 0, -1, -2, -4 or -5.
// Every sum is made for each case and the case picked last.
wire        t2_c1 = !(t2_ldr || t2_ldf || t2_st);
wire        t2_c5 = t2_ldf && !b_fl && t2_fs[3];
wire        t2_c4 = t2_ldf && !t2_c5;
wire        t2_c2 = !t2_c1 && !t2_c4 && !t2_c5;
wire signed [23:0] t2_E = t2_c1 ? q_t1 + 24'sd1 : t2_c2 ? q_t1 + 24'sd2 : t2_c4 ? q_t1 + 24'sd4 : q_t1 + 24'sd5;
wire signed [23:0] t2_Em1 = t2_c1 ? q_t1 : t2_c2 ? q_t1 + 24'sd1 : t2_c4 ? q_t1 + 24'sd3 : q_t1 + 24'sd4;
// fd gain less cE, by case
wire        t2_k0 = (t2_ldf && b_fl) || (t2_st && !b_fl && !pair);
wire        t2_k1 = b_simple || (t2_ldr && !b_fl) || (t2_st && !b_fl && pair);
wire        t2_k2 = (t2_ldr && b_fl) || (t2_st && b_fl);
wire        t2_k4 = t2_ldf && !b_fl && !t2_fs[3];
function signed [23:0] clampn(input signed [23:0] v);
	clampn = (v < -24'sd1024) ? -24'sd1024 : v;
endfunction
wire signed [23:0] t2_nfc = t2_k0 ? clampn(q_u) : t2_k1 ? clampn(q_u - 24'sd1) : t2_k2 ? clampn(q_u - 24'sd2) :
                            t2_k4 ? clampn(q_u - 24'sd4) : clampn(q_u - 24'sd5);
// a taken branch: fetching starts over at the target once the word in flight
// is in; an SRAM access in the two instructions before hides a cycle
wire        t2_hid = recent[1:0] != 0;
wire signed [23:0] t2_tin = b_fl ? (t2_hid ? q_u + 24'sd2 : q_u + 24'sd3) : (t2_hid ? q_u - 24'sd1 : q_u);   // + word - hid - 1
wire signed [23:0] t2_tst = t2_hid ? -24'sd1 : 24'sd0;
wire signed [23:0] t2_tfd = (t2_tin > t2_tst) ? t2_tin : t2_tst;
wire signed [24:0] t2_dv0 = $signed({20'd0, div_rem}) - {q_t1[23], q_t1};
wire signed [24:0] t2_dv = t2_c1 ? t2_dv0 - 25'sd1 : t2_c2 ? t2_dv0 - 25'sd2 : t2_c4 ? t2_dv0 - 25'sd4 : t2_dv0 - 25'sd5;

// T1's first stage for D1 as it moves into an empty B (from the same state
// as stage 1 would use the clock after: nothing commits in between that
// changes it, A's packet being what a fast commit leaves)
wire               d1_fl = d1_pc[31:16] == 16'd0;
wire signed [17:0] t1d_kk = $signed({3'b000, d1_wend}) - fw_s;
wire signed [23:0] t1d_mfd = d1_fl ? ((fd_s < -24'sd8) ? -24'sd8 : fd_s) : ((fd_s < -24'sd2) ? -24'sd2 : fd_s);

// T1's first stage for D1, from the packet B is about to give A (the bypass)
// (each sum for both ways of the branch, picked last; max(fd, -8 or -2) of a
// fd that is no lower than -1024 is max(the sum before the clamp, -8 or -2),
// and a taken branch's fd is never below -1)
wire signed [17:0] by_fw = b_tk ? ($signed({4'd0, b_npc[15:2]}) - 18'sd1) : q_fwpf;
wire signed [23:0] by_fd = b_tk ? t2_tfd : t2_nfc;
wire  [4:0]        by_dv = t2_dv[24] ? 5'd0 : t2_dv[4:0];
wire signed [17:0] by_kkt = $signed({3'b000, d1_wend}) + 18'sd1 - $signed({4'd0, b_npc[15:2]});
wire signed [17:0] by_kkn = $signed({3'b000, d1_wend}) - q_fwpf;
wire signed [17:0] by_kk = b_tk ? by_kkt : by_kkn;
wire               by_kp = b_tk ? (by_kkt > 0) : (by_kkn > 0);
wire signed [23:0] by_kc = t2_k0 ? 24'sd0 : t2_k1 ? 24'sd1 : t2_k2 ? 24'sd2 : t2_k4 ? 24'sd4 : 24'sd5;
wire signed [23:0] by_nn = q_u - by_kc;
wire signed [23:0] by_fl = (d1_pc[31:16] == 16'd0) ? -24'sd8 : -24'sd2;
wire signed [23:0] by_mfd = b_tk ? t2_tfd : (by_nn < by_fl) ? by_fl : by_nn;

// =====================================================================================
// The decision (chg_run's loop) and the fast commit
// =====================================================================================

// The boundary clock can be the decision clock too (and a board cycle, as
// with spec), when spec would follow it for sure: A fast and ready since the
// clock before (mspec), no peripheral event due at it (irq_ev: the lines are
// those of the clock before, irq_q), SPI/DMA with nothing due (swork: no sync,
// and nothing else that sees boundary minds the ce in the same clock), and
// the lines as settled as for spec (stale).
assign mgo = mspec && boundary && !stale && !irq_ev && !swork;
// A port B write is seen by the front end only when it can matter: any write to
// flash, or to SRAM while an instruction in the pipeline (or the one being fetched)
// is in SRAM. DMA bytes into a data buffer no longer throw the pipeline away.
wire code_ram = (fpc[31:29] == 3'b001) || (ir_pc[31:29] == 3'b001) || (d1_pc[31:29] == 3'b001) ||
                (b_pc[31:29] == 3'b001) || (a_pc[31:29] == 3'b001) || (pc[31:29] == 3'b001);
// (registered: the decision looks at it a clock later than the pulse; the notes in chg_soc: SPI/DMA is
// still busy then, and the flash controller's access is still waiting for its answer)
reg  mbw;
always @(posedge clk) mbw <= mem_b_wr && (!mem_b_ram || code_ram);
wire dphase  = state == S_RUN && owe_z && (bseen || nochk);
wire d_go    = (dphase && !stale && (spec ? ce : !busy)) || (mgo && ce);
wire spec_e  = spec || mgo;
wire irq_take = irq_ok && pick_want;   // pick_irq (unless it would find nothing again)
wire d_chk   = !nochk && !refetch;
wire d_flush = d_go && refetch;
wire d_rst   = d_go && d_chk && rst_req;
wire d_irq   = d_go && d_chk && !rst_req && irq_take;
wire d_sleep = d_go && d_chk && !rst_req && !irq_take && wfi && !ready_fast;
wire d_exec  = d_go && !refetch && (nochk || (!rst_req && !irq_take && (!wfi || ready_fast)));
wire fc      = d_exec && a_v && a_rdy && k_fast;               // the fast commit
wire d_slow  = d_exec && a_v && a_rdy && !k_fast;
wire d_take  = d_flush || d_rst || d_irq || d_sleep || fc || d_slow;
wire a_fr_n  = a_v ? (a_rdy && k_fast) : (xfer && t2_fast);   // A fast and ready next clock
wire spec_n  = state == S_RUN && owe_z && (bseen || boundary) && !nochk && !wfi && !rst_req && !(boundary && irq_ev) &&
               !refetch && !mem_b_wr && !mbw && !d_take && a_fr_n;
// the next clock is a boundary (this one is the last cycle of the instruction)
// with A fast and ready, nothing that throws it away, no WFI or reset request
wire mspec_n = state == S_RUN && !owe_z && owe == 24'd1 && ce && !bseen && !nochk && !wfi && !rst_req && !pfic_reset_req &&
               !refetch && !mem_b_wr && !mbw && !fc_rd && !sl_r && !ld_p && a_fr_n;   // (ld_p: its register write is in the boundary clock)

// a store near the code being fetched (A's)
wire        a_hit = !a_fl && ((a_ea[14:2] - a_pc[14:2]) < 13'd8);

// ---- the operation for the slow path ------------------------------------------------

// csr_read
wire [11:0] csr = a_imm[11:0];
reg  [63:0] mcyc;                                   // m->cycles at issue
reg  [31:0] csr_old;
always @* begin
	case (csr)
	12'h300: csr_old = mstatus;
	12'h301: csr_old = 32'h40901105;                // RV32IMAC + X
	12'h305: csr_old = mtvec;
	12'h340: csr_old = mscratch;
	12'h341: csr_old = mepc;
	12'h342: csr_old = mcause;
	12'h343: csr_old = mtval;
	12'h800: csr_old = mstatus & (MIE | MPIE);
	12'h804: csr_old = intsyscr;
	12'hBC0: csr_old = corecfgr;
	12'hF11: csr_old = 32'h00000489;                // WCH's JEDEC id
	12'hF12: csr_old = 32'hDC68D841;                // QingKe V4C
	12'hF13, 12'hF14: csr_old = 0;
	12'hB00, 12'hC00: csr_old = mcyc[31:0];
	12'hB80, 12'hC80: csr_old = mcyc[63:32];
	default: csr_old = (csr >= 12'h7A0 && csr <= 12'h7AF) ? 32'd0 : misc[csr[3:0]];
	endcase
end
wire [31:0] csr_src = (a_op >= OP_CSRRWI) ? {27'd0, a_rs1} : a_opa;
reg  [31:0] csr_or;                                // csr_old, registered in S_EXC
reg  [31:0] csr_nr;                                // csr_new, registered in S_EX1
reg         csr_wer;
wire [31:0] csr_new = (a_op == OP_CSRRW || a_op == OP_CSRRWI) ? csr_src :
                      (a_op == OP_CSRRS || a_op == OP_CSRRSI) ? (csr_or | csr_src) : (csr_or & ~csr_src);
wire        csr_we = (a_op == OP_CSRRW || a_op == OP_CSRRWI) || a_rs1 != 0;

// the multiplier: 33 x 33 signed, inputs and product registered
reg  [63:0] prod;
reg signed [32:0] mul_ar, mul_br;
wire        mul_sa = a_op == OP_MULH || a_op == OP_MULHSU;
wire        mul_sb = a_op == OP_MULH;
wire signed [65:0] mul_p = mul_ar * mul_br;

// the divider
reg         div_start;
wire        div_done;
wire [31:0] div_result;
chg_cpu_div u_div (
	.clk(clk), .start(div_start), .a(a_opa), .b(a_opb), .kind(a_op[1:0] - OP_DIV[1:0]),
	.done(div_done), .result(div_result)
);

// AMO
reg  [31:0] amo_old;
reg  [31:0] amo_v, amo_r;
reg         am_lts, am_ltu;             // S_MEM_AMO3: the compares and the sum
reg  [31:0] am_sum;
wire        am_eq = amo_old == a_opb;
always @* begin
	case (a_op)
	OP_AMOADD:  amo_v = am_sum;
	OP_AMOXOR:  amo_v = amo_old ^ a_opb;
	OP_AMOAND:  amo_v = amo_old & a_opb;
	OP_AMOOR:   amo_v = amo_old | a_opb;
	OP_AMOMIN:  amo_v = am_lts ? amo_old : a_opb;
	OP_AMOMAX:  amo_v = (!am_lts && !am_eq) ? amo_old : a_opb;
	OP_AMOMINU: amo_v = am_ltu ? amo_old : a_opb;
	OP_AMOMAXU: amo_v = (!am_ltu && !am_eq) ? amo_old : a_opb;
	default:    amo_v = a_opb;                       // AMOSWAP
	endcase
end

// ---- the access engine: load() and store() ---------------------------------------

reg        ax_we, ax_noflag;
reg [31:0] ax_addr, ax_wdata, ax_val, ax_d0;
reg  [2:0] ax_size;
reg  [1:0] ax_bi;
reg        sa_fl, sa_ram, sa_pf, sa_sys, sa_hi, ax_ramr;
reg [31:0] pf_q;

wire [31:0] ax_off = ax_addr - 32'h20000000;
wire [31:0] ax_sz = {29'd0, ax_size};
wire        ax_ramf_c = ax_off <= (32'h00005000 - ax_sz);  // load()/store(): the SRAM fast path
wire        ax_flf_c = ax_addr <= (32'h00010000 - ax_sz);  // load(): the flash fast path
reg         ax_ramf, ax_flf;                                // ... registered in AX_PRE
wire  [1:0] ax_b = ax_addr[1:0];
wire        ax_need2 = ({1'b0, ax_b} + ax_size) > 3'd4;
wire [12:0] ax_rw0 = ax_off[14:2];
wire [13:0] ax_fw0 = ax_addr[15:2];
wire  [3:0] ax_m4 = (ax_size == 1) ? 4'h1 : (ax_size == 2) ? 4'h3 : 4'hf;
wire  [7:0] ax_be = {4'd0, ax_m4} << ax_b;
wire [63:0] ax_wd = {32'd0, ax_wdata} << {ax_b, 3'b000};
wire [31:0] ax_mask = (ax_size == 1) ? 32'hff : (ax_size == 2) ? 32'hffff : 32'hffffffff;
// bus_read_slow / bus_write_slow: misaligned accesses below the peripherals go a byte at a time
wire        ax_mis = (ax_addr & (ax_sz - 1)) != 0;
wire        ax_split = ax_mis && ax_addr < 32'h40000000;
reg  [2:0] ss_r;
reg [31:0] sdata_r;
assign ss = ss_r;
assign sdata = sdata_r;
wire [31:0] sa_n = ax_addr + {30'd0, ax_bi};             // AX_SL0: the address of this access
wire [31:0] sa_noff = sa_n - 32'h20000000;
wire [31:0] sa_off = sa_r - 32'h20000000;
wire [31:0] s_mask = (ss == 1) ? 32'hff : (ss == 2) ? 32'hffff : 32'hffffffff;
wire  [3:0] s_m4 = (ss == 1) ? 4'h1 : (ss == 2) ? 4'h3 : 4'hf;
wire [31:0] s_frd = (fl_rdata >> {sa_r[1:0], 3'b000}) & s_mask;
wire [31:0] s_rrd = (ram_rdata >> {sa_r[1:0], 3'b000}) & s_mask;
wire [31:0] ax_acc_sh = {27'd0, ax_bi, 3'b000};

assign pf_rd = state == AX_SL && !busy && !ax_we && !sa_fl && !sa_ram && sa_pf;
assign pf_wr = state == AX_SL && !busy && ax_we && !sa_ram && !sa_fl && sa_pf;

// port A addresses: the front end's (fl_f, ra_f: fpc's word, or the next
// one with fe_rqhi) unless something else has the port (fl_o, ra_o). fe_rqhi
// comes late (from the word just read) and only picks between the two.
reg  [13:0] fl_o;
reg  [12:0] ra_o;
reg         fl_f, ra_f;
always @* begin
	fl_addr = fe_rqhi ? (fl_f ? fe_fl1 : fl_o) : fl_o;
	ram_addr = fe_rqhi ? (ra_f ? fe_ra1 : ra_o) : ra_o;
end
always @* begin
	fl_o = rd_b ? b_npc[15:2] : fpc[15:2];
	ra_o = rd_b ? b_npc[14:2] : fpc[14:2];
	fl_f = !rd_b;
	ra_f = !rd_b;
	ram_we = 0;
	ram_wdata = 0;
	if (dacc) begin
		// the fast access, made in the decision clock (the write only if it commits)
		begin fl_o = a_ea[15:2]; fl_f = 0; end
		begin ra_o = a_ea[14:2]; ra_f = 0; end
	end
	if (st_p) begin
		// a fast store, the clock after it committed (its instruction's
		// cycles are still running: nothing reads the word before)
		begin ra_o = st_a; ra_f = 0; end
		ram_we = st_be;
		ram_wdata = st_wd;
	end
	case (state)
	AX_START: begin
		begin fl_o = ax_fw0; fl_f = 0; end
		begin ra_o = ax_rw0; ra_f = 0; end
		if (ax_we && ax_ramf && !busy) begin
			ram_we = ax_be[3:0];
			ram_wdata = ax_wd[31:0];
		end
	end
	AX_RD1: begin
		begin fl_o = ax_fw0 + 1'd1; fl_f = 0; end
		begin ra_o = ax_rw0 + 1'd1; ra_f = 0; end
	end
	AX_WR2: begin
		begin ra_o = ax_rw0 + 1'd1; ra_f = 0; end
		ram_we = ax_be[7:4];
		ram_wdata = ax_wd[63:32];
	end
	AX_SL: begin
		begin fl_o = sa_r[15:2]; fl_f = 0; end
		begin ra_o = sa_off[14:2]; ra_f = 0; end
		if (ax_we && sa_ram && !busy) begin
			ram_we = s_m4 << sa_r[1:0];
			ram_wdata = sdata << {sa_r[1:0], 3'b000};
		end
	end
	default: ;
	endcase
end

// ---- a load from the peripheral bus (S_BL0, S_BL1) ----------------------------------
// LB..LHU at 0x4000_0000 and up, not the PFIC: one bus access, never split
// (bus_read_slow). The access is made straight from the instruction, and the
// timing after it (FIN, FIN1, FIN3) is worked out while the bus answers, for
// no stall; with a stall it goes on as AX_BUSW would (AX_SLN, S_FIN ...).
wire        bl_c = a_isld && a_ea[31:30] != 2'b00 && a_ea[31:12] != 20'hE000E;
reg   [1:0] bl_n;
reg         bl_ack;
reg  [31:0] bl_d;
reg  [16:0] bl_st;
wire [31:0] bl_m = (a_sz == 3'd1) ? 32'hff : (a_sz == 3'd2) ? 32'hffff : 32'hffffffff;
wire [31:0] bl_raw = (bl_ack ? bl_d : bus_rdata) & bl_m;
wire [16:0] bl_stall = bl_ack ? bl_st : stall;
wire [31:0] bl_ext = (a_op == OP_LB) ? {{24{bl_raw[7]}}, bl_raw[7:0]} : (a_op == OP_LH) ? {{16{bl_raw[15]}}, bl_raw[15:0]} : bl_raw;

// ---- the timing unit after issue, for the slow path (three clocks: FIN, FIN1, FIN2) --

reg signed [23:0] r_Tx, r_divrdy, t1d_r;
reg [23:0] extra, elapsed;
reg  [3:0] flags;                   // WAIT_FLASH_DATA, WAIT_RAM_STORE, WAIT_RAM_LOAD, WAIT_LOAD
localparam F_FLD = 0, F_RST = 1, F_RLD = 2, F_LD = 3;
reg        wb_en, taken, is_wfi, ldx, sc_ok;
reg        x_div, x_mret, x_csr, x_exc, x_ecall, x_ebreak, x_wfi;   // S_EX0: what S_EX1 does
reg [31:0] wb_val, npc;
wire       in_flash = a_fl;                            // pc < 0x10000 (pc is A's)
wire signed [23:0] wordc = in_flash ? 24'sd4 : 24'sd1;     // FLASH_WORD : RAM_WORD

reg signed [23:0] fin_E, fin_fd;
reg  [2:0] fin_frac;
reg  [4:0] fin_fs;
reg        fin_pair;
reg  [2:0] fin_rec;
always @* begin
	fin_E = r_Tx + $signed({21'd0, a_cost}) + $signed(extra);
	fin_fd = p_fdpf;
	fin_frac = frac;
	fin_fs = 0;
	if (flags[F_FLD]) begin
		if (in_flash) begin
			// the load had the flash; the prefetcher waited for it
			fin_fd = fin_fd + 24'sd4;
			if (fin_E < p_t1 + 24'sd4) fin_E = p_t1 + 24'sd4;
		end else begin
			// FLASH_LOAD_RAM8 - 8 * LOAD_CYCLES eighths
			fin_fs = {2'b00, frac} + 5'd21;
			fin_E = fin_E + $signed({22'd0, fin_fs[4:3]});
			fin_frac = fin_fs[2:0];
		end
	end
	fin_pair = pair;
	if (!in_flash) begin
		// the SRAM port was the prefetcher's for these cycles
		if (flags[F_RLD]) fin_fd = fin_fd + 24'sd1;
		if (flags[F_RST]) begin
			fin_fd = fin_fd + (pair ? 24'sd1 : 24'sd2);
			fin_pair = !pair;
		end
	end
	if (!flags[F_RST]) fin_pair = 0;
	fin_rec = {recent[1:0], !in_flash && (flags[F_RST] || flags[F_RLD] || flags[F_LD])};
end
// FIN -> FIN1
reg signed [23:0] f_E, f_fd;
reg  [2:0] f_frac, f_rec;
reg        f_pair;
// FIN1: the branch refetch, then everything relative to the next boundary
wire       f_hid = f_rec[2:1] != 0;
wire signed [23:0] f_st = p_t1 + 24'sd1 - (f_hid ? 24'sd1 : 24'sd0);
wire signed [23:0] f_in = f_fd + wordc - (f_hid ? 24'sd1 : 24'sd0);
wire signed [23:0] f_fdt = taken ? ((f_in > f_st) ? f_in : f_st) : f_fd;
reg  signed [23:0] h_fdt;
wire signed [23:0] f_fds0 = h_fdt - f_E;
wire signed [23:0] f_dv0 = r_divrdy - f_E;
// FIN1 -> FIN2
reg signed [23:0] g_fd;
reg signed [17:0] g_fw;
reg  [4:0] g_div;
reg [23:0] g_owe;

// The clock after a fast load: the loaded value, extended
wire [31:0] ld_word = ld_ram ? ram_rdata : fl_rdata;
wire [31:0] ld_raw = (ld_w >> {ld_b2, 3'b000}) & ((ld_sz2 == 1) ? 32'hff : (ld_sz2 == 2) ? 32'hffff : 32'hffffffff);
wire [31:0] ld_ext = (ld_sx2 && ld_sz2 == 1) ? {{24{ld_raw[7]}}, ld_raw[7:0]} :
                     (ld_sx2 && ld_sz2 == 2) ? {{16{ld_raw[15]}}, ld_raw[15:0]} : ld_raw;

// ... and the clock before, straight from port A (for B: b_wa*)
wire [31:0] ld_raw0 = (ld_word >> {ld_b, 3'b000}) & ((ld_sz == 1) ? 32'hff : (ld_sz == 2) ? 32'hffff : 32'hffffffff);
wire [31:0] ld_x0 = (ld_sx && ld_sz == 1) ? {{24{ld_raw0[7]}}, ld_raw0[7:0]} :
                    (ld_sx && ld_sz == 2) ? {{16{ld_raw0[15]}}, ld_raw0[15:0]} : ld_raw0;

// ---- trap entry and mret registers ------------------------------------------------

reg [31:0] te_pc, te_cause, te_tval, te_val, w0;
reg  [5:0] te_vec, te_irqn;
reg        te_isirq;
reg  [2:0] te_idx;
reg  [4:0] te_i;
reg  [2:0] mr_d;
reg  [4:0] mr_i;
wire [2:0] dep_idx = (depth < 8) ? depth[2:0] : 3'd7;
wire [2:0] dep_m1 = depth[2:0] - 1'd1;

// ---- testbench view -------------------------------------------------------------

genvar g;
generate for (g = 0; g < 32; g = g + 1) begin : gx
	// with the write that is still on its way
	assign dbg_x[g * 32 +: 32] = (ld_p2 && ld_rd2 == g) ? ld_ext : (wr_en && wr_rd == g) ? wr_val : xr[g];
end endgenerate
assign dbg_pc = pc;
assign dbg_mstatus = mstatus;
assign dbg_mepc = mepc;
assign dbg_mcause = mcause;
assign dbg_depth = depth;
assign dbg_wfi = wfi;
assign dbg_fw = fw;
assign dbg_fd = fd;
assign dbg_div_rem = div_rem;
assign dbg_div_rd = div_rd;
assign dbg_frac = frac;
assign dbg_pair = pair;
assign dbg_recent = recent;
assign dbg_op = a_op;
assign dbg_rd = a_rd;
assign dbg_rs2 = a_rs2;
assign dbg_state = state;
assign dbg_ea = a_ea;

// ---- the state machine ----------------------------------------------------------

function signed [23:0] fd_less(input signed [23:0] v, input [23:0] n);
	reg signed [23:0] t;
	begin
		t = v - $signed(n);
		fd_less = (t < -24'sd1024) ? -24'sd1024 : t;
	end
endfunction

function [4:0] rem_less(input [4:0] v, input [4:0] n);
	rem_less = (v > n) ? v - n : 5'd0;
endfunction

wire [63:0] ax_rd2 = {ax_ramr ? ram_rdata : fl_rdata, ax_d0} >> {ax_b, 3'b000};
integer i;

// a redirect of the front end (and everything younger than A)
reg        fl_y, fl_b;              // fl_b: fl_pc is b_npc (the branch case)
reg [31:0] fl_pc;
reg        fc_rd, sl_r;
always @* begin
	fl_y = 0;
	fl_b = 0;
	fl_pc = fpc;
	if (fc_rd) begin
		// the clock after a fast commit that may have changed the code ahead
		// (a store near it) or that was fetched at the boundary: everything
		// fetched since goes, the instruction that just went into A too
		fl_y = 1; fl_b = 0;
		fl_pc = pc;
	end
	if (xfer && b_tk != b_pred) begin
		// a branch or jump the front end did not follow (or followed wrongly)
		fl_y = 1; fl_b = 0;
		fl_pc = b_npc;
		fl_b = 1;
	end
	case (state)
	S_RUN: if (d_flush || sl_r) begin
		// fetched before port B wrote: again (the C fetches after bus_events);
		// or the clock after a sleeping cycle: timed again
		fl_y = 1; fl_b = 0;
		fl_pc = pc;
	end
	S_TE_DONE: if (te_isirq) begin
		fl_y = 1; fl_b = 0;
		fl_pc = te_val;
	end
	S_FIN2:
		// anything fetched after a jump, a store (the code may have changed)
		// or a slow fetch goes
		if (taken || a_fs || a_op == OP_SB || a_op == OP_SH || a_op == OP_SW || a_op == OP_SC ||
		    (a_op >= OP_AMOSWAP && a_op <= OP_AMOMAXU)) begin
			fl_y = 1; fl_b = 0;
			fl_pc = npc;
		end
	default: ;
	endcase
end

always @(posedge clk) begin
	retire <= 0;
	sys_reset <= 0;
	pfic_clr <= 0;
	hpe_we <= 0;
	div_start <= 0;
	wr_en <= 0;
	ld_p <= 0;
	st_p <= 0;
	fc_rd <= 0;

	if (wr_en) xr[wr_rd] <= wr_val;
	ld_p2 <= ld_p;
	if (ld_p) begin
		ld_w <= ld_word;
		ld_rd2 <= ld_rd; ld_b2 <= ld_b; ld_sz2 <= ld_sz; ld_sx2 <= ld_sx;
	end
	if (ld_p2 && ld_rd2 != 6'd32) begin
		wr_en <= 1;
		wr_rd <= ld_rd2[4:0];
		wr_val <= ld_ext;
	end

	if (ce) cyc <= cyc + 1'd1;
	if (ce && !owe_z) begin
		owe <= owe - 1'd1;
		owe_z <= owe == 24'd1;
	end
	if (pfic_reset_req) rst_req <= 1;
	if (mbw) refetch <= 1;
	if (boundary) bseen <= 1;
	spec <= spec_n;
	mspec <= mspec_n;

	// ---- the pipeline's moves (a redirect or a commit below overrides them) ----
	if (xfer) b_v <= 0;
	if (rr_go) begin
		d1_v <= 0;
		b_v <= 1;
		b_mk <= d1_mk; b_fs <= d1_fs; b_fl <= d1_pc[31:16] == 16'd0;
		b_fwd <= d1_alu && d1_rd != 6'd32 && !d1_mk;
		b_simple <= (d1_op >= OP_LUI && d1_op <= OP_BGEU) || (d1_op >= OP_ADDI && d1_op <= OP_AND) ||
		            (d1_op >= OP_MUL && d1_op <= OP_MULHU) || d1_op == OP_FENCE || d1_op == OP_WFI;
		b_isld <= d1_op >= OP_LB && d1_op <= OP_LHU;
		b_isst <= d1_op >= OP_SB && d1_op <= OP_SW;
		b_sz <= (d1_op == OP_LB || d1_op == OP_LBU || d1_op == OP_SB) ? 3'd1 :
		        (d1_op == OP_LH || d1_op == OP_LHU || d1_op == OP_SH) ? 3'd2 : 3'd4;
		b_op <= d1_op; b_rd <= d1_mk ? 6'd32 : d1_rd; b_rs1 <= d1_rs1; b_rs2 <= d1_rs2;
		b_imm <= d1_imm; b_pc <= d1_pc; b_cost <= d1_cost; b_wend <= d1_wend;
		b_opa <= rr_a; b_opb <= rr_b;
		b_wl1 <= rr_l1; b_wl2 <= rr_l2; b_wr1 <= rr_r1; b_wr2 <= rr_r2;
		b_wa1 <= rr_a1; b_wa2 <= rr_a2;
		b_seq <= d1_pc + (d1_len4 ? 32'd4 : 32'd2);
		b_pci <= rr_pci;
		b_pred <= rr_pred;
	end
	if (dec_go) begin
		ir_v <= 0;
		d1_v <= 1;
		d1_pd <= 0;
		d1_mk <= ir_mk; d1_fs <= ir_fs;
		d1_op <= dc_op; d1_rd <= dc_rd; d1_rs1 <= ir_mk ? 5'd0 : dc_rs1; d1_rs2 <= ir_mk ? 5'd0 : dc_rs2;
		d1_imm <= dc_imm; d1_len4 <= dc_len4; d1_cost <= dc_cost; d1_wend <= dc_wend; d1_pc <= ir_pc;
	end

	// ---- the front end ----
	if (fe_mark) begin
		// outside flash and SRAM: fetched after the decision (S_SFETCH)
		if (ir_free) begin
			ir_v <= 1; ir_mk <= 1; ir_fs <= 0; ir <= 0; ir_pc <= fpc;
			fe_st <= FE_STOP;
		end
	end else case (fe_st)
	FE_ADDR: if (fe_issue) begin
		fe_ram <= fe_okram;
		fe_st <= FE_DATA;
	end
	FE_HIADDR: if (fe_issue) fe_st <= FE_HIDATA;
	FE_DATA, FE_HELD, FE_HIDATA, FE_HIHELD: begin
		curd <= fe_w;
		if (fe_strad) begin
			hold16 <= fe_w[31:16];
			fe_st <= fe_issue ? FE_HIDATA : FE_HIADDR;
		end else if (ir_free) begin
			ir_v <= 1; ir_mk <= 0; ir_fs <= 0; ir <= fe_ir; ir_pc <= fpc;
			fpc <= fpc + (fe_len4 ? 32'd4 : 32'd2);
			if (fpc[15:2] == 14'h3FFF && (fe_len4 || fpc[1])) fe_hfl <= fpc[31:16] == 16'hFFFF;
			fe_st <= (fe_same || fe_hi) ? FE_HELD : FE_ADDR;
		end else
			fe_st <= fe_hi ? FE_HIHELD : FE_HELD;
	end
	default: ;   // FE_STOP: until a redirect
	endcase

	if (d1_v && rr_pred && !d1_pd) begin
		// followed (once, whether D1 moves on now or not): what was fetched
		// after it goes (an older redirect below wins)
		ir_v <= 0;
		if (rr_go) d1_v <= 0;
		d1_pd <= 1;
		fpc <= rr_pci;
		fe_hfl <= rr_pci[31:16] == 16'd0;
		fe_hram <= rr_pci[31:15] == 17'h04000;
		fe_st <= FE_ADDR;
	end

	// ---- B: EX, and T1 in two stages ----
	if (!rr_go) begin
		// a load's value for an operand (see rr_l*)
		b_wl1 <= 0; b_wl2 <= 0; b_wr1 <= 0; b_wr2 <= 0;
		if (ld_p) begin b_wa1 <= 0; b_wa2 <= 0; end
		else begin b_wa1 <= b_wa1; b_wa2 <= b_wa2; end
		if (b_wl1) b_opa <= ld_ext;
		if (b_wr1) b_opa <= wr_val;
		if (b_wl2) b_opb <= ld_ext;
		if (b_wr2) b_opb <= wr_val;
		// (A's load is younger than one in ld_p2 or wr_*)
		if (b_wa1 && ld_p) b_opa <= ld_x0;
		if (b_wa2 && ld_p) b_opb <= ld_x0;
	end
	b_e1 <= b_v && !rr_go && !xfer && !b_wt;
	b_exv <= b_v && !rr_go && !xfer && !b_wt && (!b_ismul || b_e1);
	b_prod <= b_mp[63:0];
	b_res <= !b_ismul ? alu : (b_op == OP_MUL) ? b_prod[31:0] : b_prod[63:32];
	b_tk <= ex_taken; b_npc <= ex_npc; b_ea <= ex_ea;
	// where b_ea is (ready with T1's second stage, which xfer waits for)
	// (made with b_res, from ex_ea: a load or store can move with b_exv)
	b_ramf <= ex_ea[31:15] == 17'h04000 && ex_ea[14:0] <= (15'h5000 - {12'd0, b_sz});
	b_flf <= ex_ea[31:16] == 16'd0 && {1'b0, ex_ea[15:0]} <= (17'h10000 - {14'd0, b_sz});
	b_n2 <= ({1'b0, ex_ea[1:0]} + b_sz) > 3'd4;
	s1v <= b_v && !rr_go && !xfer && src_ok;
	s1_kk <= t1_kk;
	s1_kpos <= t1_kk > 0;
	s1_fwpf <= (t1_kk > 0) ? $signed({3'b000, b_wend}) : fw_s;
	s1_mfd <= t1_mfd;
	s1_fd <= fd_s;
	s1_dv <= dr_s;
	s1_fl <= b_fl;
	s1_rsm <= {1'b0, b_rs1} == div_rd || {1'b0, b_rs2} == div_rd;
	q_v <= s1v && b_v && !rr_go && !xfer && src_ok;
	q_t1 <= t1_t1; q_fdpf <= t1_fdpf; q_u <= t1_u; q_fwpf <= s1_fwpf;
	if (!b_v) begin
		// (whether D1 moves or not: B holds it next clock only if it did)
		s1v <= src_ok;
		s1_kk <= t1d_kk;
		s1_kpos <= t1d_kk > 0;
		s1_fwpf <= (t1d_kk > 0) ? $signed({3'b000, d1_wend}) : fw_s;
		s1_mfd <= t1d_mfd;
		s1_fd <= fd_s;
		s1_dv <= dr_s;
		s1_fl <= d1_fl;
		s1_rsm <= {1'b0, d1_rs1} == div_rd || {1'b0, d1_rs2} == div_rd;
	end
	if (xfer && rr_go && t2_fast) begin
		// B moves to A as D1 moves to B: T1's first stage for D1 now, from the
		// packet A is getting (k_fw, k_fd, k_div), so that B can go a clock sooner
		s1v <= 1;
		s1_kk <= by_kk;
		s1_kpos <= by_kp;
		s1_fwpf <= by_kp ? $signed({3'b000, d1_wend}) : by_fw;
		s1_mfd <= by_mfd;
		s1_fd <= by_fd;
		s1_dv <= by_dv;
		s1_fl <= d1_pc[31:16] == 16'd0;
		s1_rsm <= {1'b0, d1_rs1} == div_rd || {1'b0, d1_rs2} == div_rd;
	end

	// ---- B -> A, and T2: A's packet ----
	if (xfer) begin
		a_v <= 1; a_rdy <= 1;
		a_mk <= b_mk; a_fs <= b_fs; a_fl <= b_fl; a_fwd <= b_fwd;
		a_simple <= b_simple; a_isld <= b_isld; a_isst <= b_isst; a_sz <= b_sz; a_pred <= b_pred;
		a_acc <= b_isld || b_isst || (b_op >= OP_LR && b_op <= OP_AMOMAXU);
		a_op <= b_op; a_rd <= b_rd; a_rs1 <= b_rs1; a_rs2 <= b_rs2;
		a_imm <= b_imm; a_pc <= b_pc; a_opa <= b_opa; a_opb <= b_opb; a_seq <= b_seq; a_pci <= b_pci;
		a_cost <= b_cost; a_wend <= b_wend;
		a_res <= b_res; a_taken <= b_tk; a_npc <= b_npc; a_ea <= b_ea;
		p_t1 <= q_t1; p_fdpf <= q_fdpf; p_fwpf <= q_fwpf;
		k_fast <= t2_fast;
		k_ld <= t2_ldr || t2_ldf;
		k_st <= t2_st;
		k_ram <= t2_ramf;
		k_sz <= t2_sz;
		k_wb <= b_fwd;
		k_wfi <= b_op == OP_WFI;
		k_E <= t2_E;
		k_Em1 <= t2_Em1;
		k_E1 <= q_t1 == 0 && t2_c1;
		k_fw <= b_tk ? ($signed({4'd0, b_npc[15:2]}) - 18'sd1) : q_fwpf;
		k_fd <= b_tk ? t2_tfd : t2_nfc;
		k_div <= t2_dv[24] ? 5'd0 : t2_dv[4:0];
		k_frac <= (t2_ldf && !b_fl) ? t2_fs[2:0] : frac;
		k_pair <= t2_st ? (b_fl ? pair : !pair) : 1'b0;
		k_rec <= {recent[1:0], !b_fl && (t2_st || t2_ldr)};
		k_be <= ((t2_sz == 1) ? 4'h1 : (t2_sz == 2) ? 4'h3 : 4'hf) << b_ea[1:0];
		k_wd <= b_opb << {b_ea[1:0], 3'b000};
	end

	case (state)

	// ---- the boundary: chg_run's loop, and the fast commit ----
	S_RUN: begin
		// what a fast commit writes besides, whether it comes or not (only
		// the valid bits below wait for the decision)
		if (!ld_p2) begin
			wr_rd <= a_rd[4:0];
			wr_val <= a_res;
		end
		st_a <= a_ea[14:2];
		st_be <= k_be;
		st_wd <= k_wd;
		ld_rd <= a_rd;
		ld_b <= a_ea[1:0];
		ld_sz <= k_sz;
		ld_ram <= k_ram;
		ld_sx <= a_op == OP_LB || a_op == OP_LH;
		if (d_flush) begin
			// fetched before port B wrote: again (the C fetches after bus_events)
			refetch <= mbw;
			a_v <= 0;
		end
		if (d_rst) begin
			rst_hard <= 0;
			state <= S_RSTW;
			sys_reset <= 1;
			pfic_clr <= 1;
			a_v <= 0;
		end
		if (d_irq) begin
			dce <= spec_e;
			state <= S_PICK;
		end
		if (d_exec && !nochk && ready_fast) wfi <= 0;   // an irq waiting ends WFI
		if (d_sleep) begin
			// asleep: one cycle
			owe <= 24'd1;
			owe_z <= 0;
			fd <= fd_less(fd, 24'd1);
			div_rem <= rem_less(div_rem, 5'd1);
			bseen <= 0;
		end
		if (d_slow) begin
			bseen <= 0;
			nochk <= 0;
			if (a_mk) begin
				a_v <= 0;
				state <= S_SFETCH;
			end else
				state <= S_EX0;
		end
		if (fc) begin
			pc <= a_npc;
			fw <= k_fw;
			fd <= k_fd;
			div_rem <= k_div;
			frac <= k_frac;
			pair <= k_pair;
			{recent2, recent1, recent0} <= k_rec;
			owe <= (spec_e || dce) ? k_Em1 : k_E;
			owe_z <= (spec_e || dce) ? k_E1 : 1'b0;
			if (k_wfi) wfi <= 1;
			icount <= icount + 1'd1;
			retire <= 1;
			retire_pc <= pc;
			retire_instret <= icount + 1'd1;
			retire_cycles <= cyc + {40'd0, dce ? k_Em1 : k_E};
			wr_en <= k_wb;
			ld_p <= k_ld;
			st_p <= k_st;
			bseen <= 0;
			nochk <= 0;
			dce <= 0;
			a_v <= 0;
			fc_rd <= a_fs || (k_st && a_hit);
		end
	end

	S_PICK: state <= S_PICKW;
	S_PICKW: if (pick_done) begin
		bseen <= 0;
		if (pick_valid)
			state <= S_ITRAP;
		else begin
			// nothing to take: the instruction runs, with no second look
			wfi <= 0;
			nochk <= 1;
			state <= S_RUN;
		end
	end
	S_ITRAP: begin
		// pending bit cleared (take), trap_enter(pc, 0x80000000 | irq, 0, irq, irq)
		te_pc <= pc;
		te_cause <= {26'h2000000, pick_irq};
		te_tval <= 0;
		te_vec <= pick_irq;
		te_irqn <= pick_irq;
		te_isirq <= 1;
		state <= S_TE0;
	end

	// ---- fetch() outside the decoded regions: bus_read_slow, after the decision ----
	S_SFETCH: begin
		ax_we <= 0; ax_noflag <= 1; ax_addr <= pc; ax_size <= 3'd2; ax_val <= 0; ax_bi <= 0;
		ax_ret <= S_SFETCH2;
		state <= AX_PRE;
	end
	S_SFETCH2: begin
		w0 <= ax_val;
		if (ax_val[1:0] == 2'b11) begin
			ax_we <= 0; ax_noflag <= 1; ax_addr <= pc + 32'd2; ax_size <= 3'd2; ax_val <= 0; ax_bi <= 0;
			ax_ret <= S_SFETCH3;
			state <= AX_PRE;
		end else begin
			ir_v <= 1; ir_mk <= 0; ir_fs <= 1; ir <= ax_val; ir_pc <= pc;
			d1_v <= 0; b_v <= 0;
			nochk <= 1;
			state <= S_RUN;
		end
	end
	S_SFETCH3: begin
		ir_v <= 1; ir_mk <= 0; ir_fs <= 1; ir <= {ax_val[15:0], w0[15:0]}; ir_pc <= pc;
		d1_v <= 0; b_v <= 0;
		nochk <= 1;
		state <= S_RUN;
	end

	// ---- execute (the slow path) ----
	S_EX0: begin
		r_Tx <= p_t1;
		r_divrdy <= $signed({19'd0, div_rem});
		t1d_r <= ($signed({19'd0, div_rem}) > p_t1) ? $signed({19'd0, div_rem}) : p_t1;   // DIVIDER()
		mcyc <= cyc + {{40{p_t1[23]}}, p_t1};
		flags <= 0;
		extra <= 0;
		elapsed <= 0;
		wb_en <= 0;
		wb_val <= 0;
		taken <= 0;
		npc <= a_seq;
		is_wfi <= 0;
		ldx <= 0;
		sc_ok <= lr_addr == a_opa;
		x_div <= a_op >= OP_DIV && a_op <= OP_REMU;
		x_mret <= a_op == OP_MRET;
		x_csr <= a_op >= OP_CSRRW && a_op <= OP_CSRRCI;
		x_exc <= a_op == OP_ECALL || a_op == OP_EBREAK || a_op == OP_ILLEGAL || a_op == OP_NONE;
		x_ecall <= a_op == OP_ECALL;
		x_ebreak <= a_op == OP_EBREAK;
		x_wfi <= a_op == OP_WFI;
		mul_ar <= {mul_sa & a_opa[31], a_opa};
		mul_br <= {mul_sb & a_opb[31], a_opb};
		state <= (a_op >= OP_CSRRW && a_op <= OP_CSRRCI) ? S_EXC : S_EX1;
		if (a_acc) begin
			// S_EX1's part for an access: board time goes to the issue cycle
			// first, the access is made then
			if (p_t1 != 0) begin
				owe <= p_t1;
				owe_z <= 0;
				elapsed <= p_t1;
			end
			state <= S_MEM0;
			if (bl_c) begin
				// a bus load: S_MEM0 .. AX_START's part here, the access in S_BL0
				flags <= 4'b1000;            // WAIT_LOAD (F_LD)
				ax_we <= 0; ax_noflag <= 0; ax_addr <= a_ea; ax_size <= a_sz; ax_val <= 0; ax_bi <= 0;
				ax_ret <= S_FIN;
				ldx <= 1;
				state <= S_BL0;
			end
		end
	end
	S_EXC: begin
		csr_or <= csr_old;
		state <= S_EX1;
	end
	S_CSRW: state <= S_FIN;                 // csr_write (below)
	S_EX1: begin
		// (accesses went from S_EX0 to S_MEM0; MUL and the one-cycle operations are fast)
		state <= S_FIN;
		if (x_div) begin
			r_Tx <= t1d_r;
			r_divrdy <= t1d_r + 24'sd10;           // DIV_LATENCY
			div_rd <= a_rd;
			div_start <= 1;
			state <= S_DIVW;
		end else if (x_mret)
			state <= S_MRET;
		else if (x_csr) begin
			wb_en <= 1;
			wb_val <= csr_or;
			csr_nr <= csr_new;
			csr_wer <= csr_we;
			state <= S_CSRW;
		end else if (x_exc) begin
			// exception(): the vector, and the first HardFault noted
			te_pc <= pc;
			te_irqn <= 0;
			te_isirq <= 0;
			if (x_ecall) begin
				te_cause <= 32'd11; te_tval <= 0; te_vec <= 6'd5;
			end else if (x_ebreak) begin
				te_cause <= 32'd3; te_tval <= pc; te_vec <= 6'd9;
			end else begin
				te_cause <= 32'd2; te_tval <= a_imm; te_vec <= 6'd3;
				if (!faulted) begin
					faulted <= 1;
					fault_cause <= 32'd2;
					fault_pc <= pc;
					fault_tval <= a_imm;
				end
			end
			state <= S_TE0;
		end else begin
			// (not reached: what the fast path does)
			if (a_fwd) begin wb_en <= 1; wb_val <= a_res; end
			if (a_taken) begin npc <= a_npc; taken <= 1; end
			if (x_wfi) is_wfi <= 1;
		end
	end

	S_MEM0: if (owe_z && !busy) begin
		ax_noflag <= 0; ax_val <= 0; ax_bi <= 0;
		state <= AX_PRE;
		case (a_op)
		OP_LB, OP_LBU: begin ax_we <= 0; ax_addr <= a_ea; ax_size <= 3'd1; ax_ret <= S_FIN; ldx <= 1; end
		OP_LH, OP_LHU: begin ax_we <= 0; ax_addr <= a_ea; ax_size <= 3'd2; ax_ret <= S_FIN; ldx <= 1; end
		OP_LW: begin ax_we <= 0; ax_addr <= a_ea; ax_size <= 3'd4; ax_ret <= S_FIN; ldx <= 1; end
		OP_SB: begin ax_we <= 1; ax_addr <= a_ea; ax_size <= 3'd1; ax_wdata <= a_opb; ax_ret <= S_FIN; end
		OP_SH: begin ax_we <= 1; ax_addr <= a_ea; ax_size <= 3'd2; ax_wdata <= a_opb; ax_ret <= S_FIN; end
		OP_SW: begin ax_we <= 1; ax_addr <= a_ea; ax_size <= 3'd4; ax_wdata <= a_opb; ax_ret <= S_FIN; end
		OP_LR: begin
			ax_we <= 0; ax_addr <= a_opa; ax_size <= 3'd4; ax_ret <= S_FIN; ldx <= 1;
			lr_valid <= 1;
			lr_addr <= a_opa;
		end
		OP_SC: begin
			lr_valid <= 0;
			wb_en <= 1;
			if (lr_valid && sc_ok) begin
				wb_val <= 0;
				ax_we <= 1; ax_addr <= a_opa; ax_size <= 3'd4; ax_wdata <= a_opb; ax_ret <= S_FIN;
			end else begin
				wb_val <= 1;
				state <= S_FIN;
			end
		end
		default: begin  // AMO: the load, then the store
			ax_we <= 0; ax_addr <= a_opa; ax_size <= 3'd4; ax_ret <= S_MEM_AMO;
		end
		endcase
	end
	S_BL0: begin
		// FIN's sums (no stall), for S_BL1
		f_E <= fin_E;
		f_fd <= fin_fd;
		f_frac <= fin_frac;
		f_pair <= fin_pair;
		f_rec <= fin_rec;
		if (owe_z && !busy) begin
			// board time is at the issue cycle: the access (AX_SL's)
			bus_req <= 1; bus_we <= 0; bus_addr <= a_ea; bus_size <= a_sz; bus_wdata <= 0;
			sa_r <= a_ea; ss_r <= a_sz;
			bl_n <= 0;
			bl_ack <= 0;
			state <= S_BL1;
		end
	end
	S_BL1: begin
		// FIN1 and FIN3, every clock (valid from the third)
		bus_req <= 0;
		h_fdt <= f_fdt;
		g_fd <= (f_fds0 < -24'sd1024) ? -24'sd1024 : f_fds0;
		g_div <= f_dv0[23] ? 5'd0 : f_dv0[4:0];
		g_fw <= taken ? ($signed({4'd0, npc[15:2]}) - 18'sd1) : p_fwpf;
		g_owe <= f_E - elapsed;
		if (bl_n != 2'd3) bl_n <= bl_n + 1'd1;
		if (bus_ack) begin bl_ack <= 1; bl_d <= bus_rdata; bl_st <= stall; end
		if ((bus_ack || bl_ack) && bl_n >= 2'd2) begin
			if (bl_stall == 0) begin
				wb_en <= 1;
				wb_val <= bl_ext;
				state <= S_FIN2;
			end else begin
				// the peripheral took board time: as AX_BUSW, then AX_SLN, S_FIN ...
				ax_val <= bl_raw;
				owe <= {7'd0, bl_stall};
				owe_z <= 0;
				extra <= extra + {7'd0, bl_stall};
				elapsed <= elapsed + {7'd0, bl_stall};
				state <= AX_SLN;
			end
		end
	end
	S_MEM_LD: begin
		wb_en <= 1;
		case (a_op)
		OP_LB: wb_val <= {{24{ax_val[7]}}, ax_val[7:0]};
		OP_LH: wb_val <= {{16{ax_val[15]}}, ax_val[15:0]};
		default: wb_val <= ax_val;
		endcase
		state <= S_FIN;
	end
	S_MEM_AMO: begin
		amo_old <= ax_val;
		state <= S_MEM_AMO3;
	end
	S_MEM_AMO3: begin
		am_lts <= $signed(amo_old) < $signed(a_opb);
		am_ltu <= amo_old < a_opb;
		am_sum <= amo_old + a_opb;
		state <= S_MEM_AMO4;
	end
	S_MEM_AMO4: begin
		amo_r <= amo_v;
		state <= S_MEM_AMO2;
	end
	S_MEM_AMO2: begin
		// the store of the AMO's result (amo_v needs amo_old: one clock)
		ax_we <= 1; ax_addr <= a_opa; ax_size <= 3'd4; ax_val <= 0; ax_bi <= 0;
		ax_wdata <= amo_r;
		ax_ret <= S_FIN;
		state <= AX_PRE;
		wb_en <= 1;
		wb_val <= amo_old;
	end

	S_MUL: state <= S_MUL1;
	S_MUL1: begin
		prod <= mul_p[63:0];
		state <= S_MUL2;
	end
	S_MUL2: begin
		wb_en <= 1;
		wb_val <= (a_op == OP_MUL) ? prod[31:0] : prod[63:32];
		state <= S_FIN;
	end
	S_DIVW: if (div_done) begin
		wb_en <= 1;
		wb_val <= div_result;
		state <= S_FIN;
	end

	// ---- mret() ----
	S_MRET: begin
		npc <= mepc;
		taken <= 1;
		if (depth != 0) begin
			depth <= depth - 1'd1;
			// back to what the trap interrupted, handler or not
			mstatus <= (t_mst[dep_m1] & ~MIE) | (mstatus[7] ? MIE : 32'd0);
			mepc <= t_mepc[dep_m1];
			mcause <= t_mcause[dep_m1];
			if (t_hpe[dep_m1]) begin
				mr_d <= dep_m1;
				mr_i <= 0;
				state <= S_MR_HPE;
			end else
				state <= S_FIN;
		end else begin
			// an mret with no trap to return from
			mstatus <= (mstatus & ~(MIE | MPP)) | (mstatus[7] ? MIE : 32'd0) | MPIE;
			state <= S_FIN;
		end
	end
	S_MR_HPE: begin
		// read hpe[d][mr_i] (hpe_ra below); write the one read the clock before
		if (mr_i != 0) begin
			wr_en <= 1;
			wr_rd <= hpe_reg(mr_i[3:0] - 1'd1);
			wr_val <= hpe_q;
		end
		mr_i <= mr_i + 1'd1;
		if (mr_i == 16) state <= S_FIN;
	end

	// ---- trap_enter() ----
	S_TE0: begin
		t_mepc[dep_idx] <= mepc;
		t_mcause[dep_idx] <= mcause;
		t_mst[dep_idx] <= mstatus;
		mepc <= te_pc;
		mcause <= te_cause;
		mtval <= te_tval;
		// with nesting on the core keeps MIE on an interrupt: the PFIC's
		// priorities decide what may preempt
		mstatus <= ((mstatus & ~(MPIE | MPP)) | (mstatus[3] ? MPIE : 32'd0) | MPP) &
		           ~((intsyscr[1] && te_isirq) ? 32'd0 : MIE);
		t_hpe[dep_idx] <= intsyscr[0];
		t_exc[dep_idx] <= !te_isirq;
		t_irq[dep_idx] <= te_irqn;
		if (depth < 8) depth <= depth + 1'd1;
		wfi <= 0;
		te_idx <= dep_idx;
		te_i <= 0;
		state <= intsyscr[0] ? S_TE_HPE : S_TE_VEC;
	end
	S_TE_HPE: begin
		hpe_we <= 1;
		hpe_wa <= {te_idx, te_i[3:0]};
		hpe_wd <= xr[hpe_reg(te_i[3:0])];
		te_i <= te_i + 1'd1;
		if (te_i == 15) state <= S_TE_VEC;
	end
	S_TE_VEC: begin
		if (!mtvec[0]) begin
			te_val <= {mtvec[31:2], 2'b00};
			state <= S_TE_DONE;
		end else if (mtvec[1]) begin
			// the vector table holds addresses
			ax_we <= 0; ax_noflag <= 1; ax_size <= 3'd4; ax_val <= 0; ax_bi <= 0;
			ax_addr <= {mtvec[31:2], 2'b00} + {24'd0, te_vec, 2'b00};
			ax_ret <= S_TE_VL;
			state <= AX_PRE;
		end else begin
			te_val <= {mtvec[31:2], 2'b00} + {24'd0, te_vec, 2'b00};
			state <= S_TE_DONE;
		end
	end
	S_TE_VL: begin
		te_val <= ax_val & ~32'd1;
		state <= S_TE_DONE;
	end
	S_TE_DONE: begin
		if (te_isirq) begin
			// cpu_take_interrupt: 1 + FLASH_WORD cycles (one of them may have
			// been the decision clock's), fetching starts over
			pc <= te_val;
			owe <= dce ? 24'd4 : 24'd5;
			owe_z <= 0;
			fw <= $signed({4'd0, te_val[15:2]}) - 18'sd1;
			fd <= 0;
			div_rem <= rem_less(div_rem, 5'd5);
			nochk <= 1;
			dce <= 0;
			a_v <= 0;
			state <= S_RUN;
		end else begin
			npc <= te_val;
			taken <= 1;
			wb_en <= 0;
			state <= S_FIN;
		end
	end

	// ---- the end of cpu_exec's loop body (slow path) ----
	S_FIN: begin
		if (ldx) begin
			// a load's value
			wb_en <= 1;
			case (a_op)
			OP_LB: wb_val <= {{24{ax_val[7]}}, ax_val[7:0]};
			OP_LH: wb_val <= {{16{ax_val[15]}}, ax_val[15:0]};
			default: wb_val <= ax_val;
			endcase
		end
		ldx <= 0;
		f_E <= fin_E;
		f_fd <= fin_fd;
		f_frac <= fin_frac;
		f_pair <= fin_pair;
		f_rec <= fin_rec;
		state <= S_FIN1;
	end
	S_FIN1: begin
		h_fdt <= f_fdt;
		state <= S_FIN3;
	end
	S_FIN3: begin
		g_fd <= (f_fds0 < -24'sd1024) ? -24'sd1024 : f_fds0;
		g_div <= f_dv0[23] ? 5'd0 : f_dv0[4:0];
		g_fw <= taken ? ($signed({4'd0, npc[15:2]}) - 18'sd1) : p_fwpf;
		g_owe <= f_E - elapsed;
		state <= S_FIN2;
	end
	S_FIN2: begin
		if (wb_en && a_rd != 6'd32) begin
			wr_en <= 1;
			wr_rd <= a_rd[4:0];
			wr_val <= wb_val;
		end
		if (is_wfi) wfi <= 1;
		fw <= g_fw;
		fd <= g_fd;
		div_rem <= g_div;
		frac <= f_frac;
		pair <= f_pair;
		{recent2, recent1, recent0} <= f_rec;
		owe <= g_owe;
		owe_z <= g_owe == 0;
		pc <= npc;
		icount <= icount + 1'd1;
		retire <= 1;
		retire_pc <= pc;
		retire_instret <= icount + 1'd1;
		retire_cycles <= cyc + {40'd0, g_owe};
		nochk <= 0;
		dce <= 0;
		a_v <= 0;
		state <= S_RUN;
	end

	// ---- load() / store() ----
	AX_PRE: begin
		ax_ramf <= ax_ramf_c;
		ax_flf <= ax_flf_c;
		state <= AX_START;
	end
	AX_START: if (!busy) begin
		ax_ramr <= ax_ramf;
		if (ax_ramf) begin
			if (!ax_noflag) flags[ax_we ? F_RST : F_RLD] <= 1;
			if (ax_we) state <= ax_need2 ? AX_WR2 : ax_ret;
			else state <= AX_RD1;
		end else if (ax_flf && !ax_we) begin
			if (!ax_noflag) flags[F_FLD] <= 1;
			state <= AX_RD1;
		end else begin
			if (!ax_noflag && !ax_we) flags[F_LD] <= 1;
			// AX_SL0's work for the first byte (ax_bi is 0 here)
			sa_r <= ax_addr;
			sa_fl <= ax_addr < 32'h00010000 || (ax_addr >= 32'h08000000 && ax_addr < 32'h08010000);
			sa_ram <= ax_off < 32'h00005000;
			sa_pf <= ax_addr >= 32'hE000E000 && ax_addr < 32'hE000F000;
			sa_sys <= ax_addr >= 32'h1FFF0000 && ax_addr < 32'h20000000;
			sa_hi <= ax_addr >= 32'h40000000;
			ss_r <= ax_split ? 3'd1 : ax_size;
			sdata_r <= ax_split ? {24'd0, ax_wdata[7:0]} : ax_wdata;
			state <= AX_SL;
		end
	end
	AX_RD1: begin
		ax_d0 <= ax_ramr ? ram_rdata : fl_rdata;
		if (ax_need2) state <= AX_RD2;
		else begin
			ax_val <= ((ax_ramr ? ram_rdata : fl_rdata) >> {ax_b, 3'b000}) & ax_mask;
			state <= ax_ret;
		end
	end
	AX_RD2: begin
		ax_val <= ax_rd2[31:0] & ax_mask;
		state <= ax_ret;
	end
	AX_WR2: state <= ax_ret;
	AX_SL0: begin
		// the address of this access and where it goes
		sa_r <= sa_n;
		ss_r <= ax_split ? 3'd1 : ax_size;
		sdata_r <= ax_split ? {24'd0, ax_wdata[{ax_bi, 3'b000} +: 8]} : ax_wdata;
		sa_fl <= sa_n < 32'h00010000 || (sa_n >= 32'h08000000 && sa_n < 32'h08010000);
		sa_ram <= sa_noff < 32'h00005000;
		sa_pf <= sa_n >= 32'hE000E000 && sa_n < 32'hE000F000;
		sa_sys <= sa_n >= 32'h1FFF0000 && sa_n < 32'h20000000;
		sa_hi <= sa_n >= 32'h40000000;
		state <= AX_SL;
	end
	AX_SL: if (!busy) begin
		// one access of bus_read_slow / bus_write_slow (a byte of a split one)
		if (!ax_we) begin
			if (sa_fl) state <= AX_SLF;
			else if (sa_ram) state <= AX_SLR;
			else if (sa_pf) state <= AX_SLP;
			else if (sa_hi || sa_sys) begin
				bus_req <= 1; bus_we <= 0; bus_addr <= sa_r; bus_size <= ss; bus_wdata <= 0;
				state <= AX_BUSW;
			end else
				state <= AX_SLN;                 // nothing there: reads 0
		end else begin
			if (sa_ram || sa_pf) state <= AX_SLN; // written on port A / in the PFIC this clock
			else if (sa_fl || sa_hi) begin
				bus_req <= 1; bus_we <= 1; bus_addr <= sa_r; bus_size <= ss; bus_wdata <= sdata;
				state <= AX_BUSW;
			end else
				state <= AX_SLN;                 // nothing there
		end
	end
	AX_SLF: begin
		ax_val <= ax_val | (s_frd << ax_acc_sh);
		state <= AX_SLN;
	end
	AX_SLR: begin
		ax_val <= ax_val | (s_rrd << ax_acc_sh);
		state <= AX_SLN;
	end
	AX_SLP: state <= AX_SLP3;               // the PFIC's word is registered
	AX_SLP3: begin
		pf_q <= pf_rdata;
		state <= AX_SLP2;
	end
	AX_SLP2: begin
		ax_val <= ax_val | (pf_q << ax_acc_sh);
		state <= AX_SLN;
	end
	AX_BUSW: begin
		bus_req <= 0;
		if (bus_ack) begin
			if (!ax_we) ax_val <= ax_val | ((bus_rdata & s_mask) << ax_acc_sh);
			if (stall != 0) begin
				// the peripheral took board time (m->cycles += inside the access): it
				// passes now, before the instruction goes on; no boundary comes in it
				owe <= {7'd0, stall};
				owe_z <= 0;
				extra <= extra + {7'd0, stall};
				elapsed <= elapsed + {7'd0, stall};
			end
			state <= AX_SLN;
		end
	end
	AX_SLN: if (owe_z) begin
		if (ax_split && {1'b0, ax_bi} != ax_size - 1'd1) begin
			ax_bi <= ax_bi + 1'd1;
			state <= AX_SL0;
		end else
			state <= ax_ret;
	end

	// ---- reset: cpu_reset, and bus_reset's part (the PFIC) ----
	S_RSTW: begin
		if (!pfic_busy && !pfic_clr) begin
			nochk <= !rst_hard;
			a_v <= 0;
			fc_rd <= 1;                      // everything is fetched again, from pc
			state <= S_RUN;
		end
	end

	default: state <= S_RUN;
	endcase

	// csr_write
	if (state == S_CSRW && csr_wer)
		case (csr)
		12'h300: mstatus <= csr_nr & 32'h00001888;
		12'h305: mtvec <= csr_nr;
		12'h340: mscratch <= csr_nr;
		12'h341: mepc <= csr_nr & ~32'd1;
		12'h342: mcause <= csr_nr;
		12'h343: mtval <= csr_nr;
		12'h800: mstatus <= (mstatus & ~(MIE | MPIE)) | (csr_nr & (MIE | MPIE));
		12'h804: intsyscr <= csr_nr;
		12'hBC0: corecfgr <= csr_nr;
		default: misc[csr[3:0]] <= csr_nr;
		endcase

	// a sleeping cycle moved fd and div_rem: everything is timed again
	sl_r <= d_sleep;
	if (d_sleep || sl_r) a_v <= 0;

	if (rd_b && !xfer) begin
		// a branch the front end did not follow (or followed wrongly), put
		// right as soon as EX knows, while it waits in B (fl_y wins)
		ir_v <= 0;
		d1_v <= 0;
		b_pred <= b_tk;
		fpc <= b_npc;
		fe_hfl <= b_npc[31:16] == 16'd0;
		fe_hram <= b_npc[31:15] == 17'h04000;
		fe_ram <= rd_ram;
		fe_st <= rd_iss ? FE_DATA : FE_ADDR;
	end
	if (fc_rd) a_v <= 0;
	// the redirect: everything younger than A goes, the front end starts at fl_pc
	if (fl_y) begin
		ir_v <= 0;
		d1_v <= 0;
		b_v <= 0;
		fpc <= fl_pc;
		fe_hfl <= fl_pc[31:16] == 16'd0;
		fe_hram <= fl_pc[31:15] == 17'h04000;
		fe_st <= FE_ADDR;
		if (fl_b && rd_iss) begin
			fe_ram <= rd_ram;
			fe_st <= FE_DATA;
		end
	end

	// cpu_reset: everything in ChgCpu to zero, pc = entry, mstatus = MPP
	if (state == S_RSTW) begin
		// (in S_RSTW, which a power-on reset or a software one always goes
		// through: nothing runs between the decision and the end of it)
		for (i = 0; i < 32; i = i + 1) xr[i] <= 0;
		for (i = 0; i < 16; i = i + 1) misc[i] <= 0;
		for (i = 0; i < 8; i = i + 1) begin
			t_irq[i] <= 0; t_mepc[i] <= 0; t_mcause[i] <= 0; t_mst[i] <= 0;
		end
	end
	if (d_rst || reset) begin
		t_exc <= 0;
		t_hpe <= 0;
		pc <= entry;
		mstatus <= MPP;
		mtvec <= 0; mscratch <= 0; mepc <= 0; mcause <= 0; mtval <= 0;
		intsyscr <= 0; corecfgr <= 0;
		depth <= 0;
		wfi <= 0;
		lr_valid <= 0;
		lr_addr <= 0;
		faulted <= 0;
		fault_cause <= 0; fault_pc <= 0; fault_tval <= 0;
		fw <= $signed({4'd0, entry[15:2]}) - 18'sd1;
		fd <= 0;
		div_rem <= 0;
		div_rd <= 0;
		frac <= 0;
		pair <= 0;
		{recent2, recent1, recent0} <= 0;
		rst_req <= 0;
		bseen <= 0;
		refetch <= 0;
		wr_en <= 0;
		ld_p <= 0;
		ld_p2 <= 0;
		st_p <= 0;
		dce <= 0;
		a_v <= 0;
		ir_v <= 0;
		d1_v <= 0;
		b_v <= 0;
		fe_st <= FE_STOP;
	end
	if (reset) begin
		state <= S_RSTW;
		rst_hard <= 1;
		pfic_clr <= 1;
		owe <= 0;
		owe_z <= 1;
		spec <= 0;
		mspec <= 0;
		cyc <= 0;
		icount <= 0;
		nochk <= 0;
		bus_req <= 0;
		sys_reset <= 0;
	end
end

// the HPE RAM's read address: mret's restore
always @* hpe_ra = {mr_d, mr_i[3:0]};

endmodule
