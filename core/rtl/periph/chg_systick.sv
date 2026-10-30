// The QingKe SysTick at 0xE000_F000: CTLR, SR, CNT (64 bit), CMP (64 bit).
//
// Translates stk_read, stk_write, stk_event, stk_count, stk_plan and
// stk_rebase in emu/src/bus.c. The C keeps a base count and the cycle it was
// taken, and works the counter out from them when asked; here the counter
// runs, one tick every 8 board cycles (STCLK=0) or every cycle (STCLK=1).
// Every write rebases in the C, which restarts the divide-by-8 phase: so
// does a write here.
//
// Outputs to the PFIC:
// - irq: the level stk_write and stk_event give pfic_set_line for
//   CHG_IRQ_SYSTICK. A match raises it (when STIE) at the boundary after it;
//   only a write lowers it.
// - sw_pend: one clock, with the ack of a CTLR write that has SWIE (bit 31)
//   set. The C does m->pfic.pending |= 1 << CHG_IRQ_SW there: the PFIC ORs
//   this pulse into its pending bit 14. SysTick itself never touches
//   pending for its own interrupt.
//
// What the counter does, per mode (stk_count):
//   up, no reload      counts up for ever; matches once, at CMP
//   up, reload         0..CMP and round again; matches at CMP. Above CMP
//                      (CNT written larger) it counts up for ever, no match
//   down, no reload    counts down through 0 (and on, wrapping at 2^64);
//                      matches once, at 0
//   down, reload       CMP..0 and round again; matches at 0
// A match is found on the ce edge that reaches it (the C's next_match). Its
// effect, SR bit 0 and the line, waits for the next instruction boundary:
// the C runs stk_event from bus_events at the top of chg_run's loop only,
// so a load whose issue cycle is past the match but whose instruction began
// before it (rv32.c moves m->cycles on to the issue cycle for fetch and
// divider waits first) still reads SR clear. 'boundary' is that moment; a
// match not yet taken is 'pend'. A write in between replans in the C
// (stk_plan from now), and the match is never taken: a write clears 'pend'.
// Matches are found by the count reaching the value instead of by the C's
// planned cycle: the same cycles, since nothing but a write changes CMP or
// the mode, and a write replans.
//
// Down with reload and CNT above CMP is odd in the C. stk_count works out
//   CMP - ((CMP - CNT + ticks) mod 2^64) mod (CMP + 1)
// so straight after the write the counter reads CMP - ((2^64 - (CNT - CMP))
// mod (CMP+1)), not CNT, and counts down from there with reloads to CMP;
// when the 64-bit sum wraps (CNT - CMP ticks on) it goes to CMP, and the
// planned match is CNT ticks on, at 0. That is modelled: 'weird' and
// 'wrap_left', and a 64-step divider (the write is acked when it is done).
//
// Not modelled: the C's planned cycle wrapping past 2^64 (stk_plan with a
// count near 2^64, for example a write after a down count without reload
// has passed 0), and CMP = 2^64-1 with reload, where the C
// divides by zero.

module chg_systick
(
	input             clk,
	input             reset,        // chg_reset (power-on or software): bus_reset
	input             ce,
	input             boundary,     // an instruction boundary: chg_run's bus_events (see above)

	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,
	output reg        ack,

	output reg        irq,          // CHG_IRQ_SYSTICK line
	output reg        sw_pend,      // set pending CHG_IRQ_SW (one clock)
	output            ev            // an event the next boundary takes (it may change irq)
);

localparam STE = 0, STIE = 1, STCLK = 2, STRE = 3, MODE = 4, INIT = 5;

reg  [31:0] ctlr;
reg         sr;
reg  [63:0] cmp;
reg  [63:0] cmp_m1;         // cmp - 1 (counting up, cnt + 1 == cmp is cnt == cmp_m1)
reg  [63:0] cnt;            // what stk_count gives now
reg   [2:0] pre;            // board cycles into the present tick (STCLK=0)
reg         weird;          // down, reload, CNT was above CMP: not wrapped yet
reg  [63:0] wrap_left;      // ticks until it wraps
reg         pend;           // a match the C's stk_event has not taken yet

// a boundary in the same clock as an access comes first
wire        take = pend & boundary;
assign ev = pend;
wire        sr_b = sr | take;

// ---------------------------------------------------------------------------
// the bus: byte lanes as bus_read_slow / bus_write_slow shift them
// ---------------------------------------------------------------------------

wire  [4:0] sh = {addr[1:0], 3'b000};
wire [31:0] wv = wdata << sh;           // the C's v = value << shift
wire  [7:0] off = {addr[7:2], 2'b00};

function [31:0] lane_r(input [31:0] v, input [4:0] s, input [2:0] sz);
	reg [31:0] x;
	begin
		x = v >> s;
		if (sz == 3'd1) x = x & 32'h000000ff;
		else if (sz == 3'd2) x = x & 32'h0000ffff;
		lane_r = x;
	end
endfunction

reg [31:0] rv;
always @* begin
	case (off)
	8'h00:   rv = ctlr;
	8'h04:   rv = {31'd0, sr_b};
	8'h08:   rv = cnt[31:0];
	8'h0c:   rv = cnt[63:32];
	8'h10:   rv = cmp[31:0];
	8'h14:   rv = cmp[63:32];
	default: rv = 32'd0;
	endcase
end

// the state after a write (stk_write up to stk_plan)
reg  [31:0] n_ctlr;
reg         n_sr;
reg  [63:0] n_cmp, n_base;
always @* begin
	n_ctlr = ctlr; n_sr = sr_b; n_cmp = cmp;
	n_base = cnt;                       // stk_rebase
	case (off)
	8'h00: begin
		n_ctlr = wv & ~32'h80000020;    // ~STK_INIT & ~STK_SWIE
		if (wv[INIT]) n_base = wv[MODE] ? cmp : 64'd0;
	end
	8'h04: n_sr = wv[0];
	8'h08: n_base[31:0]  = wv;
	8'h0c: n_base[63:32] = wv;
	8'h10: n_cmp[31:0]   = wv;
	8'h14: n_cmp[63:32]  = wv;
	default: ;
	endcase
end
reg         wr2, w_sr, w_swp;
reg  [31:0] w_ctlr;
reg  [63:0] w_cmp, w_base;
wire w_weird = w_ctlr[STE] & w_ctlr[MODE] & w_ctlr[STRE] & (w_base > w_cmp);

// ---------------------------------------------------------------------------
// one tick (stk_count advanced by one, and whether stk_event matches)
// ---------------------------------------------------------------------------

reg [63:0] t_cnt;
reg        t_match;
always @* begin
	if (ctlr[MODE]) begin
		// (t_cnt == 0 worked out from cnt and cmp, not from the sum: 96 MHz)
		if (ctlr[STRE]) begin
			t_cnt = (cnt == 64'd0) ? cmp : cnt - 64'd1;
			if (weird && wrap_left == 64'd1) t_cnt = cmp;
			t_match = (((cnt == 64'd0) || (weird && wrap_left == 64'd1)) ? (cmp == 64'd0) : (cnt == 64'd1)) &&
			          !(weird && wrap_left != 64'd1);
		end else begin
			t_cnt = cnt - 64'd1;
			t_match = cnt == 64'd1;
		end
	end else begin
		t_cnt = (ctlr[STRE] && cnt == cmp) ? 64'd0 : cnt + 64'd1;
		t_match = (ctlr[STRE] && cnt == cmp) ? (cmp == 64'd0) : (cnt == cmp_m1);
	end
end
wire tick = ctlr[STE] & (ctlr[STCLK] | (pre == 3'd7));

// ---------------------------------------------------------------------------
// the 64-bit remainder for the odd case: (2^64 - (CNT - CMP)) mod (CMP + 1)
// ---------------------------------------------------------------------------

reg         busy;
reg   [6:0] dstep;
reg  [63:0] dnum;           // dividend, shifted out from the top
reg  [64:0] drem;
reg  [63:0] dden;           // divisor CMP + 1
wire [64:0] dtry = {drem[63:0], dnum[63]};

always @(posedge clk) begin
	ack <= 0;
	sw_pend <= 0;
	if (reset) begin
		ctlr <= 0; sr <= 0; cmp <= 0; cmp_m1 <= 64'hFFFFFFFFFFFFFFFF; cnt <= 0; pre <= 0;
		weird <= 0; wrap_left <= 0; irq <= 0; busy <= 0; pend <= 0; wr2 <= 0;
	end else if (busy) begin
		// restoring division, one bit a clock (board time is held meanwhile)
		drem <= (dtry >= {1'b0, dden}) ? dtry - {1'b0, dden} : dtry;
		dnum <= {dnum[62:0], 1'b0};
		dstep <= dstep - 7'd1;
		if (dstep == 7'd0) begin
			busy <= 0;
			ack <= 1;
			cnt <= cmp - drem[63:0];
		end
	end else if (wr2) begin
		// a write, its second clock: from the registered new state (96 MHz)
		wr2 <= 0;
		ctlr <= w_ctlr;
		sr <= w_sr;
		cmp <= w_cmp;
		cmp_m1 <= w_cmp - 64'd1;
		cnt <= w_base;
		pre <= 0;
		pend <= 0;
		irq <= w_sr & w_ctlr[STIE];
		if (w_swp) sw_pend <= 1;
		weird <= w_weird;
		wrap_left <= w_base - w_cmp;
		if (w_weird) begin
			busy <= 1;
			dstep <= 7'd64;             // 64 steps, then the result
			dnum <= w_cmp - w_base;     // ~(base - cmp) + 1
			drem <= 0;
			dden <= w_cmp + 64'd1;
		end else
			ack <= 1;
	end else if (req) begin
		if (we) begin
			// the state after the write, applied the clock after
			wr2 <= 1;
			w_ctlr <= n_ctlr;
			w_sr <= n_sr;
			w_cmp <= n_cmp;
			w_base <= n_base;
			w_swp <= off == 8'h00 && wv[31];
		end else begin
			rdata <= lane_r(rv, sh, size);
			ack <= 1;
			if (take) begin
				sr <= 1;
				if (ctlr[STIE]) irq <= 1;
				pend <= 0;
			end
		end
	end else begin
		if (take) begin                     // stk_event
			sr <= 1;
			if (ctlr[STIE]) irq <= 1;
			pend <= 0;
		end
		if (ce && ctlr[STE]) begin
			pre <= ctlr[STCLK] ? 3'd0 : pre + 3'd1;
			if (tick) begin
				cnt <= t_cnt;
				if (weird) begin
					wrap_left <= wrap_left - 64'd1;
					if (wrap_left == 64'd1) weird <= 0;
				end
				if (t_match) pend <= 1;
			end
		end
	end
end

endmodule
