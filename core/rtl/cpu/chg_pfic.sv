// The PFIC, the QingKe's interrupt controller at 0xE000_E000.
//
// Translates pfic_read, pfic_write (emu/src/bus.c) and pick_irq,
// cpu_irq_waiting (emu/src/rv32.c). The trap stack that pick_irq and the
// GISR/IACTR reads look at lives in chg_cpu, which hands in what is needed
// (depth, the set of active interrupts).
//
// State, as ChgPfic: enabled, pending (set by software, or by SysTick's
// SWIE through sw_pend), the 256 priority bytes, ITHRESDR, SCTLR, and the
// VTF registers. The VTF registers are storage only: the C keeps them for
// reads but never vectors through them (trap_enter uses mtvec), so neither
// does this. 'line' is not kept here: it is irq_line, the levels the
// peripherals drive (pfic_set_line).
//
// The priorities are a RAM in four byte lanes (prio[4j+b] in lane b, word
// j), read a word at a time. pick_irq is a scan of the 16 words that hold
// the priorities of interrupts 0..63, one a clock: 18 clocks with board time
// stopped, and only when an interrupt is ready while MIE is set (chg_cpu).
// (Each word is reduced in two clocks, then folded in.)
// A clear (reset) zeroes the RAM in 64 clocks; busy is high meanwhile.
//
// Accesses: rd_req or wr_req for one clock with off = addr - 0xE000E000,
// size and wdata as bus_read_slow/bus_write_slow get them; rdata is valid
// two clocks after rd_req, shifted and masked as bus_read_slow returns it.
//
// As the C does it: a priority write that runs past byte 255 (a 2- or 4-byte
// store at 0xE000E4FD..F) writes past the end of prio[], into the low bytes
// of ITHRESDR, which follows it in ChgPfic. So ITHRESDR is kept 32 bits wide:
// it reads back whole, and pick_irq's "ithresdr &&" tests all of it.

module chg_pfic
(
	input             clk,
	input             clr,          // bus_reset: power-on or software reset
	output            busy,

	input      [63:0] irq_line,     // ChgPfic.line (chg_cpu: as it was the clock before)
	input      [63:0] irq_raw,      // ... and as it is now (only to work the two below out a clock early)
	input             sw_pend,      // SysTick CTLR write with SWIE: pending |= 1 << CHG_IRQ_SW

	input             rd_req,
	input             wr_req,
	input      [11:0] off,
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,

	input       [3:0] depth,        // trap_depth
	input      [63:0] active,       // the interrupts in the trap stack (IACTR)
	output            ready_any,    // cpu_irq_waiting
	output            ready_fast,   // the same, with pending & enabled a clock late (for the decision)
	output            pick_want,    // a scan is worth running: ready_fast, less what the last scan turned down

	input             pick_start,
	input             in_trap,      // trap_depth > 0
	input             nest,         // INTSYSCR bit 1
	output reg        pick_done,
	output reg        pick_valid,
	output reg  [5:0] pick_irq,

	input             take,         // the interrupt taken: its pending bit cleared
	input       [5:0] take_irq,

	output reg        reset_req     // one clock: KEY3+SYSRESET at 0x48, or SCTLR bit 31
);

reg [63:0] enabled, pending;
reg [31:0] ithresdr;
reg [31:0] sctlr;
reg [31:0] vtf_addr0, vtf_addr1, vtf_addr2, vtf_addr3;
reg [31:0] vtf_id;

wire [63:0] ready = (pending | irq_line) & enabled;
assign ready_any = ready != 0;
wire [63:0] ready_lp = pending | irq_line;     // what the pending registers read
reg         pe_any;
reg  [63:0] ready_r;
always @(posedge clk) ready_r <= ready;

// The last scan found nothing, and since then no interrupt has become ready
// that was not ready then (fewer candidates cannot make pick_irq find one),
// nothing was written, and the trap stack is as it was.
reg         noirq;
reg  [63:0] ready_scan;
reg   [3:0] depth_r;
reg         nest_r;
always @(posedge clk) begin
	depth_r <= depth;
	nest_r <= nest;
end
// as one mask (registered: none of it changes in the clock before a decision)
reg  [63:0] want_m;
reg         want_p;
always @(posedge clk) begin
	want_m <= noirq ? (enabled & ~ready_scan) : enabled;
	want_p <= noirq ? ((pending & enabled & ~ready_scan) != 0) : ((pending & enabled) != 0);
end
always @(posedge clk) lines_want <= (irq_raw & want_m) != 0;
assign pick_want = want_p || lines_want;
always @(posedge clk) pe_any <= (pending & enabled) != 0;
// (irq_line & enabled) != 0, from irq_raw a clock early: irq_line is irq_raw
// registered, and enabled does not change in the clock before a decision
reg         lines_en, lines_want;
always @(posedge clk) lines_en <= (irq_raw & enabled) != 0;
assign ready_fast = pe_any || lines_en;

// ---- the priority RAM ------------------------------------------------------

reg [7:0] pr0 [0:63];
reg [7:0] pr1 [0:63];
reg [7:0] pr2 [0:63];
reg [7:0] pr3 [0:63];
reg [7:0] q0, q1, q2, q3;
reg [5:0] p_ra;
reg       p_we0, p_we1, p_we2, p_we3;
reg [5:0] p_wa0, p_wa1, p_wa2, p_wa3;
reg [7:0] p_wd0, p_wd1, p_wd2, p_wd3;

always @(posedge clk) begin
	if (p_we0) pr0[p_wa0] <= p_wd0;
	if (p_we1) pr1[p_wa1] <= p_wd1;
	if (p_we2) pr2[p_wa2] <= p_wd2;
	if (p_we3) pr3[p_wa3] <= p_wd3;
	q0 <= pr0[p_ra];
	q1 <= pr1[p_ra];
	q2 <= pr2[p_ra];
	q3 <= pr3[p_ra];
end

reg  [6:0] clr_n = 7'd64;
assign busy = clr || clr_n != 0;

// ---- pick_irq: a scan of words 0..15 ---------------------------------------

reg        scan;
reg  [4:0] sj;          // word presented
reg        pv;          // q holds word pj
reg  [3:0] pj;
reg  [8:0] best_p, cur_p;
reg  [5:0] best_i;
reg        best_v;

// One word of the loop over i in pick_irq, as a tree (the first of the
// lowest wins, as the loop's strict < gives), and the lowest priority of an
// active one (cur), registered (w_*); the next clock folds it into best/cur
// (strict <: an earlier word wins a tie); the clock after that decides.
reg  [7:0] gq0, gq1, gq2, gq3;
reg        e0, e1, e2, e3, s01, s23, s4, v01, v23;
reg  [7:0] p01, p23;
reg  [8:0] c01, c23, ca0, ca1, ca2, ca3;
reg  [8:0] w_p, w_c;
reg  [5:0] w_i;
reg        wd_v, w_ok, w_last, fin;
reg        x_ok, x_last, x_v01, x_v23, x_s01, x_s23;
reg  [3:0] x_j;
reg  [7:0] x_p01, x_p23;
reg  [8:0] x_c01, x_c23;
wire       x_s4 = x_v01 && (!x_v23 || x_p01 <= x_p23);
always @* begin
	gq0 = q0; gq1 = q1; gq2 = q2; gq3 = q3;
	e0 = ready_r[{pj, 2'd0}] && !(ithresdr != 0 && gq0 >= ithresdr[7:0]);
	e1 = ready_r[{pj, 2'd1}] && !(ithresdr != 0 && gq1 >= ithresdr[7:0]);
	e2 = ready_r[{pj, 2'd2}] && !(ithresdr != 0 && gq2 >= ithresdr[7:0]);
	e3 = ready_r[{pj, 2'd3}] && !(ithresdr != 0 && gq3 >= ithresdr[7:0]);
	s01 = e0 && (!e1 || gq0 <= gq1); p01 = s01 ? gq0 : gq1; v01 = e0 || e1;
	s23 = e2 && (!e3 || gq2 <= gq3); p23 = s23 ? gq2 : gq3; v23 = e2 || e3;
	s4 = v01 && (!v23 || p01 <= p23);
	ca0 = active[{pj, 2'd0}] ? {1'b0, gq0} : 9'd256;
	ca1 = active[{pj, 2'd1}] ? {1'b0, gq1} : 9'd256;
	ca2 = active[{pj, 2'd2}] ? {1'b0, gq2} : 9'd256;
	ca3 = active[{pj, 2'd3}] ? {1'b0, gq3} : 9'd256;
	c01 = (ca1 < ca0) ? ca1 : ca0;
	c23 = (ca3 < ca2) ? ca3 : ca2;
end

// ---- reads: pfic_read of the word, then the shift and mask of bus_read_slow --

reg  [11:0] r_off;
reg   [2:0] r_size;
wire [11:0] r_word = {r_off[11:2], 2'b00};
reg  [31:0] rv, rv2;
always @* begin
	rv = 0;
	if (r_word < 12'h020)
		rv = (r_word == 12'h000) ? enabled[31:0] : (r_word == 12'h004) ? enabled[63:32] : 32'd0;
	else if (r_word < 12'h040)
		rv = (r_word == 12'h020) ? ready_lp[31:0] : (r_word == 12'h024) ? ready_lp[63:32] : 32'd0;
	else if (r_word == 12'h040) rv = ithresdr;
	else if (r_word == 12'h048) rv = 0;
	else if (r_word == 12'h04c) rv = {22'd0, ready_any, depth != 0, 4'd0, depth};
	else if (r_word == 12'h050) rv = vtf_id;
	else if (r_word == 12'h060) rv = vtf_addr0;
	else if (r_word == 12'h064) rv = vtf_addr1;
	else if (r_word == 12'h068) rv = vtf_addr2;
	else if (r_word == 12'h06c) rv = vtf_addr3;
	else if (r_word == 12'h100) rv = enabled[31:0];
	else if (r_word == 12'h104) rv = enabled[63:32];
	else if (r_word == 12'h300) rv = active[31:0];
	else if (r_word == 12'h304) rv = active[63:32];
	else if (r_word >= 12'h400 && r_word < 12'h500) rv = {q3, q2, q1, q0};
	else if (r_word == 12'hd10) rv = sctlr;
end
// the word registered, then shifted and masked (the CPU waits a clock more)
reg [31:0] rw;
always @(posedge clk) rw <= rv;
always @* begin
	rv2 = rw >> {r_off[1:0], 3'b000};
	if (r_size == 1) rv2 = rv2 & 32'hff;
	else if (r_size == 2) rv2 = rv2 & 32'hffff;
end
always @* rdata = rv2;

// ---- writes ----------------------------------------------------------------

wire [11:0] w_word = {off[11:2], 2'b00};
wire        w_prio = w_word >= 12'h400 && w_word < 12'h500;
wire [31:0] w_v = wdata << {off[1:0], 3'b000};           // the lane of a narrow write
wire [63:0] w_bits = (w_word[6:2] == 0) ? {32'd0, w_v} : (w_word[6:2] == 1) ? {w_v, 32'd0} : 64'd0;
wire  [8:0] w_n = {1'b0, off[7:0]};                      // off - 0x400 for a priority write

// the priority bytes a write of 'size' bytes from byte w_n covers: lane b
// gets byte i = (b - w_n) & 3 when i < size, at word (w_n + i) >> 2
wire [1:0] i0 = 2'd0 - w_n[1:0], i1 = 2'd1 - w_n[1:0], i2 = 2'd2 - w_n[1:0], i3 = 2'd3 - w_n[1:0];
wire [8:0] n0 = w_n + {7'd0, i0}, n1 = w_n + {7'd0, i1}, n2 = w_n + {7'd0, i2}, n3 = w_n + {7'd0, i3};
wire       wp = wr_req && w_prio && !busy;
wire [31:0] wd0 = wdata >> {i0, 3'b000}, wd1 = wdata >> {i1, 3'b000};
wire [31:0] wd2 = wdata >> {i2, 3'b000}, wd3 = wdata >> {i3, 3'b000};

always @* begin
	p_ra = scan ? {1'b0, sj} : off[7:2];
	p_we0 = wp && {1'b0, i0} < size && !n0[8]; p_wa0 = n0[7:2]; p_wd0 = wd0[7:0];
	p_we1 = wp && {1'b0, i1} < size && !n1[8]; p_wa1 = n1[7:2]; p_wd1 = wd1[7:0];
	p_we2 = wp && {1'b0, i2} < size && !n2[8]; p_wa2 = n2[7:2]; p_wd2 = wd2[7:0];
	p_we3 = wp && {1'b0, i3} < size && !n3[8]; p_wa3 = n3[7:2]; p_wd3 = wd3[7:0];
	if (clr_n != 0) begin
		p_we0 = 1; p_we1 = 1; p_we2 = 1; p_we3 = 1;
		p_wa0 = clr_n[5:0] - 1'd1; p_wa1 = p_wa0; p_wa2 = p_wa0; p_wa3 = p_wa0;
		p_wd0 = 0; p_wd1 = 0; p_wd2 = 0; p_wd3 = 0;
	end
end

always @(posedge clk) begin
	reset_req <= 0;
	pick_done <= 0;
	if (rd_req) begin
		r_off <= off;
		r_size <= size;
	end
	if (clr) begin
		enabled <= 0;
		pending <= 0;
		ithresdr <= 0;
		sctlr <= 0;
		vtf_addr0 <= 0; vtf_addr1 <= 0; vtf_addr2 <= 0; vtf_addr3 <= 0;
		vtf_id <= 0;
		scan <= 0;
		pv <= 0;
		noirq <= 0;
		clr_n <= 7'd64;
	end else begin
		if (clr_n != 0)
			clr_n <= clr_n - 1'd1;
		if (wr_req && !w_prio) begin
			case (w_word & ~12'h07f)
			12'h100: enabled <= enabled | w_bits;
			12'h180: enabled <= enabled & ~w_bits;
			12'h200: pending <= pending | w_bits;
			12'h280: pending <= pending & ~w_bits;
			default:
				if (w_word == 12'h040) ithresdr <= {24'd0, w_v[7:0]};
				else if (w_word == 12'h048) begin
					// KEY3 with SYSRESET: NVIC_SystemReset()
					if (w_v[31:16] == 16'hBEEF && w_v[7]) reset_req <= 1;
				end else if (w_word == 12'h050) vtf_id <= w_v;
				else if (w_word == 12'h060) vtf_addr0 <= w_v;
				else if (w_word == 12'h064) vtf_addr1 <= w_v;
				else if (w_word == 12'h068) vtf_addr2 <= w_v;
				else if (w_word == 12'h06c) vtf_addr3 <= w_v;
				else if (w_word == 12'hd10) begin
					if (w_v[31]) reset_req <= 1;
					sctlr <= {1'b0, w_v[30:0]};
				end
			endcase
		end
		// the bytes of a priority write past prio[255]: ITHRESDR's, in the C
		if (wp && {1'b0, i0} < size && n0[8]) ithresdr[{n0[1:0], 3'b000} +: 8] <= wd0[7:0];
		if (wp && {1'b0, i1} < size && n1[8]) ithresdr[{n1[1:0], 3'b000} +: 8] <= wd1[7:0];
		if (wp && {1'b0, i2} < size && n2[8]) ithresdr[{n2[1:0], 3'b000} +: 8] <= wd2[7:0];
		if (wp && {1'b0, i3} < size && n3[8]) ithresdr[{n3[1:0], 3'b000} +: 8] <= wd3[7:0];
		if (sw_pend)
			pending[14] <= 1'b1;
		if (take)
			pending[take_irq] <= 1'b0;

		// the scan
		w_ok <= 0;
		x_ok <= 0;
		fin <= 0;
		if (wr_req || take || sw_pend || depth != depth_r || nest != nest_r) noirq <= 0;
		if (pick_start) begin
			noirq <= 0;
			scan <= 1;
			sj <= 0;
			pv <= 0;
			best_p <= 9'd256;
			best_v <= 0;
			best_i <= 0;
			cur_p <= 9'd256;
		end else begin
			if (scan) begin
				pj <= sj[3:0];
				pv <= 1;
				if (sj == 15) scan <= 0;
				sj <= sj + 1'd1;
			end else
				pv <= 0;
			x_ok <= pv;
			if (pv) begin
				x_last <= !scan;
				x_j <= pj;
				x_v01 <= v01; x_v23 <= v23; x_s01 <= s01; x_s23 <= s23;
				x_p01 <= p01; x_p23 <= p23; x_c01 <= c01; x_c23 <= c23;
			end
			if (x_ok) begin
				w_ok <= 1;
				w_last <= x_last;
				wd_v <= x_v01 || x_v23;
				w_p <= {1'b0, x_s4 ? x_p01 : x_p23};
				w_i <= {x_j, x_s4 ? {1'b0, !x_s01} : {1'b1, !x_s23}};
				w_c <= (x_c23 < x_c01) ? x_c23 : x_c01;
			end
			if (w_ok) begin
				if (wd_v && w_p < best_p) begin
					best_p <= w_p; best_i <= w_i; best_v <= 1;
				end
				if (w_c < cur_p) cur_p <= w_c;
				fin <= w_last;
			end
			if (fin) begin
				// the end of pick_irq. Inside a handler only a higher preemption
				// level (the top priority bit) gets in, and only with nesting on
				pick_done <= 1;
				pick_irq <= best_i;
				pick_valid <= best_v && (!in_trap || (nest && best_p[8:7] < cur_p[8:7]));
				noirq <= !(best_v && (!in_trap || (nest && best_p[8:7] < cur_p[8:7])));
				ready_scan <= ready_r;
			end
		end
	end
end

endmodule
