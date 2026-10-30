// One of the timers TIM1 (advanced, 0x4001_2C00), TIM2 (0x4000_0000) and
// TIM3 (0x4000_0400). The C treats the three alike (the 'advanced' flag is
// set for TIM1 and read nowhere), so ADVANCED here only names the instance.
//
// Translates tim_read, tim_write, tim_events, tim_count, tim_plan and
// tim_rebase in emu/src/bus.c. The C keeps a base count and its cycle and
// works the counter out when asked; here it runs. Every write rebases in
// the C, which drops the part of a prescaler tick already counted: so does
// a write here (pcnt goes to 0), whatever register it is to.
//
//   counter     with CEN, one tick every PSC+1 board cycles; after ATRLR
//               it goes to 0 (the C's (cnt_base + ticks) % (ATRLR + 1))
//   update      the tick that wraps it to 0, only while CEN and UIE
//               (tim_plan plans nothing without UIE: in the C the flag is
//               not set by a wrap without UIE, so it is not here): UIF and
//               the interrupt line go high at the next instruction boundary
//               ('boundary': the C runs tim_events from bus_events at the
//               top of chg_run's loop, so an access whose instruction began
//               before the update does not see it). A write to the timer
//               before that replans (tim_plan) and the update is never
//               taken: 'pend' is cleared
//   irq         the level tim_write and tim_events give pfic_set_line
//               (TIM1_UP 35, TIM2_UP 38, TIM3 54): set by an update, after a
//               write it is UIF & UIE
//   UG          while running the counter goes to 0; UIF is set unless URS
//               (tim_write's comment: measured on a CHGame)
//   CNT, ATRLR  after any write the count is taken modulo ATRLR+1 when it
//               is not below it (a 16-step divider; the ack waits for it)
//   INTFR       bits are cleared by writing 0; only UIF is ever set
//
// oc2 is TIM1 channel 2 as buzzer_update in bus.c works it out for PB10
// (the GPIO side picks it when PB10 is an alternate-function output and
// AFIO has the partial remap): 0 without CC2E and MOE; otherwise the
// output-compare level of OC2M against the live count, inverted by CC2P.
// buzzer_update freezes it as a steady level or as a PWM wave with a period
// and an origin taken at the time of the call; both come out the same as
// the live compare here, since every TIM1 write calls it again.

module chg_tim
#(
	parameter ADVANCED = 0
)
(
	input             clk,
	input             reset,        // chg_reset: bus_reset
	input             ce,
	input             boundary,     // an instruction boundary: chg_run's bus_events

	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,
	output reg        ack,

	output reg        irq,          // the update interrupt line
	output            oc2,          // channel 2 output level (TIM1: the buzzer)
	output            ev            // an update the next boundary takes (it may change irq)
);

reg [15:0] ctlr1, ctlr2, smcfgr, dmaintenr, chctlr1, chctlr2, ccer;
reg [15:0] psc, atrlr, rptcr, ccr1, ccr2, ccr3, ccr4, bdtr, dmacfgr, spec;
reg        uif;
reg [15:0] cnt;
reg [15:0] pcnt;                // board cycles into the present tick
reg        pend;                // an update tim_events has not taken yet
wire       take = pend & boundary;  // a boundary in the same clock as an access comes first
assign ev = pend;
wire       uif_b = uif | take;

wire [4:0] sh = {addr[1:0], 3'b000};
wire [31:0] wv = wdata << sh;
wire [15:0] v = wv[15:0];       // tim_write: v &= 0xffff
wire [7:0] off = {addr[7:2], 2'b00};
wire       hi = |addr[9:8];     // offsets 0x100..0x3ff: nothing there

function [31:0] lane_r(input [31:0] x0, input [4:0] s, input [2:0] sz);
	reg [31:0] x;
	begin
		x = x0 >> s;
		if (sz == 3'd1) x = x & 32'h000000ff;
		else if (sz == 3'd2) x = x & 32'h0000ffff;
		lane_r = x;
	end
endfunction

reg [15:0] rv;
always @* begin
	rv = 16'd0;
	if (!hi)
		case (off)
		8'h00: rv = ctlr1;
		8'h04: rv = ctlr2;
		8'h08: rv = smcfgr;
		8'h0c: rv = dmaintenr;
		8'h10: rv = {15'd0, uif_b};
		8'h18: rv = chctlr1;
		8'h1c: rv = chctlr2;
		8'h20: rv = ccer;
		8'h24: rv = cnt;
		8'h28: rv = psc;
		8'h2c: rv = atrlr;
		8'h30: rv = rptcr;
		8'h34: rv = ccr1;
		8'h38: rv = ccr2;
		8'h3c: rv = ccr3;
		8'h40: rv = ccr4;
		8'h44: rv = bdtr;
		8'h48: rv = dmacfgr;
		8'h50: rv = spec;
		default: rv = 16'd0;
		endcase
end

// the write (tim_write's switch), as next values
reg [15:0] n_dmaintenr, n_atrlr, n_cnt;
reg        n_uif;
always @* begin
	n_dmaintenr = dmaintenr; n_atrlr = atrlr; n_cnt = cnt; n_uif = uif_b;
	if (!hi)
		case (off)
		8'h0c: n_dmaintenr = v;
		8'h10: n_uif = uif_b & v[0];
		8'h14: if (v[0]) begin
			if (ctlr1[0]) n_cnt = 16'd0;
			if (!ctlr1[2]) n_uif = 1'b1;
		end
		8'h24: n_cnt = v;
		8'h2c: n_atrlr = v;
		default: ;
		endcase
end

// CNT modulo ATRLR+1, a bit a clock
reg        busy;
reg  [4:0] dstep;
reg [15:0] dnum;
reg [16:0] drem;
reg [16:0] dden;
wire [16:0] dtry = {drem[15:0], dnum[15]};

always @(posedge clk) begin
	ack <= 0;
	if (reset) begin
		ctlr1 <= 0; ctlr2 <= 0; smcfgr <= 0; dmaintenr <= 0; chctlr1 <= 0; chctlr2 <= 0;
		ccer <= 0; psc <= 0; atrlr <= 16'hffff; rptcr <= 0; ccr1 <= 0; ccr2 <= 0;
		ccr3 <= 0; ccr4 <= 0; bdtr <= 0; dmacfgr <= 0; spec <= 0;
		uif <= 0; cnt <= 0; pcnt <= 0; irq <= 0; busy <= 0; pend <= 0;
	end else if (busy) begin
		drem <= (dtry >= dden) ? dtry - dden : dtry;
		dnum <= {dnum[14:0], 1'b0};
		dstep <= dstep - 5'd1;
		if (dstep == 5'd0) begin
			busy <= 0;
			ack <= 1;
			cnt <= drem[15:0];
		end
	end else if (req) begin
		if (we) begin
			pcnt <= 0;                          // tim_rebase
			pend <= 0;                          // tim_plan
			if (!hi)
				case (off)
				8'h00: ctlr1 <= v;
				8'h04: ctlr2 <= v;
				8'h08: smcfgr <= v;
				8'h0c: dmaintenr <= v;
				8'h18: chctlr1 <= v;
				8'h1c: chctlr2 <= v;
				8'h20: ccer <= v;
				8'h28: psc <= v;
				8'h2c: atrlr <= v;
				8'h30: rptcr <= v;
				8'h34: ccr1 <= v;
				8'h38: ccr2 <= v;
				8'h3c: ccr3 <= v;
				8'h40: ccr4 <= v;
				8'h44: bdtr <= v;
				8'h48: dmacfgr <= v;
				8'h50: spec <= v;
				default: ;
				endcase
			uif <= n_uif;
			irq <= n_uif & n_dmaintenr[0];
			cnt <= n_cnt;
			// cnt_base %= period when it is not below it
			if ({1'b0, n_cnt} >= {1'b0, n_atrlr} + 17'd1) begin
				busy <= 1;
				dstep <= 5'd16;
				dnum <= n_cnt;
				drem <= 0;
				dden <= {1'b0, n_atrlr} + 17'd1;
			end else
				ack <= 1;
		end else begin
			rdata <= lane_r({16'd0, rv}, sh, size);
			ack <= 1;
			if (take) begin uif <= 1; irq <= 1; pend <= 0; end
		end
	end else begin
		if (take) begin                     // tim_events
			uif <= 1;
			irq <= 1;
			pend <= 0;
		end
		if (ce && ctlr1[0]) begin
			if (pcnt == psc) begin
				pcnt <= 0;
				if (cnt == atrlr) begin
					cnt <= 0;
					if (dmaintenr[0]) pend <= 1;
				end else
					cnt <= cnt + 16'd1;
			end else
				pcnt <= pcnt + 16'd1;
		end
	end
end

// buzzer_update's TIM1 branch
wire [2:0] oc2m = chctlr1[14:12];
reg        oc2_active;
always @* begin
	case (oc2m)
	3'd5:    oc2_active = 1'b1;
	3'd6:    oc2_active = cnt < ccr2;
	3'd7:    oc2_active = !(cnt < ccr2);
	default: oc2_active = 1'b0;
	endcase
end
assign oc2 = (ccer[4] & bdtr[15]) ? (oc2_active ^ ccer[5]) : 1'b0;

endmodule
