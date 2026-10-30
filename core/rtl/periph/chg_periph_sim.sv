// FOR THE TESTBENCHES ONLY (sim/tb_periph*.cpp), not part of the core: the
// peripherals of this directory (all but SPI, DMA and SD) joined with the
// address decode of chg_soc, so one Verilator model can be run
// beside emu/src. The decode is what bus_read_slow / bus_write_slow do:
//
//   0xE000_F000..F0FF        chg_systick
//   0x4001_0000, 0800, 0C00, 1000 (1 KB each)   chg_gpio (AFIO, A, B, C)
//   0x4002_1000              chg_rcc
//   0x4001_2C00 / 0x4000_0000 / 0x4000_0400     chg_tim TIM1 / TIM2 / TIM3
//   0x4002_2000, and stores to 0x0000_0000 / 0x0800_0000 + 64 KB  chg_flashctl
//   0x4001_2400              chg_adc
//   anything else            chg_misc
// (the testbench never sends SPI, DMA, PFIC, flash reads or SRAM here).

module chg_periph_sim
(
	input             clk,
	input             init,
	input             por,          // power-on reset
	input             sreset,       // software reset
	input             ce,
	input             boundary,

	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output     [31:0] rdata,
	output            ack,
	output     [16:0] stall,

	input       [7:0] buttons,
	input             spi_nss,

	output            pa4, pb0, pb9, pb10, pb11, pb12,
	output            irq_systick, sw_pend, irq_tim1, irq_tim2, irq_tim3,
	output            irq_ev,       // the next boundary may change one of these lines

	output            mreq,
	output            mwe,
	output     [31:0] maddr,
	output      [2:0] msize,
	output     [31:0] mwdata,
	input      [31:0] mrdata,
	input             mack,
	output            fl_page_stb,
	output      [7:0] fl_page,
	output            serial_stb,
	output      [7:0] serial_byte
);

wire reset = por | sreset;
wire [21:0] w = addr[31:10];

wire s_stk  = addr[31:8] == 24'hE000F0;
wire s_gpio = w == 22'h100040 || w == 22'h100042 || w == 22'h100043 || w == 22'h100044;
wire s_rcc  = w == 22'h100084;
wire s_t1   = w == 22'h10004B;
wire s_t2   = w == 22'h100000;
wire s_t3   = w == 22'h100001;
wire s_fl   = w == 22'h100088 || (we && (addr < 32'h00010000 || addr[31:16] == 16'h0800));
wire s_adc  = w == 22'h100049;
wire s_misc = !(s_stk | s_gpio | s_rcc | s_t1 | s_t2 | s_t3 | s_fl | s_adc);

wire [31:0] r_stk, r_gpio, r_rcc, r_t1, r_t2, r_t3, r_fl, r_adc, r_misc;
wire        a_stk, a_gpio, a_rcc, a_t1, a_t2, a_t3, a_fl, a_adc, a_misc;
wire [16:0] st_fl, st_adc;
wire        oc2_t1, oc2_t2, oc2_t3;
wire        ev_stk, ev_t1, ev_t2, ev_t3;
assign irq_ev = ev_stk | ev_t1 | ev_t2 | ev_t3;
wire [31:0] pcfr1;


// The request registered once more, a copy for each module next to it (96
// MHz: rq in chg_soc drives them all). Board time is held during an access.
(* preserve *) reg qstk_req, qstk_we;
(* preserve *) reg [31:0] qstk_addr, qstk_wdata;
(* preserve *) reg [2:0] qstk_size;
(* preserve *) reg qgpio_req, qgpio_we;
(* preserve *) reg [31:0] qgpio_addr, qgpio_wdata;
(* preserve *) reg [2:0] qgpio_size;
(* preserve *) reg qrcc_req, qrcc_we;
(* preserve *) reg [31:0] qrcc_addr, qrcc_wdata;
(* preserve *) reg [2:0] qrcc_size;
(* preserve *) reg qt1_req, qt1_we;
(* preserve *) reg [31:0] qt1_addr, qt1_wdata;
(* preserve *) reg [2:0] qt1_size;
(* preserve *) reg qt2_req, qt2_we;
(* preserve *) reg [31:0] qt2_addr, qt2_wdata;
(* preserve *) reg [2:0] qt2_size;
(* preserve *) reg qt3_req, qt3_we;
(* preserve *) reg [31:0] qt3_addr, qt3_wdata;
(* preserve *) reg [2:0] qt3_size;
(* preserve *) reg qfl_req, qfl_we;
(* preserve *) reg [31:0] qfl_addr, qfl_wdata;
(* preserve *) reg [2:0] qfl_size;
(* preserve *) reg qadc_req, qadc_we;
(* preserve *) reg [31:0] qadc_addr, qadc_wdata;
(* preserve *) reg [2:0] qadc_size;
(* preserve *) reg qmisc_req, qmisc_we;
(* preserve *) reg [31:0] qmisc_addr, qmisc_wdata;
(* preserve *) reg [2:0] qmisc_size;
always @(posedge clk) begin
	qstk_req <= req & s_stk; qstk_we <= we; qstk_addr <= addr; qstk_wdata <= wdata; qstk_size <= size;
	qgpio_req <= req & s_gpio; qgpio_we <= we; qgpio_addr <= addr; qgpio_wdata <= wdata; qgpio_size <= size;
	qrcc_req <= req & s_rcc; qrcc_we <= we; qrcc_addr <= addr; qrcc_wdata <= wdata; qrcc_size <= size;
	qt1_req <= req & s_t1; qt1_we <= we; qt1_addr <= addr; qt1_wdata <= wdata; qt1_size <= size;
	qt2_req <= req & s_t2; qt2_we <= we; qt2_addr <= addr; qt2_wdata <= wdata; qt2_size <= size;
	qt3_req <= req & s_t3; qt3_we <= we; qt3_addr <= addr; qt3_wdata <= wdata; qt3_size <= size;
	qfl_req <= req & s_fl; qfl_we <= we; qfl_addr <= addr; qfl_wdata <= wdata; qfl_size <= size;
	qadc_req <= req & s_adc; qadc_we <= we; qadc_addr <= addr; qadc_wdata <= wdata; qadc_size <= size;
	qmisc_req <= req & s_misc; qmisc_we <= we; qmisc_addr <= addr; qmisc_wdata <= wdata; qmisc_size <= size;
end

// the selected module answers; one access at a time
reg [8:0] sel;
always @(posedge clk) if (req) sel <= {s_misc, s_adc, s_fl, s_t3, s_t2, s_t1, s_rcc, s_gpio, s_stk};

assign ack   = a_stk | a_gpio | a_rcc | a_t1 | a_t2 | a_t3 | a_fl | a_adc | a_misc;
assign rdata = sel[0] ? r_stk : sel[1] ? r_gpio : sel[2] ? r_rcc : sel[3] ? r_t1 : sel[4] ? r_t2 :
               sel[5] ? r_t3 : sel[6] ? r_fl : sel[7] ? r_adc : r_misc;
assign stall = sel[6] ? st_fl : sel[7] ? st_adc : 17'd0;

chg_systick u_stk (.clk(clk), .reset(reset), .ce(ce), .boundary(boundary), .req(qstk_req), .we(qstk_we), .addr(qstk_addr), .size(qstk_size),
	.wdata(qstk_wdata), .rdata(r_stk), .ack(a_stk), .irq(irq_systick), .sw_pend(sw_pend), .ev(ev_stk));

chg_gpio u_gpio (.clk(clk), .reset(reset), .ce(ce), .req(qgpio_req), .we(qgpio_we), .addr(qgpio_addr), .size(qgpio_size),
	.wdata(qgpio_wdata), .rdata(r_gpio), .ack(a_gpio), .buttons(buttons), .spi_nss(spi_nss), .tim1_oc2(oc2_t1),
	.pa4(pa4), .pb0(pb0), .pb9(pb9), .pb10(pb10), .pb11(pb11), .pb12(pb12), .afio_pcfr1(pcfr1));

chg_rcc u_rcc (.clk(clk), .power_on_reset(por), .soft_reset(sreset), .ce(ce), .req(qrcc_req), .we(qrcc_we),
	.addr(qrcc_addr), .size(qrcc_size), .wdata(qrcc_wdata), .rdata(r_rcc), .ack(a_rcc));

chg_tim #(.ADVANCED(1)) u_t1 (.clk(clk), .reset(reset), .ce(ce), .boundary(boundary), .req(qt1_req), .we(qt1_we), .addr(qt1_addr),
	.size(qt1_size), .wdata(qt1_wdata), .rdata(r_t1), .ack(a_t1), .irq(irq_tim1), .oc2(oc2_t1), .ev(ev_t1));
chg_tim u_t2 (.clk(clk), .reset(reset), .ce(ce), .boundary(boundary), .req(qt2_req), .we(qt2_we), .addr(qt2_addr),
	.size(qt2_size), .wdata(qt2_wdata), .rdata(r_t2), .ack(a_t2), .irq(irq_tim2), .oc2(oc2_t2), .ev(ev_t2));
chg_tim u_t3 (.clk(clk), .reset(reset), .ce(ce), .boundary(boundary), .req(qt3_req), .we(qt3_we), .addr(qt3_addr),
	.size(qt3_size), .wdata(qt3_wdata), .rdata(r_t3), .ack(a_t3), .irq(irq_tim3), .oc2(oc2_t3), .ev(ev_t3));

chg_flashctl u_fl (.clk(clk), .reset(reset), .ce(ce), .req(qfl_req), .we(qfl_we), .addr(qfl_addr), .size(qfl_size),
	.wdata(qfl_wdata), .rdata(r_fl), .ack(a_fl), .stall(st_fl), .mreq(mreq), .mwe(mwe), .maddr(maddr),
	.msize(msize), .mwdata(mwdata), .mrdata(mrdata), .mack(mack), .fl_page_stb(fl_page_stb), .fl_page(fl_page));

chg_adc u_adc (.clk(clk), .init(init), .reset(reset), .ce(ce), .req(qadc_req), .we(qadc_we), .addr(qadc_addr),
	.size(qadc_size), .wdata(qadc_wdata), .rdata(r_adc), .ack(a_adc), .stall(st_adc));

chg_misc u_misc (.clk(clk), .reset(reset), .ce(ce), .req(qmisc_req), .we(qmisc_we), .addr(qmisc_addr), .size(qmisc_size),
	.wdata(qmisc_wdata), .rdata(r_misc), .ack(a_misc), .serial_stb(serial_stb), .serial_byte(serial_byte));

endmodule
