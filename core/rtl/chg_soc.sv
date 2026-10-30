// The CH32X035 and what the CHGame wires to it (CONTRACT.md): the CPU, the
// flash and SRAM, the peripherals, SPI/DMA, the microSD card and the ST7735,
// joined. Board time is ce: one 48 MHz cycle of the board.
//
// Memory map (machine.h):
//   0x0000_0000  flash, 64 KB (alias 0x0800_0000): bootloader, game from 0x3000,
//                metadata page 0xF700
//   0x2000_0000  SRAM, 20 KB
//   0x4000_0000  peripherals; SPI1 0x4001_3000 and DMA1 0x4002_0000 go to
//                chg_spi_tb_top, everything else to chg_periph_sim (its own decode)
//   0xE000_E000  PFIC (inside the CPU), 0xE000_F000 SysTick
//
// Both memories are dual port: port A is the CPU's, port B is shared by the
// loader (while the board is held in reset), the flash controller and the DMA.

module chg_soc
(
	input             clk,
	input             reset,        // power-on reset of the whole board (the flash keeps its contents)
	input             run,          // board time may advance (the throttle in CHGame.sv)
	output            ce,           // a board cycle happened on this clock
	input             fast_card,    // the loading detector may act (1 in the core; the lockstep testbenches leave it 0: cycle exact)
	input             fast_always,  // Fast card loading: Always on (fast whenever the card is selected)
	output            card_sel,     // fast loading is on and the card is selected: the board may run unthrottled
	input             hold_mode,    // the card: 1 holds board time for late blocks (simulation), 0 answers busy (MiSTer)

	// writing the flash while the board is held in reset: the loader, the save's restore
	input             fl_we,
	input      [15:0] fl_addr,
	input       [7:0] fl_data,

	// the game's own writes to its flash, as port B carries them, for the save
	output            gfw_we,
	output     [15:0] gfw_addr,
	output      [2:0] gfw_size,
	output     [31:0] gfw_data,

	input       [7:0] buttons,      // CHG_BTN_* order: up down left right A B SELECT START; 1 = held

	// the glass, for the video side (st7735.sv)
	input       [6:0] vx,
	input       [6:0] vy,
	input             vline,
	output     [23:0] vrgb,

	output            buzzer,       // PB10
	output            led,          // PB9

	// the microSD card (virtual disk 1)
	input             sd_mounted,
	input      [63:0] sd_size,
	output     [31:0] sd_lba,
	output            sd_rd,
	output            sd_wr,
	input             sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din,
	input             sd_buff_wr,

	// for the testbench
	output            dbg_boundary,
	output            retire,
	output     [31:0] retire_pc,
	output     [63:0] retire_instret,
	output     [63:0] retire_cycles,
	output   [1023:0] dbg_x,
	output            faulted
);

// ---------------------------------------------------------------------------
// the CPU
// ---------------------------------------------------------------------------

// Power-on: as chg_reset(m, true), SRAM comes up holding junk, the C's
// xorshift pattern, written through port A after each power-on reset (a game
// load, the OSD's reset) while the board stays in reset (5,120 clocks)
reg         rst_d, filling;
reg  [12:0] fill_i;
reg  [31:0] fill_r;
wire [31:0] fill_n0 = fill_r ^ (fill_r << 13);
wire [31:0] fill_n1 = fill_n0 ^ (fill_n0 >> 17);
wire [31:0] fill_n  = fill_n1 ^ (fill_n1 << 5);
always @(posedge clk) begin
	rst_d <= reset;
	if (reset && !rst_d) begin filling <= 1; fill_i <= 0; fill_r <= 32'h9e3779b9; end
	else if (filling) begin
		fill_r <= fill_n;
		fill_i <= fill_i + 1'd1;
		if (fill_i == 13'd5119) filling <= 0;
	end
end
wire rst = reset || filling;

// ---------------------------------------------------------------------------

wire        hold, busy, busy_q, boundary, sys_reset, mem_b_wr, mem_b_ram;
wire        p_irq_ev, s_irq_ev, s_work;
wire [13:0] fa_addr;
reg  [31:0] fa_q;
wire [12:0] ra_addr;
wire  [3:0] ra_we;
wire [31:0] ra_wdata;
reg  [31:0] ra_q;
wire        bus_req, bus_we;
wire [31:0] bus_addr, bus_wdata;
wire  [2:0] bus_size;
wire [31:0] bus_rdata;
wire        bus_ack;
wire [16:0] stall, p_stall;
wire [63:0] irq_line;
wire        sw_pend;

// Board time and the CPU look at busy_q, busy less its combinational part
// ((boundary or gpio_wr) and work due, or SPI/DMA's own request). That part
// is only ever high in a clock where hold is high anyway (the boundary
// clock; a bus access waiting for its answer), and the clock after it the
// registered part has taken over (SPI/DMA's state machine is running).
assign ce = run && !rst && !hold && !busy_q;
assign dbg_boundary = boundary;

chg_cpu cpu
(
	.clk(clk), .reset(rst), .entry(32'd0),
	.hold(hold), .ce(ce), .busy(busy_q), .irq_ev(p_irq_ev | s_irq_ev), .swork(s_work),
	.fl_addr(fa_addr), .fl_rdata(fa_q),
	.ram_addr(ra_addr), .ram_we(ra_we), .ram_wdata(ra_wdata), .ram_rdata(ra_q),
	.mem_b_wr(mem_b_wr), .mem_b_ram(mem_b_ram),
	.bus_req(bus_req), .bus_we(bus_we), .bus_addr(bus_addr), .bus_size(bus_size), .bus_wdata(bus_wdata),
	.bus_rdata(bus_rdata_q), .bus_ack(bus_ack_q), .stall(stall_q),
	.irq_line(irq_line), .sw_pend(sw_pend),
	.sys_reset(sys_reset), .boundary(boundary),
	.retire(retire), .retire_pc(retire_pc), .retire_instret(retire_instret), .retire_cycles(retire_cycles),
	.cycles(), .instret(),
	.faulted(faulted), .fault_cause(), .fault_pc(), .fault_tval(),
	.dbg_x(dbg_x), .dbg_pc(), .dbg_mstatus(), .dbg_mepc(), .dbg_mcause(), .dbg_depth(), .dbg_wfi(),
	.dbg_fw(), .dbg_fd(), .dbg_div_rem(), .dbg_div_rd(), .dbg_frac(), .dbg_pair(), .dbg_recent(),
	.dbg_op(), .dbg_rd(), .dbg_rs2(), .dbg_state(), .dbg_ea()
);

// ---------------------------------------------------------------------------
// the peripheral bus
// ---------------------------------------------------------------------------

// The request is registered once on its way to the peripherals (96 MHz).
// Board time is frozen during an access, so this costs a fabric clock only.
reg         rq;
reg         rq_we;
reg  [31:0] rq_addr, rq_wdata;
reg   [2:0] rq_size;
reg         sel_sd, rq_gpio, rq_gchg;
// the peripherals (chg_periph_sim) register the request themselves, next to each
// module: they take it from the CPU's own request, one stage earlier than SPI/DMA
wire        sel_sd_c = bus_addr[31:10] == 22'(32'h4001_3000 >> 10) || bus_addr[31:10] == 22'(32'h4002_0000 >> 10);
always @(posedge clk) begin
	rq <= bus_req && !rst;
	if (bus_req) begin
		rq_we <= bus_we; rq_addr <= bus_addr; rq_wdata <= bus_wdata; rq_size <= bus_size;
		sel_sd <= bus_addr[31:10] == 22'(32'h4001_3000 >> 10) || bus_addr[31:10] == 22'(32'h4002_0000 >> 10);
		rq_gpio <= bus_we && bus_addr >= 32'h4001_0800 && bus_addr < 32'h4001_1400;
		rq_gchg <= bus_we && ((bus_addr >= 32'h4001_0000 && bus_addr < 32'h4001_1400) ||
		           (bus_addr[31:10] == 22'(32'h4001_3000 >> 10) && (bus_addr[9:2] == 8'd0 || bus_addr[9:2] == 8'd1)));
	end
end

// GPIO port writes run the C's spi_sync first (gpio_write), with the old pin levels
wire gpio_wr = rq && rq_gpio;

wire [31:0] p_rdata, s_rdata;
wire        p_ack, s_ack;
reg         last_sd;
always @(posedge clk) if (rq) last_sd <= sel_sd;
assign bus_rdata = last_sd ? s_rdata : p_rdata;
assign bus_ack   = p_ack | s_ack;
assign stall     = p_stall;
// ... and registered once more on its way back to the CPU (board time is held meanwhile)
reg         bus_ack_q;
reg  [31:0] bus_rdata_q;
reg  [16:0] stall_q;
always @(posedge clk) begin
	bus_ack_q <= bus_ack && !rst;
	bus_rdata_q <= bus_rdata;
	stall_q <= stall;
end

// a bus access still waiting for its ack
reg hold_bus;
always @(posedge clk) hold_bus <= rst ? 1'b0 : bus_req ? 1'b1 : bus_ack_q ? 1'b0 : hold_bus;

// ---------------------------------------------------------------------------
// port B: the flash controller and the DMA (and the loader in reset)
// ---------------------------------------------------------------------------

wire        p_mreq, p_mwe, s_mreq, s_mwe;
wire [31:0] p_maddr, p_mwdata, s_maddr, s_mwdata;
wire  [2:0] p_msize, s_msize;
reg  [31:0] mb_rdata;
reg         p_mack, s_mack;

// one request a clock; a DMA request that meets a flash controller one waits a clock
reg         s_pend;
reg         s_pwe;
reg  [31:0] s_paddr, s_pwdata;
reg   [2:0] s_psize;
always @(posedge clk) begin
	if (p_mreq && s_mreq) begin s_pend <= 1; s_pwe <= s_mwe; s_paddr <= s_maddr; s_psize <= s_msize; s_pwdata <= s_mwdata; end
	else if (!p_mreq) s_pend <= 0;
end

wire        b_from_s = !p_mreq && (s_mreq || s_pend);
wire        b_req    = rst ? fl_we : (p_mreq || s_mreq || s_pend);
wire        b_we     = rst ? 1'b1 : p_mreq ? p_mwe : s_pend ? s_pwe : s_mwe;
wire [31:0] b_addr   = rst ? {16'd0, fl_addr} : p_mreq ? p_maddr : s_pend ? s_paddr : s_maddr;
wire  [2:0] b_size   = rst ? 3'd1 : p_mreq ? p_msize : s_pend ? s_psize : s_msize;
wire [31:0] b_wdata  = rst ? {24'd0, fl_data} : p_mreq ? p_mwdata : s_pend ? s_pwdata : s_mwdata;

wire        b_ram    = b_addr[29];                  // 0x2000_0000
wire  [3:0] b_be     = b_size == 3'd4 ? 4'b1111 : b_size == 3'd2 ? 4'b0011 << b_addr[1:0] : 4'b0001 << b_addr[1:0];
wire [31:0] b_d      = b_wdata << (8 * b_addr[1:0]);
wire        b_wr     = b_req && b_we;

// ... registered once on its way to the memories (96 MHz: the flash is
// 64 KB of block RAM); the answer comes a clock later for it
// (MAX_FANOUT: each register drives all 64 flash blocks; let Quartus copy it)
(* altera_attribute = "-name MAX_FANOUT 16" *) reg bq_req, bq_we, bq_ram;
reg         bq_s, bq_p, bq_rst;
(* altera_attribute = "-name MAX_FANOUT 16" *) reg [31:0] bq_addr, bq_d;
reg  [31:0] bq_wdata;
reg   [2:0] bq_size;
(* altera_attribute = "-name MAX_FANOUT 16" *) reg [3:0] bq_be;
always @(posedge clk) begin
	bq_req <= b_req; bq_we <= b_we; bq_ram <= b_ram; bq_s <= b_from_s; bq_p <= p_mreq; bq_rst <= rst;
	bq_addr <= b_addr; bq_wdata <= b_wdata; bq_d <= b_d; bq_size <= b_size; bq_be <= b_be;
end
wire        bq_wr = bq_req && bq_we;

assign mem_b_wr = bq_wr && !bq_rst;
assign mem_b_ram = bq_ram;
assign gfw_we   = bq_wr && !bq_rst && !bq_ram && bq_p;
assign gfw_addr = bq_addr[15:0];
assign gfw_size = bq_size;
assign gfw_data = bq_wdata;

// ---------------------------------------------------------------------------
// the memories
// ---------------------------------------------------------------------------

reg [3:0][7:0] flash[16384];
`ifdef VERILATOR
initial $readmemh("core/rtl/flash_init.hex", flash);
`else
initial $readmemh("rtl/flash_init.hex", flash);
`endif

// the flash's write enables, made a stage ahead like the SRAM's (bs_we)
(* preserve *) reg  [3:0] bf_we;
always @(posedge clk) bf_we <= (b_wr && !b_ram) ? b_be : 4'd0;
reg  [31:0] fb_q;
wire [31:0] rb_q;
reg   [1:0] b_lane;
reg         b_was_ram, b_was_s, b_was_req, b_was_w;

// flash: port A reads (the CPU), port B reads and writes
always @(posedge clk) fa_q <= flash[fa_addr];
always @(posedge clk) begin
	if (bf_we[0]) flash[bq_addr[15:2]][0] <= bq_d[7:0];
	if (bf_we[1]) flash[bq_addr[15:2]][1] <= bq_d[15:8];
	if (bf_we[2]) flash[bq_addr[15:2]][2] <= bq_d[23:16];
	if (bf_we[3]) flash[bq_addr[15:2]][3] <= bq_d[31:24];
	fb_q <= flash[bq_addr[15:2]];
end

// SRAM: both ports read and write
// (the SRAM's copy of port B's registers is its own: Quartus would otherwise
// share the flash's, packed into the flash blocks)
(* preserve *) reg [12:0] bs_addr;
(* preserve *) reg  [3:0] bs_we;
(* preserve *) reg [31:0] bs_d;
always @(posedge clk) begin
	bs_addr <= 13'((b_addr - 32'h2000_0000) >> 2);
	bs_we <= (b_wr && b_ram) ? b_be : 4'd0;
	bs_d <= b_d;
end
wire [12:0] pa_addr  = filling ? fill_i : ra_addr;
wire  [3:0] pa_we    = filling ? 4'b1111 : ra_we;
wire [31:0] pa_wdata = filling ? fill_n : ra_wdata;
wire [31:0] ra_qw;
always @* ra_q = ra_qw;
chg_dpram #(.AW(13), .BYTES(4), .WORDS(5120)) sram
(
	.clk(clk),
	.a_addr(pa_addr), .a_we(pa_we), .a_d(pa_wdata), .a_q(ra_qw),
	.b_addr(bs_addr), .b_we(bs_we), .b_d(bs_d), .b_q(rb_q)
);

// the answer a clock later, right aligned
always @(posedge clk) begin
	b_lane    <= bq_addr[1:0];
	b_was_ram <= bq_ram;
	b_was_req <= bq_req && !bq_rst;
	b_was_s   <= bq_s;
	b_was_w   <= bq_we;
end
always @* begin
	mb_rdata = (b_was_ram ? rb_q : fb_q) >> (8 * b_lane);
	p_mack   = b_was_req && !b_was_s;
	// a DMA write is acked as it goes to port B (the memory takes it two clocks on, before anything
	// the CPU does after SPI/DMA lets go can read it); a read is answered when its data are there
	s_mack   = (b_was_req && b_was_s && !b_was_w) || (b_req && b_we && b_from_s && !rst);
end

// ---------------------------------------------------------------------------
// the display's reset line, as the C's gpio_changed: rst_level follows PB12
// after every GPIO, AFIO or SPI CTLR write (once SPI/DMA has caught up), and a
// fall resets the controller; a reset of the board sets it high and resets
// the controller (chg_reset)
// ---------------------------------------------------------------------------

wire pb12, pa4, pb0, pb11, pb9;
wire dbg_lcd_stb, dbg_lcd_dc;
wire [7:0] dbg_lcd_byte;
reg  rst_level, lcd_reset, gchg_pend;
wire gchg_wr = rq && rq_gchg;
always @(posedge clk) begin
	lcd_reset <= 0;
	if (rst || sys_reset) begin
		rst_level <= 1; lcd_reset <= 1; gchg_pend <= 0;
	end
	else if (gchg_wr) gchg_pend <= 1;
	else if (gchg_pend && !hold_bus && !busy) begin
		gchg_pend <= 0;
		if (pb12 != rst_level) begin
			rst_level <= pb12;
			if (!pb12) lcd_reset <= 1;
		end
	end
end

// ---------------------------------------------------------------------------
// the peripherals
// ---------------------------------------------------------------------------

wire       spi_nss, irq_systick, irq_t1, irq_t2, irq_t3, irq_spi;
wire [6:0] irq_dma;

chg_periph_sim periph
(
	.clk(clk), .init(rst), .por(rst), .sreset(sys_reset), .ce(ce), .boundary(boundary),
	.req(bus_req && !rst && !sel_sd_c), .we(bus_we), .addr(bus_addr), .size(bus_size), .wdata(bus_wdata),
	.rdata(p_rdata), .ack(p_ack), .stall(p_stall),
	.buttons(buttons), .spi_nss(spi_nss),
	.pa4(pa4), .pb0(pb0), .pb9(pb9), .pb10(buzzer), .pb11(pb11), .pb12(pb12),
	.irq_systick(irq_systick), .sw_pend(sw_pend), .irq_tim1(irq_t1), .irq_tim2(irq_t2), .irq_tim3(irq_t3),
	.irq_ev(p_irq_ev),
	.mreq(p_mreq), .mwe(p_mwe), .maddr(p_maddr), .msize(p_msize), .mwdata(p_mwdata),
	.mrdata(mb_rdata), .mack(p_mack),
	.fl_page_stb(), .fl_page(), .serial_stb(), .serial_byte()
);
assign led = pb9;

// ---------------------------------------------------------------------------
// Fast card loading, automatic: the OSD switch (fast_card) only allows it. It is on while the game
// is loading: the card has delivered at least LM_BMIN blocks and the game has sent fewer than
// LM_FMAX RAMWR (frame) commands to the display, both over the last six windows of 2^21 board
// cycles (about 0.26 s). Gameplay and title screens draw about 30 frames a second; a loading
// screen hardly draws. Counted in board cycles (ce), so it is the same however fast the board runs.
// ---------------------------------------------------------------------------
localparam LM_BMIN = 7'd32, LM_FMAX = 6'd4;
reg  [20:0] lm_t;
reg   [2:0] lm_f;
reg   [6:0] lm_b;
reg  [14:0] lm_fh;                       // five earlier windows' frame counts
reg  [34:0] lm_bh;                       // ... and block counts
reg         lm_loading, sd_rd_q;
wire        lm_ramwr = dbg_lcd_stb && !dbg_lcd_dc && dbg_lcd_byte == 8'h2C;
wire  [5:0] lm_ftot = {3'd0, lm_f} + {3'd0, lm_fh[2:0]} + {3'd0, lm_fh[5:3]} + {3'd0, lm_fh[8:6]} + {3'd0, lm_fh[11:9]} + {3'd0, lm_fh[14:12]};
wire  [9:0] lm_btot = {3'd0, lm_b} + {3'd0, lm_bh[6:0]} + {3'd0, lm_bh[13:7]} + {3'd0, lm_bh[20:14]} + {3'd0, lm_bh[27:21]} + {3'd0, lm_bh[34:28]};
always @(posedge clk) begin
	sd_rd_q <= sd_rd;
	if (rst || !fast_card) begin
		lm_t <= 0; lm_f <= 0; lm_b <= 0; lm_fh <= 0; lm_bh <= 0; lm_loading <= 0;
	end else begin
		if (lm_ramwr && lm_f != 3'd7) lm_f <= lm_f + 1'd1;
		if (sd_rd && !sd_rd_q && lm_b != 7'd127) lm_b <= lm_b + 1'd1;
		if (ce) begin
			lm_t <= lm_t + 1'd1;
			if (lm_t == 21'h1FFFFF) begin
				lm_fh <= {lm_fh[11:0], lm_f};
				lm_bh <= {lm_bh[27:0], lm_b};
				lm_f <= 0; lm_b <= 0;
				lm_loading <= lm_btot >= {3'd0, LM_BMIN} && lm_ftot < LM_FMAX;
			end
		end
	end
end
wire fast_on = fast_card && (lm_loading || fast_always);
assign card_sel = fast_on && !pb11;

chg_spi_tb_top spi
(
	.clk(clk), .por(rst), .reset(rst || sys_reset), .ce(ce), .boundary(boundary), .gpio_wr(gpio_wr), .fast_card(fast_on),
	.req(rq && sel_sd), .we(rq_we), .addr(rq_addr), .size(rq_size), .wdata(rq_wdata),
	.rdata(s_rdata), .ack(s_ack),
	.pa4_af(1'b0), .pa4_gpio(pa4), .pb0(pb0), .pb11(pb11),
	.lcd_rst_level(rst_level), .lcd_reset(lcd_reset),
	.mreq(s_mreq), .mwe(s_mwe), .maddr(s_maddr), .msize(s_msize), .mwdata(s_mwdata),
	.mrdata(mb_rdata), .mack(s_mack),
	.hold_mode(hold_mode), .img_mounted(sd_mounted), .img_size(sd_size),
	.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
	.vx(vx), .vy(vy), .vline(vline), .vrgb(vrgb),
	.spi_nss_out(spi_nss), .irq_spi(irq_spi), .irq_dma(irq_dma), .busy(busy), .busy_q(busy_q), .irq_ev(s_irq_ev), .work_q(s_work),
	.dbg_now(), .dbg_t(), .dbg_shift_end(), .dbg_rx_data(), .dbg_tx_data(), .dbg_shift_data(), .dbg_flags(),
	.dbg_intfr(), .dbg_cntr1(), .dbg_cntr2(), .dbg_pos1(), .dbg_pos2(),
	.dbg_lcd_stb(dbg_lcd_stb), .dbg_lcd_dc(dbg_lcd_dc), .dbg_lcd_byte(dbg_lcd_byte)
);

// the PFIC's lines, numbered as machine.h numbers them
assign irq_line = (64'(irq_systick) << 12) | (64'(irq_spi) << 33) | (64'(irq_t1) << 35) |
                  (64'(irq_t2) << 38) | (64'(irq_t3) << 54) | (64'(irq_dma) << 22);

endmodule
