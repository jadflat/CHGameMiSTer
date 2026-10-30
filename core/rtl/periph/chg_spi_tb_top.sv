// Testbench top for sim/tb_spi.cpp: SPI1, DMA1 and the SD card joined as
// chg_soc would join them, with the ST7735 on the SPI. The testbench is the
// rest of the board: it drives the bus, the GPIO pins (a stand-in for the
// GPIO module: PA4 follows spi_nss when it is in alternate function mode, as
// af_level has it), answers port B from its own copy of SRAM and flash, and
// is hps_io for the card. Not part of the core.

module chg_spi_tb_top
(
	input             clk,
	input             por,
	input             reset,
	input             ce,
	input             boundary,
	input             gpio_wr,
	input             fast_card,

	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output     [31:0] rdata,
	output            ack,

	input             pa4_af,       // PA4 is an alternate function output: SPI1 NSS
	input             pa4_gpio,     // PA4 otherwise
	input             pb0,
	input             pb11,
	input             lcd_rst_level,
	input             lcd_reset,    // st7735_reset (PB12 falling, chg_reset)

	output            mreq,
	output            mwe,
	output     [31:0] maddr,
	output      [2:0] msize,
	output     [31:0] mwdata,
	input      [31:0] mrdata,
	input             mack,

	input             hold_mode,
	input             img_mounted,
	input      [63:0] img_size,
	output     [31:0] sd_lba,
	output            sd_rd,
	output            sd_wr,
	input             sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din,
	input             sd_buff_wr,

	input       [6:0] vx,
	input       [6:0] vy,
	input             vline,
	output     [23:0] vrgb,

	output            spi_nss_out,      // for chg_gpio: PA4 in its alternate function (chg_soc)
	output            irq_spi,
	output      [6:0] irq_dma,
	output            busy,
	output            busy_q,       // busy less what comes straight from boundary, gpio_wr and req
	output            irq_ev,       // SPI1's line may change at the next boundary (DMA's only while busy)
	output            work_q,       // SPI1 has sync work due in this clock (registered; chg_spi)

	output     [63:0] dbg_now,
	output     [63:0] dbg_t,
	output     [63:0] dbg_shift_end,
	output     [15:0] dbg_rx_data,
	output     [15:0] dbg_tx_data,
	output     [15:0] dbg_shift_data,
	output      [7:0] dbg_flags,
	output     [31:0] dbg_intfr,
	output     [15:0] dbg_cntr1,
	output     [15:0] dbg_cntr2,
	output     [15:0] dbg_pos1,
	output     [15:0] dbg_pos2,
	output            dbg_lcd_stb,
	output            dbg_lcd_dc,
	output      [7:0] dbg_lcd_byte
);

wire sel_spi = addr[31:10] == 22'(32'h4001_3000 >> 10);
wire sel_dma = addr[31:10] == 22'(32'h4002_0000 >> 10);

wire [31:0] spi_rdata, dma_rdata;
wire        spi_ack, dma_ack;
assign rdata = sel_spi ? spi_rdata : dma_rdata;
assign ack   = spi_ack | dma_ack;

wire spi_nss;
assign spi_nss_out = spi_nss;
wire pa4 = pa4_af ? spi_nss : pa4_gpio;

wire        lcd_stb, lcd_dc;
wire  [7:0] lcd_byte;
wire        sd_stb, sd_done, sd_desel;
wire  [7:0] sd_mosi, sd_miso;
wire        dma_sync_req, dma_sync_done, tx_ok, rx_ok, tx_req, tx_done, rx_req, rx_done;
wire [15:0] tx_data, rx_wdata;
wire        spi_busy, dma_busy, spi_busy_q, dma_busy_q;
assign busy = spi_busy | dma_busy;
assign busy_q = spi_busy_q | dma_busy_q;
assign dbg_lcd_stb = lcd_stb;
assign dbg_lcd_dc = lcd_dc;
assign dbg_lcd_byte = lcd_byte;

wire rx_ff, cf_ok;

chg_spi spi
(
	.clk(clk), .por(por), .reset(reset), .ce(ce),
	.req(req && sel_spi), .we(we), .addr(addr), .size(size), .wdata(wdata), .rdata(spi_rdata), .ack(spi_ack),
	.boundary(boundary), .gpio_wr(gpio_wr), .fast_card(fast_card),
	.pa4(pa4), .pb0(pb0), .pb11(pb11), .lcd_rst_level(lcd_rst_level), .spi_nss(spi_nss),
	.lcd_stb(lcd_stb), .lcd_dc(lcd_dc), .lcd_byte(lcd_byte),
	.sd_stb(sd_stb), .sd_mosi(sd_mosi), .sd_done(sd_done), .sd_miso(sd_miso), .sd_desel(sd_desel),
	.dma_sync_req(dma_sync_req), .dma_sync_done(dma_sync_done),
	.tx_ok(tx_ok), .rx_ok(rx_ok), .tx_req(tx_req), .tx_done(tx_done), .tx_data_in(tx_data),
	.rx_req(rx_req), .rx_ff(rx_ff), .cf_ok(cf_ok), .rx_wdata(rx_wdata), .rx_done(rx_done),
	.irq(irq_spi), .busy(spi_busy), .busy_q(spi_busy_q), .irq_ev(irq_ev), .work_q(work_q),
	.dbg_now(dbg_now), .dbg_t(dbg_t), .dbg_shift_end(dbg_shift_end), .dbg_rx_data(dbg_rx_data),
	.dbg_tx_data(dbg_tx_data), .dbg_shift_data(dbg_shift_data), .dbg_flags(dbg_flags)
);

chg_dma dma
(
	.clk(clk), .reset(reset),
	.req(req && sel_dma), .we(we), .addr(addr), .size(size), .wdata(wdata), .rdata(dma_rdata), .ack(dma_ack),
	.irq(irq_dma),
	.spi_sync_req(dma_sync_req), .spi_sync_done(dma_sync_done),
	.tx_ok(tx_ok), .rx_ok(rx_ok), .tx_req(tx_req), .tx_done(tx_done), .tx_data(tx_data),
	.rx_req(rx_req), .rx_ff(rx_ff), .cf_ok(cf_ok), .rx_wdata(rx_wdata), .rx_done(rx_done),
	.mreq(mreq), .mwe(mwe), .maddr(maddr), .msize(msize), .mwdata(mwdata), .mrdata(mrdata), .mack(mack),
	.busy(dma_busy), .busy_q(dma_busy_q),
	.dbg_intfr(dbg_intfr), .dbg_cntr1(dbg_cntr1), .dbg_cntr2(dbg_cntr2), .dbg_pos1(dbg_pos1), .dbg_pos2(dbg_pos2)
);

chg_sdspi sd
(
	.clk(clk), .reset(por), .hold_mode(hold_mode),
	.xfer_stb(sd_stb), .mosi(sd_mosi), .xfer_done(sd_done), .miso(sd_miso), .desel_stb(sd_desel),
	.img_mounted(img_mounted), .img_size(img_size),
	.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr)
);

st7735 lcd
(
	.clk(clk), .reset(lcd_reset), .rst_level(lcd_rst_level),
	.byte_stb(lcd_stb), .dc(lcd_dc), .byte_in(lcd_byte),
	.vx(vx), .vy(vy), .vline(vline), .vrgb(vrgb)
);

endmodule
