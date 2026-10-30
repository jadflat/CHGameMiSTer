// SPI1 of the CH32X035 as emu/src/bus.c has it, and what hangs on it: the
// ST7735 (CS PA4, D/C PB0) and the microSD card (CS PB11).
//
// Translates spi_frame_cycles, spi_deliver, spi_sync, spi_read, spi_write
// and af_level's PA4 (spi_nss). A frame takes bits * (2 << BR) board cycles
// from when it enters the shift register; TXE, BSY, RXNE, OVR as spi_read
// makes them; 8 or 16-bit frames, LSB first for the display; the SPI1
// interrupt (TXEIE, RXNEIE) as spi_sync sets it.
//
// When things happen. The C keeps two cycle stamps (t, shift_end) and does
// the work lazily in spi_sync(now): frames whose shift_end <= now leave, the
// display and the card get them, the receive DMA stores them, the transmit
// DMA reads the next items, the interrupt line is set. It is called
//   - before and after every SPI1 or DMA1 register access, and before every
//     GPIO port write (gpio_write), at the access's cycle;
//   - at the end of every chg_run, which in the reference (chg_run one cycle
//     at a time) is the end of every instruction and every cycle of WFI: the
//     same cycle as the top of the next chg_run loop, where bus_events runs
//     (its spi_sync then has nothing more to do).
// Between those calls nothing of it changes. A CPU load from SRAM is made at
// the instruction's issue cycle, which can be later than the end of the
// instruction before it (fetch and divider waits), so a DMA write into SRAM
// for a frame that ended in that wait is not there yet for the load; the
// transmit DMA's SRAM read for it sees a store the instruction makes. This
// module keeps the same stamps (now counts ce) and does the same work at the
// same calls, so everything the CPU can see matches:
//   boundary  CONTRACT section 2: the top of each chg_run loop, i.e. the end
//             of the instruction before, at that cycle: before
//             cpu_take_interrupt's cycles and before the next instruction's
//             fetch and divider waits; and every cycle of WFI. Tied high it
//             gives a plain real-time SPI (a frame leaves on the cycle it
//             ends), which does not match the C.
//   gpio_wr   a GPIO port write takes effect at this clock: what has left the
//             shift register by now goes out with the pins as they were
//   own register accesses (req)
// While it works, busy is high: board time must not move (ce low) and the CPU
// must not make its next access. A sync with nothing to do costs no clock.

module chg_spi
(
	input             clk,
	input             por,          // power-on: board time back to 0
	input             reset,        // bus_reset (power-on or software reset)
	input             ce,           // one board cycle

	// the peripheral bus (CONTRACT section 3), 0x4001_3000..0x4001_33FF
	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,
	output reg        ack,

	input             boundary,
	input             gpio_wr,
	input             fast_card,    // OSD "Fast card loading": a frame to the selected card takes 1-2 board cycles (not the C's)

	// pins (gpio_pin), and the display's reset as gpio_changed last saw it
	input             pa4,          // LCD CS
	input             pb0,          // LCD D/C
	input             pb11,         // SD CS
	input             lcd_rst_level,
	output            spi_nss,      // PA4 in alternate function mode (af_level): !(SSOE && SPE)

	// the display (st7735.sv)
	output reg        lcd_stb,
	output reg        lcd_dc,
	output reg  [7:0] lcd_byte,

	// the card (chg_sdspi.sv)
	output reg        sd_stb,
	output reg  [7:0] sd_mosi,
	input             sd_done,
	input       [7:0] sd_miso,
	output reg        sd_desel,

	// DMA1 (chg_dma.sv)
	input             dma_sync_req,
	output reg        dma_sync_done,
	input             tx_ok,
	input             rx_ok,
	output reg        tx_req,
	input             tx_done,
	input      [15:0] tx_data_in,
	output reg        rx_req,
	output reg        rx_ff,        // with rx_req: the card fast path (the DMA writes at once)
	input             cf_ok,        // the DMA's items for it are plain SRAM/flash accesses that do not overlap
	output     [15:0] rx_wdata,
	input             rx_done,

	output reg        irq,          // CHG_IRQ_SPI1
	output            busy,
	output            busy_q,       // busy's registered part (see chg_soc: what ce and the CPU need)
	output reg        irq_ev,       // the next boundary's sync (with nothing to do) changes irq
	output reg        work_q,       // 'work' as it is in this clock, from registers (see below)

	// for the testbench
	output     [63:0] dbg_now,
	output     [63:0] dbg_t,
	output     [63:0] dbg_shift_end,
	output     [15:0] dbg_rx_data,
	output     [15:0] dbg_tx_data,
	output     [15:0] dbg_shift_data,
	output      [7:0] dbg_flags     // shifting, tx_full, rx_full, sd_selected, OVR
);

localparam SPI_SPE = 6, SPI_LSB = 7, SPI_DFF = 11;
localparam SPI_RXDMAEN = 0, SPI_TXDMAEN = 1, SPI_SSOE = 2, SPI_RXNEIE = 6, SPI_TXEIE = 7;

// ChgSpi
reg [15:0] ctlr1, ctlr2, crcr, hscr;
reg [15:0] statr_sticky;
reg        tx_full;
reg [15:0] tx_data;
reg        shifting;
reg [15:0] shift_data;
reg [63:0] shift_end;
reg [63:0] t;
reg        rx_full;
reg [15:0] rx_data;
reg        sd_selected;
reg [63:0] now;

assign spi_nss  = !(ctlr2[SPI_SSOE] && ctlr1[SPI_SPE]);
assign rx_wdata = rx_data;

assign dbg_now = now;
assign dbg_t = t;
assign dbg_shift_end = shift_end;
assign dbg_rx_data = rx_data;
assign dbg_tx_data = tx_data;
assign dbg_shift_data = shift_data;
assign dbg_flags = {3'b000, statr_sticky[6], sd_selected, rx_full, tx_full, shifting};

// spi_frame_cycles
wire [13:0] frame = (fast_card && !pb11) ? (ctlr1[SPI_DFF] ? 14'd2 : 14'd1) : (ctlr1[SPI_DFF] ? 14'd32 : 14'd16) << ctlr1[5:3];

function [7:0] rev8(input [7:0] b);
	for (int i = 0; i < 8; i++) rev8[i] = b[7 - i];
endfunction

// board time
always @(posedge clk) begin
	if (por) now <= 0;
	else if (ce) now <= now + 1'd1;
end

// spi_sync has something to do (otherwise it only moves t and sets the line)
wire work = ctlr1[SPI_SPE] &&
            ((!tx_full && ctlr2[SPI_TXDMAEN] && tx_ok) || (!shifting && tx_full) ||
             (shifting && shift_end <= now));
wire irq_now = (ctlr2[SPI_TXEIE] && !tx_full) || (ctlr2[SPI_RXNEIE] && rx_full);
// work, a clock ahead: what it is in the next clock if nothing but board time
// changes meanwhile (no sync running, no access: chg_cpu only uses it then,
// at a boundary it may make a board cycle of, CONTRACT.md 6a)
reg  [63:0] now1;                   // now + 1
always @(posedge clk) begin
	if (por) now1 <= 64'd1;
	else if (ce) now1 <= now1 + 1'd1;
	work_q <= ctlr1[SPI_SPE] &&
	          ((!tx_full && ctlr2[SPI_TXDMAEN] && tx_ok) || (!shifting && tx_full) ||
	           (shifting && (ce ? shift_end <= now1 : shift_end <= now)));
end
// (registered, so a clock behind: it is only looked at with boundary, and a
// boundary never comes in the clock after anything here changed: a sync or
// a register access is followed by more of the instruction, a GPIO write's
// sync by the rest of its access)
always @(posedge clk) irq_ev <= ctlr1[SPI_SPE] && irq != irq_now;

// the pins a sync delivers with: as they were when it was asked for
reg pa4_q, pb0_q, pb11_q, rst_q;

localparam
	S_IDLE = 5'd0, S_A = 5'd1, S_AW = 5'd2, S_B = 5'd3, S_D1 = 5'd4, S_D2 = 5'd5, S_D3 = 5'd6,
	S_D3W = 5'd7, S_D4W = 5'd8, S_D5 = 5'd9, S_D6 = 5'd10, S_D6W = 5'd11, S_END = 5'd12,
	R_DO = 5'd13, R_SET1 = 5'd14, R_SET2 = 5'd15, R_SET3 = 5'd16, R_ACK = 5'd17,
	S_F1 = 5'd18, S_F2 = 5'd19, S_FW = 5'd20,
	S_C1 = 5'd21, S_CW = 5'd22, S_CR = 5'd23, S_CT = 5'd24;
localparam RET_NONE = 2'd0, RET_DO = 2'd1, RET_ACK = 2'd2, RET_DMA = 2'd3;

reg  [4:0] st;
reg  [1:0] ret;
reg        r_we;
reg  [9:0] r_off;
reg  [1:0] r_lane;
reg  [2:0] r_size;
reg [31:0] r_v;
reg  [7:0] miso_hi;
reg [15:0] miso;
reg        sd_sel;

// The common sync, a fast path (S_F1 .. S_FW): one frame has left, the next
// is in the buffer, the transmit DMA refills it, the receive DMA is off and
// the card not selected. The same work as S_A .. S_END, in fewer clocks: the
// DMA's read is asked for at once (nothing writes memory in between).
wire fast = shifting && shift_end <= now && tx_full && ctlr2[SPI_TXDMAEN] && tx_ok &&
            !(ctlr2[SPI_RXDMAEN] && rx_ok) && pb11;
reg        f_lcd, f_due;
reg  [7:0] f_b2;

// The card's common sync (S_C1 .. S_CT): the same as the fast path above, but
// the card is selected, the receive DMA is on and frames are 8 bits. The card
// is asked at once, the transmit DMA reads its next item meanwhile (it does
// not overlap what the receive DMA writes, cf_ok, so the order does not
// matter), and the receive DMA writes straight to port B.
wire fast_sd = shifting && shift_end <= now && tx_full && ctlr2[SPI_TXDMAEN] && tx_ok &&
               ctlr2[SPI_RXDMAEN] && rx_ok && !pb11 && pa4 && !ctlr1[SPI_DFF] && cf_ok;
reg        c_tx;                       // the transmit DMA's item has not come yet

assign busy = st != S_IDLE || req || dma_sync_req || ((boundary || gpio_wr) && work);
assign busy_q = st != S_IDLE || dma_sync_req;

wire [9:0] off = {addr[9:2], 2'b00};

always @(posedge clk) begin
	ack <= 0;
	lcd_stb <= 0;
	sd_stb <= 0;
	sd_desel <= 0;
	tx_req <= 0;
	rx_req <= 0;
	rx_ff <= 0;
	dma_sync_done <= 0;

	if (st == S_IDLE || st == R_SET1 || st == R_SET2 || st == R_SET3)
		{pa4_q, pb0_q, pb11_q, rst_q} <= {pa4, pb0, pb11, lcd_rst_level};

	if (reset) begin
		ctlr1 <= 0; ctlr2 <= 0; crcr <= 0; hscr <= 0; statr_sticky <= 0;
		tx_full <= 0; tx_data <= 0; shifting <= 0; shift_data <= 0; shift_end <= 0; t <= 0;
		rx_full <= 0; rx_data <= 0; sd_selected <= 0;
		irq <= 0;
		st <= S_IDLE;
	end
	else case (st)
	S_IDLE:
		if (req || dma_sync_req || boundary || gpio_wr) begin
			// spi_read / spi_write / dma_read / dma_write: spi_sync first; or a boundary / GPIO write
			if (req) begin
				r_we <= we;
				r_off <= off;
				r_lane <= addr[1:0];
				r_size <= size;
				r_v <= wdata << {addr[1:0], 3'b000};
				ret <= RET_DO;
			end
			else if (dma_sync_req) ret <= RET_DMA;
			else ret <= RET_NONE;
			if (work) begin
				if (fast) begin
					tx_req <= 1;
					st <= S_F1;
				end
				else if (fast_sd) begin
					tx_req <= 1;
					sd_stb <= 1;
					sd_mosi <= shift_data[7:0];
					c_tx <= 1;
					st <= S_C1;
				end else
					st <= S_A;
			end
			else begin
				// nothing to do: only t and the line move (S_A .. S_END in this clock)
				if (!ctlr1[SPI_SPE] || !shifting) t <= now;
				if (ctlr1[SPI_SPE]) irq <= irq_now;
				if (req) st <= R_DO;
				else if (dma_sync_req) dma_sync_done <= 1;
			end
		end

	// ---- spi_sync ----
	S_A:
		if (!ctlr1[SPI_SPE]) begin
			t <= now;
			st <= S_END;
		end
		// the transmit DMA refills the buffer as soon as it is empty
		else if (!tx_full && ctlr2[SPI_TXDMAEN] && tx_ok) begin
			tx_req <= 1;
			st <= S_AW;
		end
		else st <= S_B;
	S_AW:
		if (tx_done) begin
			tx_data <= tx_data_in;
			tx_full <= 1;
			st <= S_B;
		end
	S_B:
		if (!shifting && tx_full) begin
			shifting <= 1;
			shift_data <= tx_data;
			tx_full <= 0;
			shift_end <= ((t > shift_end) ? t : shift_end) + 64'(frame);
			st <= S_A;
		end
		else if (shifting && shift_end <= now) begin
			shifting <= 0;
			t <= shift_end;
			st <= S_D1;
		end
		else st <= S_END;

	// ---- spi_deliver: the display listens while its CS is low and it is not in reset ----
	S_D1: begin
		if (!pa4_q && rst_q) begin
			lcd_stb <= 1;
			lcd_dc <= pb0_q;
			if (ctlr1[SPI_DFF]) lcd_byte <= ctlr1[SPI_LSB] ? rev8(shift_data[7:0]) : shift_data[15:8];
			else lcd_byte <= ctlr1[SPI_LSB] ? rev8(shift_data[7:0]) : shift_data[7:0];
		end
		st <= S_D2;
	end
	S_D2: begin
		if (!pa4_q && rst_q && ctlr1[SPI_DFF]) begin
			lcd_stb <= 1;
			lcd_byte <= ctlr1[SPI_LSB] ? rev8(shift_data[15:8]) : shift_data[7:0];
		end
		st <= S_D3;
	end
	// the card answers while its CS is low; otherwise MISO floats high
	S_D3: begin
		sd_sel <= !pb11_q;
		if (!pb11_q) begin
			sd_stb <= 1;
			sd_mosi <= ctlr1[SPI_DFF] ? shift_data[15:8] : shift_data[7:0];
			st <= S_D3W;
		end
		else begin
			if (sd_selected) sd_desel <= 1;
			miso <= ctlr1[SPI_DFF] ? 16'hFFFF : 16'h00FF;
			st <= S_D5;
		end
	end
	S_D3W:
		if (sd_done) begin
			if (ctlr1[SPI_DFF]) begin
				miso_hi <= sd_miso;
				sd_stb <= 1;
				sd_mosi <= shift_data[7:0];
				st <= S_D4W;
			end
			else begin
				miso <= {8'h00, sd_miso};
				st <= S_D5;
			end
		end
	S_D4W:
		if (sd_done) begin
			miso <= {miso_hi, sd_miso};
			st <= S_D5;
		end
	S_D5: begin
		sd_selected <= sd_sel;
		if (rx_full) statr_sticky <= statr_sticky | 16'h0040;     // OVR
		rx_full <= 1;
		rx_data <= miso;
		st <= S_D6;
	end
	// the receive DMA empties the buffer
	S_D6:
		if (ctlr2[SPI_RXDMAEN] && rx_ok) begin
			rx_req <= 1;
			st <= S_D6W;
		end
		else st <= S_A;
	S_D6W:
		if (rx_done) begin
			rx_full <= 0;
			st <= S_A;
		end
	// ---- the fast path: deliver, start the next frame, refill ----
	S_F1: begin
		// spi_deliver (the display; MISO floats high: the card is not selected)
		f_lcd <= !pa4_q && rst_q;
		if (!pa4_q && rst_q) begin
			lcd_stb <= 1;
			lcd_dc <= pb0_q;
			if (ctlr1[SPI_DFF]) lcd_byte <= ctlr1[SPI_LSB] ? rev8(shift_data[7:0]) : shift_data[15:8];
			else lcd_byte <= ctlr1[SPI_LSB] ? rev8(shift_data[7:0]) : shift_data[7:0];
		end
		f_b2 <= ctlr1[SPI_LSB] ? rev8(shift_data[15:8]) : shift_data[7:0];
		if (sd_selected) sd_desel <= 1;
		sd_selected <= 0;
		if (rx_full) statr_sticky <= statr_sticky | 16'h0040;     // OVR
		rx_full <= 1;
		rx_data <= ctlr1[SPI_DFF] ? 16'hFFFF : 16'h00FF;
		// the next frame starts where this one ended (t = shift_end)
		t <= shift_end;
		shift_data <= tx_data;
		tx_full <= 0;
		shift_end <= shift_end + 64'(frame);
		st <= S_F2;
	end
	S_F2: begin
		if (f_lcd && ctlr1[SPI_DFF]) begin
			lcd_stb <= 1;
			lcd_byte <= f_b2;
		end
		f_due <= shift_end <= now;
		st <= S_FW;
	end
	S_FW:
		if (tx_done) begin
			tx_data <= tx_data_in;
			tx_full <= 1;
			if (f_due) st <= S_A;           // that one has left too: the loop goes on
			else begin
				irq <= ctlr2[SPI_RXNEIE];     // irq_now with tx_full and rx_full
				st <= (ret == RET_NONE) ? S_IDLE : S_END;
			end
		end

	// ---- the card's fast path ----
	S_C1: begin
		// spi_deliver is under way (the card); the next frame starts where this one ended
		sd_selected <= 1;
		if (rx_full) statr_sticky <= statr_sticky | 16'h0040;     // OVR
		t <= shift_end;
		shift_data <= tx_data;
		tx_full <= 0;
		shift_end <= shift_end + 64'(frame);
		st <= S_CW;
	end
	S_CW: begin
		if (tx_done) begin tx_data <= tx_data_in; tx_full <= 1; c_tx <= 0; end
		f_due <= shift_end <= now;
		if (sd_done) begin
			rx_data <= {8'h00, sd_miso};
			rx_req <= 1;
			rx_ff <= 1;
			st <= S_CR;
		end
	end
	S_CR: begin
		if (tx_done) begin tx_data <= tx_data_in; tx_full <= 1; c_tx <= 0; end
		if (rx_done) begin
			rx_full <= 0;
			if (!c_tx || tx_done) begin
				if (f_due) st <= S_A;
				else begin irq <= 0; st <= (ret == RET_NONE) ? S_IDLE : S_END; end       // tx_full, not rx_full
			end
			else st <= S_CT;
		end
	end
	S_CT:
		if (tx_done) begin
			tx_data <= tx_data_in;
			tx_full <= 1;
			c_tx <= 0;
			if (f_due) st <= S_A;
			else begin irq <= 0; st <= S_IDLE; end
		end

	S_END: begin
		if (ctlr1[SPI_SPE]) begin
			if (!shifting) t <= now;
			irq <= irq_now;
		end
		case (ret)
			RET_DO:  st <= R_DO;
			RET_ACK: st <= R_ACK;
			RET_DMA: begin dma_sync_done <= 1; st <= S_IDLE; end
			default: st <= S_IDLE;
		endcase
	end

	// ---- the register access itself ----
	R_DO:
		if (!r_we) begin
			reg [31:0] v;
			case (r_off)
				10'h00: v = {16'd0, ctlr1};
				10'h04: v = {16'd0, ctlr2};
				10'h08: v = {16'd0, statr_sticky | {8'd0, shifting || tx_full, 5'd0, !tx_full, rx_full}};
				10'h0C: begin
					v = {16'd0, rx_data};
					rx_full <= 0;
					statr_sticky <= statr_sticky & ~16'h0040;
				end
				10'h10: v = {16'd0, crcr};
				10'h1C: v = {16'd0, hscr};
				default: v = 0;
			endcase
			v = v >> {r_lane, 3'b000};
			rdata <= (r_size == 1) ? (v & 32'hFF) : (r_size == 2) ? (v & 32'hFFFF) : v;
			ack <= 1;
			st <= S_IDLE;
		end
		else begin
			case (r_off)
				10'h00: begin
					ctlr1 <= r_v[15:0];
					if (!r_v[SPI_SPE]) begin shifting <= 0; tx_full <= 0; end
					t <= now;
				end
				10'h04: ctlr2 <= r_v[15:0];
				10'h08: statr_sticky <= statr_sticky & r_v[15:0];
				10'h0C:
					if (ctlr1[SPI_SPE]) begin
						tx_data <= r_v[15:0];
						tx_full <= 1;
						if (!shifting) t <= now;
					end
				10'h10: crcr <= r_v[15:0];
				10'h1C: hscr <= r_v[15:0];
				default: ;
			endcase
			// then spi_sync again, once PA4 (af_level) has followed CTLR1/CTLR2
			st <= R_SET1;
		end
	R_SET1: st <= R_SET2;
	R_SET2: st <= R_SET3;
	R_SET3: begin
		ret <= RET_ACK;
		if (work) begin
			if (fast) begin
				tx_req <= 1;
				st <= S_F1;
			end
			else if (fast_sd) begin
				tx_req <= 1;
				sd_stb <= 1;
				sd_mosi <= shift_data[7:0];
				c_tx <= 1;
				st <= S_C1;
			end else
				st <= S_A;
		end
		else begin
			if (!ctlr1[SPI_SPE] || !shifting) t <= now;
			if (ctlr1[SPI_SPE]) irq <= irq_now;
			st <= R_ACK;
		end
	end
	R_ACK: begin
		ack <= 1;
		st <= S_IDLE;
	end
	default: st <= S_IDLE;
	endcase
end

endmodule
