// DMA1 of the CH32X035, as emu/src/bus.c has it: eight channels with their
// CFGR, CNTR, PADDR, MADDR, the shared INTFR/INTFCR, and the interrupt lines
// CHG_IRQ_DMA1_CH1 + n for channels 0..6 (channel index n is DMA1_Channel
// n+1, as in the C's m->dma.ch[n]).
//
// Translates dma_read, dma_write, dma_flag, dma_m2m, dma_take (channel 2,
// SPI1 TX), the receive half of spi_sync (channel 1, SPI1 RX) and
// dma_mem_read. What moves when is chg_spi's: the SPI asks for one item at a
// time (tx_req, rx_req) from inside its spi_sync, and every DMA register
// access is bracketed by spi_syncs as dma_read/dma_write do (spi_sync_req).
// Memory goes through port B (CONTRACT section 6) while board time is held
// (busy).
//
// Memory, as dma_mem_read and bus_write_slow reach it:
//   reads   SRAM 0x2000_0000, flash 0x0000_0000 and its 0x0800_0000 alias;
//           anything else reads 0 (the C would call bus_read_slow: a DMA
//           reading a peripheral register is not modelled here)
//   writes  SRAM only (a memory-to-memory write to flash or a peripheral,
//           which the C passes to bus_write_slow, is dropped)
// Misaligned items are split into bytes, as bus_read_slow/bus_write_slow do.
// Port B is only ever given aligned accesses; data are right aligned.
//
// Memory-to-memory transfers run entirely inside the CFGR write that starts
// them (dma_m2m), so they take no board time.

module chg_dma
(
	input             clk,
	input             reset,        // bus_reset (power-on or software reset)

	// the peripheral bus (CONTRACT section 3), 0x4002_0000..0x4002_03FF
	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,
	output reg        ack,

	output reg  [6:0] irq,          // CHG_IRQ_DMA1_CH1 + n

	// chg_spi: the spi_sync around each register access
	output reg        spi_sync_req,
	input             spi_sync_done,

	// chg_spi: one item for SPI1 TX (dma_take on channel 2), one from RX (channel 1)
	output            tx_ok,        // dma_ready(2) and memory to peripheral
	output            rx_ok,        // dma_ready(1) and peripheral to memory
	input             tx_req,
	output reg        tx_done,
	output reg [15:0] tx_data,
	input             rx_req,
	input             rx_ff,        // with rx_req: write at once (chg_spi's card fast path)
	output reg        cf_ok,        // channels 1 and 2 are plain aligned SRAM/flash items that do not overlap
	input      [15:0] rx_wdata,
	output reg        rx_done,

	// port B (CONTRACT section 6)
	output reg        mreq,
	output reg        mwe,
	output reg [31:0] maddr,
	output reg  [2:0] msize,
	output reg [31:0] mwdata,
	input      [31:0] mrdata,
	input             mack,

	output            busy,
	output            busy_q,       // busy's registered part

	// for the testbench
	output     [31:0] dbg_intfr,
	output     [15:0] dbg_cntr1,
	output     [15:0] dbg_cntr2,
	output     [15:0] dbg_pos1,
	output     [15:0] dbg_pos2
);

localparam DMA_EN = 0, DMA_TCIE = 1, DMA_HTIE = 2, DMA_DIR = 4, DMA_CIRC = 5, DMA_PINC = 6,
           DMA_MINC = 7, DMA_M2M = 14;

reg [14:0] cfgr[8];
reg [15:0] cntr[8];
reg [31:0] paddr[8], cmaddr[8];
reg [15:0] reload[8];               // count_reload
reg [15:0] pos[8];
reg [31:0] intfr;

assign tx_ok = cfgr[2][DMA_EN] && cntr[2] != 0 && cfgr[2][DMA_DIR];
assign rx_ok = cfgr[1][DMA_EN] && cntr[1] != 0 && !cfgr[1][DMA_DIR];

assign dbg_intfr = intfr;
assign dbg_cntr1 = cntr[1];
assign dbg_cntr2 = cntr[2];
assign dbg_pos1 = pos[1];
assign dbg_pos2 = pos[2];

// dma_size: 1, 2, 4 (and 8 for the reserved 11, as the C has it)
function [3:0] dsize(input [1:0] f); dsize = 4'd1 << f; endfunction

// the interrupt a channel asks for with these flags
function chan_irq(input [31:0] f, input [14:0] c, input [2:0] ch);
	chan_irq = (f[{ch, 2'd1}] && c[DMA_TCIE]) || (f[{ch, 2'd2}] && c[DMA_HTIE]);
endfunction

localparam [31:0] RAM_BASE = 32'h2000_0000, RAM_SIZE = 32'h5000, FLASH_SIZE = 32'h1_0000,
                  ALIAS = 32'h0800_0000;

// ---------------------------------------------------------------------------
// the state machine
// ---------------------------------------------------------------------------

localparam
	D_IDLE = 5'd0, D_PRE = 5'd1, D_ACC = 5'd2, D_POST = 5'd3, D_ACK = 5'd4,
	D_TX = 5'd5, D_ACC2 = 5'd6, D_RX = 5'd7, D_RX2 = 5'd8,
	D_M2M = 5'd9, D_M2M_R = 5'd10, D_M2M_A = 5'd11, D_M2M_N = 5'd12,
	M_START = 5'd13, M_ISSUE = 5'd14, M_WAIT = 5'd15;

reg  [4:0] st;
reg  [4:0] sret;                    // where a TX/RX service goes back to (D_IDLE, D_PRE, D_POST)
reg  [4:0] mret;                    // where a memory access goes back to
reg        txp, rxp;                // tx_req / rx_req not served yet
reg        swait;                   // waiting for spi_sync_done

reg        r_we;
reg  [9:0] r_off;
reg [31:0] r_word;                   // word offset in the page, in bytes
reg  [1:0] r_lane;
reg  [2:0] r_size;
reg [31:0] r_v;                     // the written value in its lane

// one memory access: m_addr, m_size (1, 2, 4, 8), m_we, m_wdata -> m_rdata
reg [31:0] m_addr;
reg  [3:0] m_size;
reg        m_we;
reg [31:0] m_wdata;
reg [31:0] m_rdata;
reg  [3:0] m_i;                     // bytes done
reg        m_split;                 // byte by byte

// memory to memory
reg  [2:0] mch;
reg [31:0] tx_ma, rx_ma;
reg        ma_ok;
reg        tx_fast, tx_fok, rx_fast, io_ov;
reg        rxff, m_rxf;
wire       rx_mw = st == M_WAIT && mack && m_rxf;
wire       tx_mw = st == M_WAIT && mack && !m_split && mret == D_TX;
reg [14:0] mc_cfg;                  // channel mch's CFGR, pos, PADDR, MADDR, CNTR != 0 (D_M2M_A)
reg [15:0] mc_pos;
reg [31:0] mc_pa, mc_ma;
reg        mc_cnz;
reg [31:0] mv;

// whatever changes a register that the next item's address comes from
wire chg = st == D_ACC || st == D_TX || tx_mw || rx_mw || st == D_RX || st == D_RX2 || st == D_M2M_N || reset;
assign busy = st != D_IDLE || req || txp || rxp;
assign busy_q = st != D_IDLE || txp || rxp;

wire [9:0] off = {addr[9:2], 2'b00};

// a register's value as dma_read returns it (word offset o)
// the channel registers 0x08..0xA4: channel (o - 8) / 20, register (o - 8) % 20,
// worked out when the access comes in (r_in, r_ch, r_fld; a table, not a divider)
function [5:0] chan_reg(input [9:0] o);
	integer k;
	begin
		chan_reg = 6'd0;
		for (k = 0; k < 40; k = k + 1)
			if (o == 10'h08 + 10'(4 * k)) chan_reg = {1'b1, 3'(k / 5), 2'(k % 5)};   // k % 5 == 4: r_res
	end
endfunction
// the reserved word after MADDR (k % 5 == 4)
function chan_res(input [9:0] o);
	integer k;
	begin
		chan_res = 1'b0;
		for (k = 0; k < 40; k = k + 1)
			if (o == 10'h08 + 10'(4 * k) && k % 5 == 4) chan_res = 1'b1;
	end
endfunction
reg        r_in, r_res;
reg  [2:0] r_ch;
reg  [1:0] r_fld;
function [31:0] reg_value(input [9:0] o);
	begin
		reg_value = 0;
		if (o == 10'h00) reg_value = intfr;
		else if (r_in && !r_res)
			case (r_fld)
				2'd0: reg_value = {17'd0, cfgr[r_ch]};
				2'd1: reg_value = {16'd0, cntr[r_ch]};
				2'd2: reg_value = paddr[r_ch];
				default: reg_value = cmaddr[r_ch];
			endcase
	end
endfunction

// does a whole access of n bytes at a go to memory port B? (dma_mem_read)
function rd_mem(input [31:0] a, input [3:0] n);
	rd_mem = (a - RAM_BASE <= RAM_SIZE - 32'(n)) || (a <= FLASH_SIZE - 32'(n)) ||
	         (a >= ALIAS && a < ALIAS + FLASH_SIZE);
endfunction
function wr_mem(input [31:0] a); wr_mem = a - RAM_BASE < RAM_SIZE; endfunction
// the port B address: flash through its low alias
function [31:0] pb_addr(input [31:0] a);
	pb_addr = (a >= ALIAS && a < ALIAS + FLASH_SIZE) ? {16'd0, a[15:0]} : a;
endfunction

always @(posedge clk) begin
	ack <= 0;
	mreq <= 0;
	tx_done <= 0;
	rx_done <= 0;
	spi_sync_req <= 0;
	if (tx_req) txp <= 1;
	if (rx_req) begin rxp <= 1; rxff <= rx_ff; end
	// channel 2's and 1's next memory address, worked out a clock ahead (96 MHz);
	// ma_ok: no register or pos changed in the clock before
	tx_ma <= cmaddr[2] + (cfgr[2][DMA_MINC] ? 32'(pos[2]) * 32'(dsize(cfgr[2][11:10])) : 0);
	rx_ma <= cmaddr[1] + (cfgr[1][DMA_MINC] ? 32'(pos[1]) * 32'(dsize(cfgr[1][11:10])) : 0);
	ma_ok <= !chg;
	// ... and whether channel 2's item is one aligned access of SRAM or flash
	// (registered from tx_ma: valid a clock after ma_ok)
	tx_fast <= ((tx_ma & 32'(dsize(cfgr[2][11:10]) - 1'd1)) == 0) && cfgr[2][11:10] != 2'd3 &&
	           rd_mem(tx_ma, dsize(cfgr[2][11:10]));
	tx_fok <= ma_ok && !chg;
	// the card's fast path (chg_spi fast_sd): channel 1's write is one aligned SRAM access too,
	// and the two items do not touch the same bytes, so that doing the read first changes nothing
	rx_fast <= ((rx_ma & 32'(dsize(cfgr[1][11:10]) - 1'd1)) == 0) && cfgr[1][11:10] != 2'd3 &&
	           wr_mem(rx_ma);
	io_ov <= tx_ma[31:2] == rx_ma[31:2] &&
	         ((cfgr[2][11:10] == 2'd0 ? 4'b0001 << tx_ma[1:0] : cfgr[2][11:10] == 2'd1 ? 4'b0011 << tx_ma[1:0] : 4'b1111) &
	          (cfgr[1][11:10] == 2'd0 ? 4'b0001 << rx_ma[1:0] : cfgr[1][11:10] == 2'd1 ? 4'b0011 << rx_ma[1:0] : 4'b1111)) != 0;
	cf_ok <= tx_fok && !chg && tx_fast && rx_fast && !io_ov;

	if (reset) begin
		for (int i = 0; i < 8; i++) begin
			cfgr[i] <= 0; cntr[i] <= 0; paddr[i] <= 0; cmaddr[i] <= 0; reload[i] <= 0; pos[i] <= 0;
		end
		intfr <= 0;
		irq <= 0;
		st <= D_IDLE;
		txp <= 0; rxp <= 0; swait <= 0; m_rxf <= 0;
	end
	else case (st)
	D_IDLE, D_PRE, D_POST:
		// SPI1's DMA items, asked for from inside its spi_sync
		if ((txp || tx_req) && !ma_ok) begin
			// the address is a clock behind a register or pos change: wait for it
			if (tx_req) txp <= 1;
			if (rx_req) rxp <= 1;
		end
		else if (txp || tx_req) begin
			txp <= 0;
			sret <= st;
			// dma_take(2)
			m_addr <= tx_ma;
			m_size <= dsize(cfgr[2][11:10]);
			m_we <= 0;
			mret <= D_TX;
			st <= M_START;
			if (tx_fok && tx_fast) begin
				// one aligned access: straight to port B, and dma_take's
				// work done with the answer (M_WAIT, tx_mw)
				m_split <= 0;
				mreq <= 1;
				mwe <= 0;
				maddr <= pb_addr(tx_ma);
				msize <= 3'(dsize(cfgr[2][11:10]));
				st <= M_WAIT;
			end
		end
		else if ((rxp || rx_req) && (rx_req ? rx_ff : rxff)) begin
			// the card's fast path: the write straight to port B, dma_take's work with its answer
			rxp <= 0;
			sret <= st;
			m_split <= 0;
			m_rxf <= 1;
			mreq <= 1;
			mwe <= 1;
			maddr <= rx_ma;
			msize <= 3'(dsize(cfgr[1][11:10]));
			mwdata <= {16'd0, rx_wdata};
			st <= M_WAIT;
		end
		else if ((rxp || rx_req) && !ma_ok) begin
			if (rx_req) rxp <= 1;
		end
		else if (rxp || rx_req) begin
			rxp <= 0;
			sret <= st;
			// the receive half of spi_sync
			m_addr <= rx_ma;
			m_size <= dsize(cfgr[1][11:10]);
			m_we <= 1;
			m_wdata <= {16'd0, rx_wdata};
			st <= D_RX;
		end
		else if (st == D_IDLE) begin
			if (req) begin
				// dma_read / dma_write: spi_sync first
				r_we <= we;
				r_off <= off;
				{r_in, r_ch, r_fld} <= chan_reg(off);
				r_res <= chan_res(off);
				r_lane <= addr[1:0];
				r_size <= size;
				r_v <= wdata << {addr[1:0], 3'b000};
				spi_sync_req <= 1;
				swait <= 1;
				st <= D_PRE;
			end
		end
		else if (swait) begin
			if (spi_sync_done) begin
				swait <= 0;
				st <= (st == D_PRE) ? D_ACC : D_ACK;
			end
		end
	D_ACC:
		if (!r_we) begin
			r_word <= reg_value(r_off);
			st <= D_ACC2;
		end
		else begin
			st <= D_POST;
			spi_sync_req <= 1;
			swait <= 1;
			if (r_off == 10'h04) begin
				// INTFCR: the global flag goes when the others of its channel have
				automatic reg [31:0] f = intfr & ~r_v;
				for (int ch = 0; ch < 8; ch++)
					if (f[ch*4 +: 4] == 4'b0001) f[ch*4] = 1'b0;
				intfr <= f;
				for (int ch = 0; ch < 7; ch++) irq[ch] <= chan_irq(f, cfgr[ch], 3'(ch));
			end
			else if (r_in && !r_res) begin
				automatic reg [2:0] ch = r_ch;
				case (r_fld)
				2'd0: begin
					cfgr[ch] <= r_v[14:0];
					if (!cfgr[ch][DMA_EN] && r_v[DMA_EN]) begin
						pos[ch] <= 0;
						reload[ch] <= cntr[ch];
						if (r_v[DMA_M2M]) begin
							mch <= ch;
							st <= D_M2M_A;
							spi_sync_req <= 0;
							swait <= 0;
						end
					end
				end
				2'd1: if (!cfgr[ch][DMA_EN]) cntr[ch] <= r_v[15:0];
				2'd2: paddr[ch] <= r_v;
				default: cmaddr[ch] <= r_v;
				endcase
			end
		end
	D_ACC2: begin
		// the read's lane and size, a clock after the register (96 MHz)
		automatic reg [31:0] v = r_word >> {r_lane, 3'b000};
		rdata <= (r_size == 1) ? (v & 32'hFF) : (r_size == 2) ? (v & 32'hFFFF) : v;
		ack <= 1;
		st <= D_IDLE;
	end
	D_ACK: begin
		ack <= 1;
		st <= D_IDLE;
	end

	// ---- dma_take(2): the item is in m_rdata ----
	D_TX: begin
		automatic reg [15:0] n = cntr[2] - 1'd1;
		automatic reg [31:0] f = intfr;
		tx_data <= m_rdata[15:0];
		pos[2] <= pos[2] + 1'd1;
		cntr[2] <= n;
		if (n == reload[2] >> 1) f = f | (32'h5 << 8);           // dma_flag(2, HT)
		if (n == 0) begin
			if (cfgr[2][DMA_CIRC]) begin cntr[2] <= reload[2]; pos[2] <= 0; end
			f = f | (32'h3 << 8);                                  // dma_flag(2, TC)
		end
		if (f != intfr || n == 0 || n == reload[2] >> 1) begin
			intfr <= f;
			if (n == 0 || n == reload[2] >> 1) irq[2] <= chan_irq(f, cfgr[2], 3'd2);
		end
		tx_done <= 1;
		st <= sret;
	end

	// ---- the receive DMA empties the buffer ----
	D_RX:
		if (m_addr - RAM_BASE <= RAM_SIZE - 32'(m_size)) begin
			mret <= D_RX2;
			st <= M_START;
		end
		else st <= D_RX2;
	D_RX2: begin
		automatic reg [15:0] n = cntr[1] - 1'd1;
		pos[1] <= pos[1] + 1'd1;
		cntr[1] <= n;
		if (n == 0) begin
			automatic reg [31:0] f = intfr | (32'h3 << 4);         // dma_flag(1, TC)
			if (cfgr[1][DMA_CIRC]) begin cntr[1] <= reload[1]; pos[1] <= 0; end
			intfr <= f;
			irq[1] <= chan_irq(f, cfgr[1], 3'd1);
		end
		rx_done <= 1;
		st <= sret;
	end

	// ---- dma_m2m ----
	// the channel's registers first (96 MHz: one mux, then the sums)
	D_M2M_A: begin
		mc_cfg <= cfgr[mch]; mc_pos <= pos[mch]; mc_pa <= paddr[mch]; mc_ma <= cmaddr[mch];
		mc_cnz <= cntr[mch] != 0;
		st <= D_M2M;
	end
	D_M2M:
		if (mc_cnz) begin
			automatic reg [3:0] ps = dsize(mc_cfg[9:8]), ms = dsize(mc_cfg[11:10]);
			automatic reg [31:0] pa = mc_pa + (mc_cfg[DMA_PINC] ? 32'(mc_pos) * 32'(ps) : 0);
			automatic reg [31:0] ma = mc_ma + (mc_cfg[DMA_MINC] ? 32'(mc_pos) * 32'(ms) : 0);
			m_addr <= mc_cfg[DMA_DIR] ? ma : pa;
			m_size <= mc_cfg[DMA_DIR] ? ms : ps;
			m_we <= 0;
			mv <= mc_cfg[DMA_DIR] ? pa : ma;            // where it goes
			mret <= D_M2M_R;
			st <= M_START;
		end
		else begin
			automatic reg [31:0] f = intfr | (32'h3 << {mch, 2'b00});   // dma_flag(ch, TC)
			intfr <= f;
			if (mch < 7) irq[mch] <= chan_irq(f, cfgr[mch], mch);
			spi_sync_req <= 1;
			swait <= 1;
			st <= D_POST;
		end
	D_M2M_R: begin
		m_wdata <= m_rdata;
		m_addr <= mv;
		m_size <= cfgr[mch][DMA_DIR] ? dsize(cfgr[mch][9:8]) : dsize(cfgr[mch][11:10]);
		m_we <= 1;
		mret <= D_M2M_N;
		st <= M_START;
	end
	D_M2M_N: begin
		pos[mch] <= pos[mch] + 1'd1;
		cntr[mch] <= cntr[mch] - 1'd1;
		st <= D_M2M_A;
	end

	// ---- one memory access through port B ----
	M_START: begin
		m_i <= 0;
		m_rdata <= 0;
		// misaligned: byte by byte (bus_read_slow / bus_write_slow split it)
		m_split <= (m_addr & 32'(m_size - 1'd1)) != 0 || m_size > 4;
		st <= M_ISSUE;
	end
	M_ISSUE: begin
		automatic reg [31:0] a = m_split ? m_addr + 32'(m_i) : m_addr;
		automatic reg [3:0] n = m_split ? 4'd1 : m_size;
		automatic reg go = m_we ? wr_mem(a) : rd_mem(a, n);
		if (m_split && m_i >= m_size) st <= mret;
		else if (m_split && m_i >= 4) m_i <= m_i + 1'd1;      // an 8-byte item: only 4 fit
		else if (go) begin
			mreq <= 1;
			mwe <= m_we;
			maddr <= pb_addr(a);
			msize <= 3'(n);
			mwdata <= m_split ? 32'(m_wdata[{m_i[1:0], 3'b000} +: 8]) : m_wdata;
			st <= M_WAIT;
		end
		else begin
			// nothing there: reads 0, a write goes nowhere
			if (m_split) m_i <= m_i + 1'd1;
			else st <= mret;
		end
	end
	M_WAIT:
		if (mack && m_rxf) begin
			// the receive half of spi_sync, as D_RX2
			automatic reg [15:0] n = cntr[1] - 1'd1;
			pos[1] <= pos[1] + 1'd1;
			cntr[1] <= n;
			if (n == 0) begin
				automatic reg [31:0] f = intfr | (32'h3 << 4);         // dma_flag(1, TC)
				if (cfgr[1][DMA_CIRC]) begin cntr[1] <= reload[1]; pos[1] <= 0; end
				intfr <= f;
				irq[1] <= chan_irq(f, cfgr[1], 3'd1);
			end
			rx_done <= 1;
			m_rxf <= 0;
			st <= sret;
		end
		else if (mack && !m_split && mret == D_TX) begin
			// dma_take(2), as D_TX, with the item straight from port B
			automatic reg [15:0] n = cntr[2] - 1'd1;
			automatic reg [31:0] f = intfr;
			tx_data <= (m_size == 4'd1) ? {8'd0, mrdata[7:0]} : mrdata[15:0];
			pos[2] <= pos[2] + 1'd1;
			cntr[2] <= n;
			if (n == reload[2] >> 1) f = f | (32'h5 << 8);           // dma_flag(2, HT)
			if (n == 0) begin
				if (cfgr[2][DMA_CIRC]) begin cntr[2] <= reload[2]; pos[2] <= 0; end
				f = f | (32'h3 << 8);                                  // dma_flag(2, TC)
			end
			if (f != intfr || n == 0 || n == reload[2] >> 1) begin
				intfr <= f;
				if (n == 0 || n == reload[2] >> 1) irq[2] <= chan_irq(f, cfgr[2], 3'd2);
			end
			tx_done <= 1;
			st <= sret;
			if (rx_req ? rx_ff : (rxp && rxff)) begin
				// the card's fast path, receive half asked for meanwhile: its write goes out now
				rxp <= 0;
				m_split <= 0;
				m_rxf <= 1;
				mreq <= 1;
				mwe <= 1;
				maddr <= rx_ma;
				msize <= 3'(dsize(cfgr[1][11:10]));
				mwdata <= {16'd0, rx_wdata};
				st <= M_WAIT;
			end
		end
		else if (mack) begin
			if (!m_split) begin
				if (!m_we) m_rdata <= (m_size == 1) ? (mrdata & 32'hFF) : (m_size == 2) ? (mrdata & 32'hFFFF) : mrdata;
				st <= mret;
			end
			else begin
				if (!m_we) m_rdata[{m_i[1:0], 3'b000} +: 8] <= mrdata[7:0];
				m_i <= m_i + 1'd1;
				st <= M_ISSUE;
			end
		end
	default: st <= D_IDLE;
	endcase
end

endmodule
