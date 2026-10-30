// A true dual-port block RAM with byte enables, both ports reading and
// writing, one clock of read latency. Quartus 17 does not infer this from
// plain code, so it gets Altera's altsyncram; Verilator gets the plain code.
// What a port reads in the clock the other port writes the same word is not
// defined (no CHGame memory is used that way).

module chg_dpram #(parameter AW = 13, parameter BYTES = 4, parameter WORDS = 1 << AW, parameter INIT = "")
(
	input                    clk,

	input           [AW-1:0] a_addr,
	input        [BYTES-1:0] a_we,
	input      [8*BYTES-1:0] a_d,
	output     [8*BYTES-1:0] a_q,

	input           [AW-1:0] b_addr,
	input        [BYTES-1:0] b_we,
	input      [8*BYTES-1:0] b_d,
	output     [8*BYTES-1:0] b_q
);

`ifdef VERILATOR

reg [BYTES-1:0][7:0] mem[WORDS];
initial if (INIT != "") $readmemh(INIT, mem);
reg [8*BYTES-1:0] qa, qb;
always @(posedge clk) begin
	for (int i = 0; i < BYTES; i++) if (a_we[i]) mem[a_addr][i] <= a_d[8*i +: 8];
	qa <= mem[a_addr];
end
always @(posedge clk) begin
	for (int i = 0; i < BYTES; i++) if (b_we[i]) mem[b_addr][i] <= b_d[8*i +: 8];
	qb <= mem[b_addr];
end
assign a_q = qa;
assign b_q = qb;

`else

altsyncram #(
	.operation_mode("BIDIR_DUAL_PORT"),
	.width_a(8 * BYTES), .widthad_a(AW), .numwords_a(WORDS),
	.width_b(8 * BYTES), .widthad_b(AW), .numwords_b(WORDS),
	.width_byteena_a(BYTES), .width_byteena_b(BYTES), .byte_size(8),
	.outdata_reg_a("UNREGISTERED"), .outdata_reg_b("UNREGISTERED"),
	.address_reg_b("CLOCK0"), .indata_reg_b("CLOCK0"), .wrcontrol_wraddress_reg_b("CLOCK0"),
	.byteena_reg_b("CLOCK0"),
	.read_during_write_mode_mixed_ports("DONT_CARE"),
	.read_during_write_mode_port_a("NEW_DATA_NO_NBE_READ"),
	.read_during_write_mode_port_b("NEW_DATA_NO_NBE_READ"),
	.init_file(INIT == "" ? "UNUSED" : INIT),
	.intended_device_family("Cyclone V"),
	.lpm_type("altsyncram"),
	.power_up_uninitialized("FALSE")
) ram (
	.clock0(clk),
	.address_a(a_addr), .wren_a(|a_we), .byteena_a(a_we), .data_a(a_d), .q_a(a_q),
	.address_b(b_addr), .wren_b(|b_we), .byteena_b(b_we), .data_b(b_d), .q_b(b_q),
	.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0),
	.clocken0(1'b1), .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
	.clock1(1'b1), .rden_a(1'b1), .rden_b(1'b1), .eccstatus()
);

`endif

endmodule
