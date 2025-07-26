module brc (
	input logic [31:0] i_rs1_data, // Dữ liệu từ thanh ghi 1
	input logic [31:0] i_rs2_data, // Dữ liệu từ thanh ghi 2
	input logic i_br_un,          // Chế độ so sánh (1: signed, 0: unsigned)
	output logic o_br_less,       // Kết quả rs1 < rs2
	output logic o_br_equal       // Kết quả rs1 == rs2
);

	logic signed_eq, signed_lt;
	logic unsigned_eq, unsigned_lt;
    
	// Comparator signed
	brc_comparator_signed comp_signed (
		.a(i_rs1_data),
		.b(i_rs2_data),
		.A_eq_B(signed_eq),
		.A_lt_B(signed_lt)
	);
    
	// Comparator unsigned
	brc_comparator_unsigned comp_unsigned (
		.a(i_rs1_data),
		.b(i_rs2_data),
		.A_eq_B(unsigned_eq),
		.A_lt_B(unsigned_lt)
	);
    
	// Multiplexers chọn kết quả theo chế độ so sánh
	mux2_1 mux_eq (
		.a(unsigned_eq),
		.b(signed_eq),
		.s(i_br_un),
		.y(o_br_equal)
	);
    
	mux2_1 mux_lt (
		.a(unsigned_lt),
		.b(signed_lt),
		.s(i_br_un),
		.y(o_br_less)
	);

endmodule
