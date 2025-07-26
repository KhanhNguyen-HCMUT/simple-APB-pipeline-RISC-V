module alu (
	input logic [31:0] i_op_a,
	input logic [31:0] i_op_b,
	input logic [3:0] i_alu_op,
	output logic [31:0] o_alu_data
	);

	logic cout;
	logic [31:0] o_add;
	logic [31:0] o_sub;
	logic [31:0] o_slt;
	logic [31:0] o_sltu;
	logic [31:0] o_xor;
	logic [31:0] o_or;
	logic [31:0] o_and;
	logic [31:0] o_sll;
	logic [31:0] o_srl;
	logic [31:0] o_sra;
	logic [31:0] mux_inputs [15:0];

	assign mux_inputs[0]  = o_add;
	assign mux_inputs[1]  = o_sub;
	assign mux_inputs[2]  = o_slt;
	assign mux_inputs[3]  = o_sltu;
	assign mux_inputs[4]  = o_xor;
	assign mux_inputs[5]  = o_or;
	assign mux_inputs[6]  = o_and;
	assign mux_inputs[7]  = o_sll;
	assign mux_inputs[8]  = o_srl;
	assign mux_inputs[9]  = o_sra;
	assign mux_inputs[10] = 32'b0;  // Giá trị mặc định cho các trường hợp chưa sử dụng
	assign mux_inputs[11] = 32'b0;
	assign mux_inputs[12] = 32'b0;
	assign mux_inputs[13] = 32'b0;
	assign mux_inputs[14] = 32'b0;
	assign mux_inputs[15] = 32'b0;

	assign o_slt[31:1] = 31'b0;
	assign o_sltu[31:1] = 31'b0;

	fulladder_32 ADD (.a (i_op_a),
							.b (i_op_b),
							.sum (o_add));
							
	sub_32 SUB (.a (i_op_a),
					.b (i_op_b),
					.s (o_sub),
					.cout (cout)); //note
						
	comparator_32bit SLT (.a (i_op_a),
								 .b (i_op_b),
								 .A_lt_B (o_slt[0]));
								 
	comparator_32bit_unsigned SLTU (.a (i_op_a),
											  .b (i_op_b),
											  .a_lt_b (o_sltu[0]));
											  
	xor_32 XOR (.a (i_op_a),
					.b (i_op_b),
					.c (o_xor));
					
	or_32 OR (.a (i_op_a),
				 .b (i_op_b),
				 .c (o_or));
	
	and_32 AND (.a (i_op_a),
					.b (i_op_b),
					.c (o_and));
					
	sll_32 SLL (.a (i_op_a),
					.b (i_op_b),
					.c (o_sll));
					
	srl_32 SRL (.a (i_op_a),
					.b (i_op_b),
					.c (o_srl));
					
	sra_32 SRA (.a (i_op_a),
					.b (i_op_b),
					.c (o_sra));

	mux16_1 MUX (.in(mux_inputs),
						.sel(i_alu_op),
						.out(o_alu_data));
						
endmodule
