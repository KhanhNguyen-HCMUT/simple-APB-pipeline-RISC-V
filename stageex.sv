module stageex (
		input logic br_un_ex, opa_sel_ex, opb_sel_ex,
		input logic [31:0] pc_ex, rs1_data_ex, rs2_data_ex, immgen_ex,
		input logic [3:0] alu_op_ex,

		output logic br_less, br_equal,
		output logic [31:0] alu_data,
		
		//UPDATE
		input logic branch_ex,		
		input logic [2:0] funct3_ex,  // FIXED: Changed from fun3_ex to funct3_ex
		input logic pc_sel_ex_temp, 
		input logic control_hazard,
		
		output logic branch_taken,
		output logic pc_sel_ex
		//update forward

		
);
		logic [31:0] operand_a, operand_b;
   // Branch comparison logic
    brc brc (
        .i_rs1_data      (rs1_data_ex),
        .i_rs2_data      (rs2_data_ex),
        .i_br_un         (br_un_ex),
        .o_br_less       (br_less),
        .o_br_equal      (br_equal)
    );
	     // ALU operation
    alu alu (
        .i_op_a      (operand_a),
        .i_op_b      (operand_b),
        .i_alu_op    (alu_op_ex),
        .o_alu_data  (alu_data)
    );

    // Operand selection
    mux2_1_32bit opa (
        .a (rs1_data_ex),
        .b (pc_ex),
        .s (opa_sel_ex),
        .y (operand_a)
    );

    mux2_1_32bit opb (
        .a (rs2_data_ex),
        .b (immgen_ex),
        .s (opb_sel_ex),
        .y (operand_b)
    );
	 
	 branch_decision branch_decision (
		.fun3 			(funct3_ex),     // FIXED: Connect funct3_ex to fun3 port
		.branch			(branch_ex),               // Từ Control Unit qua ID/EX register
		.br_equal		(br_equal), 
		.br_less			(br_less),    // Từ BRC
		.branch_taken  (branch_taken)
	);
	
	mux2_1 pc_sel_mux (
		.a (pc_sel_ex_temp),
		.b (1'b1),
		.s (control_hazard),
		.y (pc_sel_ex)
	);
	

	
endmodule