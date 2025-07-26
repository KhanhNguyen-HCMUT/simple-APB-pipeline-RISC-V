module stageid(
	input logic i_clk, i_reset,
	input logic rd_wren_wb,
	input logic [31:0] instr_id, pc_id, pc_four_id, instr_wb, wb_data,
	
	output logic pc_sel, rd_wren, inst_vld, opa_sel, opb_sel, mem_wren,
	output logic [3:0] alu_op, lsu_op,
	output logic [1:0] wb_sel,
	output logic [31:0] rs1_data, rs2_data, immgen,
	//update
	output logic branch,
	output logic [2:0] funct3,    // FIXED: Changed from fun3 to funct3
	output logic br_un,
	
			input logic [31:0] alu_data_mem, alu_data,
		input logic [1:0] forward_a, forward_b
);
	logic [31:0] rs1_data_src, rs2_data_src;

    //regfile
    regfile regfile (
        .i_clk          (i_clk),
        .i_reset        (i_reset),
        .i_rs1_addr     (instr_id [19:15]),
        .i_rs2_addr     (instr_id [24:20]),
        .i_rd_addr      (instr_wb [11:7]),
        .i_rd_data      (wb_data),
        .i_rd_wren      (rd_wren_wb),
        .o_rs1_data     (rs1_data_src),
        .o_rs2_data     (rs2_data_src)
);
		//immgen
	    immgenn immgenn (
        .i_inst (instr_id),
        .o_imm (immgen)
);
    // Control unit
    controlunit controlunit (
        .i_instr    (instr_id),
        //.br_less    (br_less),
        //.br_equal   (br_equal),
        .pc_sel     (pc_sel),
        .rd_wren    (rd_wren),
		  .inst_vld		(inst_vld),
        .br_un      (br_un),
        .opa_sel    (opa_sel),
        .opb_sel    (opb_sel),
        .mem_wren   (mem_wren),
        .alu_op     (alu_op),
        .wb_sel     (wb_sel),
		  .lsu_op      (lsu_op),
		  .branch		(branch), //update
		  .funct3		(funct3)  // FIXED: Changed from fun3 to funct3
    );
	 
	 	mux4_1_32bit rs1_sel (
		.a (rs1_data_src),
		.b (wb_data),
		.c	(alu_data_mem),
		.d	(alu_data),
		.s	(forward_a),
		.y	(rs1_data)
	);
	
	mux4_1_32bit rs2_sel (
		.a (rs2_data_src),
		.b (wb_data),
		.c	(alu_data_mem),
		.d	(alu_data),
		.s	(forward_b),
		.y	(rs2_data)
	);

endmodule