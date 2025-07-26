module pipelined (
    // Inputs
    input logic i_clk,
    input logic i_reset,
    input logic [31:0] i_io_sw,

    // NEW: APB Master Interface for external peripherals
    output logic [31:0] o_paddr,
    output logic        o_pwrite,
    output logic        o_psel,
    output logic        o_penable,
    output logic [31:0] o_pwdata,
    input  logic [31:0] i_prdata,
    input  logic        i_pready,

    // Outputs
    output logic [31:0] o_io_lcd,
    output logic [31:0] o_io_ledg,
    output logic [31:0] o_io_ledr,
    output logic [6:0] o_io_hex0,
    output logic [6:0] o_io_hex1,
    output logic [6:0] o_io_hex2,
    output logic [6:0] o_io_hex3,
    output logic [6:0] o_io_hex4,
    output logic [6:0] o_io_hex5,
    output logic [6:0] o_io_hex6,
    output logic [6:0] o_io_hex7,
    // Debugging signal
    output logic [31:0] o_pc_debug,
    output logic	o_insn_vld
);
	
	logic i_reset_pc,i_reset_if, i_reset_id, i_reset_ex, i_reset_mem;
	logic i_enable_pc, i_enable_if, i_enable_id, i_enable_ex, i_enable_mem;
	//IF
   logic pc_sel_wb;
	logic [31:0] alu_data_mem, pc_four, pc, instr;
	//ID
   logic pc_sel, rd_wren, inst_vld, br_un, opa_sel, opb_sel, mem_wren;
	logic [31:0] instr_id, pc_id, pc_four_id, rs1_data, rs2_data, immgen;
	logic [3:0] alu_op, lsu_op; 
	logic [1:0] wb_sel;
		//UPDATE
	logic branch;
	logic [2:0] funct3;              // Changed from fun3 to funct3
	
	//EX
	logic pc_sel_ex, rd_wren_ex, inst_vld_ex, mem_wren_ex, br_un_ex,br_equal, br_less, opa_sel_ex, opb_sel_ex;
	logic [31:0] alu_data, rs1_data_ex, rs2_data_ex, immgen_ex, pc_ex, pc_four_ex, instr_ex;
	logic [3:0] alu_op_ex, lsu_op_ex;
	logic [1:0] wb_sel_ex;
		//UPDATE
	logic branch_ex;
	logic [2:0] funct3_ex;           // Changed from fun3_ex to funct3_ex
	logic pc_sel_ex_temp;
	logic control_hazard;
	logic branch_taken;
	
	//MEM
	logic pc_sel_mem, rd_wren_mem, inst_vld_mem, mem_wren_mem;
	logic [31:0] rs2_data_mem, immgen_mem , pc_mem, pc_four_mem, instr_mem, ld_data;
	logic [3:0] lsu_op_mem;
	logic [2:0] funct3_mem;     // NEW: funct3 for MEM stage
	logic [6:0] opcode_mem;     // NEW: opcode for MEM stage
	logic [1:0] wb_sel_mem;
	logic lsu_stall;            // NEW: LSU stall signal
	
	//WB
	logic rd_wren_wb, inst_vld_wb;
	logic [31:0] pc_four_wb, alu_data_wb, ld_data_wb, immgen_wb, wb_data, pc_wb, instr_wb;
	logic [1:0] wb_sel_wb;
	//IO
	logic [31:0] o_io_lcd_mem, o_io_ledg_mem, o_io_ledr_mem, i_io_sw_wb;
	logic [6:0] o_io_hex0_mem, o_io_hex1_mem, o_io_hex2_mem, o_io_hex3_mem, o_io_hex4_mem, o_io_hex5_mem, o_io_hex6_mem, o_io_hex7_mem; 
	
	//Forward
	logic [1:0] forward_a, forward_b;
	
//////////////////////////////////////////////////////////////////////
	//STAGE IF
//////////////////////////////////////////////////////////////////////	
		stageif stageif(
		.i_clk		(i_clk),
		.i_reset		(i_reset),
		.i_reset_pc	(i_reset_pc),
		.i_enable_pc (i_enable_pc),
		.pc_sel_ex	(pc_sel_ex),
		.alu_data	(alu_data),
		.instr 			(instr),
		.pc				(pc),
		.pc_four			(pc_four)
		);
	// IF_FF
	if_ff if_ff(
		//in
		.i_clk 				(i_clk),
		.i_reset_if 		(i_reset_if),
		.i_enable_if		(i_enable_if),
		.instr 				(instr),
		.pc					(pc),
		.pc_four				(pc_four),
		//out
		.instr_id 			(instr_id),
		.pc_id 				(pc_id),
		.pc_four_id 		(pc_four_id)
	);
/////////////////////////////////////////////////////////////////////
//STAGE ID
/////////////////////////////////////////////////////////////////////
		stageid stageid(
		//in
		.i_clk          (i_clk),
		.i_reset        (i_reset),
		.rd_wren_wb  	  (rd_wren_wb),
		.instr_id 			(instr_id),
		.pc_id 				(pc_id),
		.pc_four_id 		(pc_four_id),
		.instr_wb			(instr_wb),
		.wb_data				(wb_data),
		.alu_data_mem	(alu_data_mem),
		.alu_data		(alu_data),
		.forward_a		(forward_a),
		.forward_b		(forward_b),
		//out
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
        .rs1_data     (rs1_data),
        .rs2_data     (rs2_data),
		  .immgen		 (immgen),
		  .branch		(branch),
		  .funct3		(funct3)
		);

		// ID_FF
		id_ff id_ff(
			//in
		  .i_clk			(i_clk),
		  .i_reset_id 	(i_reset_id),
		  .i_enable_id	(i_enable_id),
        .pc_sel     (pc_sel),
        .rd_wren    (rd_wren),
		  .inst_vld		(inst_vld),
        .br_un      (br_un),
        .opa_sel    (opa_sel),
        .opb_sel    (opb_sel),
        .mem_wren   (mem_wren),
        .alu_op     (alu_op),
        .wb_sel     (wb_sel),
		  .lsu_op     (lsu_op),
		  .rs1_data	  (rs1_data),
		  .rs2_data		(rs2_data),
		  .immgen		(immgen),
		  .instr_id		(instr_id),
		  .pc_id			(pc_id),
		  .pc_four_id	(pc_four_id),
		  //update
		  .branch		(branch),
		  .funct3		(funct3),
		  //out
		  .pc_sel_ex_temp     (pc_sel_ex_temp),
        .rd_wren_ex    (rd_wren_ex),
		  .inst_vld_ex		(inst_vld_ex),
        .br_un_ex      (br_un_ex),
        .opa_sel_ex    (opa_sel_ex),
        .opb_sel_ex    (opb_sel_ex),
        .mem_wren_ex   (mem_wren_ex),
        .alu_op_ex     (alu_op_ex),
        .wb_sel_ex     (wb_sel_ex),
		  .lsu_op_ex     (lsu_op_ex),
		  .rs1_data_ex	  (rs1_data_ex),
		  .rs2_data_ex		(rs2_data_ex),
		  .immgen_ex		(immgen_ex),
		  .instr_ex		(instr_ex),
		  .pc_ex			(pc_ex),
		  .pc_four_ex	(pc_four_ex),
		  //update
		  .branch_ex	(branch_ex),
		  .funct3_ex	(funct3_ex)
		  
		);
		
/////////////////////////////////////////////////////////////////////
//STAGE EX
/////////////////////////////////////////////////////////////////////
		stageex stageex (
		//in
		.br_un_ex      (br_un_ex),
		.opa_sel_ex    (opa_sel_ex),
		.opb_sel_ex    (opb_sel_ex),
		.pc_ex			(pc_ex),
		.rs1_data_ex	(rs1_data_ex),
		.rs2_data_ex	(rs2_data_ex),
		.immgen_ex		(immgen_ex),
		.alu_op_ex     (alu_op_ex),
		.branch_ex		(branch_ex),		
		.funct3_ex		(funct3_ex), 
		.pc_sel_ex_temp (pc_sel_ex_temp), 
		.control_hazard (control_hazard),
		//out
		.br_less  	 	 (br_less),
		.br_equal   	(br_equal),
		.alu_data		(alu_data),
		.branch_taken	(branch_taken),
		.pc_sel_ex		(pc_sel_ex)
		);
 
	 //EX_FF - MODIFIED to include funct3
	 ex_ff ex_ff (
			//in
				.i_clk 				(i_clk),
				.i_reset_ex			(i_reset_ex),
				.i_enable_ex		(i_enable_ex),
				.pc_sel_ex			(pc_sel_ex),
				.rd_wren_ex 		(rd_wren_ex),
				.inst_vld_ex		(inst_vld_ex),
				.mem_wren_ex		(mem_wren_ex),
				.wb_sel_ex			(wb_sel_ex),
				.alu_data			(alu_data),
				.rs2_data_ex		(rs2_data_ex),
				.immgen_ex			(immgen_ex),
				.instr_ex			(instr_ex),
				.pc_ex				(pc_ex),
				.pc_four_ex			(pc_four_ex),
				.lsu_op_ex 			(lsu_op_ex),
				.funct3_ex          (funct3_ex),       // NEW: connect funct3
				.opcode_ex          (instr_ex[6:0]),   // NEW: connect opcode
			//out
				.pc_sel_mem			(pc_sel_mem),
				.rd_wren_mem 		(rd_wren_mem),
				.inst_vld_mem		(inst_vld_mem),
				.mem_wren_mem		(mem_wren_mem),
				.wb_sel_mem			(wb_sel_mem),
				.alu_data_mem		(alu_data_mem),
				.rs2_data_mem		(rs2_data_mem),
				.immgen_mem			(immgen_mem),
				.instr_mem			(instr_mem),
				.pc_mem				(pc_mem),
				.pc_four_mem		(pc_four_mem),
				.lsu_op_mem 		(lsu_op_mem),
				.funct3_mem         (funct3_mem),     // NEW: output funct3
				.opcode_mem         (opcode_mem)      // NEW: output opcode
				);
/////////////////////////////////////////////////////////////////////////
//STAGE MEM - MODIFIED to include APB interface
/////////////////////////////////////////////////////////////////////////
		stagemem stagemem (
		//in 
		.i_clk			(i_clk),
		.i_reset 		(i_reset),
		.mem_wren_mem	(mem_wren_mem),
		.lsu_op_mem		(lsu_op_mem),
		.funct3_mem     (funct3_mem),        // NEW: funct3 input
		.opcode_mem     (opcode_mem),        // NEW: opcode input
		.alu_data_mem	(alu_data_mem),
		.rs2_data_mem	(rs2_data_mem),
		.i_io_sw_wb		(i_io_sw_wb),
		
		// NEW: APB Master Interface
		.o_paddr        (o_paddr),
		.o_pwrite       (o_pwrite),
		.o_psel         (o_psel),
		.o_penable      (o_penable),
		.o_pwdata       (o_pwdata),
		.i_prdata       (i_prdata),
		.i_pready       (i_pready),
		
		//out
		.ld_data 		(ld_data),
		.lsu_stall      (lsu_stall),         // NEW: stall signal
		.o_io_lcd_mem	(o_io_lcd_mem),
		.o_io_ledg_mem	(o_io_ledg_mem),
		.o_io_ledr_mem	(o_io_ledr_mem),
		.o_io_hex0_mem	(o_io_hex0_mem),
		.o_io_hex1_mem	(o_io_hex1_mem),
		.o_io_hex2_mem	(o_io_hex2_mem),
		.o_io_hex3_mem	(o_io_hex3_mem),
		.o_io_hex4_mem	(o_io_hex4_mem),
		.o_io_hex5_mem	(o_io_hex5_mem),
		.o_io_hex6_mem	(o_io_hex6_mem),
		.o_io_hex7_mem	(o_io_hex7_mem)
		
    );
		//MEM_FF
			mem_ff mem_ff (
			//in
				.i_clk			(i_clk),
				.i_reset_mem	(i_reset_mem),
				.i_enable_mem	(i_enable_mem),
				.pc_sel_mem		(pc_sel_mem),
				.rd_wren_mem	(rd_wren_mem),
				.inst_vld_mem  (inst_vld_mem),
				.wb_sel_mem		(wb_sel_mem),		
				.ld_data			(ld_data),
				.alu_data_mem	(alu_data_mem),
				.immgen_mem		(immgen_mem),
				.instr_mem		(instr_mem),
				.pc_four_mem	(pc_four_mem),
				.pc_mem			(pc_mem),
        .o_io_ledr_mem     (o_io_ledr_mem),
        .o_io_ledg_mem     (o_io_ledg_mem),
        .o_io_hex0_mem     (o_io_hex0_mem),
        .o_io_hex1_mem     (o_io_hex1_mem),
        .o_io_hex2_mem     (o_io_hex2_mem),
        .o_io_hex3_mem     (o_io_hex3_mem),
        .o_io_hex4_mem     (o_io_hex4_mem),
        .o_io_hex5_mem     (o_io_hex5_mem),
        .o_io_hex6_mem     (o_io_hex6_mem),
        .o_io_hex7_mem     (o_io_hex7_mem),
        .o_io_lcd_mem      (o_io_lcd_mem),
        .i_io_sw       (i_io_sw),
			//out
				.pc_sel_wb		(pc_sel_wb),
				.rd_wren_wb		(rd_wren_wb),
				.inst_vld_wb  (inst_vld_wb),
				.wb_sel_wb		(wb_sel_wb),		
				.ld_data_wb		(ld_data_wb),
				.alu_data_wb	(alu_data_wb),
				.immgen_wb		(immgen_wb),
				.instr_wb		(instr_wb),
				.pc_four_wb		(pc_four_wb),
				.pc_wb			(pc_wb),
        .o_io_ledr     (o_io_ledr),
        .o_io_ledg    (o_io_ledg),
        .o_io_hex0     (o_io_hex0),
        .o_io_hex1    (o_io_hex1),
        .o_io_hex2     (o_io_hex2),
        .o_io_hex3    (o_io_hex3),
        .o_io_hex4     (o_io_hex4),
        .o_io_hex5     (o_io_hex5),
        .o_io_hex6     (o_io_hex6),
        .o_io_hex7    (o_io_hex7),
        .o_io_lcd     (o_io_lcd),
        .i_io_sw_wb       (i_io_sw_wb)
			);
//////////////////////////////////////////////////////////////////////////////
//STAGE WB
//////////////////////////////////////////////////////////////////////////////
		stagewb stagewb(
	    .alu_data_wb	 (alu_data_wb),
	    .ld_data_wb	 (ld_data_wb),
	    .pc_four_wb	 (pc_four_wb),
       .immgen_wb		 (immgen_wb),
       .wb_sel_wb		 (wb_sel_wb),
       .wb_data (wb_data)
		);
	always_ff @(posedge i_clk) begin
		o_pc_debug <= pc_wb;
		
		end
	assign o_insn_vld = !inst_vld_wb;

//HAZARD DETECTION UNIT - MODIFIED to include LSU stall
	hdu hdu (
		.i_clk			(i_clk),
		.i_reset			(i_reset),
		.instr_id 		(instr_id),
		.instr_ex 		(instr_ex),
		.instr_mem		(instr_mem),
		.instr_wb		(instr_wb),
		.rd_wren_ex		(rd_wren_ex),
		.rd_wren_mem	(rd_wren_mem),
		.rd_wren_wb		(rd_wren_wb),
		.lsu_stall      (lsu_stall),         // NEW: APB stall input
		.i_reset_pc 	(i_reset_pc),
		.i_reset_if		(i_reset_if),
		.i_reset_id		(i_reset_id),
		.i_reset_ex		(i_reset_ex),
		.i_reset_mem	(i_reset_mem),
		.i_enable_pc	(i_enable_pc),
		.i_enable_if	(i_enable_if),
		.i_enable_id	(i_enable_id),		
		.i_enable_ex	(i_enable_ex),		
		.i_enable_mem	(i_enable_mem),
		.control_hazard (control_hazard),
		.branch_taken	(branch_taken),
		.pc_sel_ex_temp (pc_sel_ex_temp),
		.forward_a 		(forward_a),
		.forward_b		(forward_b),
		.lsu_op_ex		(lsu_op_ex),
		.mem_wren_ex	(mem_wren_ex),
		.mem_wren_mem	(mem_wren_mem)
	);

			
		
endmodule