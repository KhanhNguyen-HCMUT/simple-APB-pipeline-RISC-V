module hdu (
	input logic 		 i_clk,
	input logic 		 i_reset,
	input logic [31:0] instr_id,
	input logic [31:0] instr_ex,
	input logic [31:0] instr_mem,
	input logic [31:0] instr_wb,
	input logic rd_wren_ex, rd_wren_mem, rd_wren_wb,
	input logic pc_sel_ex_temp,
	input logic branch_taken,
	input logic [3:0] lsu_op_ex,
	input logic mem_wren_ex,
	input logic mem_wren_mem,
	input logic lsu_stall,        // NEW: APB stall signal from LSU

	output logic i_reset_pc, i_enable_pc,
	output logic i_reset_if, i_enable_if,	
	output logic i_reset_id, i_enable_id,
	output logic i_reset_ex, i_enable_ex,
	output logic i_reset_mem, i_enable_mem,
	output logic control_hazard,

	output logic [1:0] forward_a, forward_b
);
	logic [4:0] rs1_addr_id, rs2_addr_id, rd_addr_ex, rd_addr_mem, rd_addr_wb, rs1_addr_ex, rs2_addr_ex;
	logic is_rs2_addr_id;
	logic data_hazard_ex;
	logic [4:0] opcode_ex, opcode_mem;
	logic load_use_hazard;
	logic apb_pipeline_stall;     // NEW: APB stall for pipeline
	
	assign opcode_ex = instr_ex[6:2];
	assign opcode_mem = instr_mem[6:2];
	
	assign is_rs2_addr_id = ((instr_id[6:0] == 7'b0110011) ||   // R-type
							 (instr_id[6:0] == 7'b0100011) ||   // S-type
							 (instr_id[6:0] == 7'b1100011));    // B-type

	assign rs1_addr_id = instr_id[19:15];
	assign rs2_addr_id = instr_id[24:20];
	assign rs1_addr_ex = instr_ex[19:15];
	assign rs2_addr_ex = instr_ex[24:20];
	assign rd_addr_ex	 = instr_ex[11:7];
	assign rd_addr_mem = instr_mem[11:7];
	assign rd_addr_wb	 = instr_wb[11:7];

	// Chỉ stall nếu cần giá trị từ EX stage
	assign data_hazard_ex = (rd_wren_ex && (rd_addr_ex != 5'd0) &&((rd_addr_ex == rs1_addr_id) || (rd_addr_ex == rs2_addr_id && is_rs2_addr_id)));

	assign control_hazard = branch_taken || pc_sel_ex_temp;

	assign load_use_hazard = (rd_wren_ex && (rd_addr_ex != 5'd0) && 
                             !mem_wren_ex && (opcode_ex ==5'b0000) && ((rd_addr_ex == rs1_addr_id) || (rd_addr_ex == rs2_addr_id && is_rs2_addr_id))) ||
									  (rd_wren_mem && (rd_addr_mem != 5'd0) && 
                             !mem_wren_mem && (opcode_mem ==5'b0000) && ((rd_addr_mem == rs1_addr_id) || (rd_addr_mem == rs2_addr_id && is_rs2_addr_id)))
									  || (opcode_ex == 5'b01101) || (opcode_mem == 5'b01101);

	// NEW: APB stall logic - stall entire pipeline when APB transaction is active
	assign apb_pipeline_stall = lsu_stall;

	/* Forwarding logic: 
	00: no forward
	01: forward from WB
	10: forward from MEM
	11: forward from EX
	*/
	always_comb begin
		 // Forwarding cho rs1 (operand A)
		 if ((rd_wren_ex) && (rd_addr_ex != 5'h0) && (rd_addr_ex == rs1_addr_id)) begin
			  forward_a = 2'b11; // Forward từ EX (không áp dụng cho load)
		 end else if ((rd_wren_mem) && (rd_addr_mem != 5'h0) && (rd_addr_mem == rs1_addr_id)) begin
			  forward_a = 2'b10; // Forward từ MEM
		 end else if ((rd_wren_wb) && (rd_addr_wb != 5'h0) && (rd_addr_wb == rs1_addr_id)) begin
        forward_a = 2'b01; // Forward từ WB
		 end else begin
			  forward_a = 2'b00; // Không forward
		 end

		 // Forwarding cho rs2 (operand B)
		 if ((rd_wren_ex) && (rd_addr_ex != 5'h0) && (rd_addr_ex == rs2_addr_id) && is_rs2_addr_id) begin
			  forward_b = 2'b11; // Forward từ EX
		 end else if ((rd_wren_mem) && (rd_addr_mem != 5'h0) && (rd_addr_mem == rs2_addr_id) && is_rs2_addr_id) begin
			  forward_b = 2'b10; // Forward từ EM
		 end else if ((rd_wren_wb) && (rd_addr_wb != 5'h0) && (rd_addr_wb == rs2_addr_id) && is_rs2_addr_id) begin
			  forward_b = 2'b01; // Forward từ WB
		 end else begin
			  forward_b = 2'b00; // Không forward
		 end
	end

	// Pipeline control (stall hoặc flush)
	always_comb begin
		i_reset_pc   = i_reset;
		i_reset_if   = i_reset;
		i_reset_id   = i_reset;
		i_reset_ex   = i_reset;
		i_reset_mem  = i_reset;

		i_enable_pc  = 1'b1;
		i_enable_if  = 1'b1;
		i_enable_id  = 1'b1;
		i_enable_ex  = 1'b1;
		i_enable_mem = 1'b1;

		// NEW: APB stall has highest priority - stall entire pipeline
		if (apb_pipeline_stall) begin
			i_enable_pc  = 1'b0;  // Stop PC update
			i_enable_if  = 1'b0;  // Stop IF stage
			i_enable_id  = 1'b0;  // Stop ID stage  
			i_enable_ex  = 1'b0;  // Stop EX stage
			i_enable_mem = 1'b0;  // Stop MEM stage (keep APB transaction active)
		end
		else if (control_hazard) begin
			i_reset_if = 1'b0;
			i_reset_id = 1'b0;
		end
		else if (load_use_hazard) begin
            i_enable_pc = 1'b0;  // Ngừng PC để không fetch lệnh mới
            i_enable_if = 1'b0;  // Ngừng IF để không lấy lệnh mới
            i_enable_id = 1'b0;  // Ngừng ID để giữ lệnh hiện tại
            i_reset_id  = 1'b0;  // Flush EX để tránh thực thi sai
		end
	end
endmodule