module stageif (
	input logic i_clk, i_reset, 
	input logic pc_sel_ex,
	input logic i_reset_pc, i_enable_pc,
	input logic [31:0] alu_data,
	
	output logic [31:0] instr,
	output logic [31:0] pc,
	output logic [31:0] pc_four
);
	logic [31:0] pc_next ;
		
	   assign pc_four = pc + 32'd4;
    //mux-PC source
    always_comb begin
        if (pc_sel_ex) begin //PC4-0 and ALU_DATA-1
            pc_next = alu_data;
        end
        else begin
            pc_next = pc_four;
        end
    end
    //PC counter
    always_ff @(posedge i_clk) begin
        if (!i_reset) begin 
		  pc <= 32'h0;
		  end
        else if (i_enable_pc) begin
		  pc <= pc_next;
		  end
		  else begin
		  pc <= pc;
		  end
    end
    // Memory unit (for fetching instructions)
	imem imem (
	.i_addr (pc),
	.o_inst (instr)
	);

endmodule

