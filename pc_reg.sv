module pc_reg (
	input logic i_clk,
	input logic i_reset,
	input logic [31:0] i_pc_next,
	output logic [31:0] o_pc
	);
	
	// Thanh ghi lưu trữ giá trị PC
	always_ff @(posedge i_clk) begin
		if (i_reset) 
				o_pc <= 32'h0; // Reset về 0 khi có tín hiệu reset
		else 
				o_pc <= i_pc_next; // Cập nhật PC với giá trị tiếp theo
	end

endmodule
