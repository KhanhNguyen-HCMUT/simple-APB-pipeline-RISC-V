module if_ff (
		input logic i_clk, i_enable_if, i_reset_if,
		input logic [31:0] instr,
		input logic [31:0] pc,
		input logic [31:0] pc_four,
		
		output logic [31:0] instr_id,
		output logic [31:0] pc_id,
		output logic [31:0] pc_four_id
);
	always_ff @(posedge i_clk) begin
		if (!i_reset_if) begin
			instr_id 	<= 32'h0000_0013;
			pc_id 		<= 32'h0;
			pc_four_id	<= 32'h0;
			end
		else if (i_enable_if) begin
			instr_id 	<= instr;
			pc_id 		<= pc;
			pc_four_id	<= pc_four;
			end
		end
endmodule
