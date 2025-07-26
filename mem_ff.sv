module mem_ff (
		input logic i_clk, i_reset_mem, i_enable_mem,
		input logic pc_sel_mem, rd_wren_mem, inst_vld_mem,
		input logic [1:0] wb_sel_mem,
		input logic [31:0] ld_data,
		input logic [31:0] alu_data_mem,
		input logic [31:0] immgen_mem,
		input logic [31:0] instr_mem,
		input logic [31:0] pc_four_mem,
		input logic [31:0] pc_mem,
		input logic [31:0] i_io_sw,
		input logic [31:0] o_io_lcd_mem,
		input logic [31:0] o_io_ledg_mem,
		input logic [31:0] o_io_ledr_mem,
		input logic [6:0] o_io_hex0_mem,
		input logic [6:0] o_io_hex1_mem,
		input logic [6:0] o_io_hex2_mem,
		input logic [6:0] o_io_hex3_mem,
		input logic [6:0] o_io_hex4_mem,
		input logic [6:0] o_io_hex5_mem,
		input logic [6:0] o_io_hex6_mem,
		input logic [6:0] o_io_hex7_mem,
		
		output logic pc_sel_wb, rd_wren_wb, inst_vld_wb,
		output logic [1:0] wb_sel_wb,
		output logic [31:0] ld_data_wb,
		output logic [31:0] alu_data_wb,
		output logic [31:0] immgen_wb,
		output logic [31:0] instr_wb,
		output logic [31:0] pc_four_wb,
		output logic [31:0] pc_wb,
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
		output logic [31:0] i_io_sw_wb
);
		always_ff @(posedge i_clk) begin
			if (!i_reset_mem) begin
				pc_sel_wb 		<= 1'b0;
				rd_wren_wb 		<= 1'b0;
				inst_vld_wb		<= 1'b1;
				wb_sel_wb		<= 2'b00;
				ld_data_wb		<= 32'h0;
				alu_data_wb		<= 32'h0;
				immgen_wb		<= 32'h0;
				instr_wb			<= 32'h0000_0013;
				pc_four_wb		<= 32'h0;
				pc_wb				<= 32'h0;
				o_io_lcd		<= 32'h0;
				o_io_ledg	<= 32'h0;
				o_io_ledr	<= 32'h0;
				o_io_hex0	<= 7'b0000_000;
				o_io_hex1	<= 7'b0000_000;
				o_io_hex2	<= 7'b0000_000;
				o_io_hex3	<= 7'b0000_000;
				o_io_hex4	<= 7'b0000_000;
				o_io_hex5	<= 7'b0000_000;
				o_io_hex6	<= 7'b0000_000;
				o_io_hex7	<= 7'b0000_000;
				i_io_sw_wb		<= 32'h0;
				end
			else if (i_enable_mem) begin
				pc_sel_wb 		<= pc_sel_mem;
				rd_wren_wb 		<= rd_wren_mem;
				inst_vld_wb		<= inst_vld_mem;
				wb_sel_wb		<= wb_sel_mem;
				ld_data_wb		<= ld_data;
				alu_data_wb		<= alu_data_mem;
				immgen_wb		<= immgen_mem;
				instr_wb			<= instr_mem;
				pc_four_wb		<= pc_four_mem;
				pc_wb				<= pc_mem;
				o_io_lcd		<= o_io_lcd_mem;
				o_io_ledg	<= o_io_ledg_mem;
				o_io_ledr	<= o_io_ledr_mem;
				o_io_hex0	<= o_io_hex0_mem;
				o_io_hex1	<= o_io_hex1_mem;
				o_io_hex2	<= o_io_hex2_mem;
				o_io_hex3	<= o_io_hex3_mem;
				o_io_hex4	<= o_io_hex4_mem;
				o_io_hex5	<= o_io_hex5_mem;
				o_io_hex6	<= o_io_hex6_mem;
				o_io_hex7	<= o_io_hex7_mem;
				i_io_sw_wb		<= i_io_sw;
				end
			end
endmodule
