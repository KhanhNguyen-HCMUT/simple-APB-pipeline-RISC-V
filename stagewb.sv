module stagewb (
		input logic [31:0] pc_four_wb, alu_data_wb, ld_data_wb, immgen_wb,
		input logic [1:0] wb_sel_wb,
		
		output logic [31:0] wb_data
);
	    // Write-back multiplexer
    mux4_1_32bit wb (
	    .a (alu_data_wb),
	    .b (ld_data_wb),
	    .c (pc_four_wb),
        .d (immgen_wb),
        .s (wb_sel_wb),
        .y (wb_data)
    );

endmodule
