module mux2_1_32bit (
	input  logic [31:0] a,
	input  logic [31:0] b,
	input  logic        s,
	output logic [31:0] y 
);
	assign y = s ? b : a;
endmodule
