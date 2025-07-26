module mux4_1_32bit (
	input logic [31:0] a,
	input logic [31:0] b,
	input logic [31:0] c,
	input logic [31:0] d,
	input logic [1:0] s,
	output logic [31:0] y
);

	assign y = (s == 2'b00) ? a :
              (s == 2'b01) ? b :
              (s == 2'b10) ? c :
			d;
endmodule
