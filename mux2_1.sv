module mux2_1 (
	input logic a,
	input logic b,
	input logic s,
	output logic y
	);

	assign y = s ? b : a;

endmodule
