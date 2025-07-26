module fulladder_32(
	input logic [31:0] a,
	input logic [31:0] b,
	output logic [31:0] sum
	);
	
	logic [32:0] carry; 
	assign carry[0]=0;
	
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: adder_loop
            full_adder fa (
                .a(a[i]),
                .b(b[i]),
                .cin(carry[i]),
                .sum(sum[i]),
                .cout(carry[i+1])
            );
        end
    endgenerate

endmodule

