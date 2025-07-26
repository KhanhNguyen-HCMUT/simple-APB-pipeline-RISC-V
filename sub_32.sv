module sub_32 (
	input logic [31:0] a,
	input logic [31:0] b,
	output logic [31:0] s,
	output logic cout
	);
	
	logic [31:0] b_inv;
	logic [32:0] carry;
	
    assign b_inv = ~b;  // Lấy bù một của b
    assign carry[0] = 1; // Cộng thêm 1 vào b_inv để lấy bù hai

	
	genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin: adder_loop
            full_adder fa (
                .a(a[i]),
                .b(b_inv[i]),
                .cin(carry[i]),
                .sum(s[i]),
                .cout(carry[i+1])
            );
        end
    endgenerate

    assign cout = carry[32];
endmodule
