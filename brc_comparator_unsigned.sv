module brc_comparator_unsigned(
    input logic [31:0] a, b,    // Hai số cần so sánh
    output logic A_eq_B,        // A == B
    output logic A_lt_B       // A < B
);
    logic [31:0] sub_result;
    logic borrow;

    // Sử dụng bộ trừ 32-bit
    sub_32 subtractor (
        .a(a),
        .b(b),
        .s(sub_result),
        .cout(borrow) // borrow = 0 nếu A < B
    );

    // So sánh kết quả
    assign A_eq_B = (sub_result == 32'b0); // Nếu kết quả phép trừ là 0 thì A == B
    assign A_lt_B = ~borrow;               // Nếu có mượn (borrow = 0) thì A < B
	 
endmodule
