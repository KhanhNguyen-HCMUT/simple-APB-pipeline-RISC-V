module comparator_32bit_unsigned(
    input logic [31:0] a, b,   
    output logic a_lt_b       
);
    logic [31:0] sub_result;
    logic borrow;

    sub_32 subtractor (
        .a(a),
        .b(b),
        .s(sub_result),
        .cout(borrow) 
    );

    assign a_lt_b = ~borrow; 
endmodule
