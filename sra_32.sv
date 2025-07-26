module sra_32 (
    input  logic [31:0] a,    // Giá trị đầu vào cần dịch
    input  logic [31:0] b,    // Số bit dịch (5-bit)
    output logic [31:0] c     // Kết quả sau khi dịch
);
    logic [31:0] stage1, stage2, stage3, stage4, stage5;
    logic [4:0] shift;
    logic sign_extend; // Bit mở rộng dựa trên bit dấu

    assign shift = b[4:0];
    assign sign_extend = a[31]; // Lấy bit dấu của a

    // Tầng dịch 16-bit
    assign stage1 = shift[4] ? {{16{sign_extend}}, a[31:16]} : a;

    // Tầng dịch 8-bit
    assign stage2 = shift[3] ? {{8{sign_extend}}, stage1[31:8]} : stage1;

    // Tầng dịch 4-bit
    assign stage3 = shift[2] ? {{4{sign_extend}}, stage2[31:4]} : stage2;

    // Tầng dịch 2-bit
    assign stage4 = shift[1] ? {{2{sign_extend}}, stage3[31:2]} : stage3;

    // Tầng dịch 1-bit
    assign stage5 = shift[0] ? {{1{sign_extend}}, stage4[31:1]} : stage4;

    // Kết quả cuối cùng
    assign c = stage5;

endmodule
