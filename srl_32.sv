module srl_32 (
    input  logic [31:0] a,    // Giá trị đầu vào cần dịch
    input  logic [31:0] b,    // Số bit dịch (5-bit)
    output logic [31:0] c     // Kết quả sau khi dịch
);
    logic [31:0] stage1, stage2, stage3, stage4, stage5;
    logic [4:0] shift;
    
    assign shift = b[4:0];

    // Tầng dịch 16-bit
    assign stage1 = shift[4] ? {16'b0, a[31:16]} : a;

    // Tầng dịch 8-bit
    assign stage2 = shift[3] ? {8'b0, stage1[31:8]} : stage1;

    // Tầng dịch 4-bit
    assign stage3 = shift[2] ? {4'b0, stage2[31:4]} : stage2;

    // Tầng dịch 2-bit
    assign stage4 = shift[1] ? {2'b0, stage3[31:2]} : stage3;

    // Tầng dịch 1-bit
    assign stage5 = shift[0] ? {1'b0, stage4[31:1]} : stage4;

    // Kết quả cuối cùng
    assign c = stage5;

endmodule
