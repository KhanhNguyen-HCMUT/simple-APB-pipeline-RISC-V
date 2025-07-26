module mux16_1 (
    input  logic [31:0] in [15:0], // 16 đầu vào, mỗi đầu vào 32-bit
    input  logic [3:0] sel,        // 4-bit chọn
    output logic [31:0] out        // Đầu ra 32-bit
);

    assign out = (in[0]  & {32{~sel[3] & ~sel[2] & ~sel[1] & ~sel[0]}}) |
                 (in[1]  & {32{~sel[3] & ~sel[2] & ~sel[1] &  sel[0]}}) |
                 (in[2]  & {32{~sel[3] & ~sel[2] &  sel[1] & ~sel[0]}}) |
                 (in[3]  & {32{~sel[3] & ~sel[2] &  sel[1] &  sel[0]}}) |
                 (in[4]  & {32{~sel[3] &  sel[2] & ~sel[1] & ~sel[0]}}) |
                 (in[5]  & {32{~sel[3] &  sel[2] & ~sel[1] &  sel[0]}}) |
                 (in[6]  & {32{~sel[3] &  sel[2] &  sel[1] & ~sel[0]}}) |
                 (in[7]  & {32{~sel[3] &  sel[2] &  sel[1] &  sel[0]}}) |
                 (in[8]  & {32{ sel[3] & ~sel[2] & ~sel[1] & ~sel[0]}}) |
                 (in[9]  & {32{ sel[3] & ~sel[2] & ~sel[1] &  sel[0]}}) |
                 (in[10] & {32{ sel[3] & ~sel[2] &  sel[1] & ~sel[0]}}) |
                 (in[11] & {32{ sel[3] & ~sel[2] &  sel[1] &  sel[0]}}) |
                 (in[12] & {32{ sel[3] &  sel[2] & ~sel[1] & ~sel[0]}}) |
                 (in[13] & {32{ sel[3] &  sel[2] & ~sel[1] &  sel[0]}}) |
                 (in[14] & {32{ sel[3] &  sel[2] &  sel[1] & ~sel[0]}}) |
                 (in[15] & {32{ sel[3] &  sel[2] &  sel[1] &  sel[0]}});

endmodule


