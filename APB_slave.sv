module APB_slave(
input wire clk,
input wire rst_n,
input wire psel,
input wire penable,
input wire pwrite,
output wire wr_en,
output wire rd_en,
output wire pready
);
assign wr_en = rst_n & psel & penable & pwrite;
assign rd_en = rst_n & psel & penable & !pwrite;
assign pready = rst_n & psel & penable;
endmodule
