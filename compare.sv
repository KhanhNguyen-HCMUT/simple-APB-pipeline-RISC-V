module compare(
input wire rst_n,
input wire clk,
input wire [31:0]addr,
input wire [31:0]wdata,
input wire wr_en,
output wire [63:0]tcmp,
output wire [31:0]tcmp1,
output wire [31:0]tcmp0
);
reg [31:0]tcmp1_r;
reg [31:0]tcmp0_r;
wire tcmp1_wr_sel;
wire tcmp0_wr_sel;
assign tcmp0_wr_sel = wr_en & (addr == 32'h2000000c);
assign tcmp1_wr_sel = wr_en & (addr == 32'h20000010);
always@(posedge clk or negedge rst_n)begin
if(rst_n == 0) tcmp0_r <= 32'hffff_ffff;
else begin
case(tcmp0_wr_sel)
1'h0: tcmp0_r <= tcmp0_r;
1'h1: tcmp0_r <= wdata;
endcase
end
end
always@(posedge clk or negedge rst_n)begin
if(rst_n == 0) tcmp1_r <= 32'hffff_ffff;
else begin
case(tcmp1_wr_sel)
1'h0: tcmp1_r <= tcmp1_r;
1'h1: tcmp1_r <= wdata;
endcase
end
end
assign tcmp = {tcmp1_r,tcmp0_r};
assign tcmp1 = tcmp1_r;
assign tcmp0 = tcmp0_r;
endmodule
