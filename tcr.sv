module tcr(
input wire rst_n,
input wire clk,
input wire [31:0]addr,
input wire wr_en,
input wire [31:0]wdata,
output wire timer_en,
output wire div_en,
output wire [3:0]div_val,
output wire [31:0]tcr
);
wire tcr_wr_sel;
reg timer_en_r;
reg div_en_r;
reg [3:0]div_val_r;
assign tcr_wr_sel = wr_en & (addr == 32'h20000000);
always@(posedge clk or negedge rst_n)begin
if(!rst_n) timer_en_r <= 1'b0;
else begin 
case(tcr_wr_sel)
1'b1: timer_en_r <= wdata[0];
1'b0: timer_en_r <= timer_en_r;
endcase
end
end
always@(posedge clk or negedge rst_n)begin
if(!rst_n) div_en_r <= 1'b0;
else begin
case(tcr_wr_sel)
1'b1: div_en_r <= wdata[1];
1'b0: div_en_r <= div_en_r;
endcase
end
end
always@(posedge clk or negedge rst_n)begin
if(!rst_n) div_val_r <= 4'h1;
else begin
case(tcr_wr_sel & (wdata[11:8] < 4'b1001))
1'b1: div_val_r <= wdata[11:8];
1'b0: div_val_r <= div_val_r;
endcase
end
end

assign tcr = {20'h0, div_val_r, 6'h0, div_en_r, timer_en_r};
assign timer_en = timer_en_r;
assign div_en = div_en_r;
assign div_val = div_val_r;
endmodule
