module interrupt(
input wire clk,
input wire rst_n,
input wire [31:0] wdata,
input wire [31:0] addr,
input wire wr_en,
input wire [63:0] cnt,
input wire [63:0] tcmp,
output wire tim_int,
output wire [31:0]tier,
output wire [31:0]tisr
);
wire int_set;
wire int_clr;
wire tisr_wr_sel;
wire tier_wr_sel;
reg int_en;
reg int_st;
assign int_set = (cnt == tcmp);
assign tisr_wr_sel = wr_en & (addr == 32'h20000018);
assign tier_wr_sel = wr_en & (addr == 32'h20000014); 
assign int_clr = tisr_wr_sel & (wdata[0] == 1'b1);
always@(posedge clk or negedge rst_n)begin
if(rst_n == 0) int_en <= 1'h0;
else begin 
case(tier_wr_sel)
1'h1: int_en <= wdata[0];
1'h0: int_en <= int_en;
endcase
end
end
assign tier = {31'h0, int_en};
always@(posedge clk or negedge rst_n)begin
if(rst_n == 0) int_st <= 1'b0; 
else begin if(int_clr) int_st <= 1'b0; 
	   else begin case(int_set)
	   	      1'b1: int_st <= 1'b1;
		      1'b0: int_st <= int_st;
		      endcase
		end
end
end
assign tisr = {31'h0, int_st};
assign tim_int = int_en & int_st;
endmodule
