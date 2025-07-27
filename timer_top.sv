module timer_top(
input wire sys_clk,
input wire sys_rst_n,
input wire tim_psel,
input wire [31:0]tim_paddr,
input wire tim_penable,
input wire [31:0]tim_pwdata,
input wire tim_pwrite,
output wire [31:0]tim_prdata,
output wire tim_int,
output wire tim_pready,
output wire [63:0]cnt
);
wire cnt_en;
wire div_en;
wire [3:0]div_val;
wire timer_en;
wire wr_en;
wire rd_en;
wire [63:0]tcmp;
wire [31:0]tcr;
wire [31:0]tisr;
wire [31:0]tier;
wire [31:0]tdr_0;
wire [31:0]tdr_1;
wire [31:0]tcmp0;
wire [31:0]tcmp1;

APB_slave APB_slave(.clk(sys_clk), .rst_n(sys_rst_n), .psel(tim_psel), .penable(tim_penable), .pwrite(tim_pwrite), .wr_en(wr_en), .rd_en(rd_en), .pready(tim_pready));

tcr TCR(.clk(sys_clk), .rst_n(sys_rst_n), .addr(tim_paddr), .wr_en(wr_en), .wdata(tim_pwdata), .timer_en(timer_en), .div_en(div_en), .div_val(div_val), .tcr(tcr));

counter_ctrl counter_ctrl(.clk(sys_clk), .rst_n(sys_rst_n), .div_val(div_val), .div_en(div_en), .timer_en(timer_en), .cnt_en(cnt_en));

counter counter(.clk(sys_clk), .rst_n(sys_rst_n), .cnt_en(cnt_en), .addr(tim_paddr), .wr_en(wr_en), .wdata(tim_pwdata), .cnt(cnt), .tdr_1(tdr_1), .tdr_0(tdr_0));

interrupt interupt(.clk(sys_clk), .rst_n(sys_rst_n), .wdata(tim_pwdata), .addr(tim_paddr), .wr_en(wr_en), .cnt(cnt), .tcmp(tcmp), .tim_int(tim_int), .tier(tier), .tisr(tisr));

compare compare(.clk(sys_clk), .rst_n(sys_rst_n), .addr(tim_paddr), .wdata(tim_pwdata), .wr_en(wr_en), .tcmp(tcmp), .tcmp1(tcmp1), .tcmp0(tcmp0));

register register(.addr(tim_paddr), .tcr(tcr), .tisr(tisr), .tier(tier), .tdr_0(tdr_0), .tdr_1(tdr_1), .tcmp0(tcmp0), .tcmp1(tcmp1), .rdata(tim_prdata), .rd_en(rd_en));

endmodule
