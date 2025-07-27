module counter(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cnt_en,
    input  wire [31:0] addr,
    input  wire        wr_en,
    input  wire [31:0] wdata,
    output wire [63:0] cnt,
    output wire [31:0] tdr_0,
    output wire [31:0] tdr_1
);
    reg [31:0] tdr_0_r;
    reg [31:0] tdr_1_r;
    
    // Tạo tín hiệu chọn thanh ghi để ghi
    wire tdr0_wr_sel = wr_en & (addr == 32'h20000004);
    wire tdr1_wr_sel = wr_en & (addr == 32'h20000008);
    
    // Cập nhật giá trị bộ đếm
    wire [63:0] cnt_next = cnt + 64'h1;
    
    // Cập nhật tdr_0_r
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tdr_0_r <= 32'h0;
        else if (tdr0_wr_sel)
            tdr_0_r <= wdata;
        else if (cnt_en)
            tdr_0_r <= cnt_next[31:0];
    end
    
    // Cập nhật tdr_1_r
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tdr_1_r <= 32'h0;
        else if (tdr1_wr_sel)
            tdr_1_r <= wdata;
        else if (cnt_en)
            tdr_1_r <= cnt_next[63:32];
    end
    
    // Kết nối đầu ra
    assign cnt = {tdr_1_r, tdr_0_r};
    assign tdr_0 = tdr_0_r;
    assign tdr_1 = tdr_1_r;
    
endmodule