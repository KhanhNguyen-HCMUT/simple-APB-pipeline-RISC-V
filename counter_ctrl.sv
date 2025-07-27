module counter_ctrl(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [3:0] div_val,
    input  wire       div_en,
    input  wire       timer_en,
    output wire       cnt_en
);
    reg [7:0] int_cnt;
    reg [7:0] limit;
    
    // Tạo giá trị limit từ div_val (2^div_val - 1)
    always @(*) begin
        case(div_val)
            4'h0:    limit = 8'h0;
            4'h1:    limit = 8'h1;
            4'h2:    limit = 8'h3;
            4'h3:    limit = 8'h7;
            4'h4:    limit = 8'hf;
            4'h5:    limit = 8'h1f;
            4'h6:    limit = 8'h3f;
            4'h7:    limit = 8'h7f;
            4'h8:    limit = 8'hff;
            default: limit = 8'h0;  // Mặc định về 0 thay vì giữ nguyên
        endcase
    end
    
    // Điều kiện reset bộ đếm nội
    wire cnt_rst = !div_en || !timer_en || (int_cnt == limit);
    
    // Cập nhật bộ đếm nội
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            int_cnt <= 8'h0;
        else if (cnt_rst)
            int_cnt <= 8'h0;
        else if (timer_en && div_en && div_val != 0)
            int_cnt <= int_cnt + 8'h1;
    end
    
    // Điều kiện kích hoạt bộ đếm chính (đơn giản hóa logic)
    assign cnt_en = timer_en && (
                      !div_en || 
                      (div_val == 0 && div_en) || 
                      (div_val != 0 && div_en && int_cnt == limit)
                    );
    
endmodule