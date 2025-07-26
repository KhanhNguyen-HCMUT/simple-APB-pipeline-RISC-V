module regfile (
    input  logic        i_clk,
    input  logic        i_reset,
    input  logic [4:0]  i_rs1_addr,  // Địa chỉ thanh ghi nguồn 1
    input  logic [4:0]  i_rs2_addr,  // Địa chỉ thanh ghi nguồn 2  
    input  logic [4:0]  i_rd_addr,   // Địa chỉ thanh ghi đích
    input  logic [31:0] i_rd_data,   // Dữ liệu ghi vào thanh ghi đích
    input  logic        i_rd_wren,   // Cho phép ghi
	 output logic [31:0] o_rs1_data,  // Dữ liệu từ thanh ghi nguồn 1
    output logic [31:0] o_rs2_data  // Dữ liệu từ thanh ghi nguồn 2
);

    // Khai báo mảng thanh ghi 32-bit x 32 registers
    logic [31:0] registers [31:0];

    // Đọc không đồng bộ
    assign o_rs1_data = (i_rs1_addr == 5'd0) ? 32'd0 : registers[i_rs1_addr];
    assign o_rs2_data = (i_rs2_addr == 5'd0) ? 32'd0 : registers[i_rs2_addr];

    // Ghi đồng bộ với clock
    always_ff @(posedge i_clk) begin
        if (!i_reset) begin
            // Đặt tất cả thanh ghi về 0 khi reset
            for (int i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'd0;
            end
        end else if (i_rd_wren && i_rd_addr != 5'd0) begin
            registers[i_rd_addr] <= i_rd_data; // Không ghi vào register 0
        end
    end

endmodule
