module memory (
    input  logic        i_clk,
    input  logic        i_reset,
    input  logic [31:0] i_addr,
    input  logic [31:0] i_wdata,
    input  logic [3:0]  i_bmask,
    input  logic        i_wren,
    output logic [31:0] o_rdata
);

    parameter MEM_SIZE = 4096;
    logic [31:0] mem [MEM_SIZE-1:0];

    // Đọc file dữ liệu/mã lệnh
    initial begin
        $readmemh("D:/Study/Thesis/RISCV_Pipelined-main/RISCV_Pipelined-main/pl-test-fw/isa.mem", mem);
    end

    // Đọc không đồng bộ
    assign o_rdata = mem[i_addr[13:2]];  // Căn chỉnh theo từ (word-aligned)

    // Ghi đồng bộ với byte mask
    always_ff @(posedge i_clk or posedge i_reset) begin
        if (i_reset) begin
            for (int i = 0; i < 1024; i = i + 1)
                mem[i] <= 32'b0;
        end else if (i_wren) begin
            if (i_bmask[0]) mem[i_addr[13:2]][7:0]   <= i_wdata[7:0];
            if (i_bmask[1]) mem[i_addr[13:2]][15:8]  <= i_wdata[15:8];
            if (i_bmask[2]) mem[i_addr[13:2]][23:16] <= i_wdata[23:16];
            if (i_bmask[3]) mem[i_addr[13:2]][31:24] <= i_wdata[31:24];
        end
    end

endmodule
