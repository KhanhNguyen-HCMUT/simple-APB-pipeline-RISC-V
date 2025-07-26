module immgenn (
    input  logic [31:0] i_inst,  // Lệnh đầu vào (32-bit)
    output logic [31:0] o_imm    // Giá trị Immediate đã mở rộng
);
    logic [4:0] opcode;          // Trích xuất opcode từ lệnh
    assign opcode = i_inst[6:2];

    always_comb begin
        case (opcode)
            // I-Type: Các lệnh số học sử dụng Immediate (ADDI, ANDI, ORI, XORI, SLTI, SLTIU, SLLI, SRLI, SRAI)
            5'b00100: begin
                o_imm = {{21{i_inst[31]}}, i_inst[30:20]}; // mở rộng bit thứ 31 ra 20 lần và lưu giá trị từ bit 31 đến bit 20
            end

            // Load-Type: Các lệnh tải dữ liệu (LW, LH, LB, LBU, LHU)
            5'b00000: begin
                o_imm = {{21{i_inst[31]}}, i_inst[30:20]};
            end

            // Store-Type: Các lệnh lưu dữ liệu (SW, SH, SB)
            5'b01000: begin
                o_imm = {{21{i_inst[31]}}, i_inst[30:25], i_inst[11:7]};
            end

            // Branch-Type: Các lệnh rẽ nhánh (BEQ, BNE, BLT, BGE, BLTU, BGEU)
            5'b11000: begin
                o_imm = {{20{i_inst[31]}} , i_inst[7], i_inst[30:25], i_inst[11:8], 1'b0}; // Shift left 1
            end

            // Jump-Type: Lệnh nhảy không điều kiện (JAL)
            5'b11011: begin
                o_imm = {{12{i_inst[31]}}, i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0}; // Shift left 1
            end

            // Upper Immediate-Type: Lệnh nạp giá trị tức thời vào thanh ghi (LUI)
            5'b01101: begin
                o_imm = {i_inst[31:12], 12'b0}; // Không cần mở rộng dấu
            end

            // Upper Immediate-Type: Lệnh tính toán địa chỉ (AUIPC)
            5'b00101: begin
                o_imm = {i_inst[31:12], 12'b0}; // Không cần mở rộng dấu
            end

            // JALR-Type: Lệnh nhảy có điều kiện qua thanh ghi (JALR)
            5'b11001: begin
                o_imm = {{21{i_inst[31]}}, i_inst[30:20]};
            end

            // Mặc định: Không xác định opcode → Immediate = 0
            default: begin
                o_imm = 32'b0;
            end
        endcase
    end
endmodule
