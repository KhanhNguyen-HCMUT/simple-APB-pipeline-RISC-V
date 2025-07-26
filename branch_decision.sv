module branch_decision (
    input logic [2:0] fun3,           // Từ lệnh qua ID/EX register
    input logic branch,               // Từ Control Unit qua ID/EX register
    input logic br_equal, br_less,    // Từ BRC
    output logic branch_taken                // Tín hiệu flush
);

    always_comb begin
        // Quyết định nhảy dựa trên fun3 và kết quả so sánh
		 if (branch) begin
        case (fun3)
				  3'b000: branch_taken = br_equal;        // BEQ so sánh bằng có dấu
				  3'b001: branch_taken = !br_equal;       // BNE so sánh không bằng có dấu
				  3'b100: branch_taken = br_less;         // BLT nhảy nếu rs1 < rs2 có dấu
				  3'b101: branch_taken = !br_less;        // BGE nhảy nếu rs >= rs2 có dấu
				  3'b110: branch_taken = br_less;         // BLTU nhảy nếu rs1 < rs2 ko dấu
				  3'b111: branch_taken = !br_less;        // BGEU nhảy nếu rs1 >= rs2 ko dấu
            default: branch_taken = 1'b0;
        endcase
		end
		else branch_taken = 1'b0;
	end
endmodule

