module controlunit (
    // Tin hieu ngo vao
    input logic [31:0] i_instr,
    //input logic br_less, br_equal, // Không sử dụng trực tiếp ở đây nữa
    
    // Tin hieu ngo ra
    output logic pc_sel, rd_wren, br_un, opa_sel, opb_sel, mem_wren, inst_vld,
    output logic [3:0] alu_op,
    output logic [1:0] wb_sel,
    output logic [3:0] lsu_op,
    output logic branch,  // Update
    output logic [2:0] funct3  // FIXED: Changed from fun3 to funct3
);
    // Khai bao cac tin hieu can
    logic [4:0] opcode;
    logic fun7;
    logic br_unte;

    // Gan cac tin hieu dua tren ma may
    always_comb begin 
        opcode = i_instr[6:2];
        funct3 = i_instr[14:12]; // FIXED: Gán trực tiếp cho funct3 (đầu ra)
        fun7   = i_instr[30];
        br_unte = i_instr[13];
    end

    // Gan cac gia tri ban dau cho cac ngo ra
    always_comb begin
        // Gán giá trị mặc định cho tất cả tín hiệu đầu ra
        pc_sel    = 1'b0;    // PC = PC + 4 (quyết định ở EX)
        br_un     = 1'b0;    // So sanh co dau mac dinh
        rd_wren   = 1'b0;    // Read
        opa_sel   = 1'b0;    // ALU nhan rs1
        opb_sel   = 1'b1;    // ALU nhan imm
        mem_wren  = 1'b0;    // Read ALU (load)
        alu_op    = 4'b0000; // Add
        wb_sel    = 2'b00;   // ALU_data
        lsu_op    = 4'b0100; // LW
        inst_vld  = 1'b1;
        branch    = 1'b0;    // Mặc định không phải lệnh nhánh
        // funct3 đã được gán trực tiếp ở trên, không cần gán lại ở đây
	inst_vld = (i_instr == 32'h00000013) ? 1'b0 : 1'b1;

        // Xử lý dựa trên opcode
        case (opcode)
				// R_Format
				5'b01100: begin
					pc_sel  = 1'b0;
					rd_wren = 1'b1;
					opa_sel = 1'b0;
					opb_sel = 1'b0; //rs2
					wb_sel  = 2'b00; //alu_data
					inst_vld = 1'b0;
								case  (funct3)  // FIXED: Use funct3 instead of fun3
										3'b000: begin
															case  (fun7)
																1'b0: alu_op = 4'b0000; //lenh ADD
																1'b1: alu_op = 4'b0001; //lenh SUB
															endcase
													end
										3'b001: alu_op = 4'b0111; //lenh SLL
										3'b010: alu_op = 4'b0010; //lenh SLT
										3'b011: alu_op = 4'b0011; //lenh SLTU
										3'b100: alu_op = 4'b0100; //lenh XOR
										3'b101: begin
															case (fun7)
																1'b0: alu_op = 4'b1000; //lenh SRL
																1'b1: alu_op = 4'b1001; //lenh SRA
															endcase
													end
										3'b110: alu_op = 4'b0101; //lenh OR
										3'b111: alu_op = 4'b0110; //lenh AND
								endcase													
						end
				// I_Format tinh toan
				5'b00100: begin
					pc_sel  = 1'b0;
					rd_wren = 1'b1;
					opa_sel = 1'b0;
					opb_sel = 1'b1; //imm
					wb_sel  = 2'b00; //alu_data
					inst_vld = 1'b0;
									case (funct3)  // FIXED: Use funct3 instead of fun3
											3'b000: alu_op = 4'b0000; // lenh ADDI
											3'b001: alu_op = 4'b0111; // lenh SLLI
											3'b010: alu_op = 4'b0010; // lenh SLTI
											3'b011: alu_op = 4'b0011; // lenh SLTIU
											3'b100: alu_op = 4'b0100; // lenh XORI
											3'b101: begin
															case (fun7)
																1'b0: alu_op = 4'b1000; // lenh SRLI
																1'b1: alu_op = 4'b1001; // lenh SRAI
															endcase
														end
											3'b110: alu_op = 4'b0101; // lenh ORI
											3'b111: alu_op = 4'b0110; // lenh ANDI
									endcase
							end
            // B-Format (khong quyet dinh pc_sel o day)
            5'b11000: begin
                opa_sel   = 1'b1;
                opb_sel   = 1'b1; // imm
                rd_wren   = 1'b0;
                mem_wren  = 1'b0;
                inst_vld  = 1'b0;
                branch    = 1'b1;  // Là lệnh nhánh
                // Xac dinh br_un
                case (br_unte)
                    1'b0: br_un = 1'b1; // Unsigned
                    1'b1: br_un = 1'b0; // Signed
                endcase
                // Khong gan pc_sel tai day, de EX xu ly
            end
				// S_Format lenh Store ghi du lieu vao LSU
				5'b01000: begin
					pc_sel   = 1'b0;
					rd_wren  = 1'b0;
					opa_sel  = 1'b0;
					opb_sel  = 1'b1; //imm
					mem_wren = 1'b1;
					wb_sel   = 2'b01; //lsu
					inst_vld = 1'b0;
					case (funct3)  // FIXED: Use funct3 instead of fun3
							3'b000: lsu_op = 4'b1000; //SB
							3'b001: lsu_op = 4'b1001; //SH
							3'b010: lsu_op = 4'b1010; //SW
							default: begin
										end
					endcase
							 end
				// I_Format lenh Load doc du leu tu LSU
				5'b00000: begin
					pc_sel   = 1'b0;
					rd_wren  = 1'b1;
					opa_sel  = 1'b0;
					opb_sel  = 1'b1; //imm
					mem_wren = 1'b0;
					wb_sel   = 2'b01; //lsu	
					inst_vld = 1'b0;
					case (funct3)  // FIXED: Use funct3 instead of fun3
							3'b000: lsu_op = 4'b0000; //LB
							3'b001: lsu_op = 4'b0010; //LH
							3'b010: lsu_op = 4'b0100; //LW
							3'b100: lsu_op = 4'b0001; //LBU
							3'b101: lsu_op = 4'b0011; //LHU
							default: begin
										end
					endcase
							 end			
				//U_Format lenh nhay khong dieu kien
				5'b11011: begin   //lenh JAL
					pc_sel   = 1'b1;
					rd_wren  = 1'b1;
					opa_sel  = 1'b1;
					opb_sel  = 1'b1; //imm
					mem_wren = 1'b0;
					wb_sel   = 2'b10; //pc+4
					inst_vld = 1'b0;
							 end
				5'b11001: begin 	//lenh JALR
					pc_sel   = 1'b1;
					rd_wren  = 1'b1;
					opa_sel  = 1'b0;
					opb_sel  = 1'b1; //imm
					mem_wren = 1'b0;
					wb_sel   = 2'b10; //pc+4
					inst_vld = 1'b0;
							end
				//I_Format 
				//lenh LUI
				5'b01101: begin 
					pc_sel   = 1'b0;
					rd_wren  = 1'b1;
					mem_wren = 1'b0;
					opb_sel  = 1'b1;
					wb_sel   = 2'b11;  // sửa ở trong hình anh Hải cho thêm phần nối từ immgen đến mux 4 sang 1
					inst_vld = 1'b0;
						end
				//lenh AUIPC
				5'b00101: begin
					pc_sel   = 1'b0;
					rd_wren  = 1'b1;
					opa_sel  = 1'b1;
					opb_sel  = 1'b1; //imm
					mem_wren = 1'b0;
					wb_sel   = 2'b00; //alu_data
					inst_vld = 1'b0;
						end
				default: begin
				end
			endcase
		end
endmodule