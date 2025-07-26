module lsu (
    input  logic        i_clk,
    input  logic        i_reset,
    input  logic [31:0] i_addr,
    input  logic [31:0] i_wdata,
    input  logic [3:0]  i_bmask,
    input  logic        i_wren,
    input  logic [2:0]  i_funct3,        // Funct3 for memory access type
    input  logic [6:0]  i_opcode,        // NEW: Opcode to determine memory operations
    output logic [31:0] o_rdata,
    output logic        o_stall,         // Stall signal for pipeline

    // I/O Ports
    output logic [31:0] o_io_ledr,    // Red LEDs
    output logic [31:0] o_io_ledg,    // Green LEDs
    output logic [6:0]  o_io_hex0,    // 7-seg 0
    output logic [6:0]  o_io_hex1,    // 7-seg 1
    output logic [6:0]  o_io_hex2,    // 7-seg 2
    output logic [6:0]  o_io_hex3,    // 7-seg 3
    output logic [6:0]  o_io_hex4,    // 7-seg 4
    output logic [6:0]  o_io_hex5,    // 7-seg 5
    output logic [6:0]  o_io_hex6,    // 7-seg 6
    output logic [6:0]  o_io_hex7,    // 7-seg 7
    output logic [31:0] o_io_lcd,     // LCD Control
    input  logic [31:0] i_io_sw,      // Switches

    // APB Master Interface
    output logic [31:0] o_paddr,      // APB Address
    output logic        o_pwrite,     // APB Write enable
    output logic        o_psel,       // APB Select
    output logic        o_penable,    // APB Enable
    output logic [31:0] o_pwdata,     // APB Write data
    input  logic [31:0] i_prdata,     // APB Read data
    input  logic        i_pready      // APB Ready
);

    // Memory and I/O Registers
    logic [31:0] data_memory [0:511];  // 2KB RAM (512 words)
    logic [31:0] lcd_reg;              // LCD Control Register
    logic [31:0] ledr_reg;             // Red LEDs
    logic [31:0] ledg_reg;             // Green LEDs
    logic [6:0]  hex_reg [0:7];        // Individual 7-segment registers

    // APB State Machine
    typedef enum logic [1:0] {
        APB_IDLE    = 2'b00,
        APB_SETUP   = 2'b01,
        APB_ACCESS  = 2'b10
    } apb_state_t;
    
    apb_state_t apb_current_state, apb_next_state;

    // APB Stable Registers - FIXED: Proper declarations
    logic [31:0] apb_addr_stable;
    logic [31:0] apb_wdata_stable; 
    logic [31:0] apb_rdata_reg;
    logic apb_write_stable;
    logic [1:0] apb_addr_low_stable;

    // Address Decoding
    logic is_ram_access;       // RAM access
    logic is_ledr_access;      // Red LEDs access
    logic is_ledg_access;      // Green LEDs access
    logic is_hex0_3_access;    // HEX0-3 access
    logic is_hex4_7_access;    // HEX4-7 access
    logic is_lcd_access;       // LCD access
    logic is_sw_access;        // Switches access
    logic is_apb_access;       // APB peripheral access
    
    // Internal signals
    logic addr_misaligned;
    logic mem_req, mem_write, mem_read;   // CHANGED: Follow BIU style
    logic [1:0] addr_low;
    
    assign addr_low = i_addr[1:0];
    
    // CHANGED: Follow BIU logic for memory request detection with debug
    assign mem_write = (i_opcode == 7'b0100011);  // Store instructions
    assign mem_read = (i_opcode == 7'b0000011);   // Load instructions  
    assign mem_req = (mem_write | mem_read) && is_apb_access && (i_opcode !== 7'bxxxxxxx);
    
    // DEBUG: Check if load instruction is being detected properly
    // For debugging: lw instruction should have opcode = 7'b0000011

    // Address Decoding Logic
    always_comb begin
        // Default to internal I/O access if address is unknown
        is_ram_access    = 1'b0;
        is_ledr_access   = 1'b0;
        is_ledg_access   = 1'b0;
        is_hex0_3_access = 1'b0;
        is_hex4_7_access = 1'b0;
        is_lcd_access    = 1'b0;
        is_sw_access     = 1'b0;
        is_apb_access    = 1'b0;
        
        // Only decode if address is valid (not X)
        if (i_addr !== 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx) begin
            is_ram_access    = (i_addr[31:11] == 21'b0000_0000_0000_0000_0000_0);     // 0x0000_0000-0x0000_07FF
            is_ledr_access   = (i_addr[31:12] == 20'h10000);     // 0x1000_0000-0x1000_0FFF
            is_ledg_access   = (i_addr[31:12] == 20'h10001);     // 0x1000_1000-0x1000_1FFF
            is_hex0_3_access = (i_addr[31:12] == 20'h10002);     // 0x1000_2000-0x1000_2FFF
            is_hex4_7_access = (i_addr[31:12] == 20'h10003);     // 0x1000_3000-0x1000_3FFF
            is_lcd_access    = (i_addr[31:12] == 20'h10004);     // 0x1000_4000-0x1000_4FFF
            is_sw_access     = (i_addr[31:12] == 20'h10010);     // 0x1001_0000-0x1001_0FFF
            
            // APB access for all other addresses
            is_apb_access = !(is_ram_access || is_ledr_access || is_ledg_access || 
                             is_hex0_3_access || is_hex4_7_access || is_lcd_access || is_sw_access);
        end
        
        // Check for misaligned accesses
        addr_misaligned = (i_bmask[3] && i_addr[1:0] != 2'b00) ||  // LW/SW check
                          (i_bmask[2] && i_addr[0] != 1'b0);       // LH/LHU/SH check
    end

    // APB Write Data Processing
    logic [31:0] apb_wdata_processed;
    always_comb begin
        // Default to current write data
        apb_wdata_processed = i_wdata;
        
        // Only process if we have valid inputs
        if (i_funct3 !== 3'bxxx && i_addr[1:0] !== 2'bxx) begin
            case (i_funct3)
                3'b000: begin // SB - Store Byte
                    case (addr_low)
                        2'b00: apb_wdata_processed = {24'b0, i_wdata[7:0]};
                        2'b01: apb_wdata_processed = {16'b0, i_wdata[7:0], 8'b0};
                        2'b10: apb_wdata_processed = {8'b0, i_wdata[7:0], 16'b0};
                        2'b11: apb_wdata_processed = {i_wdata[7:0], 24'b0};
                    endcase
                end
                
                3'b001: begin // SH - Store Half-word
                    case (addr_low[1])
                        1'b0: apb_wdata_processed = {16'b0, i_wdata[15:0]};
                        1'b1: apb_wdata_processed = {i_wdata[15:0], 16'b0};
                    endcase
                end
                
                3'b010: begin // SW - Store Word
                    apb_wdata_processed = i_wdata;
                end
                
                default: apb_wdata_processed = i_wdata;
            endcase
        end
    end

    // APB Read Data Processing
    logic [31:0] apb_rdata_processed;
    always_comb begin
        // Default to registered read data
        apb_rdata_processed = apb_rdata_reg;
        
        // Only process if we have valid read data
        if (apb_rdata_reg !== 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx) begin
            case (i_funct3)
                3'b000: begin // LB - Load Byte (sign-extended)
                    case (apb_addr_low_stable)  // Use stable addr_low
                        2'b00: apb_rdata_processed = {{24{apb_rdata_reg[7]}}, apb_rdata_reg[7:0]};
                        2'b01: apb_rdata_processed = {{24{apb_rdata_reg[15]}}, apb_rdata_reg[15:8]};
                        2'b10: apb_rdata_processed = {{24{apb_rdata_reg[23]}}, apb_rdata_reg[23:16]};
                        2'b11: apb_rdata_processed = {{24{apb_rdata_reg[31]}}, apb_rdata_reg[31:24]};
                    endcase
                end
                
                3'b001: begin // LH - Load Half-word (sign-extended)
                    case (apb_addr_low_stable[1])  // Use stable addr_low
                        1'b0: apb_rdata_processed = {{16{apb_rdata_reg[15]}}, apb_rdata_reg[15:0]};
                        1'b1: apb_rdata_processed = {{16{apb_rdata_reg[31]}}, apb_rdata_reg[31:16]};
                    endcase
                end
                
                3'b010: begin // LW - Load Word
                    apb_rdata_processed = apb_rdata_reg;
                end
                
                3'b100: begin // LBU - Load Byte (zero-extended)
                    case (apb_addr_low_stable)  // Use stable addr_low
                        2'b00: apb_rdata_processed = {24'b0, apb_rdata_reg[7:0]};
                        2'b01: apb_rdata_processed = {24'b0, apb_rdata_reg[15:8]};
                        2'b10: apb_rdata_processed = {24'b0, apb_rdata_reg[23:16]};
                        2'b11: apb_rdata_processed = {24'b0, apb_rdata_reg[31:24]};
                    endcase
                end
                
                3'b101: begin // LHU - Load Half-word (zero-extended)
                    case (apb_addr_low_stable[1])  // Use stable addr_low
                        1'b0: apb_rdata_processed = {16'b0, apb_rdata_reg[15:0]};
                        1'b1: apb_rdata_processed = {16'b0, apb_rdata_reg[31:16]};
                    endcase
                end
                
                default: apb_rdata_processed = apb_rdata_reg;
            endcase
        end
    end

    // APB State Machine Implementation
    always_ff @(posedge i_clk) begin
        if (!i_reset) begin
            apb_current_state <= APB_IDLE;
            apb_rdata_reg <= 32'b0;
            apb_addr_stable <= 32'b0;
            apb_wdata_stable <= 32'b0;
            apb_write_stable <= 1'b0;
            apb_addr_low_stable <= 2'b0;
        end else begin
            apb_current_state <= apb_next_state;
            
            // Capture stable values when starting transaction
            if (apb_current_state == APB_IDLE && apb_next_state == APB_SETUP) begin
                apb_addr_stable <= {i_addr[31:2], 2'b00};
                apb_wdata_stable <= apb_wdata_processed;
                apb_write_stable <= mem_write;
                apb_addr_low_stable <= i_addr[1:0];
            end
            
            // Save read data when transaction completes
            if (apb_current_state == APB_ACCESS && i_pready && !apb_write_stable) begin
                apb_rdata_reg <= i_prdata;
            end
        end
    end

    // APB State Machine Logic
    always_comb begin
        apb_next_state = apb_current_state;
        
        case (apb_current_state)
            APB_IDLE: begin
                // FIXED: Only transition when we have valid APB request
                if (mem_req && !addr_misaligned)
                    apb_next_state = APB_SETUP;
            end
            
            APB_SETUP: begin
                apb_next_state = APB_ACCESS;
            end
            
            APB_ACCESS: begin
                if (i_pready)
                    apb_next_state = APB_IDLE;
            end
            
            default: apb_next_state = APB_IDLE;
        endcase
    end

    // APB Signals Generation - Use stable values during transaction
    always_comb begin
        // Default all signals to inactive
        o_psel = 1'b0;
        o_penable = 1'b0;
        o_pwrite = 1'b0;
        o_paddr = 32'b0;
        o_pwdata = 32'b0;
        
        case (apb_current_state)
            APB_SETUP: begin
                // In SETUP phase, PSEL is active but PENABLE is not
                o_psel = 1'b1;
                o_penable = 1'b0;
                o_pwrite = apb_write_stable;                 // Use stable value
                o_paddr = apb_addr_stable;                   // Use stable value
                o_pwdata = apb_wdata_stable;                 // Use stable value
            end
            
            APB_ACCESS: begin
                // In ACCESS phase, both PSEL and PENABLE are active
                o_psel = 1'b1;
                o_penable = 1'b1;
                o_pwrite = apb_write_stable;                 // Use stable value
                o_paddr = apb_addr_stable;                   // Use stable value
                o_pwdata = apb_wdata_stable;                 // Use stable value
            end
            
            default: begin // IDLE state
                o_psel = 1'b0;
                o_penable = 1'b0;
                o_pwrite = 1'b0;
                o_paddr = 32'b0;
                o_pwdata = 32'b0;
            end
        endcase
    end

    // Stall Logic for Pipeline
    // FIXED: Only stall when actually in APB transaction, not just APB access detection
    assign o_stall = (apb_current_state == APB_SETUP) || 
                     (apb_current_state == APB_ACCESS && !i_pready);

    // Read Logic - Simplified and Fixed
    always_comb begin
        o_rdata = 32'h0;
        
        if (!addr_misaligned) begin
            case (1'b1)
                is_ram_access: begin
                    // Only for non-write operations (!i_wren)
                    if (!i_wren) begin
                        case (i_bmask)
                            // lb
                            4'b0000: begin
                                case (i_addr[1:0])
                                    2'b00: o_rdata = {{24{data_memory[i_addr[10:2]][7]}},  data_memory[i_addr[10:2]][7:0]};
                                    2'b01: o_rdata = {{24{data_memory[i_addr[10:2]][15]}}, data_memory[i_addr[10:2]][15:8]};
                                    2'b10: o_rdata = {{24{data_memory[i_addr[10:2]][23]}}, data_memory[i_addr[10:2]][23:16]};
                                    2'b11: o_rdata = {{24{data_memory[i_addr[10:2]][31]}}, data_memory[i_addr[10:2]][31:24]};
                                endcase
                            end
                            // lbu
                            4'b0001: begin
                                case (i_addr[1:0])
                                    2'b00: o_rdata = {24'h0, data_memory[i_addr[10:2]][7:0]};
                                    2'b01: o_rdata = {24'h0, data_memory[i_addr[10:2]][15:8]};
                                    2'b10: o_rdata = {24'h0, data_memory[i_addr[10:2]][23:16]};
                                    2'b11: o_rdata = {24'h0, data_memory[i_addr[10:2]][31:24]};
                                endcase
                            end
                            // lh
                            4'b0010: begin
                                if (i_addr[1] == 1'b0)
                                    o_rdata = {{16{data_memory[i_addr[10:2]][15]}}, data_memory[i_addr[10:2]][15:0]};
                                else
                                    o_rdata = {{16{data_memory[i_addr[10:2]][31]}}, data_memory[i_addr[10:2]][31:16]};
                            end
                            // lhu
                            4'b0011: begin
                                if (i_addr[1] == 1'b0)
                                    o_rdata = {16'h0, data_memory[i_addr[10:2]][15:0]};
                                else
                                    o_rdata = {16'h0, data_memory[i_addr[10:2]][31:16]};
                            end
                            // lw
                            4'b0100: o_rdata = data_memory[i_addr[10:2]];
                            default: o_rdata = 32'h0;
                        endcase
                    end
                end
                
                is_ledr_access:   o_rdata = (!i_wren) ? ledr_reg : 32'h0;
                is_ledg_access:   o_rdata = (!i_wren) ? ledg_reg : 32'h0;
                is_hex0_3_access: o_rdata = (!i_wren) ? {1'b0, hex_reg[3], 1'b0, hex_reg[2], 1'b0, hex_reg[1], 1'b0, hex_reg[0]} : 32'h0;
                is_hex4_7_access: o_rdata = (!i_wren) ? {1'b0, hex_reg[7], 1'b0, hex_reg[6], 1'b0, hex_reg[5], 1'b0, hex_reg[4]} : 32'h0;
                is_lcd_access:    o_rdata = (!i_wren) ? lcd_reg : 32'h0;
                is_sw_access:     o_rdata = i_io_sw;  // Switches always readable
                
                is_apb_access: begin
                    // APB read: return processed data when transaction completed
                    if (!i_wren) begin
                        o_rdata = apb_rdata_processed;
                    end
                end
                
                default: o_rdata = 32'h0;
            endcase
        end
    end

    // Write Logic (Synchronous)
    always_ff @(posedge i_clk) begin
        if (!i_reset) begin
            // Reset all registers
            lcd_reg <= 0;
            ledr_reg <= 0;
            ledg_reg <= 0;
            for (int i = 0; i < 8; i++) begin
                hex_reg[i] <= 0;
            end
            
            // Initialize memory if needed  
            // $readmemh("mem.dump", data_memory);
        end
        else if (i_wren && !is_apb_access) begin // Don't write to local registers for APB accesses
            case (1'b1)
                is_ram_access: begin
                    case (i_bmask)
                        // sb - store byte
                        4'b1000: 
                            case (i_addr[1:0]) 
                                2'b00: data_memory[i_addr[10:2]][7:0] <= i_wdata[7:0];
                                2'b01: data_memory[i_addr[10:2]][15:8] <= i_wdata[7:0];
                                2'b10: data_memory[i_addr[10:2]][23:16] <= i_wdata[7:0];
                                2'b11: data_memory[i_addr[10:2]][31:24] <= i_wdata[7:0];
                            endcase
                        // sh - store half-word
                        4'b1001:
                            case (i_addr[1:0])
                                2'b00: data_memory[i_addr[10:2]][15:0] <= i_wdata[15:0];
                                2'b10: data_memory[i_addr[10:2]][31:16] <= i_wdata[15:0];
                                default: ;
                            endcase
                        
                        // sw - store word
                        4'b1010: data_memory[i_addr[10:2]] <= i_wdata;
                        default: ; // Ignore
                    endcase
                end
                is_ledr_access: begin
                    ledr_reg <= i_wdata;
                end
                is_ledg_access: begin
                    ledg_reg <= i_wdata;
                end
                is_hex0_3_access: begin
                    if (i_bmask[0]) hex_reg[0] <= i_wdata[6:0];
                    if (i_bmask[1]) hex_reg[1] <= i_wdata[14:8];
                    if (i_bmask[2]) hex_reg[2] <= i_wdata[22:16];
                    if (i_bmask[3]) hex_reg[3] <= i_wdata[30:24];
                end
                is_hex4_7_access: begin
                    if (i_bmask[0]) hex_reg[4] <= i_wdata[6:0];
                    if (i_bmask[1]) hex_reg[5] <= i_wdata[14:8];
                    if (i_bmask[2]) hex_reg[6] <= i_wdata[22:16];
                    if (i_bmask[3]) hex_reg[7] <= i_wdata[30:24];
                end
                is_lcd_access: begin
                    lcd_reg <= i_wdata;
                end
                default: ; // Ignore other writes
            endcase
        end
    end

    // Connect I/O Outputs
    assign o_io_ledr = ledr_reg;
    assign o_io_ledg = ledg_reg;
    assign o_io_lcd  = lcd_reg;
    assign o_io_hex0 = hex_reg[0];
    assign o_io_hex1 = hex_reg[1];
    assign o_io_hex2 = hex_reg[2];
    assign o_io_hex3 = hex_reg[3];
    assign o_io_hex4 = hex_reg[4];
    assign o_io_hex5 = hex_reg[5];
    assign o_io_hex6 = hex_reg[6];
    assign o_io_hex7 = hex_reg[7];
     
endmodule