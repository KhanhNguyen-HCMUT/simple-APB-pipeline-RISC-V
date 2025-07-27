module top (
    // System inputs
    input logic i_clk,
    input logic i_reset,
    input logic [31:0] i_io_sw,

    // Outputs from RISC-V core
    output logic [31:0] o_io_lcd,
    output logic [31:0] o_io_ledg,
    output logic [31:0] o_io_ledr,
    output logic [6:0] o_io_hex0,
    output logic [6:0] o_io_hex1,
    output logic [6:0] o_io_hex2,
    output logic [6:0] o_io_hex3,
    output logic [6:0] o_io_hex4,
    output logic [6:0] o_io_hex5,
    output logic [6:0] o_io_hex6,
    output logic [6:0] o_io_hex7,
    
    // Debug outputs
    output logic [31:0] o_pc_debug,
    output logic o_insn_vld,
    
    // Timer outputs
    output logic tim_int,
    output logic [63:0] cnt
);

    // APB interface signals between core and timer
    logic [31:0] paddr;
    logic pwrite;
    logic psel;
    logic penable;
    logic [31:0] pwdata;
    logic [31:0] prdata;
    logic pready;
    
    // Timer uses active-low reset directly
    logic sys_rst_n;
    assign sys_rst_n = i_reset;

    // RISC-V Pipelined Core
    pipelined riscv_core (
        // System signals
        .i_clk(i_clk),
        .i_reset(i_reset),
        .i_io_sw(i_io_sw),
        
        // APB Master Interface (to timer)
        .o_paddr(paddr),
        .o_pwrite(pwrite),
        .o_psel(psel),
        .o_penable(penable),
        .o_pwdata(pwdata),
        .i_prdata(prdata),
        .i_pready(pready),
        
        // I/O outputs
        .o_io_lcd(o_io_lcd),
        .o_io_ledg(o_io_ledg),
        .o_io_ledr(o_io_ledr),
        .o_io_hex0(o_io_hex0),
        .o_io_hex1(o_io_hex1),
        .o_io_hex2(o_io_hex2),
        .o_io_hex3(o_io_hex3),
        .o_io_hex4(o_io_hex4),
        .o_io_hex5(o_io_hex5),
        .o_io_hex6(o_io_hex6),
        .o_io_hex7(o_io_hex7),
        
        // Debug outputs
        .o_pc_debug(o_pc_debug),
        .o_insn_vld(o_insn_vld)
    );

    // Timer Module
    timer_top timer (
        // System signals
        .sys_clk(i_clk),
        .sys_rst_n(sys_rst_n),
        
        // APB Slave Interface (from core)
        .tim_psel(psel),
        .tim_paddr(paddr),
        .tim_penable(penable),
        .tim_pwdata(pwdata),
        .tim_pwrite(pwrite),
        .tim_prdata(prdata),
        .tim_pready(pready),
        
        // Timer outputs
        .tim_int(tim_int),
        .cnt(cnt)
    );

endmodule