module register(
    input  wire [31:0] addr,
    input  wire [31:0] tcr,
    input  wire [31:0] tier,
    input  wire [31:0] tisr,
    input  wire [31:0] tdr_0,
    input  wire [31:0] tdr_1,
    input  wire [31:0] tcmp0,
    input  wire [31:0] tcmp1,
    input  wire        rd_en,
    output wire [31:0] rdata
);

    // Multiplexer cho dữ liệu đọc
    reg [31:0] rdata_int;
    
    always @(*) begin
        if (rd_en) begin
            case (addr)
                32'h20000000:  rdata_int = tcr;
                32'h20000004:  rdata_int = tdr_0;
                32'h20000008:  rdata_int = tdr_1;
                32'h2000000C:  rdata_int = tcmp0;
                32'h20000010: rdata_int = tcmp1;
                32'h20000014: rdata_int = tier;
                32'h20000018: rdata_int = tisr;
                default: rdata_int = 32'h0;
            endcase
        end else begin
            rdata_int = 32'h0;
        end
    end
    
    assign rdata = rdata_int;
    
endmodule