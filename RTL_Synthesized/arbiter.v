//Designed By: Sanidhya Saxena
//Project: 32-bit RISCV Design 
//Guide: Kuruvilla Varghese

module arbiter #(
    parameter BASE0_ADDR = 32'h2000_0000,
    parameter BASE0_MASK = 32'h0000_FFFF,  // 4KB range
    parameter BASE1_ADDR = 32'h5F00_0000,
    parameter BASE1_MASK = 32'h0000_0FFF,  // 4KB range
    parameter BASE2_ADDR = 32'h5C00_0000,
    parameter BASE2_MASK = 32'h0000_0FFF,  // 4KB range
    parameter BASE3_ADDR = 32'h5D00_0000,
    parameter BASE3_MASK = 32'h0000_0FFF   // 4KB range
)(
    input  [31:0] m_addr,
    output reg    s0_grant,
    output reg    s1_grant,
    output reg    s2_grant,
    output reg    s3_grant
);

    always @(*) begin
        // Default: no grant
        s0_grant = 0;
        s1_grant = 0;
        s2_grant = 0;
        s3_grant = 0;

        // Priority-based routing: use ~MASK to zero out lower bits
        if ((m_addr & ~BASE0_MASK) == BASE0_ADDR)
            s0_grant = 1;
        else if ((m_addr & ~BASE1_MASK) == BASE1_ADDR)
            s1_grant = 1;
        else if ((m_addr & ~BASE2_MASK) == BASE2_ADDR)
            s2_grant = 1;
        else if ((m_addr & ~BASE3_MASK) == BASE3_ADDR)
            s3_grant = 1;
    end

endmodule

