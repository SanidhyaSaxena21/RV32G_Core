`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/05/2025 01:51:11 AM
// Design Name: 
// Module Name: rv32_clint
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// CLINT controller (Core Local Interrupt Controller), implementing next CSRs:
//
//   - MTIME / MTIMECMP (machine time registers)
//   - a platform specific MSIP output to interrupt another hart
//
// Registers mapping:
//
// - 0x00 - 0x03:   MSIP
// - 0x04 - 0x0B:   MTIME
// - 0x0C - 0x13:   MTIMECMP
//
///////////////////////////////////////////////////////////////////////////////


module rv32_clint

    #(
        // APB address width
        parameter ADDRW = 16,
        // Architecture setup
        parameter XLEN = 32
    )(
        // clock & reset
        input  wire                   CLK,
        input  wire                   RST,
        // APB slave interface
        input  wire                   s_en,
        input  wire                   s_wr,
        input  wire  [ADDRW     -1:0] s_addr,
        input  wire  [XLEN      -1:0] s_wdata,
        input  wire  [XLEN/8    -1:0] s_strb,
        output logic [XLEN      -1:0] s_rdata,
        output logic                  s_ready,
        // real-time clock, shared across the harts
        input  wire                   rtc,
        // software interrupt 
        output logic                  sw_irq,
        // timer interrupt 
        output logic                  timer_irq
    );

    logic [64   -1:0] mtime;
    logic [64   -1:0] mtimecmp;
    logic             rtc_sync;
    logic             mtime_en;

    assign mtime_en = |mtimecmp;

    // Synchronize the real-time clock tick into the peripheral clock domain
    rv32_2dffsync rtc_synchronizer
    (
        .clk    (CLK),
        .rst    (RST),
        .bit_i   (rtc),
        .bit_o   (rtc_sync)
    );


    // READY assertion
    always @(posedge CLK or posedge RST) begin
      if(RST) s_ready <= 1'b0;
      else begin
        if(s_en && ~s_ready) s_ready <= 1'b1;
        else s_ready <= 1'b0;
      end
    end

    always @ (posedge CLK or posedge RST) begin

        if (RST) begin
            sw_irq <= 1'b0;
            timer_irq <= 1'b0;
            s_rdata <= {XLEN{1'b0}};
            mtime <= 64'b0;
            mtimecmp <= 64'b0;
        end else begin


            // Registers Access
            if (s_en) begin

                // MSIP register
                if (s_addr[4:0]=={5'b0}) begin

                    if (s_wr) begin
                        if (s_strb[0]) sw_irq <= s_wdata[0];
                    end

                    s_rdata <= {31'h0, sw_irq};

                // MTIME Register, 32 bits LSB
                end else if (s_addr[4:0]=={1'b0,4'h4}) begin

                    if (s_wr) begin
                        if (s_strb[0]) mtime[ 0+:8] <= s_wdata[ 0+:8];
                        if (s_strb[1]) mtime[ 8+:8] <= s_wdata[ 8+:8];
                        if (s_strb[2]) mtime[16+:8] <= s_wdata[16+:8];
                        if (s_strb[3]) mtime[24+:8] <= s_wdata[24+:8];
                    end

                    s_rdata <= mtime[0+:32];

                // MTIME Register, 32 bits MSB
                end else if (s_addr[4:0]=={1'b0,4'h8}) begin

                    if (s_wr) begin
                        if (s_strb[0]) mtime[32+:8] <= s_wdata[ 0+:8];
                        if (s_strb[1]) mtime[40+:8] <= s_wdata[ 8+:8];
                        if (s_strb[2]) mtime[48+:8] <= s_wdata[16+:8];
                        if (s_strb[3]) mtime[56+:8] <= s_wdata[24+:8];
                    end

                    s_rdata <= mtime[32+:32];

                // MTIMECMP Register, 32 bits LSB
                end else if (s_addr[4:0]=={1'b0,4'hC}) begin

                    if (s_wr) begin
                        if (s_strb[0]) mtimecmp[ 0+:8] <= s_wdata[ 0+:8];
                        if (s_strb[1]) mtimecmp[ 8+:8] <= s_wdata[ 8+:8];
                        if (s_strb[2]) mtimecmp[16+:8] <= s_wdata[16+:8];
                        if (s_strb[3]) mtimecmp[24+:8] <= s_wdata[24+:8];
                    end 

                    s_rdata <= mtimecmp[0+:32];

                // MTIMECMP Register, 32 bits MSB
                end else if (s_addr[4:0]=={5'h10}) begin

                    if (s_wr) begin
                        if (s_strb[0]) mtimecmp[32+:8] <= s_wdata[ 0+:8];
                        if (s_strb[1]) mtimecmp[40+:8] <= s_wdata[ 8+:8];
                        if (s_strb[2]) mtimecmp[48+:8] <= s_wdata[16+:8];
                        if (s_strb[3]) mtimecmp[56+:8] <= s_wdata[24+:8];
                    end

                    s_rdata <= mtimecmp[32+:32];

                end

            // Execute the timer and its comparator
            end else begin

                if (mtime_en) begin
                    if (rtc_sync)
                        mtime <= mtime + 1;
                end else begin
                    mtime <= 64'b0;
                end

                if (mtime_en) begin
                    if (mtime >= mtimecmp)
                        timer_irq <= 1'b1;
                    else
                        timer_irq <= 1'b0;
                end else begin
                    timer_irq <= 1'b0;
                end
            end
        end
    end
    

endmodule

