`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/01/2025 07:53:10 PM
// Design Name: 
// Module Name: dual_port_RAM
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


module dual_port_RAM #(
    parameter DATA_WIDTH = 56,  // Data width
    parameter ADDR_WIDTH = 7,    // Address width (BTB_Read_Addr[8:2] -> 7 bits)
    parameter BYTE_EN = (DATA_WIDTH/8)
)(
    input wire clka,            // Clock for Port A
    input wire rsta,            // Reset for Port A
    input wire [BYTE_EN-1:0] wea,       // Write enable for Port A (7-bit wide for possible multi-bit writes)
    input wire [ADDR_WIDTH-1:0] addra, // Address for Port A
    input wire [DATA_WIDTH-1:0] dina,  // Data input for Port A
    output reg [DATA_WIDTH-1:0] douta, // Data output for Port A
    input wire ena,             // Enable signal for Port A

    input wire clkb,            // Clock for Port B
    input wire rstb,            // Reset for Port B
    input wire [6:0] web,       // Write enable for Port B (7-bit wide)
    input wire [ADDR_WIDTH-1:0] addrb, // Address for Port B
    input wire [DATA_WIDTH-1:0] dinb,  // Data input for Port B
    output reg [DATA_WIDTH-1:0] doutb  // Data output for Port B (unused in instantiation)

);

    // True Dual-Port Memory Array
    reg [DATA_WIDTH-1:0] ram [0:(1 << ADDR_WIDTH) - 1];
    integer i;

    // Port A - Read/Write Operations
    always @(posedge clka) begin
        if (rsta)
            douta <= {DATA_WIDTH{1'b0}};  // Reset data output
        else if (ena) begin
            if (|wea) begin  // Write operation if write enable is active
                for (i=0;i<7;i=i+1) begin
                  //if(wea[i]) ram[addra][(8*(i+1)-1):8*i] <= dina[(8*(i+1)-1):8*i];
                  if(wea[i]) ram[addra][8*i+:8] <= dina[8*i+:8];
                end
            end
            else begin
              douta <= ram[addra];  // Synchronous read (1-cycle latency)
            end
        end
    end

    integer j;
    // Port B - Read/Write Operation
    always @(posedge clkb) begin
        if (rstb)
            doutb <= {DATA_WIDTH{1'b0}};  // Reset data output
        else if (ena) begin
            if (|web) begin  // Write operation if write enable is active
                for (j=0;j<BYTE_EN;j=j+1) begin
                  //if(web[j]) ram[addrb][(8*(j+1)-1):8*j] <= dinb[(8*(j+1)-1):8*j];
                  if(web[j]) ram[addrb][8*j+:8] <= dinb[8*j+:8];
                end
            end
            else begin
              doutb <= ram[addrb];  // Synchronous read (1-cycle latency)
            end
        end
    end

endmodule

