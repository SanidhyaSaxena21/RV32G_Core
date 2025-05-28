`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/16/2025 10:05:13 PM
// Design Name: 
// Module Name: Custom11
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


module Custom8(
        input wire clk,
        input wire rst_n,
        
        input wire [31:0]   wdata,
        
        output reg [31:0]   abstractcs_probe
    );
    always @(posedge clk) begin
        if(!rst_n)
            abstractcs_probe <= 32'd0;
        else
            abstractcs_probe <= wdata;
    end
endmodule