`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/26/2025 04:39:50 PM
// Design Name: 
// Module Name: Custom6
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


module Custom6(
        input wire clk,
        input wire rst_n,
        
        input wire [31:0]   wdata,
        
        output reg [31:0]   p_other_signals
    );
    always @(posedge clk) begin
        if(!rst_n)
            p_other_signals <= 32'd0;
        else
            p_other_signals <= wdata;
    end
endmodule
