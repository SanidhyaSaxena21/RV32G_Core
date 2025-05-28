`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/26/2025 12:40:28 PM
// Design Name: 
// Module Name: Custom2
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


module Custom2(
        input wire clk,
        input wire rst_n,
        
        input wire          wr_en,
        input wire [31:0]   wdata,
        
        output reg [31:0]   signals
    );
    always @(posedge clk) begin
        if(!rst_n)
            signals <= 32'd0;
        else if(wr_en)
            signals <= wdata;
    end
endmodule
