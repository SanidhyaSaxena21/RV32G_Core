`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/26/2025 12:40:38 PM
// Design Name: 
// Module Name: Custom3
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


module Custom3(
        input wire clk,
        input wire rst_n,
        
        input wire          wr_en,
        input wire [31:0]   wdata,
        
        output reg [31:0]   mux_lock
    );
    always @(posedge clk) begin
        if(!rst_n)
            mux_lock <= 32'd0;
        else if(wr_en)
            mux_lock <= wdata;
    end
endmodule
