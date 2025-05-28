`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/26/2025 04:39:36 PM
// Design Name: 
// Module Name: Custom5
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


module Custom5(
        input wire clk,
        input wire rst_n,
        
        input wire [31:0]   wdata,
        
        output reg [31:0]   p_ptr_data1_data
    );
    always @(posedge clk) begin
        if(!rst_n)
            p_ptr_data1_data <= 32'd0;
        else
            p_ptr_data1_data <= wdata;
    end
endmodule
