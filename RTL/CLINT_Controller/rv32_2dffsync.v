`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/05/2025 02:24:48 AM
// Design Name: 
// Module Name: rv32_2dffsync
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


module rv32_2dffsync (
    input clk,
    input rst,
    input bit_i,
    output bit_o

    );

    reg sync;
    reg meta;

    always @(posedge clk or posedge rst) begin
      if(rst) begin
        meta <= 1'b0;
        sync <= 1'b0;
      end
      else begin
        meta <= bit_i;
        sync <= meta;
      end
    end

    assign bit_o = sync;
endmodule
