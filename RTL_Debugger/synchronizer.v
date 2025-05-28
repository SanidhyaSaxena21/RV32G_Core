`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/10/2025 04:11:57 PM
// Design Name: 
// Module Name: synchronizer
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


module synchronizer(
    input  wire clk,        // Destination clock domain
    input  wire async_in,   // Asynchronous input signal
    output wire sync_out    // Synchronized output
);

    reg sync_ff1 = 0;
    reg sync_ff2 = 0;

    always @(posedge clk) begin
        sync_ff1 <= async_in;   // First stage
        sync_ff2 <= sync_ff1;   // Second stage
    end

    assign sync_out = sync_ff2;

endmodule