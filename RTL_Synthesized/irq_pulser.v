`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/16/2025 03:13:56 AM
// Design Name: 
// Module Name: irq_pulser
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


module irq_pulser(
        input  wire        aclk,
        input  wire        areset,
        input  wire        intp,
        output reg      pulse
    );
    
    reg intp_reg;

    always @ (posedge aclk or posedge areset) begin
        if (areset) begin
            intp_reg <= 1'b0;
            pulse <= 1'b0;
        end else begin
            intp_reg <= intp;
            pulse <= intp & !intp_reg;
        end
    end
endmodule
