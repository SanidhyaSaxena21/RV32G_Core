`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/10/2025 12:18:05 PM
// Design Name: 
// Module Name: Bypass_Reg
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


module Bypass_Reg(
        input wire rst_n,
        input wire Data_in,
        input wire Shift_DR,
        input wire Clock_DR,
        input wire TCK,
        input wire [3:0] TAP_state,
        
        output reg Data_out 
    );

    localparam CAPTURE_DR       = 4'd3;
           
    always @(negedge TCK)begin //changed-toms
        if(!rst_n)
            Data_out <= 0;
        else if(Clock_DR) begin
            if(TAP_state == CAPTURE_DR)
                Data_out <= 0;
            else
                Data_out <= (Shift_DR == 0) ? 0 : Data_in;
        end
    end
endmodule
