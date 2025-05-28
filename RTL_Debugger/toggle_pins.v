`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/08/2025 12:05:51 PM
// Design Name: 
// Module Name: toggle_pins
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


module toggle_pins (
    input  wire SYSCLK_P,
    input  wire SYSCLK_N,
//    input wire clk,           // Input clock
//    output wire pin0,         // Output pin 0
//    output wire pin1,         // Output pin 1
//    output wire pin2,         // Output pin 2
//    output wire pin3          // Output pin 3
    
    output reg TDI,         // Output pin 0
    output reg TD0,         // Output pin 1
    output reg TMS,         // Output pin 2
    output reg TCK          // Output pin 3
);

    wire clk;

   clk_wiz_0 clk_gen
   (
    // Clock out ports
    .clk_out1(clk),     // output clk_out1
   // Clock in ports
    .clk_in1_p(SYSCLK_P),    // input clk_in1_p
    .clk_in1_n(SYSCLK_N)    // input clk_in1_n
    );
    
    // Clock dividers for different toggle rates
    reg [8:0] counter; // For TDI, divide clock by 2^24

    // Initialize output pins to a known state
    initial begin
        counter=0;
        TDI = 0;
        TD0 = 0;
        TMS = 0;
        TCK = 0;
    end

    // Clock divider for TDI (toggle at a slower rate)
    always @(posedge clk) begin
        counter <= counter + 1;
        
    end
    
    always @(*) begin
        {TDI, TD0, TMS, TCK} <= counter[6:3];
    end

endmodule