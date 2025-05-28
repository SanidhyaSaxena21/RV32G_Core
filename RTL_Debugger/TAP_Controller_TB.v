`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/15/2025 03:02:05 PM
// Design Name: 
// Module Name: TAP_Controller_TB
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


module TAP_Controller_TB(

    );
    
    reg TCK, TMS;
    wire [3:0] state;
    
    // Instantiate the TAP Controller
    TAP_Controller uut (
        .TCK(TCK),
        .TMS(TMS),
        .state(state)
    );
    
    // Clock generation
    initial begin
        TCK = 0;
        forever #5 TCK = ~TCK; // 10ns clock period (100MHz)
    end
    
    // Test sequence
    initial begin
        // Initialize signals
        TMS = 1;
        
        #100;
        // Hold reset state for at least 5 cycles
        repeat (5) begin
            TMS = 1; // Move to Test-Logic-Reset
            #10;
        end

        // Move to Shift-DR state
        TMS = 0; #10; // Run-Test/Idle
        TMS = 1; #10; // Select-DR-Scan
        TMS = 0; #10; // Capture-DR
        TMS = 0; #10; // Shift-DR
    
        #300;
        $stop;
    end
endmodule
