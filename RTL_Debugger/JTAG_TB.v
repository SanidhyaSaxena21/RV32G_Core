`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/15/2025 02:30:12 PM
// Design Name: 
// Module Name: JTAG_TB
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


module JTAG_TB(

    );
    
    // Testbench signals
    reg TDI, TMS, TCK;
    wire TDO;
    wire [3:0] TAP_State;
    wire rst_out;
    wire [5:0] Instr;
    
    // Instantiate the JTAG module
    FTDI_connections uut (
        .TDO(TDO),
        .TDI(TDI),
        .TMS(TMS),
        .TCK(TCK),
        .state_out(TAP_State),
        .rst_out(rst_out),
        .probing_out(Instr)
    );
    
    // Clock generation
    always begin
        #5 TCK = 0; // 10ns clock period (100MHz)
        #5 TCK = 1; // 10ns clock period (100MHz)
    end
    
    initial begin
        // Initialize signals
        TDI = 1;
        TMS = 1;
        
        #105;
        
        // Hold reset state for at least 5 cycles
        repeat (5) begin
            TMS = 1; // Move to Test-Logic-Reset
            #10;
        end

//        // Move to Shift-DR state
//        TMS = 0; #10; // Run-Test/Idle
//        TMS = 1; #10; // Select-DR-Scan
//        TMS = 0; #10; // Capture-DR
//        TMS = 0; #10; // Shift-DR
        
        // Move to Shift-DR state
        TMS = 0; #10; // Run-Test/Idle
        TMS = 1; #10; // Select-DR-Scan
        TMS = 1; #10; // Select-IR-Scan
        TMS = 0; #10; // Capture-IR
        TMS = 0; #10; // Shift-IR
    
        // Shift out data
        repeat (5) begin
            TMS = 0; // Move to Test-Logic-Reset
            #10;
        end
        
        
        // Hold reset state for at least 5 cycles
        repeat (5) begin
            TMS = 1; // Move to Test-Logic-Reset
            #10;
        end
    
    
        #300;
        $stop;
    end
    
endmodule
