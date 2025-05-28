`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/26/2025 12:45:40 PM
// Design Name: 
// Module Name: RISC_V_Bypass_signal_Gen
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


module RISC_V_Bypass_signal_Gen(
        input wire clk,
        
        input wire [31:0]  ptr_regno_data_custom0,
        input wire [31:0]  ptr_data1_data_custom1,
        input wire [31:0]  signals_custom2,
        

        output reg [31:0]  ptr_regno_data_db,
        output reg [31:0]  ptr_data1_data_db,
        
        output reg         command_complete_db,
        output reg         reg_access_complete_db,
        output reg         mem_access_complete_db,
        
        output reg         corehalted_db,
        output reg         corerunning_db,
        output reg         coreunavail_db,
        output reg         coreexist_db,
        output reg         coreresumeack_db,
        output reg         corehavereset_db,
        
        output reg         abstract_cmd_failed_db,
        output reg         abstract_cmd_wrong_hart_state_db,
        output reg         abstract_cmd_exception_db,
        output reg         abstract_cmd_bus_error_db,
        
        output reg [7:0]   PC_debug_db
    );
    
    always @(*)begin
        ptr_regno_data_db                  <= ptr_regno_data_custom0;
        ptr_data1_data_db                  <= ptr_data1_data_custom1;
        
        command_complete_db                <= signals_custom2[20];
        reg_access_complete_db             <= signals_custom2[19];
        mem_access_complete_db             <= signals_custom2[18];
        
        corehalted_db                      <= signals_custom2[17];
        corerunning_db                     <= signals_custom2[16];
        coreunavail_db                     <= signals_custom2[15];
        coreexist_db                       <= signals_custom2[14];
        coreresumeack_db                   <= signals_custom2[13];
        corehavereset_db                   <= signals_custom2[12];
        
        abstract_cmd_failed_db             <= signals_custom2[11];
        abstract_cmd_wrong_hart_state_db   <= signals_custom2[10];
        abstract_cmd_exception_db          <= signals_custom2[9];
        abstract_cmd_bus_error_db          <= signals_custom2[8];
        
        PC_debug_db                        <= signals_custom2[7:0];
    end
    
    
endmodule
