`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/10/2025 12:11:14 PM
// Design Name: 
// Module Name: JTAG
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


module JTAG #(
    parameter WIDTH = 8 // IR width
)(
//    output reg TDO,   
    input wire TDI,
    input wire TMS,
    input wire TCK,
    
    output wire IR_shift_out,          // IR shift output
    output wire Bypass_out,            // Bypass Register output
    output wire ID_out,                // ID Register output

    output wire [3:0] state_out,
    output wire rst_out,
    output wire [WIDTH-1:0] probing_out  //TDI
    );
    
    // Internal Signals
    wire Clock_DR, Shift_DR, Update_DR;
    wire Clock_IR, Shift_IR, Update_IR;
    wire Reset_n, Select, Enable;
    wire [3:0] TAP_state;

    wire [WIDTH-1:0] Instr;     // IR width
//    wire IR_shift_out;          // IR shift output
//    wire Bypass_out;            // Bypass Register output
//    wire ID_out;                // ID Register output
    
    localparam TEST_LOGIC_RESET = 4'd0,
               RUN_TEST_IDLE    = 4'd1,
               SELECT_DR_SCAN   = 4'd2,
               CAPTURE_DR       = 4'd3,
               SHIFT_DR         = 4'd4,
               EXIT1_DR         = 4'd5,
               PAUSE_DR         = 4'd6,
               EXIT2_DR         = 4'd7,
               UPDATE_DR        = 4'd8,
               
               SELECT_IR_SCAN   = 4'd9,
               CAPTURE_IR       = 4'd10,
               SHIFT_IR         = 4'd11,
               EXIT1_IR         = 4'd12,
               PAUSE_IR         = 4'd13,
               EXIT2_IR         = 4'd14,
               UPDATE_IR        = 4'd15;
    
    //Debug
    assign state_out = TAP_state;
    assign rst_out = Reset_n;

    TAP_Controller u_TAP_Controller (
//        .TDO(TDO),
        .TCK(TCK),
//        .TDI(TDI),
        .TMS(TMS),
        
        
        // DR Control Signals
        .Clock_DR(Clock_DR),
        .Shift_DR(Shift_DR),
        .Update_DR(Update_DR),
        
        // IR Control Signals
        .Clock_IR(Clock_IR),
        .Shift_IR(Shift_IR),
        .Update_IR(Update_IR),
        
        // Generic Signals
        .Reset_n(Reset_n),
        .Select(Select),
        .Enable(Enable),
        
        .state(TAP_state)
    );
    
    JTAG_IR #(.WIDTH(WIDTH)) u_JTAG_IR (
        .rst_n(Reset_n),
        .TCK(TCK),
        
        .Clock_IR(Clock_IR),
        .Shift_IR(Shift_IR),
        .Update_IR(Update_IR),
        
        .TAP_state(TAP_state),
        
        .IR_shift_in(TDI),
        .IR_shift_out(IR_shift_out),
        .IR_parallel_out(Instr)
    );
    
    Bypass_Reg u_Bypass_Reg (
        .rst_n(Reset_n),
        .Data_in(TDI),
        .Shift_DR(Shift_DR),
        .Clock_DR(Clock_DR),
        .TCK(TCK),
        .TAP_state(TAP_state),
        
        .Data_out(Bypass_out)
    );
    
    ID_Reg u_ID_Reg (
        .Data_in(TDI),
        .Shift_DR(Shift_DR),
        .Clock_DR(Clock_DR),
        .TCK(TCK),
        
        .Data_out(ID_out)
    );
    
    
    assign probing_out = Instr;
    
//     TDO Selection Logic
//    always @(*) begin
//        if(TAP_state == SHIFT_IR) begin
//            TDO <= IR_shift_out;
//        end
//        else if (TAP_state == SHIFT_DR) begin
//            case (Instr)
//                8'hFF: TDO <= Bypass_out;  // Bypass Instruction
//                8'h03: TDO <= ID_out;      // IDCODE Instruction
//                // Add more instructions here if needed
//                default: TDO <= ID_out;    // Default to ID_out
//            endcase
//        end
//        else
//            TDO = ID_out;
//    end                                  
    
endmodule