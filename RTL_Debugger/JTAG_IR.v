`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/09/2025 08:14:01 PM
// Design Name: 
// Module Name: JTAG_IR
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

// instructions
// (mandatory) Bypass - 5'b11111 and all unused codes
// (mandatory) Sample/Preload - NOT DEFINED // selects the boundary scan registers
// (mandatory) Extest - 5'b00000
// idcode - 5'b00010

module JTAG_IR #(
    parameter WIDTH = 8 // IR width
) (
    input wire rst_n,
    input wire TCK,
    
    input wire Clock_IR,
    input wire Shift_IR,
    input wire Update_IR,
    
    input wire [3:0] TAP_state,
    
    input wire IR_shift_in,
    output reg IR_shift_out,
    output wire [WIDTH-1:0] IR_parallel_out
);

localparam ID_CODE_INSTR    = 5'd2,
           BYPASS_INSTR     = 5'd1;

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

reg [WIDTH-1:0] scan_reg;
reg [WIDTH-1:0] hold_reg;
assign IR_parallel_out = hold_reg;

//assign IR_shift_out = scan_reg[0];
always @(negedge TCK) begin //edited-toms
    if(!rst_n)
        IR_shift_out <= 0;
    else
        IR_shift_out <= scan_reg[0];

end

always @(posedge TCK) begin //changed-toms-changedback
    if(!rst_n) begin
        scan_reg <= 1;      // capturing 01 pattern
        hold_reg <= ID_CODE_INSTR;
    end
    else if(Clock_IR)begin    
        if(Shift_IR == 1)
            scan_reg <= {IR_shift_in , scan_reg[WIDTH-1:1]};
            
        else if(Update_IR == 1)
            hold_reg <= scan_reg;
            
        else if(TAP_state == CAPTURE_IR)
//            scan_reg <= 1;
            scan_reg <= hold_reg;
    end      
end

endmodule