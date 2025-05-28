`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/10/2025 10:50:16 AM
// Design Name: 
// Module Name: TAP_Controller
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

// NOTE:
//      1. The DR that actually responds is the one enabled by the conditional 
//         control signals generated at the paralled outputs of the IR acc. to 
//         the particular instruction
//      2. State transition occurs on the positive edge of TCK
//      3. Output values change on the negative edge of TCK
//      4. Update_IR causes IT to be deselected as the register connected between TDI and TDO 
//         and the data register identified byt he current instruction is to be selected as
//         the new target register between TDI and TDO  



// Signal Definition:
//      1.  CLOCKDR: External data register clock. It is activated during TAP states of 
//                   Capture_IR, Shift_IR and Update_IR states only. CLOCKDR is also used 
//                   to clock the internal data register, i.e. the Device ID register.
//                      - CLOCKDR is derived from TCK.

//      2.   SHIFTDR: External data register shifting enable. SHIFTDR is also used to enable the 
//                    shifting operation of the internal data register, i.e. the Device ID register.
//                      - SHIFTDR is synchronized with the rising edge of TCK.
//
//      3.   UPDATEDR: External data register update enable. UPDATEDR is also used to enable the 
//                     internal data register's updating, i.e. the Device ID register.
//                      - UPDATEDR is synchronized with the falling edge of TCK.
//
//      4.   RESET: External data register reset signal.
//                      - RESET is an asynchronous signal.
//

module TAP_Controller(
//        input wire TDO,
//        output wire TCK,
//        output wire TDI,
//        output wire TMS,

        
        //Generic signals to all DR
        output wire Clock_DR,
        output wire Shift_DR,
        output wire Update_DR,
        
        //Dedicated signals to IR
        output wire Clock_IR,
        output wire Shift_IR,
        output wire Update_IR,
        
        //Generic signals
        output wire Reset_n,
        output wire Select,
        output wire Enable,
        
        output reg [3:0] state,
        
        //For debugging purposes
        input wire TCK,
        input wire TMS
    );

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

    // Initialize state
    initial state = TEST_LOGIC_RESET;
    
//    // BSCANE2 Instantiation
//    BSCANE2 #(.JTAG_CHAIN(1)) bscane2_inst (
//        .TCK(TCK),
//        .TDI(TDI),
//        .TDO(TDO),
//        .TMS(TMS),
//        .SEL(SEL)
//    );
    
    // TAP State Machine
    always @(posedge TCK) begin  //changed-toms-changed back
        case (state)
            TEST_LOGIC_RESET: state <= (TMS) ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
            RUN_TEST_IDLE:    state <= (TMS) ? SELECT_DR_SCAN : RUN_TEST_IDLE;
            
            // DR path
            SELECT_DR_SCAN:   state <= (TMS) ? SELECT_IR_SCAN : CAPTURE_DR;
            CAPTURE_DR:       state <= (TMS) ? EXIT1_DR : SHIFT_DR;
            SHIFT_DR:         state <= (TMS) ? EXIT1_DR : SHIFT_DR;
            EXIT1_DR:         state <= (TMS) ? UPDATE_DR : PAUSE_DR;
            PAUSE_DR:         state <= (TMS) ? EXIT2_DR : PAUSE_DR;
            EXIT2_DR:         state <= (TMS) ? UPDATE_DR : SHIFT_DR;
            UPDATE_DR:        state <= (TMS) ? SELECT_DR_SCAN : RUN_TEST_IDLE;
            
            // IR path
            SELECT_IR_SCAN:   state <= (TMS) ? TEST_LOGIC_RESET : CAPTURE_IR;
            CAPTURE_IR:       state <= (TMS) ? EXIT1_IR : SHIFT_IR;
            SHIFT_IR:         state <= (TMS) ? EXIT1_IR : SHIFT_IR;
            EXIT1_IR:         state <= (TMS) ? UPDATE_IR : PAUSE_IR;
            PAUSE_IR:         state <= (TMS) ? EXIT2_IR : PAUSE_IR;
            EXIT2_IR:         state <= (TMS) ? UPDATE_IR : SHIFT_IR;
            UPDATE_IR:        state <= (TMS) ? SELECT_DR_SCAN : RUN_TEST_IDLE;
            
            default:          state <= TEST_LOGIC_RESET;
        endcase
    end
    
    assign Clock_DR     = (state == UPDATE_DR) || (state == SHIFT_DR) || (state == CAPTURE_DR);
    assign Shift_DR     = (state == SHIFT_DR);
    assign Update_DR    = (state == UPDATE_DR);
        
    assign Clock_IR     = (state == UPDATE_IR) || (state == SHIFT_IR) || (state == CAPTURE_IR);
    assign Shift_IR     = (state == SHIFT_IR);
    assign Update_IR    = (state == UPDATE_IR);
        
    assign Reset_n      = (state == TEST_LOGIC_RESET) ? 0 : 1;
    assign Select       = (state == SELECT_DR_SCAN) || (state == SELECT_IR_SCAN);
    assign Enable       = (state != TEST_LOGIC_RESET); 

endmodule