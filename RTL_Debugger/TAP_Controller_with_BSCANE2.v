`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: TAP_Controller_with_BSCANE2
// Description: Implements JTAG TAP controller with BSCANE2 for OpenOCD detection
//////////////////////////////////////////////////////////////////////////////////

module TAP_Controller_with_BSCANE2 (
    input wire clk,
    output wire TDO,
    output wire TDI,
    output wire TMS,
    output wire TCK,
    output wire SEL,
    output wire [3:0] state_out,
    output wire rst_out,
    output wire [2:0] probing_out,
    output wire [2:0] which_probe_out
    
    
);

    reg [4:0] IR; // 5-bit instruction register
    reg [31:0] IDCODE = 32'h03722093; // Artix-7 IDCODE
    reg [31:0] DR; // Data Register
    reg TDO_reg;
    reg [2:0] probing;
    reg [2:0] which_probe;

    assign TDO = (state == SHIFT_DR || state == SHIFT_IR) ? TDO_reg : 1'bz;
    
    // TAP Controller States
    reg [3:0] state;
    assign state_out = state;
    
    assign which_probe_out = which_probe;
    assign rst_out = (state == TEST_LOGIC_RESET) ? 0 : 1;
    
    localparam PROBE_IR         = 2'd0,
               PROBE_DR         = 2'd1,
               PROBE_TDO        = 2'd2;
               
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
               PAUSE_IR        = 4'd13,
               EXIT2_IR         = 4'd14,
               UPDATE_IR        = 4'd15;
    
    // Initialize state
    initial state = TEST_LOGIC_RESET;
    
    // BSCANE2 Instantiation
    BSCANE2 #(.JTAG_CHAIN(1)) bscane2_inst (
        .TCK(TCK),
        .TDI(TDI),
        .TDO(TDO),
        .TMS(TMS),
        .SEL(SEL)
    );
    
    // TAP State Machine
    always @(posedge TCK) begin
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
            PAUSE_IR:        state <= (TMS) ? EXIT2_IR : PAUSE_IR;
            EXIT2_IR:         state <= (TMS) ? UPDATE_IR : SHIFT_IR;
            UPDATE_IR:        state <= (TMS) ? SELECT_DR_SCAN : RUN_TEST_IDLE;
            
            default:          state <= TEST_LOGIC_RESET;
        endcase
    end
    
    
    assign probing_out = (which_probe[PROBE_TDO] == 1) ? 
                                    (state == SHIFT_DR) ? 6 : 7 :
                                     probing;
                                     
    always @(posedge TCK) begin
        // ---------- IR ---------- 
        if (state == CAPTURE_IR)
                probing <= 1;
        else if (state == SHIFT_IR)
                probing <= 2;
                
        // ---------- DR ----------         
        else if (state == CAPTURE_DR) begin
            if (IR == 5'b00001) 
                probing <= 3;
            else 
                probing <= 4;
            end
        else if (state == SHIFT_DR) 
            probing <= 5;
    end
    
    
    // Instruction Register (IR) Handling
    always @(posedge TCK) begin
        if (state == CAPTURE_IR) begin
                IR <= 5'b00001; // Must match OpenOCD expectation
                which_probe[PROBE_IR] <= 1;
            end
        else if (state == SHIFT_IR) begin
                IR <= {TDI, IR[4:1]};
                which_probe[PROBE_IR] <= 1;
            end
        else begin
                which_probe[PROBE_IR] <= 0;
            end
    end
    
    // Data Register (DR) Handling
    always @(posedge TCK) begin
        if (state == CAPTURE_DR) begin
            if (IR == 5'b00001) begin
                    DR <= IDCODE; // IDCODE for OpenOCD detection
                    which_probe[PROBE_DR] <= 1;
                end
            else begin
                    DR <= 32'h00000000;
                    which_probe[PROBE_DR] <= 1;
                end
            end 
        else if (state == SHIFT_DR) begin
                DR <= {TDI, DR[31:1]};
                which_probe[PROBE_DR] <= 1;
            end
        else begin
                which_probe[PROBE_DR] <= 0;
            end
    end
    
    // TDO Output Logic
    always @(negedge TCK) begin
        if (state == SHIFT_DR) begin
                TDO_reg <= DR[0];
                which_probe[PROBE_TDO] <= 1;
                //6
            end
        else if (state == SHIFT_IR) begin
                TDO_reg <= IR[0];
                which_probe[PROBE_TDO] <= 1;
                //7
            end
        else begin
                which_probe[PROBE_TDO] <= 0;
            end
    end
endmodule
