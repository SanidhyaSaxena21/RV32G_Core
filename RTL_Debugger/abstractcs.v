`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/12/2025 11:18:25 AM
// Design Name: 
// Module Name: abstractcs
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

/*
+--------+--------------------------------------------------------------+
| Type   | Description                                                  |
+--------+--------------------------------------------------------------+
| R      | Read-only.                                                   |
| R/W    | Read/Write                                       .           |
| R/W1C  | Read/Write Ones to Clear.                                    |
|        | - Writing 1 clears the corresponding bit.                    |
|        | - Writing 0 has no effect.                                   |
|        | - Other values: undefined behavior.                          |
| WARZ   | Write Any, Read Zero.                                        |
|        | - Any value may be written.                                  |
|        | - Always reads as 0.                                         |
| W1     | Write Only.                                                  |
|        | - Only writing 1 has an effect.                              |
|        | - Reads always return 0.                                     |
| WARL   | Write Any, Read Legal.                                       |
|        | - Any value may be written.                                  |
|        | - Unsupported values coerced to legal values.                |
+--------+--------------------------------------------------------------+
*/
//write enable remaining
module abstractcs #(
    parameter PROGBUF_SIZE  = 5'd0,     // Max program buffer size (in 32-bit words)
    parameter DATAREG_COUNT = 5'd2      // 2 data registers: data0 and data1
)(
    input wire clk,                          // Clock signal
    input wire rst_n,                        // Active-low reset
    input wire [7:0] addr,
    input wire [3:0] TAP_state,
    
    
    // Abstract command control inputs
    input  wire abstract_cmd_complete,            // Indicates command execution has completed
    input  wire abstract_cmd_reg_written,         // Abstract command register was written
    input  wire abstract_auto_reg_written,        // Auto-execution configuration was written
    input  wire abstract_cmd_not_supported,       // Error: unsupported abstract command
    input  wire abstract_cmd_exception,           // Error: exception during command execution
    input  wire abstract_cmd_wrong_hart_state,    // Error: hart in wrong state for command
    input  wire abstract_cmd_bus_error,           // Error: bus access failed
    input  wire abstract_cmd_failed,              // Error: unspecified command failure
    
    input  wire hart_unavailable,
    
    // Additional register write flags
    input wire data_reg_written,                  // Data register was written
    input wire progbuf_reg_written,               // Program buffer register was written
    
    input wire command_wr_en,
    
    // Write data input for field updates
    input wire [31:0] wdata,                      // Data to be written to abstractcs fields
    input wire wr_en,
    
    // AbstractCS output register (read by debugger host)
    output reg [31:0] abstractcs_out,             // Composite output register
    output reg [2:0] cmderr,                      // Command error code field [24:22]
    output reg busy                               // Busy flag: command in progress [26]
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
               
               
    // Internal fields representing abstractcs register structure
    reg [4:0] progbufsize = PROGBUF_SIZE;         // Program buffer size field [31:27]
    // Busy flag: command in progress [26]
    reg       relaxedpriv;                        // Relaxed privilege checking [25]
    // Command error code field [24:22]
    reg [3:0] datacount = DATAREG_COUNT;          // Number of data registers [20:17]
    reg abstract_cmd_reg_written_floped;
    reg data_reg_written_floped;

    // Decode write attempts for privileged fields
  //  wire write_relaxedpriv  = wdata[11];          // Write input for relaxedpriv field
    wire [2:0] write_cmderr = wdata[10:8];        // Write input for cmderr field

    wire data_reg_read_written = ((addr == 8'h04) || (addr == 8'h05)) && ((TAP_state==UPDATE_DR) || (TAP_state==SHIFT_DR));
    wire command_reg_written_edge = !abstract_cmd_reg_written_floped && abstract_cmd_reg_written;

    // Sequential logic block: reset and update abstractcs internal state
    always @(posedge clk) begin
        abstract_cmd_reg_written_floped <= abstract_cmd_reg_written;
        data_reg_written_floped <= data_reg_written;
        if (!rst_n) begin
            // Reset abstractcs fields to default hardware values
            progbufsize <= PROGBUF_SIZE;
            busy        <= 1'b0;
            relaxedpriv <= 1'b1;
            cmderr      <= 3'b000;
            datacount   <= DATAREG_COUNT;
        end else begin
            // Priority-based error detection and recording logic
            if ((cmderr == 3'd0) && busy && (data_reg_read_written || command_wr_en))
                cmderr <= 3'd1;  // Modified command during execution - illegal
            else if ((cmderr == 3'd0) && busy && (command_reg_written_edge || abstract_auto_reg_written || data_reg_written || progbuf_reg_written || wr_en))
                cmderr <= 3'd1;  // Illegal write while busy
            else if (abstract_cmd_not_supported)
                cmderr <= 3'd2;  // Unsupported abstract command
            else if (abstract_cmd_exception)
                cmderr <= 3'd3;  // Exception occurred during command
            else if (abstract_cmd_wrong_hart_state || hart_unavailable)
                cmderr <= 3'd4;  // Hart not in the required state
            else if (abstract_cmd_bus_error)
                cmderr <= 3'd5;  // System Bus error
            else if (abstract_cmd_failed)
                cmderr <= 3'd7;  // Generic failure
            else if (write_cmderr != 3'd0 && wr_en) 
                cmderr <= cmderr & (~write_cmderr);  // Clear error bits based on write mask
            
            
            // Busy flag management logic
            if (abstract_cmd_reg_written && !abstract_cmd_not_supported && ~busy && ~abstract_cmd_complete)
                busy <= 1'b1;  // Command begins
            else if (abstract_cmd_complete && abstract_cmd_reg_written)
                busy <= 1'b0;  // Command finished
            
        end
    end

    // Combinational logic: assemble abstractcs register for external read
    always @(*) begin
        abstractcs_out = {
            3'd0,                // Reserved bits [31:29]
            progbufsize,         // Program buffer size field [28:24]
            11'd0,               // Reserved bits [27:17]
            busy,                // Busy flag [16]
            relaxedpriv,         // Relaxed privilege bit [15]
            cmderr,              // Command error status [14:12]
            4'd0,                // Reserved bits [11:8]
            datacount            // Data register count [7:4]
        };
    end


endmodule
