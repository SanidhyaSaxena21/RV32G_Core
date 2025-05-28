`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/03/2025 04:10:49 PM
// Design Name: 
// Module Name: dmcontrol
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
 
//hartsel = hartselhi + hartsello
 
// A debugger should discover HARTSELLEN by writing all ones to hartsel 
// (assuming the maximum size) and reading back the value to see which 
// bits were actually set.
 
// Debugger should not change hartsel while an abstract command is executing
 
/*
Bit Fields of Key Registers:
 
============================================================================================================
| Register    | Bit(s)      | Bit name                      | Description                                  |
|=============|=============|===============================|==============================================|
| dmcontrol   | [0]         | dmactive                      | Activate debug module                        |
|             | [1]         | ndmreset                      | Non-debug module reset                       |
|             | [2]         | clrresethaltreq               | Clear resethaltreq                           |
|             | [3]         | setresethaltreq               | Set resethaltreq                             |
|             | [4]         | clrkeepalive                  | Clear keepalive                              |
|             | [5]         | setkeepalive                  | Set keepalive                                |
|             | [15:6]      | hartselhi                     | High bits of hartsel                         |
|             | [25:16]     | hartsello                     | Low bits of hartsel                          |
|             | [26]        | hasel                         | Hartsel enable                               |
|             | [27]        | ackunavail                    | Ack unavailable                              |
|             | [28]        | ackhavereset                  | Ack hart reset                               |
|             | [29]        | hartreset                     | Reset selected harts                         |
|             | [30]        | resumereq                     | Request hart resume                          |
|             | [31]        | haltreq                       | Request hart halt                            |
============================================================================================================
*/
 
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
 
// autoclear ackhavereset, ackunavail, resumereq
`define HARTSELLEN 1 //can be anything between 0 and 20 (inc. both 0 and 20)
 
module dmcontrol (
    input wire clk,
    input wire rst_n,
    input wire wr_en,
    input wire [31:0] wdata,
    output reg [31:0] dmcontrol_out,
    output reg haltreq,
    output reg resumereq,
    output reg hartreset,
    output reg ackhavereset,
    output reg ackunavail,
    input core_resumeack,
    
    output reg setresethaltreq,
    output reg clrresethaltreq,
    output reg ndmreset,
    output wire dmactive
);
 
    // Register Fields
//    reg haltreq;          // [31]       WARZ - Set to halt selected harts; cleared to cancel. Ignored during abstract command execution.
//    reg resumereq;        // [30]       W1   - Resume selected harts if halted; clears resume-ack. Ignored if haltreq is set.
//    reg hartreset;        // [29]       WARL - Optional: Set to reset selected harts; must clear by writing 0 to deassert.
//    reg ackhavereset;     // [28]       W1   - Write 1 to clear havereset status for selected harts.
//    reg ackunavail;       // [27]       W1   - Write 1 to clear unavail status if selected harts are currently available.
    reg hasel;            // [26]       WARL - 0: Select single hart via hartsel. 1: Select multiple via hartsel + hart array mask.
    reg [9:0] hartsello;  // [25:16]    WARL - Low bits of hartsel index (DM-specific). Always part of selected harts.
    reg [9:0] hartselhi;  // [15:6]     WARL - High bits of hartsel index (DM-specific). Always part of selected harts.
    reg setkeepalive;     // [5]        W1   - Optional: Set keepalive on selected harts (unless clrkeepalive also set).
    reg clrkeepalive;     // [4]        W1   - Optional: Clear keepalive on selected harts.
//    reg setresethaltreq;  // [3]        W1   - Optional: Set resethaltreq—halt harts after reset. Not auto-cleared.
//    reg clrresethaltreq;  // [2]        W1   - Optional: Clear resethaltreq for selected harts.
//    reg ndmreset;         // [1]        R/W  - Reset entire system except Debug Module. Set to 1 to reset, then clear to deassert.
//    reg dmactive;         // [0]        R/W  - Activates the Debug Module. Must be set before using other fields. DM resets when cleared.
    
    
    
    //Reset signals
    reg rst_n_1, rst_n_2;
    
    wire [`HARTSELLEN-1:0] hartsel = 0;
    assign dmactive = rst_n_2;
    
    always @(posedge clk) begin
        if (~rst_n) begin
            haltreq         <= 1'b0;
            resumereq       <= 1'b0;
            hartreset       <= 1'b0;
            ackhavereset    <= 1'b0;
            ackunavail      <= 1'b0;
            hasel           <= 1'b0;
            // Optional: ignore these fields in single-hart case
            hartsello       <= 10'b0;
            hartselhi       <= 10'b0;
            
            setkeepalive    <= 1'b0;
            clrkeepalive    <= 1'b0;
            setresethaltreq <= 1'b0;
            clrresethaltreq <= 1'b0;
            ndmreset        <= 1'b0;
          end
//            dmactive        <= 1'b1;


        else if (wr_en) begin
            // WARZ
            haltreq         <= wdata[31] ? 1'b1 : 1'b0;
 
            // W1 fields
            if (!haltreq) begin 
              resumereq <= wdata[30];
            end
            if (wdata[28]) ackhavereset <= 1'b1;
            if (wdata[27]) ackunavail   <= 1'b1;
            if (wdata[5])  setkeepalive <= 1'b0;
            if (wdata[4])  clrkeepalive <= 1'b0;
            if (wdata[3] && !wdata[2])  setresethaltreq <= 1'b1;
            else if (wdata[2]) setresethaltreq <= 1'b0;
            if (wdata[2])  clrresethaltreq <= 1'b1;
 
            // WARL and R/W fields
            hartreset   <= wdata[29];
            hasel       <= wdata[26] & 1'b0;           //tying this bit to 0
            
            // Optional: ignore these fields in single-hart case
            hartsello       <= 10'b0;
            hartselhi       <= 10'b0;
            
            ndmreset    <= wdata[1];
        end
        if(wr_en)
            rst_n_1    <= wdata[0];
        rst_n_2 <= rst_n_1;
    end
 
    // Output current register state
    always @(*) begin
        dmcontrol_out = {
            haltreq & 1'b0 ,     // [31] WARZ - Write-Any Read-Zero
            resumereq,           // [30]
            hartreset,           // [29] WARL - Write-Any Read-Legal
            ackhavereset,        // [28]
            ackunavail,          // [27]
            hasel,               // [26]
            hartsello,           // [25:16]
            hartselhi,           // [15:6]
            setkeepalive,        // [5]
            clrkeepalive,        // [4]
            setresethaltreq,     // [3]
            clrresethaltreq,     // [2]
            ndmreset,            // [1]
            dmactive             // [0]
        };
    end
 
endmodule
