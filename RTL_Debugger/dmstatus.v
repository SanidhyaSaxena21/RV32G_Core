`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/11/2025 05:30:03 PM
// Design Name: 
// Module Name: dmstatus
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
Bit Fields of Key Registers:

==============================================================================================================================
| Register   | Bit(s)     | Bit name           | Description                                                                 |
|============|============|====================|=============================================================================|
| dmstatus   | [0:3]      | version            | Version of the Debug Module interface   (should be 3)                       |
|            | [4]        | confstrptrvalid    | confstrptr0-3 contain a valid configuration structure pointer               |
|            | [5]        | hasresethaltreq    | Module supports halting harts on reset                                      |
|            | [6]        | authbusy           | Authentication process is ongoing                                           |
|            | [7]        | authenticated      | Debug module has been successfully authenticated                            |
|            | [8]        | anyhalted          | At least one selected hart is halted                                        |
|            | [9]        | allhalted          | All selected harts are halted                                               |
|            | [10]       | anyrunning         | At least one selected hart is running                                       |
|            | [11]       | allrunning         | All selected harts are running                                              |
|            | [12]       | anyunavail         | At least one selected hart is unavailable                                   |
|            | [13]       | allunavail         | All selected harts are unavailable                                          |
|            | [14]       | anynonexistent     | At least one selected hart does not exist                                   |
|            | [15]       | allnonexistent     | All selected harts do not exist                                             |
|            | [16]       | anyresumeack       | At least one selected hart has acknowledged a resume request                |
|            | [17]       | allresumeack       | All selected harts have acknowledged a resume request                       |
|            | [18]       | anyhavereset       | At least one selected hart has been reset and not acknowledged              |
|            | [19]       | allhavereset       | All selected harts have been reset and not acknowledged                     |
|            | [21:20]    | 0                  | Reserved bits                                                               |
|            | [22]       | impebreak          | Implicit ebreak in Program Buffer triggers halt                             |
|            | [23]       | stickyunavail      | Unavail status is sticky until cleared                                     |
|            | [24]       | ndmresetpending    | A system-wide reset is pending or ongoing                                   |
|            | [31:25]    | 0                  | Reserved bits                                                               |
==============================================================================================================================
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

module dmstatus#(
    parameter HARTSELLEN = 1 // Width of hartsel input for selecting a hart
) (
    input wire clk,                      // Clock signal
    input wire rst_n,                    // Active-low reset
    input wire [31:0] dmcontrol,        // Debug Module control register
    input wire [HARTSELLEN-1:0] hartsel,// Hart select signal (not used in single-hart system)
    
    
    // The following input signals are provided by the single RISC-V core:
    input wire hart_halted,             // Indicates if the selected hart is halted
    input wire hart_running,            // Indicates if the selected hart is running
    input wire hart_unavail,            // Indicates if the selected hart is unavailable (e.g., in reset or power-down)
    input wire hart_exist,              // Indicates if the selected hart exists (always 1'b1 for single-hart systems)
    input wire hart_resumeack,          // Indicates if the selected hart acknowledged a resume request
    input wire hart_havereset,          // Indicates if the selected hart has been reset since last acknowledgment

    output reg anyunavail,
    output reg [31:0] dmstatus_out      // Output: the dmstatus register value
);

    // Fixed fields per RISC-V Debug Spec
    localparam [3:0] VERSION = 4'd3;          // Version of Debug Spec
    localparam CONFSTRPTRVALID = 1'b0;        // No confstrptr implemented
    localparam HASRESETHALTREQ = 1'b1;        // reset-halt request is supported
    localparam AUTHBUSY = 1'b0;               // Authentication busy status
    localparam AUTHENTICATED = 1'b1;          // Authentication is not required
    localparam IMPEBREAK = 1'b1;              // Implicit ebreak support is present
    localparam STICKY_UNAVAIL = 1'b0;         
    
    
    
    // Status bits (combinational and sticky)
    reg anyhalted, allhalted;           // Reflect halt status of hart
    reg anyrunning, allrunning;         // Reflect running status of hart
    reg allunavail;         // Reflect unavailability status
    reg anynonexistent, allnonexistent; // Reflect non-existent harts (should be 0 in single-hart)
    reg anyresumeack, allresumeack;     // Reflect resume acknowledgment
    reg anyhavereset, allhavereset;     // Reflect havereset status
    reg stickyunavail;                  // Sticky bit set when hart becomes unavailable
    reg ndmresetpending;                // Set when ndmreset is issued (for edge-detection)

    // Register to latch dmcontrol for edge-triggered actions
    // This prevents asynchronous issues and enables detecting rising edges
    reg [31:0] dmcontrol_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dmcontrol_q <= 32'b0;         // Clear on reset
        else
            dmcontrol_q <= dmcontrol;     // Latch latest dmcontrol to detect changes
    end
    
    // Decode control bits from latched dmcontrol_q
    wire ackhavereset = dmcontrol_q[28];  // Clear havereset bits when set
    wire ackunavail   = dmcontrol_q[27];  // Clear stickyunavail when set
    wire ndmreset     = dmcontrol_q[1];   // Non-debug module reset request
    
   

    // Update status registers
    always @(posedge clk) begin
        if(!rst_n)begin
            // Clear all status bits on reset
            anyhalted       <= 1'b0;
            allhalted       <= 1'b0;
            anyrunning      <= 1'b0;
            allrunning      <= 1'b0;
            anyunavail      <= 1'b0;
            allunavail      <= 1'b0;
            anynonexistent  <= 1'b0;
            allnonexistent  <= 1'b0;
            anyresumeack    <= 1'b0;
            allresumeack    <= 1'b0;
            anyhavereset    <= 1'b0;
            allhavereset    <= 1'b0;
            stickyunavail   <= 1'b0;
            ndmresetpending <= 1'b0;
        end
        else begin
            // Sample hart signals to update dmstatus bits
            // For single-hart system, anyX and allX are the same
            anyhalted       <= hart_halted;
            allhalted       <= hart_halted;
            anyrunning      <= hart_running;
            allrunning      <= hart_running;
            anynonexistent  <= ~hart_exist;
            allnonexistent  <= ~hart_exist;
            anyresumeack    <= hart_resumeack;
            allresumeack    <= hart_resumeack;
            
            // These fields are set when the hart has been reset and reset has not been acknowledged.
            if(hart_havereset) begin        
                anyhavereset    <= 1'b1;
                allhavereset    <= 1'b1;
            end
            
            // Sticky bit for unavailable hart
            stickyunavail <= STICKY_UNAVAIL;

            // Current unavailability status
            if (!stickyunavail) begin           // The per hart unavail bits reflect the current state of the hart
                anyunavail <= hart_unavail;
                allunavail <= hart_unavail;
            end
            else begin                          // The per hart unavail bits are sticky. Cleared only when ackunavail is set but debugger
                if (ackunavail)begin
                    anyunavail <= 1'b0;
                    allunavail <= 1'b0; 
                end
                else if (hart_unavail)begin
                    anyunavail <= 1'b1;
                    allunavail <= 1'b1;       
                end
            end

            // Clear havereset flags if acknowledged via dmcontrol
            if (ackhavereset) begin
                anyhavereset <= 1'b0;
                allhavereset <= 1'b0;
            end

            // Set ndmresetpending when ndmreset is high (could be changed to edge detect)
            if (ndmreset)
                ndmresetpending <= 1'b1;
            else
                ndmresetpending <= 1'b0;
        end 
    end
    
    // Combinational logic to assemble dmstatus output register
    always @(*) begin
        dmstatus_out = {
            7'd0,                // [31:25] Reserved
            ndmresetpending,     // [24] Indicates if ndmreset is pending
            stickyunavail,       // [23] Sticky hart unavailable bit
            IMPEBREAK,           // [22] Implicit ebreak is implemented
            2'd0,                // [21:20] Reserved
            allhavereset,        // [19] All selected harts have havereset set
            anyhavereset,        // [18] Any selected hart has havereset set
            allresumeack,        // [17] All harts acknowledged resume
            anyresumeack,        // [16] Any hart acknowledged resume
            allnonexistent,      // [15] All selected harts don't exist
            anynonexistent,      // [14] Any selected hart doesn't exist
            allunavail,          // [13] All selected harts are unavailable
            anyunavail,          // [12] Any selected hart is unavailable
            allrunning,          // [11] All selected harts are running
            anyrunning,          // [10] Any selected hart is running
            allhalted,           // [9]  All selected harts are halted
            anyhalted,           // [8]  Any selected hart is halted
            AUTHENTICATED,       // [7]  Authenticated with debug module
            AUTHBUSY,            // [6]  No authentication operation in progress
            HASRESETHALTREQ,     // [5]  reset-halt request is supported
            CONFSTRPTRVALID,     // [4]  No confstrptr is provided
            VERSION              // [3:0] Version of debug module
        };
    end
endmodule


