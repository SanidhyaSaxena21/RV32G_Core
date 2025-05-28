`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/12/2025 01:09:23 PM
// Design Name: 
// Module Name: command
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


/*
==========================================================================================================================
| Abstract Command Summary: Access Register (cmdtype = 0)
==========================================================================================================================
| Purpose:
| - Allows debugger to read/write CPU registers and optionally execute the Program Buffer.
|
| Execution Sequence:
| 1) If transfer == 1 and write == 0:
|    -> data0    <= [regno]
|    -> Read the register specified by regno into arg0 (Data[0]).
| 2) If transfer == 1 and write == 1:
|    -> [regno] <= data0
|    -> Write arg0 (Data[0]) into the register specified by regno.
| 3) If aarpostincrement == 1:
|    -> regno <= regno + 1
|    -> Increment regno after access. (Incrementing beyond valid range makes regno UNSPECIFIED.)
| 4) If postexec == 1:
|    -> Execute Program Buffer once after transfer step.
|
| Failure Handling:
| - If any step fails, cmderr is set, and subsequent steps are skipped.
| - If the specified regno is invalid or inaccessible in the hart, cmderr must be set to 3 (exception).
|
| Field Descriptions:
| cmdtype (8-bit)         : Must be 0 for this access-register command.
| aarsize (3-bit)         : Specifies access width (2-32 bits, 3-64 bits, or 4-128 bits). Larger than reg size causes failure.
| aarpostincrement (1-bit): Optional. Enables auto-increment of regno after access.
| postexec (1-bit)        : Optional. Executes Program Buffer after transfer if set.
| transfer (1-bit)        : Controls if register transfer is performed.
| write (1-bit)           : 0 = Read register into arg0; 1 = Write arg0 to register.
| regno (16-bit)          : Specifies which register is targeted.
|
| Notes:
| - This command modifies only Data[0] (arg0) when reading. other data registers are not changed
| - Program Buffer execution is allowed only if postexec is set.
| - Debug modules must support read/write to all GPRs when halted.
| - Accessing other registers or running-hart registers is optional.
| - The debugger must ensure the hart's state allows access to the requested register.
==========================================================================================================================
| Register Number Mapping (regno field for Abstract Commands)
==========================================================================================================================
| Range            | Group Description
|------------------|--------------------------------------------------------------
| 0x0000 - 0x0FFF  | CSRs (Control and Status Registers) - includes dpc for PC.
| 0x1000 - 0x101F  | GPRs (General Purpose Registers) - standard integer registers.
| 0x1020 - 0x103F  | FPRs (Floating Point Registers) - if implemented.
| 0xC000 - 0xFFFF  | Reserved - Non-standard extensions or internal use.
|
| Notes:
| - regno is a 16-bit field in the abstract command.
| - When using cmdtype = 0 (Access Register), this field selects which register to read/write.
| - If regno points to an invalid or unsupported register, cmderr must be set to 3 (exception).
==========================================================================================================================
*/


/*
==========================================================================================================================
| Abstract Command Summary: Quick Access (cmdtype = 1) (optional)
==========================================================================================================================
| Purpose:
| - Allows the debugger to rapidly execute a Program Buffer on the target hart without touching Data registers.
| - Simplifies command flow for quick single-step code execution.
|
| Execution Sequence:
| 1) If the hart is already halted:
|    -> cmderr <= 4 (halt/resume error) and abort.
|
| 2) Halt the hart.
|    -> If the hart halts for a reason other than this command (e.g., breakpoint),
|       cmderr <= 4 (halt/resume error) and abort.
|
| 3) Execute the Program Buffer.
|    -> If an exception occurs during execution:
|       - cmderr <= 3 (exception).
|       - Program Buffer execution is stopped.
|       - The hart remains halted with cause code set to 3.
|
| 4) If Program Buffer completes without exception:
|    -> Resume the hart.
|
| Failure Handling:
| - Any failure during the sequence sets cmderr, and the remaining steps are skipped.
| - cmderr values:
|    -> 3: Exception occurred.
|    -> 4: Halt/Resume error (hart not in required state).
|
| Field Descriptions:
| cmdtype (8-bit)  : Must be 1 for this Quick Access command.
| [23:0] Reserved  : Must be written as 0.
|
| Notes:
| - This command is optional to implement.
| - Data registers are not used or modified.
| - Only the Program Buffer and hart state are affected.
| - Debugger is responsible for ensuring the Program Buffer contents are valid before issuing this command.
==========================================================================================================================
*/


/*
==========================================================================================================================
| Abstract Command Summary: Access Memory (cmdtype = 2)
==========================================================================================================================
| Purpose:
| - Allows the debugger to perform memory load/store operations through the selected hart,
|   respecting the same memory view and permissions as M-mode code.
| - Enables access to RAM, MMIO, or hart-local memory-mapped registers.

| Execution Sequence:
| 1) If write == 0:
|    -> data0 <= [data1] 
|    -> Reads data from the memory address specified by arg1 into arg0 (Data[0]).
|
| 2) If write == 1:
|    -> [data1] <= data0
|    -> Writes the value from arg0 (Data[0]) to the memory address specified by arg1.
|
| 3) If aampostincrement == 1:
|    -> data1 <= data1 + size_in_bytes (determined by aamsize)
|    -> After the access, arg1 is incremented by the width of the memory operation.
|
| Failure Handling:
| - If any step fails (due to privilege, alignment, or invalid access), cmderr is set and remaining steps are skipped.
| - Early detection of failure is allowed - optional - hardware may detect and report errors before attempting the actual memory access.
| - Access must only fail if the same access would fail in M-mode software on the hart.

| Field Descriptions:
| cmdtype (8-bit)      : Must be set to 2 for this command.
| aamvirtual (1-bit)   : Addressing mode.
|                      -> 0: Physical addresses (default and mandatory).
|                      -> 1: Virtual addresses (if supported, using MPRV translation rules).
|
| aamsize (3-bit)      : Size of the memory transfer.
|                      -> 0: 8-bit access.
|                      -> 1: 16-bit access.
|                      -> 2: 32-bit access.
|                      -> 3: 64-bit access.
|                      -> 4: 128-bit access.
|                      * Sizes larger than supported cause failure.
|
| aampostincrement(1-bit): Optional, enables address auto-increment.
|                        -> After successful access, arg1 += size.
|                        -> Great for memory streaming - highly recommended if supported.
|
| write (1-bit)        : Read/Write control.
|                      -> 0: Perform memory read into arg0.
|                      -> 1: Write value from arg0 to memory.
|
| target-specific (2-bit): Reserved, implementation-defined behavior.
|
| Notes:
| - Only data[0] (arg0) and arg1 are involved.
| - arg0 is only modified during a memory read.
| - arg1 is incremented if aampostincrement is set.
| - Supporting memory access while the hart is running is optional - but if so, halted mode must also be supported.
| - The encoding for aamsize matches sbaccess in the System Bus Access (sbcs) field.

==========================================================================================================================
*/



//The specification for the control field is not a single fixed layout - it's dependent on the cmdtype. 

// GPRs - 0x1000 to 0x101f

// CSRs:
`define satp            16'h180
`define mstatus         16'h300
`define misa            16'h301
`define mie             16'h304
`define mtvec           16'h305
`define mepc            16'h341
`define mcause          16'h342
`define mtval           16'h343
`define mip             16'h344
`define debug_dcsr      12'h7b0
`define debug_dpc       12'h7b1
`define debug_dscratch0 12'h7b2
`define debug_dscratch1 12'h7b3
`define PMPCFG0         16'h3A0
`define PMPCFG1         16'h3A1
`define PMPCFG2         16'h3A2
`define PMPCFG3         16'h3A3
`define PMPADDR0        16'h3B0
`define PMPADDR1        16'h3B1
`define PMPADDR2        16'h3B2
`define PMPADDR3        16'h3B3
`define PMPADDR4        16'h3B4
`define PMPADDR5        16'h3B5
`define PMPADDR6        16'h3B6
`define PMPADDR7        16'h3B7
`define PMPADDR8        16'h3B8
`define PMPADDR9        16'h3B9
`define PMPADDR10       16'h3BA
`define PMPADDR11       16'h3BB
`define PMPADDR12       16'h3BC
`define PMPADDR13       16'h3BD
`define PMPADDR14       16'h3BE
`define PMPADDR15       16'h3BF
`define cache_flush     16'h400



    
module command (
    input  wire        clk,           // Clock input
    input  wire        rst_n,         // Active-low synchronous reset

    input  wire [31:0] wdata,         // Incoming write data to command register
    input  wire        wr_en,         // Write enable signal for command register
    input  wire [2:0]  cmderr,        // Current abstract command error status
    input  wire        abstracts_busy,// 'busy' bit from abstractcs (command execution status)

    input  wire        inc_regno,             // Increment regno
    input  wire        clear_new_cmd_flag,    // Clears the new_cmd_flag when busy gets set
    input  wire        reg_access_complete,   // Indicates register access was successful and complete
    input  wire        mem_access_complete,   // Indicates memory access was successful and complete
    
    output reg         new_cmd_flag,            // Set high when a new command is accepted
    output reg         unsupported_aarsize,     // Flag if requested aarsize is larger than supported
    output reg         cmd_not_supported,       // Set high for unrecognized regno or cmdtype
    output reg [7:0]  cmdtype,                 // Extracted cmdtype for external use
    output reg         AR_transfer,             // Access Register: Transfer flag
    output reg         AR_write,                // Access Register: Write flag
    output reg         AM_write,                // Access Memory: Write flag
    output reg         AM_aampostincrement,     //should go to data registers (data1) should be incremented only after a successful memory access
    output reg [2:0]   AM_aamsize,              // arg1 should be incremented by this when aampostincrement is 1
    output reg         AR_aarpostincrement,
    
    output wire        exec_progbuf,         // Execute program buffer exactly once if this flag is set
    output wire [31:0] command_read,          // Always returns zero (WARZ behavior)
    output reg [31:0]  command
);

        // Registers for decoded Access Register command fields
    reg [2:0] AR_aarsize;
    reg AR_postexec;
    reg [15:0] AR_regno;
    
    // Internal: Pack cmdtype and control field
    reg [23:0] control;
//    assign {cmdtype, control} = (wr_en==1) ? wdata : {cmdtype, control};

    // WARZ behavior: Reads always return zero
//    assign command_read = 32'd0;
    assign command_read = command;

    // Program buffer execution trigger if postexec is set
    assign exec_progbuf = AR_postexec;
                   
    // Registers for decoded Access Memory command fields
    reg AM_aamvirtual;
    reg [1:0] AM_target_specific;

    reg clear_new_cmd_flag_q;
    wire clear_new_cmd_flag_pulse;
    always @(posedge clk) begin
      if(!rst_n) clear_new_cmd_flag_q <= 1'b0;
      else clear_new_cmd_flag_q <= clear_new_cmd_flag;
    end
    
    assign clear_new_cmd_flag_pulse = clear_new_cmd_flag & ~clear_new_cmd_flag_q;
    
    // Sequential logic for handling command register writes
    always @(posedge clk) begin
        if (!rst_n) begin
            // Access Register command fields reset
            AR_aarsize           <= 3'd0;
            AR_aarpostincrement  <= 1'b0;
            AR_postexec          <= 1'b0;
            AR_transfer          <= 1'b0;
            AR_write             <= 1'b0;
            AR_regno             <= 16'd0;
    
            // Access Memory command fields reset
            AM_aamvirtual        <= 1'b0;
            AM_aamsize           <= 3'd0;
            AM_aampostincrement  <= 1'b0;
            AM_write             <= 1'b0;
            AM_target_specific   <= 2'd0;
            
            new_cmd_flag         <= 1'b0;
            cmd_not_supported    <= 1'b0;
            unsupported_aarsize  <= 1'b0;
            
        end else begin
            if(clear_new_cmd_flag_q)   //if busy flag get set then clear the new command_flag
                new_cmd_flag    <= 1'b0;
            
            
            /* Accept new command only if:
                - Write enable is high
                - cmderr == 0 (no error)
                - abstracts_busy == 0 (debug unit not busy)
            */
            //else if (wr_en && ((cmderr == 3'd0) || (cmderr == 3'd2)) && (!abstracts_busy)) begin
            else if (wr_en && (!abstracts_busy)) begin
                case (cmdtype)
                    8'd0: begin
                        // Access Register Command (cmdtype = 0)
                        new_cmd_flag    <= 1'b1;
                        cmd_not_supported <= 0;
                        
                        // Access Register Command
                        AR_aarsize          <= control[22:20];
                        AR_aarpostincrement <= control[19];
                        AR_postexec         <= control[18];
                        AR_transfer         <= control[17];
                        AR_write            <= control[16];
                        AR_regno            <= control[15:0];
                        
                        if(control[22:20] != 3'd2) begin
                            unsupported_aarsize <= 1;
                            cmd_not_supported <= 1;
                        end
                        
                        case (control[15:0])
                            `satp, `mstatus, `misa, `mie,`mtvec, `mepc, `mcause, `mtval, `mip,
                             `debug_dcsr, `debug_dpc, `debug_dscratch0, `debug_dscratch1,
                            `PMPCFG0, `PMPCFG1, `PMPCFG2, `PMPCFG3,
                            `PMPADDR0, `PMPADDR1, `PMPADDR2, `PMPADDR3,
                            `PMPADDR4, `PMPADDR5, `PMPADDR6, `PMPADDR7,
                            `PMPADDR8, `PMPADDR9, `PMPADDR10, `PMPADDR11,
                            `PMPADDR12, `PMPADDR13, `PMPADDR14, `PMPADDR15,
                            `cache_flush, `mcycle, `mtime, `minstret, `mcycleh, `mtimeh, `minstreth:
                                cmd_not_supported <= 1'b0;
                             default:
                                // Range check for GPRs: 0x1000 to 0x101F - 32 registers
                                if ((control[15:0] >= 16'h1000 && control[15:0] <= 16'h101F) || (control[15:0] == 16'h07b0) || (control[15:0] == 16'h07b1))
                                    cmd_not_supported <= 1'b0;   // Valid GPR
                                else
                                    cmd_not_supported <= 1'b1;   // Invalid regno
                        endcase 
                    end
    
                    8'd1: begin
                        /* Quick Access Command
                           No fields to extract - all reserved.
                           No state update needed.
                        */
                    end
    
                    8'd2: begin
                         // Access Memory Command (cmdtype = 2)
                        new_cmd_flag    <= 1'b1;
                        cmd_not_supported <= 0;
                    
                        // Access Memory Command
                        AM_aamvirtual        <= control[23];
                        AM_aamsize           <= control[22:20];
                        AM_aampostincrement  <= control[19];
                        AM_write             <= control[16];
                        AM_target_specific   <= control[15:14];
                        
                        if(control[23])                 //Virtual addressing is not supported; so cmd not supported
                            cmd_not_supported <= 1;
                    end
    
                    default: begin
                        // Unrecognized command type
                        cmd_not_supported <= 1;
                    end
                endcase
            end
            else if (reg_access_complete && inc_regno)
                // Post-increment AR_regno after register access completes successfully
                AR_regno <= AR_regno + 1;
        end
    end
    
    
    // Combinational logic to assemble dmcontrol output register
    always @(*) begin
        if(wr_en == 1)
            {cmdtype, control} <= wdata;
    
        if(cmdtype == 0) begin
            command <= {
                cmdtype,
                1'b0,
                AR_aarsize,
                AR_aarpostincrement,
                AR_postexec,
                AR_transfer,
                AR_write,
                AR_regno
            };
        end
        else if(cmdtype == 2) begin
            command <= {
                cmdtype,
                AM_aamvirtual,
                AM_aamsize,
                AM_aampostincrement,
                2'b0,
                AM_write,
                AM_target_specific,
                14'b0
            };
        end
    end
endmodule
