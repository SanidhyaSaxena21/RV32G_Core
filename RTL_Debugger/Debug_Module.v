`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/07/2025 04:00:53 PM
// Design Name: 
// Module Name: Debug_Module
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

============================================================================================================
| Register    | Bit(s)      | Bit name                      | Description                                  |
|=============|=============|===============================|==============================================|
| dmstatus    | [0]         | authenticated                 | Authenticated                                |
|             | [1]         | authbusy                      | Auth busy                                    |
|             | [2]         | hasresethaltreq               | Has resethaltreq                             |
|             | [3]         | confstrptrvalid               | confstrptr valid                             |
|             | [6:4]       | version                       | Debug spec version                           |
|             | [7]         | anyrunning                    | At least one hart running                    |
|             | [8]         | allhalted                     | All harts halted                             |
|             | [9]         | anyhalted                     | At least one hart halted                     |
|             | [10]        | allrunning                    | All harts running                            |
|             | [11]        | allresumeack                  | All resume acknowledgments                   |
|             | [12]        | anyresumeack                  | At least one hart resume ack                 |
|             | [13]        | allnonexistent                | All selected harts nonexistent               |
|             | [14]        | anynonexistent                | At least one selected hart nonexistent       |
|             | [15]        | allunavail                    | All selected harts unavailable               |
|             | [16]        | anyunavail                    | At least one selected hart unavailable       |
|             | [17]        | allhavereset                  | All selected harts have been reset           |
|             | [18]        | anyhavereset                  | At least one selected hart has been reset    |
|             | [20]        | impebreak                     | Imprecise break                              |
|             | [23:21]     | reserved                      | Reserved bits                                |
|             | [24]        | stickyunavail                 | Sticky unavailable                           |
|             | [25]        | ndmresetpending               | Non-debug module reset pending               |
| hartinfo    | [0]         | dataaccess                    | Indicates support for abstract data access   |
|             | [4:3]       | datasize                      | Size of data register access (log2)          |
|             | [31:24]     | nscratch                      | Number of scratch registers                  |
| abstractcs  | [0]         | busy                          | Abstract command busy                        |
|             | [10:8]      | cmderr                        | Command error code                           |
| command     | [6:0]       | cmdtype                       | Command type                                 |
|             | [31:20]     | aarsize, aarpostincrement     | Access size and post-increment flags         |
| abstractauto| [0]         | autoexecdata0                 | Auto execute when writing data0              |
|             | [16]        | autoexecprogbuf0              | Auto execute when writing progbuf0           |
| sbcs        | [2:0]       | sbaccess                      | System bus access size                       |
|             | [20]        | sbbusy                        | System bus busy                              |
============================================================================================================
*/


/*
The following registers are not implemented because it is a single hart system
- hawindowsel
- hawindow 
*/
module Debug_Module #(parameter DM_BASE_ADDR = 0) (
    input wire clk,
    input wire clk_jtag,
    input wire rst_n_switch,
    input wire [7:0] addr,
    input wire [3:0] TAP_state,
    input wire TDI,
    
    input wire IR_shift_out,          // IR shift output
    input wire Bypass_out,            // Bypass Register output
    input wire ID_out,                // ID Register output
    
    output reg TDO,
    
    
    
    // ==================
    // From RISC-V Core
    // ==================
    input wire [31:0] ptr_regno_data_riscv,    
    input wire [31:0] ptr_data1_data_riscv,    
    input wire        core_command_complete_riscv,    
    input wire        core_reg_access_complete_riscv,    
    input wire        core_mem_access_complete_riscv,    
    input wire        core_halted_riscv,    
    input wire        core_running_riscv,    
    input wire        core_unavail_riscv,    
    input wire        core_exist_riscv,    
    input wire        core_resumeack_riscv,    
    input wire        core_havereset_riscv,    
    input wire        core_abstract_cmd_failed_riscv,    
    input wire        core_abstract_cmd_wrong_hart_state_riscv,    
    input wire        core_abstract_cmd_exception_riscv,    
    input wire        core_abstract_cmd_bus_error_riscv,    
    input wire [7:0]  core_PC_debug_riscv,  
    
    // ==================
    // To RISC-V Core
    // ==================
    output wire         to_core_haltreq,
    output wire         to_core_resumereq,
    output wire         to_core_hartreset,
    output wire         to_core_ackhavereset,
    output wire         to_core_ackunavail,
    output wire         to_core_resethaltreq_reg,
    output wire         to_core_ndmreset,
    output wire [31:0]  to_core_data0,
    output wire [31:0]  to_core_data1,
    output wire [31:0]  to_core_instr_bus,
    output wire         to_core_new_cmd_flag,
    output wire         to_core_cmd_not_supported,
    
//        ========probing========
//        output wire [31:0]  probe_data_write,
//        output wire         probe_wr_en,
//        output wire         probe_wr_en_flopped,
//        output wire         probe_cmd_AR_transfer,
//        output wire         probe_cmd_AR_write,
//        output wire         probe_cmd_new_cmd_flag,
//        output wire [7:0]   probe_cmd_cmdtype,
//        output wire [31:0]  probe_cmd,
        
//        output wire         probe_cmd_wr_en, 
//        output wire [2:0]   probe_cmderr, 
//        output wire         probe_abstracts_busy, 
//        output wire [31:0]  probe_cmd_wdata,
//        output wire [1:0] probe_data0_selector, 
//        output wire [31:0] probe_temp,
//        =======================
    
    //troubleshooting
    output wire [31:0]  data_0_probe,
    output wire [15:0]  data_display
    
);

    localparam MUX_UNLOCK_VALUE = 32'h12345678;

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
               

    // ==================
    // Signals from Other Logic
    // ==================
    reg [31:0]  temp;
    reg         TDO_temp;
    wire [31:0] instr_bus;
    wire        rst_n;
    
    // ==================
    // Signals from Abstract Data 0 reg
    // ==================
    wire [31:0] data0;
    wire        data0_written;
    
    
    
    // ==================
    // Signals from Abstract Data 1 reg
    // ==================
    wire [31:0] data1;
    wire        data1_written;
    
    
    
    // ==================
    // Signals from dmcontrol reg
    // ==================
    wire [31:0] dmcontrol_out;
    wire        haltreq; 
    wire        resumereq; 
    wire        hartreset; 
    wire        ackhavereset; 
    wire        ackunavail; 
    wire        setresethaltreq;
    wire        clrresethaltreq;
    reg         resethaltreq_reg;
    wire        ndmreset;
    wire        internal_reset_n;
    
    
    // ==================
    // Signals from hartinfo reg
    // ==================
    wire [31:0] hartinfo_value;
    
    
    
    // ==================
    // Signals from abstractcs reg
    // ==================
    wire        data_reg_written;
    wire [31:0] abstractcs_value;
    wire        abcs_busy;
    wire [2:0]  abcs_cmderr;
    
    
    // ==================
    // Signals from command reg
    // ==================
    wire        cmd_new_cmd_flag;
    wire        cmd_unsupported_aarsize;
    wire        cmd_cmd_not_supported;
    wire [7:0]  cmd_cmdtype;
    wire        cmd_AR_transfer;
    wire        cmd_AR_write;
    wire        cmd_AM_write;
    wire        cmd_AM_aampostincrement;
    wire [2:0]  cmd_AM_aamsize;
    wire        cmd_AR_aarpostincrement;
    wire        cmd_exec_progbuf;
    wire [31:0] cmd_command_read;
    wire [31:0] cmd;
    
    
    // ==================
    // Signals from dmstatus reg
    // ==================
    wire [31:0] dmstatus_out;
    wire        any_hart_unavail;
    
    
    
    // ==================
    // Signals from MUX
    // ==================
    reg [31:0] ptr_regno_data;    
    reg [31:0] ptr_data1_data;    
    reg        core_command_complete;    
    reg        core_reg_access_complete;    
    reg        core_mem_access_complete;    
    reg        core_halted;    
    reg        core_running;    
    reg        core_unavail;    
    reg        core_exist;    
    reg        core_resumeack;    
    reg        core_havereset;    
    reg        core_abstract_cmd_failed;    
    reg        core_abstract_cmd_wrong_hart_state;    
    reg        core_abstract_cmd_exception;    
    reg        core_abstract_cmd_bus_error;    
    reg [7:0]  core_PC_debug;    
    
    
    // ==================
    // Signals from write enables
    // ================== 
    wire        update_data0;
    wire        update_data1;
    wire        update_dmcontrol;
    wire        update_abstractcs;
    wire        update_command;
    wire        inc_regno;

    
    // ==========================
    // Custom reg signals
    // ==========================
    wire [31:0] ptr_regno_data_custom0;
    wire [31:0] ptr_data1_data_custom1;
    wire [31:0] signals_custom2;
    wire [31:0] mux_lock_custom3;
    wire [31:0] p_ptr_regno_data;
    wire [31:0] p_ptr_data1_data;
    wire [31:0] p_other_signals;
    wire [31:0] other_signals;
    
    wire [31:0] command_probe;
    wire [31:0] abstractcs_probe;
    wire [31:0] dmcontrol_probe;
    wire [31:0] data0_probe;
    wire [31:0] data1_probe;
    
    // ==========================
    // RISC_V_Bypass_signal_Gen outputs
    // ==========================
    wire [31:0] ptr_regno_data_db;
    wire [31:0] ptr_data1_data_db;

    wire        command_complete_db;
    wire        reg_access_complete_db;
    wire        mem_access_complete_db;

    wire        corehalted_db;
    wire        corerunning_db;
    wire        coreunavail_db;
    wire        coreexist_db;
    wire        coreresumeack_db;
    wire        corehavereset_db;

    wire        abstract_cmd_failed_db;
    wire        abstract_cmd_wrong_hart_state_db;
    wire        abstract_cmd_exception_db;
    wire        abstract_cmd_bus_error_db;

    wire [7:0]  PC_debug_db;

    // ==========================
    // MUX signals
    // ==========================
    wire        SEL_db;
    
    
    reg update_command_flopped;
    
    //        ========probing========
//    assign probe_cmd_AR_transfer = cmd_AR_transfer;
//    assign probe_cmd_AR_write = cmd_AR_write;
//    assign probe_cmd_new_cmd_flag = cmd_new_cmd_flag;
//    assign probe_cmd_cmdtype = cmd_cmdtype;
//    assign probe_cmd = cmd;
    //        =======================
    
    // ==================
    // Generation of all wr_en and inc_regno and other logic
    // ================== 
    assign update_data0         = ((addr == 8'h04) && (TAP_state == UPDATE_DR))
                               || (cmd_new_cmd_flag && ((cmd_AR_write==0) && (cmd_AR_transfer==1) && cmd_cmdtype==0)) 
                               || (cmd_new_cmd_flag && ((cmd_AM_write==0) && cmd_cmdtype==2));
    assign update_data1         = ((addr == 8'h05) && (TAP_state == UPDATE_DR)) || (cmd_new_cmd_flag && (cmd_AM_aampostincrement==1) && (cmd_cmdtype ==2)); 
    assign update_dmcontrol     = (addr == 8'h10) && (TAP_state == UPDATE_DR);
    assign update_abstractcs    = (addr == 8'h16) && (TAP_state == UPDATE_DR);
    assign update_command       = (addr == 8'h17) && (TAP_state == UPDATE_DR);
    assign inc_regno            = ((cmd_AR_aarpostincrement == 1) && (cmd_cmdtype == 0));
    assign update_custom0       = (addr == 8'h70) && (TAP_state == UPDATE_DR);
    assign update_custom1       = (addr == 8'h71) && (TAP_state == UPDATE_DR);
    assign update_custom2       = (addr == 8'h72) && (TAP_state == UPDATE_DR);
    assign update_custom3       = (addr == 8'h73) && (TAP_state == UPDATE_DR);
    
    assign data_reg_written     = (data0_written || data1_written);
    assign instr_bus = (core_PC_debug == 8'h17) ? cmd : 32'd0;
    
    
    // Assign signals to core-facing outputs
    assign to_core_haltreq              = haltreq;
    assign to_core_resumereq            = resumereq;
    assign to_core_hartreset            = hartreset;
    assign to_core_ackhavereset         = ackhavereset;
    assign to_core_ackunavail           = ackunavail;
    assign to_core_resethaltreq_reg     = resethaltreq_reg;
    assign to_core_ndmreset             = ndmreset;
    assign to_core_data0                = data0;
    assign to_core_data1                = data1;
    assign to_core_instr_bus            = instr_bus;
    assign to_core_new_cmd_flag         = cmd_new_cmd_flag;
    assign to_core_cmd_not_supported    = cmd_cmd_not_supported;
    
    // Reset signal generation
    assign rst_n                        = rst_n_switch && internal_reset_n;
    
    //MUX signals
    assign SEL_db                       = (mux_lock_custom3 == MUX_UNLOCK_VALUE);
    
    //troubleshooting
    assign data_display = data0[15:0];
    
    
    assign data_0_probe = data0;
    
    
    
    
    
    // ==========================
    // Instance of data0 module (0x04)
    // ==========================
    data0 u_data0 (
        .clk(clk),                          // System clock
        .rst_n(rst_n),                      // Active-low reset
        
        .addr(addr),                        // Address input
        .TAP_state(TAP_state),              // Current JTAG TAP controller state
//        .new_cmd_flag(cmd_new_cmd_flag),    // Indicates a new abstract command has started
//        .write(cmd_AR_write),               // Write flag from command field
//        .transfer(cmd_AR_transfer),         // Transfer flag from command field
        .cmdtype(cmd_cmdtype),              // Abstract command type field
        .wr_en(update_data0),
        
        .temp(temp),                        // Temporary buffer data
        .regno_data(ptr_regno_data),        // Data from memory/register pointed by regno
        .data1_data(ptr_data1_data),        // Data from memory pointed by data1
        
//        //        ========probing========
//        .probe_data_write(probe_data_write),
//        .probe_wr_en(probe_wr_en),
//        .probe_wr_en_flopped(probe_wr_en_flopped),
//        .probe_data0_selector(probe_data0_selector),
//        .probe_temp(probe_temp),
        //        =======================
        
        .data0_out(data0),                  // Output: selected data for data0 register
        .data0_written(data0_written)       // Output: data0 write strobe indicator
    );

    // ==========================
    // Instance of data1 module (0x05)
    // ==========================
    data1 u_data1 (
        .clk(clk),                                  // System clock
        .rst_n(rst_n),                              // Active-low reset
    
        .addr(addr),                                // Address input for data1 register
        .TAP_state(TAP_state),                      // Current JTAG TAP controller state
//        .new_cmd_flag(cmd_new_cmd_flag),            // Indicates a new abstract command has started
//        .aampostincrement(cmd_AM_aampostincrement), // Flag: post-increment enable for abstract memory
//        .cmdtype(cmd_cmdtype),                      // Abstract command type
        .wr_en(update_data1),
        
        .temp(temp),                                // Temporary buffer data (from TDI or other source)
        .aamsize(cmd_AM_aamsize),                   // Memory access size (8/16/32/64/128 bit)
    
        .data1_out(data1),                          // Output: value to write to memory address (arg1)
        .data1_written(data1_written)               // Output: write strobe for data1 register
    );
    
    
    // =======================================
    // Instance of dmcontrol Module (0x10)
    // =======================================
    dmcontrol u_dmcontrol (
        .clk(clk),                         // Input: System clock
        .rst_n(rst_n),                     // Input: Active-low reset
        .wr_en(update_dmcontrol),                     // Input: Write enable for DMCONTROL register
        .wdata(temp),                     // Input: Data to write into DMCONTROL register
    
        .dmcontrol_out(dmcontrol_out),     // Output: DMCONTROL register current state
        .haltreq(haltreq),                 // Output: Request to halt the hart
        .resumereq(resumereq),             // Output: Request to resume the hart
        .hartreset(hartreset),             // Output: Request to reset the selected hart
        .ackhavereset(ackhavereset),       // Output: Acknowledge the hart's havereset flag
        .ackunavail(ackunavail),           // Output: Acknowledge the hart's unavailable state
        .setresethaltreq(setresethaltreq), // Output: Set resethaltreq field
        .clrresethaltreq(clrresethaltreq), // Output: Clear resethaltreq field
        .ndmreset(ndmreset),                // Output: Reset signal for the entire debug module
        .dmactive(internal_reset_n)
    );

    
    
    // ==========================
    // Instance of hartinfo module (0x12)
    // ==========================
    hartinfo u_hartinfo (
        .hartinfo_out(hartinfo_value)  // Output: Fixed Hart Info Register Value
    );
    
    
    // =======================================
    // Instance of abstractcs Module (0x16)
    // =======================================
    abstractcs #(
        .PROGBUF_SIZE(5'd0),         // Program buffer size (default 0)
        .DATAREG_COUNT(5'd2)         // Number of data registers (default 2: data0 and data1)
    ) u_abstractcs (
        .clk(clk),                                 // System clock
        .rst_n(rst_n),                             // Active-low reset
        .addr(addr),
        .TAP_state(TAP_state),
    
        // Abstract command control signals
        .abstract_cmd_complete(core_command_complete),
        .abstract_cmd_reg_written(cmd_new_cmd_flag),
        .abstract_auto_reg_written(0),
        .abstract_cmd_not_supported(cmd_cmd_not_supported),
        .abstract_cmd_exception(core_abstract_cmd_exception),
        .abstract_cmd_wrong_hart_state(core_abstract_cmd_wrong_hart_state),
        .abstract_cmd_bus_error(core_abstract_cmd_bus_error),
        .abstract_cmd_failed(core_abstract_cmd_failed),
        
        .hart_unavailable(any_hart_unavail),
        
        // Additional register write flags
        .data_reg_written(data_reg_written),
        .progbuf_reg_written(0),
    
        .command_wr_en(update_command && !update_command_flopped),
    
        // Write interface
        .wdata(temp),
        .wr_en(update_abstractcs),
    
        // Outputs
        .abstractcs_out(abstractcs_value),              // AbstractCS register value
        .cmderr(abcs_cmderr),                           // Command error code output
        .busy(abcs_busy)                                // Busy status output
    );



    // =======================================
    // Instance of command Module (0x17)
    // =======================================
    command u_command (
        .clk(clk),                                      // System clock input
        .rst_n(rst_n),                                  // Active-low synchronous reset
    
        .wdata(temp),                                   // Incoming data for the command register
        .wr_en(update_command && !update_command_flopped),                         // Write enable for command register
        .cmderr(abcs_cmderr),                           // AbstractCS command error input
        .abstracts_busy(abcs_busy),                     // Busy signal from abstractcs (indicates ongoing command)
    
        .inc_regno(inc_regno),                          // Signal to increment regno if aarp increment is active
        .clear_new_cmd_flag(core_command_complete),                 // Clears the new_cmd_flag when busy gets set
        .reg_access_complete(core_reg_access_complete), // Register access successful completion flag
        .mem_access_complete(core_mem_access_complete), // Memory access successful completion flag
    
        .new_cmd_flag(cmd_new_cmd_flag),                // Output: New command flag status
        .unsupported_aarsize(cmd_unsupported_aarsize),  // Output: Unsupported aarsize flag
        .cmd_not_supported(cmd_cmd_not_supported),      // Output: Unsupported command flag
    
        .cmdtype(cmd_cmdtype),                          // Output: Extracted command type
        .AR_transfer(cmd_AR_transfer),                  // Output: Access Register Transfer control flag
        .AR_write(cmd_AR_write),                        // Output: Access Register Write control flag
    
        .AM_write(cmd_AM_write),                        // Output: Access Memory Write control flag
        .AM_aampostincrement(cmd_AM_aampostincrement),  // Output: Access Memory Post Increment control
        .AM_aamsize(cmd_AM_aamsize),                    // Output: Access Memory Transfer size info
        .AR_aarpostincrement(cmd_AR_aarpostincrement),  // Output: Access Register Post Increment control
    
        //        ========probing========
//        .probe_cmd_wr_en(probe_cmd_wr_en),
//        .probe_cmderr(probe_cmderr),
//        .probe_abstracts_busy(probe_abstracts_busy),
//        .probe_cmd_wdata(probe_cmd_wdata),
        
        //        =======================
    
    
        .exec_progbuf(cmd_exec_progbuf),                // Output: Program Buffer execution trigger
        .command_read(cmd_command_read),                // Output: Read view of command register (WARZ - always zero)
        .command(cmd)                                   // Output: Full raw command register value
    );




    // =======================================
    // Instance of dmstatus Module (0x11)
    // =======================================
    dmstatus #(
        .HARTSELLEN(1)                      // Hart selection width (1 for single-hart systems)
    ) u_dmstatus (
        .clk(clk),                          // System clock
        .rst_n(rst_n),                      // Active-low reset
        .dmcontrol(dmcontrol_out),          // Input: DMCONTROL register value from Debug Module
        .hartsel(1),                        // Input: Hart select (typically unused for single-hart)
    
        // Inputs from the RISC-V hart's debug interface
        .hart_halted(core_halted),          // Input: Hart halted status
        .hart_running(core_running),        // Input: Hart running status
        .hart_unavail(core_unavail),        // Input: Hart unavailable status
        .hart_exist(core_exist),            // Input: Hart existence (always 1'b1 in single-hart setups)
        .hart_resumeack(core_resumeack),    // Input: Resume acknowledgment signal from hart
        .hart_havereset(core_havereset),    // Input: Indicates hart has been reset since last acknowledge
    
        .anyunavail(any_hart_unavail),
        .dmstatus_out(dmstatus_out)         // Output: Computed DMSTATUS value
    );


    // ==========================
    // Instance of Custom0 module - ptr_regno_data
    // ==========================
    Custom0 u_Custom0 (
        .clk(clk),                          // System clock
        .rst_n(rst_n),                      // Active-low reset
        
        .wr_en(update_custom0),                      // Write enable input
        .wdata(temp),                      // 32-bit write data input
        
        .ptr_regno_data(ptr_regno_data_custom0)     // 32-bit output data
    );
    
    
    
    
    // ==========================
    // Instance of Custom1 module - ptr_data1_data
    // ==========================
    Custom1 u_Custom1 (
        .clk(clk),                          // System clock
        .rst_n(rst_n),                      // Active-low reset
        
        .wr_en(update_custom1),                      // Write enable input
        .wdata(temp),                      // 32-bit write data input
        
        .ptr_data1_data(ptr_data1_data_custom1)      // 32-bit output data
    );
    
    
    
    
    // ==========================
    // Instance of Custom2 module - other signals
    // ==========================
    Custom2 u_Custom2 (
        .clk(clk),                          // System clock
        .rst_n(rst_n),                      // Active-low reset
        
        .wr_en(update_custom2),                      // Write enable input
        .wdata(temp),                      // 32-bit write data input
        
        .signals(signals_custom2)                   // 32-bit output signals
    );
    
    
    // ==========================
    // Instance of Custom3 module - MUX control
    // ==========================
    Custom3 u_Custom3 (
        .clk(clk),                          // System clock
        .rst_n(rst_n),                      // Active-low reset
        
        .wr_en(update_custom3),                      // Write enable input
        .wdata(temp),                      // 32-bit write data input
        
        .mux_lock(mux_lock_custom3)                 // 32-bit output mux_lock
    );

    
    // ========================================
    // Instance of RISC_V_Bypass_signal_Gen module
    // ========================================
    RISC_V_Bypass_signal_Gen u_RISC_V_Bypass_signal_Gen (
        .clk                                    (clk),                          // System clock

        .ptr_regno_data_custom0                      (ptr_regno_data_custom0),             // DB input: regno data
        .ptr_data1_data_custom1                      (ptr_data1_data_custom1),             // DB input: data1 data
        .signals_custom2                             (signals_custom2),                    // DB input: signals

        .ptr_regno_data_db                   (ptr_regno_data_db),          // Output: regno data for debug
        .ptr_data1_data_db                   (ptr_data1_data_db),          // Output: data1 data for debug

        .command_complete_db                 (command_complete_db),        // Output: command complete
        .reg_access_complete_db              (reg_access_complete_db),     // Output: register access complete
        .mem_access_complete_db              (mem_access_complete_db),     // Output: memory access complete

        .corehalted_db                       (corehalted_db),              // Output: core halted status
        .corerunning_db                      (corerunning_db),             // Output: core running status
        .coreunavail_db                      (coreunavail_db),             // Output: core unavailable status
        .coreexist_db                        (coreexist_db),               // Output: core exist status
        .coreresumeack_db                    (coreresumeack_db),           // Output: core resume acknowledgment
        .corehavereset_db                    (corehavereset_db),           // Output: core have reset

        .abstract_cmd_failed_db              (abstract_cmd_failed_db),     // Output: abstract command failed
        .abstract_cmd_wrong_hart_state_db    (abstract_cmd_wrong_hart_state_db), // Output: wrong hart state
        .abstract_cmd_exception_db           (abstract_cmd_exception_db),  // Output: abstract command exception
        .abstract_cmd_bus_error_db           (abstract_cmd_bus_error_db),  // Output: abstract command bus error

        .PC_debug_db                         (PC_debug_db)                // Output: debug PC value
    );

    // ==========================
    // Instance of Custom4 module
    // ==========================
    Custom4 u_Custom4 (
        .clk                (clk),              // System clock
        .rst_n              (rst_n),             // Active-low reset
    
        .wdata              (ptr_regno_data),       // Input data
        .p_ptr_regno_data   (p_ptr_regno_data)    // Output: registered ptr_regno_data
    );


    // ==========================
    // Instance of Custom5 module
    // ==========================
    Custom5 u_Custom5 (
        .clk                (clk),              // System clock
        .rst_n              (rst_n),             // Active-low reset
    
        .wdata              (ptr_data1_data),       // Input data
        .p_ptr_data1_data   (p_ptr_data1_data)   // Output: registered ptr_data1_data
    );

    // ==========================
    // Instance of Custom6 module
    // ==========================
    Custom6 u_Custom6 (
        .clk               (clk),              // System clock
        .rst_n             (rst_n),             // Active-low reset
    
        .wdata             (other_signals),       // Input data
        .p_other_signals   (p_other_signals)    // Output: registered other signals
    );

    // ==========================
    // Instance of Custom7 module
    // ==========================
    Custom7 u_Custom7 (
        .clk               (clk),              // System clock
        .rst_n             (rst_n),             // Active-low reset
    
        .wdata             (cmd),       // Input data
        .command_probe     (command_probe)    // Output: registered other signals
    );

    // ==========================
    // Instance of Custom8 module
    // ==========================
    Custom8 u_Custom8 (
        .clk               (clk),              // System clock
        .rst_n             (rst_n),             // Active-low reset
    
        .wdata             (abstractcs_value),       // Input data
        .abstractcs_probe  (abstractcs_probe)    // Output: registered other signals
    );

    // ==========================
    // Instance of Custom6 module
    // ==========================
    Custom9 u_Custom9 (
        .clk               (clk),              // System clock
        .rst_n             (rst_n),             // Active-low reset
    
        .wdata             (dmcontrol_out),       // Input data
        .dmcontrol_probe   (dmcontrol_probe)    // Output: registered other signals
    );

    // ==========================
    // Instance of Custom6 module
    // ==========================
    Custom10 u_Custom10 (
        .clk               (clk),              // System clock
        .rst_n             (rst_n),             // Active-low reset
    
        .wdata             (data0),       // Input data
        .data0_probe       (data0_probe)    // Output: registered other signals
    );

    // ==========================
    // Instance of Custom6 module
    // ==========================
    Custom11 u_Custom11 (
        .clk               (clk),              // System clock
        .rst_n             (rst_n),             // Active-low reset
    
        .wdata             (data1),       // Input data
        .data1_probe       (data1_probe)    // Output: registered other signals
    );



    always @(posedge clk_jtag) begin
        if (!rst_n_switch) begin
            temp <= 0;
        end
        else if(TAP_state == CAPTURE_DR)begin
            case(addr)
                8'h04: temp <= data0;               // data0
                8'h05: temp <= data1;               // data1
                8'h10: temp <= dmcontrol_out;       // dmcontrol
                8'h12: temp <= hartinfo_value;      // hartinfo
                8'h16: temp <= abstractcs_value;    // abstractcs
                8'h17: temp <= cmd_command_read;    // command
                8'h11: temp <= dmstatus_out;        // dmstatus
                8'h74: temp <= p_ptr_regno_data;    //custom4
                8'h75: temp <= p_ptr_data1_data;    //custom5
                8'h76: temp <= p_other_signals;     //custom6
                
                8'h77: temp <= command_probe;       //custom7
                8'h78: temp <= abstractcs_probe;    //custom8
                8'h79: temp <= dmcontrol_probe;     //custom9
                8'h80: temp <= data0_probe;         //custom10
                8'h81: temp <= data1_probe;         //custom11
                default: temp <= 32'b0;
            endcase
        end
        else if (TAP_state == SHIFT_DR) begin       //shifting data
            temp <= {TDI, temp[31:1]};
        end
    end
    
    
    always @(negedge clk_jtag) begin
        if (!rst_n_switch) begin
            TDO_temp <= 0;
        end
        else if (TAP_state == SHIFT_DR) begin
            case (addr)
                8'hFF: TDO_temp <= Bypass_out;   // Bypass Instruction
                8'h03: TDO_temp <= ID_out;       // IDCODE Instruction
                8'h04: TDO_temp <= temp[0];      // data0
                8'h05: TDO_temp <= temp[0];      // data1
                8'h10: TDO_temp <= temp[0];      // dmcontrol
                8'h12: TDO_temp <= temp[0];      // hartinfo
                8'h16: TDO_temp <= temp[0];      // abstractcs
                8'h17: TDO_temp <= temp[0];      // command
                8'h11: TDO_temp <= temp[0];      // dmstatus
                8'h74: TDO_temp <= temp[0];     //custom4
                8'h75: TDO_temp <= temp[0];     //custom5
                8'h76: TDO_temp <= temp[0];     //custom6
                8'h77: TDO_temp <= temp[0];     //custom7
                8'h78: TDO_temp <= temp[0];     //custom8
                8'h79: TDO_temp <= temp[0];     //custom9
                8'h80: TDO_temp <= temp[0];     //custom10
                8'h81: TDO_temp <= temp[0];     //custom11
                default: TDO_temp <= ID_out;     // Default to ID_out
            endcase
        end
        else
            TDO_temp = ID_out;                   // IDCODE Instruction
    end     

    
    always @(*) begin
        if(TAP_state == SHIFT_IR) begin
            TDO <= IR_shift_out;
        end
        else if (TAP_state == SHIFT_DR) begin
            TDO <= TDO_temp;
        end
        else
            TDO <= TDO_temp;
    end  
    
    
    always @(posedge clk) begin
        update_command_flopped <= update_command;
        if(!rst_n_switch)
            resethaltreq_reg <= 0;
        else if (setresethaltreq)
            resethaltreq_reg <= 1;
        else if (clrresethaltreq)
            resethaltreq_reg <= 0;
    end



    //MUX
    always @(*)begin
        // ==================
        // Signals from MUX
        // ==================
        if (SEL_db == 1) begin
            ptr_regno_data                = ptr_regno_data_db;
            ptr_data1_data                = ptr_data1_data_db;
            core_command_complete         = command_complete_db;
            core_reg_access_complete      = reg_access_complete_db;
            core_mem_access_complete      = mem_access_complete_db;
            core_halted                   = corehalted_db;
            core_running                  = corerunning_db;
            core_unavail                  = coreunavail_db;
            core_exist                    = coreexist_db;
            core_resumeack                = coreresumeack_db;
            core_havereset                = corehavereset_db;
            core_abstract_cmd_failed      = abstract_cmd_failed_db;
            core_abstract_cmd_wrong_hart_state = abstract_cmd_wrong_hart_state_db;
            core_abstract_cmd_exception   = abstract_cmd_exception_db;
            core_abstract_cmd_bus_error   = abstract_cmd_bus_error_db;
            core_PC_debug                 = PC_debug_db;
        end
        else begin
            ptr_regno_data                = ptr_regno_data_riscv;
            ptr_data1_data                = ptr_data1_data_riscv;
            core_command_complete         = core_command_complete_riscv;
            core_reg_access_complete      = core_reg_access_complete_riscv;
            core_mem_access_complete      = core_mem_access_complete_riscv;
            core_halted                   = core_halted_riscv;
            core_running                  = core_running_riscv;
            core_unavail                  = core_unavail_riscv;
            core_exist                    = core_exist_riscv;
            core_resumeack                = core_resumeack_riscv;
            core_havereset                = core_havereset_riscv;
            core_abstract_cmd_failed      = core_abstract_cmd_failed_riscv;
            core_abstract_cmd_wrong_hart_state = core_abstract_cmd_wrong_hart_state_riscv;
            core_abstract_cmd_exception   = core_abstract_cmd_exception_riscv;
            core_abstract_cmd_bus_error   = core_abstract_cmd_bus_error_riscv;
            core_PC_debug                 = core_PC_debug_riscv;
        end
    end
    assign other_signals = {11'd0,
            core_command_complete,
            core_reg_access_complete,
            core_mem_access_complete,
            core_halted,
            core_running,
            core_unavail,
            core_exist,
            core_resumeack,
            core_havereset,
            core_abstract_cmd_failed,
            core_abstract_cmd_wrong_hart_state,
            core_abstract_cmd_exception,
            core_abstract_cmd_bus_error,
            core_PC_debug};
endmodule

