`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/28/2025 01:02:03 PM
// Design Name: 
// Module Name: Top_JTAG
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
`include "defines.v"

module Top_CPU_JTAG#(
    parameter WIDTH = 8                 // IR width
)(

    //Global Signals
    input cpu_clock,
    input RESET_BUTTON, //Active High

    //JTAG Signals
    output wire TDO,
    input wire TDI,
    input wire TMS,
    input wire TCK,

    output [2:0] state,
    output [7:0] debug_signals,
  
    //Instruction Memory signals 
    output  [31:0]  IADDR,
    output  [1:0]   IBURST, //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
    output          IREQ,
    output          IWRB,
    output  [31:0]  IWDATA,
    input      [31:0]  IRDATA,
    input              IACK,
    input              ISTALL,
    input              ITLAST,
    output  [3:0]   IBSTROBE,
    
    //Data Memory signals
    output  [31:0]  DADDR,
    output  [1:0]   DBURST, //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
    output          DREQ,
    output          DWRB,
    output  [31:0]  DWDATA,
    input      [31:0]  DRDATA,
    input              DACK,
    input              DSTALL,
    input              DTLAST,
    output  [3:0]   DBSTROBE,

    output [63:0] led,
    
    input DEBUG_OVERWRITE,

    input cache_en,
    output [`CSR_SB_W-1:0] csr_pmp_sb,
    
    `ifdef itlb_def
    output vpn_to_ppn_req,
    `endif 
     
    input [31:0] irq_i,

    //==========Testing========== 
//    output wire [31:0] data_0_probe,
//    output wire [31:0] dpc,
//    output wire [15:0] R4 
      output wire halted
    );
    
    
    
    wire [3:0] state_out;        // TAP state
    wire rst_out;
    //wire cpu_clock;                 // 100 Mhz clock source on Basys 3 FPGA
    wire TCK_sync; 
   
    
    synchronizer u_sync (
        .clk(cpu_clock),
        .async_in(TCK),
        .sync_out(TCK_sync)
    );
    
    
    wire [WIDTH-1:0] Instr;             //Instr
    reg [15:0] seven_seg_data;
    
    wire IR_shift_out;          // IR shift output
    wire Bypass_out;            // Bypass Register output
    wire ID_out;                // ID Register output
    
    assign Instr_probe = Instr[WIDTH-3:0];

    assign debug_signals = {to_core_haltreq,to_core_resumereq,to_core_hartreset,to_core_resethaltreq_reg,to_core_ndmreset,core_halted_riscv,core_running_riscv,core_havereset_riscv};
    
    
    // ==================
    // From RISC-V Core to DM
    // ==================
    wire [31:0] ptr_regno_data_riscv;    
    wire [31:0] ptr_data1_data_riscv;    
    wire        core_command_complete_riscv;    
    wire        core_reg_access_complete_riscv;    
    wire        core_mem_access_complete_riscv;    
    wire        core_halted_riscv;    
    wire        core_running_riscv;    
    wire        core_unavail_riscv;    
    wire        core_exist_riscv;    
    wire        core_resumeack_riscv;    
    wire        core_havereset_riscv;    
    wire        core_abstract_cmd_failed_riscv;    
    wire        core_abstract_cmd_wrong_hart_state_riscv;    
    wire        core_abstract_cmd_exception_riscv;    
    wire        core_abstract_cmd_bus_error_riscv;    
    wire [31:0]  core_PC_debug_riscv;  
    
    // ==================
    // From DM To RISC-V Core
    // ==================
    wire         to_core_haltreq;
    wire         to_core_resumereq;
    wire         to_core_hartreset;
    wire         to_core_ackhavereset;
    wire         to_core_ackunavail;
    wire         to_core_resethaltreq_reg;
    wire         to_core_ndmreset;
    wire [31:0]  to_core_data0;
    wire [31:0]  to_core_data1;
    wire [31:0]  to_core_instr_bus;
    wire         to_core_new_cmd_flag;
    wire         to_core_cmd_not_supported;
    
    //troubleshooting
    wire [15:0]  data_display;
    
    
    assign halted = core_halted_riscv;


    //=============== JTAG ==============================================
    // Instantiate JTAG module
    JTAG #(
        .WIDTH(WIDTH)  // Set IR width
    ) jtag_inst (
        .TDI(TDI),
        .TMS(TMS),
        .TCK(TCK),
        
        .IR_shift_out(IR_shift_out),            // IR shift output
        .Bypass_out(Bypass_out),                // Bypass Register output
        .ID_out(ID_out),                        // ID Register output(
        
        .state_out(state_out),
        .rst_out(rst_out),
        .probing_out(Instr)
    );
    
    //====================DEBUG MODULE==================================
    Debug_Module #(
        .DM_BASE_ADDR(32'h0)  // Example base address, change as needed
    ) debug_module_inst (
        .clk            (cpu_clock),
        .clk_jtag       (TCK),
        .rst_n_switch   (rst_out && ~RESET_BUTTON),
        .addr           (Instr),
        .TAP_state      (state_out),
        .TDI            (TDI),
        .TDO            (TDO),
        
        .IR_shift_out   (IR_shift_out),            // IR shift output
        .Bypass_out     (Bypass_out),                // Bypass Register output
        .ID_out         (ID_out),                         // ID Register output
        
        
        // From RISC-V Core
        .ptr_regno_data_riscv               (ptr_regno_data_riscv),
        .ptr_data1_data_riscv               (ptr_data1_data_riscv),
        .core_command_complete_riscv        (core_command_complete_riscv),
        .core_reg_access_complete_riscv     (core_reg_access_complete_riscv),
        .core_mem_access_complete_riscv     (core_mem_access_complete_riscv),
        .core_halted_riscv                  (core_halted_riscv),
        .core_running_riscv                 (core_running_riscv),
        .core_unavail_riscv                 (core_unavail_riscv),
        .core_exist_riscv                   (core_exist_riscv),
        .core_resumeack_riscv               (core_resumeack_riscv),
        .core_havereset_riscv               (core_havereset_riscv),
        .core_abstract_cmd_failed_riscv     (core_abstract_cmd_failed_riscv),
        .core_abstract_cmd_wrong_hart_state_riscv (core_abstract_cmd_wrong_hart_state_riscv),
        .core_abstract_cmd_exception_riscv  (core_abstract_cmd_exception_riscv),
        .core_abstract_cmd_bus_error_riscv  (core_abstract_cmd_bus_error_riscv),
        .core_PC_debug_riscv                (core_PC_debug_riscv[7:0]),
    
        // To RISC-V Core
        .to_core_haltreq              (to_core_haltreq),
        .to_core_resumereq            (to_core_resumereq),
        .to_core_hartreset            (to_core_hartreset),
        .to_core_ackhavereset         (to_core_ackhavereset),
        .to_core_ackunavail           (to_core_ackunavail),
        .to_core_resethaltreq_reg     (to_core_resethaltreq_reg),
        .to_core_ndmreset             (to_core_ndmreset),
        .to_core_data0                (to_core_data0),
        .to_core_data1                (to_core_data1),
        .to_core_instr_bus            (to_core_instr_bus),
        .to_core_new_cmd_flag         (to_core_new_cmd_flag),
        .to_core_cmd_not_supported    (to_core_cmd_not_supported),
        
        //troubleshooting
        .data_0_probe(data_0_probe),
        .data_display(data_display)
    );

    //`undef DEBUG_RESET
    `define DEBUG_RESET
   //===============RISCV CPU Core========================= 
    cpu u_riscv_core (
        .clk(cpu_clock),      // Should be 100MHz
        `ifdef DEBUG_RESET
          .RESET_BUTTON((RESET_BUTTON | ~to_core_hartreset) & ~DEBUG_OVERWRITE),  // Dm should be able to reset it as well
        `else
          .RESET_BUTTON(RESET_BUTTON),
        `endif

        .IADDR(IADDR),
        .IBURST(IBURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
        .IREQ(IREQ),
        .IWRB(IWRB),
        .IWDATA(IWDATA),
        .IRDATA(IRDATA),
        .IACK(IACK),
        .ISTALL(ISTALL),
        .IBSTROBE(IBSTROBE),
        .ITLAST(ITLAST),

        .DADDR(DADDR),
        .DBURST(DBURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
        .DREQ(DREQ),
        .DWRB(DWRB),
        .DWDATA(DWDATA),
        .DRDATA(DRDATA),
        .DACK(DACK),
        .DSTALL(DSTALL),
        .DBSTROBE(DBSTROBE),
        .DTLAST(DTLAST),

        .led(led),
   
        .cache_en(1'b0),
        .csr_pmp_sb(csr_pmp_sb),

        .irq_i(irq_i),

        .freeze_int_dcache(),

       `ifdef itlb_def
       .vpn_to_ppn_req(vpn_to_ppn_req),
       `endif 
        
        .dm_halt_addr(32'h17),
        .dm_exception_addr(32'h17),
        .pbuffer_execution(1'b0),

        .state(state),
        // From Debug Module to Core
        .haltreq_i(to_core_haltreq),
        .resumereq_i(to_core_resumereq),
        .resethaltreq_i(to_core_resethaltreq_reg),
        .ackhavereset(to_core_ackhavereset),
        .ackunavail_i(to_core_ackunavail),
        `ifdef DEBUG_RESET
          .ndmreset_i((~to_core_ndmreset) & ~DEBUG_OVERWRITE),
          .hartreset_i((~to_core_hartreset) & ~DEBUG_OVERWRITE),
        `else
          .ndmreset_i(1'b0),
          .hartreset_i(1'b0),
        `endif

        .data0_i(to_core_data0),
        .data1_i(to_core_data1),
        .instr_bus(to_core_instr_bus),
        .new_cmd_flag_i(to_core_new_cmd_flag),
        .cmd_not_supported_i(to_core_cmd_not_supported),
    
        // From Core to Debug Module
        .ptr_regno_data_o(ptr_regno_data_riscv),
        .ptr_data1_data_o(ptr_data1_data_riscv),
        .command_complete_o(core_command_complete_riscv),
        .reg_access_complete_o(core_reg_access_complete_riscv),
        .mem_access_complete_o(core_mem_access_complete_riscv),
        .core_halted_o(core_halted_riscv),
        .core_running_o(core_running_riscv),
        .core_unavail_o(core_unavail_riscv),
        .core_exist_o(core_exist_riscv),
        .core_resumeack_o(core_resumeack_riscv),
        .core_havereset_o(core_havereset_riscv),
        .abstract_cmd_failed_o(core_abstract_cmd_failed_riscv),
        .abstract_cmd_wrong_hart_state_o(core_abstract_cmd_wrong_hart_state_riscv),
        .abstract_cmd_exception_o(core_abstract_cmd_exception_riscv),
        .abstract_cmd_bus_error_o(core_abstract_cmd_bus_error_riscv),
        .PC_debug(core_PC_debug_riscv)    
    ); 
    
endmodule
