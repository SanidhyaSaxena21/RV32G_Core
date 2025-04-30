`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/04/2025 11:50:35 PM
// Design Name: 
// Module Name: riscv_platform
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

module riscv_platform (
  input clk_int,
  input rst,

  input RTC_CLOCK,
  input ext_irq,

  input cache_en_int,

  output tick_en,
  output addr_exception,
  input [31:0] interrupt
  `ifdef itlb_def
  ,output vpn_to_ppn_req
  `endif 
  
  `ifdef TEST
  ,output [31:0] block_instr_int
  `endif

    );

      //-------------DATA MEMORY INTERFACE---------------------
   wire  [31:0]  DADDR;
   wire  [1:0]   DBURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
   wire          DREQ;
   wire          DWRB;
   wire [31:0]  DWDATA;
   wire      [31:0]  DRDATA;
   wire              DACK;
   wire              DSTALL;
   wire [3:0]   DBSTROBE;

  //-------------INSTRUCTION MEMORY INTERFACE---------------------
   wire  [31:0]  IADDR;
   wire  [1:0]   IBURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
   wire          IREQ;
   wire          IWRB;
   wire [31:0]  IWDATA;
   wire      [31:0]  IRDATA;
   wire              IACK;
   wire              ISTALL;
   wire [3:0]   IBSTROBE;



  //-------------MEMORY INTERFACE---------------------
   wire  [31:0]  ADDR;
   wire  [1:0]   BURST; //00-Normal; 01-NCR; 10-WRAP; 11-Reserved
   wire          REQ;
   wire          WRB;
   wire [31:0]  WDATA;
   wire      [31:0]  RDATA;
   wire              ACK;
   wire              STALL;
   wire [3:0]   BSTROBE;

   wire [31:0] s_RDATA;
   wire [31:0] m_RDATA;
   wire s_ACK;
   wire s_STALL;
   wire m_ACK;
   wire m_STALL;
   wire clint_en;

   wire timer_irq;
   wire sw_irq;
   wire [`CSR_SB_W-1:0] csr_pmp_sb;

   wire [63:0] lcd_reg;
   wire [3:0] imem_allow,dmem_allow; // L/X/W/R

   //Debug Module Interface 
   wire   haltreq_i;
   wire   resumereq_i;
   wire   resethaltreq_i;

   wire  core_havereset_o;
   wire   ackhavereset;

   //TODO: How to use them
   wire  core_unavail_o;
   wire   ackunavail;

   wire  core_halted_o;
   wire  core_running_o;
   wire  core_resumeack_o;

   //Address for Debugger
   wire [31:0] dm_exception_addr;
   wire [31:0] dm_halt_addr;

   wire ndmreset;

   wire new_cmd_flag_i;
   wire cmd_not_supported_i;
   

   //Command Success signals to Debug Module
   wire command_complete_o;
   wire reg_access_complete_o;
   wire mem_access_complete_o;
   
   //Command Failure Signals to Debug Module
   wire abstract_cmd_fail_o;
   wire abstract_cmd_wrong_hart_state_o;
   wire abstract_cmd_exception_o;
   wire abstract_cmd_bus_error;


   wire [31:0] PC_DEBUG;
   wire [31:0] instruction_DM;

   wire freeze_int_dcache;

   wire pbuffer_execution;

       cpu cpu1(.clk(clk_int),/*.clk_x2(clk_x2),*/.RESET_BUTTON(rst), .ext_irq(ext_irq), .timer_irq(timer_irq), .sw_irq(sw_irq),
           .cache_en(cache_en_int),.tick_en(tick_en),.csr_pmp_sb(csr_pmp_sb),
           .addr_exception(addr_exception),.interrupt(interrupt),

           //Debug Interface
           .instruction_DM(instruction_DM),
           .PC_DEBUG(PC_DEBUG),
           .freeze_int_dcache(freeze_int_dcache),
           .haltreq_i(haltreq_i),
           .resumereq_i(resumereq_i),
           .resethaltreq_i(resethaltreq_i),
           .core_havereset_o(core_havereset_o),
           .ackhavereset(ackhavereset),
           .ackunavail(ackunavail),
           .pbuffer_execution(pbuffer_execution),
           .core_halted_o(core_halted_o),.core_running_o(core_running_o),.core_resumeack_o(core_resumeack_o),
           .ndmreset(ndmreset),.cmd_not_supported_i(cmd_not_supported_i),.new_cmd_flag_i(new_cmd_flag_i),
           .command_complete_o(command_complete_o),.reg_access_complete_o(reg_access_complete_o),.mem_access_complete_o(mem_access_complete_o),
           .dm_halt_addr(dm_halt_addr),
           .dm_exception_addr(dm_exception_addr),
           .abstract_cmd_fail_o(abstract_cmd_fail_o),
           .abstract_cmd_wrong_hart_state_o(abstract_cmd_wrong_hart_state_o),
           .abstract_cmd_exception_o(abstract_cmd_exception_o),
           .abstract_cmd_bus_error(abstract_cmd_bus_error),
           

          .IADDR(IADDR),
          .IBURST(IBURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
          .IREQ(IREQ),
          .IWRB(IWRB),
          .IWDATA(IWDATA),
          .IRDATA(IRDATA),
          .IACK(IACK),
          .ISTALL(ISTALL),
          .IBSTROBE(IBSTROBE),

          .DADDR(DADDR),
          .DBURST(DBURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
          .DREQ(DREQ),
          .DWRB(DWRB),
          .DWDATA(DWDATA),
          .DRDATA(DRDATA),
          .DACK(DACK),
          .DSTALL(DSTALL),
          .DBSTROBE(DBSTROBE)
       `ifdef TEST
       ,.block_instr_int(blck_instr_int)
       `endif
       `ifdef itlb_def
       ,.vpn_to_ppn_req(vpn_to_ppn_req)
       `endif  
       );

  
       Debug_Module DEBUG_MODULE (
           .clk(clk_int),
           .reset(rst),
           //Debug Interface
           .INSTRUCTION_DEBUG(instruction_DM),
           .PC_DEBUG(PC_DEBUG),
           .freeze_int_dcache(freeze_int_dcache),
           .haltreq_i(haltreq_i),
           .resumereq_i(resumereq_i),
           .resethaltreq_i(resethaltreq_i),
           .core_havereset_o(core_havereset_o),
           .ackhavereset(ackhavereset),
           .ackunavail(ackunavail),
           .core_halted_o(core_halted_o),.core_running_o(core_running_o),.core_resumeack_o(core_resumeack_o),
           .ndmreset(ndmreset),.cmd_not_supported_i(cmd_not_supported_i),.new_cmd_flag_i(new_cmd_flag_i),
           .command_complete_o(command_complete_o),.reg_access_complete_o(reg_access_complete_o),.mem_access_complete_o(mem_access_complete_o),
           .pbuffer_execution(pbuffer_execution),
           .dm_halt_addr(dm_halt_addr),
           .dm_exception_addr(dm_exception_addr),
           .abstract_cmd_fail_o(abstract_cmd_fail_o),
           .abstract_cmd_wrong_hart_state_o(abstract_cmd_wrong_hart_state_o),
           .abstract_cmd_exception_o(abstract_cmd_exception_o),
           .abstract_cmd_bus_error(abstract_cmd_bus_error)
       );

       rv32_mpu #(.ILEN(),.XLEN(), .MPU_SUPPORT(), .NB_PMP_REGION(),.MAX_PMP_REGION(),.MMU_SUPPORT(0)) IMEM_MPU (
        .aclk(clk_int),
        .aresetn(~rst),
        .imem_addr(IADDR),
        .imem_allow(imem_allow),
        .dmem_addr(DADDR),
        .dmem_allow(dmem_allow),
        .csr_sb(csr_pmp_sb)
       );
      
      assign dmem_read_allowed = (dmem_allow == 4'd5); 
      assign dmem_write_allowed = (dmem_allow == 4'd6); 
      assign dmem_read_write_allowed = (dmem_allow == 4'd7);

      assign imem_read_allowed = (dmem_allow == 4'd5); 
      assign imem_write_allowed = (dmem_allow == 4'd6); 
      assign imem_read_write_allowed = (dmem_allow == 4'd7);

       interconnect interconnect (
        .clk(clk_int),
        .reset(rst),
        

    // Instruction Bus Interface
        .instr_req(IREQ && imem_read_write_allowed),        // Instruction request signal
        .instr_addr(IADDR),       // Instruction address bus
        .instr_burst(IBURST),
        .instr_write(IWRB),
        .instr_write_data(IWDATA),
        .instr_bstrobe(IBSTROBE), 

        .instr_data(IRDATA),       // Instruction read data
        .instr_ready(IACK),      // Instruction bus ready signal
        .instr_stall(ISTALL),

    // Data Bus Interface
        .data_req(DREQ && dmem_read_write_allowed),         // Data request signal
        .data_write(DWRB),       // Data write enable
        .data_addr(DADDR),        // Data address bus
        .data_write_data(DWDATA),  // Data write data
        .data_burst(DBURST),
        .data_bstrobe(DBSTROBE),

        .data_read_data(m_RDATA),   // Data read data
        .data_ready(m_ACK),       // Data bus ready signal
        .data_stall(m_STALL),

    // Memory Interface (Output Interface to Memory)
        .mem_req(REQ),          // Memory request signal
        .mem_write(WRB),        // Memory write enable
        .mem_addr(ADDR),         // Memory address
        .mem_burst(BURST),
        .mem_bstrobe(BSTROBE),
        .mem_write_data(WDATA),   // Memory write data

        .mem_read_data(RDATA),    // Memory read data
        .mem_stall(STALL),
        .mem_ready(ACK)         // Memory ready signal
       
      
      );


      assign clint_en = (DADDR[31:24]==`PERIPH_BASE);
      assign s_STALL = 1'b0;

      assign DRDATA   = (clint_en) ? s_RDATA    : m_RDATA;
      assign DACK     = (clint_en) ? s_ACK      : m_ACK;
      assign DSTALL   = (clint_en) ? s_STALL    : m_STALL;

    rv32_clint #(.ADDRW(32),.XLEN(32)) CLINT (
        .CLK(clk_int),
        .RST(rst),
        .s_en(DREQ),
        .s_wr(DWRB),
        .s_addr(DADDR),
        .s_wdata(DWDATA),
        .s_strb(BSTROBE),
        .s_rdata(s_RDATA),
        .s_ready(s_ACK),

        .rtc(RTC_CLOCK),
        .sw_irq(sw_irq),
        .timer_irq(timer_irq)
      );

     DATA_MEMORY #(.ADDR_WIDTH(14),.DATA_WIDTH(32)) Instruction_Memory (
        .clk(clk_int),
        .rst(rst),
        .ADDR(ADDR),
        .REQ(REQ),
        .WRB(WRB),
        .BSTROBE(BSTROBE),
        .WDATA(WDATA),
        .RDATA(RDATA),
        .BURST(BURST),
        .ACK(ACK),
        .STALL(STALL)
      );
endmodule
