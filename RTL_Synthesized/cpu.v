`timescale 1ns / 1ps
`include "defines.v"

(* keep_hierarchy = "yes" *)
module cpu 
(
input clk,
input RESET_BUTTON,

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

//output [31:0] out_t0,
//output [31:0] out_t1,
//output [31:0] out_t2,

input cache_en,
output [`CSR_SB_W-1:0] csr_pmp_sb,

`ifdef itlb_def
output vpn_to_ppn_req,
`endif 

output addr_exception,

input [31:0] irq_i,
output freeze_int_dcache,

//Debug Interface
//input [31:0] instruction_DM,
//output [31:0] PC_DEBUG,

//Inputs from Debug Module

output [2:0] state,
input haltreq_i,
input resumereq_i,
input resethaltreq_i,
input hartreset_i,
input ackhavereset,
input ackunavail_i,
input ndmreset_i,
input cmd_not_supported_i,
input new_cmd_flag_i,
input [31:0] instr_bus,
input [31:0] data0_i,
input [31:0] data1_i,

//Outputs to Debug Module
output core_halted_o,
output core_running_o,
output core_resumeack_o,
output core_unavail_o,
output core_exist_o,
output core_havereset_o,
output [31:0] PC_debug,
output abstract_cmd_failed_o,
output abstract_cmd_wrong_hart_state_o,
output abstract_cmd_exception_o,
output abstract_cmd_bus_error_o,
output [31:0] ptr_regno_data_o,  
output [31:0] ptr_data1_data_o,  
output command_complete_o,
output reg_access_complete_o,
output mem_access_complete_o,

input [31:0] dm_halt_addr,dm_exception_addr,
output [63:0] led,

input pbuffer_execution

);



fet_dec_ex_mem fdem( .RESET_BUTTON(RESET_BUTTON),.clk(clk),
                     .irq_i(irq_i),.cache_en(cache_en),.csr_pmp_sb(csr_pmp_sb),.led(led)
                     ,.DADDR(DADDR),.DBURST(DBURST),.DREQ(DREQ),.DWRB(DWRB),.DWDATA(DWDATA),.DRDATA(DRDATA),.DACK(DACK),.DSTALL(DSTALL),.DTLAST(DTLAST),.DBSTROBE(DBSTROBE)
                     ,.IADDR(IADDR),.IBURST(IBURST),.IREQ(IREQ),.IWRB(IWRB),.IWDATA(IWDATA),.IRDATA(IRDATA),.IACK(IACK),.ISTALL(ISTALL),.ITLAST(ITLAST),.IBSTROBE(IBSTROBE), .freeze_int_dcache(freeze_int_dcache),

                      .instruction_DM(instr_bus),
                      .PC_DEBUG(PC_debug),

                      //Inputs from Debug Module
                      .state(state),
                      .haltreq_i(haltreq_i),
                      .resumereq_i(resumereq_i),
                      .resethaltreq_i(resethaltreq_i),
                      .ackhavereset(ackhavereset),
                      .ackunavail_i(ackunavail_i),
                      .ndmreset_i(ndmreset_i),
                      .data0_i(data0_i),
                      .data1_i(data1_i),
                      .cmd_not_supported_i(cmd_not_supported_i),
                      .new_cmd_flag_i(new_cmd_flag_i),
                      .hartreset_i(hartreset_i),


                      //Output to Debug Module
                      .ptr_regno_data_o(ptr_regno_data_o),
                      .ptr_data1_data_o(ptr_data1_data_o),
                      .command_complete_o(command_complete_o),
                      .reg_access_complete_o(reg_access_complete_o),
                      .mem_access_complete_o(mem_access_complete_o),
                      .core_halted_o(core_halted_o),
                      .core_running_o(core_running_o),
                      .core_resumeack_o(core_resumeack_o),
                      .core_havereset_o(core_havereset_o),
                      .core_unavail_o(core_unavail_o),
                      .core_exist_o(core_exist_o),
                      .abstract_cmd_failed_o(abstract_cmd_failed_o),
                      .abstract_cmd_wrong_hart_state_o(abstract_cmd_wrong_hart_state_o),
                      .abstract_cmd_exception_o(abstract_cmd_exception_o),
                      .abstract_cmd_bus_error_o(abstract_cmd_bus_error_o),

                      .dm_halt_addr(dm_halt_addr),
                      .dm_exception_addr(dm_exception_addr),
                      
                      .pbuffer_execution(pbuffer_execution)
                      
                       
                    `ifdef itlb_def
                    ,.vpn_to_ppn_req(vpn_to_ppn_req)
                    `endif  
                    );


endmodule



