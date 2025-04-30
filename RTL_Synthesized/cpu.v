`timescale 1ns / 1ps
`include "defines.v"

module cpu /*(clk,rst,ext_irq,sw_irq,timer_irq,csr_pmp_sb,
          DADDR,DBURST,DREQ,DWRB,DWDATA,DRDATA,DACK,DSTALL,DBSTROBE,
          IADDR,IBURST,IREQ,IWRB,IWDATA,IRDATA,IACK,ISTALL,IBSTROBE

`ifdef TEST
,block_instr_int
`endif
`ifdef itlb_def
,vpn_to_ppn_req
`endif  
,cache_en,tick_en,addr_exception
,interrupt
//,out_t0,out_t1,out_t2,sp
);*/
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
output  [3:0]   IBSTROBE,

output  [31:0]  DADDR,
output  [1:0]   DBURST, //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
output          DREQ,
output          DWRB,
output  [31:0]  DWDATA,
input      [31:0]  DRDATA,
input              DACK,
input              DSTALL,
output  [3:0]   DBSTROBE,

input  wire                    ext_irq,
input  wire                    sw_irq,
input  wire                    timer_irq,
//output [31:0] out_t0,
//output [31:0] out_t1,
//output [31:0] out_t2,

input cache_en,
output [`CSR_SB_W-1:0] csr_pmp_sb,

`ifdef itlb_def
output vpn_to_ppn_req,
`endif 

`ifdef TEST
output [31:0] block_instr_int,
`endif

output tick_en,
output addr_exception,

input [31:0] interrupt,

input [31:0] instruction_DM,
output [31:0] PC_DEBUG,
output freeze_int_dcache,
input haltreq_i,
input resumereq_i,
input resethaltreq_i,
output core_havereset_o,
input pbuffer_execution,
input ackhavereset,ackunavail,
output core_halted_o,core_running_o,core_resumeack_o,
input ndmreset,cmd_not_supported_i,new_cmd_flag_i,
output command_complete_o,reg_access_complete_o,mem_access_complete_o,
input [31:0] dm_halt_addr,dm_exception_addr,

output abstract_cmd_fail_o,
output abstract_cmd_wrong_hart_state_o,
output abstract_cmd_exception_o,
output abstract_cmd_bus_error );


wire [31:0] instruction_int;
wire wb_op_o_int;
wire wb_stall_o_int;
wire [31:0] wb_data_o_int;
wire instr_clk;

`ifdef TEST
assign block_instr_int = instruction_int;
`endif


fet_dec_ex_mem fdem( .RESET_BUTTON(RESET_BUTTON),.clk(clk),/*.led(led),*/.tick_en(tick_en),.addr_exception(addr_exception)
                      ,.ext_irq(ext_irq),.sw_irq(sw_irq),.timer_irq(timer_irq)
                     ,.interrupt(interrupt),.clmode(2'b00),.cache_en(cache_en),.csr_pmp_sb(csr_pmp_sb)
                     ,.DADDR(DADDR),.DBURST(DBURST),.DREQ(DREQ),.DWRB(DWRB),.DWDATA(DWDATA),.DRDATA(DRDATA),.DACK(DACK),.DSTALL(DSTALL),.DBSTROBE(DBSTROBE)
                     ,.IADDR(IADDR),.IBURST(IBURST),.IREQ(IREQ),.IWRB(IWRB),.IWDATA(IWDATA),.IRDATA(IRDATA),.IACK(IACK),.ISTALL(ISTALL),.IBSTROBE(IBSTROBE),
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
                      .abstract_cmd_bus_error(abstract_cmd_bus_error)
                       
                    `ifdef itlb_def
                    ,.vpn_to_ppn_req(vpn_to_ppn_req)
                    `endif  
                    );


endmodule



