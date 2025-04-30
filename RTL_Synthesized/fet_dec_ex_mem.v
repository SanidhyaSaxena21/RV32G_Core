`timescale 1ns / 1ps
`include "defines.v"

(* keep_hierarchy = "yes" *)
module fet_dec_ex_mem(
// master inputs
//rst,clk,clk_x2,led,
//wb_ack_i, wb_err_i, wb_rty_i, wb_dat_i,wb_cyc_o, wb_adr_o, wb_stb_o, wb_we_o, wb_sel_o, wb_dat_o,wb_cti_o, wb_bte_o,
//clmode,cache_flush,cache_en,tick_en, addr_exception,ADDR,BURST,REQ,WRB,WDATA,RDATA,ACK,STALL,BSTROBE,
//interrupt
//,out_t0,out_t1,out_t2,sp

input RESET_BUTTON,
input clk,
//output [63:0] led,
input [1:0] clmode,

/*input [1:0] wb_ack_i,
input [1:0] wb_err_i,
input [1:0] wb_rty_i,
input [63:0] wb_dat_i,
output [1:0] wb_cyc_o,
output [63:0] wb_adr_o,
output [1:0] wb_stb_o,
output [1:0] wb_we_o,
output [7:0] wb_sel_o,
output [63:0] wb_dat_o,
output [5:0] wb_cti_o,
output [3:0] wb_bte_o,*/

output  [31:0]  IADDR,
output  [1:0]   IBURST, //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
output          IREQ,
output          IWRB,
output  [31:0]  IWDATA,
input   [31:0]  IRDATA,
input           IACK,
input           ISTALL,
output  [3:0]   IBSTROBE,

output  [31:0]  DADDR,
output  [1:0]   DBURST, //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
output          DREQ,
output          DWRB,
output  [31:0]  DWDATA,
input   [31:0]  DRDATA,
input           DACK,
input           DSTALL,
output  [3:0]   DBSTROBE,
input ext_irq,
input sw_irq,
input timer_irq,
input cache_en,

//Debug Interface
output [31:0] PC_DEBUG,
input [31:0] instruction_DM,
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
output abstract_cmd_bus_error,


output [`CSR_SB_W-1:0] csr_pmp_sb,
`ifdef itlb_def
output vpn_to_ppn_req,
`endif 

output tick_en,
//output [31:0] sp,
output addr_exception,
input [31:0] interrupt
//`ifdef itlb_def
//,vpn_to_ppn_req
//`endif  
);

/*
input rst;
input clk;
input clk_x2;
output [63:0] led;
input [1:0] clmode; */

/*input [1:0] wb_ack_i;
input [1:0] wb_err_i;
input [1:0] wb_rty_i;
input [63:0] wb_dat_i;
output [1:0] wb_cyc_o;
output [63:0] wb_adr_o;
output [1:0] wb_stb_o;
output [1:0] wb_we_o;
output [7:0] wb_sel_o;
output [63:0] wb_dat_o;
output [5:0] wb_cti_o;
output [3:0] wb_bte_o;*/

/*
output  [31:0]  IADDR;
output  [1:0]   IBURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
output          IREQ;
output          IWRB;
output  [31:0]  IWDATA;
input   [31:0]  IRDATA;
input           IACK;
input           ISTALL;
output  [3:0]   IBSTROBE;

output  [31:0]  DADDR;
output  [1:0]   DBURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
output          DREQ;
output          DWRB;
output  [31:0]  DWDATA;
input   [31:0]  DRDATA;
input           DACK;
input           DSTALL;
output  [3:0]   DBSTROBE;

input cache_flush;
input cache_en;

`ifdef itlb_def
output vpn_to_ppn_req;
`endif 

output tick_en;
//output [31:0] sp;
output addr_exception;
input [31:0] interrupt;*/

wire lsustall_int;
wire [4:0] lsuop_int;
wire [31:0] store_data_int;
wire [31:0] wb_data_int;
wire ll_int;
wire sc_int;
wire icache_en_o;
wire [31:0] instruction;
wire [31:0] instruction_ICACHE;


///dcache signal
wire [31:0] proc_addr_port1_int;
wire [31:0] proc_data_port1_int;
wire [31:0] amo_load_val_i;
wire [31:0] proc_addr_port2_int;
wire [4:0] lsu_op_port2_int;

wire [63:0] RF_value;

wire FPU__Stall;

wire cache_flush_csr;
wire tlb_trans_off;
wire [31:0] csr_satp;
wire dtlb_trans_off;

assign dtlb_trans_off = tlb_trans_off;

wire instruction_page_fault;
wire data_page_fault;
wire badaddr_data;

wire icache_freeze;
wire flush_csr_clr;
wire stall_mul;
//wire freeze_int_dcache;
wire stall;
wire debug_stall;
wire debug_mode;
wire [31:0] pc_cache;
wire core_dbg_reset;
wire RESET;
//Mux to select instruction should come from the Debug Module or From Instruction memory

assign PC_DEBUG = pc_cache;
assign instruction = (debug_mode) ? instruction_DM : instruction_ICACHE; 
assign RESET = RESET_BUTTON | core_dbg_reset;

IF_ID_EX Pipeline( .CLK(clk),
                   .RESET_BUTTON(RESET_BUTTON),
                   .ext_irq(ext_irq),
                   .sw_irq(sw_irq),
                   .timer_irq(timer_irq),
                   .Load_Store_Op__Port1(lsuop_int),
                   .proc_addr_port1(proc_addr_port1_int),
                   .badaddr_data(badaddr_data),
                   .Store_Data(store_data_int),
                   .lsustall_o(lsustall_int),
                   .Load__Stall(stall),
                   //Debug Interface
                
                   .core_dbg_reset(core_dbg_reset), 
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
                   .debug_stall(debug_stall),
                   .debug_mode(debug_mode),

                   //Dcache Interface
                   .Data_Cache__Stall(freeze_int_dcache), 
                   .proc_data_port1_int(proc_data_port1_int),
                   .lsu_op_port2(lsu_op_port2_int),
                   .proc_addr_port2(proc_addr_port2_int),
                   .amo_load_val_i(amo_load_val_i),

                   //Instruction Cache interafce
                   .Inst_Cache_Freeze(icache_en_o),
                   .pc_cache(pc_cache),
                   .Inst_Cache__Stall(icache_freeze),
                   .Instruction__IF_ID(instruction),
                   .instruction_page_fault(instruction_page_fault),
                   .data_page_fault(data_page_fault),
                   .LR_Inst(ll_int),
                   .SC_Inst(sc_int),
                   .Mult_Div_unit__Stall(stall_mul),
                   .FPU__Stall(FPU__Stall),
                   //.eret_ack(eret_ack),
                   .tick_en(tick_en),
                   .addr_exception(addr_exception),
                   .interrupt(interrupt),
                   .cache_flush_csr(cache_flush_csr),
                   .flush_csr_clr(flush_csr_clr),
                   .csr_satp(csr_satp),
                   .csr_pmp_sb(csr_pmp_sb),
                   .led()
                   
                   
                   `ifdef itlb_def
                     ,.tlb_trans_off(tlb_trans_off)
                   ,.vpn_to_ppn_req(vpn_to_ppn_req)
                   `endif  
                   );


dcache_biu db1( //wishbone and controller interfacee I/Os
                .proc_clk(clk),
                //.clk_x2(clk_x2),
                .proc_rst(RESET),
                .lsustall(lsustall_int),
                .cache_en(cache_en),
                .clmode(clmode),
                .badaddr_data(badaddr_data),
                .ADDR(DADDR),
                .BURST(DBURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
                .REQ(DREQ),
                .WRB(DWRB),
                .WDATA(DWDATA),
                .RDATA(DRDATA),
                .ACK(DACK),
                .STALL(DSTALL),
                .BSTROBE(DBSTROBE),
                .csr_satp(csr_satp),
                .dtlb_trans_off(dtlb_trans_off),
                .lsu_op_port1(lsuop_int),
                .lsu_op_port2(lsu_op_port2_int),
                .dcache_freeze(icache_freeze | stall_mul | FPU__Stall | debug_stall),
                .proc_data_in_port1(store_data_int),
                .proc_data_in_port2(32'b0),
                .proc_addr_in_port1(proc_addr_port1_int),
                .proc_addr_in_port2(proc_addr_port2_int),
                .freeze(freeze_int_dcache),
                .proc_data_port1(proc_data_port1_int),
                .proc_data_port2(amo_load_val_i),
                .ll_i(ll_int),
                .sc_i(sc_int),
                .data_page_fault(data_page_fault),
                .addr_exception(addr_exception)
                //.csr_satp(csr_satp)
                /////////////////////////
                //Cache Flushing Currently under test                
                `ifdef CACHE_FLUSH_TEST
                ,.cache_flush(cache_flush_csr)
                ,.flush_csr_clr(flush_csr_clr)
                `else
                ,.cache_flush(cache_flush_csr)
                ,.flush_csr_clr(flush_csr_clr)
                `endif
                /////////////////////////////////////
                );

mem_hier mh(
            .clk(clk),
            //.clk_x2(clk_x2),
            .reset(RESET),
            .freeze_in(icache_en_o),
            .i_addr({pc_cache[31:2],2'b00}),
            .instr_out(instruction_ICACHE),
            .stall_out(icache_freeze),
            //.eret_ack(eret_ack),
            .stall_load(stall),
            .instruction_page_fault(instruction_page_fault),
            .ADDR(IADDR),
            .BURST(IBURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
            .REQ(IREQ),
            .WRB(IWRB),
            .WDATA(IWDATA),
            .RDATA(IRDATA),
            .ACK(IACK),
            .STALL(ISTALL),
            .BSTROBE(IBSTROBE),
            .csr_satp(csr_satp)
            `ifdef itlb_def
             ,.tlb_trans_off(tlb_trans_off)
            ,.vpn_to_ppn_req_in(vpn_to_ppn_req)
            `endif  
            );

wire ADDR_CHK = ((pc_cache == 32'h0000887C)) ? 1'b1 : 1'b0;

wire ADDR_CHK_2 = ((pc_cache == 32'h00008970)) ? 1'b1 : 1'b0;


//assign led = {pc_cache,proc_addr_port1_int};
//assign led = 64'b0;

//ila_0 debugger( .clk(clk),.probe0(pc_cache),.probe1(instruction),.probe2(proc_addr_port1_int),.probe3(RF_value[31:0]),.probe4(freeze_int),.probe5(icache_freeze));

endmodule
