`timescale 1ns / 1ps
`include "defines.v"

(* keep_hierarchy = "yes" *)
module fet_dec_ex_mem(

input RESET_BUTTON,
input clk,

output  [31:0]  IADDR,
output  [1:0]   IBURST, //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
output          IREQ,
output          IWRB,
output  [31:0]  IWDATA,
input   [31:0]  IRDATA,
input           IACK,
input           ISTALL,
input           ITLAST,
output  [3:0]   IBSTROBE,

output  [31:0]  DADDR,
output  [1:0]   DBURST, //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
output          DREQ,
output          DWRB,
output  [31:0]  DWDATA,
input   [31:0]  DRDATA,
input           DACK,
input           DSTALL,
input           DTLAST,
output  [3:0]   DBSTROBE,
input cache_en,

//Debug Interface
output [31:0] PC_DEBUG,
input [31:0] instruction_DM,
output freeze_int_dcache,

//Inputs from Debug Module
output [2:0] state,
input haltreq_i,
input resumereq_i,
input resethaltreq_i,
input ackhavereset,
input ackunavail_i,
input ndmreset_i,
input [31:0] data0_i,
input [31:0] data1_i,
input cmd_not_supported_i,
input new_cmd_flag_i,
input hartreset_i,

//Outputs to Debug Module
output [31:0] ptr_regno_data_o,
output [31:0] ptr_data1_data_o,
output command_complete_o,
output reg_access_complete_o,
output mem_access_complete_o,
output core_halted_o,
output core_running_o,
output core_resumeack_o,
output core_havereset_o,
output core_unavail_o,
output core_exist_o,
output abstract_cmd_failed_o,
output abstract_cmd_wrong_hart_state_o,
output abstract_cmd_exception_o,
output abstract_cmd_bus_error_o,

input [31:0] dm_halt_addr,dm_exception_addr,


input pbuffer_execution,
output [63:0] led,



output [`CSR_SB_W-1:0] csr_pmp_sb,
`ifdef itlb_def
output vpn_to_ppn_req,
`endif 

input [31:0] irq_i
);

wire addr_exception;
//wire [63:0] led;
wire lsustall_int;
wire [4:0] lsuop_int;
wire [31:0] store_data_int;
wire [31:0] wb_data_int;
wire ll_int;
wire sc_int;
wire icache_en_o;
wire [31:0] instruction;
wire [31:0] instruction_ICACHE;

wire imem_allow;
wire dmem_allow;

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
//assign instruction = instruction_ICACHE; 
assign RESET = RESET_BUTTON | core_dbg_reset | hartreset_i;
//assign RESET = RESET_BUTTON;

wire load_access_fault,store_access_fault;
wire load_page_fault,store_page_fault;


IF_ID_EX Pipeline( .CLK(clk),
                   .RESET_BUTTON(RESET_BUTTON | hartreset_i),
                   //.ext_irq(ext_irq),
                   //.sw_irq(sw_irq),
                   //.timer_irq(timer_irq),
                   .Load_Store_Op__Port1(lsuop_int),
                   .proc_addr_port1(proc_addr_port1_int),
                   .badaddr_data(badaddr_data),
                   .Store_Data(store_data_int),
                   .dbg_memory_req(dbg_memory_req),
                   .lsustall_o(lsustall_int),
                   .Load__Stall(stall),
                   .load_access_fault(load_access_fault),
                   .store_access_fault(store_access_fault),
                   .load_page_fault(load_page_fault),
                   .store_page_fault(store_page_fault),

                   //Debug Interface
                
                   .state(state),
                   .haltreq_i(haltreq_i),
                   .resumereq_i(resumereq_i),
                   .resethaltreq_i(resethaltreq_i),
                   .ackhavereset(ackhavereset),
                   .ackunavail_i(ackunavail_i),
                   .ndmreset_i(ndmreset_i),
                   .cmd_not_supported_i(cmd_not_supported_i),
                   .new_cmd_flag_i(new_cmd_flag_i),
                   .data0_i(data0_i),
                   .data1_i(data1_i),

                   .ptr_data1_data_o(ptr_data1_data_o),
                   .ptr_regno_data_o(ptr_regno_data_o),
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

                   .debug_stall(debug_stall),
                   .debug_mode(debug_mode),

                   .core_dbg_reset(core_dbg_reset),
                   .dm_halt_addr(dm_halt_addr),
                   .dm_exception_addr(dm_exception_addr),
                   .pbuffer_execution(pbuffer_execution),

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
                   .addr_exception(addr_exception),
                   .irq_i(irq_i),
                   .cache_flush_csr(cache_flush_csr),
                   .flush_csr_clr(flush_csr_clr),
                   .csr_satp(csr_satp),
                   .led(led),
                   .csr_pmp_sb(csr_pmp_sb)
                   
                   
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
                .cache_dis(dbg_memory_req),
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
                .TLAST(DTLAST),
                .csr_satp(csr_satp),
                .dtlb_trans_off(dtlb_trans_off),
                .lsu_op_port1(lsuop_int),
                .lsu_op_port2(lsu_op_port2_int),
                .dcache_freeze(icache_freeze | stall_mul | FPU__Stall),
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
                .load_access_fault(load_access_fault),
                .store_access_fault(store_access_fault),
                .load_page_fault(load_page_fault),
                .store_page_fault(store_page_fault),
                .csr_pmp_sb(csr_pmp_sb),
                .dmem_allow(dmem_allow),

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
            .core_resumeack_i(core_resumeack_o),
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
            .TLAST(ITLAST),
            .BSTROBE(IBSTROBE),
            .csr_satp(csr_satp)
            `ifdef itlb_def
             ,.tlb_trans_off(tlb_trans_off)
            ,.vpn_to_ppn_req_in(vpn_to_ppn_req)
            ,.imem_allow(imem_allow)
            ,.csr_pmp_sb(csr_pmp_sb)
            `endif  
            );

wire ADDR_CHK = ((pc_cache == 32'h0000887C)) ? 1'b1 : 1'b0;

wire ADDR_CHK_2 = ((pc_cache == 32'h00008970)) ? 1'b1 : 1'b0;


//assign led = {pc_cache,proc_addr_port1_int};
//assign led = 64'b0;

//ila_0 debugger1( .clk(clk),.probe0(pc_cache),.probe1(instruction),.probe2({30'd0,imem_allow,icache_freeze}),.probe3(IADDR),.probe4(IRDATA));

endmodule
