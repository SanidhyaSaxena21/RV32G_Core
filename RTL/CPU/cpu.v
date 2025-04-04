`timescale 1ns / 1ps
`include "/home/rclab/FINAL_PROJECT/RV32G_Core/RV32G_Core/RTL/DEFINES/defines.v"

module cpu(clk,clk_x2,rst,led,ext_irq,sw_irq,timer_irq,csr_pmp_sb,
           /*wb_ack_i,wb_err_i, wb_rty_i, wb_dat_i,wb_cyc_o, wb_adr_o, wb_stb_o, 
           wb_we_o, wb_sel_o, wb_dat_o,wb_cti_o, wb_bte_o,*/ 
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
);

input clk;
input clk_x2;
input rst;

output [63:0] led;

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

output  [31:0]  IADDR;
output  [1:0]   IBURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
output          IREQ;
output          IWRB;
output  [31:0]  IWDATA;
input      [31:0]  IRDATA;
input              IACK;
input              ISTALL;
output  [3:0]   IBSTROBE;

output  [31:0]  DADDR;
output  [1:0]   DBURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
output          DREQ;
output          DWRB;
output  [31:0]  DWDATA;
input      [31:0]  DRDATA;
input              DACK;
input              DSTALL;
output  [3:0]   DBSTROBE;

input  wire                    ext_irq;
input  wire                    sw_irq;
input  wire                    timer_irq;
//output [31:0] out_t0;
//output [31:0] out_t1;
//output [31:0] out_t2;

input cache_en;
output [`CSR_SB_W-1:0] csr_pmp_sb;

`ifdef itlb_def
output vpn_to_ppn_req;
`endif 

`ifdef TEST
output [31:0] block_instr_int;
`endif

output tick_en;
output addr_exception;

input [31:0] interrupt;


wire [31:0] instruction_int;
wire wb_op_o_int;
wire wb_stall_o_int;
wire [31:0] wb_data_o_int;
wire instr_clk;

`ifdef TEST
assign block_instr_int = instruction_int;
`endif

//`ifdef rom
//blk_mem_gen_v7_3_1 block_mem(
// .clka(clk), // input clka
// .ena(~icache_freeze & ~stall_mul_int & ~freeze_int), // input ena   ; active low for simulating freeze signal
// .addra(pc_cache_int), // input [31 : 0] addra
// .douta(instruction_int) // output [31 : 0] douta
//);
//`endif

//irq_interface ii(
//.clk(clk),
//.rst(rst),
//.stall_mul(stall_mul_int),
//.freeze(freeze_int),
//.icache_freeze(icache_freeze),          // added on 22/11/2016    because count1 and 2 were enable and interface inst were getting executed becuase of i cache miss
//.irq(irq),
//.eret(eret),
//.irq_ack(irq_ack),
//.eret_ack(eret_ack),
//.inst_inj(inst_inj),
//.irq_ctrl(irq_ctrl),
//.irq_ctrl_wb(irq_ctrl_wb_i),
//.irq_if_ctrl(irq_if_ctrl),
//.irq_ctrl_dec_src1(irq_ctrl_dec_src1),
//.irq_ctrl_dec_src2(irq_ctrl_dec_src2),
//.if_id_freeze(if_id_freeze_irq),
//.irq_icache_freeze(irq_icache_freeze)
//);

fet_dec_ex_mem fdem( .rst(rst),.clk(clk),.clk_x2(clk_x2),.led(led),.tick_en(tick_en),.addr_exception(addr_exception)
                      ,.ext_irq(ext_irq),.sw_irq(sw_irq),.timer_irq(timer_irq)
                     ,.interrupt(interrupt),.clmode(2'b00),.cache_en(cache_en),.csr_pmp_sb(csr_pmp_sb)
                     //,.wb_ack_i(wb_ack_i),.wb_err_i(wb_err_i),.wb_rty_i(wb_rty_i),.wb_dat_i(wb_dat_i)
                     //,.wb_cyc_o(wb_cyc_o),.wb_adr_o(wb_adr_o),.wb_stb_o(wb_stb_o),.wb_we_o(wb_we_o)
                     //,.wb_sel_o(wb_sel_o),.wb_dat_o(wb_dat_o),.wb_cti_o(wb_cti_o),.wb_bte_o(wb_bte_o)
                     ,.DADDR(DADDR),.DBURST(DBURST),.DREQ(DREQ),.DWRB(DWRB),.DWDATA(DWDATA),.DRDATA(DRDATA),.DACK(DACK),.DSTALL(DSTALL),.DBSTROBE(DBSTROBE)
                     ,.IADDR(IADDR),.IBURST(IBURST),.IREQ(IREQ),.IWRB(IWRB),.IWDATA(IWDATA),.IRDATA(IRDATA),.IACK(IACK),.ISTALL(ISTALL),.IBSTROBE(IBSTROBE)
                    `ifdef itlb_def
                    ,.vpn_to_ppn_req(vpn_to_ppn_req)
                    `endif  
                    );


endmodule



