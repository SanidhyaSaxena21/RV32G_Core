`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.12.2015 20:03:02
// Design Name: 
// Module Name: icache
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

module mem_hier(

      input clk,clk_x2, rst_n,
      //input i_acc;
      input [31:0]  i_addr,
      input freeze_in,
      input stall_load,
      /*
      input				wb_clk_i,	// clock input
      input				wb_rst_i,	// reset input
      input				wb_ack_i,	// normal termination
      input				wb_err_i,	// termination w/ error
      input				wb_rty_i,	// termination w/ retry
      input  [31:0]       wb_dat_i,
      output				wb_cyc_o,
      output              wb_stb_o,	// strobe output
      output              wb_we_o,	// indicates write transfer
      output [31:0] 		wb_adr_o,	
      output [1:0]        wb_bte_o,
      output [2:0] 		wb_cti_o,
      output [3:0]        wb_sel_o,	// byte select outputs for the signals-byte select and extend
      output [31:0]       wb_dat_o,	// output data bus*/
      
      output  [31:0]  ADDR,
      output  [1:0]   BURST, //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
      output          REQ,
      output          WRB,
      output  [31:0]  WDATA,
      input   [31:0]  RDATA,
      input           ACK,
      input           STALL,
      output  [3:0]   BSTROBE,

      input [31:0] csr_satp,
     
      output [31:0] instr_out,
      output instruction_page_fault,
      output stall_out

`ifdef itlb_def
,input tlb_trans_off
,input vpn_to_ppn_req_in
`endif
);

//----------------Parameters-----------------
parameter offset_start_bit = 0;
parameter page_offset_start = 0;
parameter page_offset_last = 11;
parameter offset_last_bit = 4;
parameter index_start_bit = 5;
parameter index_last_bit = 11;
parameter tag_start_bit = 12;
parameter tag_last_bit = 31;
parameter tag_tlb_start_bit = 4;
parameter tag_tlb_last_bit = 23;
parameter tag_phy_start_bit = 0;
parameter tag_phy_last_bit = 19;

/*
input clk,clk_x2, rst_n;
//input i_acc;
input [31:0]  i_addr;
input freeze_in;
input eret_ack;
input stall_load;
input				wb_clk_i;	// clock input
input				wb_rst_i;	// reset input
input				wb_ack_i;	// normal termination
input				wb_err_i;	// termination w/ error
input				wb_rty_i;	// termination w/ retry
input  [31:0]       wb_dat_i;
output				wb_cyc_o;
output              wb_stb_o;	// strobe output
output              wb_we_o;	// indicates write transfer
output [31:0] 		wb_adr_o;	
output [1:0]        wb_bte_o;
output [2:0] 		wb_cti_o;
output [3:0]        wb_sel_o;	// byte select outputs for the signals-byte select and extend
output [31:0]       wb_dat_o;	// output data bus
output [31:0] instr_out;
output stall_out;

`ifdef itlb_def
input vpn_to_ppn_req_in;
`endif */


// ---------Wire & Reg
wire				biu_we_i;	// WB write enable
wire				i_hit;	// cycle valid output
wire [255:0] bus_line;	// output data bus
wire bus_rdy;   //interface to the dcache unit

wire [1:0] state;
wire [31:0] i_addr_cache;

//reg [31:0]  i_addr_cache_my;
reg [31:0]  virtual_addr;
//reg stall_int;
wire stall;

wire [3:0] burst_len;
wire  i_we;
wire d_we,rdy;
wire [255:0]       m_data;
wire[255:0] set_0,set_1;
wire [18:0] tag_0,tag_1;
wire i_acc_int;
wire i_acc;
wire [31:0] 			biu_dat_o;
wire [31:0] instr;
reg [31:0] instr_reg;
wire [3:0] 	biu_sel_i;	// byte selects

//register to concatenate the words
wire [2:0] 			wb_cti_o;	// cycle type identifier
wire [1:0] 				wb_fsm_state_cur;
wire rd, wr,m_rdy_n;
wire [31:0] addr;
wire biu_cyc_i,biu_stb_i,biu_cab_i;

wire [255:0] bus_data;          
wire[255:0] i_data;
wire [31:0] addr_latch;
wire freeze;
wire x_freeze;
reg freeze_int;
reg stall_int;
reg stall_load_int;
wire vpn_to_ppn_req_in1;
wire vpn_to_ppn_req7;
reg freeze_hit_status;
wire [tag_phy_last_bit : tag_phy_start_bit] physical_tag;

/*wire				wb_cyc_o_int;
wire              wb_stb_o_int;	// strobe output
wire              wb_we_o_int;	// indicates write transfer
wire [31:0] 		wb_adr_o_int;	
wire [1:0]        wb_bte_o_int;
wire [2:0] 		wb_cti_o_int;
wire [3:0]        wb_sel_o_int;	// byte select outputs for the signals-byte select and extend
wire [31:0]       wb_dat_o_int;	// output data bus*/



wire  [31:0]  ICACHE_ADDR;
wire  [1:0]   ICACHE_BURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
wire          ICACHE_REQ;
wire          ICACHE_WRB;
wire  [31:0]  ICACHE_WDATA;
wire  [31:0]  ICACHE_RDATA;
wire          ICACHE_ACK;
wire          ICACHE_STALL;
wire  [3:0]   ICACHE_BSTROBE;
//-----------------IFDEF---------------------------------
`ifdef itlb_def
wire  [31:0]  TLB_ADDR;
wire  [1:0]   TLB_BURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
wire          TLB_REQ;
wire          TLB_WRB;
wire  [31:0]  TLB_WDATA;
wire  [31:0]  TLB_RDATA;
wire          TLB_ACK;
wire          TLB_STALL;
wire  [3:0]   TLB_BSTROBE;
  
/*wire 				wb_cyc_tlb_o;          
wire               wb_stb_tlb_o;
wire               wb_we_tlb_o;	
wire  [31:0] 		wb_adr_tlb_o;	   
wire  [1:0]        wb_bte_tlb_o;
wire  [2:0] 		wb_cti_tlb_o;     
wire  [3:0]        wb_sel_tlb_o;
wire  [31:0]       wb_dat_tlb_o;*/

reg [31:0]  physical_adr;
reg [31:0]  i_addr_int;
wire [tag_phy_last_bit:tag_phy_start_bit] tag_o_tlb;
wire tag_hit;

//assign    wb_cyc_o = wb_cyc_o_int | wb_cyc_tlb_o;      
//assign    wb_adr_o = wb_stb_tlb_o ?  wb_adr_tlb_o : wb_adr_o_int ;
//assign    wb_stb_o = wb_stb_o_int | wb_stb_tlb_o;
//assign    wb_we_o  = wb_we_o_int  | wb_we_tlb_o ; 
//assign    wb_sel_o = wb_stb_tlb_o ? wb_sel_tlb_o : wb_sel_o_int ;
//assign    wb_dat_o = wb_stb_tlb_o ? wb_dat_tlb_o : wb_dat_o_int ;
//assign    wb_cti_o = wb_stb_tlb_o ? wb_cti_tlb_o : wb_cti_o_int ;
//assign    wb_bte_o = wb_stb_tlb_o ? wb_bte_tlb_o : wb_bte_o_int ;

assign REQ     = TLB_REQ | ICACHE_REQ;
assign ADDR    =  TLB_REQ ? TLB_ADDR : ICACHE_ADDR;
assign WRB     = TLB_WRB | ICACHE_WRB;
assign BSTROBE = TLB_REQ ? TLB_BSTROBE : ICACHE_BSTROBE;
assign WDATA   = TLB_REQ ? TLB_WDATA : ICACHE_WDATA;
assign BURST   = TLB_REQ ? TLB_BURST : ICACHE_BURST;


assign freeze = freeze_tlb_out | freeze_in;
assign i_acc  = i_acc_int;

`else

assign REQ     = ICACHE_REQ;
assign ADDR    = ICACHE_ADDR;
assign WRB     = ICACHE_WRB;
assign BSTROBE = ICACHE_BSTROBE;
assign WDATA   = ICACHE_WDATA;
assign BURST   = ICACHE_BURST;

/*assign    wb_cyc_o = wb_cyc_o_int;     
assign    wb_adr_o =  wb_adr_o_int ;
assign    wb_stb_o = wb_stb_o_int ;
assign    wb_we_o  = wb_we_o_int;  
assign    wb_sel_o = wb_sel_o_int ;
assign    wb_dat_o = wb_dat_o_int ;
assign    wb_cti_o = wb_cti_o_int ;
assign    wb_bte_o = wb_bte_o_int ;*/

assign freeze = freeze_in;
assign i_acc  = i_acc_int;

`endif 
//----------------------------------------------------
assign vpn_to_ppn_req_in1 = vpn_to_ppn_req_in;
assign stall_out = stall;
assign i_acc_int = 1'b1;
assign #1 instr_out = (vpn_to_ppn_req7 ? instr_reg :instr);
assign biu_we_i = 1'b0;

reg dat;

always @(posedge clk ) begin
    if(rst_n) begin
        dat <= 1'b0;
    end
    else if ((i_addr == 32'h3e78)) begin // || (i_addr == 32'h1064) || (i_addr == 32'h1078) || (i_addr == 32'h1080) )begin
        dat <= 1'b1;
    end
    else
        dat <= 1'b0;
end

always@( posedge clk)
    begin
    if(rst_n)
        virtual_addr <= 0;
    else if(~(stall || freeze_in))
        virtual_addr <= stall_load ? (i_addr -32'd4) : i_addr;
    end
always@(posedge clk)
    begin
    if(rst_n)
        freeze_hit_status <= 1'b0;
    else if(x_freeze && freeze_in)
        freeze_hit_status<= i_hit;
    end

always@(posedge clk)
    begin
    if(rst_n)
        instr_reg <= 32'b0;
    else if(stall_load_int)
        instr_reg<=32'b0;
    else if(x_freeze && freeze_in)
        instr_reg<=instr;
    else if(tag_hit && i_hit && freeze_in && stall_int )
        instr_reg<=instr;
        
    end

always@(posedge clk )
    begin
        if(rst_n) begin
            freeze_int <=0;
            stall_int <=0;
            stall_load_int <=0;
        end
        else begin
            freeze_int <= freeze_in;    
            stall_int <= stall;    
            stall_load_int <=stall_load;
        end
    end
    
assign x_freeze = (freeze_int ^ freeze_in);    
assign vpn_to_ppn_req7 = (freeze_int && x_freeze );
    

//------------Module instantiations---------------------
//--------------WB BIU module--------------------------- 
/*or1200_wb_biu1 a1(
    // RISC clock, reset and clock control
    .clk(clk), 
    .rst(rst_n), 
    .clmode(2'b00),    
    // WISHBONE interface
    .wb_clk_i(wb_clk_i), .wb_rst_i(wb_rst_i),
    .wb_ack_i(wb_ack_i), .wb_err_i(wb_err_i),
    .wb_rty_i(wb_rty_i), .wb_dat_i(wb_dat_i),
    .wb_cyc_o(wb_cyc_o_int), .wb_adr_o(wb_adr_o_int), 
    .wb_stb_o(wb_stb_o_int), .wb_we_o(wb_we_o_int), 
    .wb_sel_o(wb_sel_o_int), .wb_dat_o(wb_dat_o_int),
    .wb_cti_o(wb_cti_o_int), .wb_bte_o(wb_bte_o_int),
    //to be removed///////////
    // Internal RISC bus
    .biu_adr_i({physical_tag, virtual_addr[page_offset_last:page_offset_start]} & 32'hffffffe0), 
    .biu_cyc_i(biu_cyc_i),
    .biu_stb_i(biu_stb_i),
    .biu_we_i(biu_we_i),
    .biu_sel_i(biu_sel_i),
    .biu_cab_i(biu_cab_i),
    .biu_dat_o(biu_dat_o),
    /////////////////////////		     
    //risc-v proc interface
    .bus_data(bus_data),
    .bus_rdy(bus_rdy),
    //.burst_len(burst_len),
    //.wb_fsm_state_cur(wb_fsm_state_cur),
    //.bus_line(bus_line)
);*/

interface_router INTERFACE_ROUTER_IMEM (
   // Global Signals
   .clk(clk),
   .reset(rst_n),

   // Internal RISC interface
   .biu_adr_i({physical_tag, virtual_addr[page_offset_last:page_offset_start]} & 32'hffffffe0),	  // address bus
   .biu_cyc_i(biu_cyc_i),	  // WB cycle
   .biu_stb_i(biu_stb_i),	  // WB strobe
   .biu_we_i(biu_we_i),	    // WB write enable
   .biu_cab_i(biu_cab_i),	  // CAB input
   .biu_sel_i(biu_sel_i),	  // byte selects
   .biu_dat_o(biu_dat_o),	  // output data bus
   .bus_rdy(bus_rdy),      //interface to the dcache unit
   .bus_data(bus_data),     //to be able to communicate with proc interface 
   
   //Memory Interface
   .ADDR(ICACHE_ADDR),
   .BURST(ICACHE_BURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
   .REQ(ICACHE_REQ),
   .WRB(ICACHE_WRB),
   .WDATA(ICACHE_WDATA),
   .RDATA(RDATA),
   .ACK(ACK),
   .STALL(STALL),
   .BSTROBE(ICACHE_BSTROBE),
   .peripheral_access(1'b0),
  
   //Control Signals
   .freeze(freeze)

);

//---------------/*  Cache */------------------------
icache icache1(
    .clk(clk),//input
    .clk_x2(clk_x2),//input
    .rst_n(rst_n),//input
    .freeze(freeze && ~stall_out), 
    .freeze_in(freeze_in && ~stall_out), 
    .stall_load(stall_load), 
    .wr_data(i_data),
    .we(i_we),
    .re(i_acc_int),
    .hit_out(i_hit),//output
    .i_addr(i_addr),
    .virtual_addr(virtual_addr),
    .state_fsm(state),
    .physical_tag(physical_tag),
    .vpn_to_ppn_req3(vpn_to_ppn_req3),
    .vpn_to_ppn_req7(vpn_to_ppn_req7),
    .freeze_hit_status(freeze_hit_status),
    .stall(stall),
    .re_int(i_acc_int),
    .instr(instr),
    .csr_satp(csr_satp),
    .instruction_page_fault(instruction_page_fault),
    .vpn_to_ppn_req(vpn_to_ppn_req_in1 || vpn_to_ppn_req_in2)
    `ifdef itlb_def
    ,.tag_o_tlb(tag_o_tlb),.freeze_tlb_out(freeze_tlb_out),.tag_hit(tag_hit),.tlb_trans_off(tlb_trans_off),
      .ADDR(TLB_ADDR),
      .BURST(TLB_BURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
      .REQ(TLB_REQ),
      .WRB(TLB_WRB),
      .WDATA(TLB_WDATA),
      .RDATA(RDATA),
      .ACK(ACK),
      .STALL(STALL),
      .BSTROBE(TLB_BSTROBE)
      
    /*  
    .wb_ack_i(wb_ack_i), .wb_err_i(wb_err_i), .wb_rty_i(wb_rty_i), 
    .wb_dat_i(wb_dat_i), .wb_cyc_o(wb_cyc_tlb_o), .wb_stb_o(wb_stb_tlb_o),
    .wb_we_o(wb_we_tlb_o), .wb_adr_o(wb_adr_tlb_o), .wb_bte_o(wb_bte_tlb_o),
    .wb_cti_o(wb_cti_tlb_o), .wb_sel_o(wb_sel_tlb_o), .wb_dat_o(wb_dat_tlb_o)*/
    `endif
);

//-------------------/* Cache FSM */--------------------
cachefsm a3(
    .clk(clk),	
    .rst_n(rst_n),
    .freeze((freeze && ~stall_out)), 
    .freeze_in (freeze_in),
    .i_hit(i_hit),
    .m_line_full(bus_data),
    `ifdef itlb_def
    .tag_hit(tag_hit),
    `endif    
    .i_acc(i_acc),
    .i_we(i_we),//output
    .i_data(i_data),
    .vpn_to_ppn_req_out(vpn_to_ppn_req_in2),
    .vpn_to_ppn_req3(vpn_to_ppn_req3),
    .state(state),
    .stall(stall),
    .biu_cyc_i(biu_cyc_i),
    .biu_stb_i(biu_stb_i),
    .biu_cab_i (biu_cab_i),
    .biu_sel_i(biu_sel_i),
    .wb_ack_i(ACK && REQ)

);
//------------------------------------------------------

endmodule
