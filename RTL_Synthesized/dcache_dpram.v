`timescale 1ns / 1ps



`include "defines.v"
`define TSMC_RAM_EN 0

module dcache_dpram
(
    input rst,
    input clk,
    input clk_x2,
    input [255:0] dcache_in_a_w0,
    input [255:0] dcache_in_b_w0,
    input [255:0] dcache_in_a_w1,
    input [255:0] dcache_in_b_w1,
    
    input [31:0] addr_in_a,
    input [31:0] addr_in_b,
    input [6:0] dcache_addr_w0_a,
    input [6:0] dcache_addr_w1_a,
    input [6:0] dcache_addr_w0_b,
    input [6:0] dcache_addr_w1_b,
    
    input [31:0] we_a_w0,
    input [31:0] we_b_w0,
    input [31:0] we_a_w1,
    input [31:0] we_b_w1,
    
    input [3:0] we_tag_a_w0,
    input [3:0] we_tag_b_w0,
    input [3:0] we_tag_a_w1,
    input [3:0] we_tag_b_w1,
    
    input [6:0] tag_addr_a_w0,
    input [6:0] tag_addr_a_w1,
    input [6:0] tag_addr_b_w0,
    input [6:0] tag_addr_b_w1,
    input [31:0] tag_data_a_w0,
    input [31:0] tag_data_a_w1,
    input [31:0] tag_data_b_w0,
    input [31:0] tag_data_b_w1,
    
    output reg [31:0] tag_a_w0_o,
    output reg [31:0] tag_a_w1_o,
    output reg [31:0] tag_b_w0_o,
    output reg [31:0] tag_b_w1_o,
    
    input [6:0] Dirty_bit_Addr_a_w0, 
    input [6:0] Dirty_bit_Addr_a_w1, 
    output reg Dirty_bit_Read_Data_a_w0,       
    output reg Dirty_bit_Read_Data_a_w1,       
    input Dirty_bit_Write_Data_a_w0, 
    input Dirty_bit_Write_Data_a_w1, 
    input Dirty_bit_Write_En_a_w0,   
    input Dirty_bit_Write_En_a_w1,  
    
    input [6:0] Dirty_bit_Addr_b_w0, 
    input [6:0] Dirty_bit_Addr_b_w1, 
    output reg Dirty_bit_Read_Data_b_w0,       
    output reg Dirty_bit_Read_Data_b_w1,       
    input Dirty_bit_Write_Data_b_w0, 
    input Dirty_bit_Write_Data_b_w1, 
    input Dirty_bit_Write_En_b_w0,   
    input Dirty_bit_Write_En_b_w1,   
   
    output dmem_allow,
    input  [`CSR_SB_W-1:0] csr_pmp_sb,
    
    input [4:0] lsu_op_port1,
    input [4:0] lsu_op_port2,
   
    input badaddr_data, 
    input freeze,
    input byp_a,                        //Bypass the input register and feed the address directly. 
    input byp_b,  
    
    input vpn_to_ppn_req_port1,
    input vpn_to_ppn_req_port2,
    input freeze_tlb,
    output tlb_freeze_dcache,
    output [25:0] tag_out_tlb_port1,
    output [25:0] tag_out_tlb_port2,
    output tag_hit_tlb_port1,
    output tag_hit_tlb_port2,
    /*------------ Wishbone Signals -------------- 
    
    //input				wb_clk_i;	// clock input
    //input				wb_rst_i;	// reset input
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
    output [31:0]       wb_dat_o,	// output data bus
    */
    //-----------New: Memory Interface --------------------
    
    output  [31:0]  ADDR,
    output  [1:0]   BURST, //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
    output          REQ,
    output          WRB,
    output wire [31:0]  WDATA,
    input      [31:0]  RDATA,
    input              ACK,
    input              STALL,
    output  [3:0]   BSTROBE,
                          
    output reg [255:0] w0_data_a,
    output reg [255:0] w1_data_a,
    output reg [255:0] w0_data_b,
    output reg [255:0] w1_data_b,
    
    output reg [31:0] dout_a,
    output reg [31:0] dout_b,
    output reg hit_a,
    output reg hit_b,
    output reg mis_a,
    output reg mis_b,
    output reg a_w0_hit,          //signals the fsm which way was hit. that way will be written  in case of write
    output reg a_w1_hit,          //  "       "   "   "   "   "   "       "   "   "   "   "       "   "   "   "
    output reg b_w0_hit,          //signals the fsm which way was hit. that way will be written  in case of write
    output reg b_w1_hit,          //  "       "   "   "   "   "   "       "   "   "   "   "       "   "   "   "wire [4:0] index_a;
   
    input dtlb_trans_off,
    input [31:0] csr_satp,
    output data_page_fault, 
    output addr_exception_port1,
    output addr_exception_port2,
    output load_page_fault,
    output store_page_fault,

    output load_access_fault,
    output store_access_fault,

    output read_exception_port1,
    output read_exception_port2,
    output write_exception_port1,
    output write_exception_port2
);

parameter offset_start_bit = 0;
parameter offset_last_bit = 4;
parameter index_start_bit = 5;
parameter index_last_bit = 11;
parameter tag_start_bit = 12;
parameter tag_last_bit = 31;
parameter vpn_width = 20;
parameter tag_width=22;

//-------TLB tag out----------------------------------
//Currently TLB tag out consists of 22 bit physical tag and 4 bit i.e. U, X, W, R. 
//But Current design uses 20 bit physical tag. So by assuming MSB 2 bits as zero in 22 bit physical tag
//it will become 20 bit value and i.e. taken for comparison. Because of this reason 23:4 is considered for tag
parameter tag_tlb_start_bit = 4;
parameter tag_tlb_last_bit = 23;
//----------------------------------------------------


  wire [3:0] dmem_porta_permissions;
  wire [3:0] dmem_portb_permissions;
  wire dmem_porta_allow;
  wire dmem_portb_allow;


wire [21:0] tag_a;
wire [6:0] index_a;
wire [2:0] blk_offst_a;
wire [6:0] index_b;
wire [2:0] blk_offst_b;
wire [21:0] tag_b;

reg [127:0] w0_a_1;
reg [127:0] w1_a_1;
reg [127:0] w0_b_1;
reg [127:0] w1_b_1;
reg [127:0] w0_a_2;
reg [127:0] w1_a_2;
reg [127:0] w0_b_2;
reg [127:0] w1_b_2;

wire [127:0] w0_a_1_d;
wire [127:0] w1_a_1_d;
wire [127:0] w0_b_1_d;
wire [127:0] w1_b_1_d;
wire [127:0] w0_a_2_d;
wire [127:0] w1_a_2_d;
wire [127:0] w0_b_2_d;
wire [127:0] w1_b_2_d;
reg [31:0] tag_a_w0_int;
wire [31:0] tag_a_w0_int_d;
reg [31:0] tag_a_w1_int;
wire [31:0] tag_a_w1_int_d;
reg [31:0] tag_b_w0_int;
wire [31:0] tag_b_w0_int_d;
reg [31:0] tag_b_w1_int;
wire [31:0] tag_b_w1_int_d;

wire tag_comp_w0_a;
wire tag_comp_w1_a;
wire tag_comp_w0_b;
wire tag_comp_w1_b;

wire proc_rq_port1;
wire proc_rq_port2;




reg [31:0] addr_in_a_int;
reg [31:0] addr_in_b_int;
reg [4:0] lsu_op_port1_int;
reg [4:0] lsu_op_port2_int;

localparam PULSE_START = 2'b00;
localparam PULSE_DEL = 2'b01;
localparam PULSE_HI = 2'b10;
localparam PULSE_LOW = 2'b11;

//wire read_exception_port1;
//wire read_exception_port2;
//wire write_exception_port1;
//wire write_exception_port2;

//assign load_exception = read_exception_port1 || read_exception_port2;
//assign store_exception = write_exception_port1 || write_exception_port2;

assign read_exception_port1 = ((lsu_op_port1_int == 2'b01) && (~tag_out_tlb_port1[1]) && tag_hit_tlb_port1);
assign read_exception_port2 = ((lsu_op_port2_int == 2'b01) && (~tag_out_tlb_port2[1]) && tag_hit_tlb_port2);
assign write_exception_port1 = ((lsu_op_port1_int == 2'b10) && (~(tag_out_tlb_port1[2:1] == 2'b11)) && tag_hit_tlb_port1);
assign write_exception_port2 = ((lsu_op_port2_int == 2'b10) && (~(tag_out_tlb_port2[2:1] == 2'b11)) && tag_hit_tlb_port2);

assign load_page_fault = ((lsu_op_port1_int == 2'b01) & data_page_fault) || ((lsu_op_port2_int == 2'b01) & data_page_fault) || read_exception_port1 || read_exception_port2;
assign store_page_fault = ((lsu_op_port1_int == 2'b10) & data_page_fault) || ((lsu_op_port2_int == 2'b10) & data_page_fault) || write_exception_port1 || write_exception_port2;

assign load_access_fault = ((lsu_op_port1_int == 2'b01) & ~dmem_porta_permissions[0] & tag_hit_tlb_port1) || ((lsu_op_port2_int == 2'b01) & ~dmem_portb_permissions[0] & tag_hit_tlb_port2);
assign store_access_fault = ((lsu_op_port1_int == 2'b10) & ~dmem_porta_allow & tag_hit_tlb_port1) || ((lsu_op_port2_int == 2'b10) & ~dmem_portb_allow & tag_hit_tlb_port2);


assign addr_exception_port1 =  read_exception_port1 || write_exception_port1 || badaddr_data || load_page_fault || store_page_fault || load_access_fault ||  store_access_fault;
assign addr_exception_port2 =  read_exception_port2 || write_exception_port2 || store_page_fault || load_page_fault || load_access_fault || store_access_fault ;

assign proc_rq_port1 = (lsu_op_port1[1] ^ lsu_op_port1[0]) & ~badaddr_data;
assign proc_rq_port2 = (lsu_op_port2[1] ^ lsu_op_port2[0]) & ~badaddr_data;



//input register to take in the operand values;disabled by 'byp' pin
always @(posedge clk) begin
    if(rst) begin
        addr_in_a_int <= 32'b0;
        addr_in_b_int <= 32'b0;        
        lsu_op_port1_int <= 5'b0;
        lsu_op_port2_int <= 5'b0;
    end
    else begin
        if(~freeze) begin
            addr_in_a_int <= addr_in_a;
            addr_in_b_int <= addr_in_b;
            lsu_op_port1_int <= lsu_op_port1;
            lsu_op_port2_int <= lsu_op_port2;        
        end
    end
end


assign blk_offst_a = byp_a ? addr_in_a[4:2] : addr_in_a_int[4:2];
assign blk_offst_b = byp_b ? addr_in_b[4:2] : addr_in_b_int[4:2];
assign index_a = addr_in_a[index_last_bit:index_start_bit];
assign index_b = addr_in_b[index_last_bit:index_start_bit];
assign tag_a = byp_a ? addr_in_a[tag_last_bit:tag_start_bit] : addr_in_a_int[tag_last_bit:tag_start_bit];
assign tag_b = byp_b ? addr_in_b[tag_last_bit:tag_start_bit] : addr_in_b_int[tag_last_bit:tag_start_bit];
assign tag_comp_w0_a = ((tag_out_tlb_port1[tag_tlb_last_bit:tag_tlb_start_bit] == tag_a_w0_int[(tag_last_bit-tag_start_bit):0]) & tag_a_w0_int[22] & tag_hit_tlb_port1 & dmem_porta_allow) ? 1'b1 : 1'b0;
assign tag_comp_w1_a = ((tag_out_tlb_port1[tag_tlb_last_bit:tag_tlb_start_bit] == tag_a_w1_int[(tag_last_bit-tag_start_bit):0]) & tag_a_w1_int[22] & tag_hit_tlb_port1 & dmem_porta_allow) ? 1'b1 : 1'b0; 
assign tag_comp_w0_b = ((tag_out_tlb_port2[tag_tlb_last_bit:tag_tlb_start_bit] == tag_b_w0_int[(tag_last_bit-tag_start_bit):0]) & tag_b_w0_int[22] & tag_hit_tlb_port2 & dmem_portb_allow) ? 1'b1 : 1'b0;
assign tag_comp_w1_b = ((tag_out_tlb_port2[tag_tlb_last_bit:tag_tlb_start_bit] == tag_b_w1_int[(tag_last_bit-tag_start_bit):0]) & tag_b_w1_int[22] & tag_hit_tlb_port2 & dmem_portb_allow) ? 1'b1 : 1'b0; 


always @(*) begin
    if(rst) begin
        a_w0_hit = 1'b0;
        a_w1_hit = 1'b0;
        b_w0_hit = 1'b0;
        b_w1_hit = 1'b0;        
    end
    else begin
        a_w0_hit = tag_comp_w0_a;
        a_w1_hit = tag_comp_w1_a;
        b_w0_hit = tag_comp_w0_b;
        b_w1_hit = tag_comp_w1_b;            
    end
end

always @(*) begin
    hit_a = (tag_comp_w0_a) | (tag_comp_w1_a);
    hit_b = (tag_comp_w0_b) | (tag_comp_w1_b);       
    mis_a = ~hit_a;
    mis_b = ~hit_b;
end

always @(*) begin
    case(blk_offst_a)
        3'b000: dout_a = tag_comp_w0_a ? w0_a_1[31:0] : w1_a_1[31:0];
        3'b001: dout_a = tag_comp_w0_a ? w0_a_1[63:32] : w1_a_1[63:32];
        3'b010: dout_a = tag_comp_w0_a ? w0_a_1[95:64] : w1_a_1[95:64];
        3'b011: dout_a = tag_comp_w0_a ? w0_a_1[127:96] : w1_a_1[127:96];
        3'b100: dout_a = tag_comp_w0_a ? w0_a_2[31:0] : w1_a_2[31:0];
        3'b101: dout_a = tag_comp_w0_a ? w0_a_2[63:32] : w1_a_2[63:32];
        3'b110: dout_a = tag_comp_w0_a ? w0_a_2[95:64] : w1_a_2[95:64];
        3'b111: dout_a = tag_comp_w0_a ? w0_a_2[127:96] : w1_a_2[127:96];
        default: dout_a = 32'b0;
    endcase
end
 
always @(*) begin
    case(blk_offst_b)
        3'b000: dout_b = tag_comp_w0_b ? w0_b_1[31:0] : w1_b_1[31:0];
        3'b001: dout_b = tag_comp_w0_b ? w0_b_1[63:32] : w1_b_1[63:32];
        3'b010: dout_b = tag_comp_w0_b ? w0_b_1[95:64] : w1_b_1[95:64];
        3'b011: dout_b = tag_comp_w0_b ? w0_b_1[127:96] : w1_b_1[127:96];
        3'b100: dout_b = tag_comp_w0_b ? w0_b_2[31:0] : w1_b_2[31:0];
        3'b101: dout_b = tag_comp_w0_b ? w0_b_2[63:32] : w1_b_2[63:32];
        3'b110: dout_b = tag_comp_w0_b ? w0_b_2[95:64] : w1_b_2[95:64];
        3'b111: dout_b = tag_comp_w0_b ? w0_b_2[127:96] : w1_b_2[127:96];
        default: dout_b = 32'b0;
    endcase
end

always @(*) begin
    w0_data_a = {{w0_a_2},{w0_a_1}};
    w1_data_a = {{w1_a_2},{w1_a_1}};
    w0_data_b = {{w0_b_2},{w0_b_1}};
    w1_data_b = {{w1_b_2},{w1_b_1}};
    tag_a_w0_o = tag_a_w0_int;
    tag_a_w1_o = tag_a_w1_int;
    tag_b_w0_o = tag_b_w0_int;
    tag_b_w1_o = tag_b_w1_int;
end
 
reg [127:0] Dirty_Bit_w0;
reg [127:0] Dirty_Bit_w1;

always @(posedge clk) begin
    if(rst) begin
		 Dirty_Bit_w0 <= 0;
    end
    else begin
        if(Dirty_bit_Write_En_a_w0) begin
            Dirty_Bit_w0[Dirty_bit_Addr_a_w0] <= Dirty_bit_Write_Data_a_w0;
        end
        if(Dirty_bit_Write_En_b_w0) begin
            Dirty_Bit_w0[Dirty_bit_Addr_b_w0] <= Dirty_bit_Write_Data_b_w0;
        end
    end
end

always @(posedge clk) begin
    if(rst) begin
		 Dirty_Bit_w1 <= 0;
    end
    else begin
        if(Dirty_bit_Write_En_a_w1) begin
            Dirty_Bit_w1[Dirty_bit_Addr_a_w1] <= Dirty_bit_Write_Data_a_w1;
        end
        if(Dirty_bit_Write_En_b_w1) begin
            Dirty_Bit_w1[Dirty_bit_Addr_b_w1] <= Dirty_bit_Write_Data_b_w1;
        end
    end
end

always @(*) begin
    if(rst) begin
        Dirty_bit_Read_Data_a_w0 = 0;
        Dirty_bit_Read_Data_a_w1 = 0;
        Dirty_bit_Read_Data_b_w0 = 0;
        Dirty_bit_Read_Data_b_w1 = 0;
    end
    else begin
        Dirty_bit_Read_Data_a_w0 = Dirty_Bit_w0[Dirty_bit_Addr_a_w0];
        Dirty_bit_Read_Data_a_w1 = Dirty_Bit_w1[Dirty_bit_Addr_a_w1];
        Dirty_bit_Read_Data_b_w0 = Dirty_Bit_w0[Dirty_bit_Addr_b_w0];
        Dirty_bit_Read_Data_b_w1 = Dirty_Bit_w1[Dirty_bit_Addr_b_w1];
    end
end

//------------WAY0-------------------------------------------
//-------------Bank1-----------------------------------------
generate
  if(`TSMC_RAM_EN) begin
    wire [127:0] bit_write_en_a_w0_1;
    wire [127:0] bit_write_en_b_w0_1;
    
    assign bit_write_en_a_w0_1 ={	{8{~we_a_w0[15]}},{8{~we_a_w0[14]}},{8{~we_a_w0[13]}},{8{~we_a_w0[12]}},
    				{8{~we_a_w0[11]}},{8{~we_a_w0[10]}},{8{~we_a_w0[9]}}, {8{~we_a_w0[8]}},
    				{8{~we_a_w0[7]}},{8{~we_a_w0[6]}},  {8{~we_a_w0[5]}}, {8{~we_a_w0[4]}},
    				{8{~we_a_w0[3]}},{8{~we_a_w0[2]}},  {8{~we_a_w0[1]}}, {8{~we_a_w0[0]}} };
    
    assign bit_write_en_b_w0_1 ={	{8{~we_b_w0[15]}},{8{~we_b_w0[14]}},{8{~we_b_w0[13]}},{8{~we_b_w0[12]}},
    				{8{~we_b_w0[11]}},{8{~we_b_w0[10]}},{8{~we_b_w0[9]}}, {8{~we_b_w0[8]}},
    				{8{~we_b_w0[7]}}, {8{~we_b_w0[6]}}, {8{~we_b_w0[5]}}, {8{~we_b_w0[4]}},
    				{8{~we_b_w0[3]}}, {8{~we_b_w0[2]}}, {8{~we_b_w0[1]}}, {8{~we_b_w0[0]}} };
    
    always @(posedge clk) begin
    	if(rst) begin
    		w0_a_1 <= 128'd0;
    		w1_a_1 <= 128'd0;
    		w0_b_1 <= 128'd0;
    		w1_b_1 <= 128'd0;
    		w0_a_2 <= 128'd0;
    		w1_a_2 <= 128'd0;
    		w0_b_2 <= 128'd0;
    		w1_b_2 <= 128'd0;
    		
    	end
    	else begin
    		w0_a_1 <= w0_a_1_d;
    		w1_a_1 <= w1_a_1_d;
    		w0_b_1 <= w0_b_1_d;
    		w1_b_1 <= w1_b_1_d;
    		w0_a_2 <= w0_a_2_d;
    		w1_a_2 <= w1_a_2_d;
    		w0_b_2 <= w0_b_2_d;
    		w1_b_2 <= w1_b_2_d;
    		
    	end
    end	
    
    TSDN65LPLLA128X128M4F ram_w0_1 (
    .AA(dcache_addr_w0_a), 			// Address of A: Addra[6:0]
    .DA(dcache_in_a_w0[127:0]),			// Data in of A: douta[127:0]	
    .BWEBA(bit_write_en_a_w0_1),			// Bit-Write ~en of A: {128{~en}}	
    .WEBA(~|we_a_w0[15:0]),.CEBA(1'b0),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
    .AB(dcache_addr_w0_b),			// Address of B: Addra[6:0]
    .DB(dcache_in_b_w0[127:0]),			// Data in of B: douta[127:0]
    .BWEBB(bit_write_en_b_w0_1),			// Bit-Write ~en of B: {128{~en}}
    .WEBB(~|we_b_w0[15:0]),.CEBB(1'b0),.CLKB(~clk),	// Write-~en, Chip-~en, CLKB
    .AMA(7'd0),
    .DMA(128'd0),
    .BWEBMA(128'hffff_ffff_ffff_ffff),
    .WEBMA(1'b1),.CEBMA(1'b1),
    .AMB(7'd0),
    .DMB(128'd0),
    .BWEBMB(128'hffff_ffff_ffff_ffff),
    .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
    .QA(w0_a_1_d),
    .QB(w0_b_1_d)
	  );
  end
  else begin

  assign bit_write_en_a_w0_1 = 128'd0;
  assign bit_write_en_b_w0_1 = 128'd0;
  //assign w0_a_1_d = 128'd0;
  //assign w0_b_1_d = 128'd0;

  always @(*) begin
  		w0_a_1 <= w0_a_1_d;
  		w1_a_1 <= w1_a_1_d;
  		w0_b_1 <= w0_b_1_d;
  		w1_b_1 <= w1_b_1_d;
  		w0_a_2 <= w0_a_2_d;
  		w1_a_2 <= w1_a_2_d;
  		w0_b_2 <= w0_b_2_d;
  		w1_b_2 <= w1_b_2_d;		
  end	


dcache_DPRAM ram_w0_1 (                                             //w0 data ram bank
  .clka(clk),.rsta(rst),.wea(we_a_w0[15:0]),.addra(dcache_addr_w0_a),.dina(dcache_in_a_w0[127:0]),.douta(w0_a_1_d),
  .clkb(clk),.rstb(rst),.web(we_b_w0[15:0]),.addrb(dcache_addr_w0_b),.dinb(dcache_in_b_w0[127:0]),.doutb(w0_b_1_d)
);

  end
endgenerate
//-------------Bank2-----------------------------------------

generate 
  if(`TSMC_RAM_EN) begin
    wire [127:0] bit_write_en_a_w0_2;
    wire [127:0] bit_write_en_b_w0_2;
    
    assign bit_write_en_a_w0_2 ={	{8{~we_a_w0[31]}},{8{~we_a_w0[30]}},{8{~we_a_w0[29]}},{8{~we_a_w0[28]}},
    				{8{~we_a_w0[27]}},{8{~we_a_w0[26]}},{8{~we_a_w0[25]}},{8{~we_a_w0[24]}},
    				{8{~we_a_w0[23]}},{8{~we_a_w0[22]}},{8{~we_a_w0[21]}},{8{~we_a_w0[20]}},
    				{8{~we_a_w0[19]}},{8{~we_a_w0[18]}},{8{~we_a_w0[17]}},{8{~we_a_w0[16]}}};
    
    assign bit_write_en_b_w0_2 ={	{8{~we_b_w0[31]}},{8{~we_b_w0[30]}},{8{~we_b_w0[29]}},{8{~we_b_w0[28]}},
    				{8{~we_b_w0[27]}},{8{~we_b_w0[26]}},{8{~we_b_w0[25]}},{8{~we_b_w0[24]}},
    				{8{~we_b_w0[23]}},{8{~we_b_w0[22]}},{8{~we_b_w0[21]}},{8{~we_b_w0[20]}},
    				{8{~we_b_w0[19]}},{8{~we_b_w0[18]}},{8{~we_b_w0[17]}},{8{~we_b_w0[16]}}};
    
    
    
    TSDN65LPLLA128X128M4F ram_w0_2 (
      .AA(dcache_addr_w0_a), 			// Address of A: Addra[6:0]
      .DA(dcache_in_a_w0[255:128]),			// Data in of A: douta[127:0]	
      .BWEBA(bit_write_en_a_w0_2),			// Bit-Write ~en of A: {128{~en}}	
      .WEBA(~|we_a_w0[31:16]),.CEBA(1'b0),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
      .AB(dcache_addr_w0_b),			// Address of B: Addra[6:0]
      .DB(dcache_in_b_w0[255:128]),				// Data in of B: douta[127:0]
      .BWEBB(bit_write_en_b_w0_2),			// Bit-Write ~en of B: {128{~en}}
      .WEBB(~|we_b_w0[31:16]),.CEBB(1'b0),.CLKB(~clk),	// Write-~en, Chip-~en, CLKB
      .AMA(7'd0),
      .DMA(128'd0),
      .BWEBMA(128'hffff_ffff_ffff_ffff),
      .WEBMA(1'b1),.CEBMA(1'b1),
      .AMB(7'd0),
      .DMB(128'd0),
      .BWEBMB(128'hffff_ffff_ffff_ffff),
      .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
      .QA(w0_a_2_d),
      .QB(w0_b_2_d)
    	);
    end
    else begin
      assign bit_write_en_a_w0_2 = 128'd0;
      assign bit_write_en_b_w0_2 = 128'd0;
      //assign w0_a_2_d = 128'd0;
      //assign w0_b_2_d = 128'd0;

      dcache_DPRAM ram_w0_2 (                                             //w0 data ram bank
        .clka(clk),.rsta(rst),.wea(we_a_w0[31:16]),.addra(dcache_addr_w0_a),.dina(dcache_in_a_w0[255:128]),.douta(w0_a_2_d),
        .clkb(clk),.rstb(rst),.web(we_b_w0[31:16]),.addrb(dcache_addr_w0_b),.dinb(dcache_in_b_w0[255:128]),.doutb(w0_b_2_d)
      );
    end
  endgenerate




//------------WAY1-------------------------------------------
//-------------Bank1-----------------------------------------
generate
  if (`TSMC_RAM_EN) begin
    wire [127:0] bit_write_en_a_w1_1;
    wire [127:0] bit_write_en_b_w1_1;
    
    assign bit_write_en_a_w1_1 ={	{8{~we_a_w1[15]}},{8{~we_a_w1[14]}},{8{~we_a_w1[13]}},{8{~we_a_w1[12]}},
    				{8{~we_a_w1[11]}},{8{~we_a_w1[10]}},{8{~we_a_w1[9]}}, {8{~we_a_w1[8]}},
    				{8{~we_a_w1[7]}},{8{~we_a_w1[6]}},  {8{~we_a_w1[5]}}, {8{~we_a_w1[4]}},
    				{8{~we_a_w1[3]}},{8{~we_a_w1[2]}},  {8{~we_a_w1[1]}}, {8{~we_a_w1[0]}} };
    
    assign bit_write_en_b_w1_1 ={	{8{~we_b_w1[15]}},{8{~we_b_w1[14]}},{8{~we_b_w1[13]}},{8{~we_b_w1[12]}},
    				{8{~we_b_w1[11]}},{8{~we_b_w1[10]}},{8{~we_b_w1[9]}}, {8{~we_b_w1[8]}},
    				{8{~we_b_w1[7]}}, {8{~we_b_w1[6]}}, {8{~we_b_w1[5]}}, {8{~we_b_w1[4]}},
    				{8{~we_b_w1[3]}}, {8{~we_b_w1[2]}}, {8{~we_b_w1[1]}}, {8{~we_b_w1[0]}} };
    
    
    TSDN65LPLLA128X128M4F ram_w1_1 (
      .AA(dcache_addr_w1_a), 			// Address of A: Addra[6:0]
      .DA(dcache_in_a_w1[127:0]),			// Data in of A: douta[127:0]	
      .BWEBA(bit_write_en_a_w1_1),			// Bit-Write ~en of A: {128{~en}}	
      .WEBA(~|we_a_w1[15:0]),.CEBA(1'b0),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
      .AB(dcache_addr_w1_b),			// Address of B: Addra[6:0]
      .DB(dcache_in_b_w1[127:0]),			// Data in of B: douta[127:0]
      .BWEBB(bit_write_en_b_w1_1),			// Bit-Write ~en of B: {128{~en}}
      .WEBB(~|we_b_w1[15:0]),.CEBB(1'b0),.CLKB(~clk),	// Write-~en, Chip-~en, CLKB
      .AMA(7'd0),
      .DMA(128'd0),
      .BWEBMA(128'hffff_ffff_ffff_ffff),
      .WEBMA(1'b1),.CEBMA(1'b1),
      .AMB(7'd0),
      .DMB(128'd0),
      .BWEBMB(128'hffff_ffff_ffff_ffff),
      .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
      .QA(w1_a_1_d),
      .QB(w1_b_1_d)
    	);
    end
    else begin

      assign bit_write_en_a_w1_1 = 128'd0;
      assign bit_write_en_b_w1_1 = 128'd0;
      //assign w1_a_1_d = 128'd0;
      //assign w1_b_1_d = 128'd0;

      dcache_DPRAM ram_w1_1 (                                             //w1 data ram bank
        .clka(clk),.rsta(rst),.wea(we_a_w1[15:0]),.addra(dcache_addr_w1_a),.dina(dcache_in_a_w1[127:0]),.douta(w1_a_1_d),
        .clkb(clk),.rstb(rst),.web(we_b_w1[15:0]),.addrb(dcache_addr_w1_b),.dinb(dcache_in_b_w1[127:0]),.doutb(w1_b_1_d)
      );
    end
  endgenerate


//-------------Bank2-----------------------------------------
generate
  if (`TSMC_RAM_EN) begin
    wire [127:0] bit_write_en_a_w1_2;
    wire [127:0] bit_write_en_b_w1_2;
    
    assign bit_write_en_a_w1_2 ={	{8{~we_a_w1[31]}},{8{~we_a_w1[30]}},{8{~we_a_w1[29]}},{8{~we_a_w1[28]}},
    				{8{~we_a_w1[27]}},{8{~we_a_w1[26]}},{8{~we_a_w1[25]}},{8{~we_a_w1[24]}},
    				{8{~we_a_w1[23]}},{8{~we_a_w1[22]}},{8{~we_a_w1[21]}},{8{~we_a_w1[20]}},
    				{8{~we_a_w1[19]}},{8{~we_a_w1[18]}},{8{~we_a_w1[17]}},{8{~we_a_w1[16]}}};
    
    assign bit_write_en_b_w1_2 ={	{8{~we_b_w1[31]}},{8{~we_b_w1[30]}},{8{~we_b_w1[29]}},{8{~we_b_w1[28]}},
    				{8{~we_b_w1[27]}},{8{~we_b_w1[26]}},{8{~we_b_w1[25]}},{8{~we_b_w1[24]}},
    				{8{~we_b_w1[23]}},{8{~we_b_w1[22]}},{8{~we_b_w1[21]}},{8{~we_b_w1[20]}},
    				{8{~we_b_w1[19]}},{8{~we_b_w1[18]}},{8{~we_b_w1[17]}},{8{~we_b_w1[16]}}};
    
    
    
    TSDN65LPLLA128X128M4F ram_w1_2 (
      .AA(dcache_addr_w1_a), 			// Address of A: Addra[6:0]
      .DA(dcache_in_a_w1[255:128]),			// Data in of A: douta[127:0]	
      .BWEBA(bit_write_en_a_w1_2),			// Bit-Write ~en of A: {128{~en}}	
      .WEBA(~|we_a_w1[31:16]),.CEBA(1'b0),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
      .AB(dcache_addr_w1_b),			// Address of B: Addra[6:0]
      .DB(dcache_in_b_w1[255:128]),				// Data in of B: douta[127:0]
      .BWEBB(bit_write_en_b_w1_2),			// Bit-Write ~en of B: {128{~en}}
      .WEBB(~|we_b_w1[31:16]),.CEBB(1'b0),.CLKB(~clk),	// Write-~en, Chip-~en, CLKB
      .AMA(7'd0),
      .DMA(128'd0),
      .BWEBMA(128'hffff_ffff_ffff_ffff),
      .WEBMA(1'b1),.CEBMA(1'b1),
      .AMB(7'd0),
      .DMB(128'd0),
      .BWEBMB(128'hffff_ffff_ffff_ffff),
      .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
      .QA(w1_a_2_d),
      .QB(w1_b_2_d)
    	);

  end else begin

    assign bit_write_en_a_w1_2 = 128'd0;
    assign bit_write_en_b_w1_2 = 128'd0;
    //assign w1_a_2_d = 128'd0;
    //assign w1_b_2_d = 128'd0;

    dcache_DPRAM ram_w1_2 (                                             //w1 data ram bank
      .clka(clk),.rsta(rst),.wea(we_a_w1[31:16]),.addra(dcache_addr_w1_a),.dina(dcache_in_a_w1[255:128]),.douta(w1_a_2_d),
      .clkb(clk),.rstb(rst),.web(we_b_w1[31:16]),.addrb(dcache_addr_w1_b),.dinb(dcache_in_b_w1[255:128]),.doutb(w1_b_2_d)
    );

  end
endgenerate

//-------TAG ARRAY------------------------------------
generate 
  if (`TSMC_RAM_EN) begin

    wire [31:0] bit_write_en_b_tag_a_w0;
    wire [31:0] bit_write_en_b_tag_b_w0;
    always @(posedge clk) begin
    	if(rst) begin
    		tag_a_w0_int <= 0;
    		tag_b_w0_int <= 0;
    		tag_a_w1_int <= 0;
    		tag_b_w1_int <= 0;
    	end
    	else begin
    		tag_a_w0_int <= tag_a_w0_int_d;
    		tag_b_w0_int <= tag_b_w0_int_d;
    		tag_a_w1_int <= tag_a_w1_int_d;
    		tag_b_w1_int <= tag_b_w1_int_d;
    		
    	end
    end	
    
    assign bit_write_en_b_tag_a_w0 = {{8{~we_tag_a_w0[3]}},{8{~we_tag_a_w0[2]}},{8{~we_tag_a_w0[1]}},{8{~we_tag_a_w0[0]}}}; 
    assign bit_write_en_b_tag_b_w0 = {{8{~we_tag_b_w0[3]}},{8{~we_tag_b_w0[2]}},{8{~we_tag_b_w0[1]}},{8{~we_tag_b_w0[0]}}}; 
    
    TSDN65LPLLA128X32M8F tag_w0 (
      .AA(tag_addr_a_w0), 			// Address of A: Addra[6:0]
      .DA(tag_data_a_w0),			// Data in of A: douta[127:0]	
      .BWEBA(32'b0),			// Bit-Write ~en of A: {128{~en}}	
      .WEBA(~|we_tag_a_w0),.CEBA(1'b0),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
      .AB(tag_addr_b_w0),			// Address of B: Addra[6:0]
      .DB(tag_data_b_w0),				// Data in of B: douta[127:0]
      .BWEBB(32'd0),			// Bit-Write ~en of B: {128{~en}}
      .WEBB(~|we_tag_b_w0),.CEBB(1'b0),.CLKB(~clk),	// Write-~en, Chip-~en, CLKB
      .AMA(7'd0),
      .DMA(128'd0),
      .BWEBMA(128'hffff_ffff_ffff_ffff),
      .WEBMA(1'b1),.CEBMA(1'b1),
      .AMB(7'd0),
      .DMB(128'd0),
      .BWEBMB(128'hffff_ffff_ffff_ffff),
      .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
      .QA(tag_a_w0_int_d),
      .QB(tag_b_w0_int_d)
    	);

  end else begin

  assign bit_write_en_b_tag_a_w0 = 32'd0;
  assign bit_write_en_b_tag_b_w0 = 32'd0;
  //assign tag_a_w0_int_d = 32'd0; 
  //assign tag_b_w0_int_d = 32'd0; 

    always @(*) begin
    		tag_a_w0_int <= tag_a_w0_int_d;
    		tag_b_w0_int <= tag_b_w0_int_d;
    		tag_a_w1_int <= tag_a_w1_int_d;
    		tag_b_w1_int <= tag_b_w1_int_d;
    		
    end	

    TAG_DPRAM tag_w0 (
      .clka(clk), // input clka
      .rsta(rst),
      .wea(we_tag_a_w0), // input [3 : 0] wea
      .addra(tag_addr_a_w0), // input [7 : 0] addra --[6:0] now
      .dina(tag_data_a_w0), // input [31 : 0] dina
      .douta(tag_a_w0_int_d), // output [31 : 0] douta
      .clkb(clk), // input clkb
      .rstb(rst),
      .web(we_tag_b_w0), // input [3 : 0] web
      .addrb(tag_addr_b_w0), // input [7 : 0] addrb --[6:0] now
      .dinb(tag_data_b_w0), // input [31 : 0] dinb
      .doutb(tag_b_w0_int_d) // output [31 : 0] doutb
    );
    
  end
endgenerate

generate
 if (`TSMC_RAM_EN) begin

  wire [31:0] bit_write_en_b_tag_a_w1;
  wire [31:0] bit_write_en_b_tag_b_w1;


  assign bit_write_en_b_tag_a_w1 = {{8{~we_tag_a_w1[3]}},{8{~we_tag_a_w1[2]}},{8{~we_tag_a_w1[1]}},{8{~we_tag_a_w1[0]}}}; 
  assign bit_write_en_b_tag_b_w1 = {{8{~we_tag_b_w1[3]}},{8{~we_tag_b_w1[2]}},{8{~we_tag_b_w1[1]}},{8{~we_tag_b_w1[0]}}}; 

    TSDN65LPLLA128X32M8F tag_w1 (
      .AA(tag_addr_a_w1), 			// Address of A: Addra[6:0]
      .DA(tag_data_a_w1),			// Data in of A: douta[127:0]	
      .BWEBA(32'd0),			// Bit-Write ~en of A: {128{~en}}	
      .WEBA(~|we_tag_a_w1),.CEBA(1'b0),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
      .AB(tag_addr_b_w1),			// Address of B: Addra[6:0]
      .DB(tag_data_b_w1),				// Data in of B: douta[127:0]
      .BWEBB(32'd0),			// Bit-Write ~en of B: {128{~en}}
      .WEBB(~|we_tag_b_w1),.CEBB(1'b0),.CLKB(~clk),	// Write-~en, Chip-~en, CLKB
      .AMA(7'd0),
      .DMA(128'd0),
      .BWEBMA(128'hffff_ffff_ffff_ffff),
      .WEBMA(1'b1),.CEBMA(1'b1),
      .AMB(7'd0),
      .DMB(128'd0),
      .BWEBMB(128'hffff_ffff_ffff_ffff),
      .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
      .QA(tag_a_w1_int_d),
      .QB(tag_b_w1_int_d)
    	);

  end else begin

    assign bit_write_en_b_tag_a_w1 = 32'd0;
    assign bit_write_en_b_tag_b_w1 = 32'd0;
    //assign tag_a_w1_int_d = 32'd0; 
    //assign tag_b_w1_int_d = 32'd0;

    TAG_DPRAM tag_w1 (
      .clka(clk), // input clka
      .rsta(rst),
      .wea(we_tag_a_w1), // input [3 : 0] wea
      .addra(tag_addr_a_w1), // input [7 : 0] addra --[6:0] now
      .dina(tag_data_a_w1), // input [31 : 0] dina
      .douta(tag_a_w1_int_d), // output [31 : 0] douta
      .clkb(clk), // input clkb
      .rstb(rst),
      .web(we_tag_b_w1), // input [3 : 0] web
      .addrb(tag_addr_b_w1), // input [7 : 0] addrb --[6:0] now
      .dinb(tag_data_b_w1), // input [31 : 0] dinb
      .doutb(tag_b_w1_int_d) // output [31 : 0] doutb
    );

  end
endgenerate


//-------PMP Check -----------------


  //assign dmem_porta_permissions = 4'd3;
  //assign dmem_portb_permissions = 4'd3;
  /*wire dmem_porta_allow_n,dmem_portb_allow_n;
  reg dmem_porta_allow_q,dmem_portb_allow_q;


  always @(posedge clk) begin
    if(rst) begin
      dmem_porta_allow_q <= 1'b0;
      dmem_portb_allow_q <= 1'b0;
    end
    else begin
      dmem_porta_allow_q <= dmem_porta_allow_n;
      dmem_portb_allow_q <= dmem_portb_allow_n;
    end
  end*/

  //assign dmem_porta_allow_n = (dmem_porta_permissions[1:0] == 2'd3);
  assign dmem_porta_allow = (dmem_porta_permissions[1:0] == 2'd3);
  //assign dmem_portb_allow_n = (dmem_portb_permissions[1:0] == 2'd3);
  assign dmem_portb_allow = (dmem_portb_permissions[1:0] == 2'd3);

    rv32_mpu #(.MMU_SUPPORT(0),.MPU_SUPPORT(0)) IMEM_MPU (
     .aclk(clk),
     .aresetn(~rst),
     .imem_addr({tag_out_tlb_port1[tag_tlb_last_bit:tag_tlb_start_bit],addr_in_a[index_last_bit:offset_start_bit]}),
     .imem_allow(dmem_porta_permissions),
     .dmem_addr({tag_out_tlb_port2[tag_tlb_last_bit:tag_tlb_start_bit],addr_in_b[index_last_bit:offset_start_bit]}),
     .dmem_allow(dmem_portb_permissions),
     .csr_sb(csr_pmp_sb)
    );
  

    DTLB DTLB(
      .clk(clk), 
      .rst(rst), 
      .vpn_in_port1(addr_in_a[tag_last_bit:tag_start_bit]), //virtual page number
      .vpn_in_port2(addr_in_b[tag_last_bit:tag_start_bit]), //virtual page number
      .vpn_to_ppn_req_port1(vpn_to_ppn_req_port1),
      .vpn_to_ppn_req_port2(vpn_to_ppn_req_port2),
      .freeze_tlb(freeze_tlb),
      .dtlb_trans_off(dtlb_trans_off),
      .tag_out_port1(tag_out_tlb_port1), //Tag nothing but PPN
      .tag_out_port2(tag_out_tlb_port2), //Tag nothing but PPN
      .freeze(tlb_freeze_dcache),
      .tag_hit_port1(tag_hit_tlb_port1),
      .tag_hit_port2(tag_hit_tlb_port2),
      .csr_satp(csr_satp),
      .page_fault(data_page_fault),
      .ADDR(ADDR),
      .BURST(BURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
      .REQ(REQ),
      .WRB(WRB),
      .WDATA(WDATA),
      .RDATA(RDATA),
      .ACK(ACK),
      .STALL(STALL),
      .BSTROBE(BSTROBE)
      
    );
    
/*tlb dtlb
(
.clk(clk),
.clk_x2(clk_x2),
.rst(rst),
.vpn_to_ppn_req_port1(vpn_to_ppn_req_port1),
.vpn_to_ppn_req_port2(vpn_to_ppn_req_port2),
.vpn_in_port1(addr_in_a[tag_last_bit:tag_start_bit]),
.vpn_in_port2(addr_in_b[tag_last_bit:tag_start_bit]),
.tag_out_port1(tag_out_tlb_port1),
.tag_out_port2(tag_out_tlb_port2),
.tag_hit_port1(tag_hit_tlb_port1),
.tag_hit_port2(tag_hit_tlb_port2),
.freeze_tlb(freeze_tlb),
.freeze(tlb_freeze_dcache),
.wb_ack_i(wb_ack_i),
.wb_err_i(wb_err_i),
.wb_rty_i(wb_rty_i),
.wb_dat_i(wb_dat_i),
.wb_cyc_o(wb_cyc_o),
.wb_stb_o(wb_stb_o),
.wb_we_o (wb_we_o ),
.wb_adr_o(wb_adr_o),
.wb_bte_o(wb_bte_o),
.wb_cti_o(wb_cti_o),
.wb_sel_o(wb_sel_o),
.wb_dat_o(wb_dat_o)
);*/


endmodule

