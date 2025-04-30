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
module icache #(
                parameter offset_start_bit = 0,
                parameter offset_last_bit = 4,
                parameter index_start_bit = 5,
                parameter index_last_bit = 11,
                parameter tag_start_bit = 12,
                parameter tag_last_bit = 31,
                parameter tag_tlb_start_bit = 4,
                parameter tag_tlb_last_bit = 23,
                parameter tag_phy_start_bit = 0,
                parameter tag_phy_last_bit = 19,
                parameter tag_width=22
) (
    //----------------I/O declaration------------
    input clk/*,clk_x2*/,reset,freeze,freeze_in,
    input [255:0] wr_data,	
    input we,	
    input re,	
    //input [31:0] addr_int,
    input [31:0] i_addr,
    input [31:0] virtual_addr,
    input [1:0] state_fsm,
    //input stall,
    input stall_load,
    //input[31:0] i_addr_cache_my,
    //input[4:0] i_addr_cache_my_full,
    //input re_int,
    input vpn_to_ppn_req,
    input vpn_to_ppn_req3,
    input vpn_to_ppn_req7,
    input freeze_hit_status,

    input [31:0] csr_satp,
    output instruction_page_fault,
    
    output hit_out,
    //output reg[255:0] rd_data,	
    output reg[31:0] instr,
    output reg [tag_phy_last_bit : tag_phy_start_bit] physical_tag


`ifdef itlb_def
    ,output  [31:0]  ADDR,
    output  [1:0]   BURST, //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
    output          REQ,
    output          WRB,
    output wire [31:0]  WDATA,
    input      [31:0]  RDATA,
    input              ACK,
    input              STALL,
    output  [3:0]   BSTROBE,

    output  [tag_phy_last_bit:tag_phy_start_bit] tag_o_tlb,
    output  tag_hit,
    input tlb_trans_off,
    output freeze_tlb_out
  
`endif
);

/*
parameter offset_start_bit = 0;
parameter offset_last_bit = 4;
parameter index_start_bit = 5;
parameter index_last_bit = 11;
parameter tag_start_bit = 12;
parameter tag_last_bit = 31;
parameter tag_tlb_start_bit = 4;
parameter tag_tlb_last_bit = 23;
parameter tag_phy_start_bit = 0;
parameter tag_phy_last_bit = 19;
parameter tag_width=22;*/

//integer temp;//,tempp;
integer j;

localparam state_we0 = 2'b01;
localparam state_we1 = 2'b10;
localparam index_width = index_last_bit - index_start_bit + 1;
//LINT Check
reg [index_width -1 :0] temp;
/*
//----------------I/O declaration------------
input clk,clk_x2,reset,freeze,freeze_in;
input [255:0] wr_data;	
input we;	
input re;	
//input [31:0] addr_int;
input [31:0] i_addr;
input [31:0] virtual_addr;
input [1:0] state_fsm;
input stall;
input stall_load;
//input[31:0] i_addr_cache_my;
//input[4:0] i_addr_cache_my_full;
input re_int;
input vpn_to_ppn_req;
input vpn_to_ppn_req3;
input vpn_to_ppn_req7;
input eret_ack;
input freeze_hit_status;

output hit_out;
//output reg[255:0] rd_data;	
output reg[31:0] instr;
output reg [tag_phy_last_bit : tag_phy_start_bit] physical_tag;*/

//---------------------------------------------

//----------------ITLB Declarations------------
`ifdef itlb_def
/*
//------------ Wishbone Signals -------------- 
//input				wb_clk_i;	// clock input
//input				wb_rst_i;	// reset input
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

output  [tag_phy_last_bit:tag_phy_start_bit] tag_o_tlb;
output  tag_hit;
output freeze_tlb_out;
*/

//reg vpn_to_ppn_req;
wire [(tag_width-1+4):0] tag_out_tlb;
wire [tag_phy_last_bit : tag_phy_start_bit] tag;
wire vpn_to_ppn_req5;
reg vpn_to_ppn_req6;
reg hit;
reg hit_q;
reg tag_valid_w0 [0:127];
reg tag_valid_w1 [0:127];
reg tag_valid_w0_read;
reg tag_valid_w1_read;
wire [6:0] icache_set0_addr;
wire [6:0] icache_set1_addr;
reg we_tag_valid_w0;
reg we_tag_valid_w1;
`else
reg [tag_phy_last_bit : tag_phy_start_bit] tag;
`endif
//----------------------------------------

//--------Reg & Wire Definitions----------
wire wdirty;
wire [31:0] i_addr_min4;
wire [255:0] x,y;
reg lru_bit[0:127];
reg enable_set1;
reg flag;
reg we0,we1;
reg enable_tag0;
reg enable_tag1;
reg we_tag0,we_tag1;
reg enable_set0;
reg [1:0] state_we;
reg[1:0] nextstate_we;
reg[2:0] sel_inst;
wire [tag_phy_last_bit : tag_phy_start_bit] tag_w;
reg [(tag_last_bit - tag_start_bit + 1):0] dout1,dout2;
wire [(tag_last_bit - tag_start_bit + 1):0] dout1_mem,dout2_mem;
wire freeze_icache_miss;
wire [6:0] icache_tag_w0_addr;
wire [6:0] icache_tag_w1_addr;

//---------------------------------------

//---------------ITLB Logic-------------------------
`ifdef itlb_def
assign tag_o_tlb = tag_out_tlb[tag_tlb_last_bit:tag_tlb_start_bit];
assign freeze_icache_miss = (state_fsm == 2'b01);
`endif
//--------------------------------------------------

assign hit_out = hit || ((vpn_to_ppn_req7  ) && freeze_hit_status) ;
assign wdirty = 1;
assign i_addr_min4 = i_addr - 32'd4;

always @(posedge clk)
    begin
    if(reset) 
        begin
         state_we <= #2 state_we0;
        end    
    else if(freeze==0 )//|| ~(freeze && rdy))
        begin
         state_we <= #2 nextstate_we;
        end
    end

always @(posedge clk)
    begin
    if(reset) 
            sel_inst <=3'b0;
    else
            sel_inst <= (vpn_to_ppn_req3 || vpn_to_ppn_req5|| ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[4:2] : ( stall_load ? i_addr_min4[4:2] :i_addr[4:2]) ;
    end
always @(posedge clk)
    begin
    if(reset) begin
            vpn_to_ppn_req6 <= 1'b0;
        end
    else begin
            vpn_to_ppn_req6 <= vpn_to_ppn_req5;
        end
    end

always @(posedge clk) begin
	if(reset) hit_q <= 1'b0;
	else hit_q <= hit;
end
integer i,k;

//Tag Valid memory Write and Read Logic 
always @(posedge clk) begin
    if(reset) begin
	    for(i=0;i<127;i=i+1) begin
		tag_valid_w0[i] <= 1'b0;
	    end 
	    for(k=0;k<127;k=k+1) begin
		tag_valid_w1[k] <= 1'b0;
	    end
	    tag_valid_w0_read <= 1'b0;	    
	    tag_valid_w1_read <= 1'b0;	    
    end
    else if(we_tag_valid_w0) begin
	    tag_valid_w0[icache_tag_w0_addr] <= 1'b1;
	    //if(~lru_bit[temp]) tag_valid_w0[icache_tag_w0_addr] <= 1'b1;
            //else tag_valid_w1[icache_tag_w1_addr] <= 1'b1;
	    
    end
    else if(we_tag_valid_w1) begin
	    tag_valid_w1[icache_tag_w1_addr] <= 1'b1;
    end
    else begin
	    tag_valid_w0_read <= tag_valid_w0[icache_tag_w0_addr];
	    tag_valid_w1_read <= tag_valid_w1[icache_tag_w0_addr];
    end

end

/*always @(posedge clk) begin
    if(reset) begin
	    tag_valid_w0_read <= 1'b0;
	    tag_valid_w1_read <= 1'b0;
    end
    else if(vpn_to_ppn_req3) begin
	    tag_valid_w0_read <= tag_valid_w0[temp];
	    tag_valid_w1_read <= tag_valid_w1[temp];
    end
end*/
always @(posedge clk)
    begin
    if(reset) begin
            physical_tag <=20'b0;
            //tag_valid <= 1'b0;
    end
    else if( tag_hit && ~hit && (state_fsm != 2'b01)) begin
            physical_tag <= tag_w;
    end
    end

always @(*) begin
  if(reset) begin
    enable_set0 = 0;
    enable_set1 = 0;
    enable_tag0 = 0;
    enable_tag1 = 0;
  end
  else begin
    enable_set0 = 1'b1;
    enable_set1 = 1'b1;
    enable_tag0 = 1'b1;
    enable_tag1 = 1'b1;
  end
end
always @(*)
	//LINT Violation
    /*begin
    if (reset)
        begin
        enable_set0=0;
        enable_set1=0;
        enable_tag0=0;
        enable_tag1=0;
        nextstate_we=state_we0;
        hit=0;
        we0=1'b0;
        we1=0;
        we_tag0=0;
        we_tag1=0;
        end
    else*/ //if(freeze==0)
        begin
        //enable_set1=1;
        //enable_tag1=1;
        //enable_set0=1;
        //enable_tag0=1;
	hit =0;
	we0 =1'b0;
	we1 =1'b0;
	we_tag0 = 1'b0;
	we_tag1 = 1'b0;
	we_tag_valid_w0 = 1'b0;
	we_tag_valid_w1 = 1'b0;
	nextstate_we = state_we0;
        case(state_we)               
            state_we0:
            begin 
              if(we)
                begin
                    hit=1'b0;
                    we0=1'b0;
                    we1=0;
                    we_tag0=0;
                    we_tag1=0;
		    we_tag_valid_w0 = 1'b0;
		    we_tag_valid_w1 = 1'b0;
                    nextstate_we=state_we1;
                end
                else  if(re)
                begin
                    we0=1'b0;
                    we1=0;
                    we_tag0=0;
                    we_tag1=0;
		    we_tag_valid_w0 = 1'b0;
		    we_tag_valid_w1 = 1'b0;		    
                    nextstate_we=state_we0;
		    hit = (tag_hit ? ((tag_valid_w1_read && (tag==dout2[(tag_last_bit - tag_start_bit):0])) ? 1'b1 :
			    	      (tag_valid_w0_read && (tag==dout1[(tag_last_bit - tag_start_bit):0])) ? 1'b1 : 1'b0)
				      : 1'b0);

                    //hit= (tag_hit ? (((tag_valid_w0_read || tag_valid_w1_read) && ((tag==dout2[(tag_last_bit - tag_start_bit):0])||(tag ==dout1[(tag_last_bit - tag_start_bit):0]))) ? 1'b1 : 1'b0) : 0) ;
                    //hit= (tag_hit ? ((((dout2[tag_last_bit-tag_start_bit+1] === 1'b1) || (dout1[tag_last_bit-tag_start_bit+1]=== 1'b1)) && ((tag==dout2[(tag_last_bit - tag_start_bit):0])||(tag ==dout1[(tag_last_bit - tag_start_bit):0]))) ? 1'b1 : 1'b0) : 0) ;

                end
                else
                begin
                    hit=1'b0;
                    we0=1'b0;
                    we1=0;
                    we_tag0=0;
                    we_tag1=0;
		    we_tag_valid_w0 = 1'b0;
		    we_tag_valid_w1 = 1'b0;		    
                    nextstate_we=state_we0;
                end
            end
            state_we1:
            begin
//                tempp= vpn_to_ppn_req3 ? virtual_addr[index_last_bit:index_start_bit]: i_addr[index_last_bit:index_start_bit];
                if(~lru_bit[temp])//(dout2[24:0] != addr[29:5]) //&&  (lru_bit[addr[4:0]]==1'b0))//tag not match
                begin
                    hit=1'b0;
                    we1=0;
                    we_tag1=0;
                    we0=1'b1;
                    we_tag0=1;
		    we_tag_valid_w0 = 1'b1;
		    we_tag_valid_w1 = 1'b0;		    
                end
                else
                begin
                    hit=0;
                    we1=1;
                    we_tag1=1;
                    we0=1'b0;
                    we_tag0=0;
		    we_tag_valid_w0 = 1'b0;
		    we_tag_valid_w1 = 1'b1;		    
                end
                nextstate_we=state_we0;
            end
            default:
            begin
                we1=0;
                we_tag1=0;
                we0=0;
                we_tag0=0;
                hit=0;
		    we_tag_valid_w0 = 1'b0;
		    we_tag_valid_w1 = 1'b0;		
                nextstate_we=state_we0;             
            end
        endcase
        //end
    end

always @( posedge clk)
    begin
    if(reset)
        begin
	temp <= 0;
        for(j=0;j<128;j=j+1)
        lru_bit[j]<=0;
        end
    else //if(freeze==0)
        begin
        temp<= vpn_to_ppn_req3 ? virtual_addr[index_last_bit:index_start_bit]: i_addr[index_last_bit:index_start_bit];
        if(state_we==state_we0 && re)
            begin
                if(hit) begin
                    if((tag_valid_w0_read) && (tag==dout1[(tag_last_bit-tag_start_bit):0]))
                        lru_bit[temp]<=1'b1;
                    else if((tag_valid_w1_read) && (tag==dout2[(tag_last_bit-tag_start_bit):0]))
                        lru_bit[temp]<=1'b0;
                    end
            end
        end
    end

`ifdef itlb_def
    assign tag = tag_out_tlb[tag_tlb_last_bit : tag_tlb_start_bit];
    assign tag_w = tag_out_tlb[tag_tlb_last_bit : tag_tlb_start_bit];
`else
always @(posedge clk)
    begin
    if(reset) 
            tag <=20'b0;
    else
            tag <= vpn_to_ppn_req3 ? virtual_addr[tag_last_bit:tag_start_bit] : i_addr[tag_last_bit:tag_start_bit] ;
    end
    assign tag_w = virtual_addr[tag_last_bit:tag_start_bit] ;
`endif

always@(*)//posedge clk or posedge reset)
    begin
   // LINT CHECK
   /*if(reset)
        begin
        instr<=0;
        end*/
    instr = 0;
    if(hit) //&& ~(i_addr == virtual_addr))
            begin
            if ((tag_valid_w1_read) && (tag == dout2[(tag_last_bit-tag_start_bit):0])) 
                begin
                case(sel_inst)
                    3'b000: instr = y[31:0];
                    3'b001: instr = y[63:32];
                    3'b010: instr = y[95:64];
                    3'b011: instr = y[127:96];
                    3'b100: instr = y[159:128];
                    3'b101: instr = y[191:160];
                    3'b110: instr = y[223:192];
                    3'b111: instr = y[255:224];
                    default: instr =0;
                endcase
                end
            else
                begin
                case(sel_inst)
                    3'b000: instr = x[31:0];
                    3'b001: instr = x[63:32];
                    3'b010: instr = x[95:64];
                    3'b011: instr = x[127:96];
                    3'b100: instr = x[159:128];
                    3'b101: instr = x[191:160];
                    3'b110: instr = x[223:192];
                    3'b111: instr = x[255:224];
                    default: instr =0;
                endcase
                end
            end
        else 
            instr = 32'b0;   
    end


//---------------Module instantiation------------------------------
//-----------------ITLB--------------------------------------------
`ifdef itlb_def

/*itlb itlb(
.clk(clk),
.clk_x2(clk_x2),
.rst(reset),
.vpn_to_ppn_req(vpn_to_ppn_req || vpn_to_ppn_req6 || vpn_to_ppn_req7),
.vpn((vpn_to_ppn_req3 || (vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[tag_last_bit:tag_start_bit]: ( stall_load ? i_addr_min4[tag_last_bit:tag_start_bit]: i_addr[tag_last_bit:tag_start_bit])), 
.freeze_tlb(freeze_in || freeze_icache_miss),
.tag_out(tag_out_tlb),
.freeze(freeze_tlb_out),
.tag_hit(tag_hit),
.vpn_to_ppn_req5(vpn_to_ppn_req5),
.wb_ack_i(wb_ack_i),.wb_err_i(wb_err_i),.wb_rty_i(wb_rty_i),.wb_dat_i(wb_dat_i),.wb_cyc_o(wb_cyc_o),
.wb_stb_o(wb_stb_o),.wb_we_o(wb_we_o),.wb_adr_o(wb_adr_o),.wb_bte_o(wb_bte_o),.wb_cti_o(wb_cti_o),
.wb_sel_o(wb_sel_o),.wb_dat_o(wb_dat_o)         );*/

ITLB ITLB(
.clk(clk),
//.clk_x2(clk_x2),
.rst(reset),
.vpn_to_ppn_req(vpn_to_ppn_req || vpn_to_ppn_req6 || vpn_to_ppn_req7),
.vpn((vpn_to_ppn_req3 || (vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[tag_last_bit:tag_start_bit]: ( stall_load ? i_addr_min4[tag_last_bit:tag_start_bit]: i_addr[tag_last_bit:tag_start_bit])),
.tlb_trans_off(tlb_trans_off),
.freeze_tlb(freeze_in || freeze_icache_miss),
.tag_out(tag_out_tlb),
.freeze(freeze_tlb_out),
.page_fault_processor(instruction_page_fault),
.tag_hit(tag_hit),
.csr_satp(csr_satp),
.vpn_to_ppn_req5(vpn_to_ppn_req5),
      .ADDR(ADDR),
      .BURST(BURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
      .REQ(REQ),
      .WRB(WRB),
      .WDATA(WDATA),
      .RDATA(RDATA),
      .ACK(ACK),
      .STALL(STALL),
      .BSTROBE(BSTROBE)
/*
.wb_ack_i(wb_ack_i),.wb_err_i(wb_err_i),.wb_rty_i(wb_rty_i),.wb_dat_i(wb_dat_i),.wb_cyc_o(wb_cyc_o),
.wb_stb_o(wb_stb_o),.wb_we_o(wb_we_o),.wb_adr_o(wb_adr_o),.wb_bte_o(wb_bte_o),.wb_cti_o(wb_cti_o),
.wb_sel_o(wb_sel_o),.wb_dat_o(wb_dat_o) */        );


`endif

///////////////////////////////////////////////////////////////
//// If tag bits match and line is valid then we have a hit //
/////////////////////////////////////////////////////////////
//							   /
//		INSTRUCTION CACHE MEMORY CURTS		   /	
////////////////////////////////////////////////////////////

//-----SET0 Instruction Cache-------------------------------
wire [127:0] x_low,x_high;
reg [127:0] x_low_q,x_high_q;
assign x = {x_high_q,x_low_q};


assign icache_set0_addr = (vpn_to_ppn_req3 || vpn_to_ppn_req5 || we0 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] );

generate
  if(`TSMC_RAM_EN) begin

    always @(posedge clk) begin
          if(reset) begin
    	 x_low_q <= 128'd0;
    	 x_high_q <= 128'd0; 
          end
          else begin
    	 x_low_q <= x_low;
    	 x_high_q <= x_high;
          end
    end
    
    TSDN65LPLLA128X128M4F set0_0 (
      .AA(icache_set0_addr), 			// Address of A: Addra[6:0]
      .DA(wr_data[127:0]),			// Data in of A: douta[127:0]	
      .BWEBA({128{~we0}}),			// Bit-Write ~en of A: {128{~en}}	
      .WEBA(~we0),.CEBA(~enable_set0),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
      .AB(7'd0),			// Address of B: Addra[6:0]
      .DB(128'd0),			// Data in of B: douta[127:0]
      .BWEBB({128{1'b1}}),			// Bit-Write ~en of B: {128{~en}}
      .WEBB(1'b1),.CEBB(1'b1),.CLKB(1'b0),	// Write-~en, Chip-~en, CLKB
      .AMA(7'd0),
      .DMA(128'd0),
      .BWEBMA(128'hffff_ffff_ffff_ffff),
      .WEBMA(1'b1),.CEBMA(1'b1),
      .AMB(7'd0),
      .DMB(128'd0),
      .BWEBMB(128'hffff_ffff_ffff_ffff),
      .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
      .QA(x_low),
      .QB()
    	);
    
    
    TSDN65LPLLA128X128M4F set0_1 (
      .AA(icache_set0_addr), 			// Address of A: Addra[6:0]
      .DA(wr_data[255:128]),			// Data in of A: douta[127:0]	
      .BWEBA({128{~we0}}),			// Bit-Write ~en of A: {128{~en}}	
      .WEBA(~we0),.CEBA(~enable_set0),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
      .AB(7'd0),			// Address of B: Addra[6:0]
      .DB(128'd0),			// Data in of B: douta[127:0]
      .BWEBB({128{1'b1}}),			// Bit-Write ~en of B: {128{~en}}
      .WEBB(1'b1),.CEBB(1'b1),.CLKB(1'b0),	// Write-~en, Chip-~en, CLKB
      .AMA(7'd0),
      .DMA(128'd0),
      .BWEBMA(128'hffff_ffff_ffff_ffff),
      .WEBMA(1'b1),.CEBMA(1'b1),
      .AMB(7'd0),
      .DMB(128'd0),
      .BWEBMB(128'hffff_ffff_ffff_ffff),
      .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
      .QA(x_high),
      .QB()
    	);
    end
    else begin
      always @(*) begin
      	 x_low_q = x_low;
      	 x_high_q = x_high;
      end
      ICACHE_DPRAM set0_0 (
        .clka(clk), // input clka
        .rsta(reset), // input rsta
        .ena(enable_set0 ), // input ena
        .wea(we0), // input [0 : 0] wea
        .addra(icache_set0_addr), // input [4 : 0] addra
        .dina( wr_data[127:0]), // input [127 : 0] dina
        .douta(x_low) // output [127 : 0] douta
      );
      
      ICACHE_DPRAM set0_1 (
        .clka(clk), // input clka
        .rsta(reset), // input rsta
        .ena(enable_set0 ), // input ena
        .wea(we0), // input [0 : 0] wea
        .addra(icache_set0_addr), // input [4 : 0] addra
        .dina( wr_data[255:128]), // input [127 : 0] dina
        .douta(x_high) // output [127 : 0] douta
      );
    end
  endgenerate

/*	
MEMORY_MACRO_CACHE #(.ADDR_WIDTH(7),.DATA_WIDTH(256),.INIT_FILE(0)) set0 (
  .clka(clk), // input clka
  .rsta(reset), // input rsta
  .ena(enable_set0 ), // input ena
  .wea(we0), // input [0 : 0] wea
  .addra((vpn_to_ppn_req3 || vpn_to_ppn_req5 || we0 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] )), // input [4 : 0] addra
  .dina( wr_data), // input [127 : 0] dina
  .douta(x) // output [127 : 0] douta
);*/



//-----SET1 Instruction Cache-------------------------------

wire [127:0] y_low,y_high;
reg [127:0] y_low_q,y_high_q;
assign y = {y_high_q,y_low_q};


assign icache_set1_addr = (vpn_to_ppn_req3 || vpn_to_ppn_req5 || we1 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] ) ;

generate
  if(`TSMC_RAM_EN) begin
    always @(posedge clk) begin
          if(reset) begin
    	 y_low_q <= 128'd0;
    	 y_high_q <= 128'd0; 
          end
          else begin
    	 y_low_q <= y_low;
    	 y_high_q <= y_high;
          end
    end

      TSDN65LPLLA128X128M4F set1_0 (
        .AA(icache_set1_addr), 			// Address of A: Addra[6:0]
        .DA(wr_data[127:0]),			// Data in of A: douta[127:0]	
        .BWEBA({128{~we1}}),			// Bit-Write ~en of A: {128{~en}}	
        .WEBA(~we1),.CEBA(~enable_set1),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
        .AB(7'd0),			// Address of B: Addra[6:0]
        .DB(128'd0),			// Data in of B: douta[127:0]
        .BWEBB({128{1'b1}}),			// Bit-Write ~en of B: {128{~en}}
        .WEBB(1'b1),.CEBB(1'b1),.CLKB(1'b0),	// Write-~en, Chip-~en, CLKB
        .AMA(7'd0),
        .DMA(128'd0),
        .BWEBMA(128'hffff_ffff_ffff_ffff),
        .WEBMA(1'b1),.CEBMA(1'b1),
        .AMB(7'd0),
        .DMB(128'd0),
        .BWEBMB(128'hffff_ffff_ffff_ffff),
        .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
        .QA(y_low),
        .QB()
      	);
      
      
      TSDN65LPLLA128X128M4F set1_1 (
        .AA(icache_set0_addr), 			// Address of A: Addra[6:0]
        .DA(wr_data[255:128]),			// Data in of A: douta[127:0]	
        .BWEBA({128{~we0}}),			// Bit-Write ~en of A: {128{~en}}	
        .WEBA(~we0),.CEBA(~enable_set1),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
        .AB(7'd0),			// Address of B: Addra[6:0]
        .DB(128'd0),			// Data in of B: douta[127:0]
        .BWEBB({128{1'b1}}),			// Bit-Write ~en of B: {128{~en}}
        .WEBB(1'b1),.CEBB(1'b1),.CLKB(1'b0),	// Write-~en, Chip-~en, CLKB
        .AMA(7'd0),
        .DMA(128'd0),
        .BWEBMA(128'hffff_ffff_ffff_ffff),
        .WEBMA(1'b1),.CEBMA(1'b1),
        .AMB(7'd0),
        .DMB(128'd0),
        .BWEBMB(128'hffff_ffff_ffff_ffff),
        .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
        .QA(y_high),
        .QB()
      	);
      end
      else begin

        always @(*) begin
        	 y_low_q = y_low;
        	 y_high_q = y_high;
        end
        ICACHE_DPRAM set1_0 (
          .clka(clk), // input clka
          .rsta(reset), // input rsta
          .ena(enable_set1 ), // input ena
          .wea(we1), // input [0 : 0] wea
          .addra(icache_set1_addr), // input [4 : 0] addra
          .dina( wr_data[127:0]), // input [127 : 0] dina
          .douta(y_low) // output [127 : 0] douta
        );
        
        ICACHE_DPRAM set1_1 (
          .clka(clk), // input clka
          .rsta(reset), // input rsta
          .ena(enable_set1 ), // input ena
          .wea(we1), // input [0 : 0] wea
          .addra(icache_set1_addr), // input [4 : 0] addra
          .dina( wr_data[255:128]), // input [127 : 0] dina
          .douta(y_high) // output [127 : 0] douta
        );
      end
    endgenerate
/*
MEMORY_MACRO_CACHE #(.ADDR_WIDTH(7),.DATA_WIDTH(256),.INIT_FILE(0)) set1 (
  .clka(clk), // input clka
  .rsta(reset), // input rsta
  .ena(enable_set1 ), // input ena
  .wea(we1), // input [0 : 0] wea
  .addra((vpn_to_ppn_req3 || vpn_to_ppn_req5 || we1 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] )), // input [4 : 0] addra
  .dina(wr_data), // input [255 : 0] dina
  .douta(y) // output [255: 0] douta
);*/

//--------------Tag Array Set0------------------------------------------------
assign icache_tag_w0_addr = (vpn_to_ppn_req3 || vpn_to_ppn_req5 || we_tag0 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] );

assign icache_tag_w1_addr = (vpn_to_ppn_req3 || vpn_to_ppn_req5 || we_tag1 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] );

generate
  if(`TSMC_RAM_EN) begin
    always @(posedge clk) begin
          if(reset) begin
    	 dout1 <= 21'd0;
    	 dout2 <= 21'd0; 
          end
          else begin
    	 dout1 <= dout1_mem;
    	 dout2 <= dout2_mem;
          end
    end
    
    
    TSDN65LPLLA128X21M8F tag0_v_dirty (
      .AA(icache_tag_w0_addr), 			// Address of A: Addra[6:0]
      .DA({tag_valid_w0[icache_set0_addr], physical_tag}),			// Data in of A: douta[127:0]	
      .BWEBA({21{~we_tag0}}),			// Bit-Write ~en of A: {128{~en}}	
      .WEBA(~we_tag0),.CEBA(~enable_tag0),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
      .AB(7'd0),			// Address of B: Addra[6:0]
      .DB(128'd0),			// Data in of B: douta[127:0]
      .BWEBB({128{1'b1}}),			// Bit-Write ~en of B: {128{~en}}
      .WEBB(1'b1),.CEBB(1'b1),.CLKB(1'b0),	// Write-~en, Chip-~en, CLKB
      .AMA(7'd0),
      .DMA(128'd0),
      .BWEBMA(128'hffff_ffff_ffff_ffff),
      .WEBMA(1'b1),.CEBMA(1'b1),
      .AMB(7'd0),
      .DMB(128'd0),
      .BWEBMB({21{1'b1}}),
      .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
      .QA(dout1_mem),
      .QB()
    	);

    TSDN65LPLLA128X21M8F tag1_v_dirty (
      .AA(icache_tag_w1_addr), 			// Address of A: Addra[6:0]
      .DA({tag_valid_w1[icache_tag_w1_addr], physical_tag}),			// Data in of A: douta[127:0]	
      .BWEBA({21{~we_tag1}}),			// Bit-Write ~en of A: {128{~en}}	
      .WEBA(~we_tag1),.CEBA(~enable_tag1),.CLKA(~clk),	// Write-~en, Chip-~en, CLKA	
      .AB(7'd0),			// Address of B: Addra[6:0]
      .DB(128'd0),			// Data in of B: douta[127:0]
      .BWEBB({128{1'b1}}),			// Bit-Write ~en of B: {128{~en}}
      .WEBB(1'b1),.CEBB(1'b1),.CLKB(1'b0),	// Write-~en, Chip-~en, CLKB
      .AMA(7'd0),
      .DMA(128'd0),
      .BWEBMA(128'hffff_ffff_ffff_ffff),
      .WEBMA(1'b1),.CEBMA(1'b1),
      .AMB(7'd0),
      .DMB(128'd0),
      .BWEBMB({21{1'b1}}),
      .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
      .QA(dout2_mem),
      .QB()
    	);
      
    end
    else begin
    always @(*) begin
    	 dout1 = dout1_mem;
    	 dout2 = dout2_mem;
     end

     TAG_ICACHE_RAM tag0_v_dirty (
       .clka(clk), // input clka
       .rsta(reset), // input rsta
       .ena(enable_tag0 ), // input ena
       .wea(we_tag0), // input [0 : 0] wea
       .addra(icache_tag_w0_addr), // input [4 : 0] addra
       .dina({tag_valid_w0[icache_set0_addr], physical_tag}), // input [18 : 0] dina
       .douta(dout1_mem) // output [18 : 0] douta
     );

     TAG_ICACHE_RAM tag1_v_dirty (
       .clka(clk), // input clka
       .rsta(reset), // input rsta
       .ena(enable_tag1), // input ena
       .wea(we_tag1), // input [0 : 0] wea
       .addra(icache_tag_w1_addr), // input [4 : 0] addra
       .dina({tag_valid_w1[icache_tag_w1_addr], physical_tag}), // input [18 : 0] dina
       .douta(dout2_mem) // output [18 : 0] douta
     );
        
    end
  endgenerate
/*
MEMORY_MACRO_CACHE #(.ADDR_WIDTH(7),.DATA_WIDTH(21),.INIT_FILE(0)) tag0_v_dirty (
  .clka(clk), // input clka
  .rsta(reset), // input rsta
  .ena(enable_tag0 ), // input ena
  .wea(we_tag0), // input [0 : 0] wea
  .addra((vpn_to_ppn_req3 || vpn_to_ppn_req5 || we_tag0 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] )), // input [4 : 0] addra
  .dina({tag_valid, physical_tag}), // input [18 : 0] dina
  .douta(dout1) // output [18 : 0] douta
);*/

/*blk_mem_gen_v7_3_2 tag0_v_dirty (
  .clka(clk), // input clka
  .rsta(reset), // input rsta
  .ena(enable_tag0 ), // input ena
  .wea(we_tag0), // input [0 : 0] wea
  .addra((vpn_to_ppn_req3 || vpn_to_ppn_req5 || we_tag0 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] )), // input [4 : 0] addra
  .dina(physical_tag), // input [18 : 0] dina
  .douta(dout1) // output [18 : 0] douta
);*/

//------------Tag Array Set1-------------------------------------------

//wire [6:0] icache_tag_w1_addr;



/*
MEMORY_MACRO_CACHE #(.ADDR_WIDTH(7),.DATA_WIDTH(21),.INIT_FILE(0)) tag1_v_dirty (
  .clka(clk), // input clka
  .rsta(reset), // input rsta
  .ena(enable_tag1), // input ena
  .wea(we_tag1), // input [0 : 0] wea
  .addra((vpn_to_ppn_req3 || vpn_to_ppn_req5 || we_tag1 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] )), // input [4 : 0] addra
  .dina({tag_valid, physical_tag}), // input [24 : 0] dina
  .douta(dout2) // output [24 : 0] douta
);*/

/*blk_mem_gen_v7_3_2 tag1_v_dirty (
  .clka(clk), // input clka
  .rsta(reset), // input rsta
  .ena(enable_tag1), // input ena
  .wea(we_tag1), // input [0 : 0] wea
  .addra((vpn_to_ppn_req3 || vpn_to_ppn_req5 || we_tag1 || ( vpn_to_ppn_req7 && ~freeze_hit_status )) ? virtual_addr[index_last_bit:index_start_bit]: ( stall_load ? i_addr_min4[index_last_bit:index_start_bit] : i_addr[index_last_bit:index_start_bit] )), // input [4 : 0] addra
  .dina(physical_tag), // input [24 : 0] dina
  .douta(dout2) // output [24 : 0] douta
);*/

endmodule
