`timescale 1ns / 1ps
/////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02.01.2017 12:47:44
// Design Name: 
// Module Name: tlb
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
/////////////////////////////////////////////////////////////////

//////////Guidelines////////////////////////


module ITLB 
  #(parameter tag_width = 22, 
    parameter virtual_width=32,
    parameter vpn_width=20,
    parameter offset=12,
    parameter dw = 32,
    parameter aw = 32,
    parameter bl = 8
  )
(
  input clk, rst,clk_x2,
  input [vpn_width-1:0] vpn, //virtual page number
  input vpn_to_ppn_req,
  input freeze_tlb,

  input tlb_trans_off,

  output [(tag_width-1+4):0] tag_out, //Tag nothing but PPN
  output reg freeze,
  //output stall_out,
  output tag_hit,
  output vpn_to_ppn_req5,

  input [31:0] csr_satp,
  output page_fault,

  //-------------MEMORY INTERFACE---------------------
   output reg [31:0]  ADDR,
   output reg [1:0]   BURST, //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
   output reg         REQ,
   output reg         WRB,
   output wire [31:0]  WDATA,
   input      [31:0]  RDATA,
   input              ACK,
   input              STALL,
   output reg [3:0]   BSTROBE
/*
  input				wb_ack_i,	// normal termination
  input				wb_err_i,	// termination w/ error
  input				wb_rty_i,	// termination w/ retry
  input  [31:0]       wb_dat_i,
  output reg				wb_cyc_o,
  output reg              wb_stb_o,	// strobe output reg
  output reg              wb_we_o,	// indicates write transfer
  output reg [31:0] 		wb_adr_o,	
  output reg [1:0]        wb_bte_o,
  output reg [2:0] 		wb_cti_o,
  output reg [3:0]        wb_sel_o,	// byte select output regs for the signals-byte select and extend
  output reg [31:0]       wb_dat_o	// output reg data bus*/
);

//change the following according to the design//////
`define VPN_MASK 'hFFFFF000

/*
parameter tag_width=22;
parameter virtual_width=32;
parameter vpn_width=20;
parameter offset=12;

parameter dw = 32;
parameter aw = 32;
parameter bl = 8;


input clk, rst,clk_x2;
input [vpn_width-1:0] vpn; //virtual page number
input vpn_to_ppn_req;
input freeze_tlb;

output [(tag_width-1+4):0] tag_out; //Tag nothing but PPN
output reg freeze;
//output stall_out;
output tag_hit;
output vpn_to_ppn_req5;

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
output [31:0]       wb_dat_o;	// output data bus */

//--------Wire Definitions----------------------
wire [51:0] write_data;
wire [4:0] write_addr;
wire we;
wire miss;
wire [25:0] output_data;

wire full;
wire empty;
wire wr_en;
wire rd_en;
wire [4:0] din;
wire [4:0] dout;
wire wb_ack;
wire valid_data;
wire [4:0] access_addr;
wire [4:0] lru_reg_int;
wire [4:0] eva_reg_int;
//wire [vpn_width-1:0] vpn;
reg [vpn_width-1:0] vpn_int;

//wire TLB_MISS;
wire TLB_VALID;
wire [25:0] TLB_DATA;

reg  compare;
reg  read_cam;
reg [31:0] data_in;
reg [4:0] lru_reg;
//reg [4:0] eva_reg;
reg [3:0] state_PTW;
reg [3:0] next_state_PTW;
reg access;


reg [31:0] data_in_L1,data_in_L2;
wire next_pte;
reg [31:0] next_addr, addr_pte;
wire  leaf_page;

//---------PAGE TABLE WALKER FSM----------------
localparam IDLE          = 3'd0;
localparam FIRST_LEVEL   = 3'd1;
localparam SECOND_LEVEL  = 3'd2;
localparam REPLACE       = 3'd3;
localparam OUTPUT_STATE  = 3'd4;

localparam BASE_ADDR = 32'd72;

/*
parameter idle = 2'b00;
parameter send_entry_req = 2'b01;
//parameter wait_for_data = 2;
parameter replace = 2'b10;
parameter output_state_PTW = 2'b11;*/

assign tag_out = output_data;
assign tag_hit = valid_data;
assign we = leaf_page && ACK && REQ;
//assign we = wb_ack_i & wb_stb_o;
assign write_data = { {vpn_int}, {data_in} };
assign write_addr = lru_reg;
//assign write_addr = eva_reg;
//assign vpn = (virtual_addr & `VPN_MASK) >> offset;
assign vpn_to_ppn_req5 = read_cam;
//assign wb_dat_o = 32'b0;

always @(posedge clk)
begin
        if(rst) begin
//            re <= 1'b0;
            vpn_int <= 20'b0;
        end
        else if(vpn_to_ppn_req & (state_PTW == IDLE) & (~miss)) begin
//            re <= 1'b1;
            vpn_int <= vpn;
        end
//        else re <= 1'b0;
end

always @(posedge clk)
begin
        if(rst)  access <= 1'b0;
        else if(~miss) access <= 1'b1;
        else access <= 1'b0;
end


//---------PAGE TABLE WALKER DESIGN ------------------------------
// Virtual Address:     [------------VPN[31:12]--------------------][-------Page offset[11:0]]
//                                |                     |                   
//                                |                     |
//                                |                     |
//                                V                     V
// Virtual Address:     [-----VPN[31:22]-----][------VPN[21:12-----][-------Page offset[11:0]]
//                                |                     | 
//                                |                     |
//                                |                     |
//                                |                     |
//                                V                     V
// Virtual Address:     [-----PPN[31:22]-----][------PPN[21:12-----][-------Page offset[11:0]]

always @(posedge clk)
begin
    if(rst) begin 
        state_PTW <= IDLE;
        lru_reg <= 5'b0;
//        eva_reg = 5'b0;
    end
    else if (~(freeze_tlb)) begin  
        state_PTW <= next_state_PTW;
        lru_reg <= lru_reg_int;
//        eva_reg = eva_reg_int;
    end
end

always @(posedge clk) begin
  if(rst) addr_pte <= {1'b0,csr_satp[30:0]};
  else if (~freeze_tlb) addr_pte <= next_addr;
end

always @(*)
begin
    case (state_PTW)
        IDLE:   begin
                REQ    =  1'b0;
                BURST    =  2'b00;   
                WRB     =  1'b0;
                BSTROBE    =  4'hf;
                ADDR    =  {aw{1'b0}};
                data_in     =  32'b0;
                read_cam    = 1'b0;
                compare     = 1'b0;
                    if(miss) begin
                        next_state_PTW = FIRST_LEVEL;
                        freeze  = 1'b1;
                        next_addr = {1'b0,csr_satp[30:0]};
                    end
                    else begin           
                        next_state_PTW = IDLE;
                        freeze  = 1'b0;
                        next_addr = 32'd0;
                    end    
                end
                
        FIRST_LEVEL:   begin
                REQ    =  1'b1;
                BURST    =  2'b00;   
                WRB     =  1'b0;
                BSTROBE    =  4'hf;
                ADDR    =  addr_pte + {20'd0,vpn_int[19:10],2'b00}; //if accessing pte violates a PMA or PMP check, raise an access-fault exception corresponding to the original access type
                freeze      = 1'b1;
                read_cam    = 1'b0;
                compare     = 1'b0;
                data_in     = RDATA;
                  if(ACK && ~STALL && next_pte) begin 
                    next_state_PTW  = SECOND_LEVEL;
                    next_addr = {data_in[29:10],12'd0};
                    //compare    = 1'b0;
                  end
                  else if (ACK && ~STALL && leaf_page) begin
                    next_state_PTW  = REPLACE;
                    next_addr = 32'd0;
                    //compare    = 1'b1;
                  end
                  else if(page_fault) begin
                    next_state_PTW = IDLE;
                    next_addr = 32'd0;
                  end                   
                  else begin 
                    next_state_PTW  = FIRST_LEVEL;
                    next_addr = addr_pte;
                  end
                end
                   
        SECOND_LEVEL:   begin
                REQ    =  1'b1;
                BURST    =  2'b00;   
                WRB     =  1'b0;
                BSTROBE    =  4'hf;
                ADDR    =  addr_pte + {20'd0,vpn_int[9:0],2'b00}; //if accessing pte violates a PMA or PMP check, raise an access-fault exception corresponding to the original access type        
                freeze      = 1'b1;
                read_cam    = 1'b0;
                compare     = 1'b1;
                data_in     = RDATA;
                  if(ACK && ~STALL && leaf_page) begin
                        next_state_PTW  = REPLACE;
                        next_addr = 32'd0;
                  end
                  else if (page_fault) begin
                        next_state_PTW = IDLE;
                        next_addr = 32'd0;
                  end                  
                else 
                        next_state_PTW  = SECOND_LEVEL;
                      next_addr = addr_pte;
                end
        REPLACE:   begin

                REQ    =  1'b0;
                BURST    =  2'b00;   
                WRB     =  1'b0;
                BSTROBE    =  4'hf;
                ADDR    = {aw{1'b0}}; 
                freeze  = 1'b1;
                read_cam    = 1'b0;
                compare     = 1'b0;
                next_state_PTW  = OUTPUT_STATE;
                data_in     = RDATA;
                end
                
        OUTPUT_STATE: begin
                REQ    =  1'b0;
                BURST    =  2'b00;   
                WRB     =  1'b0;
                BSTROBE    =  4'hf;
                ADDR    = {aw{1'b0}}; 
                freeze      = 1'b1;
                compare     = 1'b0;
                read_cam    = 1'b1;
                next_state_PTW  = IDLE;            
                data_in     =  32'b0;
                end       
                
        default:begin
                REQ    =  1'b0;
                BURST    =  2'b00;   
                WRB     =  1'b0;
                BSTROBE    =  4'hf;
                ADDR    = {aw{1'b0}}; 
                data_in     =  32'b0;
                freeze  = 1'b0;
                compare     = 1'b0;
                read_cam    = 1'b0;

                end
        endcase

end
/*
always @(*)
begin
    case (state_PTW)
        idle:   begin
                wb_cyc_o	=  1'b0;
                wb_stb_o    =  1'b0;
                wb_cti_o    =  3'b111;  
                wb_bte_o    =  2'b00;   
                wb_we_o     =  1'b0;
                wb_sel_o    =  4'hf;
                wb_adr_o    =  {aw{1'b0}};
                data_in     =  32'b0;
                read_cam    = 1'b0;
                compare     = 1'b0;
                    if(miss) begin
                        freeze  = 1'b1;
                        next_state_PTW = send_entry_req;
                    end
                    else  begin       
                        next_state_PTW = idle;
                        freeze  = 1'b0;
                    end
                end
                
        send_entry_req:   begin
                wb_cyc_o	=  1'b1;
                wb_stb_o    =  1'b1;
                wb_cti_o    =  3'b111;  
                wb_bte_o    =  2'b00;   
                wb_we_o     =  1'b0;
                wb_sel_o    =  4'hf;
                wb_adr_o    =  {10'b0,vpn_int,2'b0};
                freeze  = 1'b1;
                read_cam    = 1'b0;
                compare     = 1'b1;
                data_in     = wb_dat_i;
                    if(wb_ack) 
                        next_state_PTW  = replace;                   
                    else 
                        next_state_PTW  = send_entry_req;
                end
                   
        replace:   begin
                wb_cyc_o	=  1'b0;
                wb_stb_o    =  1'b0;
                wb_cti_o    =  3'b111;  
                wb_bte_o    =  2'b00;   
                wb_we_o     =  1'b0;
                wb_sel_o    =  4'hf;
                wb_adr_o    =  {aw{1'b0}};
                freeze  = 1'b1;
                read_cam    = 1'b0;
                compare     = 1'b0;
                next_state_PTW  = output_state_PTW;
                data_in     = wb_dat_i;
                end
                
        output_state_PTW: begin
                wb_cyc_o	=  1'b0;
                wb_stb_o    =  1'b0;
                wb_cti_o    =  3'b111;  
                wb_bte_o    =  2'b00;   
                wb_we_o     =  1'b0;
                wb_sel_o    =  4'hf;
                wb_adr_o    =  {aw{1'b0}};
                freeze  = 1'b1;
                compare     = 1'b0;
                read_cam    = 1'b1;
                next_state_PTW  = idle;  
                data_in     =  32'b0;
                         
                end       
                
        default:begin
                wb_cyc_o	=  1'b0;
                wb_stb_o    =  1'b0;
                wb_cti_o    =  3'b111;  
                wb_bte_o    =  2'b00;   
                wb_we_o     =  1'b0;
                wb_sel_o    =  4'hf;
                wb_adr_o    =  {aw{1'b0}};
                data_in     =  32'b0;
                freeze  = 1'b0;
                compare     = 1'b0;
                read_cam    = 1'b0;

                end
        endcase

end*/


assign page_fault = (REQ && ACK) & (~data_in[0] | ~data_in[2] & data_in[1]);
assign next_pte   = (REQ && ACK) & (data_in[0] & ~data_in[1] & ~data_in[2] & ~data_in[3]);
assign RO_page  =   (REQ && ACK) & (data_in[1] & ~data_in[2] & ~data_in[3]);
assign RW_page  =   (REQ && ACK) & (data_in[1] & data_in[2] & ~data_in[3]);
assign XO_page  =   (REQ && ACK) & (~data_in[1] & ~data_in[2] & data_in[3]);
assign XOR_page =   (REQ && ACK) & (data_in[1] & ~data_in[2] & data_in[3]);
assign RWX_page =   (REQ && ACK) & (data_in[1] & data_in[2] & data_in[3]);
assign leaf_page = RO_page || RW_page || XO_page || XOR_page || RWX_page;
//assign wb_ack = wb_ack_i & !wb_err_i & !wb_rty_i; 

////////////////////////******** code by piyush ********////////////////////////

    reg compare_delayed;
    wire EVA_en;
    reg re;
    
    always @(posedge clk)
    begin
        compare_delayed = compare;
        re = (~(freeze_tlb)) & (vpn_to_ppn_req | read_cam);
        end

    assign EVA_en = compare & ( !(compare_delayed));

////////////////////////******** addition code by piyush ********////////////////////////

/*
reg valid_trans_off;
wire [19:0] valid_data_trans_off;

//assign TLB_MISS = (tlb_trans_off) ? 1'b0 :  miss;
assign TLB_VALID = (tlb_trans_off) ? valid_trans_off : valid_data;
assign TLB_DATA = (tlb_trans_off) ? {2'b00,valid_data_trans_off,4'b0111} : output_data; 

assign valid_data_trans_off = read_cam? vpn_int : vpn;
always @(posedge clk) begin
    if(rst) valid_trans_off <= 1'b0;
    else if((~(freeze_tlb)) &  ((vpn_to_ppn_req && ~freeze )| read_cam)) valid_trans_off <= 1'b1;
    else valid_trans_off <= 1'b0;
end*/
iregisters cam( .clk(clk), .rst(rst),
    .tlb_trans_off(tlb_trans_off),
    .we(we), .re(/*~tlb_trans_off & */((~(freeze_tlb)) &  ((vpn_to_ppn_req && ~freeze )| read_cam))),  
    .write_addr(write_addr), 
    .write_data(write_data), 
    .vpn(read_cam? vpn_int : vpn), 
    .miss(miss), 
    .valid_data(valid_data),
    .output_data(output_data) , 
    .access_addr(access_addr)
    );
    
ilru tlru( .clk(clk), .rst(rst),
    .access((~(freeze_tlb)) & access),
    .addr_access(access_addr),
    .compare(compare),
    .lru_addr(lru_reg_int)
  
        );
    

   /* replacement_algo i_replacement_algo(.clk(clk), .clk_x2(clk_x2), .rst(rst), .re(re), .we(we), .hit(valid_data), .miss(miss), .access_addr(access_addr),
                                        .EVA_en(EVA_en), .EVA_addr(eva_reg_int)
                                        );*/

    endmodule
