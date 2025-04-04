`timescale 1ns / 1ps
/////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Sanidhya Saxena, Toms Jiji Varghese
// 
// Create Date: 
// Design Name: 
// Module Name: DTLB
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


module DTLB #(
              parameter tag_width=22,
              parameter virtual_width=32,
              parameter vpn_width=20,
              parameter offset=12,
              
              parameter dw = 32,
              parameter aw = 32,
              parameter bl = 8
)  
(
  input                       clk, rst, clk_x2,
  input [vpn_width-1:0]       vpn_in_port1, //virtual page number
  input [vpn_width-1:0]       vpn_in_port2, //virtual page number
  input                       vpn_to_ppn_req_port1,
  input                       vpn_to_ppn_req_port2,
  input                       freeze_tlb,

  input dtlb_trans_off,
  input [31:0] csr_satp,
  output page_fault,
  
  output [(tag_width-1+4):0]  tag_out_port1, //Tag nothing but PPN
  output [(tag_width-1+4):0]  tag_out_port2, //Tag nothing but PPN
  output reg                  freeze,
  //output stall_out,
  output                      tag_hit_port1,
  output                      tag_hit_port2,
  

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
  

);

//change the following according to the design//////
`define VPN_MASK 'hFFFFF000



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
//wire [4:0] eva_reg_int;
wire [vpn_width-1:0] vpn;
reg [vpn_width-1:0] vpn_int;


wire TLB_VALID;
wire [25:0] TLB_DATA;
//--------Reg definitions-----------------------

reg  compare;
reg  read_cam;
reg [31:0] data_in;
reg [4:0] lru_reg;
//reg [4:0] eva_reg;
reg [1:0] state;
reg [1:0] next_state;
reg access;
//reg re;

//---------PAGE TABLE WALKER FSM----------------
parameter IDLE          = 3'd0;
parameter FIRST_LEVEL   = 3'd1;
parameter SECOND_LEVEL  = 3'd2;
parameter REPLACE       = 3'd3;
parameter OUTPUT_STATE  = 3'd4;

parameter BASE_ADDR = 32'd48;



wire vpn_to_ppn_req;
wire [vpn_width-1:0] vpn_in;
wire [(tag_width-1+4):0] tag_out;
wire tag_hit;
reg port_decision_flag;

reg [3:0] state_PTW,next_state_PTW;
reg [31:0] data_in_L1,data_in_L2;
//wire page_fault;
wire next_pte;
reg [31:0] next_addr, addr_pte;
wire  leaf_page;

// --  two port to 1 CAM read request -- //
assign vpn_to_ppn_req = vpn_to_ppn_req_port1 | vpn_to_ppn_req_port2;
assign vpn_in = vpn_to_ppn_req_port1 ? vpn_in_port1 : vpn_in_port2;
assign tag_out_port1 = port_decision_flag ? tag_out : 26'b0;
assign tag_out_port2 = (~port_decision_flag) ? tag_out : 26'b0;
assign tag_hit_port1 = port_decision_flag ? tag_hit : 1'b0;
assign tag_hit_port2 = (~port_decision_flag) ? tag_hit : 1'b0; 
assign WDATA = 32'b0;

assign tag_out = output_data;
assign tag_hit = valid_data;
assign we = leaf_page && ACK & REQ;
assign write_data = { {vpn_int}, {data_in} };
assign write_addr = lru_reg;
//assign write_addr = eva_reg;
assign vpn = vpn_in;


always @(posedge clk)
begin
        if(rst) begin
//            re <= 1'b0;
            vpn_int <= 20'b0;
            port_decision_flag <= 1'b0;
        end
        else if(vpn_to_ppn_req & (state_PTW == IDLE) & (~miss)) begin
//            re <= 1'b1;
            vpn_int <= vpn_in;
            port_decision_flag <= vpn_to_ppn_req_port1;

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


always @(posedge clk) begin
  if(rst) state_PTW <= IDLE;
  else if(~freeze_tlb)  state_PTW <= next_state_PTW;
end 

always @(posedge clk) begin
  if(rst) addr_pte <= {csr_satp[19:0],12'd0};
  else if(~freeze_tlb)  addr_pte <= next_addr;
end 

/*always @(posedge clk) begin
  if(rst) eva_reg <= 5'd0;
  else if(~freeze_tlb) eva_reg <= eva_reg_int;
end*/


always @(posedge clk) begin
  if(rst) lru_reg <= 5'd0;
  else if(~freeze_tlb) lru_reg <= lru_reg_int;
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

// data_in_L1[31:0] = 
assign page_fault = (REQ && ACK) & (~data_in[0] | ~data_in[2] & data_in[1]);
assign next_pte   = (REQ && ACK) & (data_in[0] & ~data_in[1] & ~data_in[2] & ~data_in[3]);
assign RO_page  =   (REQ && ACK) & (data_in[1] & ~data_in[2] & ~data_in[3]);
assign RW_page  =   (REQ && ACK) & (data_in[1] & data_in[2] & ~data_in[3]);
assign XO_page  =   (REQ && ACK) & (~data_in[1] & ~data_in[2] & data_in[3]);
assign XOR_page =   (REQ && ACK) & (data_in[1] & ~data_in[2] & data_in[3]);
assign RWX_page =   (REQ && ACK) & (data_in[1] & data_in[2] & data_in[3]);
assign leaf_page = RO_page || RW_page || XO_page || XOR_page || RWX_page;

//assign wb_ack = wb_ack_i & !wb_err_i & !wb_rty_i; 

//Output signals
//always @(posedge clk)
//begin
//    if(rst) begin
//         wb_cyc_o	<=  1'b0;
//         wb_stb_o    <=  1'b0;
//         wb_cti_o    <=  3'b111;  
//         wb_bte_o    <=  2'b00;   
//         wb_we_o     <=  1'b0;
//         wb_sel_o    <=  4'hf;
//         wb_adr_o    <=  {aw{1'b0}};
//         data_in     <=  32'b0;
//    end
//    else
//    begin
//         wb_adr_o    <=  vpn;  //assuming vpn will be constant
//         wb_we_o     <=  1'b0; //Indicates read
//         wb_sel_o    <=  4'hf; //Word

//         if(wb_ack) begin
//            data_in  <=  wb_dat_i;           
//            wb_cyc_o    <=  1'b0;
//            wb_stb_o    <=  1'b0;
//         end
//         else begin
//            wb_cyc_o    <=  1'b1;
//            wb_stb_o    <=  1'b1;
//            data_in     <=  32'b0;
//         end
    
//    end
   
    

//end


////////////////////////******** code by piyush ********////////////////////////

   /* reg compare_delayed;
    wire EVA_en;
    reg re;
    
    always @(posedge clk)
    begin
        compare_delayed = compare;
        re = (~(freeze_tlb)) & (vpn_to_ppn_req | read_cam);
        end

    assign EVA_en = compare & ( !(compare_delayed));*/

////////////////////////******** addition code by piyush ********////////////////////////

/*reg tlb_valid_off;
reg [25:0] tlb_data_off;
assign TLB_VALID = (dtlb_trans_off) ? tlb_valid_off : valid_data;


always @(posedge clk or posedge rst) begin
    if(rst) tlb_valid_off <= 1'b0;
    else if(dtlb_trans_off & vpn_to_ppn_req) begin
        tlb_valid_off <= 1'b1;
    end
    else tlb_valid_off <= 1'b0;
end

always @(posedge clk or posedge rst) begin
    if(rst) tlb_data_off <= 26'd0;
    else if(dtlb_trans_off & vpn_to_ppn_req) begin
        tlb_data_off <= {2'b00,vpn,4'b0111};
    end
    else tlb_data_off <= 26'd0;
end
assign TLB_DATA = (dtlb_trans_off) ? tlb_data_off : output_data; */

Registers cam( .clk(clk), .rst(rst),.dtlb_trans_off(dtlb_trans_off),
    .we(we), .re(/*~dtlb_trans_off & */((~(freeze_tlb)) & (vpn_to_ppn_req | read_cam))),  
    .write_addr(write_addr), 
    .write_data(write_data), 
    .vpn(read_cam? vpn_int : vpn), 
    .miss(miss), 
    .valid_data(valid_data),
    .output_data(output_data) , 
    .access_addr(access_addr)
    );
    
lru tlru( .clk(clk), .rst(rst),
    .access((~(freeze_tlb)) & access),
    .addr_access(access_addr),
    .compare(compare),
    .lru_addr(lru_reg_int)
  
        );

    /*replacement_algo d_replacement_algo(.clk(clk), .clk_x2(clk_x2), .rst(rst), .re(re), .we(we), .hit(valid_data), .miss(miss), .access_addr(access_addr),
                                        .EVA_en(EVA_en), .EVA_addr(eva_reg_int)
                                        );*/
    
    endmodule
