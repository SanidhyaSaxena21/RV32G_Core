`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/16/2025 07:11:31 PM
// Design Name: 
// Module Name: tb_instruction_memory
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


module tb_instruction_memory(

    );

      reg clk_int,clk_x2, rst;
      //reg i_acc;
      reg [31:0]  i_addr;
      reg freeze_in;
      reg stall_load;
      
      wire  [31:0]  ADDR;
      wire  [1:0]   BURST; //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
      wire          REQ;
      wire          WRB;
      wire  [31:0]  WDATA;
      wire   [31:0]  RDATA;
      reg           ACK;
      reg           STALL;
      wire  [3:0]   BSTROBE;

      reg [31:0] csr_satp;
     
      wire [31:0] instr_out;
      wire instruction_page_fault;
      wire stall_out;

      reg tlb_trans_off;
      reg vpn_to_ppn_req_in;
      localparam BURST_LENGTH = 8;
     
  mem_hier IMEM_CACHE(

      .clk(clk_int),
      .clk_x2(clk_x2), 
      .rst_n(rst),
      .i_addr(i_addr),
      .freeze_in(freeze_in),
      .stall_load(stall_load),
      
      .ADDR(ADDR),
      .BURST(BURST), //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
      .REQ(REQ),
      .WRB(WRB),
      .WDATA(WDATA),
      .RDATA(RDATA),
      .ACK(ACK),
      .STALL(STALL),
      .BSTROBE(BSTROBE),

      .csr_satp(csr_satp),
     
      .instr_out(instr_out),
      .instruction_page_fault(instruction_page_fault),
      .stall_out(stall_out),

      .tlb_trans_off(tlb_trans_off),
      .vpn_to_ppn_req_in(vpn_to_ppn_req_in)
);

wire instruction_req;
reg start;
reg [3:0] counter;

assign instruction_req  = ((ADDR[15:14] == 2'b00) && REQ);

MEMORY_MACRO  #(.ADDR_WIDTH(32),.DATA_WIDTH(32),.INPUT_FILE("IMEM_Greatest_2.mem")) Instruction_memory(
  .clka(clk_int),
  .rsta(rst),
  .ena(instruction_req),
  .wea(WRB),
  .addra(ADDR),
  .dina(WDATA),
  .douta(RDATA)
);

    always @(posedge clk_int) begin
    if(rst) begin
      counter <= 4'b0000;
      start <= 1'b0;
    end
    else begin
      if((REQ | start) && (counter < BURST_LENGTH) && ~STALL && (BURST == 2'b01)) begin // TODO: Need to Add ACK or not ?
        start <= 1'b1;
        counter <= counter + 1;
      end
      else if(start && counter == BURST_LENGTH && ~STALL /*&& (BURST == 2'b01)*/) begin
        counter <= 4'd0;
        start <= 1'b0;
      end
    end
  end
  
  always @(posedge clk_int) begin
    if(rst) begin
        ACK <= 1'b0;
        //data_out <= 32'd0;
    end
    else if(REQ && counter < BURST_LENGTH && ~STALL && (BURST == 2'b01)) begin
        ACK <= 1'b1;
        //data_out <= RDATA;
    end
    else if (ACK && REQ && (BURST == 2'b00)) begin
        ACK <= 1'b0;
    end   
    else if(REQ && (BURST == 2'b00)) begin
        ACK <= 1'b1;
    end
    else if(counter == BURST_LENGTH /*&& (BURST == 2'b01)*/) begin
        ACK <= 1'b0;
        //data_out <= 32'd0;
    end
    else begin
        ACK <= ACK;
        //data_out <= data_out;
    end
  end
  
  // Free running Clock
  initial begin
  clk_int = 0; 
  clk_x2 = 0;
  end
  always #10 clk_int <= ~clk_int;

  always #5 clk_x2 <= ~clk_x2;
  
  // Stimulus    
  initial begin
    STALL = 1'b0; csr_satp = 32'd0; freeze_in = 1'b1;
    stall_load = 1'b0; tlb_trans_off = 1'b1;i_addr=32'd0;
    vpn_to_ppn_req_in = 1'b0;
    rst = 1'b1;
    
    #150 rst = 1'b0;
    #100
    freeze_in = 1'b0;
    
  end
  
endmodule
