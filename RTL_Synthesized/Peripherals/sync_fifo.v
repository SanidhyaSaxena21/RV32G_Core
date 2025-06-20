`timescale 1ns / 1ps
//Designed By: Sanidhya Saxena
//Guide: Kuruvilla Varghese


`define BUF_WIDTH 3    // BUF_SIZE = 16 -> BUF_WIDTH = 4, no. of bits to be used in pointer
`define BUF_SIZE ( 1<<`BUF_WIDTH )

module Sync_FIFO( wr_clk, rd_clk, rst, buf_in, buf_out, wr_en, rd_en, buf_empty, buf_full);

input                 rst, wr_clk, rd_clk, wr_en, rd_en;   
// reset, system clock, write enable and read enable.
input [7:0]           buf_in;                   
// data input to be pushed to buffer
output[7:0]           buf_out;                  
// port to output the data using pop.
output                buf_empty, buf_full;      
// buffer empty and full indication 

wire[7:0]              buf_out;
wire                   buf_empty, buf_full;
reg[`BUF_WIDTH :0]    fifo_counter;
reg[`BUF_WIDTH:0]     rd_ptr, wr_ptr;           // pointer to read and write addresses  
reg[7:0]              buf_mem[`BUF_SIZE -1 : 0]; //  


assign buf_full   = (wr_ptr[`BUF_WIDTH] != rd_ptr[`BUF_WIDTH]) && (wr_ptr[`BUF_WIDTH-1:0] == rd_ptr[`BUF_WIDTH-1:0]);
assign buf_empty  =  wr_ptr[`BUF_WIDTH:0] == rd_ptr[`BUF_WIDTH:0];

//Write Logic
always @(posedge wr_clk)
begin
   if( wr_en && !buf_full )
      buf_mem[wr_ptr[`BUF_WIDTH-1:0]] <= buf_in;		//Writing 8 bit data input to buffer location indicated by write pointer

   else
      buf_mem[wr_ptr[`BUF_WIDTH-1:0]] <= buf_mem[wr_ptr[`BUF_WIDTH-1:0]];
end

always @(posedge wr_clk or posedge rst) begin
  if(rst) begin 
    wr_ptr <= 0;
  end
  else begin
    if(!buf_full && wr_en) wr_ptr <= wr_ptr + 1;
    else wr_ptr <= wr_ptr;
  end
end

// Read Logic

always @(posedge rd_clk or posedge rst) begin
  if(rst) rd_ptr <= 0;
  else begin
    if(!buf_empty && rd_en) rd_ptr <= rd_ptr + 1;
    else rd_ptr <= rd_ptr;
  end
end

assign buf_out = buf_mem[rd_ptr[`BUF_WIDTH-1:0]];

endmodule
