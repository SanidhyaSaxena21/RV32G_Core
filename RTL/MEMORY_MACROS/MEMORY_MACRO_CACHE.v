`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/03/2024 08:04:15 PM
// Design Name: 
// Module Name: MEMORY_MACRO
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


module MEMORY_MACRO_CACHE #(parameter ADDR_WIDTH=32, DATA_WIDTH=32, INIT_FILE = 1, IS_CACHE=1, INIT_VALUE = 'h0000, INPUT_FILE = "Instruction_mem.mif") (
    input   clka,
    input   rsta,
    input   ena,
    input   wea,
    input   [ADDR_WIDTH-1:0] addra,
    input   [DATA_WIDTH-1:0] dina,
    output  [DATA_WIDTH-1:0] douta
    );



    wire  [ADDR_WIDTH-1:0] aligned_addr;
    reg   [DATA_WIDTH-1:0] rdata;
    localparam DEPTH = (1 << ADDR_WIDTH);

    reg [DATA_WIDTH-1:0] MEM_MACRO [DEPTH-1:0];

    assign aligned_addr = (IS_CACHE) ? addra : addra >> 2;

    integer i;
    initial begin
      if(INIT_FILE) $readmemh(INPUT_FILE,MEM_MACRO );
      else begin
        for(i=0;i<DEPTH;i=i+1) begin
          //MEM_MACRO[i] = {DATA_WIDTH{1'b0}};
          MEM_MACRO[i] = INIT_VALUE;
        end
      end
    end

    always @(posedge clka) begin
      if(ena) begin
        if(wea) MEM_MACRO[aligned_addr] <= dina ;
      end
    end

    always @(posedge clka or posedge rsta) begin
      if(rsta) rdata <= {DATA_WIDTH{1'b0}};
      else if(ena) begin
        if(wea) rdata <= dina;
        else rdata <= MEM_MACRO[aligned_addr];
      end
      else begin
        rdata <= {DATA_WIDTH{1'b0}};
      end
    end

    //assign douta = ((wea) ? MEM_MACRO[aligned_addr] : rdata) & {DATA_WIDTH{ena}};
    assign douta = rdata;
endmodule
