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


module MEMORY_MACRO #(parameter ADDR_WIDTH=32, DATA_WIDTH=32, INPUT_FILE = "Instruction_mem.mif") (
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

    reg [DATA_WIDTH-1:0] MEM_MACRO [2048-1:0];

    assign aligned_addr = addra >> 2;

    initial begin
      $readmemh(INPUT_FILE,MEM_MACRO );
    end

    always @(posedge clka or posedge rsta) begin
      if(rsta) begin
        rdata <= {DATA_WIDTH{1'b0}};
      end
      else if(ena) begin
        if(wea)
          MEM_MACRO[aligned_addr] <= dina ;
        else
          rdata <= MEM_MACRO[aligned_addr];
      end
      else begin
        rdata <= {DATA_WIDTH{1'b0}};
      end
    end


    assign douta = rdata;
endmodule
