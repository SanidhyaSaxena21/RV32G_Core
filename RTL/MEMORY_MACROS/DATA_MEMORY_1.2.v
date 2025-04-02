//Design: Sanidhya Saxena, Toms JiJi Varghese
//Guide:  Kuruvilla Varghese

`timescale 1ns / 1ps
module DATA_MEMORY #(parameter  ADDR_WIDTH = 18,
                                DATA_WIDTH = 32,
                              INSTR_INPUT_FILE = "Instruction_mem.mif",
                              DATA_INPUT_FILE = "Instruction_mem.mif",
                              PT_INPUT_FILE = "Instruction_mem.mif",
                              HANDLER_INPUT_FILE = "Instruction_mem.mif"
                            ) 
(
  input                     clk,
  input                     rst,
  input                     [31:0] ADDR,
  input                     REQ,
  input                     WRB,
  input   [DATA_WIDTH-1:0]  WDATA,
  output  [DATA_WIDTH-1:0]  RDATA,
  output reg                ACK,
  input   [1:0]             BURST,
  input   [3:0]             BSTROBE,
  output                    STALL
                                       
);


//Memory Hard Macro

wire [ADDR_WIDTH-1:0] ADDR_REDUCED;
reg start;
reg [3:0] counter;
wire instruction_req, data_req, pt_req, clint_req;
wire [31:0] DRDATA, IRDATA, PRDATA,HRDATA;

localparam BURST_LENGTH = 8;

//---------Decoding Logic ------------------------
// Code Region  : 0x0000_0000    <-->    0x0000_3FFF (16KB)
// Data Region  : 0x0000_4000    <-->    0x0000_7FFF (16KB)
// Page Table   : 0x0000_8000    <-->    0x0000_BFFF (16KB)
// CLINT        : 0x0000_C000    <-->    0x0000_FFFF (16KB)
//

assign instruction_req  = ((ADDR[15:14] == 2'b00) && REQ);
assign data_req         = ((ADDR[15:14] == 2'b01) && REQ); 
assign pt_req           = ((ADDR[15:14] == 2'b10) && REQ);
assign clint_req        = ((ADDR[15:14] == 2'b11) && REQ);

assign RDATA            = ~WRB & ((instruction_req) ? IRDATA :
                                  (data_req)        ? DRDATA :
                                  (pt_req)          ? PRDATA :
                                  (clint_req)          ? HRDATA :
                                  32'd0
                                 );
assign ADDR_REDUCED = ADDR[ADDR_WIDTH-1:0];

 MEMORY_MACRO #(.ADDR_WIDTH(ADDR_WIDTH),.DATA_WIDTH(DATA_WIDTH),.INPUT_FILE(DATA_INPUT_FILE)) Data_Memory(
   .clka(clk), // input clka
   .rsta(rst),      // reset
   .byte_en(BSTROBE),
   .ena(data_req && ~STALL), // input ena
   .wea(WRB), // input [3 : 0] wea
   .addra(ADDR & 32'h00003fff), // input [31 : 0] addra
   .dina(WDATA), // input [31 : 0] dina
   .douta(DRDATA) // output [31 : 0] douta
 );
 MEMORY_MACRO #(.ADDR_WIDTH(ADDR_WIDTH),.DATA_WIDTH(DATA_WIDTH),.INPUT_FILE(INSTR_INPUT_FILE)) Instruction_Memory(
   .clka(clk), // input clka
   .rsta(rst),      // reset
   .byte_en(BSTROBE),
   .ena(instruction_req && ~STALL), // input ena
   .wea(WRB), // input [3 : 0] wea
   .addra(ADDR & 32'h00003fff), // input [31 : 0] addra
   .dina(WDATA), // input [31 : 0] dina
   .douta(IRDATA) // output [31 : 0] douta
 );


 MEMORY_MACRO #(.ADDR_WIDTH(ADDR_WIDTH),.DATA_WIDTH(DATA_WIDTH),.INPUT_FILE(PT_INPUT_FILE)) Page_Table_memory(
   .clka(clk), // input clka
   .rsta(rst),      // reset
   .ena(pt_req && ~STALL), // input ena
   .byte_en(BSTROBE),
   .wea(WRB), // input [3 : 0] wea
   .addra(ADDR & 32'h00003fff), // input [31 : 0] addra
   .dina(WDATA), // input [31 : 0] dina
   .douta(PRDATA) // output [31 : 0] douta
 );

 MEMORY_MACRO #(.ADDR_WIDTH(ADDR_WIDTH),.DATA_WIDTH(DATA_WIDTH),.INPUT_FILE(HANDLER_INPUT_FILE)) handler_memory(
   .clka(clk), // input clka
   .rsta(rst),      // reset
   .byte_en(BSTROBE),
   .ena(clint_req && ~STALL), // input ena
   .wea(WRB), // input [3 : 0] wea
   .addra(ADDR & 32'h00003fff), // input [31 : 0] addra
   .dina(WDATA), // input [31 : 0] dina
   .douta(HRDATA) // output [31 : 0] douta
 );
    always @(posedge clk) begin
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
  
  always @(posedge clk) begin
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

  // Stall Logic for Memory, May add any activity signal which says memory busy

  assign STALL = 1'b0;
endmodule
