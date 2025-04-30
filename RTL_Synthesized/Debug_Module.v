//Designer: Sanidhya Saxena
//
//
//---STUB for Debug Module for Testing purpose

`include "defines.v"

module Debug_Module (
  input clk,
  input reset,

  //Debug Module Interface 
  output   haltreq_i,
  output   resumereq_i,
  output   resethaltreq_i,

  input  core_havereset_o,
  output   ackhavereset,

  //TODO: How to use them
  input  core_unavail_o,
  output   ackunavail,

  input  core_halted_o,
  input  core_running_o,
  input  core_resumeack_o,

  //Address for Debugger
  output [31:0] dm_exception_addr,
  output [31:0] dm_halt_addr,

  output ndmreset,

  output new_cmd_flag_i,
  output cmd_not_supported_i,
  output pbuffer_execution,
  

  //Command Success signals to Debug Module
  input command_complete_o,
  input reg_access_complete_o,
  input mem_access_complete_o,
  
  //Command Failure Signals to Debug Module
  input abstract_cmd_fail_o,
  input abstract_cmd_wrong_hart_state_o,
  input abstract_cmd_exception_o,
  input abstract_cmd_bus_error,


  input [31:0] PC_DEBUG,
  output [31:0] INSTRUCTION_DEBUG,
  input freeze_int_dcache
);

reg [31:0] PC_DEBUG_Q;
wire [31:0] PC_REQUEST;

wire pbuffer_req;
wire [31:0] PBUFFER_INSTRUCTION;

wire abstract_command_req;
reg [31:0] ABSTRACT_COMMAND;

always @(posedge clk or posedge reset) begin
  if(reset) PC_DEBUG_Q <= 32'd0;
  else if(~freeze_int_dcache) PC_DEBUG_Q <= PC_DEBUG;
end

//Selection between PROGRAM BUFFER and ABSTRACT COMMAND

assign INSTRUCTION_DEBUG = (pbuffer_req) ?  PBUFFER_INSTRUCTION : 
                           (abstract_command_req) ? ABSTRACT_COMMAND :
                           32'd0;
//Address of Debugger: 32'h5000_0000 <--> 32'h5000_03ff

assign pbuffer_req = (PC_REQUEST[31:24] == 8'b0101_0000);
assign abstract_command_req = (PC_REQUEST[31:24] == 8'b0101_0001);

assign PC_REQUEST = freeze_int_dcache ? PC_DEBUG_Q : PC_DEBUG;
assign dm_halt_addr = 32'h50000000; //Program Buffer Address 
assign dm_exception_addr = 32'h51000000;  //Abstract Register address


//Program Buffer Emulation as Memory

 MEMORY_MACRO #(.ADDR_WIDTH(10),.DATA_WIDTH(32),.INPUT_FILE("debug.mem")) debug_memory(
   .clka(clk), // input clka
   .rsta(reset),      // reset
   .byte_en(4'b1111),
   .ena(pbuffer_req), // input ena
   .wea(1'b0), // input [3 : 0] wea
   .addra(PC_REQUEST & 32'h000003ff), // input [31 : 0] addra
   .dina(32'd0), // input [31 : 0] dina
   .douta(PBUFFER_INSTRUCTION) // output [31 : 0] douta
 );


//Abstract Register emulation as register

always @(posedge clk or posedge reset) begin
  if(reset) ABSTRACT_COMMAND <= 32'd0;
  else if(abstract_command_req) begin
    ABSTRACT_COMMAND <= 32'h00a38393;
  end
end


 assign haltreq_i = 1'b0;
 assign resumereq_i = 1'b0;
 assign resethaltreq_i = 1'b0;

 assign ackhavereset = 1'b0;
 assign ackunavail = 1'b0;

 assign ndmreset = 1'b0;


 //Logic: 
 //When we go into the halted state, the processor is halted and wont execute the instructions from Abstract Reg or Program buffer until the new_cmd_flag_i = 1, so as to accomodate with the slow nature of JTAG and Debugger.
 
reg [2:0] delay_counter;
reg new_cmd_flag;
reg new_pbuff_flag;

assign new_cmd_flag_i = new_cmd_flag;
//assign new_cmd_flag_i = 1'b0;
//assign pbuffer_execution = new_pbuff_flag;
assign pbuffer_execution = 1'b0;
assign cmd_not_supported_i = 1'b0;


always @(posedge clk or posedge reset) begin
  if(reset) delay_counter <= 3'd0;
  else if(core_halted_o) delay_counter <= delay_counter + 3'd1; //Start the counter only when core_halted_o is high
  else if(command_complete_o) delay_counter <= 3'd0;
end

//Program Buffer flag
always @(posedge clk or posedge reset) begin
  if(reset) new_pbuff_flag <= 1'b0;
  else begin
    if(core_halted_o && (delay_counter == 3'd6) & ~core_running_o & ~core_resumeack_o) new_pbuff_flag <= 1'b1;
    else if(core_resumeack_o || core_running_o) new_pbuff_flag <= 1'b0;
  end
end

//Abstract Command flag
always @(posedge clk or posedge reset) begin
  if(reset) new_cmd_flag <= 1'b0;
  else begin
    if(core_halted_o && (delay_counter == 3'd6) & ~core_running_o & ~core_resumeack_o) new_cmd_flag <= 1'b1;
    else if(core_resumeack_o || core_running_o || command_complete_o ) new_cmd_flag <= 1'b0;
  end
end


endmodule

