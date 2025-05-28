//Designer: Sanidhya Saxena
//
//
//---STUB for Debug Module for Testing purpose

`include "defines.v"
(* keep_hierarchy = "yes" *)
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

  output valid_abs_cmd,
  output abs_cmd_ack,
  output [31:0] data0_o,

  input [31:0] data1_i,
  input regRW_done,
  input memDone,

  output reg [31:0] arg0_o,
  output reg [31:0] arg1_o,
  input [31:0] arg1_i,

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
wire [31:0] ABSTRACT_COMMAND_encoder;
reg [31:0] DATA0,DATA1;
reg abs_cmd_ack_q;
wire [7:0] cmdtype;
wire transfer,write;
wire [15:0] regno;
wire [31:0] write_data;

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


 reg valid_abs_cmd_q;


 assign haltreq_i = 1'b0;
 assign resumereq_i = 1'b0;
 assign resethaltreq_i = 1'b0;

 assign ackhavereset = 1'b0;
 assign ackunavail = 1'b0;

 assign ndmreset = 1'b0;

 wire [31:0] MEM_ADDR;

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
    if(core_halted_o & /*(delay_counter == 3'd6) &*/ ~core_running_o & ~core_resumeack_o & ~command_complete_o) new_cmd_flag <= 1'b1;
    else if(core_resumeack_o || core_running_o || command_complete_o ) new_cmd_flag <= 1'b0;
  end
end


//Abstract Command Handlshaking

//Abstract Register emulation as register

//DATA0: Transmit
//DATA1: Receive 


assign abs_cmd_ack = abs_cmd_ack_q;

always @(posedge clk or posedge reset) begin
  if(reset) begin
    DATA1 <= 32'd0;
  end
  else if(regRW_done | memDone) begin
    DATA1 <= regRW_done ? data1_i : 
             memDone    ? arg1_i : 32'd0 ; 
  end
end


always @(posedge clk or posedge reset) begin
  if(reset) abs_cmd_ack_q <= 1'b0;
  else if((regRW_done | memDone) & ~abs_cmd_ack_q) abs_cmd_ack_q <= 1'b1;
  else abs_cmd_ack_q <= 1'b0;
end

//Abstract command encoder

assign cmdtype   = 8'd2;//0-RegRead/Write, 1-CSR, 2-Memory
assign transfer  = 1'b1;
assign write     = 1'b0;
assign regno     = 16'd17;
assign write_data = 32'd45;

assign ABSTRACT_COMMAND_encoder = {cmdtype,6'd0,transfer,write,regno};
assign valid_abs_cmd  = valid_abs_cmd_q;
assign data0_o        = DATA0;

assign MEM_ADDR = 32'h20000000;

always @(posedge clk or posedge reset) begin
  if(reset) begin 
    ABSTRACT_COMMAND <= 32'd0;
    valid_abs_cmd_q  <= 1'b0;
    DATA0            <= 32'd0;
    arg0_o           <= 32'd0;
    arg1_o           <= 32'd0;    
  end
  else if(abstract_command_req & ~regRW_done & ~memDone) begin
    //ABSTRACT_COMMAND <= 32'h00a38393;
    ABSTRACT_COMMAND <= ABSTRACT_COMMAND_encoder;
    valid_abs_cmd_q  <= 1'b1;
    DATA0            <= write_data;
    arg0_o           <= MEM_ADDR;
    arg1_o           <= write_data;
  end
  else if(regRW_done | memDone) begin
    ABSTRACT_COMMAND <= 32'd0;
    valid_abs_cmd_q <= 1'b0;
    DATA0           <= 32'd0;
    arg0_o          <= 32'd0;
    arg1_o          <= 32'd0;
  end
end



endmodule

