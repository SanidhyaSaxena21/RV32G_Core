//Designer: Sanidhya Saxena, Toms JiJi Varghese
//Guide   : Kuruvilla Varghese
//Date    : 25/4/25
//
//
//
//
//DESIGN NAME: Debug Controller

`include "defines.v"


module DEBUG_CONTROLLER #(parameter PC_BOOT = 32'h0000_0000)(
  input clk,
  input proc_rst,
  input freeze,

  output debug_mode,

  //Debug Module Interface 
  input   haltreq_i,
  input   resumereq_i,
  input   resethaltreq_i,
  input   ackhavereset,
  input   ackunavail_i,
  input   ndmreset_i,

  output  core_havereset_o,
  output  core_unavail_o,
  output  core_halted_o,
  output  core_running_o,
  output  core_resumeack_o,
  output  core_exist_o,

  //Signals to IF stage for changing PC
  output  pc_valid_o,
  output  [31:0] pc_o,
  output reg instr_req,
  output reg return_pc_valid,
  output reg [31:0] return_pc,

  //Output to CSR Block
  output csr_dbg_wr,
  output [2:0] csr_dbg_cause,
  output [31:0] csr_dbg_dpc,
  output reg clr_step,
  input [31:0] dcsr,
  input [31:0] dpc,
  output reg [31:0] csr_dbg_dscratch0,
  output reg [31:0] csr_dbg_dscratch1,

  //Input from the DM module 
  input [31:0] dm_exception_addr,
  input [31:0] dm_halt_addr,

  //==Probing State======
  output reg [2:0] state,

  //Core Warm reset 
  output core_dbg_reset,


  input new_cmd_flag_i,
  input cmd_not_supported_i,


  input pbuffer_execution,

  //Command Success signals from Abstract Command decoder
  input command_complete_i,

  //Interrupt Disabling
  output reg interrupt_dbg_global_disable,
  input interrupt_pending, //TODO
  //Input PC bus,
  input [95:0] PC_BUS,

  input [5:0] sys,

  //Pipeline control signals
  output reg halt_if,
  output reg halt_id,
  output reg halt_ie,
  output reg halt_mem,
  
  output reg kill_if,
  output reg kill_id,
  output reg kill_ie,
  output reg kill_mem


);

// Debug Entry Causes
localparam DBG_CAUSE_TRIGGER = 2;
localparam DBG_CAUSE_EBREAK = 1;
localparam DBG_CAUSE_NONE = 0;
localparam DBG_CAUSE_HALTREQ = 3;
localparam DBG_CAUSE_RESET_HALTREQ = 5;
localparam DBG_CAUSE_STEP = 4;

// Bits for dcsr register

//localparam EBREAKM = 15;
//localparam STEP = 2;



// When reset is done
//assign core_havereset_o = reset_done;

//If the core is in reset, the core_running_o will be 0
//assign core_running_o = ~proc_rst;

wire async_dbg_allowed,pending_async_dbg;
wire sync_dbg_allowed,pending_sync_dbg;
wire pending_single_step;
wire [2:0] sync_debug_cause;
wire trigger_in_id;

reg [2:0] next_state;

reg pc_dbg_valid;

//These signals will be used If we encounter EBREAK while we are in Debug Mode
reg pc_ebreak_halted;


reg [31:0] pc_dbg, pc_dpc, pc_step;
reg [2:0]  debug_cause_n, debug_cause_q;
reg single_step_halt_if_q, single_step_halt_if_n;
reg debug_mode_n,debug_mode_q;
reg core_warm_reset;
reg [1:0] state_dbg,ns_dbg;

wire [31:0] PC_WB,PC_IF,PC_ID;

reg pbuffer_execution_reg,new_cmd_flag_reg;
wire pbuffer_execution_pulse,new_cmd_flag_pulse;

reg freeze_q;

assign PC_IF = PC_BUS[31:0];
assign PC_ID = PC_BUS[63:32];
assign PC_WB = PC_BUS[95:64];

wire ebreak_in_id;


//assign ebreak_in_id = (sys[`IS_EBREAK]);
assign trigger_in_id = 1'b0;

assign core_unavail_o = 1'b0;
assign core_exist_o   = 1'b1;

//Registering sys signal from Decode to avoid halt combo loops

reg [5:0] sys_q;

always @(posedge clk) begin
  if(proc_rst) begin
    sys_q <= 6'd0;
  end
  else begin
    sys_q <= sys;
  end
end

assign ebreak_in_id = (sys_q[`IS_EBREAK]);

//External debug should be allowed, if the pipeline is currently not freezed
//(i.e No pending Load/Store Instruction being executed or No instruction being fetched from the ICACHE unit)
//No branches should be there in the Execute stage or Decode Stage


//assign async_dbg_allowed = !freeze_q && !Branch_EX_MEM && !Branch_ex_mem;
//So if freeze and haltreq_i went high on same time, then the state will not
//change and in next cycle, allowed will go 0.

always @(posedge clk) begin
  if(proc_rst) begin
    freeze_q <= 1'b0;
  end
  else begin
    freeze_q <= freeze;
  end
end

assign async_dbg_allowed = !freeze_q;

//Sync Debug should be treated just like the Pipeline interrupts
assign sync_dbg_allowed = 1'b1;


//Allow Sync debug to take place if we encounter EBREAK instruction in WriteBack stage and DCSR.EBREAKM == 1'b1 
//or If are in the debug mode and then we encounter EBREAK instruction

wire ebreak_in_wb;
reg [31:0] pc_ebreak_latch;
reg ebreak_id_detected;

//assign pc_ebreak_latch = ebreak_in_id ? PC_ID : pc_ebreak_latch;
//assign ebreak_in_mem = (pc_ebreak_latch == PC_WB);

always @(posedge clk) begin
  if(proc_rst) begin 
    pc_ebreak_latch <= 32'd0;
    ebreak_id_detected <= 1'b0;
  end
  else if(sys[`IS_EBREAK]) begin 
    pc_ebreak_latch <= PC_ID;
    ebreak_id_detected <= 1'b1;
  end
end

assign ebreak_in_wb = (pc_ebreak_latch == PC_WB) && ebreak_id_detected;

assign pending_sync_dbg = ((ebreak_in_wb && dcsr[`EBREAKM] && !debug_mode_q)); // || (ebreak_in_id && debug_mode_q));

//Async debugs are pending if we have the debug request from the Debug Module
//and currently we are not in the debug state. 
//TODO: Check if the ~resumereq_i is required or not. 

assign pending_async_dbg = (haltreq_i /*|| resethaltreq_i*/ & ~resumereq_i) & !debug_mode_q;

// Single stepping should not halt the procoessor, but should execute single
// instruction at a time. It becomes pending when the dcsr.step = 1 and
// processor is not stalled. When asserted, it will halt the IF stage and let
// the other stage complete by pushing NOP in ID stage. When all the
// instructions are done with the execution, then the current waiting
// instruction in the IF stage starts executing one by one.  
assign pending_single_step = (!debug_mode_q && dcsr[`STEP] && ~freeze);

//TODO: SPEC: All the debug request should be held high, until the pipeline allows
//the debug to happen and the pending requests are taken into consideration. 


//Debug Entry:
//We have 2 methods of debug entry, synchronous and Asynchronous
//1. Trigger  (Sync)
//2. Ebreak   (Sync)
//3. haltreq_i  (Async)
//4. resethaltreq_i (Async)
//

//TODO: We need to write the cause of Debug in the CSR with the Flopped version
//of it, when we enter in the debug state. 
assign sync_debug_cause = (trigger_in_id)                                   ? DBG_CAUSE_TRIGGER :
                          (ebreak_in_id && dcsr[`EBREAKM] && !debug_mode_q)  ? DBG_CAUSE_EBREAK:
                          (ebreak_in_id && debug_mode_q)                    ? DBG_CAUSE_EBREAK:
                                                                              DBG_CAUSE_NONE;




//FLopping Resume request
reg resumereq_q;
wire resumereq_pulse;
 
assign resumereq_pulse = resumereq_i & ~resumereq_q;

always @(posedge clk) begin
  if(proc_rst) resumereq_q <= 1'b0;
  else resumereq_q <= resumereq_i;
end



// DEBUG FSM Design 
// State: 
// 2'b00: Functional State: Normal pipeline operations will be going on here, hence no KILL and no FLUSH needs to be there. 
// 2'b01: DBG_RESET_REQ: When the ndmreset_i is asserted, we enter this stage. Here we give the reset to the whole core, and when we come out of the reset, we check for the resethaltreq_i signal. 
// 2'b11: DEBUG_TAKEN: If we get async or sync debug request, we enter in this
// stage. We kill all the instructions in the pipeline and store the most latest PC in the dpc register. PC of the IF stage needs to be loaded with that of the dm_halt_addr, so that we start fetching from that location. 
//
//

//Ndmreset should be treated as normal reset, and after that bit has to be checked. 

reg dbg_csr_save;

assign pc_valid_o = pc_dbg_valid;
assign pc_o = pc_dbg;
assign csr_dbg_wr = dbg_csr_save;
assign csr_dbg_cause = debug_cause_q;
assign csr_dbg_dpc = pc_dpc;
assign core_dbg_reset = core_warm_reset;
assign debug_mode = debug_mode_q;
localparam RESET = 3'b000;
localparam FUNCTIONAL = 3'b001;
localparam DBG_RESET_REQ = 3'b010;
localparam DBG_TAKEN = 3'b011;
localparam DBG_MODE = 3'b100;
localparam EXIT_DEBUG = 3'b101;
localparam PROGRAM_BUFFER = 3'b110;
localparam ABSTRACT_COMMAND = 3'b111;


always @(*) begin
//Default Values

  next_state      = state;
  pc_dbg_valid    = 0;
  pc_dbg          = 32'd0;
  halt_if         = single_step_halt_if_q;
  halt_id         = single_step_halt_if_q;
  halt_ie         = 0;
  halt_mem        = 0;

  kill_if         = 0;
  kill_id         = 0;
  kill_ie         = 0;
  kill_mem        = 0;

  debug_cause_n   = DBG_CAUSE_NONE;
  debug_mode_n    = debug_mode_q;
  dbg_csr_save    = 0;
  clr_step        = 0;

  
  pc_dpc          = PC_BOOT;
  pc_step         = 32'd0;
  
  single_step_halt_if_n = single_step_halt_if_q;
  core_warm_reset = 1'b0;
  
  instr_req       = 1'b1;
  
  interrupt_dbg_global_disable = 1'b0;
  
  return_pc_valid = 1'b0;
  return_pc = 32'd0;
  csr_dbg_dscratch0 = 32'd0;
  csr_dbg_dscratch1 = 32'd0;

  case(state)

    // Reset State: When the processor is in RESET
    RESET: begin
      if(haltreq_i | resethaltreq_i) begin
        next_state = DBG_TAKEN;
        //halt_if = 1'b1;
        //kill_if = 1'b1;
        instr_req = 1'b1;
      end
      else begin
        next_state = FUNCTIONAL;
      end
    end

    //Normal Pipelined operation of the processor
    FUNCTIONAL: begin
      if(ndmreset_i) begin
        next_state = DBG_RESET_REQ;
        halt_id = 1'b1;
        halt_ie = 1'b1;
        halt_mem = 1'b1;
        single_step_halt_if_n = 1'b0;
        //pc_dbg_valid = pc_wb;
        //pc_dbg_valid = 1'b1;
      end
      else if(pending_async_dbg & async_dbg_allowed) begin
        next_state = DBG_TAKEN;
        halt_if = 1'b1;
        halt_id = 1'b1;
        halt_ie = 1'b1;
        halt_mem = 1'b1;
        single_step_halt_if_n = 1'b0;
        debug_cause_n = DBG_CAUSE_HALTREQ;
      end
      else if(pending_sync_dbg & sync_dbg_allowed) begin

        next_state = DBG_TAKEN;
        halt_if = 1'b1;
        halt_id = 1'b1;
        halt_ie = 1'b1;
        halt_mem = 1'b1;
        single_step_halt_if_n = 1'b0;
        debug_cause_n = sync_debug_cause;
      end
      else if(pending_single_step) begin

        // In single step, processor is not halted. 
        // IF Stage needs to be halted to prevent the current instruction going
        // forward
        //next_state = DBG_TAKEN;
        single_step_halt_if_n = 1'b1;
        //halt_if = 1'b1;
        pc_step = PC_IF; // Put the current PC in IF stage in pc_curr. 
        debug_cause_n = DBG_CAUSE_STEP;
        if(step_counter == 2'b11) begin
          next_state = DBG_TAKEN;
        end
        else begin
          next_state = FUNCTIONAL;
        end
      end
    end

    //DBG Reset Request state: When the ndmreset_i is asserted. 
    //TODO: Confirm the timings of ndmreset_i and resethaltreq_i from the DEBUG MODULE
    DBG_RESET_REQ: begin
      core_warm_reset = 1'b1; //Reset the complete processor
      if(ndmreset_i) begin
        next_state = DBG_RESET_REQ;
      end

      //When the ndmreset_i deasserts, check the condition of the 
      else if(resethaltreq_i) begin
        next_state = DBG_TAKEN;
        debug_cause_n = DBG_CAUSE_RESET_HALTREQ;
        halt_if = 1'b1;
        halt_id = 1'b1;
        halt_ie = 1'b1;
        halt_mem = 1'b1;
      end
      else begin
        halt_if = 1'b1;
        halt_id = 1'b1;
        halt_ie = 1'b1;
        halt_mem = 1'b1;
      end

    end


    DBG_TAKEN: begin
      //Indicate the core is halted
      //core_halted_o = 1'b1;

      interrupt_dbg_global_disable = 1'b1;
      //PC needs to be changed to DEBUG PC
      pc_dbg_valid = 1'b1;
      pc_dbg = dm_halt_addr;
      instr_req = 1'b0;
      single_step_halt_if_n = 1'b0;

      //CSR Needs to written
      dbg_csr_save = 1'b1;
      //Store the debug_cause_q to the CSR dcsr register
      //pc_dpc = PC_WB;

      if(debug_cause_q != DBG_CAUSE_STEP) begin
        //halt_if = 1'b1;
        kill_if = 1'b1;
        kill_id = 1'b1;
        kill_ie = 1'b1;
        kill_mem = 1'b1;

        //Make sure that PC signal doesnt get killed in pipeline
        pc_dpc = PC_WB;
      end
      else begin
        halt_if = 1'b1;
        halt_id = 1'b0;
        halt_ie = 1'b0;
        halt_mem = 1'b0;
        pc_dpc = PC_ID;
        clr_step = 1'b1;

        //TODO: Logic for dpc needs to be designed
      end

      debug_mode_n = 1'b1;
      next_state = DBG_MODE;

    end
    DBG_MODE: begin
      interrupt_dbg_global_disable = 1'b1; // All interruts are disable
      //instr_req = 1'b1;
      //This is for executing Abstract commands
      pc_dbg_valid = 1'b1;
      pc_dbg = dm_halt_addr;
      single_step_halt_if_n = 1'b0;

      if(new_cmd_flag_pulse & ~cmd_not_supported_i & ~pbuffer_execution_pulse & ~resumereq_pulse) begin
        halt_if <= 1'b1;
        halt_id = 1'b1;
        halt_ie = 1'b1;
        halt_mem = 1'b1;
        instr_req = 1'b0;
        //pc_dbg_valid = 1'b1;
        //pc_dbg = dm_exception_addr; 
        next_state = ABSTRACT_COMMAND;
      end 
      else if(~new_cmd_flag_pulse & ~pbuffer_execution_pulse & ~resumereq_pulse) begin
        halt_if = 1'b1;
        halt_id = 1'b1;
        halt_ie = 1'b1;
        halt_mem = 1'b1;
        instr_req = 1'b0;
      end
      else if(pbuffer_execution_pulse & ~new_cmd_flag_pulse & ~resumereq_pulse) begin
        next_state = PROGRAM_BUFFER;
        halt_if = 1'b0;
        halt_id = 1'b0;
        halt_ie = 1'b0;
        halt_mem = 1'b0;
        instr_req = 1'b0;
      end
      else if(resumereq_pulse) begin
        next_state = EXIT_DEBUG;
        halt_if = 1'b0;
        instr_req = 1'b1;
        kill_if = 1'b1;
      end

    end
    ABSTRACT_COMMAND: begin
      interrupt_dbg_global_disable = 1'b1;
      pc_dbg_valid = 1'b1;
      pc_dbg = dm_exception_addr;
      halt_if = 1'b1;
      single_step_halt_if_n = 1'b0;
      //instr_req = 1'b0;
      //halt_if = 1'b1;
      /*if(~pc_match_id_imem) begin
        halt_if = 1'b1;
        instr_req = 1'b0;
        next_state = ABSTRACT_COMMAND;
      end
      else if(pc_match_id_imem) begin
        halt_if = 1'b1;
        instr_req = 1'b0;
        next_state = DBG_MODE;
      end*/
      if(command_complete_i) begin
        //halt_if = 1'b1;
        instr_req = 1'b0;
        next_state = DBG_MODE;
      end
      else if(resumereq_pulse) begin
        instr_req = 1'b1;
        debug_mode_n = 1'b0;
        //halt_if = 1'b1;
        next_state = EXIT_DEBUG;
      end
      else begin
        instr_req = 1'b0;
      end
    end
    PROGRAM_BUFFER: begin
      interrupt_dbg_global_disable = 1'b1;
      single_step_halt_if_n = 1'b0;
      // If we encounter EBREAK as terminating instruction in P-BUFFER
      instr_req = 1'b0;
      if(ebreak_in_id) begin
        //halt_if = 1'b1;
        //pc_dbg_valid = 1'b1;
        //pc_dbg = dm_halt_addr;
        kill_if = 1'b1;
        next_state = DBG_MODE;
      end

      //Return Logic
      else if(resumereq_pulse) begin
        //instr_req = 1'b1; //We can allow instructions now to be fetched from ICACHE
        debug_mode_n = 1'b0;
        //halt_if = 1'b1;
        kill_if = 1'b1;
        next_state = EXIT_DEBUG;
      end
      else begin
        halt_if = 1'b0;
      end
      
    end
    EXIT_DEBUG: begin
      return_pc = dpc;
      return_pc_valid = 1'b1;
      single_step_halt_if_n = 1'b0;
      //instr_req = 1'b1;
      interrupt_dbg_global_disable = 1'b0;
      next_state = FUNCTIONAL;
      debug_mode_n = 1'b0;
    end
  endcase

end


always @(posedge clk or posedge proc_rst) begin
  if(proc_rst) begin
    debug_cause_q <= DBG_CAUSE_NONE;
    debug_mode_q  <= 1'b0;
    state         <= RESET;
    single_step_halt_if_q <= 1'b0;
  end
  else begin
    debug_cause_q <= debug_cause_n;
    debug_mode_q  <= debug_mode_n;
    state         <= next_state;
    single_step_halt_if_q <= single_step_halt_if_n;
  end
end 

//For single stepping, we need a counter which will count the number of cycles
reg [1:0] step_counter;
always @(posedge clk) begin
  if(proc_rst) step_counter <= 2'b00;
  else if(state == FUNCTIONAL && pending_single_step && ~freeze) step_counter <= step_counter + 2'd1;
end
//Pulse logic for new_cmd_flag and pbuffer_execution flags


assign pbuffer_execution_pulse = pbuffer_execution & ~pbuffer_execution_reg;
assign new_cmd_flag_pulse = new_cmd_flag_i & ~new_cmd_flag_reg;


always @(posedge clk or posedge proc_rst) begin
  if(proc_rst) begin
    pbuffer_execution_reg <= 1'b0;
    new_cmd_flag_reg <= 1'b0;
  end
  else begin
    pbuffer_execution_reg <= pbuffer_execution;
    new_cmd_flag_reg <= new_cmd_flag_i; 
  end
end


//EBREAK When encountered at the end of program buffer
always @(posedge clk or posedge proc_rst) begin
  if(proc_rst) begin
    pc_ebreak_halted <= 1'b0;
  end
  else if(state == PROGRAM_BUFFER && ebreak_in_id) begin
    pc_ebreak_halted <= 1'b1;
  end
end


//DEBUG HAVERESET FSM
localparam HAVERESET = 2'b00;
localparam RUNNING   = 2'b01;
localparam HALTED    = 2'b11;


always @(posedge clk or posedge proc_rst) begin
  if(proc_rst) begin
    state_dbg <= HAVERESET;
  end
  else begin
    state_dbg <= ns_dbg;
  end
end


always @(*) begin
  ns_dbg = state_dbg;

  case(state_dbg)
    HAVERESET: begin
      if(debug_mode_n || next_state == FUNCTIONAL) begin
        if(debug_mode_n) begin
          ns_dbg = HALTED;
        end
        else begin
          ns_dbg = RUNNING;
        end
      end
    end

    RUNNING: begin
      if(debug_mode_n) begin
         ns_dbg = HALTED;
      end
      else if(ndmreset_i) begin
        ns_dbg = HAVERESET;
      end
    end

    HALTED: begin
      if(!debug_mode_n) begin
        ns_dbg = RUNNING;
      end
    end
    default: begin
      ns_dbg = HAVERESET;
    end
  endcase
end

assign core_havereset_o = (state_dbg == HAVERESET);
assign core_halted_o    = (state_dbg == HALTED);
assign core_running_o   = (state_dbg == RUNNING);

assign core_resumeack_o = (state == EXIT_DEBUG);

endmodule
