`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/12/2025 05:28:52 PM
// Design Name: 
// Module Name: EXCEPTION_UNIT
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

`include "/home/rclab/FINAL_PROJECT/MtechESE_RISCV_2024-25/Working_Project_New/processor_1.2/processor.srcs/sources_1/imports/processor/rtl/defines.v"
//`include "defines.v"
module EXCEPTION_UNIT #(parameter XLEN = 32,
                        parameter SB_LENGTH = 4*(XLEN+1)+1)
   (
    input CLK,
    input RST,
    output reg PC_Control__IRQ,
    output PC_Freeze_IRQ,
    output ID_IE_FLUSH,
    output IF_ID_nop,
    output reg PC_HALT_BREAK,

    input [6-1:0] sys,
    input IF_ID_freeze,

    // Exception PC and Instruction
    input [3*XLEN-1:0]  PC_EXCEPTION_BUS,
    input [3*XLEN-1:0]  INSTRUCTION_EXCEPTION_BUS,
    input [XLEN-1:0]    MEM_ADDR_EXCEPTION,
    input [63:0] sync_exceptions,
    // CSR Registers 
    input [`XLEN-1:0] csr_mstatus,
    input [`XLEN-1:0] csr_mip,
    input [`XLEN-1:0] csr_mie,
    input [`XLEN-1:0] csr_mtvec,

    input Branch_Taken__ex_mem,
    input Branch_Taken__EX_MEM,
    input [XLEN-1:0] Branch_Target_Addr__ex_mem, 
    input [XLEN-1:0] Branch_Target_Addr__EX_MEM, 
    //CSR Interface
    input             csr_ready,
    input             csr_ro_wr,

    output reg pc_ctrl_ecall,
    output reg pc_ctrl_mret,
    output reg pc_ctrl_ebreak,
    output reg [`XLEN-1:0]  mepc,
    output reg              mepc_wr,
    output reg [`XLEN-1:0]  mstatus,
    output reg              mstatus_wr,
    output reg [`XLEN-1:0]  mcause,
    output reg              mcause_wr,
    output reg [`XLEN-1:0]  mtval,
    output reg              mtval_wr,
    output [`XLEN-1:0]                  mtvec_addr,
    //output reg [63:0]       instret,
    output reg              clr_meip,
    output reg              interrupt_pending,
    output [SB_LENGTH-1:0]  sb_csr
    );


    

    assign sb_csr = { clr_meip,
                      mtval_wr,mtval,
                      mcause_wr,mcause,
                      mstatus_wr,mstatus,
                      mepc_wr,mepc};

    wire csr_mstatus_mie ;
    wire csr_mip_msip; 
    wire csr_mie_msie    ;
    wire csr_mip_mtip    ;
    wire csr_mie_mtie    ;
    wire csr_mip_meip    ;
    wire csr_mie_meie    ;
    wire [`XLEN-1:0] mcause_code;
    reg [`XLEN-1:0] mcause_code_int;
    wire [`XLEN-1:0] mtval_info;
    reg [`XLEN-1:0] mtval_info_int;

    wire [`XLEN-1:0] exp_addr;
    wire [`XLEN-1:0] exp_instruction;
    wire [`XLEN-1:0] exp_pc;
    wire [`XLEN-1:0] pc_reg_IF;
    wire [`XLEN-1:0] pc_reg_ID;
    
    wire [`XLEN-1:0] mstatus_for_trap;
    wire [`XLEN-1:0] mstatus_for_mret;

    wire [1:0] priv_mode;
    wire inst_addr_misaligned; 
    wire inst_access_fault;    
    wire illegal_instruction;  
    //wire csr_ro_wr;            
    //wire inst_addr_misaligned; 
    wire load_access_fault;    
    wire store_access_fault;   
    wire load_misaligned;      
    wire store_misaligned;
    wire instruction_page_fault;
    wire load_page_fault;    
    wire inst_dec_error;       

    wire async_trap_occuring;
    wire sync_trap_occuring;
    wire trap_occuring;

    reg [XLEN-1:0] pc_sync_interrupt;
    reg [1:0] count;


    wire MEM_STAGE_EXCEPTION, DECODE_STAGE_EXCEPTION, ECALL_EBREAK_EXCEPTION, FETCH_STAGE_EXCEPTION ;
    wire SOFTWARE_INTERRUPT, TIMER_INTERRUPT, EXTERNAL_INTERRUPT;
    
    reg exp_trap, exp_ecall;
    reg PC_Control__IRQ_int;
    wire PC_Control__IRQ_pulse;
    
    reg [31:0] pc_latch;
    reg pc_latch_cache_stall;
    reg async_trap_occuring_int;
    reg MEM_STAGE_EXCEPTION_INT;
    reg DECODE_STAGE_EXCEPTION_INT;
    reg async_int;
    wire async_pulse;
    
    reg Branch_Taken__EX_MEM_reg, Branch_Taken__ex_mem_reg;
    reg [XLEN-1:0] Branch_Target_Addr__EX_MEM_reg, Branch_Target_Addr__ex_mem_reg;

    assign priv_mode = 2'b00; // Only Machine mode supported

    assign csr_mstatus_mie  = csr_mstatus[`MSTATUS_MIE];
    assign csr_mip_msip     = csr_mip[`MIP_MSIP]; 
    assign csr_mie_msie     = csr_mie[`MIE_MSIE];
    assign csr_mip_mtip     = csr_mip[`MIP_MTIP];
    assign csr_mie_mtie     = csr_mie[`MIE_MTIE];
    assign csr_mip_meip     = csr_mip[`MIP_MEIP];
    assign csr_mie_meie     = csr_mie[`MIE_MEIE];

    assign inst_addr_misaligned   = sync_exceptions[0];
    assign inst_access_fault      = sync_exceptions[1];
    assign illegal_instruction    = sync_exceptions[2];
    assign load_misaligned        = sync_exceptions[4];
    assign load_access_fault      = sync_exceptions[5];
    assign store_misaligned       = sync_exceptions[6];
    assign store_access_fault     = sync_exceptions[7];
    assign instruction_page_fault = sync_exceptions[12];
    assign load_page_fault        = sync_exceptions[13];
    assign inst_dec_error         = sync_exceptions[24];

    // Synchronous Interrupts
    assign MEM_STAGE_EXCEPTION      = (load_access_fault | load_misaligned | store_access_fault | store_misaligned | load_page_fault);
    assign DECODE_STAGE_EXCEPTION   = (inst_access_fault | illegal_instruction | csr_ro_wr | inst_dec_error);
    assign ECALL_EBREAK_EXCEPTION   = (sys[`IS_ECALL] | sys[`IS_EBREAK] | sys[`IS_MRET]);
    assign FETCH_STAGE_EXCEPTION    = (inst_addr_misaligned | instruction_page_fault);

    // Asynchronous Interrupts 
    assign SOFTWARE_INTERRUPT       = (csr_mip_msip & csr_mie_msie); 
    assign TIMER_INTERRUPT       = (csr_mip_mtip & csr_mie_mtie); 
    assign EXTERNAL_INTERRUPT       = (csr_mip_meip & csr_mie_meie); 


    assign exp_addr =         (MEM_STAGE_EXCEPTION)     ? MEM_ADDR_EXCEPTION                        : {XLEN{1'b0}};
    assign exp_instruction =  (DECODE_STAGE_EXCEPTION)  ? INSTRUCTION_EXCEPTION_BUS[2*XLEN-1:XLEN]  : {XLEN{1'b0}};
    assign pc_reg_IF  =       (FETCH_STAGE_EXCEPTION)   ? PC_EXCEPTION_BUS[XLEN-1:0]                : {XLEN{1'b0}};
    assign pc_reg_ID  =       (ECALL_EBREAK_EXCEPTION)  ? PC_EXCEPTION_BUS[2*XLEN-1:XLEN]           : {XLEN{1'b0}};

   /* assign pc_sync_interrupt = ((count == 2'd2) && Branch_Taken__ex_mem && ~IF_ID_freeze)  ? Branch_Target_Addr__ex_mem :
                                (count == 2'd2 && ~IF_ID_freeze)                           ? PC_EXCEPTION_BUS[XLEN-1:0] : 
                                (async_pulse && IF_ID_freeze)                              ? pc_latch : 32'd0;*/
    always @(*) begin
      if((count == 2'd2) && ~Branch_Taken__EX_MEM && Branch_Taken__EX_MEM_reg && ~IF_ID_freeze) pc_sync_interrupt = Branch_Target_Addr__EX_MEM_reg;
      else if((count == 2'd2) && ~Branch_Taken__ex_mem && Branch_Taken__ex_mem_reg && ~IF_ID_freeze) pc_sync_interrupt = Branch_Target_Addr__ex_mem_reg;
      else if((count == 2'd2) && Branch_Taken__ex_mem && ~IF_ID_freeze) pc_sync_interrupt = Branch_Target_Addr__ex_mem;
      else if(count == 2'd2 && ~IF_ID_freeze) pc_sync_interrupt = pc_latch;
      //else if(PC_Control__IRQ_pulse && IF_ID_freeze) pc_sync_interrupt = pc_latch;
    end

    always @(posedge CLK or posedge RST) begin
      if(RST) begin
        Branch_Taken__EX_MEM_reg <= 1'b0;
        Branch_Taken__ex_mem_reg <= 1'b0;
        Branch_Target_Addr__EX_MEM_reg <= 32'd0;
        Branch_Target_Addr__ex_mem_reg <= 32'd0;
      end
      else if(~PC_Control__IRQ) begin
        Branch_Taken__EX_MEM_reg <= Branch_Taken__EX_MEM;
        Branch_Taken__ex_mem_reg <= Branch_Taken__ex_mem;
        Branch_Target_Addr__EX_MEM_reg <= Branch_Target_Addr__EX_MEM;
        Branch_Target_Addr__ex_mem_reg <= Branch_Target_Addr__ex_mem;
      end
    end
    /*assign exp_pc = (MEM_STAGE_EXCEPTION)                                                               ? PC_EXCEPTION_BUS[3*XLEN-1:2*XLEN] :
                    (DECODE_STAGE_EXCEPTION)                                                            ? PC_EXCEPTION_BUS[XLEN-1:0]   :
                    (ECALL_EBREAK_EXCEPTION)                                                            ? PC_EXCEPTION_BUS[XLEN-1:0]        :
                    (FETCH_STAGE_EXCEPTION)                                                             ? PC_EXCEPTION_BUS[XLEN-1:0]  :
                    (SOFTWARE_INTERRUPT | TIMER_INTERRUPT | EXTERNAL_INTERRUPT)                         ? pc_sync_interrupt: 
                    32'd0;*/
    
    
   
   // PC Latching if async interrupts occur during the Cache fetch operations



   // Sample the Async interrupt for checking which PC to store in MEPC
    always  @(posedge CLK or posedge RST) begin
      if(RST) begin
        async_trap_occuring_int <= 1'b0;
        DECODE_STAGE_EXCEPTION_INT <= 1'b0;
        MEM_STAGE_EXCEPTION_INT <= 1'b0;
      end
      else begin 
        async_trap_occuring_int <= async_trap_occuring;
        DECODE_STAGE_EXCEPTION_INT <= DECODE_STAGE_EXCEPTION;
        MEM_STAGE_EXCEPTION_INT <= MEM_STAGE_EXCEPTION;
      end
    end


    reg pc_latch_done;
    always @(posedge CLK or posedge RST) begin
      if(RST) begin 
        pc_latch <= 32'd0;
        pc_latch_cache_stall <= 1'b0;
        pc_latch_done <= 1'b0;
      end
      else if(PC_Control__IRQ_pulse && ~IF_ID_freeze && ~pc_latch_cache_stall && ~pc_latch_done) begin
        pc_latch <=(  (async_trap_occuring_int)     ? PC_EXCEPTION_BUS[XLEN-1:0]:
                      (DECODE_STAGE_EXCEPTION_INT)  ? PC_EXCEPTION_BUS[2*XLEN-1:XLEN]:
                      (MEM_STAGE_EXCEPTION_INT)     ? PC_EXCEPTION_BUS[3*XLEN-1:2*XLEN]:
                    32'd0);
        pc_latch_done <= 1'b1;
      end
      else if(IF_ID_freeze && sync_trap_occuring && ~pc_latch_done) begin
        pc_latch <= (MEM_STAGE_EXCEPTION) ? PC_EXCEPTION_BUS[2*XLEN-1:XLEN] : PC_EXCEPTION_BUS[XLEN-1:0];
        pc_latch_done <= 1'b1;
      end
      else if(IF_ID_freeze && PC_Control__IRQ_pulse && ~pc_latch_done) begin
        pc_latch <= (MEM_STAGE_EXCEPTION_INT) ? PC_EXCEPTION_BUS[3*XLEN-1:2*XLEN]: PC_EXCEPTION_BUS[2*XLEN-1:XLEN];
        pc_latch_cache_stall <= 1'b1;
        pc_latch_done <= 1'b1;
      end
      else if(mepc_wr) begin
        pc_latch <= 32'd0;
        pc_latch_cache_stall <= 1'b0;
        pc_latch_done <= 1'b0;
      end
    end  
   // Rising edge pulse to detect Async interrupt


    assign async_pulse = async_trap_occuring & ~async_int;

    always @(posedge CLK or posedge RST) begin
      if(RST) begin
        async_int <= 1'b0;
      end
      else begin
        async_int <= async_trap_occuring; 
      end
    end
                  
    // Pipeline Stage Freeze and Control logic 


    assign ID_IE_FLUSH = (PC_Control__IRQ & exp_ecall) || (inst_dec_error);
    assign IF_ID_nop = inst_dec_error;



    always @(posedge CLK or posedge RST) begin
      if(RST) begin
        pc_ctrl_ecall   <= 1'b0;
        pc_ctrl_mret    <= 1'b0;
        pc_ctrl_ebreak  <= 1'b0;
      end
      else if(sys[`IS_MRET] && ~Branch_Taken__ex_mem & ~PC_Control__IRQ & ~IF_ID_freeze) begin
        pc_ctrl_mret    <= 1'b1;
        pc_ctrl_ecall   <= 1'b0;
        pc_ctrl_ebreak  <= 1'b0;
      end
      else if(sys[`IS_ECALL] && ~Branch_Taken__ex_mem & ~PC_Control__IRQ & ~IF_ID_freeze) begin
        pc_ctrl_ecall   <= 1'b1;
        pc_ctrl_mret    <= 1'b0;
        pc_ctrl_ebreak  <= 1'b0;
      end
      else if(sys[`IS_EBREAK] && ~Branch_Taken__ex_mem & ~PC_Control__IRQ & ~IF_ID_freeze) begin
        pc_ctrl_ecall <= 1'b0;
        pc_ctrl_mret  <= 1'b0;
        pc_ctrl_ebreak <= 1'b1;
      end
      else if(~Branch_Taken__ex_mem & PC_Control__IRQ & ~IF_ID_freeze) begin
        pc_ctrl_ecall <= 1'b0;
        pc_ctrl_mret  <= 1'b0;
        pc_ctrl_ebreak <= 1'b0;
      end
    end




    assign PC_Control__IRQ_pulse = ~PC_Control__IRQ_int & PC_Control__IRQ;
    always @(posedge CLK or posedge RST) begin
      if(RST) begin
        PC_Control__IRQ_int <= 1'b0;
      end
      else begin
        PC_Control__IRQ_int <= PC_Control__IRQ;
      end
    end


    always @(posedge CLK or posedge RST) begin
      if(RST) begin
        PC_Control__IRQ <= 1'b0;
        count <= 1'b0;
        exp_trap <= 1'b0;
        exp_ecall <= 1'b0;
        //PC_Freeze_IRQ <= 1'b0;
      end
      else if (trap_occuring & ~PC_Control__IRQ & ~interrupt_pending) begin
        PC_Control__IRQ <= 1'b1;
        count <= count + 2'd1;
        exp_trap <= 1'b1;
      end
      else if(ECALL_EBREAK_EXCEPTION & ~Branch_Taken__ex_mem & ~PC_Control__IRQ & ~IF_ID_freeze) begin
        PC_Control__IRQ <= 1'b1;
        exp_ecall <= 1'b1;
        //count <= count + 2'd2;
      end
      else if(exp_ecall && PC_Control__IRQ && ~IF_ID_freeze ) begin
        PC_Control__IRQ <= 1'b0;
        exp_ecall <= 1'b0;
      end
      else if (PC_Control__IRQ && count == 2'd2 && exp_trap && ~IF_ID_freeze) begin
        PC_Control__IRQ <= 1'b0;
        exp_trap <= 1'b0;
        count <= 2'd0;
      end
      else if (PC_Control__IRQ && count != 2'd2 && exp_trap && ~IF_ID_freeze) begin
        PC_Control__IRQ <= PC_Control__IRQ;
        count <= count + 2'd1;
      end

    end

    assign PC_Freeze_IRQ = PC_Control__IRQ;


    always @(posedge CLK or posedge RST) begin
      if(RST) interrupt_pending <= 1'b0;
      else if(((sys[`IS_ECALL] | sys[`IS_EBREAK]) & ~Branch_Taken__ex_mem & ~PC_Control__IRQ & ~IF_ID_freeze)) interrupt_pending <= 1'b1;
      else if(sys[`IS_MRET] && ~PC_Control__IRQ) interrupt_pending <= 1'b0;
    end

    // CSR Register Write 
   reg mepc_wr_done; 
    always @(posedge CLK or posedge RST) begin
      if(RST) begin
        mepc <= {XLEN{1'b0}};
        mepc_wr <= 1'b0;
        mstatus <= {XLEN{1'b0}};
        mstatus_wr <= 1'b0;
        mtval <= {XLEN{1'b0}};
        mtval_wr <= 1'b0;        
        mcause <= {XLEN{1'b0}};
        mcause_wr <= 1'b0;
        //PC_Control__IRQ <= 1'b0;
        //PC_Freeze_IRQ <= 1'b0;
        clr_meip <= 1'b0;
        PC_HALT_BREAK <= 1'b0;
      end
      else if(count == 2'd2/*trap_occuring*/ && ~IF_ID_freeze) begin
        mepc_wr <= 1'b1;
        mepc <= pc_sync_interrupt;
        mcause_wr <= 1'b1;
        mcause <= mcause_code_int;
        mtval_wr <= 1'b1;
        mtval <= mtval_info_int;
        mstatus_wr <= 1'b1;
        mstatus <= mstatus_for_trap;
        clr_meip <= (mcause_code == 'h8000000B);
       //PC_Control__IRQ <= 1'b1;
      //PC_Freeze_IRQ <= 1'b1; 
      end
      /*else if(count == 2'd1 && IF_ID_freeze &&  PC_Control__IRQ_pulse) begin
        mepc_wr <= 1'b1;
        mepc <= exp_pc;
        mcause_wr <= 1'b1;
        mcause <= mcause_code;
        mtval_wr <= 1'b1;
        mtval <= mtval_info;
        mstatus_wr <= 1'b1;
        mstatus <= mstatus_for_trap;
        clr_meip <= (mcause_code == 'h8000000B);
        
      end*/
      else if(sys[`IS_ECALL] && ~Branch_Taken__ex_mem && ~IF_ID_freeze) begin // If ECALL Occurs, then make pc <- mtvec, and write other CSRs like mcause, mtval, mepc with current PC
        mepc_wr <= 1'b1;
        mepc <= PC_EXCEPTION_BUS[XLEN-1:0];
        mcause_wr <= 1'b1;
        mcause <= mcause_code;
        mtval_wr <= 1'b1;
        mtval <= mtval_info;
        mstatus_wr <= 1'b1;
        mstatus <= mstatus_for_trap;
      end
      else if(sys[`IS_EBREAK] && ~Branch_Taken__ex_mem /*&& ~IF_ID_freeze && exp_ecall*/) begin // If EBREAK occurs then halt the processor, Used for debugging
        mepc_wr <= 1'b1;
        mepc <= PC_EXCEPTION_BUS[XLEN-1:0];
        mcause_wr <= 1'b1;
        mcause <= mcause_code;
        PC_HALT_BREAK <= 1'b1; // Todo: Check what to do when EBREAK Occurs
      end
      else if(sys[`IS_MRET] && ~Branch_Taken__ex_mem /*&& ~IF_ID_freeze && exp_ecall*/) begin
        mstatus_wr <= 1'b1;
        mstatus <= mstatus_for_mret;
      end
      else begin
        mepc_wr <= 1'b0;
        mcause_wr <= 1'b0;
        mstatus_wr <= 1'b0;
        mtval_wr <= 1'b0;
        //PC_Control__IRQ <= 1'b0;
        //PC_Freeze_IRQ <= 1'b0;
        clr_meip <= 1'b0;
      end
    end


    // MSTATUS CSR to write when executing MRET
    assign mstatus_for_mret = {csr_mstatus[XLEN-1:23],  // WPRI
                               csr_mstatus[22],         // TSR
                               csr_mstatus[21],         // TW
                               csr_mstatus[20],         // TVM
                               csr_mstatus[19],         // MXR
                               csr_mstatus[18],         // SUM
                               csr_mstatus[17],         // MPRV
                               2'b0,                    // XS
                               csr_mstatus[14:13],      // FS
                               priv_mode,               // MPP
                               csr_mstatus[10:9],       // VS
                               1'b0,                    // SPP
                               1'b0,                    // MPIE
                               1'b0,                    // UBE
                               1'b0,                    // SPIE
                               1'b0,                    // WPRI
                               csr_mstatus[7],          // MIE
                               1'b0,                    // WPRI
                               csr_mstatus[5],          // SIE
                               1'b0};                   // WPRI

    // MSTATUS CSR when handling a trap
    assign mstatus_for_trap = {csr_mstatus[XLEN-1:23],  // WPRI
                               csr_mstatus[22],         // TSR
                               csr_mstatus[21],         // TW
                               csr_mstatus[20],         // TVM
                               csr_mstatus[19],         // MXR
                               csr_mstatus[18],         // SUM
                               csr_mstatus[17],         // MPRV
                               2'b0,                    // XS
                               csr_mstatus[14:13],      // FS
                               priv_mode,               // MPP
                               csr_mstatus[10:9],       // VS
                               1'b0,                    // SPP
                               csr_mstatus[3],          // MPIE
                               1'b0,                    // UBE
                               csr_mstatus[1],          // SPIE
                               1'b0,                    // WPRI
                               1'b0,                    // MIE
                               1'b0,                    // WPRI
                               1'b0,                    // SIE
                               1'b0};                   // WPRI


    assign mtvec_addr = (async_pulse && csr_mtvec[1:0]!=2'b00 ) ?
                        //Vectored Mode
                        {csr_mtvec[`XLEN-1:2],2'b00} + {mcause_code << 2}:
                        //Direct Mode
                        {csr_mtvec[`XLEN-1:2],2'b00};


    ///////////////////////////////////////////////////////////////////////////
    //
    // Asynchronous exceptions code:
    // ----------------------------
    //
    // Exception Code  |   Description
    // ----------------|------------------------------------------
    // 0               |   User software interrupt
    // 1               |   Supervisor software interrupt
    // 2               |   Reserved for future standard use
    // 3               |   Machine software interrupt
    // -----------------------------------------------------------
    // 4               |   User timer interrupt
    // 5               |   Supervisor timer interrupt
    // 6               |   Reserved for future standard use
    // 7               |   Machine timer interrupt
    // -----------------------------------------------------------
    // 8               |   User external interrupt
    // 9               |   Supervisor external interrupt
    // 10              |   Reserved for future standard use
    // 11              |   Machine external interrupt
    // -----------------------------------------------------------
    // 12-15           |   Reserved for future standard use
    // ≥16             |   Reserved for platform use
    // -----------------------------------------------------------
    //
    // Synchronous exception priority in decreasing priority order:
    // -----------------------------------------------------------
    //
    // Priority  |  Exception Code  |   Description
    // ----------|------------------|------------------------------------------
    // Highest   |  3               |   Instruction address breakpoint
    // ------------------------------------------------------------------------
    //           |  12              |   Instruction page fault
    //           |  1               |   Instruction access fault
    // ------------------------------------------------------------------------
    //           |  2               |   Illegal instruction
    //           |  0               |   Instruction address misaligned
    //           |  8,9,11          |   Environment call from U/S/M modes
    //           |  3               |   Environment break
    //           |  3               |   Load/Store/AMO address breakpoint
    // ------------------------------------------------------------------------
    //           |  5               |   Load access fault
    //           |  7               |   Store access fault
    // ------------------------------------------------------------------------
    //           |  6               |   Store/AMO address misaligned
    //           |  4               |   Load address misaligned
    // ------------------------------------------------------------------------
    //           |  15              |   Store/AMO page fault
    //           |  13              |   Load page fault
    // ------------------------------------------------------------------------
    // Lowest    |  7               |   Store/AMO access fault
    //           |  5               |   Load access fault
    // ------------------------------------------------------------------------
    //
    ///////////////////////////////////////////////////////////////////////////

    always @(posedge CLK or posedge RST) begin
      if(RST) mcause_code_int <= {XLEN{1'b0}};
      else if(trap_occuring) mcause_code_int <= mcause_code;
    end

    assign mcause_code = // aync exceptions have highest priority
                          (csr_mip_msip & csr_mie_msie)    ? {1'b1, {XLEN-5{1'b0}}, 4'h3} :
                          (csr_mip_mtip & csr_mie_mtie)    ? {1'b1, {XLEN-5{1'b0}}, 4'h7} :
                          (csr_mip_meip & csr_mie_meie)    ? {1'b1, {XLEN-5{1'b0}}, 4'hB} :
                          // then follow sync exceptions
                          (inst_addr_misaligned)    ? {{XLEN-4{1'b0}}, 4'h0}   :
                          (inst_access_fault)       ? {{XLEN-4{1'b0}}, 4'h1}   :
                          (illegal_instruction)     ? {{XLEN-4{1'b0}}, 4'h2}   :
                          (csr_ro_wr)               ? {{XLEN-4{1'b0}}, 4'h2}   :
                          (sys[`IS_EBREAK])         ? {{XLEN-4{1'b0}}, 4'h3}   :
                          (load_misaligned)         ? {{XLEN-4{1'b0}}, 4'h4}   :
                          (load_access_fault)       ? {{XLEN-4{1'b0}}, 4'h5}   :
                          (store_misaligned)        ? {{XLEN-4{1'b0}}, 4'h6}   :
                          (store_access_fault)      ? {{XLEN-4{1'b0}}, 4'h7}   :
                          (sys[`IS_ECALL])         ? {{XLEN-4{1'b0}}, 4'hB}   :
                          (instruction_page_fault)  ? {{XLEN-4{1'b0}}, 4'hC}   :
                          (load_page_fault)         ? {{XLEN-4{1'b0}}, 4'hD}   :
                          (inst_dec_error)          ? {{XLEN-5{1'b0}}, 5'h18}  :
                                                      32'd0;
  

    always @(posedge CLK or posedge RST) begin
      if(RST) mtval_info_int <= {XLEN{1'b0}};
      else if(trap_occuring) mtval_info_int <= mtval_info;
    end
    // MTVAL: exception-specific information
    assign mtval_info = (inst_access_fault)       ? exp_instruction :
                        (illegal_instruction)     ? exp_instruction :
                        (csr_ro_wr)               ? exp_instruction :
                        (inst_addr_misaligned)    ? pc_reg_IF      :
                        (sys[`IS_ECALL])          ? pc_reg_ID      :
                        (sys[`IS_EBREAK])         ? pc_reg_ID      :
                        (load_access_fault)       ? exp_addr    :
                        (store_access_fault)      ? exp_addr    :
                        (load_misaligned)         ? exp_addr    :
                        (store_misaligned )       ? exp_addr    :
                        (inst_dec_error)          ? exp_instruction :
                        (instruction_page_fault)  ? pc_reg_IF :
                        (load_page_fault)         ? exp_addr :
                                                 32'd0;
    assign async_trap_occuring = ((csr_mip_msip & csr_mie_msie) | 
                                  (csr_mip_mtip & csr_mie_mtie) | 
                                  (csr_mip_meip & csr_mie_meie & !clr_meip)) & csr_mstatus_mie;

    assign sync_trap_occuring = csr_ro_wr            |
                                inst_addr_misaligned |
                                inst_access_fault    |
                                illegal_instruction  |
                                load_misaligned      |
                                store_misaligned     |
                                load_access_fault    |
                                store_access_fault   |
                                inst_dec_error       ;

    assign trap_occuring = async_pulse | sync_trap_occuring;

    
endmodule
