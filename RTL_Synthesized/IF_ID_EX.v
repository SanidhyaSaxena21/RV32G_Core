`timescale 1ns / 1ps
(* keep_hierarchy = "yes" *)
module IF_ID_EX
(
    input CLK,
    input RESET_BUTTON,
    //input RST,
    
    
    
    output [4:0] Load_Store_Op__Port1,                          //memory stage operation signals to lsu
    output reg [31:0] proc_addr_port1,                  //to memory stage
    output [31:0] Store_Data,                       //data to be stored(only used for memory stage)
    output lsustall_o,                                  //mem stage Load__Stall
    output reg Load__Stall,
    input  Data_Cache__Stall,
    input  [31:0] proc_data_port1_int,
    output dbg_memory_req,
    output cache_dis,

    
    output [4:0] lsu_op_port2,
    output reg [31:0] proc_addr_port2,                  //contents of rs1 sent to dcache to read dcache
    input [31:0] amo_load_val_i,
        
    
    
    output Inst_Cache_Freeze,
    output [31:0] pc_cache,                             //send pc to data-cache
    input Inst_Cache__Stall,
    input [31:0] Instruction__IF_ID,
    input instruction_page_fault,
    input data_page_fault,
    input load_access_fault,
    input store_access_fault,
    input load_page_fault,
    input store_page_fault,
    input load_store_fault,
    
    
    output LR_Inst,                                        //to register file
    output SC_Inst,
    
    
    output Mult_Div_unit__Stall,
    output FPU__Stall,
   
    //Debug Module
    output [2:0] state, 
    input   haltreq_i,
    input   resumereq_i,
    input   resethaltreq_i,
    input   ackhavereset,
    input   ackunavail_i,
    input [31:0] data0_i,
    input [31:0] data1_i,
    input   ndmreset_i,
    input   cmd_not_supported_i,
    input   new_cmd_flag_i,

    output [31:0] ptr_regno_data_o,
    output [31:0] ptr_data1_data_o,

    output  command_complete_o,
    output  reg_access_complete_o,
    output  mem_access_complete_o,

    output  core_halted_o,
    output  core_running_o,
    output  core_resumeack_o,
    output  core_havereset_o,
    output  core_unavail_o,
    output  core_exist_o,
    output  abstract_cmd_failed_o,
    output  abstract_cmd_wrong_hart_state_o,
    output  abstract_cmd_exception_o,
    output  abstract_cmd_bus_error_o,

    output  debug_stall,
    output  debug_mode,

    input [31:0] dm_halt_addr,dm_exception_addr,
    output core_dbg_reset,
    input   pbuffer_execution,

    
    //output eret_ack,
    
    
    
    input addr_exception,
    input [31:0] irq_i,

    output badaddr_data,
    
    output [`CSR_SB_W-1:0] csr_pmp_sb, 
    
    output [63:0] led,
    
    output cache_flush_csr,
    input flush_csr_clr,
    
    output [31:0] csr_satp, 
    `ifdef itlb_def
    output tlb_trans_off,
    output vpn_to_ppn_req
    `endif
);


parameter XLEN = 32;
parameter SB_LENGTH = 4*(XLEN+1)+1;

//wire [63:0] led;
//wire eret_ack;

wire tick_en;
// IF_ID Register
wire [31:0] PC__IF_ID;
wire [31:0] PC_4__IF_ID;
wire [10:0] BPU__PHT_Read_Index__if_id;
wire [1:0] BPU__PHT_Read_Data__IF_ID;

wire BPU__Branch_Taken__IF_ID;
wire [31:0] BPU__Branch_Target_Addr__IF_ID;
reg [10:0] BPU__PHT_Read_Index__IF_ID;
wire BPU__BTB_Hit__IF_ID;


// ID_EX Register
(* DONT_TOUCH = "true" *) wire Branch_Inst__id_ex_buf1, Branch_Inst__id_ex_buf2, Branch_Inst__id_ex_buf3, Branch_Inst__id_ex_buf4;
wire [31:0] PC_ID;
reg [31:0] PC_IE;
wire illegal_instruction_ID;
reg illegal_instruction_IE;
wire [31:0] RS1_Data__id_ex;
reg  [31:0] RS1_Data__ID_EX;
wire [31:0] RS2_Data__id_ex;
reg  [31:0] RS2_Data__ID_EX;
wire [5:0] Shamt__id_ex;
reg  [5:0] Shamt__ID_EX;
wire [31:0] Immediate__id_ex;
reg  [31:0] Immediate__ID_EX; 
wire [31:0] Store_Data__id_ex;
reg  [31:0] Store_Data__ID_EX;
wire [6:0] Opcode__id_ex;
reg  [6:0] Opcode__ID_EX;
wire [6:0] Funct7__id_ex;
reg  [6:0] Funct7__ID_EX;
wire [2:0] Funct3__id_ex;
reg  [2:0] Funct3__ID_EX;
wire [4:0] RD_Addr__id_ex;
reg  [4:0] RD_Addr__ID_EX;
wire [4:0] Load_Store_Op__id_ex;
reg  [4:0] Load_Store_Op__ID_EX;
wire [1:0] Alu_Src_1_sel__id_ex;
reg  [1:0] Alu_Src_1_sel__ID_EX;
wire [1:0] Alu_Src_2_sel__id_ex;
reg  [1:0] Alu_Src_2_sel__ID_EX;
wire Branch_Inst__id_ex;
reg  Branch_Inst__ID_EX;
wire Forward_RS1_MEM__id_ex; 
reg  Forward_RS1_MEM__ID_EX;
wire Forward_RS2_MEM__id_ex;
reg  Forward_RS2_MEM__ID_EX;
wire Forward_RS1_WB__id_ex;
reg  Forward_RS1_WB__ID_EX;
wire Forward_RS2_WB__id_ex;
reg  Forward_RS2_WB__ID_EX;
wire Forward_RS1_MEM_FP__id_ex; 
reg  Forward_RS1_MEM_FP__ID_EX;
wire Forward_RS2_MEM_FP__id_ex;
reg  Forward_RS2_MEM_FP__ID_EX;
wire Forward_RS1_WB_FP__id_ex;
reg  Forward_RS1_WB_FP__ID_EX;
wire Forward_RS2_WB_FP__id_ex;
reg  Forward_RS2_WB_FP__ID_EX;
wire JAL_Inst__id_ex;
reg  JAL_Inst__ID_EX;
wire JALR_Inst__id_ex;
reg  JALR_Inst__ID_EX; 
wire Reg_Write_Enable__id_ex;
reg  Reg_Write_Enable__ID_EX;
wire AMO_Inst__id_ex;
reg  AMO_Inst__ID_EX;    
wire [1:0] Mult_Op__id_ex;
reg  [1:0] Mult_Op__ID_EX;                            
wire Mult_En__id_ex; 
reg  Mult_En__ID_EX;
wire [1:0] Div_Op__id_ex; 
reg  [1:0] Div_Op__ID_EX;                     
wire Div_En__id_ex;
reg  Div_En__ID_EX;
wire Load__Stall_id_ex;                       
reg  Load__Stall_ID_EX;
wire BPU__Branch_Taken__id_ex;
wire [31:0] BPU__Branch_Target_Addr__id_ex;
wire [10:0] BPU__PHT_Read_Index__id_ex;
wire [1:0] BPU__PHT_Read_Data__id_ex;
wire [2:0] Branch_Type__id_ex; 
wire BPU__BTB_Hit__id_ex; 
reg BPU__Branch_Taken__ID_EX;
reg [31:0] BPU__Branch_Target_Addr__ID_EX;
reg [10:0] BPU__PHT_Read_Index__ID_EX;
reg [1:0] BPU__PHT_Read_Data__ID_EX;
reg [2:0] Branch_Type__ID_EX;  
reg BPU__BTB_Hit__ID_EX;  
    

wire [4:0] FP__RD_Addr__id_ex;
wire FP__Reg_Write_En__id_ex;
wire [4:0] FP__RD_Addr_Int__id_ex;
wire FP__Reg_Write_En_Int__id_ex;
wire FP__Forward_RS1_MEM__id_ex;
wire FP__Forward_RS2_MEM__id_ex;
wire FP__Forward_RS3_MEM__id_ex;
wire FP__Forward_RS1_WB__id_ex;
wire FP__Forward_RS2_WB__id_ex;
wire FP__Forward_RS3_WB__id_ex;
wire FP__Forward_RS1_MEM_Int__id_ex;              
wire FP__Forward_RS1_WB_Int__id_ex;  
wire FP__Forward_RS1_MEM_Int_FP__id_ex;             
wire FP__Forward_RS1_WB_Int_FP__id_ex;                           
wire [2:0] FP__Fpu_Operation__id_ex;
wire [2:0] FP__Fpu_Sub_Op__id_ex;
wire [2:0] FP__Rounding_Mode__id_ex; 
wire FP__SP_DP__id_ex;
wire FP__Fpu_Inst__id_ex;
wire [63:0] FP__RS1_Data__id_ex;
wire [63:0] FP__RS2_Data__id_ex;
wire [63:0] FP__RS3_Data__id_ex;
wire [63:0] FP__Store_Data__id_ex;
wire FP__Load_Inst__id_ex;
wire FP__Store_Inst__id_ex;

reg  [4:0] FP__RD_Addr__ID_EX;
reg  FP__Reg_Write_En__ID_EX;
reg  [4:0] FP__RD_Addr_Int__ID_EX;
reg  FP__Reg_Write_En_Int__ID_EX;
reg  FP__Forward_RS1_MEM__ID_EX;
reg  FP__Forward_RS2_MEM__ID_EX;
reg  FP__Forward_RS3_MEM__ID_EX;
reg  FP__Forward_RS1_WB__ID_EX;
reg  FP__Forward_RS2_WB__ID_EX;
reg  FP__Forward_RS3_WB__ID_EX;
reg  FP__Forward_RS1_MEM_Int__ID_EX;              
reg  FP__Forward_RS1_WB_Int__ID_EX;
reg  FP__Forward_RS1_MEM_Int_FP__ID_EX;             
reg  FP__Forward_RS1_WB_Int_FP__ID_EX;                
reg  [2:0] FP__Fpu_Operation__ID_EX;
reg  [2:0] FP__Fpu_Sub_Op__ID_EX;
reg  [2:0] FP__Rounding_Mode__ID_EX; 
reg  FP__SP_DP__ID_EX;
reg  FP__Fpu_Inst__ID_EX;
reg  [63:0] FP__RS1_Data__ID_EX;
reg  [63:0] FP__RS2_Data__ID_EX;
reg  [63:0] FP__RS3_Data__ID_EX;
reg  [63:0] FP__Store_Data__ID_EX;
reg  FP__Load_Inst__ID_EX;
reg  FP__Store_Inst__ID_EX;
    
// EX_MEM Register
reg [31:0] PC_MEM;
wire [31:0] pc_ex_mem;
wire Branch_Taken__ex_mem;
reg  Branch_Taken__EX_MEM;
wire [31:0] Branch_Target_Addr__ex_mem;
reg  [31:0] Branch_Target_Addr__EX_MEM;
wire [31:0] RD_Data__ex_mem;
reg  [31:0] RD_Data__EX_MEM;          
wire [31:0] ALU_Result__ex_mem;
reg  [31:0] ALU_Result__EX_MEM;
wire [4:0] RD_Addr__ex_mem;
reg  [4:0] RD_Addr__EX_MEM;
wire Reg_Write_Enable__ex_mem;
reg  Reg_Write_Enable__EX_MEM;
wire [4:0] Load_Store_Op__ex_mem;
reg  [4:0] Load_Store_Op__EX_MEM;       //to keep track of lsuop in the current instr in mem. decides whether wb_data to be used or loaded value.
wire SC_Inst__ex_mem;
reg  SC_Inst__EX_MEM;
wire [31:0] Store_Data__ex_mem;
wire [10:0] PHT_Write_Index__ex_mem;
wire [1:0] PHT_Write_Data__ex_mem;
wire PHT_Write_En__ex_mem;
wire GHR_Write_Data__ex_mem;
wire GHR_Write_En__ex_mem;
wire [31:0] BTB_Write_Addr__ex_mem;
wire [31:0] BTB_Write_Data__ex_mem;
wire BTB_Write_En__ex_mem;
wire RAS_RET_Inst_EX__ex_mem;
wire RAS_CALL_Inst__ex_mem;
wire [31:0] RAS_CALL_Inst_nextPC__ex_mem;

wire [63:0] FP__RD_Data__ex_mem;
wire [31:0] FP__RD_Data_Int__ex_mem;
wire [4:0] FP__RD_Addr__ex_mem;
wire FP__Reg_Write_En__ex_mem;
wire [4:0] FP__RD_Addr_Int__ex_mem;
wire FP__Reg_Write_En_Int__ex_mem;
wire FP__SP_DP__ex_mem;
wire [63:0] FP__Store_Data__ex_mem;
wire FP__Load_Inst__ex_mem; 
wire FP__Store_Inst__ex_mem;

reg  [63:0] FP__RD_Data__EX_MEM;
reg  [31:0] FP__RD_Data_Int__EX_MEM;
reg  [4:0] FP__RD_Addr__EX_MEM;
reg  FP__Reg_Write_En__EX_MEM;
reg  [4:0] FP__RD_Addr_Int__EX_MEM;
reg  FP__Reg_Write_En_Int__EX_MEM;
reg  FP__SP_DP__EX_MEM;
reg  [63:0] FP__Store_Data__EX_MEM;
wire [4:0] FPU_flags;
wire [2:0] frm;


// MEM_WB Register
wire [31:0] RD_Data__mem_wb;
reg  [31:0] RD_Data__MEM_WB;
wire Branch_Taken__mem_wb;
reg  Branch_Taken__MEM_WB;

wire [63:0] FP__RD_Data__mem_wb;
reg  [63:0] FP__RD_Data__MEM_WB;
wire [31:0] FP__RD_Data_Int__mem_wb;
reg  [31:0] FP__RD_Data_Int__MEM_WB;
reg FP__Reg_Write_En_Int__MEM_WB;
wire FP__Reg_Write_En_Int__mem_wb;
 


// Register File
wire [4:0] RS1_Addr__rf;
wire [4:0] RS2_Addr__rf;
wire [31:0] RS1_Data__rf;
wire [31:0] RS2_Data__rf;

wire [4:0] FP__RS1_Addr__rf;
wire [4:0] FP__RS2_Addr__rf;
wire [4:0] FP__RS3_Addr__rf;
wire [4:0] FP__RS1_Addr_Int__rf;
wire FP__RS1_read_Int__rf;
wire [63:0] FP__RS1_Data__rf;         
wire [63:0] FP__RS2_Data__rf;
wire [63:0] FP__RS3_Data__rf;
wire [31:0] FP__RS1_Data_Int__rf;





wire NOP__IF_ID;
wire [31:0] pc_forw;
reg  [31:0] pc_id_ex;
reg  lsustall_i;
//wire irq_ctrl_int;
//reg irq_ctrl_int2;
wire  csr_wr_en_int;
//wire  mepc_res_int;
wire  mret_int;
wire Mult_Div_unit_Freeze;
wire FPU_Freeze;
wire eret_int;
//reg irq_ctrl_o;

wire [SB_LENGTH-1:0]  sb_csr;
wire [31:0] csr_mtvec;
wire [31:0] mtvec_addr;
wire [31:0] csr_mstatus;
wire [31:0] csr_mie;
wire [31:0] csr_mip;
wire [31:0] csr_mepc;
wire [31:0] csr_wrdata;
reg [11:0] csr_adr;
wire [11:0] csr_adr_int;
wire [31:0] csr_indata;
reg  csr_wr_en;
//reg  mepc_res;
wire  trap_en;
wire badaddr;
//wire badaddr_data;
wire mtie;
wire pc_ctrl_ecall,pc_ctrl_mret,pc_ctrl_ebreak;

reg badaddr_data_load, badaddr_data_store;

wire [31:0] count;                 //System timers/counter value. RDCYC[H], RDTIM[H], RDINST[H] instructions
reg [3:0] wr_sel;
wire [3:0] count_sel_int;
wire tick_en_int;

//wire [31:0] csr_indata_intr;
reg eret;
wire [5:0] device_id;
//wire [31:0] inst_inj;              //Injected Instruction__IF_ID stream
//wire irq_icache_freeze;
wire eret_o;
wire irq;
//wire irq_ack;
//wire irq_ctrl;
//wire irq_ctrl_wb_i;
wire PC_Control__IRQ;
//wire irq_ctrl_dec_src1;
//wire irq_ctrl_dec_src2;
//wire IF_ID_Freeze__irq;
wire PC_Freeze_IRQ, ID_IE_FLUSH, IF_ID_nop;

reg Inst_Cache__Stall__reg;
reg [31:0] proc_data_port1_int__reg;
reg FPU__Stall__reg;
reg [63:0] Double_Load_Data;
reg Mult_Div_unit__Stall__reg;

reg Double_Load_Store__Stall;
reg Double_Load_Store__Stall_reg;
reg Double_Load_Store__Stall_pre;
reg [31:0] Double_Load_Buffer;
reg [31:0] Double_Store_Buffer;
reg [31:0] ALU_Result__ex_mem__reg;

wire Mult_Div_unit__Stall_disable;
wire FPU__Stall_disable;

wire [31:0] inst_out;

wire [5:0] sys_id_ex;
reg [5:0] sys_ID_EX;
wire interrupt_pending;

wire PC_HALT_BREAK;


// Debug Signals
wire pc_valid_dbg;
wire [31:0] pc_dbg;
wire dbg_instr_req;
wire csr_dbg_wr;
wire [2:0] csr_dbg_cause;
wire [31:0] csr_dbg_dpc;
wire [31:0] dcsr;
wire [31:0] csr_dbg_dscratch0;
wire [31:0] csr_dbg_dscratch1;
wire return_pc_valid;
wire [31:0] return_pc;
wire [31:0] dpc;
wire clr_step;
wire [4:0] RS1_debug_addr;
wire [4:0] Rd_debug_addr;
wire RegWrite_debug;
wire RegRead_debug;
wire [31:0] Rd_debug_data;

wire csr_dbg_wren;
wire [11:0] csr_dbg_addr;
wire [31:0] csr_dbg_wdata;

wire [31:0] memory_addr_dbg,data_write_dbg,data_read_dbg;
wire [4:0] Lsu_op_dbg;
//wire dbg_memory_req;

wire ext_irq;
wire sw_irq;
wire timer_irq;

assign sw_irq = irq_i[3];
assign timer_irq = irq_i[7];
assign ext_irq = irq_i[11];
//----------------WARM RESET GENERATION----------------
//
//

assign RST = RESET_BUTTON | core_dbg_reset;
//assign RST = RESET_BUTTON;


reg halt_ie_mem_reg;
wire halt_if_id,halt_id_ie,halt_ie_mem,halt_mem_wb;
wire kill_if,kill_id,kill_ie,kill_mem;


assign debug_stall = (halt_if_id | halt_id_ie | halt_ie_mem | halt_mem_wb);
assign Inst_Cache_Freeze = (Mult_Div_unit__Stall | FPU__Stall | Data_Cache__Stall | /*irq_icache_freeze |*/ Double_Load_Store__Stall | (debug_mode & ~core_resumeack_o) | halt_if_id) ;
//assign Inst_Cache_Freeze = (Mult_Div_unit__Stall | FPU__Stall | Data_Cache__Stall | /*irq_icache_freeze |*/ Double_Load_Store__Stall | halt_if_id | halt_id_ie | halt_ie_mem | halt_mem_wb) ;

assign Mult_Div_unit_Freeze = (Mult_Div_unit__Stall) ? (Data_Cache__Stall | Double_Load_Store__Stall) : (Inst_Cache__Stall | Data_Cache__Stall | Double_Load_Store__Stall | halt_if_id | halt_id_ie | halt_ie_mem | halt_mem_wb);

assign FPU_Freeze = (FPU__Stall) ? (Data_Cache__Stall | Double_Load_Store__Stall) : (Inst_Cache__Stall | Data_Cache__Stall | Double_Load_Store__Stall | halt_if_id | halt_id_ie | halt_ie_mem | halt_mem_wb);

assign Mult_Div_unit__Stall_disable = (Data_Cache__Stall | Double_Load_Store__Stall | halt_if_id | halt_id_ie | halt_ie_mem | halt_mem_wb);

assign FPU__Stall_disable = (Data_Cache__Stall | Double_Load_Store__Stall | halt_if_id | halt_id_ie | halt_ie_mem | halt_mem_wb);





assign Store_Data = dbg_memory_req ? data_write_dbg : (Double_Load_Store__Stall_reg ? Double_Store_Buffer : (FP__Store_Inst__ex_mem ? FP__Store_Data__ex_mem[31:0] : Store_Data__ex_mem));

assign SC_Inst = SC_Inst__ex_mem;

assign Load_Store_Op__Port1 = dbg_memory_req ? Lsu_op_dbg: Load_Store_Op__ex_mem;



assign tick_en  = tick_en_int && mtie;

assign badaddr = ((~proc_addr_port1[0] && ~proc_addr_port1[1]) || (~pc_id_ex[0] && ~pc_id_ex[1])) ;
//assign badaddr_data = ~((~proc_addr_port1[0] && ~proc_addr_port1[1])) ;


assign badaddr_data = badaddr_data_load || badaddr_data_store;

always @(*) begin
  case(Load_Store_Op__Port1)
    5'b00001: badaddr_data_load = 1'b0;
    5'b00101: badaddr_data_load = proc_addr_port1[0];
    5'b01001: badaddr_data_load =  ~((~proc_addr_port1[0] && ~proc_addr_port1[1])) ;
    5'b01101: badaddr_data_load = 1'b0;
    5'b10001: badaddr_data_load = 1'b0;
    5'b10101: badaddr_data_load = proc_addr_port1[0];
    5'b11001: badaddr_data_load =  ~((~proc_addr_port1[0] && ~proc_addr_port1[1])) ;
    default: badaddr_data_load = 1'b0;
  endcase
end


always @(*) begin
  case(Load_Store_Op__Port1)
    5'b00010: badaddr_data_store = 1'b0;
    5'b00110: badaddr_data_store = proc_addr_port1[0];
    5'b01010: badaddr_data_store =  ~((~proc_addr_port1[0] && ~proc_addr_port1[1])) ;
    5'b01110: badaddr_data_store = 1'b0;
    default: badaddr_data_store = 1'b0;
  endcase
end

assign RD_Data__mem_wb = (((~Load_Store_Op__EX_MEM[1]) & (Load_Store_Op__EX_MEM[0])) | SC_Inst__EX_MEM) ? (((Inst_Cache__Stall__reg == 1'b1) || (FPU__Stall__reg == 1'b1) || (Mult_Div_unit__Stall__reg == 1'b1) || (halt_ie_mem_reg == 1'b1)) ? proc_data_port1_int__reg : proc_data_port1_int) : RD_Data__EX_MEM;         //For conventional loads

assign FP__RD_Data__mem_wb = ((~Load_Store_Op__EX_MEM[1]) & (Load_Store_Op__EX_MEM[0])) ? (((Inst_Cache__Stall__reg == 1'b1) || (FPU__Stall__reg == 1'b1) || (Mult_Div_unit__Stall__reg == 1'b1) || (halt_ie_mem_reg == 1'b1)) ? ((FP__SP_DP__EX_MEM) ? Double_Load_Data : proc_data_port1_int__reg) : ((FP__SP_DP__EX_MEM) ? {proc_data_port1_int,Double_Load_Buffer} : proc_data_port1_int)) : FP__RD_Data__EX_MEM;    

assign FP__RD_Data_Int__mem_wb = FP__RD_Data_Int__EX_MEM;

assign FP__Reg_Write_En_Int__mem_wb = FP__Reg_Write_En_Int__EX_MEM;

assign FP__RS1_Data_Int__rf = RS1_Data__rf;

assign Branch_Taken__mem_wb = Branch_Taken__EX_MEM;



always @(*) begin

    proc_addr_port1 = dbg_memory_req ? memory_addr_dbg : (AMO_Inst__ID_EX ? (Forward_RS1_MEM__ID_EX ? (ALU_Result__EX_MEM) : (Forward_RS1_WB__ID_EX ? RD_Data__MEM_WB : RS1_Data__ID_EX)) : (Double_Load_Store__Stall_reg ? (ALU_Result__ex_mem__reg + 4) : ALU_Result__ex_mem));
    
    proc_addr_port2 = Forward_RS1_MEM__id_ex ? RD_Data__ex_mem : (Forward_RS1_WB__id_ex ? RD_Data__mem_wb : RS1_Data__rf);    //forwarding for rs1 of amo
    
    Load__Stall = (Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || Double_Load_Store__Stall) ? 1'b0 : Load__Stall_id_ex;
    
end




always @(posedge CLK) begin
    if(RST) begin
        Double_Load_Store__Stall_reg <= 1'b0;
    end
    else if(~(Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || debug_stall)) begin
        Double_Load_Store__Stall_reg <= Double_Load_Store__Stall;
    end 
end

always @(*) begin
    /*if (RST) begin
        Double_Load_Store__Stall = 1'b0;
    end*/
    if (Double_Load_Store__Stall_reg == 1'b1) begin
        Double_Load_Store__Stall = 1'b0;
    end
    else if (Load_Store_Op__ex_mem[4:2] == 3'b011) begin
        Double_Load_Store__Stall = 1'b1;
    end
    else begin
        Double_Load_Store__Stall = 1'b0;
    end
end

always @(posedge CLK) begin
    if(RST) begin
        Double_Load_Buffer <= 32'h00000000;
        Double_Store_Buffer <= 32'h00000000;
    end
    else if (Double_Load_Store__Stall_pre) begin
        Double_Load_Buffer <= proc_data_port1_int;
    end 
    else if (Double_Load_Store__Stall) begin
        Double_Store_Buffer <= FP__Store_Data__ex_mem[63:32];
    end 
end

always @(posedge CLK) begin
    if(RST) begin
        ALU_Result__ex_mem__reg <= 32'h00000000;
    end
    else if(~(Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || debug_stall)) begin
        ALU_Result__ex_mem__reg <= ALU_Result__ex_mem;
    end 
end


always @(posedge CLK) begin
    if(RST) begin
        Inst_Cache__Stall__reg <= 1'b0;
        FPU__Stall__reg <= 1'b0;
        Mult_Div_unit__Stall__reg <= 1'b0;
        halt_ie_mem_reg <= 1'b0;

    end
    else begin
        Inst_Cache__Stall__reg <= Inst_Cache__Stall;
        FPU__Stall__reg <= FPU__Stall;
        Mult_Div_unit__Stall__reg <= Mult_Div_unit__Stall;
        halt_ie_mem_reg <= halt_ie_mem;
    end 
end

always @(posedge CLK) begin
    if(RST) begin
        Double_Load_Store__Stall_pre <= 1'b0;
    end
    else if (~Data_Cache__Stall) begin
        Double_Load_Store__Stall_pre <= Double_Load_Store__Stall;
    end 
end





// Register IF_ID stage
always @(posedge CLK) begin
    if(RST) begin   
        BPU__PHT_Read_Index__IF_ID <= 11'b0;            
    end
    else if(~(Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || Double_Load_Store__Stall || halt_if_id)) begin
        BPU__PHT_Read_Index__IF_ID <= BPU__PHT_Read_Index__if_id;                      
    end
end


// Register ID_EX stage
always @(posedge CLK) begin
    if(RST) begin
        RS1_Data__ID_EX <= 32'b0;
        RS2_Data__ID_EX <= 32'b0;
        Shamt__ID_EX <= 6'b0;
        Immediate__ID_EX <= 32'b0;
        Store_Data__ID_EX <= 0;
        Opcode__ID_EX <= 7'b0;
        Funct7__ID_EX <= 7'b0;
        Funct3__ID_EX <= 3'b0;
        RD_Addr__ID_EX <= 5'b0;
        Load_Store_Op__ID_EX <= 5'b0;
        Alu_Src_1_sel__ID_EX <= 2'b0;
        Alu_Src_2_sel__ID_EX <= 2'b0; 
        Branch_Inst__ID_EX <= 1'b0;      
        Forward_RS1_WB__ID_EX <= 1'b0;
        Forward_RS2_WB__ID_EX <= 1'b0;
        Forward_RS1_MEM__ID_EX <= 1'b0;
        Forward_RS2_MEM__ID_EX <= 1'b0;
        Forward_RS1_WB_FP__ID_EX <= 1'b0;
        Forward_RS2_WB_FP__ID_EX <= 1'b0;
        Forward_RS1_MEM_FP__ID_EX <= 1'b0;
        Forward_RS2_MEM_FP__ID_EX <= 1'b0;
        JAL_Inst__ID_EX <= 1'b0;
        JALR_Inst__ID_EX <= 1'b0;
        Reg_Write_Enable__ID_EX <= 1'b0;
        AMO_Inst__ID_EX <= 1'b0; 
        Mult_Op__ID_EX <= 2'b00;
        Mult_En__ID_EX <= 1'b0;
        Div_Op__ID_EX <= 2'b00;
        Div_En__ID_EX <= 1'b0;
        Load__Stall_ID_EX <= 1'b0; 
        BPU__Branch_Taken__ID_EX <= 1'b0; 
        BPU__Branch_Target_Addr__ID_EX <= 32'b0; 
        BPU__PHT_Read_Index__ID_EX <= 11'b0; 
        BPU__PHT_Read_Data__ID_EX <= 2'b0; 
        BPU__BTB_Hit__ID_EX <= 1'b0;
        Branch_Type__ID_EX <= 3'b0;    
        
        FP__RD_Addr__ID_EX <= 5'b0;
        FP__Reg_Write_En__ID_EX <= 1'b0;
        FP__RD_Addr_Int__ID_EX <= 5'b0;   
        FP__Reg_Write_En_Int__ID_EX <= 1'b0;    
        FP__Forward_RS1_MEM__ID_EX <= 1'b0;     
        FP__Forward_RS2_MEM__ID_EX <= 1'b0;     
        FP__Forward_RS3_MEM__ID_EX <= 1'b0;     
        FP__Forward_RS1_WB__ID_EX <= 1'b0;      
        FP__Forward_RS2_WB__ID_EX <= 1'b0;      
        FP__Forward_RS3_WB__ID_EX <= 1'b0;      
        FP__Forward_RS1_MEM_Int__ID_EX <= 1'b0; 
        FP__Forward_RS1_WB_Int__ID_EX <= 1'b0;  
        FP__Forward_RS1_MEM_Int_FP__ID_EX <= 1'b0;
        FP__Forward_RS1_WB_Int_FP__ID_EX <= 1'b0; 
        FP__Fpu_Operation__ID_EX <= 3'b0; 
        FP__Fpu_Sub_Op__ID_EX <= 3'b0;    
        FP__Rounding_Mode__ID_EX <= 3'b0; 
        FP__SP_DP__ID_EX <= 1'b0;               
        FP__Fpu_Inst__ID_EX <= 1'b0;            
        FP__RS1_Data__ID_EX <= 64'b0;     
        FP__RS2_Data__ID_EX <= 64'b0;     
        FP__RS3_Data__ID_EX <= 64'b0; 
        FP__Store_Data__ID_EX <= 64'b0; 
        FP__Load_Inst__ID_EX <= 1'b0; 
        FP__Store_Inst__ID_EX <= 1'b0;
     	  sys_ID_EX <= 6'd0;
        PC_IE <= 32'd0;
        illegal_instruction_IE <= 1'b0;  
    end
    else if(~(Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || Double_Load_Store__Stall || ID_IE_FLUSH || PC_HALT_BREAK || halt_id_ie)) begin
        sys_ID_EX <= sys_id_ex;
        PC_IE <= PC_ID;
        illegal_instruction_IE <= illegal_instruction_ID;
        RS1_Data__ID_EX <= RS1_Data__id_ex;
        RS2_Data__ID_EX <= RS2_Data__id_ex;
        Shamt__ID_EX <= Shamt__id_ex;
        Immediate__ID_EX <= Immediate__id_ex;
        Store_Data__ID_EX <= Store_Data__id_ex;
        Opcode__ID_EX <= Opcode__id_ex;
        Funct7__ID_EX <= Funct7__id_ex;
        Funct3__ID_EX <= Funct3__id_ex;
        RD_Addr__ID_EX <= RD_Addr__id_ex;
        Load_Store_Op__ID_EX <= Load_Store_Op__id_ex;
        Alu_Src_1_sel__ID_EX <= Alu_Src_1_sel__id_ex;
        Alu_Src_2_sel__ID_EX <= Alu_Src_2_sel__id_ex;
        Branch_Inst__ID_EX <= Branch_Inst__id_ex_buf4;
        Forward_RS1_WB__ID_EX <= Forward_RS1_WB__id_ex;
        Forward_RS2_WB__ID_EX <= Forward_RS2_WB__id_ex;
        Forward_RS1_MEM__ID_EX <= Forward_RS1_MEM__id_ex;
        Forward_RS2_MEM__ID_EX <= Forward_RS2_MEM__id_ex;
        Forward_RS1_WB_FP__ID_EX <= Forward_RS1_WB_FP__id_ex;
        Forward_RS2_WB_FP__ID_EX <= Forward_RS2_WB_FP__id_ex;
        Forward_RS1_MEM_FP__ID_EX <= Forward_RS1_MEM_FP__id_ex;
        Forward_RS2_MEM_FP__ID_EX <= Forward_RS2_MEM_FP__id_ex;
        FP__Forward_RS1_MEM_Int_FP__ID_EX <= FP__Forward_RS1_MEM_Int_FP__id_ex;
        FP__Forward_RS1_WB_Int_FP__ID_EX <= FP__Forward_RS1_WB_Int_FP__id_ex; 
        JAL_Inst__ID_EX <= JAL_Inst__id_ex;
        JALR_Inst__ID_EX <= JALR_Inst__id_ex;
        Reg_Write_Enable__ID_EX <= Reg_Write_Enable__id_ex; 
        AMO_Inst__ID_EX <= AMO_Inst__id_ex; 
        Mult_Op__ID_EX <= Mult_Op__id_ex;
        Mult_En__ID_EX <= Mult_En__id_ex;
        Div_Op__ID_EX <= Div_Op__id_ex;
        Div_En__ID_EX <= Div_En__id_ex; 
        Load__Stall_ID_EX <= Load__Stall_id_ex;
        BPU__Branch_Taken__ID_EX <= BPU__Branch_Taken__id_ex; 
        BPU__Branch_Target_Addr__ID_EX <= BPU__Branch_Target_Addr__id_ex; 
        BPU__PHT_Read_Index__ID_EX <= BPU__PHT_Read_Index__id_ex; 
        BPU__PHT_Read_Data__ID_EX <= BPU__PHT_Read_Data__id_ex; 
        BPU__BTB_Hit__ID_EX <= BPU__BTB_Hit__id_ex;
        Branch_Type__ID_EX <= Branch_Type__id_ex; 
        
        
        FP__RD_Addr__ID_EX <= FP__RD_Addr__id_ex;
        FP__Reg_Write_En__ID_EX <= FP__Reg_Write_En__id_ex;
        FP__RD_Addr_Int__ID_EX <= FP__RD_Addr_Int__id_ex;
        FP__Reg_Write_En_Int__ID_EX <= FP__Reg_Write_En_Int__id_ex;
        FP__Forward_RS1_MEM__ID_EX <= FP__Forward_RS1_MEM__id_ex;
        FP__Forward_RS2_MEM__ID_EX <= FP__Forward_RS2_MEM__id_ex;
        FP__Forward_RS3_MEM__ID_EX <= FP__Forward_RS3_MEM__id_ex;
        FP__Forward_RS1_WB__ID_EX <= FP__Forward_RS1_WB__id_ex;
        FP__Forward_RS2_WB__ID_EX <= FP__Forward_RS2_WB__id_ex;
        FP__Forward_RS3_WB__ID_EX <= FP__Forward_RS3_WB__id_ex;
        FP__Forward_RS1_MEM_Int__ID_EX <= FP__Forward_RS1_MEM_Int__id_ex;              
        FP__Forward_RS1_WB_Int__ID_EX <= FP__Forward_RS1_WB_Int__id_ex;               
        FP__Fpu_Operation__ID_EX <= FP__Fpu_Operation__id_ex;
        FP__Fpu_Sub_Op__ID_EX <= FP__Fpu_Sub_Op__id_ex;
        FP__Rounding_Mode__ID_EX <= FP__Rounding_Mode__id_ex; 
        FP__SP_DP__ID_EX <= FP__SP_DP__id_ex;
        FP__Fpu_Inst__ID_EX <= FP__Fpu_Inst__id_ex;
        FP__RS1_Data__ID_EX <= FP__RS1_Data__id_ex;
        FP__RS2_Data__ID_EX <= FP__RS2_Data__id_ex;
        FP__RS3_Data__ID_EX <= FP__RS3_Data__id_ex; 
        FP__Store_Data__ID_EX <= FP__Store_Data__id_ex;
        FP__Load_Inst__ID_EX <= FP__Load_Inst__id_ex; 
        FP__Store_Inst__ID_EX <= FP__Store_Inst__id_ex;
    end 
end


// Register EX_MEM stage
always @(posedge CLK) begin
    if(RST) begin             
        Branch_Taken__EX_MEM  <= 1'b0;
        Branch_Target_Addr__EX_MEM  <= 32'b0;
        RD_Data__EX_MEM <= 32'b0;
        ALU_Result__EX_MEM <= 32'b0;
        RD_Addr__EX_MEM  <= 5'b0;
        Reg_Write_Enable__EX_MEM  <= 1'b0;
        Load_Store_Op__EX_MEM <= 5'b0; 
        SC_Inst__EX_MEM <= 1'b0; 
        proc_data_port1_int__reg <= 32'b0;
        
        
        FP__RD_Data__EX_MEM <= 64'b0;       
        FP__RD_Data_Int__EX_MEM <= 32'b0;   
        FP__RD_Addr__EX_MEM <= 5'b0;        
        FP__Reg_Write_En__EX_MEM <= 1'b0; 
        FP__RD_Addr_Int__EX_MEM <= 5'b0;
        FP__Reg_Write_En_Int__EX_MEM <= 1'b0;
        FP__SP_DP__EX_MEM <= 1'b0;
        FP__Store_Data__EX_MEM <= 64'b0;   
        Double_Load_Data <= 64'b0;
        PC_MEM   <= 32'd0;        
    end
    else if(~(Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || Double_Load_Store__Stall || PC_Freeze_IRQ || PC_HALT_BREAK || halt_ie_mem)) begin            
        PC_MEM <= pc_ex_mem;
        Branch_Taken__EX_MEM  <= Branch_Taken__ex_mem;
        Branch_Target_Addr__EX_MEM  <= Branch_Target_Addr__ex_mem;
        RD_Data__EX_MEM <= RD_Data__ex_mem;
        ALU_Result__EX_MEM <= ALU_Result__ex_mem;
        RD_Addr__EX_MEM  <= RD_Addr__ex_mem;
        Reg_Write_Enable__EX_MEM  <= Reg_Write_Enable__ex_mem; 
        Load_Store_Op__EX_MEM <= Load_Store_Op__ex_mem; 
        SC_Inst__EX_MEM <= SC_Inst__ex_mem; 
        
        
        FP__RD_Data__EX_MEM <= FP__RD_Data__ex_mem;      
        FP__RD_Data_Int__EX_MEM <= FP__RD_Data_Int__ex_mem;  
        FP__RD_Addr__EX_MEM <= FP__RD_Addr__ex_mem;       
        FP__Reg_Write_En__EX_MEM <= FP__Reg_Write_En__ex_mem;
        FP__RD_Addr_Int__EX_MEM <= FP__RD_Addr_Int__ex_mem;
        FP__Reg_Write_En_Int__EX_MEM <= FP__Reg_Write_En_Int__ex_mem;
        FP__SP_DP__EX_MEM <= FP__SP_DP__ex_mem;
        FP__Store_Data__EX_MEM <= FP__Store_Data__ex_mem;           
    end
    else if(Branch_Taken__EX_MEM && PC_Freeze_IRQ ) begin
      Branch_Taken__EX_MEM <= 1'b0;
      Branch_Target_Addr__EX_MEM <= 32'd0;
    end 
    else if(((Inst_Cache__Stall__reg == 1'b0) && (Inst_Cache__Stall == 1'b1)) || ((FPU__Stall__reg == 1'b0) && (FPU__Stall == 1'b1)) || ((Mult_Div_unit__Stall__reg == 1'b0) && (Mult_Div_unit__Stall == 1'b1)) || (halt_ie_mem_reg == 1'b0 && (halt_ie_mem))) begin
        Reg_Write_Enable__EX_MEM  <= 1'b0;
        proc_data_port1_int__reg <= proc_data_port1_int;
        FP__Reg_Write_En__EX_MEM <= 1'b0;
        FP__Reg_Write_En_Int__EX_MEM <= 1'b0;
        Double_Load_Data <= {proc_data_port1_int,Double_Load_Buffer};
    end
end


// Register MEM_WB stage
always @(posedge CLK) begin
    if(RST) begin
        RD_Data__MEM_WB <= 32'b0;
        Branch_Taken__MEM_WB <= 1'b0;
        
        FP__RD_Data__MEM_WB <= 64'b0;
        FP__RD_Data_Int__MEM_WB <= 32'b0;
        FP__Reg_Write_En_Int__MEM_WB <= 1'b0;
    end
    else if(~(Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || Double_Load_Store__Stall_reg || PC_HALT_BREAK || halt_mem_wb)) begin
        RD_Data__MEM_WB <= RD_Data__mem_wb;
        Branch_Taken__MEM_WB <= Branch_Taken__mem_wb;
        
        FP__RD_Data__MEM_WB <= FP__RD_Data__mem_wb;
        FP__RD_Data_Int__MEM_WB <= FP__RD_Data_Int__mem_wb; 
        FP__Reg_Write_En_Int__MEM_WB <= FP__Reg_Write_En_Int__mem_wb;   
    end
end



always @(posedge CLK) begin
    if(RST) begin
        pc_id_ex <= 32'b0;
        //irq_ctrl_int2 <= 1'b0;
        //irq_ctrl_o <= 1'b0;
        wr_sel <= 3'b0;
        csr_wr_en <= 1'b0;
        //mepc_res <= 1'b0;
        csr_adr <= 12'b0;
        eret <= 1'b0;
    end
    else if(~(Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || Double_Load_Store__Stall || PC_HALT_BREAK || halt_id_ie)) begin
        pc_id_ex <= pc_forw;
        //irq_ctrl_int2 <= irq_ctrl_int;
        //irq_ctrl_o <= irq_ctrl_int2;                                //Output of decode_execute stage
        wr_sel <= count_sel_int;
        
        //mepc_res <= mepc_res_int;
        csr_adr <= csr_adr_int;
        eret <= eret_int;
        
        if(Branch_Taken__ex_mem == 1'b1)
            csr_wr_en <= 1'b0;
        else
            csr_wr_en <= csr_wr_en_int;
    end   
end


always @(posedge CLK) begin
    if(RST) 
        lsustall_i <= 1'b0;
    else
        lsustall_i <= Load__Stall_id_ex;
end



/*reg IF_ID_Freeze__irq__reg;

always @(posedge CLK ) begin
    if(RST) begin
        IF_ID_Freeze__irq__reg = 1'b0;
    end
    else  begin
        IF_ID_Freeze__irq__reg = IF_ID_Freeze__irq;
    end 
end */

//------------------------------------------------------------------------
//  
//   BRANCH PREDICTION UNIT 
//
//------------------------------------------------------------------------

(* keep_hierarchy = "yes" *)
Branch_Prediction_Unit BPU( .CLK(CLK),
                            .RST(RST),
                            .PC(pc_cache),
                            .BPU__Stall(Data_Cache__Stall || Inst_Cache__Stall || Mult_Div_unit__Stall || FPU__Stall || Double_Load_Store__Stall || PC_HALT_BREAK),
                            .Branch_Taken_n(BPU__Branch_Taken__IF_ID),
                            .Branch_Target_Addr_n(BPU__Branch_Target_Addr__IF_ID),
                            .BTB_Hit(BPU__BTB_Hit__IF_ID),
                            .PHT_Read_Index(BPU__PHT_Read_Index__if_id),
                            .PHT_Read_Data(BPU__PHT_Read_Data__IF_ID),
                            .PHT_Write_Index(PHT_Write_Index__ex_mem),
                            .PHT_Write_Data(PHT_Write_Data__ex_mem),
                            .PHT_Write_En(PHT_Write_En__ex_mem),
                            .GHR_Write_Data(GHR_Write_Data__ex_mem),
                            .GHR_Write_En(GHR_Write_En__ex_mem),
                            .BTB_Write_Addr(BTB_Write_Addr__ex_mem),
                            .BTB_Write_Data(BTB_Write_Data__ex_mem),
                            .BTB_Write_En(BTB_Write_En__ex_mem),
                            .RAS_RET_Inst_EX(RAS_RET_Inst_EX__ex_mem),
                            .RAS_CALL_Inst(RAS_CALL_Inst__ex_mem),
                            .RAS_CALL_Inst_nextPC(RAS_CALL_Inst_nextPC__ex_mem),
                            .Branch_Taken__EX_MEM(Branch_Taken__EX_MEM),
                            .PC_Control__IRQ(PC_Control__IRQ));
                           
//------------------------------------------------------------------------
//  
//   INSTRUCTION FETCH UNIT 
//
//------------------------------------------------------------------------
 
(* keep_hierarchy = "yes" *)
INST_FETCH IF( .CLK(CLK),
               .RST(RST), 
               .Branch_Taken__EX_MEM(Branch_Taken__EX_MEM),
               .Branch_Taken__MEM_WB(Branch_Taken__MEM_WB),
               .Branch_Target_Addr__EX_MEM(Branch_Target_Addr__EX_MEM),
               .BPU__Branch_Taken__IF_ID(BPU__Branch_Taken__IF_ID),
               .BPU__Branch_Target_Addr__IF_ID(BPU__Branch_Target_Addr__IF_ID),
               .CSR_mtvec(mtvec_addr),
               .Load__Stall(Load__Stall),
               .IF_ID_Freeze(PC_Freeze_IRQ || /*IF_ID_Freeze__irq ||*/ Data_Cache__Stall || (Inst_Cache__Stall & ~core_resumeack_o) || Mult_Div_unit__Stall || FPU__Stall || Double_Load_Store__Stall || PC_HALT_BREAK),
               .PC_Control__IRQ(PC_Control__IRQ),
               .pc_ctrl_ecall(pc_ctrl_ecall),
               .pc_ctrl_mret(pc_ctrl_mret),
               .pc_ctrl_ebreak(pc_ctrl_ebreak),
               .mepc(csr_mepc),
               //.Device_id(device_id),
               .csr_satp(csr_satp),
               .IF_ID_nop(IF_ID_nop),
               .PC__IF_ID(PC__IF_ID), 
               .PC_4__IF_ID(PC_4__IF_ID), 
               .NOP__IF_ID(NOP__IF_ID),
               .interrupt_pending(interrupt_pending),
               .pc(pc_cache),
               //Debug Controller --> IF Stage
               .pc_valid_dbg(pc_valid_dbg),
               .pc_dbg(pc_dbg),
               .return_pc_valid(return_pc_valid),
               .return_pc(return_pc),
               .dbg_instr_req(dbg_instr_req),
               .kill_if(kill_if),
               .halt_if(halt_if_id),
               `ifdef itlb_def
                 .tlb_trans_off(tlb_trans_off),
               .vpn_to_ppn_req1(vpn_to_ppn_req)
               `endif 
               );

//------------------------------------------------------------------------
//  
//   DEBUG CONTROLLER INSTANTIATION 
//
//
//-------------------------------------------------------------------------

assign data_read_dbg = proc_data_port1_int;
(* keep_hierarchy = "yes" *)
ABSTRACT_COMMAND_DECODER Abstract_Command_decoder (
  .CLK(CLK),
  .RESET(RST),
  .debug_mode(debug_mode),

  //Inputs from Debug Module
  .data0_i(data0_i),
  .data1_i(data1_i),
  .cmd_not_supported_i(cmd_not_supported_i),
  .new_cmd_flag_i(new_cmd_flag_i),

  .ptr_regno_data_o(ptr_regno_data_o),
  .ptr_data1_data_o(ptr_data1_data_o),
  .mem_access_complete_o(mem_access_complete_o),
  .reg_access_complete_o(reg_access_complete_o),
  .command_complete_o(command_complete_o),

  //Command Failure Signals to Debug Module
  .abstract_cmd_failed_o(abstract_cmd_failed_o),
  .abstract_cmd_wrong_hart_state_o(abstract_cmd_wrong_hart_state_o),
  .abstract_cmd_exception_o(abstract_cmd_exception_o),
  .abstract_cmd_bus_error_o(abstract_cmd_bus_error_o),
  
  .dcache_stall(Data_Cache__Stall),

  //CSR Signals,
  .csr_dbg_wren(csr_dbg_wren),
  .csr_dbg_addr(csr_dbg_addr),
  .csr_dbg_wdata(csr_dbg_wdata),
  .csr_dbg_rdata(csr_indata),


  //Register Signal
  .RS1_debug_addr(RS1_debug_addr),
  .Rd_debug_addr(Rd_debug_addr),
  .RS1_debug_data(RS1_Data__rf),
  .Rd_debug_data(Rd_debug_data),
  .RegWrite_debug(RegWrite_debug),
  .RegRead_debug(RegRead_debug),

  //Memory Signals
  .memory_addr_dbg(memory_addr_dbg),
  .data_write_dbg(data_write_dbg),
  .data_read_dbg(data_read_dbg),
  .Lsu_op_dbg(Lsu_op_dbg),
  .dbg_memory_req(dbg_memory_req),
  

  //Handshake signals,
  //.abs_cmd_ack(abs_cmd_ack),

  .Abstract_command(Instruction__IF_ID)
);


(* keep_hierarchy = "yes" *)
DEBUG_CONTROLLER #(.PC_BOOT()) debug_Controller (
    .clk(CLK),
    .proc_rst(RESET_BUTTON),
    .freeze(Data_Cache__Stall || Inst_Cache__Stall || Mult_Div_unit__Stall || FPU__Stall || Double_Load_Store__Stall),

    .debug_mode(debug_mode),

    // Input PC bus
    .PC_BUS({PC_MEM,PC__IF_ID,pc_cache}),

    // Sys bus from ID
    .sys(sys_id_ex),

    //====Probing State===
    .state(state),
    //Debug Module Interface 
    .haltreq_i(haltreq_i),
    .resumereq_i(resumereq_i),
    .resethaltreq_i(resethaltreq_i),
    .ackhavereset(ackhavereset),
    .ackunavail_i(ackunavail_i),
    .ndmreset_i(ndmreset_i),

    .core_havereset_o(core_havereset_o),
    .core_unavail_o(core_unavail_o),
    .core_exist_o(core_exist_o),
    .core_halted_o(core_halted_o),
    .core_running_o(core_running_o),
    .core_resumeack_o(core_resumeack_o),

    .new_cmd_flag_i(new_cmd_flag_i),
    .cmd_not_supported_i(cmd_not_supported_i),
    .command_complete_i(command_complete_o),

  //Signals to IF stage for changing PC
    .pc_valid_o(pc_valid_dbg),
    .pc_o(pc_dbg),
    .instr_req(dbg_instr_req),
    .return_pc_valid(return_pc_valid),
    .return_pc(return_pc),  

  //Output to CSR Block
    .csr_dbg_wr(csr_dbg_wr),
    .csr_dbg_cause(csr_dbg_cause),
    .csr_dbg_dpc(csr_dbg_dpc),
    .dcsr(dcsr),
    .dpc(dpc),
    .clr_step(clr_step),
    .csr_dbg_dscratch0(csr_dbg_dscratch0),
    .csr_dbg_dscratch1(csr_dbg_dscratch1),

  //Input from the DM module 
    .dm_exception_addr(dm_exception_addr),
    .dm_halt_addr(dm_halt_addr),

  //Core Warm reset 
    .core_dbg_reset(core_dbg_reset),


    .pbuffer_execution(pbuffer_execution),

  //Interrupt Signals,
    .interrupt_dbg_global_disable(interrupt_dbg_global_disable),

  //Pipeline control signals
    .halt_if(halt_if_id),
    .halt_id(halt_id_ie),
    .halt_ie(halt_ie_mem),
    .halt_mem(halt_mem_wb),
  
    .kill_if(kill_if),
    .kill_id(kill_id),
    .kill_ie(kill_ie),
    .kill_mem(kill_mem)
  
);



//------------------------------------------------------------------------
//  
//   EXCEPTION UNIT 
//
//------------------------------------------------------------------------


//TODO: Add PC and Instruction to memory stage for detection
//Also, try to push the Addr alignment thing to the EXECUTE Stage
//Verification Steps:
//  1. Check the reset values of all CSR (mtstatus, mtvec, mepc, mtval, mcasue)
//  2. Try with Async exception (Ext, Timer, Sw)
//  3. if it works, add sync exception 1 by 1 and then verify each of them 
//  4. Try to write the config of MTVEC through software 
//  5. Add ECALL and EBREAK to the system 
//  6. Try writing RO CSRs and check all these instructions CSRRW, CSRR, CSRW               

wire inst_addr_misaligned, inst_access_fault, illegal_instruction, load_misaligned, store_misaligned, inst_dec_error;
assign inst_addr_misaligned   = (pc_cache[1:0] != 2'b00);
assign illegal_instruction    = illegal_instruction_ID;

assign inst_access_fault      = 1'b0; // Is fetching Instruction on forbidden Memory region
//assign load_access_fault      = 1'b0;
//assign store_access_fault     = 1'b0;
assign load_misaligned        = badaddr_data_load;
assign store_misaligned       = badaddr_data_store;

(* keep_hierarchy = "yes" *)
EXCEPTION_UNIT EXECPTION_UNIT( 
                 .CLK(CLK),
                 .RST(RST),
                 .PC_Control__IRQ(PC_Control__IRQ),
                 .PC_Freeze_IRQ(PC_Freeze_IRQ),
                 .ID_IE_FLUSH(ID_IE_FLUSH),
                 .IF_ID_nop(IF_ID_nop),
                 .sys(sys_id_ex),
                 .IF_ID_freeze(Data_Cache__Stall || Inst_Cache__Stall || Mult_Div_unit__Stall || FPU__Stall || Double_Load_Store__Stall),
                 .pc_ctrl_ecall(pc_ctrl_ecall),
                 .pc_ctrl_mret(pc_ctrl_mret),
                 .pc_ctrl_ebreak(pc_ctrl_ebreak),
                 .PC_HALT_BREAK(PC_HALT_BREAK),
                // Exception PC and Instruction
                .PC_EXCEPTION_BUS({ pc_id_ex,
                                    PC__IF_ID,
                                    pc_cache}
                                  ),
                .INSTRUCTION_EXCEPTION_BUS({
                                            32'd0,
                                            Instruction__IF_ID,
                                            32'd0
                                            }),
                .MEM_ADDR_EXCEPTION(proc_addr_port1),
                .sync_exceptions({
                                  39'd0,
                                  inst_dec_error,
                                  8'd0,
                                  store_page_fault,
                                  1'd0,
                                  load_page_fault,
                                  instruction_page_fault,
                                  4'd0,
                                  store_access_fault,
                                  store_misaligned,
                                  load_access_fault,
                                  load_misaligned,
                                  1'b0,
                                  illegal_instruction,
                                  inst_access_fault,
                                  inst_addr_misaligned
                                  }),
                // CSR Registers 
                .csr_mstatus(csr_mstatus),
                .csr_mip(csr_mip),
                .csr_mie(csr_mie),
                .csr_mtvec(csr_mtvec),

                .Branch_Target_Addr__ex_mem(Branch_Target_Addr__ex_mem),
                .Branch_Target_Addr__EX_MEM(Branch_Target_Addr__EX_MEM),
                .Branch_Taken__ex_mem(Branch_Taken__ex_mem),
                .Branch_Taken__EX_MEM(Branch_Taken__EX_MEM),

                //CSR Interface
                .csr_ready(1'b1),
                .csr_ro_wr(1'b0),
                .mtvec_addr(mtvec_addr),
                .interrupt_pending(interrupt_pending),
                .interrupt_dbg_global_disable(interrupt_dbg_global_disable),
                .sb_csr(sb_csr)
              );

//------------------------------------------------------------------------
//  
//   DECODE UNIT 
//
//------------------------------------------------------------------------
// Synth ISSUE: Branch_Inst__id_ex is getting too early and causing synth sim
// mismatch



(* KEEP = "true" *) assign Branch_Inst__id_ex_buf1 = Branch_Inst__id_ex;
(* KEEP = "true" *) assign Branch_Inst__id_ex_buf2 = Branch_Inst__id_ex_buf1;
(* KEEP = "true" *) assign Branch_Inst__id_ex_buf3 = Branch_Inst__id_ex_buf2;
(* KEEP = "true" *) assign Branch_Inst__id_ex_buf4 = Branch_Inst__id_ex_buf3;

wire active_instruction_id;
(* keep_hierarchy = "yes" *)
DECODE ID( .CLK(CLK),                                                              
           .RST(RST),
           .IF_ID_Freeze(Inst_Cache__Stall || Mult_Div_unit__Stall || FPU__Stall || Double_Load_Store__Stall || Data_Cache__Stall || PC_HALT_BREAK),
           .Instruction__IF_ID(Instruction__IF_ID),
           .active_instruction_id(active_instruction_id),           
           .PC__IF_ID(PC__IF_ID),
           .PC_4__IF_ID(PC_4__IF_ID),
           .PC_ID(PC_ID),           
           .pc_forw(pc_forw),
           .debug_mode(debug_mode),
           .kill_id(Branch_Taken__ex_mem),

           .RS1_Addr__rf(RS1_Addr__rf),                                                     
           .RS2_Addr__rf(RS2_Addr__rf),
           .RS1_Data__rf(RS1_Data__rf),                                                      
           .RS2_Data__rf(RS2_Data__rf), 
           .Reg_Write_Enable__id_ex(Reg_Write_Enable__id_ex),                                          
           .RD_Addr__id_ex(RD_Addr__id_ex),
           .Reg_Write_Enable__ID_EX(Reg_Write_Enable__ID_EX),                                          
           .Reg_Write_Enable__EX_MEM(Reg_Write_Enable__EX_MEM),
           .RD_Addr__ID_EX(RD_Addr__ID_EX),                                                   
           .RD_Addr__EX_MEM(RD_Addr__EX_MEM),
           .Forward_RS1_MEM__id_ex(Forward_RS1_MEM__id_ex),                                           
           .Forward_RS2_MEM__id_ex(Forward_RS2_MEM__id_ex),
           .Forward_RS1_WB__id_ex(Forward_RS1_WB__id_ex),                                            
           .Forward_RS2_WB__id_ex(Forward_RS2_WB__id_ex),
           .FP__Reg_Write_En_Int__ID_EX(FP__Reg_Write_En_Int__ID_EX),
           .FP__Reg_Write_En_Int__EX_MEM(FP__Reg_Write_En_Int__EX_MEM),
           .FP__RD_Addr_Int__ID_EX(FP__RD_Addr_Int__ID_EX),
           .FP__RD_Addr_Int__EX_MEM(FP__RD_Addr_Int__EX_MEM),
           .Forward_RS1_MEM_FP__id_ex(Forward_RS1_MEM_FP__id_ex),
           .Forward_RS2_MEM_FP__id_ex(Forward_RS2_MEM_FP__id_ex),
           .Forward_RS1_WB_FP__id_ex(Forward_RS1_WB_FP__id_ex),
           .Forward_RS2_WB_FP__id_ex(Forward_RS2_WB_FP__id_ex),
           .Branch_Taken__EX_MEM(Branch_Taken__EX_MEM),                                              
           .NOP__IF_ID(NOP__IF_ID),                                                           
           .Load_Store_Op__id_ex(Load_Store_Op__id_ex),
           .Store_Data__id_ex(Store_Data__id_ex),                                                
           .Load_Store_Op__ID_EX(Load_Store_Op__ID_EX),
           .Alu_Src_1_sel__id_ex(Alu_Src_1_sel__id_ex),                                             
           .Alu_Src_2_sel__id_ex(Alu_Src_2_sel__id_ex),
           .Opcode__id_ex(Opcode__id_ex),                                                    
           .Funct7__id_ex(Funct7__id_ex),
           .Funct3__id_ex(Funct3__id_ex),                                                    
           .JAL_Inst__id_ex(JAL_Inst__id_ex),
           .JALR_Inst__id_ex(JALR_Inst__id_ex),                                                 
           .RS1_Data__id_ex(RS1_Data__id_ex),                                                                 
           .RS2_Data__id_ex(RS2_Data__id_ex),                                                  
           .Shamt__id_ex(Shamt__id_ex),                                                                 
           .Immediate__id_ex(Immediate__id_ex),                                                 
           .Branch_Inst__id_ex(Branch_Inst__id_ex),                                                                 
           .AMO_Inst__id_ex(AMO_Inst__id_ex),                                                  
           .Mult_Op__id_ex(Mult_Op__id_ex),                                                                 
           .Mult_En__id_ex(Mult_En__id_ex),                                                   
           .Div_Op__id_ex(Div_Op__id_ex),
           .Div_En__id_ex(Div_En__id_ex),                                                    
           .Load__Stall_id_ex(Load__Stall_id_ex),
           .Mult_Div_unit__Stall(Mult_Div_unit__Stall || FPU__Stall),  
           .BPU__Branch_Taken__IF_ID(BPU__Branch_Taken__IF_ID),
           .BPU__Branch_Target_Addr__IF_ID(BPU__Branch_Target_Addr__IF_ID),
           .BPU__PHT_Read_Index__IF_ID(BPU__PHT_Read_Index__IF_ID),
           .BPU__PHT_Read_Data__IF_ID(BPU__PHT_Read_Data__IF_ID), 
           .BPU__BTB_Hit__IF_ID(BPU__BTB_Hit__IF_ID),   
           .BPU__Branch_Taken__id_ex(BPU__Branch_Taken__id_ex),
           .BPU__Branch_Target_Addr__id_ex(BPU__Branch_Target_Addr__id_ex),
           .BPU__PHT_Read_Index__id_ex(BPU__PHT_Read_Index__id_ex),
           .BPU__PHT_Read_Data__id_ex(BPU__PHT_Read_Data__id_ex), 
           .BPU__BTB_Hit__id_ex(BPU__BTB_Hit__id_ex),       
           .Branch_Type__id_ex(Branch_Type__id_ex),                                         
           .lsu_op_port2(lsu_op_port2),
           .LR_Inst(LR_Inst),                                                             
           .mret(mret_int),
           .csr_adr(csr_adr_int),                                                          
           .csr_wr_en(csr_wr_en_int),
           //.mepc_res(mepc_res_int),
           .sys_id_ex(sys_id_ex),           
           //.inst_inj(inst_inj),
           //.irq_ctrl(1'b0),                                                         
           //.irq_ctrl_wb(irq_ctrl_wb_i),
           //.irq_ctrl_o(irq_ctrl_int),                                                       
           .count_sel(count_sel_int),
           .inst_dec_error(inst_dec_error),
           .illegal_instruction(illegal_instruction_ID),
           .eret(eret_int),
           .inst_out(inst_out));

(* keep_hierarchy = "yes" *) 
FP_DECODE FP__ID( .CLK(CLK),
                  .RST(RST),
                  .Instruction__IF_ID(inst_out),
                  .Branch_Taken__EX_MEM(Branch_Taken__EX_MEM),
                  .NOP__IF_ID(NOP__IF_ID),
                  .FP__RS1_Addr__rf(FP__RS1_Addr__rf),
                  .FP__RS2_Addr__rf(FP__RS2_Addr__rf),
                  .FP__RS3_Addr__rf(FP__RS3_Addr__rf),
                  .FP__RS1_Addr_Int__rf(FP__RS1_Addr_Int__rf),
                  .FP__RS1_read_Int__rf(FP__RS1_read_Int__rf),
                  .FP__RS1_Data__rf(FP__RS1_Data__rf),
                  .FP__RS2_Data__rf(FP__RS2_Data__rf),
                  .FP__RS3_Data__rf(FP__RS3_Data__rf),
                  .FP__RS1_Data_Int__rf(FP__RS1_Data_Int__rf),
                  .FP__RD_Addr__id_ex(FP__RD_Addr__id_ex),
                  .FP__Reg_Write_En__id_ex(FP__Reg_Write_En__id_ex),
                  .FP__RD_Addr_Int__id_ex(FP__RD_Addr_Int__id_ex),
                  .FP__Reg_Write_En_Int__id_ex(FP__Reg_Write_En_Int__id_ex),
                  .FP__Reg_Write_En__ID_EX(FP__Reg_Write_En__ID_EX),
                  .FP__Reg_Write_En__EX_MEM(FP__Reg_Write_En__EX_MEM),
                  .FP__RD_Addr__ID_EX(FP__RD_Addr__ID_EX),
                  .FP__RD_Addr__EX_MEM(FP__RD_Addr__EX_MEM),
                  .Reg_Write_Enable__ID_EX(Reg_Write_Enable__ID_EX),
                  .Reg_Write_Enable__EX_MEM(Reg_Write_Enable__EX_MEM),
                  .RD_Addr__ID_EX(RD_Addr__ID_EX),
                  .RD_Addr__EX_MEM(RD_Addr__EX_MEM),
                  .FP__Reg_Write_En_Int__ID_EX(FP__Reg_Write_En_Int__ID_EX),
                  .FP__Reg_Write_En_Int__EX_MEM(FP__Reg_Write_En_Int__EX_MEM),
                  .FP__RD_Addr_Int__ID_EX(FP__RD_Addr_Int__ID_EX),
                  .FP__RD_Addr_Int__EX_MEM(FP__RD_Addr_Int__EX_MEM),
                  .FP__Forward_RS1_MEM__id_ex(FP__Forward_RS1_MEM__id_ex),
                  .FP__Forward_RS2_MEM__id_ex(FP__Forward_RS2_MEM__id_ex),
                  .FP__Forward_RS3_MEM__id_ex(FP__Forward_RS3_MEM__id_ex),
                  .FP__Forward_RS1_WB__id_ex(FP__Forward_RS1_WB__id_ex),
                  .FP__Forward_RS2_WB__id_ex(FP__Forward_RS2_WB__id_ex),
                  .FP__Forward_RS3_WB__id_ex(FP__Forward_RS3_WB__id_ex),
                  .FP__Forward_RS1_MEM_Int__id_ex(FP__Forward_RS1_MEM_Int__id_ex),
                  .FP__Forward_RS1_WB_Int__id_ex(FP__Forward_RS1_WB_Int__id_ex),
                  .FP__Forward_RS1_MEM_Int_FP__id_ex(FP__Forward_RS1_MEM_Int_FP__id_ex),
                  .FP__Forward_RS1_WB_Int_FP__id_ex(FP__Forward_RS1_WB_Int_FP__id_ex),
                  .FP__Fpu_Operation__id_ex(FP__Fpu_Operation__id_ex),
                  .FP__Fpu_Sub_Op__id_ex(FP__Fpu_Sub_Op__id_ex),
                  .FP__Rounding_Mode__id_ex(FP__Rounding_Mode__id_ex),
                  .FP__SP_DP__id_ex(FP__SP_DP__id_ex),
                  .FP__Fpu_Inst__id_ex(FP__Fpu_Inst__id_ex),
                  .FP__RS1_Data__id_ex(FP__RS1_Data__id_ex),
                  .FP__RS2_Data__id_ex(FP__RS2_Data__id_ex),
                  .FP__RS3_Data__id_ex(FP__RS3_Data__id_ex),
                  .FP__Store_Data__id_ex(FP__Store_Data__id_ex),
                  .FP__Load_Inst__id_ex(FP__Load_Inst__id_ex),
                  .FP__Store_Inst__id_ex(FP__Store_Inst__id_ex)
                  //.inst_inj(inst_inj),
                  //.irq_ctrl(irq_ctrl)
                  );
//------------------------------------------------------------------------
//  
//   EXECUTE UNIT 
//
//------------------------------------------------------------------------

(* keep_hierarchy = "yes" *)           
EXECUTE EX( .CLK(CLK),                                                              
            .RST(RST),
            .kill_ie(kill_ie || illegal_instruction_IE),
            .pc_id_ex(pc_id_ex),
            .PC_ID_EX(PC_IE),
            .pc_ex_mem(pc_ex_mem),
            .Forward_RS1_MEM__ID_EX(Forward_RS1_MEM__ID_EX),
            .Forward_RS2_MEM__ID_EX(Forward_RS2_MEM__ID_EX),
            .Forward_RS1_WB__ID_EX(Forward_RS1_WB__ID_EX),
            .Forward_RS2_WB__ID_EX(Forward_RS2_WB__ID_EX),
            .RD_Data__EX_MEM(RD_Data__mem_wb),
            .RD_Data__MEM_WB(RD_Data__MEM_WB),
            .Forward_RS1_MEM_FP__ID_EX(Forward_RS1_MEM_FP__ID_EX),
            .Forward_RS2_MEM_FP__ID_EX(Forward_RS2_MEM_FP__ID_EX),
            .Forward_RS1_WB_FP__ID_EX(Forward_RS1_WB_FP__ID_EX),
            .Forward_RS2_WB_FP__ID_EX(Forward_RS2_WB_FP__ID_EX),
            .FP__RD_Data_Int__EX_MEM(FP__RD_Data_Int__EX_MEM),
            .FP__RD_Data_Int__MEM_WB(FP__RD_Data_Int__MEM_WB),
            .Load_Store_Op__ID_EX(Load_Store_Op__ID_EX),
            .Load_Store_Op__ex_mem(Load_Store_Op__ex_mem),
            .Alu_Src_1_sel__ID_EX(Alu_Src_1_sel__ID_EX),
            .Alu_Src_2_sel__ID_EX(Alu_Src_2_sel__ID_EX),
            .Opcode__ID_EX(Opcode__ID_EX),
            .Funct7__ID_EX(Funct7__ID_EX),
            .Funct3__ID_EX(Funct3__ID_EX),
            .JAL_Inst__ID_EX(JAL_Inst__ID_EX),
            .JALR_Inst__ID_EX(JALR_Inst__ID_EX),
            .RS1_Data__ID_EX(RS1_Data__ID_EX),
            .RS2_Data__ID_EX(RS2_Data__ID_EX),
            .Shamt__ID_EX(Shamt__ID_EX),
            .Immediate__ID_EX(Immediate__ID_EX),
            .Branch_Inst__ID_EX(Branch_Inst__ID_EX),
            .AMO_Inst__ID_EX(AMO_Inst__ID_EX),
            .Load__Stall_ID_EX(Load__Stall_ID_EX),
            .amo_load_val_i(amo_load_val_i),
            .Branch_Taken__ex_mem(Branch_Taken__ex_mem),
            .Branch_Target_Addr__ex_mem(Branch_Target_Addr__ex_mem),
            .Branch_Taken__EX_MEM(Branch_Taken__EX_MEM),
            .ALU_Result__ex_mem(ALU_Result__ex_mem),
            .RD_Data__ex_mem(RD_Data__ex_mem),
            .Mult_Op__ID_EX(Mult_Op__ID_EX),
            .Div_Op__ID_EX(Div_Op__ID_EX),
            .Mult_En__ID_EX(Mult_En__ID_EX),
            .Div_En__ID_EX(Div_En__ID_EX),
            .Mult_Div_unit__Stall(Mult_Div_unit__Stall),
            .Mult_Div_unit_Freeze(Mult_Div_unit_Freeze),
            .Mult_Div_unit__Stall_disable(Mult_Div_unit__Stall_disable),
            .Inst_Cache__Stall__reg(Inst_Cache__Stall__reg),
            .Inst_Cache__Stall(Inst_Cache__Stall),
            .Reg_Write_Enable__ID_EX(Reg_Write_Enable__ID_EX),
            .Reg_Write_Enable__ex_mem(Reg_Write_Enable__ex_mem),
            .RD_Addr__ID_EX(RD_Addr__ID_EX),
            .RD_Addr__ex_mem(RD_Addr__ex_mem),
            .Store_Data__ID_EX(Store_Data__ID_EX),
            .Store_Data__ex_mem(Store_Data__ex_mem),
            .BPU__Branch_Taken__ID_EX(BPU__Branch_Taken__ID_EX),
            .BPU__Branch_Target_Addr__ID_EX(BPU__Branch_Target_Addr__ID_EX),
            .BPU__PHT_Read_Index__ID_EX(BPU__PHT_Read_Index__ID_EX),
            .BPU__PHT_Read_Data__ID_EX(BPU__PHT_Read_Data__ID_EX), 
            .BPU__BTB_Hit__ID_EX(BPU__BTB_Hit__ID_EX),
            .PHT_Write_Index__ex_mem(PHT_Write_Index__ex_mem),
            .PHT_Write_Data__ex_mem(PHT_Write_Data__ex_mem),
            .PHT_Write_En__ex_mem(PHT_Write_En__ex_mem),
            .GHR_Write_Data__ex_mem(GHR_Write_Data__ex_mem),
            .GHR_Write_En__ex_mem(GHR_Write_En__ex_mem),
            .BTB_Write_Addr__ex_mem(BTB_Write_Addr__ex_mem),
            .BTB_Write_Data__ex_mem(BTB_Write_Data__ex_mem),
            .BTB_Write_En__ex_mem(BTB_Write_En__ex_mem),       
            .Branch_Type__ID_EX(Branch_Type__ID_EX), 
            .RAS_RET_Inst_EX__ex_mem(RAS_RET_Inst_EX__ex_mem),
            .RAS_CALL_Inst__ex_mem(RAS_CALL_Inst__ex_mem),
            .RAS_CALL_Inst_nextPC__ex_mem(RAS_CALL_Inst_nextPC__ex_mem),
            .lsustall_i(lsustall_i),
            .lsustall_o(lsustall_o),
            //.irq_ctrl(irq_ctrl_int2),
            .eret(eret),
            .eret_o(eret_o),
            .SC_Inst__ex_mem(SC_Inst__ex_mem),
            .trap_en(trap_en),
            .csr_indata(csr_indata /*| csr_indata_intr*/ | count),
            .csr_wrdata(csr_wrdata),
            .PC_Control__IRQ(PC_Control__IRQ));

//Removing Floating Point for ASIC Implementation
    assign FP__Store_Data__ex_mem = 64'd0;
    assign FP__Load_Inst__ex_mem = 1'b0; 
    assign FP__Store_Inst__ex_mem = 1'b0;
    assign FP__RD_Data__ex_mem = 64'd0;
    assign FP__RD_Data_Int__ex_mem = 32'd0;
    assign FP__RD_Addr__ex_mem = 5'd0;
    assign FP__Reg_Write_En__ex_mem = 1'b0;
    assign FP__RD_Addr_Int__ex_mem = 5'd0;
    assign FP__Reg_Write_En_Int__ex_mem = 1'b0;
    assign FP__SP_DP__ex_mem = 1'b0;
    assign FPU__Stall = 1'b0;
    assign FPU_flags = 5'd0;


/*FP_EXECUTE FP_EX( .RST(RST),
                  .CLK(CLK),
                  .FP__Forward_RS1_MEM__ID_EX(FP__Forward_RS1_MEM__ID_EX),
                  .FP__Forward_RS2_MEM__ID_EX(FP__Forward_RS2_MEM__ID_EX),
                  .FP__Forward_RS3_MEM__ID_EX(FP__Forward_RS3_MEM__ID_EX),
                  .FP__Forward_RS1_WB__ID_EX(FP__Forward_RS1_WB__ID_EX),
                  .FP__Forward_RS2_WB__ID_EX(FP__Forward_RS2_WB__ID_EX),
                  .FP__Forward_RS3_WB__ID_EX(FP__Forward_RS3_WB__ID_EX),
                  .FP__Forward_RS1_MEM_Int__ID_EX(FP__Forward_RS1_MEM_Int__ID_EX),
                  .FP__Forward_RS1_WB_Int__ID_EX(FP__Forward_RS1_WB_Int__ID_EX),
                  .FP__Forward_RS1_MEM_Int_FP__ID_EX(FP__Forward_RS1_MEM_Int_FP__ID_EX),
                  .FP__Forward_RS1_WB_Int_FP__ID_EX(FP__Forward_RS1_WB_Int_FP__ID_EX),
                  .FP__RD_Data__EX_MEM(FP__RD_Data__mem_wb),
                  .FP__RD_Data__MEM_WB(FP__RD_Data__MEM_WB),
                  .RD_Data__EX_MEM(FP__Reg_Write_En_Int__EX_MEM ? FP__RD_Data_Int__EX_MEM : RD_Data__mem_wb),
                  .RD_Data__MEM_WB(FP__Reg_Write_En_Int__MEM_WB ? FP__RD_Data_Int__MEM_WB : RD_Data__MEM_WB),
                  .FP__RD_Data_Int__EX_MEM(FP__RD_Data_Int__EX_MEM),
                  .FP__RD_Data_Int__MEM_WB(FP__RD_Data_Int__MEM_WB),
                  .FP__Fpu_Operation__ID_EX(FP__Fpu_Operation__ID_EX),
                  .FP__Fpu_Sub_Op__ID_EX(FP__Fpu_Sub_Op__ID_EX),
                  .FP__Rounding_Mode__ID_EX(FP__Rounding_Mode__ID_EX),
                  .FP__SP_DP__ID_EX(FP__SP_DP__ID_EX),
                  .FP__Fpu_Inst__ID_EX(FP__Fpu_Inst__ID_EX),
                  .FP__RS1_Data__ID_EX(FP__RS1_Data__ID_EX),
                  .FP__RS2_Data__ID_EX(FP__RS2_Data__ID_EX),
                  .FP__RS3_Data__ID_EX(FP__RS3_Data__ID_EX),
                  .FP__Store_Data__ID_EX(FP__Store_Data__ID_EX),
                  .FP__Store_Data__ex_mem(FP__Store_Data__ex_mem),
                  .FP__Load_Inst__ID_EX(FP__Load_Inst__ID_EX),
                  .FP__Store_Inst__ID_EX(FP__Store_Inst__ID_EX),
                  .FP__Load_Inst__ex_mem(FP__Load_Inst__ex_mem),
                  .FP__Store_Inst__ex_mem(FP__Store_Inst__ex_mem),
                  .FP__RD_Data__ex_mem(FP__RD_Data__ex_mem),
                  .FP__RD_Data_Int__ex_mem(FP__RD_Data_Int__ex_mem),
                  .Branch_Taken__EX_MEM(Branch_Taken__EX_MEM),
                  .FP__RD_Addr__ID_EX(FP__RD_Addr__ID_EX),
                  .FP__Reg_Write_En__ID_EX(FP__Reg_Write_En__ID_EX),
                  .FP__RD_Addr_Int__ID_EX(FP__RD_Addr_Int__ID_EX),
                  .FP__Reg_Write_En_Int__ID_EX(FP__Reg_Write_En_Int__ID_EX),
                  .FP__RD_Addr__ex_mem(FP__RD_Addr__ex_mem),
                  .FP__Reg_Write_En__ex_mem(FP__Reg_Write_En__ex_mem),
                  .FP__RD_Addr_Int__ex_mem(FP__RD_Addr_Int__ex_mem),
                  .FP__Reg_Write_En_Int__ex_mem(FP__Reg_Write_En_Int__ex_mem),
                  .FP__SP_DP__ex_mem(FP__SP_DP__ex_mem),
                  .FPU__Stall(FPU__Stall),
                  .FPU_Freeze(FPU_Freeze),
                  .FPU__Stall_disable(FPU__Stall_disable),
                  .Inst_Cache__Stall__reg(Inst_Cache__Stall__reg),
                  .Inst_Cache__Stall(Inst_Cache__Stall),
                  .frm(frm),
                  .FPU_flags(FPU_flags));*/

//------------------------------------------------------------------------
//  
//   REGISTER FILE 
//
//------------------------------------------------------------------------
                 
(* keep_hierarchy = "yes" *)
REG_FILE RF( .CLK(CLK),
             .RST(RST),
             .RS1_Read_Addr(RegRead_debug ? RS1_debug_addr  : (FP__RS1_read_Int__rf ? FP__RS1_Addr_Int__rf : RS1_Addr__rf)), 
             .RS2_Read_Addr(RS2_Addr__rf),
             .RD_Write_Addr(RegWrite_debug ? Rd_debug_addr  : (FP__Reg_Write_En_Int__EX_MEM ? FP__RD_Addr_Int__EX_MEM : RD_Addr__EX_MEM)),
             .RD_Write_Data(RegWrite_debug ? Rd_debug_data  : (FP__Reg_Write_En_Int__EX_MEM ? FP__RD_Data_Int__EX_MEM : RD_Data__mem_wb)),
             .Reg_Write_Enable__EX_MEM(RegWrite_debug  | (FP__Reg_Write_En_Int__EX_MEM ? FP__Reg_Write_En_Int__EX_MEM : Reg_Write_Enable__EX_MEM)),
             .MEM_WB_Freeze(Data_Cache__Stall | Double_Load_Store__Stall_reg),
             .RS1_Dec_Ctrl__IRQ(1'b0),
             .RS2_Dec_Ctrl__IRQ(1'b0),
             //.WB_Ctrl__IRQ(irq_ctrl_o),
             .WB_Ctrl__IRQ(1'b0),
             .RS1_Read_Data(RS1_Data__rf),
             .RS2_Read_Data(RS2_Data__rf),
             .led(led));
             
(* keep_hierarchy = "yes" *)             
FP_REG_FILE FP_RF( .RST(RST),
                   .CLK(CLK),
                   .FP__RS1_Read_Addr(FP__RS1_Addr__rf),
                   .FP__RS2_Read_Addr(FP__RS2_Addr__rf),
                   .FP__RS3_Read_Addr(FP__RS3_Addr__rf),
                   .FP__RD_Write_Addr(FP__RD_Addr__EX_MEM),
                   .FP__RD_Write_Data(FP__RD_Data__mem_wb),
                   .FP__Reg_Write_En__EX_MEM(FP__Reg_Write_En__EX_MEM),
                   .FP__SP_DP__EX_MEM(FP__SP_DP__EX_MEM),
                   .FP__MEM_WB_Freeze(Data_Cache__Stall | Double_Load_Store__Stall_reg),
                   .FP__RS1_Read_Data(FP__RS1_Data__rf),
                   .FP__RS2_Read_Data(FP__RS2_Data__rf),
                   .FP__RS3_Read_Data(FP__RS3_Data__rf),
                   .led());


(* keep_hierarchy = "yes" *)            
Sys_counter sc1( .rst(RST),
                 .proc_clk(CLK),
                 .freeze(Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || Double_Load_Store__Stall),
                 .count_sel(count_sel_int),
                 .count(count),
                 .wr_sel(wr_sel),
                 .csr_wrdata(csr_wrdata),
                 .csr_wr_en(csr_wr_en),
                 .tick_en(tick_en_int));
                 
//------------------------------------------------------------------------
//  
//   CSR UNIT 
//
//------------------------------------------------------------------------
                 
 (* keep_hierarchy = "yes" *)
 csr c1( .clk(CLK),
         .rst(RST),
         .csr_adr_wr(debug_mode   ? csr_dbg_addr  : csr_adr),
         .csr_wrdata(debug_mode   ? csr_dbg_wdata : csr_wrdata),
         .csr_write_en(debug_mode ? csr_dbg_wren  : csr_wr_en),
         //.csr_adr_rd(csr_adr_int),
         .csr_adr_rd(debug_mode ? csr_dbg_addr : csr_adr),
         .csr_rddata(csr_indata),
         .csr_mtvec(csr_mtvec),
         .csr_mstatus(csr_mstatus),
         .csr_mie(csr_mie),
         .csr_mip(csr_mip),
         .csr_mepc(csr_mepc),
         .sb_csr(sb_csr),
         .csr_pmp_sb(csr_pmp_sb),
         .ext_irq(ext_irq),
         .sw_irq(sw_irq),
         .timer_irq(timer_irq),
         .mtie(mtie),
         .mret(eret_o),
         .badaddr(badaddr),
         .trap_en(trap_en),
         .cache_dis(cache_dis),
         //.mepc_res(mepc_res),
         .addr_exception(addr_exception),
         .freeze(Mult_Div_unit__Stall || FPU__Stall || Data_Cache__Stall || Inst_Cache__Stall || Double_Load_Store__Stall || PC_HALT_BREAK),
         .FPU_Inst(FP__Fpu_Inst__ID_EX & ~FPU__Stall),
         .FPU_flags(FPU_flags),
         .frm(frm),
         //Debug Interface
         .csr_dbg_wr(csr_dbg_wr),
         .csr_dbg_cause(csr_dbg_cause),
         .csr_dbg_dpc(csr_dbg_dpc),
         .dcsr(dcsr),
         .dpc(dpc),
         .clr_step(clr_step),
         .debug_mode(debug_mode),
         .csr_dbg_dscratch0(csr_dbg_dscratch0),
         .csr_dbg_dscratch1(csr_dbg_dscratch1),
         .cache_flush_csr(cache_flush_csr),
         .flush_csr_clr(flush_csr_clr),
         .csr_satp(csr_satp),
         .pc_id_ex(pc_id_ex));
        

 // ILA Debugger
 
  wire [31:0] REG_1, REG_17;

  assign REG_17 = led[31:0]; 
  assign REG_1 = led[63:32]; 
  /*ila_0 debugger (
                    .clk(CLK),
                    .probe0(PC__IF_ID),
                    .probe1(Instruction__IF_ID),
                    .probe2(pc_cache),
                    .probe3(REG_17),
                    .probe4(REG_1));*/

/*ila_2 debugger( .clk(CLK),
                .probe0(pc_cache),
                .probe1(Instruction__IF_ID),
                .probe2(RD_Addr__EX_MEM),
                .probe3(RD_Data__mem_wb),
                .probe4(Reg_Write_Enable__EX_MEM),
                .probe5(Inst_Cache__Stall),
                .probe6(Data_Cache__Stall),
                .probe7(Mult_Div_unit__Stall),
                .probe8(FPU__Stall),
                .probe9(Load_Store_Op__Port1),
                .probe10(proc_addr_port1),
                .probe11(Store_Data),
                .probe12(proc_data_port1_int),
                .probe13(Double_Load_Store__Stall));*/
                
//ila_0 debugger( .clk(CLK),
//                .probe0(pc_cache),
//                .probe1(Instruction__IF_ID),
//                .probe2(FP__RD_Addr__EX_MEM),
//                .probe3(FP__RD_Data__mem_wb),
//                .probe4(FP__Reg_Write_En__EX_MEM),
//                .probe5(FP__Reg_Write_En_Int__EX_MEM),
//                .probe6(FP__RD_Addr_Int__EX_MEM),
//                .probe7(FP__RD_Data_Int__EX_MEM),
//                .probe8(FP__Reg_Write_En_Int__EX_MEM));


//reg [31:0] Counter__1,Counter__2,Counter__3,Counter__4,Counter__5,Counter__6;  

//ila_0 debugger( .clk(CLK),
//                .probe0(pc_cache),
//                .probe1(Instruction__IF_ID),
//                .probe2(Counter__1),
//                .probe3(Counter__2),
//                .probe4(Counter__3),
//                .probe5(Counter__4),
//                .probe6(Counter__5),
//                .probe7(Counter__6));

//ila_0 debugger( .clk(CLK),
//                .probe0(pc_cache),
//                .probe1(Instruction__IF_ID));



//always @(posedge CLK) begin
//    if (RST) begin
//        Counter__1 = 32'b0;
//        Counter__2 = 32'b0;
//    end
//    else if (~(Mult_Div_unit__Stall | FPU__Stall | Data_Cache__Stall | Inst_Cache__Stall | Double_Load_Store__Stall | Branch_Taken__EX_MEM)) begin
//        if (Branch_Inst__ID_EX == 1'b1) begin
//            Counter__1 = Counter__1 + 1;
//            if (Branch_Taken__ex_mem == 1'b1) begin
//                Counter__2 = Counter__2 + 1;
//            end
//        end
//    end 
//end     

//always @(posedge CLK) begin
//    if (RST) begin
//        Counter__3 = 32'b0;
//        Counter__4 = 32'b0;
//    end
//    else if (~(Mult_Div_unit__Stall | FPU__Stall | Data_Cache__Stall | Inst_Cache__Stall | Double_Load_Store__Stall | Branch_Taken__EX_MEM)) begin
//        if (((JAL_Inst__ID_EX == 1'b1) || (JALR_Inst__ID_EX == 1'b1)) && ((Branch_Type__ID_EX == 3'b001) || (Branch_Type__ID_EX == 3'b010))) begin
//            Counter__3 = Counter__3 + 1;
//            if (Branch_Taken__ex_mem == 1'b1) begin
//                Counter__4 = Counter__4 + 1;
//            end
//        end
//    end 
//end     

//always @(posedge CLK) begin
//    if (RST) begin
//        Counter__5 = 32'b0;
//        Counter__6 = 32'b0;
//    end
//    else if (~(Mult_Div_unit__Stall | FPU__Stall | Data_Cache__Stall | Inst_Cache__Stall | Double_Load_Store__Stall | Branch_Taken__EX_MEM)) begin
//        if ((JALR_Inst__ID_EX == 1'b1) && (Branch_Type__ID_EX == 3'b011)) begin
//            Counter__5 = Counter__5 + 1;
//            if (Branch_Taken__ex_mem == 1'b1) begin
//                Counter__6 = Counter__6 + 1;
//            end
//        end
//    end 
//end     

                               
endmodule















