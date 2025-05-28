`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/22/2025 05:17:16 PM
// Design Name: 
// Module Name: RISCV_Core
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


module RISCV_Core(
    input  wire clk,
    input  wire rst_n,

    // From Debug Module to Core
    input  wire         haltreq,
    input  wire         resumereq,
    input  wire         hartreset,
    input  wire         ackhavereset,
    input  wire         ackunavail,
    input  wire         resethaltreq,
    input  wire         ndmreset,
    input  wire [31:0]  data0,
    input  wire [31:0]  data1,
    input  wire [31:0]  instr_bus,
    input  wire         new_cmd_flag,
    input  wire         cmd_not_supported,

    // From Core to Debug Module
    output wire [31:0]  ptr_regno_data,
    output wire [31:0]  ptr_data1_data,
    output wire         command_complete,
    output wire         reg_access_complete,
    output wire         mem_access_complete,
    output reg          core_halted,
    output wire         core_running,
    output wire         core_unavail,
    output wire         core_exist,
    output wire         core_resumeack,
    output wire         core_havereset,
    output wire         abstract_cmd_failed,
    output wire         abstract_cmd_wrong_hart_state,
    output wire         abstract_cmd_exception,
    output wire         abstract_cmd_bus_error,
    output wire [7:0]   PC_debug,
    
    //==========Testing==========  
    output wire [31:0] dpc,
    output wire [31:0] R4  
    );
    
    reg kill;
    reg core_rst_n;
    wire [31:0] dcsr;
    wire [31:0] EX_MEM_PC;
    
    wire DBG_GPR_Write;
    wire DBG_CSR_Write;
    wire [31:0] DBG_Reg_Write_Data;
    wire [4:0] DBG_W_Addr_3;
    wire [31:0] DBG_MEM_Write_data;
    wire        DBG_MEM_Write;
    wire [2:0] DBG_aamsize;
    wire step;
    
    // === Internal Wires for Command_Engine ===
    wire [4:0]  reg_addr;
    wire [11:0] csr_addr;
    wire [31:0] mem_addr;
    
    wire [31:0] CSR_data = (csr_addr == 12'h7b0) ? dcsr :
                           (csr_addr == 12'h7b1) ? dpc : 0; 
    wire [31:0] REG_data;
    wire [31:0] MEM_data; 
    
//    assign core_halted  = (kill==1) && !step;
    assign core_running = !core_halted;
//    assign core_unavail = (core_rst_n==0);
    assign core_unavail = 0;
    assign core_exist = 1'b1;
    assign core_resumeack = (kill==0);
    assign core_havereset = (core_rst_n==0);
    
    assign PC_debug = 8'h17;
    
    
    always@(posedge clk) begin
        if(!rst_n)begin
            core_rst_n <= 0;    //reset core as well
            kill <= 0;
            core_halted <=0;
        end
        else begin
            core_halted  <= (kill==1) && !step;
            if(hartreset==0 ||  ndmreset==0 || rst_n==0)
                core_rst_n <= 0;
            else
                core_rst_n <= 1;  
                
                
            if(haltreq || resethaltreq)
                kill <= 1;
            else if (resumereq)
                kill <= 0;
        end
    end
    
    RISC_V_Pipelined u_riscv (
        .clk(clk),
        .rst_n(core_rst_n),
        .kill(kill && !step),
        .EX_MEM_PC(EX_MEM_PC),
        .dpc(dpc),
        
        .DBG_Reg_Write(DBG_GPR_Write),
        .DBG_Reg_Write_Data(DBG_Reg_Write_Data),
        .DBG_W_Addr_3(reg_addr),
        
        .R4(R4),
        .R_Addr_3(reg_addr),
        .Read_Data_2(REG_data),
    
        .DBG_MEM_Addr(mem_addr),
        .DBG_aamsize(DBG_aamsize),
        .DBG_MEM_Read_data(MEM_data),
        .DBG_MEM_Write(DBG_MEM_Write),
        .DBG_MEM_Write_data(DBG_MEM_Write_data)
    );
    
    
    // Instantiation of Debug_Core_Controller
    Debug_Core_Controller debug_core_ctrl_inst (
        .clk        (clk),          // Clock input
        .rst_n      (core_rst_n),        // Active-low reset
        .kill       (kill && !step),         // Kill signal input
        .EX_MEM_PC  (EX_MEM_PC),    // 32-bit program counter input from EX/MEM stage
        .DBG_CSR_Write(DBG_CSR_Write),
        .DBG_Reg_Write_Data(DBG_Reg_Write_Data),
        .DBG_W_Addr_3(csr_addr),
        
        .haltreq(haltreq),
        .resethaltreq(resethaltreq),
        .core_halted(core_halted),
        
        .step       (step),
        .dpc        (dpc),           // 32-bit debug program counter output
        .dcsr       (dcsr)
    );


    Command_Engine u_Command_Engine (
        .clk(clk),
        .rst_n(core_rst_n),
        
        .data0(data0),
        .data1(data1),
        .cmd(instr_bus),
        .core_halted(core_halted),
        .core_running(core_running),
        
        .new_cmd_flag(new_cmd_flag),
        .cmd_not_supported(cmd_not_supported),
    
        .ptr_regno_data(ptr_regno_data),
        .ptr_data1_data(ptr_data1_data),
    
        .command_complete(command_complete),
        .reg_access_complete(reg_access_complete),
        .mem_access_complete(mem_access_complete),
    
        .abstract_cmd_failed(abstract_cmd_failed),
        .abstract_cmd_wrong_hart_state(abstract_cmd_wrong_hart_state),
        .abstract_cmd_exception(abstract_cmd_exception),
        .abstract_cmd_bus_error(abstract_cmd_bus_error),
    
        .reg_addr(reg_addr),
        .csr_addr(csr_addr),
        .mem_addr(mem_addr),
        .DBG_aamsize(DBG_aamsize),
    
        .DBG_GPR_Write(DBG_GPR_Write),
        .DBG_CSR_Write(DBG_CSR_Write),
        .DBG_Reg_Write_Data(DBG_Reg_Write_Data),
        .DBG_MEM_Write(DBG_MEM_Write),
        .DBG_MEM_Write_data(DBG_MEM_Write_data),
    
        .CSR_data(CSR_data),
        .REG_data(REG_data),
        .MEM_data(MEM_data)
    );
endmodule