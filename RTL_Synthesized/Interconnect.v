//Designed By: Sanidhya Saxena
//Project: 32-bit RISCV Design 
//Guide: Kuruvilla Varghese
//Students: Sanidhya Saxena, Toms Jiji Varghese

`timescale 1ns / 1ps


module interconnect (
    input  wire         clk,              // Clock signal
    input  wire         reset,            // Reset signal

    // Instruction Bus Interface
    input  wire         instr_req,        // Instruction request signal
    input  wire [31:0]  instr_addr,       // Instruction address bus
    input  wire [1:0]   instr_burst,
    input  wire         instr_write,
    input  wire [31:0]  instr_write_data,
    input  wire [3:0]   instr_bstrobe, 

    output   [31:0]  instr_data,       // Instruction read data
    output           instr_ready,      // Instruction bus ready signal
    output           instr_stall,

    // Data Bus Interface
    input  wire         data_req,         // Data request signal
    input  wire         data_write,       // Data write enable
    input  wire [31:0]  data_addr,        // Data address bus
    input  wire [31:0]  data_write_data,  // Data write data
    input  wire [1:0]   data_burst,
    input  wire [3:0]   data_bstrobe,

    output   [31:0]  data_read_data,   // Data read data
    output           data_ready,       // Data bus ready signal
    output           data_stall,

    // Memory Interface (Output Interface to Memory)
    output reg          mem_req,          // Memory request signal
    output reg          mem_write,        // Memory write enable
    output reg  [31:0]  mem_addr,         // Memory address
    output reg  [1:0]   mem_burst,
    output reg  [3:0]   mem_bstrobe,
    output reg  [31:0]  mem_write_data,   // Memory write data

    input  wire [31:0]  mem_read_data,    // Memory read data
    input  wire         mem_stall,
    input  wire         mem_ready         // Memory ready signal
);

    // Arbitration logic: current bus granted access
    reg current_access; // 0 = instruction bus, 1 = data bus

    // Memory arbitration logic
    always @(*) begin
        if (reset) begin
            current_access <= 1'b0;
            mem_req        <= 1'b0;
        end else begin
                // Grant access based on priority: Data bus > instruction bus
                if (data_req) begin
                    current_access  <= 1'b1; // Data bus
                    mem_req         <= 1'b1;
                    mem_addr        <= data_addr;
                    mem_write       <= data_write;
                    mem_write_data  <= data_write_data;
                    mem_burst       <= data_burst;
                    mem_bstrobe      <= data_bstrobe;
                end
                else if (instr_req) begin
                    current_access  <= 1'b0; // Instruction bus
                    mem_req         <= 1'b1;
                    mem_addr        <= instr_addr;
                    mem_write       <= 1'b0; // Instruction bus is read-only
                    mem_write_data  <= instr_write_data;
                    mem_burst       <= instr_burst;
                    mem_bstrobe     <= instr_bstrobe;
                end
                else begin
                    current_access  <= 1'b0;
                    mem_req         <= 1'b0;
                    mem_addr        <= 32'd0;
                    mem_write       <= 1'b0; // Instruction bus is read-only
                    mem_write_data  <= 32'd0;
                    mem_burst       <= 2'd0;
                    mem_bstrobe     <= 4'd0;
                end
        end
    end
    
    
    assign instr_ready = (current_access == 1'b0) & mem_ready;
    assign instr_data  = (current_access == 1'b0) ? mem_read_data : 32'd0;
    assign instr_stall = (current_access == 1'b0) & mem_stall;
    
    assign data_ready = (current_access == 1'b1) & mem_ready;
    assign data_read_data  = (current_access == 1'b1) ? mem_read_data : 32'd0;
    assign data_stall = (current_access == 1'b1) & mem_stall;

endmodule

