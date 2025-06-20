(* keep_hierarchy = "yes" *)
module Main_Memory_Wrapper (
    input         clk,
    input         reset,

    // Instruction Bus Interface
    input         instr_req,
    input         instr_write,
    input  [31:0] instr_addr,
    input  [1:0]  instr_burst,
    input  [3:0]  instr_bstrobe,
    input  [31:0] instr_write_data,
    output [31:0] instr_data,
    output        instr_ready,
    output        instr_stall,
    output        instr_tlast,
    input imem_read_execute_allowed,

    // Data Bus Interface
    input dmem_read_write_allowed,
    input         data_req,
    input         data_write,
    input  [31:0] data_addr,
    input  [1:0]  data_burst,
    input  [3:0]  data_bstrobe,
    input  [31:0] data_write_data,
    output [31:0] data_read_data,
    output        data_ready,
    output        data_stall,
    output        data_tlast
);

    // Internal wires between interconnect and memory
    wire         mem_req;
    wire         mem_write;
    wire [31:0]  mem_addr;
    wire [1:0]   mem_burst;
    wire [3:0]   mem_bstrobe;
    wire [31:0]  mem_write_data;
    wire [31:0]  mem_read_data;
    wire         mem_ready;
    wire         mem_stall;
    wire         mem_tlast;

    // Instantiate interconnect
    interconnect interconnect_inst (
        .clk(clk),
        .reset(reset),

        // Instruction bus
        .instr_req(instr_req & imem_read_execute_allowed),
        .instr_addr(instr_addr),
        .instr_burst(instr_burst),
        .instr_write(instr_write),
        .instr_write_data(instr_write_data),
        .instr_bstrobe(instr_bstrobe),
        .instr_data(instr_data),
        .instr_ready(instr_ready),
        .instr_stall(instr_stall),
        .instr_tlast(instr_tlast),

        // Data bus
        .data_req(data_req & dmem_read_write_allowed),
        .data_write(data_write),
        .data_addr(data_addr),
        .data_write_data(data_write_data),
        .data_burst(data_burst),
        .data_bstrobe(data_bstrobe),
        .data_read_data(data_read_data),
        .data_ready(data_ready),
        .data_stall(data_stall),
        .data_tlast(data_tlast),

        // Memory interface (to Main Memory)
        .mem_req(mem_req),
        .mem_write(mem_write),
        .mem_addr(mem_addr),
        .mem_burst(mem_burst),
        .mem_bstrobe(mem_bstrobe),
        .mem_write_data(mem_write_data),
        .mem_read_data(mem_read_data),
        .mem_ready(mem_ready),
        .mem_stall(mem_stall),
        .mem_tlast(mem_tlast)
    );

    // Instantiate Main Memory
    (* keep_hierarchy = "yes" *)
    Main_Memory #(
        .ADDR_WIDTH(14),
        .DATA_WIDTH(32),
        .INSTR_INPUT_FILE("uart.mem"),
        .DATA_INPUT_FILE("helloworld_data.mem")
    ) Instruction_Memory (
        .clk(clk),
        .rst(reset),
        .ADDR(mem_addr),
        .REQ(mem_req),
        .WRB(mem_write),
        .BURST(mem_burst),
        .BSTROBE(mem_bstrobe),
        .WDATA(mem_write_data),
        .RDATA(mem_read_data),
        .ACK(mem_ready),
        .STALL(mem_stall),
        .TLAST(mem_tlast)
    );

endmodule

