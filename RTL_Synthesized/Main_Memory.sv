// Designer: Sanidhya Saxena
// Guide: Kuruvilla Varghese

module Main_Memory #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter BURST_LEN  = 8,
    parameter INSTR_INPUT_FILE = "Instruction.mem",
    parameter DATA_INPUT_FILE = "Data.mem",
    parameter BASE_ADDR_INSTRUCTION = 32'h00000000,
    parameter INSTRUCTION_MASK      = 32'h00003FFF,
    parameter BASE_ADDR_DATA        = 32'h20000000,
    parameter DATA_MASK             = 32'h00003FFF,
    parameter BASE_ADDR_PAGE_TABLE  = 32'h00004000,
    parameter BASE_ADDR_HANDLER     = 32'h00008000
)(
    input                      clk,
    input                      rst,
    input       [31:0]         ADDR,
    input                      REQ,
    input                      WRB,
    input      [DATA_WIDTH-1:0] WDATA,
    output     [DATA_WIDTH-1:0] RDATA,
    output reg                 ACK,
    input       [1:0]          BURST,
    input       [3:0]          BSTROBE,
    output                     STALL,
    output                  TLAST
);

    // FSM States
    typedef enum logic [1:0] {
        IDLE  = 2'b00,
        TRANS = 2'b01,
        LAST  = 2'b10
    } state_t;

    state_t state, next_state;

    reg [2:0] burst_count;
    reg [ADDR_WIDTH-1:0] addr_reg;


    // Instruction region check
    wire instr_range  = ((ADDR & ~INSTRUCTION_MASK) == BASE_ADDR_INSTRUCTION) && (state == TRANS | state == LAST) ;
    wire data_req     = ((ADDR & ~DATA_MASK) == BASE_ADDR_DATA              ) && (state == TRANS | state == LAST) ;
    wire pt_req       = ((ADDR & ~INSTRUCTION_MASK) == BASE_ADDR_PAGE_TABLE ) && (state == TRANS | state == LAST) ;
    wire clint_req    = ((ADDR & ~INSTRUCTION_MASK) == BASE_ADDR_HANDLER    ) && (state == TRANS | state == LAST) ;


    wire [ADDR_WIDTH-1:0] ADDR_INSTR = addr_reg;
    wire [ADDR_WIDTH-1:0] ADDR_DATA = addr_reg;
    wire [ADDR_WIDTH-1:0] ADDR_PT = addr_reg;
    wire [ADDR_WIDTH-1:0] ADDR_HANDLER = addr_reg;

    wire [DATA_WIDTH-1:0] IRDATA;
    wire [DATA_WIDTH-1:0] DRDATA;
    wire [DATA_WIDTH-1:0] PRDATA;
    wire [DATA_WIDTH-1:0] HRDATA;

    assign RDATA = (instr_range & REQ)  ? IRDATA :
                   (data_req & REQ)     ? DRDATA :
                   (pt_req & REQ)       ? PRDATA :
                   (clint_req & REQ)    ? HRDATA :
                    32'd0;         

    // Memory macro instantiation (handles real memory)
    /*MEMORY_MACRO #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .INPUT_FILE(INSTR_INPUT_FILE)
    ) Instruction_Memory (
        .clka(clk),
        .rsta(rst),
        .byte_en(BSTROBE),
        .ena(instr_range && REQ),
        .wea(1'b0),
        .addra(ADDR_INSTR & 10'h3ff),
        .dina(1'b0),
        .douta(IRDATA)
    );*/

      Instruction_Memory Instruction_Memory(
       .clka(clk), // input clka
       .rsta(rst),      // reset
       .wea(4'b0000),
       .ena(instr_range && REQ), // input ena
       .addra(ADDR_INSTR[11:2] & 10'h3ff), // input [31 : 0] addra
       .dina(32'd0), // input [31 : 0] dina
       .douta(IRDATA) // output [31 : 0] douta
     );
    
      MEMORY_MACRO #(.ADDR_WIDTH(ADDR_WIDTH),.DATA_WIDTH(DATA_WIDTH),.INPUT_FILE(DATA_INPUT_FILE)) Data_Memory(
        .clka(clk), // input clka
        .rsta(rst),      // reset
        .byte_en(BSTROBE),
        .ena(data_req && REQ), // input ena
        .wea(WRB & data_req & REQ), // input [3 : 0] wea
        .addra(ADDR_DATA & 32'h00003fff), // input [31 : 0] addra
        .dina(WDATA), // input [31 : 0] dina
        .douta(DRDATA) // output [31 : 0] douta
      );
    
       Page_Table Page_Table_memory(
        .clka(clk), // input clka
        .rsta(rst),      // reset
        .wea((BSTROBE & {4{WRB}})),
        .ena(pt_req && REQ), // input ena
        .addra(ADDR_PT[11:2] & 10'h3ff), // input [31 : 0] addra
        .dina(WDATA), // input [31 : 0] dina
        .douta(PRDATA) // output [31 : 0] douta
      );
      
       Handler_Memory handler_memory(
        .clka(clk), // input clka
        .rsta(rst),      // reset
        .wea((BSTROBE & {4{WRB}})),
        .ena(clint_req && REQ), // input ena
        .addra(ADDR_HANDLER[11:2] & 10'h3ff), // input [31 : 0] addra
        .dina(WDATA), // input [31 : 0] dina
        .douta(HRDATA) // output [31 : 0] douta
      );

    
    // FSM Sequential
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    // FSM Combinational
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin 
              if (REQ) next_state = TRANS; //Burst transfer
            end
            TRANS: begin 
              if (BURST == 2'b01 && burst_count == BURST_LEN - 1) next_state = LAST;
              else if(BURST == 2'b00) next_state = LAST;
            end
            LAST:  next_state = IDLE;
        endcase
    end

    // Control Logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ACK         <= 0;
            //TLAST       <= 0;
            burst_count <= 0;
            addr_reg    <= 0;
        end else begin
            case (state)
                IDLE: begin
                    ACK         <= 1'b0; 
                                   
                    //TLAST       <= 0;
                    burst_count <= 0;
                    if (REQ) begin
                        addr_reg <= ADDR;
                        //ACK      <= 1;
                    end
                end
                TRANS: begin
                    ACK <= 1;
                    if(!WRB) begin
                        addr_reg    <= addr_reg + 4;  // Increment by word (4 bytes)
                    end
                    else if(WRB & ACK) begin
                        addr_reg    <= addr_reg + 4;
                    end
                    else addr_reg <= ADDR;
                    
                    burst_count <= (BURST == 2'b01) ? (burst_count + 1) : burst_count;
                end
                LAST: begin
                    //TLAST <= 1;
                    ACK   <= 1'b0;
                end
            endcase
        end
    end
    
    assign TLAST = (state == LAST);

    assign STALL = 1'b0;

endmodule

