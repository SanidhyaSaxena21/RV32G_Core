//Designer: Sanidhya Saxena
//Guide: Kuruvilla Varghese


module ABSTRACT_COMMAND_DECODER (
  input CLK,
  input RESET,
  input debug_mode,

  input [31:0] data0_i,
  input [31:0] data1_i,
  input cmd_not_supported_i,
  input new_cmd_flag_i,

  output reg [31:0] ptr_data1_data_o,
  output reg [31:0] ptr_regno_data_o,
  
  output reg mem_access_complete_o,
  output reg reg_access_complete_o,
  output  command_complete_o,

  output  abstract_cmd_failed_o,
  output  abstract_cmd_wrong_hart_state_o,
  output  abstract_cmd_exception_o,
  output  abstract_cmd_bus_error_o,

  //CSR Singals
  output reg csr_dbg_wren,
  output reg [11:0] csr_dbg_addr,
  output reg [31:0] csr_dbg_wdata,
  input [31:0] csr_dbg_rdata,

  //Register Signals,
  output [4:0] RS1_debug_addr,
  input [31:0] RS1_debug_data,
  output [4:0] Rd_debug_addr,
  output [31:0] Rd_debug_data,

  output RegWrite_debug,
  output RegRead_debug,

  //Memory Signals,
  output reg [31:0] memory_addr_dbg,
  output reg [31:0] data_write_dbg,
  input [31:0] data_read_dbg,
  output reg [4:0] Lsu_op_dbg,
  output dbg_memory_req,

  input dcache_stall,



  input [31:0] Abstract_command
);

// DEBUG ABSTRACT COMMAND DECODING

wire [7:0] cmdtype;
wire transfer;
wire write;
wire [15:0] regno;
wire [2:0] aamsize;

reg RegWrite_debug_q,RegRead_debug_q;

reg [4:0] RS1_debug_addr_q,Rd_debug_addr_q;

reg [31:0] Rd_debug_data_q;
wire new_cmd_flag_pulse;
wire cmd_not_supported_i_pulse;
reg new_cmd_flag_reg;
reg cmd_not_supported_reg;

reg [31:0] memory_addr_dbg_q;
reg dbg_memory_req_q;
reg dbg_memory_req_i;

reg csr_dbg_ren;

assign dbg_memory_req = dbg_memory_req_q | dbg_memory_req_i;
//Abstract Command Decoding
assign cmdtype  = debug_mode ? (Abstract_command[31:24]) : 8'dx;
assign transfer = debug_mode ? (Abstract_command[17]) : 1'b0;
assign write    = debug_mode ? (Abstract_command[16]) : 1'b0;
assign regno    = debug_mode ? (Abstract_command[15:0]) : 16'd0;
assign aamsize  = debug_mode ? (Abstract_command[22:20]) : 3'd0;

assign RegWrite_debug   = RegWrite_debug_q; 
assign RegRead_debug    = RegRead_debug_q;
assign RS1_debug_addr   = RS1_debug_addr_q;
assign Rd_debug_addr    = Rd_debug_addr_q;
assign Rd_debug_data    = Rd_debug_data_q;
//assign ptr_regno_data_o = RS1_debug_data;

assign abstract_cmd_failed_o = 1'b0;
assign abstract_cmd_wrong_hart_state_o = 1'b0;
assign abstract_cmd_exception_o = 1'b0;
assign abstract_cmd_bus_error_o = 1'b0;


assign command_complete_o = reg_access_complete_o | mem_access_complete_o;
//Register Read/Write command acknowledgement
always @(posedge CLK) begin
  if(RESET) begin
     reg_access_complete_o <= 1'b0;
     ptr_regno_data_o    <= 32'd0;
  end
  else if((RegRead_debug_q | RegWrite_debug_q | csr_dbg_wren | csr_dbg_ren)) begin
     reg_access_complete_o <= 1'b1;
     ptr_regno_data_o    <= (RegRead_debug_q) ? RS1_debug_data :
                            (csr_dbg_ren)     ? csr_dbg_rdata  : 32'd0;
  end
  else if(reg_access_complete_o & ~new_cmd_flag_reg) begin
    reg_access_complete_o <= 1'b0;
    ptr_regno_data_o      <= 32'd0;
  end
end



always @(posedge CLK) begin
  if(RESET) begin
    mem_access_complete_o <= 1'b0;
    ptr_data1_data_o      <= 32'd0;
  end
  else if(dbg_memory_req_q & ~dcache_stall) begin
    mem_access_complete_o <= 1'b1;
    ptr_data1_data_o <= data_read_dbg; 
  end
  else if(mem_access_complete_o & ~new_cmd_flag_reg) begin
    mem_access_complete_o <= 1'b0;
    ptr_data1_data_o <= 32'd0;
  end
end

//Pulse of new_cmd_flag_i


assign new_cmd_flag_pulse = new_cmd_flag_i & ~new_cmd_flag_reg;
assign cmd_not_supported_i_pulse = cmd_not_supported_i & ~cmd_not_supported_reg;


always @(posedge CLK or posedge RESET) begin
  if(RESET) begin
    new_cmd_flag_reg <= 1'b0;
    cmd_not_supported_reg <= 1'b0;
  end
  else begin
    new_cmd_flag_reg <= new_cmd_flag_i;
    cmd_not_supported_reg <= cmd_not_supported_i; 
  end
end

//For Data Memory Read/Write, we need to latch the address for next cycle and
//then check if we have a dcache stall. If not, we'll get the data in next cycle


always @(posedge CLK) begin
  if(RESET) begin
    memory_addr_dbg_q   <= 32'd0;
    dbg_memory_req_q    <= 1'b0;
  end
  else if(~dcache_stall) begin
    memory_addr_dbg_q   <= memory_addr_dbg;
    dbg_memory_req_q    <= dbg_memory_req_i;
  end
end

always @(posedge CLK) begin
  if(RESET) begin
    RegWrite_debug_q  <= 1'b0;
    RegRead_debug_q   <= 1'b0;  
    RS1_debug_addr_q  <= 5'd0;
    Rd_debug_addr_q     <= 5'd0;
    Rd_debug_data_q   <= 32'd0;
    Lsu_op_dbg        <= 5'd0;
    memory_addr_dbg   <= 32'd0;
    data_write_dbg    <= 32'd0;
    dbg_memory_req_i  <= 1'b0;
    csr_dbg_wren      <= 1'b0;
    csr_dbg_ren      <= 1'b0;
    csr_dbg_addr      <= 12'd0;
    csr_dbg_wdata     <= 32'd0;
  end
  else if(new_cmd_flag_reg & ~cmd_not_supported_i_pulse) begin
    case(cmdtype)
      8'd0: begin
        RegWrite_debug_q  <= (debug_mode & transfer & write & regno[12]);
        RegRead_debug_q   <= (debug_mode & transfer & ~write & regno[12]);
        csr_dbg_wren      <= (debug_mode & transfer & write & ~regno[12]);
        csr_dbg_ren       <= (debug_mode & transfer & ~write & ~regno[12]);
        csr_dbg_addr      <= regno[11:0];
        csr_dbg_wdata     <= data0_i;
        RS1_debug_addr_q  <= regno[4:0];
        Rd_debug_addr_q   <= regno[4:0];
        Rd_debug_data_q   <= data0_i;
      end
      8'd2: begin
        data_write_dbg <= data0_i;
        memory_addr_dbg <= data1_i;
        dbg_memory_req_i <= 1'b1;
        case({write,aamsize})
          4'd0: Lsu_op_dbg <= 5'b00001;
          4'd1: Lsu_op_dbg <= 5'b00101;
          4'd2: Lsu_op_dbg <= 5'b01001;
          4'd8: Lsu_op_dbg <= 5'b00010;
          4'd9: Lsu_op_dbg <= 5'b00110;
          4'd10: Lsu_op_dbg <= 5'b01010;
          default: Lsu_op_dbg <= 5'b00000;
        endcase
      end
    endcase
  end
  else begin
    RegWrite_debug_q  <= 1'b0;
    RegRead_debug_q   <= 1'b0;  
    RS1_debug_addr_q  <= 5'd0;
    Rd_debug_addr_q     <= 5'd0;
    Rd_debug_data_q   <= 32'd0;  
    dbg_memory_req_i   <= 1'b0;
    Lsu_op_dbg <= 5'd0;
    memory_addr_dbg <= 32'd0;
    data_write_dbg <= 32'd0;
    csr_dbg_wren <= 1'b0;
    csr_dbg_addr <= 12'd0;
    csr_dbg_ren <= 1'b0;

     
  end
end



endmodule
