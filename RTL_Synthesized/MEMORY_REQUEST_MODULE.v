//Designer: Sanidhya Saxena
//Date: 5th May 2025
//Guide: Kuruvilla varghese

module MEMORY_REQUEST_MODULE (
   // Global Signals
   input clk,
   input reset,

   // Internal RISC interface
   input  [32-1:0] 			biu_adr_i,	  // address bus
   input				        biu_cyc_i,	  // WB cycle
   input				        biu_stb_i,	  // WB strobe
   input				        biu_we_i,	    // WB write enable
   input				        biu_cab_i,	  // CAB input
   input  [3:0] 				biu_sel_i,	  // byte selects
   output [31:0] 			  	biu_dat_o,	  // output data bus
   output reg           			bus_rdy,      //interface to the dcache unit
   //inout  [255:0]       			bus_data,     //to be able to communicate with proc interface 
   output  [255:0]       			o_bus_data,     //Give data back to processor Memory unit (ld) 
   input  [255:0]       			i_bus_data,     //Give data back to Main Memory (sw) 
  
   input                			peripheral_access, 
   //Memory Interface
   output reg [31:0]  ADDR,
   output reg [1:0]   BURST, //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
   output reg         REQ,
   output reg         WRB,
   output reg [31:0]  WDATA,
   input      [31:0]  RDATA,
   input              ACK,
   input              TLAST,
   input              STALL,
   output reg [3:0]   BSTROBE,
  
   //Control Signals
   input freeze

);

reg [2:0] counter;
reg [255:0] bus_reg;
reg [2:0] write_counter;

assign o_bus_data = bus_reg;

always @(posedge clk) begin
  if(reset) begin
    ADDR    <= 32'd0;
    REQ     <= 1'b0;
    WRB     <= 1'b0;
    BSTROBE <= 4'd0;
    BURST   <= 2'd0;
  end
  else if(~freeze) begin
    if(TLAST & ~STALL) begin
      ADDR  <= 32'd0;
      REQ   <= 1'b0;
      WRB   <= 1'b0;
      BSTROBE <= 4'd0;
      BURST   <= 2'd0;
    end
    else begin
      ADDR  <= biu_adr_i;
      REQ   <= biu_cyc_i & biu_stb_i & biu_cab_i;
      WRB   <= biu_we_i;
      BSTROBE <= biu_sel_i;
      BURST <= (~peripheral_access) ? 2'd1 : 2'd0; //Burst 
    end
  end
end


always @(posedge clk) begin
  if(reset) bus_rdy <= 1'b1;
  else if(~freeze) begin
    if(TLAST) bus_rdy <= 1'b1;
    else if(biu_cyc_i & biu_stb_i & biu_cab_i) bus_rdy <= 1'b0;
    else bus_rdy <= 1'b1;
  end
end

// Read Logic 
assign biu_dat_o = RDATA;


always @(posedge clk) begin
  if(reset) begin
    bus_reg <= 256'd0;
  end
  else if(~freeze & ~peripheral_access) begin
    if (ACK) begin
      case(counter)
        4'd0: bus_reg[31:0] <= RDATA;
        4'd1: bus_reg[63:32] <= RDATA;
        4'd2: bus_reg[95:64] <= RDATA;
        4'd3: bus_reg[127:96] <= RDATA;
        4'd4: bus_reg[159:128] <= RDATA;
        4'd5: bus_reg[191:160] <= RDATA;
        4'd6: bus_reg[223:192] <= RDATA;
        4'd7: bus_reg[255:224] <= RDATA;
        default: bus_reg <= 256'd0;
      endcase
    end
  end
  else if(~freeze & peripheral_access ) begin
    if(ACK) begin
      case(biu_adr_i[4:2])
        4'd0: bus_reg[31:0] <= RDATA;
        4'd1: bus_reg[63:32] <= RDATA;
        4'd2: bus_reg[95:64] <= RDATA;
        4'd3: bus_reg[127:96] <= RDATA;
        4'd4: bus_reg[159:128] <= RDATA;
        4'd5: bus_reg[191:160] <= RDATA;
        4'd6: bus_reg[223:192] <= RDATA;
        4'd7: bus_reg[255:224] <= RDATA;
        default: bus_reg <= 256'd0;
       endcase
    end
  end
end


//Write Logic

always @(posedge clk) begin
  if(reset) begin
    WDATA <= 32'd0;
  end
  else if(~freeze) begin
    if(~peripheral_access) begin
      case(write_counter)
        3'd0: WDATA <= i_bus_data[31:0];
        3'd1: WDATA <= i_bus_data[63:32];
        3'd2: WDATA <= i_bus_data[95:64];
        3'd3: WDATA <= i_bus_data[127:96];
        3'd4: WDATA <= i_bus_data[159:128];
        3'd5: WDATA <= i_bus_data[191:160];
        3'd6: WDATA <= i_bus_data[223:192];
        3'd7: WDATA <= i_bus_data[255:224];
        default: WDATA <= 32'd0;
        endcase
    end
    else begin
      case(biu_adr_i[4:2])
        3'd0: WDATA <= i_bus_data[31:0];
        3'd1: WDATA <= i_bus_data[63:32];
        3'd2: WDATA <= i_bus_data[95:64];
        3'd3: WDATA <= i_bus_data[127:96];
        3'd4: WDATA <= i_bus_data[159:128];
        3'd5: WDATA <= i_bus_data[191:160];
        3'd6: WDATA <= i_bus_data[223:192];
        3'd7: WDATA <= i_bus_data[255:224];
        default: WDATA <= 32'd0;
        endcase
    end
  end
end

always @(posedge clk) begin
  if(reset) counter <= 3'd0;
  else if(~freeze) begin
    counter <= (ACK & ~STALL & ~TLAST) ? (counter + 3'd1) : 3'd0;
  end
end

always @(posedge clk) begin
  if(reset) write_counter <= 3'd0;
  else if(~freeze & ~peripheral_access) begin
    if(biu_cyc_i & biu_stb_i & biu_cab_i & ~TLAST) begin
      write_counter <= write_counter + 3'd1;
    end
    else if(TLAST) begin
      write_counter <= 3'd0;
    end
  end
  else if(~freeze & peripheral_access) begin
    write_counter <= 3'd0;
  end
end
endmodule
