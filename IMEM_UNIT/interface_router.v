
//TODO:     Check if biu_stb or biu_cyc signals drop before burst being completed
//Designer: Sanidhya Saxena, Toms JiJi Varghese
//Data:     14/12/2024
//Guide:    Kuruvilla Varghese 
//
//
//


module interface_router (
   
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
   output [31:0] 			  biu_dat_o,	  // output data bus
   output reg           bus_rdy,      //interface to the dcache unit
   inout  [255:0]       bus_data,     //to be able to communicate with proc interface 
  
   input                peripheral_access, 
   //Memory Interface
   output reg [31:0]  ADDR,
   output reg [1:0]   BURST, //00-Normal, 01-INCR, 10-WRAP, 11-Reserved
   output reg         REQ,
   output reg         WRB,
   output reg [31:0]  WDATA,
   input      [31:0]  RDATA,
   input              ACK,
   input              STALL,
   output reg [3:0]   BSTROBE,
  
   //Control Signals
   input freeze
    
  );


  parameter IDLE = 0;
  parameter TRANS = 1;
  parameter LAST = 2;
  parameter BURST_LENGTH = 8;

  reg [1:0] present_state;
  reg [1:0] next_state;
  reg [255:0] bus_reg;
  reg [3:0] count;
  reg req_next;
  reg wrb_next;
  reg [2:0] write_counter;

  assign bus_data = biu_we_i ? 256'bz : bus_reg;
  assign biu_dat_o = RDATA;

  always @(posedge clk) begin
    if(reset) begin
      present_state <= IDLE;
    end
    else if(~freeze) begin
      present_state <= next_state;
    end
  end

  always @(*) begin
    case(present_state)
      IDLE: begin
        req_next = biu_cyc_i && biu_stb_i && biu_cab_i;
        wrb_next = biu_we_i;
        if(biu_cyc_i && biu_stb_i && biu_cab_i && ~STALL) begin
          if(~peripheral_access) begin
            next_state = TRANS;
          end
          else begin
            next_state = LAST;
          end
        end
        else next_state = IDLE;
      end
      TRANS: begin
        req_next = (~REQ | ~ACK | count < BURST_LENGTH);
        wrb_next = ((~REQ | ~ACK | count < BURST_LENGTH-2) & biu_we_i);
        if(ACK && REQ && count == (BURST_LENGTH-1))
          next_state = IDLE;
        else
          next_state = TRANS;
      end
      LAST: begin
        req_next = (~REQ | ~ACK);
        wrb_next = ((~REQ | ~ACK) & biu_we_i);
        if(ACK && REQ) next_state = IDLE;
        else next_state = LAST;
      end
    endcase
  end

  always @(posedge clk) begin
    if(reset) begin
        WRB <= 1'b0;
    end
    else if(~freeze) begin
        WRB <= wrb_next;
    end
  end
  always @(posedge clk) begin
    if(reset) begin
      ADDR <= {32{1'b0}};
      REQ <= 1'b0;
      //WRB <= 1'b0;
      BURST <= 2'b00;
      write_counter <= 3'b000;
    end
    else if(~freeze && ~peripheral_access) begin
      BURST <= 2'b01;
      if(REQ && count == (BURST_LENGTH-1) && ~STALL && ACK)
        REQ <= 1'b0;
      else
        REQ <= req_next;
      
      //WRB <= wrb_next;  
      if(present_state == IDLE) begin
        //WRB <= biu_we_i;
        BSTROBE <= biu_sel_i;
        ADDR <= biu_adr_i;
        write_counter <= 3'd0;
      end
      else if(REQ & ~STALL) begin

        if(BURST_LENGTH == 4) begin
          ADDR[3:2] <= ADDR[3:2] + 1;
          write_counter <= write_counter + 1'd1;
        end

        if(BURST_LENGTH == 8) begin
          ADDR[4:2] <= ADDR[4:2] + 1;
          write_counter <= write_counter + 1'd1;
        end
      end
    end
    else if(~freeze && peripheral_access) begin
      BURST <= 2'b00;
      REQ <= req_next;
      ADDR <= biu_adr_i;
    end
  end


  always @(*) begin
    if(reset) begin
      bus_reg <= 256'd0;
    end
    else if(~peripheral_access) begin
      if (ACK) begin
        case(count)
          4'd0: bus_reg[31:0] <= RDATA;
          4'd1: bus_reg[63:32] <= RDATA;
          4'd2: bus_reg[95:64] <= RDATA;
          4'd3: bus_reg[127:96] <= RDATA;
          4'd4: bus_reg[159:128] <= RDATA;
          4'd5: bus_reg[191:160] <= RDATA;
          4'd6: bus_reg[223:192] <= RDATA;
          4'd7: bus_reg[255:224] <= RDATA;
          default: bus_reg = bus_reg;
        endcase
      end
    end
    else begin
      if(ACK) begin
        case(biu_adr_i[4:2])
          3'b000:bus_reg[31:0] <= RDATA;
          3'b001:bus_reg[63:32] <= RDATA;
          3'b010:bus_reg[95:64] <= RDATA;
          3'b011:bus_reg[127:96] <= RDATA;
          3'b100:bus_reg[159:128] <= RDATA;
          3'b101:bus_reg[191:160] <= RDATA;
          3'b110:bus_reg[223:192] <= RDATA;
          3'b111:bus_reg[255:224] <= RDATA;
          default: bus_reg <= bus_reg;
        endcase
      end
    end
  end

  always @(*) begin
    if(reset) begin
      WDATA <= 32'd0;
    end
    else if(~peripheral_access) begin
      case(write_counter)
        4'd0: WDATA <= bus_data[31:0]    ;
        4'd1: WDATA <= bus_data[63:32]   ;
        4'd2: WDATA <= bus_data[95:64]   ;
        4'd3: WDATA <= bus_data[127:96]  ;
        4'd4: WDATA <= bus_data[159:128] ;
        4'd5: WDATA <= bus_data[191:160] ;
        4'd6: WDATA <= bus_data[223:192] ;
        4'd7: WDATA <= bus_data[255:224] ;
        default: WDATA = WDATA;
      endcase
    end
    else if(peripheral_access) begin
      case(biu_adr_i[4:2])
        3'd0: WDATA <= bus_data[31:0]    ;
        3'd1: WDATA <= bus_data[63:32]   ;
        3'd2: WDATA <= bus_data[95:64]   ;
        3'd3: WDATA <= bus_data[127:96]  ;
        3'd4: WDATA <= bus_data[159:128] ;
        3'd5: WDATA <= bus_data[191:160] ;
        3'd6: WDATA <= bus_data[223:192] ;
        3'd7: WDATA <= bus_data[255:224] ;
        default: WDATA <= bus_reg;
      endcase

    end
  end

  // Bus_rdy logic: If it is a peripheral access, ready will be ACK. If it's
  // a memory access, then bus_rdy will go high after the burst is completed
  always @( posedge clk) begin
      if(reset) begin
          bus_rdy <= 1'b1;
      end
      else if((biu_stb_i | biu_cyc_i) & (~freeze)) begin
          if(~peripheral_access && (count == BURST_LENGTH-1)) begin
              bus_rdy <= 1'b1;
          end
          else if(peripheral_access) begin
              bus_rdy <= ACK;
          end
          else begin
              bus_rdy <= 1'b0;
          end
      end
  end
  
  always @(posedge clk) begin
    if(reset) begin
      count <= 0;
    end
    else if (~freeze) begin
      if(present_state == IDLE)
        count <= 0;
      else if(ACK && (count < BURST_LENGTH-1) && ~STALL && ~peripheral_access) begin // TODO: Need to Add ACK or not ?
        count <= count + 1;
      end
      else if(ACK && count == BURST_LENGTH-1 && ~STALL) begin
        count <= 4'd0;
      end
    end
  end
endmodule
