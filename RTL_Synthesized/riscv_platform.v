`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/04/2025 11:50:35 PM
// Design Name: 
// Module Name: riscv_platform
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
`include "defines.v"

module riscv_platform (

  //Global Signals
  input SYSCLK_P,
  input SYSCLK_N,
  
  input DEBUG_OVERWRITE,
  
  input RX,
  output TX,
  
  input next_addr,
  input prev_addr,
  
  input ext_mode,
  input debug_display,

  input RESET_BUTTON,

  //JTAG Signals
  input wire TDI,
  input wire TMS,
  input wire TCK,
  output wire TDO,

  output reg [7:0] LED
  

  //input clk_int,

  //input RTC_CLOCK,
  //input ext_irq,

  //input cache_en_int

  //output tick_en,
  //output addr_exception,
  //input [31:0] interrupt

    );

      //-------------DATA MEMORY INTERFACE---------------------
   wire  [31:0]  DADDR;
   wire  [1:0]   DBURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
   wire          DREQ;
   wire          DWRB;
   wire [31:0]  DWDATA;
   wire      [31:0]  DRDATA;
   wire              DACK;
   wire              DSTALL;
   wire [3:0]   DBSTROBE;

  //-------------INSTRUCTION MEMORY INTERFACE---------------------
   wire  [31:0]  IADDR;
   wire  [1:0]   IBURST; //00-Normal; 01-INCR; 10-WRAP; 11-Reserved
   wire          IREQ;
   wire          IWRB;
   wire [31:0]  IWDATA;
   wire      [31:0]  IRDATA;
   wire              IACK;
   wire              ISTALL;
   wire [3:0]   IBSTROBE;



  //-------------MEMORY INTERFACE---------------------
   wire  [31:0]  ADDR;
   wire  [1:0]   BURST; //00-Normal; 01-NCR; 10-WRAP; 11-Reserved
   wire          REQ;
   wire          WRB;
   wire [31:0]  WDATA;
   wire      [31:0]  RDATA;
   wire              ACK;
   wire              STALL;
   wire [3:0]   BSTROBE;

   wire [31:0] s_RDATA;
   wire s_tlast,m_tlast;
   wire [31:0] m_RDATA;
   wire s_ACK;
   wire s_STALL;
   wire m_ACK;
   wire m_STALL;
   wire clint_en;

   wire timer_irq;
   wire sw_irq;
   wire [`CSR_SB_W-1:0] csr_pmp_sb;
   
   wire [2:0] state;

   wire [63:0] lcd_reg;
   wire [3:0] imem_allow,dmem_allow; // L/X/W/R
   wire  dmem_read_allowed,dmem_write_allowed,dmem_read_write_allowed;

   wire imem_read_execute_allowed;
   
   wire TLAST, ITLAST, DTLAST;
   
   wire o_buzzer;

  `ifdef itlb_def
  wire vpn_to_ppn_req;
  `endif 
  
 
  wire ext_irq;
  wire RTC_CLOCK;
  wire cache_en_int;

  wire Soc_reset;
  
  reg next_addr_meta,next_addr_sync;
    reg switch_next_q, switch_prev_q;
    wire switch_next_pulse, switch_prev_pulse;
    reg [31:0] ext_addr;
    reg [7:0] ext_rdata;
    wire [7:0] debug_signals;
    wire [7:0] lcd_reg_uart;

  //assign LED[7:0] = ext_mode ? ext_rdata : {state,lcd_reg[4:0]};

  always @(*) begin
    case({debug_display,ext_mode})
      2'b00:  LED = lcd_reg_uart[7:0];
      2'b01:  LED = ext_rdata;
      2'b10:  LED = debug_signals;
      2'b11:  LED = lcd_reg[7:0];
    endcase
  end
  assign cache_en_int = 1'b0;
//assign RTC_CLOCK = clk_int;
wire clk_int;
clk_wiz_0 Clock_module (
                        .clk_out1(clk_int),
                        .clk_out2(RTC_CLOCK),
                        .clk_in1_p(SYSCLK_P),
                        .clk_in1_n(SYSCLK_N));
                        
  
assign o_buzzer = vpn_to_ppn_req;
//Reset Synchroniser

reg rst_meta;
reg rst_sync;
wire rst;

always @(posedge clk_int or posedge RESET_BUTTON) begin
    if(RESET_BUTTON) begin //Assetion
        rst_meta <= 1'b1;
        rst_sync <= 1'b1;
    end
    else begin
        rst_meta <= 1'b0;
        rst_sync <= rst_meta;
    end
end

ila_0 your_instance_name (
	.clk(clk_int), // input wire clk


	.probe0(IADDR), // input wire [31:0]  probe0  
	.probe1(IRDATA), // input wire [31:0]  probe1 
	.probe2(probe2), // input wire [31:0]  probe2 
	.probe3({IREQ,IACK,vpn_to_ppn_req,imem_allow,1'b0}) // input wire [4:0]  probe3
);

//------------External ADDRESS Logic ------------------------
wire next_addr_clean;
wire prev_addr_clean;
debouce db0 (.reset(RESET_BUTTON),.clk(clk_int),.noisy(next_addr),.clean(next_addr_clean));
debouce db1 (.reset(RESET_BUTTON),.clk(clk_int),.noisy(prev_addr),.clean(prev_addr_clean));

reg ext_req;

always @(posedge clk_int or posedge RESET_BUTTON) begin
    if(RESET_BUTTON) begin 
        switch_next_q <= 1'b0;
        switch_prev_q <= 1'b0;
    end    
    else begin 
        switch_next_q <= next_addr_clean;
        switch_prev_q <= prev_addr_clean;
    end    
end 

assign switch_next_pulse = next_addr_clean & ~switch_next_q;
assign switch_prev_pulse = prev_addr_clean & ~switch_prev_q;

always @(posedge clk_int or posedge RESET_BUTTON) begin
    if(RESET_BUTTON) begin 
        ext_addr <= 32'h2000_0000;
        ext_req  <= 1'b0;
    end
    else if(switch_next_pulse & ~ext_req) begin
        ext_addr <= ext_addr + 32'd4;
        ext_req  <= 1'b1;
    end
    else if(switch_prev_pulse & ~ext_req) begin
        ext_addr <= ext_addr - 32'd4;
        ext_req  <= 1'b1;
    end
    else if(DACK)begin
        ext_req <= 1'b0;
    end
end  

always @(posedge clk_int or posedge RESET_BUTTON) begin
    if(RESET_BUTTON) ext_rdata <= 8'd0;
    else if(DACK) ext_rdata <= DRDATA[7:0];
end

//assign ext_rdata = DRDATA[7:0];



//------------RESET Logic ----------------------------
assign rst = rst_sync | (Soc_reset & ~DEBUG_OVERWRITE);
  
       Top_CPU_JTAG u_core_top(
           .cpu_clock(clk_int),
           .RESET_BUTTON(rst_sync),
           .state(state),
           .debug_signals(debug_signals),
           .Soc_reset(Soc_reset),

           .TDI(TDI),
           .TDO(TDO),
           .TMS(TMS),
           .TCK(TCK),

           .cache_en(cache_en_int),
           .csr_pmp_sb(csr_pmp_sb),
           .led(lcd_reg),
           .DEBUG_OVERWRITE(DEBUG_OVERWRITE),
           .irq_i({20'd0,
                      ext_irq,
                      3'd0,
                      timer_irq,
                      3'd0,
                      sw_irq,
                      3'd0}),

          .IADDR(IADDR),
          .IBURST(IBURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
          .IREQ(IREQ),
          .IWRB(IWRB),
          .IWDATA(IWDATA),
          .IRDATA(IRDATA),
          .IACK(IACK),
          .ISTALL(ISTALL),
          .IBSTROBE(IBSTROBE),
          .ITLAST(ITLAST),

          .DADDR(DADDR),
          .DBURST(DBURST), //00-Normal(), 01-INCR(), 10-WRAP(), 11-Reserved
          .DREQ(DREQ),
          .DWRB(DWRB),
          .DWDATA(DWDATA),
          .DRDATA(DRDATA),
          .DACK(DACK),
          .DSTALL(DSTALL),
          .DBSTROBE(DBSTROBE),
          .DTLAST(DTLAST)
          `ifdef itlb_def
          ,.vpn_to_ppn_req(vpn_to_ppn_req)
          `endif  
       );

  

//==================== Memory Protection Unit =======================
       (* keep_hierarchy = "yes" *)
       rv32_mpu #(.ILEN(),.XLEN(), .MPU_SUPPORT(), .NB_PMP_REGION(),.MAX_PMP_REGION(),.MMU_SUPPORT(0)) IMEM_MPU (
        .aclk(clk_int),
        .aresetn(~rst),
        .imem_addr(IADDR),
        .imem_allow(imem_allow),
        .dmem_addr(ext_mode ? ext_addr : DADDR),
        .dmem_allow(dmem_allow),
        .csr_sb(csr_pmp_sb)
       );
     
      assign dmem_read_allowed = (dmem_allow == 4'd1); 
      assign dmem_write_allowed = (dmem_allow == 4'd2); 
      assign dmem_read_write_allowed = (dmem_allow == 4'd3) | (dmem_allow == 4'd5);

      assign imem_read_execute_allowed = (imem_allow == 4'd5);





    // Internal wires between Memory_Wrapper and Main_Memory_Wrapper
    wire         s0_req;
    wire         s0_write;
    wire [31:0]  s0_addr;
    wire [1:0]   s0_burst;
    wire [3:0]   s0_bstrobe;
    wire [31:0]  s0_write_data;
    wire [31:0]  s0_rdata;
    wire         s0_ready;
    wire         s0_stall;
    wire         s0_tlast;


    // Internal wires between Memory_Wrapper and CLINT
    wire         s1_req;
    wire         s1_write;
    wire [31:0]  s1_addr;
    wire [1:0]   s1_burst;
    wire [3:0]   s1_bstrobe;
    wire [31:0]  s1_write_data;
    wire [31:0]  s1_rdata;
    wire         s1_ready;
    wire         s1_stall;
    wire         s1_tlast;


    // Internal wires between Memory_Wrapper and UART
    wire         s2_req;
    wire         s2_write;
    wire [31:0]  s2_addr;
    wire [1:0]   s2_burst;
    wire [3:0]   s2_bstrobe;
    wire [31:0]  s2_write_data;
    wire [31:0]  s2_rdata;
    wire         s2_ready;
    wire         s2_stall;
    wire         s2_tlast;


      // Instantiate Memory_Wrapper (handles peripherals, routes to s0_)
      (* keep_hierarchy = "yes" *)
    Interconnect_Wrapper Interconnect (
        .m_req         (ext_mode ? ext_req  : DREQ),
        .m_addr        (ext_mode ? ext_addr : DADDR),
        .m_burst       (ext_mode ? 2'b00    : DBURST),
        .m_write       (ext_mode ? 1'b0     : DWRB),
        .m_write_data  (DWDATA),
        .m_bstrobe     (ext_mode ? 4'b1111  : DBSTROBE),
        .m_rdata       (DRDATA),
        .m_ready       (DACK),
        .m_stall       (DSTALL),
        .m_tlast       (DTLAST),

        // Connect slave 0 (main memory) to Main_Memory_Wrapper's data interface
        .s0_req        (s0_req),
        .s0_addr       (s0_addr),
        .s0_burst      (s0_burst),
        .s0_write      (s0_write),
        .s0_write_data (s0_write_data),
        .s0_bstrobe    (s0_bstrobe),
        .s0_rdata      (s0_rdata),
        .s0_ready      (s0_ready),
        .s0_stall      (s0_stall),
        .s0_tlast      (s0_tlast),

        // Other peripherals (s1_, s2_, s3_) can be connected similarly...
        .s1_req(s1_req), 
        .s1_addr(s1_addr), 
        .s1_burst(s1_burst), 
        .s1_write(s1_write), 
        .s1_write_data(s1_write_data), 
        .s1_bstrobe(s1_bstrobe),
        .s1_rdata(s1_rdata), 
        .s1_ready(s1_ready), 
        .s1_stall(s1_stall), 
        .s1_tlast(s1_tlast),

        .s2_req(s2_req), 
        .s2_addr(s2_addr), 
        .s2_burst(s2_burst), 
        .s2_write(s2_write), 
        .s2_write_data(s2_write_data), 
        .s2_bstrobe(s2_bstrobe),
        .s2_rdata(s2_rdata), 
        .s2_ready(s2_ready),
        .s2_stall(s2_stall), 
        .s2_tlast(s2_tlast),

        .s3_req(), .s3_addr(), .s3_burst(), .s3_write(), .s3_write_data(), .s3_bstrobe(),
        .s3_rdata(32'b0), .s3_ready(1'b0), .s3_stall(1'b0), .s3_tlast(1'b0)
    );

//====================SLAVE0 Main Memory =================================
        (* keep_hierarchy = "yes" *)
       Main_Memory_Wrapper Main_Memory_Wrapper (
        .clk(clk_int),
        .reset(rst),
        

    // Instruction Bus Interface
        .instr_req(IREQ),        // Instruction request signal
        .instr_addr(IADDR),       // Instruction address bus
        .instr_burst(IBURST),
        .instr_write(IWRB),
        .instr_write_data(IWDATA),
        .instr_bstrobe(IBSTROBE), 

        .instr_data(IRDATA),       // Instruction read data
        .instr_ready(IACK),      // Instruction bus ready signal
        .instr_stall(ISTALL),
        .instr_tlast(ITLAST),
        .imem_read_execute_allowed(imem_read_execute_allowed),

    // Data Bus Interface
        .data_req(s0_req),         // Data request signal
        .data_write(s0_write),       // Data write enable
        .data_addr(s0_addr),        // Data address bus
        .data_write_data(s0_write_data),  // Data write data
        .data_burst(s0_burst),
        .data_bstrobe(s0_bstrobe),

        .data_read_data(s0_rdata),   // Data read data
        .data_ready(s0_ready),       // Data bus ready signal
        .data_stall(s0_stall),
        .data_tlast(s0_tlast),
        .dmem_read_write_allowed(dmem_read_write_allowed) 
      );


//====================SLAVE1 CLINT=================================
    (* keep_hierarchy = "yes" *)
    rv32_clint #(.ADDRW(32),.XLEN(32)) CLINT (
        .CLK(clk_int),
        .RST(rst),
        .s_en(s1_req),
        .s_wr(s1_write),
        .s_addr(s1_addr),
        .s_wdata(s1_write_data),
        .s_strb(s1_bstrobe),
        .s_rdata(s1_rdata),
        .s_tlast(s1_tlast),
        .s_ready(s1_ready),
        .s_stall(s1_stall),

        .rtc(RTC_CLOCK),
        .sw_irq(sw_irq),
        .timer_irq(timer_irq)
      );

//====================SLAVE2 UART=================================

  (* keep_hierarchy = "yes" *)
  UART_TOP #(.CLOCK_RATE(40000000),
             .BAUD_RATE(9600)) UART (
    .clk(clk_int),
    .rst(rst),

    .RX(RX),
    .TX(TX),
    .lcd_reg_uart(lcd_reg_uart),
    .addr_i   (s2_addr),
    .req_i    (s2_req),
    .burst_i  (s2_burst),
    .write_i  (s2_write),
    .wdata_i  (s2_write_data),
    .bstrobe_i(s2_bstrobe),
    .rdata_o  (s2_rdata),
    .ready_o  (s2_ready),
    .stall_o  (s2_stall),
    .tlast_o  (s2_tlast)
    
  );


//====================SLAVE0=================================
endmodule
