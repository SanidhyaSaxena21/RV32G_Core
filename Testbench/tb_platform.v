`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/05/2025 12:03:02 AM
// Design Name: 
// Module Name: tb_platform
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


module tb_platform(

    );
    
  reg clk_int;
  reg clk_x2;
  reg rst;

  reg ext_irq;
  
  reg RTC_CLOCK;

  reg cache_en_int;

  wire tick_en;
  wire addr_exception;
  reg [31:0] interrupt;

  `ifdef itlb_def
  wire vpn_to_ppn_req;
  `endif 
  
  `ifdef TEST
  wire [31:0] block_instr_int;
  `endif
  
  parameter RTC_FREQUENCY = 1000000; // In Mhz
  
  parameter RTC_TIME = (1/RTC_FREQUENCY)*1000000000;
  
  riscv_platform riscv_platform(
  .clk_int(clk_int),
  .clk_x2(clk_x2),
  .rst(rst),
  
  .RTC_CLOCK(RTC_CLOCK),

  .ext_irq(ext_irq),

  .cache_en_int(cache_en_int),

  .tick_en(tick_en),
  .addr_exception(addr_exception),
  .interrupt(interrupt)
  `ifdef itlb_def
  ,.vpn_to_ppn_req(vpn_to_ppn_req)
  `endif 
  
  `ifdef TEST
  ,.block_instr_int(block_instr_int)
  `endif );

      initial begin
         rst <= 1'b1; clk_int <= 1'b0; clk_x2 <= 1'b0;
         //wb_rty_i <= 1'b0; wb_err_i <= 1'b0;
         cache_en_int <= 1'b0;
         //STALL <= 1'b0;
         interrupt = 32'd0; //Drive Alssl interrupts to 0
         ext_irq <= 1'b0; RTC_CLOCK <= 1'b0;
         
         #200 rst <= 1'b0;
         
         /*
         #3500 ext_irq <= 1'b1;
         #20 ext_irq <= 1'b0;*/
         //#4370 ext_irq <= 1'b1;
         //#4400 ext_irq <= 1'b0;
         
         
         //#6000 cache_flush_int <= 1'b1;
         
       end

       always #10 clk_int <= ~clk_int;

       always #5 clk_x2 <= ~clk_x2;

       always #50 RTC_CLOCK <= ~RTC_CLOCK;


endmodule
