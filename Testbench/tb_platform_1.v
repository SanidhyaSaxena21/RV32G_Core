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
  reg [31:0] counter;
  reg timeout;
  reg done;
  reg pass;
  reg fail;
  wire [31:0] rf_mem_17;
  wire [31:0] rf_mem_10;
  parameter CLOCK_PERIOD = 100; //100ns
  parameter TIMEOUT = 300000; //200us
  parameter COUNT = (TIMEOUT / CLOCK_PERIOD);
  
  
  reg [31:0] total_cycle_count, num_inst;
  
  wire pipeline_stall,cache_flush;
  wire [2:0] state_dcache;

    reg SYSCLK_P,SYSCLK_N;
  riscv_platform riscv_platform(
   .clk_int(clk_int),
  .rst(rst));
  
  //.RTC_CLOCK(RTC_CLOCK),

  //.ext_irq(ext_irq),

  //.cache_en_int(cache_en_int)

  //.tick_en(tick_en),
  //.addr_exception(addr_exception),
  //.interrupt(interrupt)
  //`ifdef itlb_def
  //,.vpn_to_ppn_req(vpn_to_ppn_req)
  //`endif 
  
  //`ifdef TEST
  //,.block_instr_int(block_instr_int)
  //`endif );
`define FUNCTIONAL
  integer logfile;
	initial begin
  	logfile = $fopen("/home/rclab/FINAL_PROJECT/RV32G_Core/RV32G_Core/RV32G_Debug/tb_result.log", "w");
	end

      initial begin
         rst <= 1'b1; clk_int <= 1'b0; clk_x2 <= 1'b0;
         SYSCLK_P <= 1'b1; 
         SYSCLK_N <= 1'b0;
         //wb_rty_i <= 1'b0; wb_err_i <= 1'b0;
         cache_en_int <= 1'b0;
         //STALL <= 1'b0;
         interrupt = 32'd0; //Drive Alssl interrupts to 0
         ext_irq <= 1'b0; RTC_CLOCK <= 1'b0;
         
         #1000 rst <= 1'b0;
         
         /*
         #3500 ext_irq <= 1'b1;
         #20 ext_irq <= 1'b0;*/
         //#4370 ext_irq <= 1'b1;
         //#4400 ext_irq <= 1'b0;
         
         
         //#6000 cache_flush_int <= 1'b1;
         //
         //
         `ifdef FUNCTIONAL
         wait(done || timeout);
         $display("ENTERING DEBUG MODE");
         $display("Total Number of Instruction = %d",num_inst);
         $display("Totoal Number of Cycles = %d",total_cycle_count);

            $finish;
        `endif
         
       end

       always #50 clk_int <= ~clk_int;
       always #2.5 SYSCLK_P <= ~SYSCLK_P;
       always #2.5 SYSCLK_N <= ~SYSCLK_N;

       //always #5 clk_x2 <= ~clk_x2;

       //always #50 RTC_CLOCK <= ~RTC_CLOCK;
       

`ifdef FUNCTIONAL
      //---------------SELF TESTING LOGIC ------------------
      assign rf_mem_17 = tb_platform.riscv_platform.cpu1.fdem.Pipeline.RF.MEM[17] ;
      assign rf_mem_10 = tb_platform.riscv_platform.cpu1.fdem.Pipeline.RF.MEM[10] ; 

      wire ebreak_in_id; 
      reg detect_ebreak;
      reg [31:0] pc_ebreak_detect;
      wire [31:0] pc_ebreak;

      assign ebreak_in_id = tb_platform.riscv_platform.cpu1.fdem.Pipeline.debug_Controller.ebreak_in_id;
      assign ebreak_in_wb = tb_platform.riscv_platform.cpu1.fdem.Pipeline.debug_Controller.ebreak_in_wb;
      assign pc_ebreak    = tb_platform.riscv_platform.cpu1.fdem.Pipeline.ID.PC_ID;

      always @(posedge clk_int or posedge rst) begin
	       if(rst) begin 
		       done <= 1'b0;
		       pass <= 1'b0;
		       fail <= 1'b0;
	       end
	       else if((ebreak_in_wb) && (rf_mem_17 == 32'h00000005d)) begin
		       if((rf_mem_10 == 32'd0)) begin
			       $fdisplay(logfile, "\n");
			       $fdisplay(logfile, "   ____   ___  ___  ___ _____ ");
			       $fdisplay(logfile, "  |  _ \\ / _ \\ |  \\/  || ____|");
			       $fdisplay(logfile, "  | |_) | | | || |\\/| ||  _|  ");
			       $fdisplay(logfile, "  |  __/| |_| || |  | || |___ ");
			       $fdisplay(logfile, "  |_|    \\___/ |_|  |_||_____|");
			       $fdisplay(logfile, "");
			       $fdisplay(logfile, "\n");
			       $fdisplay(logfile, "===================================");
			       $fdisplay(logfile, "==============PASS=================");
			       $fdisplay(logfile, "===================================");
			       $fdisplay(logfile, "Test Case Passed!");


			       $display("\n");
			       $display("   ____   ___  ___  ___ _____ ");
			       $display("  |  _ \\ / _ \\ |  \\/  || ____|");
			       $display("  | |_) | | | || |\\/| ||  _|  ");
			       $display("  |  __/| |_| || |  | || |___ ");
			       $display("  |_|    \\___/ |_|  |_||_____|");
			       $display("");
			       $display("\n");
			       $display("===================================");
			       $display("==============PASS=================");
			       $display("===================================");
			       $display("rf_mem_17 = %h, rf_mem_10 = %h", rf_mem_17, rf_mem_10);
			       done <= 1'b1;
			       pass <= 1'b1;
			       fail <= 1'b0;
		       end
		       else begin
			       done <= 1'b1;
			       pass <= 1'b0;
			       fail <= 1'b1;
			       $display("===================================");
			       $display("==============FAIL=================");
			       $display("===================================");
			       $display("Test Case Failed");
			       $fdisplay(logfile, "===================================");
			       $fdisplay(logfile, "==============FAIL=================");
			       $fdisplay(logfile, "===================================");
			       $fdisplay(logfile, "Test Case Failed");

		       end
	       end
	       else if(ebreak_in_wb) begin
	           done <= 1'b1;
	           $fdisplay(logfile, "Total Number of Instruction = %d",num_inst);
               $fdisplay(logfile, "Total Number of Cycles = %d",total_cycle_count);
	           
	       end
	       else if(timeout) begin
	           done <= 1'b1;
	           fail <= 1'b1;
	           pass <= 1'b0;
	           
	       end
	       else begin 
	           done <= 1'b0;
	           pass <= 1'b0;
	           fail <= 1'b0;
	       end
       end
      
       always @(posedge clk_int) begin
         if(rst) begin
           detect_ebreak <= 1'b0;
           pc_ebreak_detect <= 32'd0;
         end
         else if(ebreak_in_id) begin
           detect_ebreak <= 1'b1;
           pc_ebreak_detect <= pc_ebreak;
           $display("EBREAK DETECTED");
           $display("PC_EBREAK = %h",pc_ebreak);
           $fdisplay(logfile, "PC_EBREAK = %h",pc_ebreak);
           $fdisplay(logfile, "EBREAK DETECTED");
         end
         else detect_ebreak <= 1'b0;
       end
//---------------------------TIMEOUT LOGIC ---------------------------------
//--------------------------------------------------------------------------
 
	always @(posedge clk_int or posedge rst) begin
		if(rst) begin
			counter <= 32'd0;
			timeout <= 1'b0;
		end
		else if(done) counter <= 32'd0; 
		else begin
			if(counter == COUNT) begin
				counter <= 32'd0;
				timeout <= 1'b1;
				$display("===================================");
				$display("============TIMEOUT================");
				$display("===================================");
				$fdisplay(logfile,"===================================");
				$fdisplay(logfile,"============TIMEOUT================");
				$fdisplay(logfile,"===================================");


			end	
			else begin 
				counter <= counter + 32'd1;
				timeout <= 1'b0;
			end
		end
	end


//-------------------------Performance LOGIC----------------------------------
//----------------------------------------------------------------------------



assign pipeline_stall = tb_platform.riscv_platform.cpu1.fdem.Pipeline.ID.IF_ID_Freeze;
assign state_dcache = tb_platform.riscv_platform.cpu1.fdem.db1.dt1.drf0.state;
assign cache_flush = tb_platform.riscv_platform.cpu1.fdem.db1.dt1.drf0.cache_flush;


always @(posedge clk_int or posedge rst) begin
  if(rst) begin
    total_cycle_count <= 32'd0;
  end
  else if((state_dcache != 3'd7) & (~cache_flush)) begin
    total_cycle_count <= total_cycle_count + 32'd1;
  end
end


always @(posedge clk_int or posedge rst) begin
  if(rst) begin
    num_inst <= 32'd0;
  end
  else if(~pipeline_stall & ~ebreak_in_id & ~cache_flush) begin
    num_inst <= num_inst + 32'd1;
  end
end
`endif

endmodule

