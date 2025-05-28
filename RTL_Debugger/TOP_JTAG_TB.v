`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/28/2025 02:57:23 PM
// Design Name: 
// Module Name: TOP_JTAG_TB
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
module TOP_JTAG_TB(

    );
    parameter MAIN_CLK_HALF_PERIOD_NS = 5;
//    parameter MAIN_CLK_HALF_PERIOD_NS = 2.5;
    parameter JTAG_CLK_HALF_PERIOD_NS = 500;
    
    
    
    // Testbench signals
    reg TDI, TMS, TCK;
    wire TDO;
    wire [3:0] TAP_State;
    wire rst_out;
    wire [5:0] Instr;
    wire [31:0] data0, data1;
    reg [31:0] read_data0;
    
    
    reg [7:0] TMS_reg;
    integer TMS_length=0, i;
    
    reg [31:0] rcvd_data_reg;
    reg [7:0] rcvd_data_reg_2;
    reg [31:0] TDI_reg;
    integer TDI_length=0, j=0;
    reg [3:0] switch;
    reg clock_100Mhz;
    
    reg SYSCLK_P, SYSCLK_N;
    reg rst_n;
    reg [31:0] cmd;
    //==========Testing==========  
    wire [31:0] dpc;
    wire [15:0] R4;
    integer m;
    reg [31:0] temp;
    reg RESET_BUTTON;
    // Instantiate the JTAG module
    /*Top_CPU_JTAG uut (
        .TDO(TDO),
        .TDI(TDI),
        .TMS(TMS),
        .TCK(TCK),
//        .rst_n(rst_n),
        
        .SYSCLK_P(SYSCLK_P),
        .SYSCLK_N(SYSCLK_N),
        
        .data_0_probe(data0),
        .dpc(dpc),
        .R4(R4)
    );*/
  reg clk_int;
  reg clk_x2;
  reg rst;
  
  
      reg [31:0] counter;
  reg timeout;
  reg done;
  reg pass;
  reg fail;
  wire [31:0] rf_mem_17;
  wire [31:0] rf_mem_10;
  parameter CLOCK_PERIOD = 100; //100ns
  parameter TIMEOUT = 3000000; //200us
  parameter COUNT = (TIMEOUT / CLOCK_PERIOD);
  
  
  reg [31:0] total_cycle_count, num_inst;
  
  wire pipeline_stall,cache_flush;
  wire [2:0] state_dcache;
  
  wire ebreak_in_id; 
  reg detect_ebreak;
  reg [31:0] pc_ebreak_detect;
  wire [31:0] pc_ebreak;
  wire ebreak_in_wb;
  
      riscv_platform riscv_platform(
        .SYSCLK_P(SYSCLK_P),
        .SYSCLK_N(SYSCLK_N),
        .RESET_BUTTON(rst),
        .TDI(TDI),
        .TDO(TDO),
        .TMS(TMS),
        .TCK(TCK));
    
    // Generate a 200MHz differential clock (period = 5ns, toggle every 2.5ns)
    initial begin
        SYSCLK_P = 0;
        SYSCLK_N = 1;
        forever begin
            #MAIN_CLK_HALF_PERIOD_NS;
            SYSCLK_P = ~SYSCLK_P;
            SYSCLK_N = ~SYSCLK_N;
        end
    end
    
    // Clock generation
    always begin
        #JTAG_CLK_HALF_PERIOD_NS TCK = 0;
        #JTAG_CLK_HALF_PERIOD_NS TCK = 1;
    end
    
    
    // TMS shifting task
    task automatic TMS_Sequence(input [7:0] pattern, input integer length);
        integer k;
        begin
            for (k = 0; k < length; k = k + 1) begin
                TMS = pattern[k];
//                #10;
                #JTAG_CLK_HALF_PERIOD_NS;
                #JTAG_CLK_HALF_PERIOD_NS;
            end
        end
    endtask
    
    
    // TDI data shifting task
    task automatic TDI_Transfer(input [31:0] send_data, input integer bit_length, output reg [31:0] received_data);
        
        begin
            received_data = 32'd0;
            TMS = 0;
            @(posedge TCK);
            @(negedge TCK);
            temp = 0;
            for (m = 0; m < bit_length; m = m + 1) begin
                if (m == bit_length - 1) TMS = 1;
                TDI = send_data[m];
                @(posedge TCK);
                temp = (received_data >> 1) | (TDO << (bit_length - 1));
                received_data = (received_data >> 1) | (TDO << (bit_length - 1));
                @(negedge TCK);
            end
        end
    endtask
    
    task automatic Update_IR(input [31:0] IR_data);
        integer m;
        begin
            //================================================================
                    // Run-Test/Idle -> Shift-IR
                    TMS_Sequence(8'h03, 3);
                  
                    // Initiate Data Transfer: 8-bit
                    TDI_Transfer(IR_data, 8, rcvd_data_reg_2);           //update IR
                    
                    // Exit1-IR -> Update-IR
                    // Update-IR -> Run-Test/Idle
                    TMS_Sequence(8'h01, 1);
                    TMS_Sequence(8'h00, 1);
                    #(20*JTAG_CLK_HALF_PERIOD_NS);
        end
    endtask
    
    // TDI data shifting task
    task automatic core_reset_halt();
        begin
            #10
            Update_Reg(32'h10, 32'h2000000B);   // set reset_halt_req
            #10;
            Update_Reg(32'h10, 32'h0000000B);   // hartreset
            #10;
            Update_Reg(32'h10, 32'h20000007);   // clr reset_halt_req
            #10;
            Update_Reg(32'h10, 32'h20000003);   
        end
    endtask
    
    task automatic core_halt();
        begin
            #10
            Update_Reg(32'h10, 32'hA0000003);   // deassert reset
        end
    endtask

    task automatic core_resume();
        begin
            #10
            Update_Reg(32'h10, 32'h60000003);   // deassert reset
        end
    endtask
    
    task automatic core_resume_clr();
        begin
            #10
            Update_Reg(32'h10, 32'h20000003);   // deassert reset
        end
    endtask
    
    task automatic clear_cmderr();
        begin
            #10
            Update_Reg(32'h16, 32'h0000FFFF);   // deassert reset
        end
    endtask
    
    task automatic Access_reg_command(input write, input transfer, input [15:0] regno, input [31:0] write_data);
        begin
            cmd = {
                    8'h00,          // [31:24] cmdtype = 0
                    1'b0,           // [23] reserved
                    3'b010,         // [22:20] aarsize = word (4 bytes)
                    1'b0,           // [19] aarpostincrement
                    1'b0,           // [18] postexec
                    transfer,          // [17] transfer
                    write,          // [16] writebar: 0 = read, 1 = write
                    regno           // [15:0] register number
                };
            #10;
            Update_Reg(32'h04, write_data);   // data0 <= write_data
            #10
            Update_Reg(32'h17, cmd);   // data0 <= write_data
            #10;
//            Read_Reg(32'h04, 32'd0, data0);
            #10;
        end
    endtask
    
    
    task automatic Access_mem_command(input write, input [2:0] aamsize, input [31:0] ptr, input [31:0] write_data);
        begin
            cmd = {
                8'h02,              // [31:24] cmdtype = 0
                1'b0,               // [23] aamvirtual
                aamsize,            // [22:20] aamsize
                1'b0,               // [19] aampostincrement
                2'b0,               // [18:17] 0
                write,              // [16] write
                2'd0,               // [15:14] target specific
                14'd0               // [13:0] 0
            };
            
            #10;
            Update_Reg(32'h04, write_data);   // data0 <= write_data
            #10;
            Update_Reg(32'h05, ptr);   // data0 <= write_data
            #10
            Update_Reg(32'h17, cmd);   // data0 <= write_data
            #10;
//            Read_Reg(32'h04, 32'd0, data0);
            #10;
        end
    endtask
    
    task automatic step();
        begin
            #100;
            Access_reg_command(.transfer(1'd1), .write(1'd1), .regno(16'h07b0), .write_data(32'd4));
            #100;
        end
    endtask
    

//    Hex Addr  -> Register Name
//    32'h04    -> data0
//    32'h05    -> data1

//    32'h10    -> dmcontrol
//    32'h11    -> dmstatus
//    32'h12    -> hartinfo
//    32'h16    -> abstractcs
//    32'h17    -> command

//    32'h70    -> custom0
//    32'h71    -> custom1
//    32'h72    -> custom2
//    32'h73    -> custom3
//    32'h74    -> custom4
//    32'h75    -> custom5
//    32'h76    -> custom6
        
    task automatic Read_Reg(input [31:0] IR_data, input [31:0] DR_data, output [31:0] DR_received);
        integer m;
        begin
            //================================================================
                    // Run-Test/Idle -> Shift-IR
                    TMS_Sequence(8'h03, 3);
                  
                    // Initiate Data Transfer: 8-bit
                    TDI_Transfer(IR_data, 8, rcvd_data_reg_2);           //update IR
                    
                    // Exit1-IR -> Update-IR
                    // Update-IR -> Run-Test/Idle
                    TMS_Sequence(8'h01, 1);
                    TMS_Sequence(8'h00, 1);
                    #(20*JTAG_CLK_HALF_PERIOD_NS);
            //================================================================
                    // Run-Test/Idle -> Shift-DR
                    TMS_Sequence(8'h01, 3);
                    // Transfer full 32-bit data
                    TDI_Transfer(DR_data, 32, rcvd_data_reg);      //update DR
                    // Exit1-DR -> Run-Test/Idle
                    TMS_Sequence(8'h01, 2);
                    
                    #(20*JTAG_CLK_HALF_PERIOD_NS);
            //================================================================
            DR_received = rcvd_data_reg;
            
        end
    endtask    
        
    task automatic Update_Reg(input [31:0] IR_data, input [31:0] DR_data);
        integer m;
        begin
            //================================================================
                    // Run-Test/Idle -> Shift-IR
                    TMS_Sequence(8'h03, 3);
                  
                    // Initiate Data Transfer: 8-bit
                    TDI_Transfer(IR_data, 8, rcvd_data_reg_2);           //update IR
                    
                    // Exit1-IR -> Update-IR
                    // Update-IR -> Run-Test/Idle
                    TMS_Sequence(8'h01, 1);
                    TMS_Sequence(8'h00, 1);
                    #(20*JTAG_CLK_HALF_PERIOD_NS);
            //================================================================
                    // Run-Test/Idle -> Shift-DR
                    TMS_Sequence(8'h01, 3);
                    // Transfer full 32-bit data
                    TDI_Transfer(DR_data, 32, rcvd_data_reg);      //update DR
                    // Exit1-DR -> Run-Test/Idle
                    TMS_Sequence(8'h01, 2);
                    
                    #(20*JTAG_CLK_HALF_PERIOD_NS);
            //================================================================
        end
    endtask    
    
    task automatic single_step();
    begin
        // Halt the core
        core_halt();
        #(10 * JTAG_CLK_HALF_PERIOD_NS);

        // Issue abstract command to write value 4 to dcsr.step (regno = 0x7b0)
        // This sets the step bit in DCSR
        Access_reg_command(.transfer(1'd1), .write(1'd1), .regno(16'h07b0), .write_data(32'h40008007));
        #(10 * JTAG_CLK_HALF_PERIOD_NS);

        // Resume execution to perform the step
        core_resume();
        #(10 * JTAG_CLK_HALF_PERIOD_NS);

        // Clear step bit to prevent further stepping
        core_resume_clr();
        #(10 * JTAG_CLK_HALF_PERIOD_NS);

        // Halt again for inspection or continuation
        //core_halt();
        //#(10 * JTAG_CLK_HALF_PERIOD_NS);
    end
    endtask

  `define FUNCTIONAL
  //`define DEBUG   
  
    integer logfile;
	initial begin
  	logfile = $fopen("/home/rclab/FINAL_PROJECT/RV32G_Core/RV32G_Core/RV32G_Debug/tb_result.log", "w");
	end
	
	
    initial begin
    clk_int <= 1'b0; clk_x2 <= 1'b0;
        #200
        // Initialize signals
        TDI = 1;
        TMS = 1;
        //RESET_BUTTON = 1;
        rst <= 1'b1; clk_int <= 1'b0; clk_x2 <= 1'b0;
        
        #1000 rst = 0;
        
        `ifdef DEBUG
        #(20*JTAG_CLK_HALF_PERIOD_NS);
        @(negedge TCK);
        // Test-Logic-Reset
        TMS_Sequence(8'h1F, 5);
        
        // Test-Logic-Reset --> Run-Test/Idle
        TMS_Sequence(8'h00, 1);
        Update_Reg(32'h10, 32'h20000003);
        
        
        /*single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();
        single_step();*/
        
        /*
        #100;
        core_resume();
        #100;
        core_resume_clr();*/
        
        
        //Update_Reg(32'h10, 32'h20000003);
        
        /*
        //Update_Reg(32'h10, 32'h20000003);   // reset command reg
        #200;
        core_halt();
        #100;
        Access_reg_command(.transfer(1'd1), .write(1'd1), .regno(16'h07b0), .write_data(32'h00000004));
        
        //Step-1
        core_resume();    
        #100;
        core_resume_clr();
        //#100;
        //core_halt();
        
        //Step-2
        #100;
        core_resume();
        #100;
        core_resume_clr();
        
        //Step-3
        #100;
        core_resume();
        #100;
        core_resume_clr();
        
        //Step-4
        #100;
        core_resume();
        #100;
        core_resume_clr();*/
        
//        core_halt();
//        core_reset_halt();
//        #100;
//        Access_reg_command(.transfer(1'd1), .write(1'd0), .regno(16'h1004), .write_data(32'h18));
//        #100;
//        Access_reg_command(.transfer(1'd1), .write(1'd1), .regno(16'h1004), .write_data(32'h18));
        
        
        
//        #100;
//        Access_mem_command(.write(1'd1), .aamsize(3'd2), .ptr(32'h0), .write_data(32'h12345678));
//        #1000;
//        Access_mem_command(.write(1'd0), .aamsize(3'd2), .ptr(32'h0), .write_data(32'h0));
//        #100;

//        step();
//        #1000;
//        step();
//        #1000;
//        step();
//        #1000;
//        step();
//        #1000;
//        step();
//        #1000;
//        step();
//        #1000;
        
//        core_resume();
        
//        Update_Reg(32'h10, 32'h00000001);   // deassert reset
       // Update_Reg(32'h04, 32'hFFFFFFFF);   // update data 0
       // Read_Reg(32'h04, 32'h00000000, read_data0);
        

//        Update_Reg(32'h10, 32'h00000000);   // deassert reset
//        Update_Reg(32'h04, 32'hFFFFFFFF);   // update data 0
//        Update_Reg(32'h10, 32'h00000001);   // deassert reset
//        Update_Reg(32'h04, 32'hFFFFFFFF);   // update data 0
        
        
//        #(100*JTAG_CLK_HALF_PERIOD_NS);
        //#50;
        //#5000;
        //$stop;
        
        //`ifdef FUNCTIONAL
         wait(done);
         $display("ENTERING DEBUG MODE");
         //$finish;
         //core_halt();
         //#100;
         
         //#500;
         //core_resume();
         //#100;
         //core_resume_clr();
         
         #100;
         core_halt();
         #500;
         Access_reg_command(.transfer(1'd1), .write(1'd0), .regno(16'h1004), .write_data(32'h18));
         #400;
         Access_reg_command(.transfer(1'd1), .write(1'd1), .regno(16'h1004), .write_data(32'h18));
         #400;
         Access_mem_command(.write(1'd0), .aamsize(3'd2), .ptr(32'h20000000), .write_data(32'h0));
         #400;
         Access_mem_command(.write(1'd1), .aamsize(3'd2), .ptr(32'h20000000), .write_data(32'h11223344));
         //#100;
         //Access_reg_command(.transfer(1'd1), .write(1'd0), .regno(16'h0300), .write_data(32'h18));
         //#100;
         //Access_reg_command(.transfer(1'd1), .write(1'd1), .regno(16'h07b0), .write_data(32'h00000004));
         
         $display("Total Number of Instruction = %d",num_inst);
         $display("Totoal Number of Cycles = %d",total_cycle_count);

         //   $finish;
        //`endif
        `else
            wait(done);
            $display("ENTERING DEBUG MODE");
        `endif    
            
    end
    
    always #50 clk_int <= ~clk_int;
    
    
    `ifdef FUNCTIONAL
      //---------------SELF TESTING LOGIC ------------------
      assign rf_mem_17 = TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.Pipeline.RF.MEM[17] ;
      assign rf_mem_10 = TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.Pipeline.RF.MEM[10] ; 



      assign ebreak_in_id = TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.Pipeline.debug_Controller.ebreak_in_id;
      assign ebreak_in_wb = TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.Pipeline.debug_Controller.ebreak_in_wb;
      assign pc_ebreak    = TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.Pipeline.ID.PC_ID;

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



assign pipeline_stall = TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.Pipeline.ID.IF_ID_Freeze;
assign state_dcache = TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.db1.dt1.drf0.state;
assign cache_flush = TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.db1.dt1.drf0.cache_flush;


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

`else

wire debug_mode;



assign debug_mode = TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.Pipeline.debug_Controller.debug_mode;
assign rf_mem_17 =  TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.Pipeline.RF.\MEM[17]__0 ;
assign rf_mem_10 =  TOP_JTAG_TB.riscv_platform.u_core_top.u_riscv_core.fdem.Pipeline.RF.\MEM[10]__0 ;
always @(posedge clk_int or posedge rst) begin
	       if(rst) begin 
		       done <= 1'b0;
		       pass <= 1'b0;
		       fail <= 1'b0;
	       end
	       else if((debug_mode) && (rf_mem_17 == 32'h00000005d)) begin
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
			       $fdisplay(logfile, "Test Case Passed in post Synthesis!");


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
`endif
    
endmodule

