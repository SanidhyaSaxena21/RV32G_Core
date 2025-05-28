`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/21/2025 02:37:53 PM
// Design Name: 
// Module Name: data0
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


module data0(
        input wire clk,
        input wire rst_n,
        
        input wire [7:0]    addr,
        input wire [3:0]    TAP_state,
//        input wire          new_cmd_flag,
//        input wire          write,
//        input wire          transfer,
        input wire [7:0]    cmdtype,
        input wire          wr_en,
        
        input wire [31:0]   temp,
        input wire [31:0]   regno_data,         //data from the memory pointed by regno
        input wire [31:0]   data1_data,          //data from the memory pointed by data1
        
        output reg [31:0]   data0_out,
        output wire          data0_written
//        output reg          data0_written
    );
    localparam TEST_LOGIC_RESET = 4'd0,
               RUN_TEST_IDLE    = 4'd1,
               SELECT_DR_SCAN   = 4'd2,
               CAPTURE_DR       = 4'd3,
               SHIFT_DR         = 4'd4,
               EXIT1_DR         = 4'd5,
               PAUSE_DR         = 4'd6,
               EXIT2_DR         = 4'd7,
               UPDATE_DR        = 4'd8,
               
               SELECT_IR_SCAN   = 4'd9,
               CAPTURE_IR       = 4'd10,
               SHIFT_IR         = 4'd11,
               EXIT1_IR         = 4'd12,
               PAUSE_IR         = 4'd13,
               EXIT2_IR         = 4'd14,
               UPDATE_IR        = 4'd15;
    
    
    reg [1:0] data0_selector;
    wire [31:0] wdata;
    reg wr_en_flopped;
//    wire wr_en;
    assign data0_written = !wr_en_flopped && wr_en;
    assign wdata =  (data0_selector == 2'd0) ?  temp :          // Select temp when JTAG writes to addr 0x04
                    (data0_selector == 2'd1) ?  regno_data :    // Select regno_data when cmdtype == 0 (Access Register)
                                                data1_data;     // Default: select data1_data (Access Mem
                                                
                                                
//    assign wr_en =  ((addr == 8'h04) && (TAP_state == UPDATE_DR)) ||
//                    (new_cmd_flag && (write == 1'b0) && (transfer == 1'b1) && (cmdtype == 8'd0)) ||
//                    (new_cmd_flag && (cmdtype == 8'd2));                

    always @(posedge clk) begin
        wr_en_flopped <= wr_en;
        if(!rst_n)begin
            data0_out <= 0;
//            data0_written <= 0;
        end
        else begin
            if(wr_en) begin
                data0_out       <= wdata;
                //$display("Received Data: %h",wdata);
//                data0_written   <= 1;
            end
//            else 
//                data0_written   <= 0;
        end
    end
    
    always @(*) begin
        if(addr==8'h04 && TAP_state == UPDATE_DR)       // selecting temp as data to be written
            data0_selector <= 2'd0;
        else if (cmdtype == 8'h0)                       // selecting [regno] as data to be written
            data0_selector <= 2'd1;
        else
            data0_selector <= 2'd2;                     // selecting [data1] as data to be written
    end
endmodule
