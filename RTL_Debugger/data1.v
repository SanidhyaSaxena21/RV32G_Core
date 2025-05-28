`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/21/2025 02:38:02 PM
// Design Name: 
// Module Name: data1
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


module data1(
        input wire clk,
        input wire rst_n,
        
        input wire [7:0]    addr,
        input wire [3:0]    TAP_state,
//        input wire          new_cmd_flag,
//        input wire          aampostincrement,
//        input wire [7:0]    cmdtype,
        input wire          wr_en,
        
        input wire [31:0]   temp,
        input wire [2:0]    aamsize,        
        
        output reg [31:0]   data1_out,
        output reg          data1_written
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
    
    
    reg data1_selector;
    wire [31:0] wdata;
    wire [31:0] data1_inc;
//    wire wr_en;
    
    assign wdata =  (data1_selector == 1'd0) ?  temp : (data1_out + data1_inc);

//    assign data1_inc =  (aamsize == 3'd0) ?  32'd1 : 
//                        (aamsize == 3'd1) ?  32'd2 : 
//                        (aamsize == 3'd2) ?  32'd4 : 
//                        (aamsize == 3'd3) ?  32'd8 :  
//                        (aamsize == 3'd4) ?  32'd16 : 0;  
                        
    assign data1_inc =  (aamsize == 3'd0) ?  32'd1 : 
                        (aamsize == 3'd1) ?  32'd2 : 
                        (aamsize == 3'd2) ?  32'd4 : 0;  
                                                
//    assign wr_en =  ((addr == 8'h05) && (TAP_state == UPDATE_DR)) ||
//                    (new_cmd_flag && (aampostincrement == 1'b1) && (cmdtype == 8'd2));                
                                                
    
    always @(posedge clk) begin
        if(!rst_n)begin
            data1_out <= 0;
            data1_written <= 0;
        end
        else begin  
            if(wr_en) begin
                data1_out       <= wdata;
                data1_written   <= 1;
            end
            else 
                data1_written   <= 0;
        end
    end
    
    always @(*) begin
        if(addr==8'h05 && TAP_state == UPDATE_DR)       // selecting temp as data to be written
            data1_selector <= 1'd0;
        else
            data1_selector <= 1'd1;                     // selecting data1 + [amsize] as data to be written
    end
endmodule
