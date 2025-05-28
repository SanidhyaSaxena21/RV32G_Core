`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/17/2025 01:09:02 PM
// Design Name: 
// Module Name: top
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


module top(
    output wire TCK,
    output wire TMS,
    output wire TDI,
    output wire TDO
    );
    
    
    wire rst, tck_int, tms_int, tdi_int, tdo_int;
    
    // Assign outputs for probing
    assign TCK = tck_int;
    assign TMS = tms_int;
    assign TDI = tdi_int;
    assign TDO = tdo_int;  // External TDO fed into internal net
    
    // Instantiate BSCANE2 for JTAG TAP access
    BSCANE2 #(
        .JTAG_CHAIN(1)  // Select JTAG chain (usually 1)
    ) bscane2_inst (
        .TCK(tck_int),
        .TMS(tms_int),
        .TDI(tdi_int),
        .TDO(tdo_int),  // Output from jtaglet should connect here
        .RESET(rst),
        .CAPTURE(),
        .DRCK(),
        .SEL(),
        .SHIFT(),
        .UPDATE()
    );
    
    
    // Instantiate jtaglet
    jtaglet #(
        .IR_LEN(4),
        .ID_PARTVER(4'h0),
        .ID_PARTNUM(16'h0000),
        .ID_MANF(11'h000),
        .USERDATA_LEN(32),
        .USEROP_LEN(8)
    ) jtaglet_inst (
        .tck(tck_int),
        .tms(tms_int),
        .tdi(tdi_int),
        .tdo(tdo_int),
        .trst(rst),
        .userData_in(userData_in),
        .userData_out(userData_out),
        .userOp(userOp),
        .userOp_ready(userOp_ready)
    );
endmodule
