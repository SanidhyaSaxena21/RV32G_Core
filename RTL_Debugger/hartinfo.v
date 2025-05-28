`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/12/2025 10:54:38 AM
// Design Name: 
// Module Name: hartinfo
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


module hartinfo (
    output wire [31:0] hartinfo_out
);
    localparam [3:0]    NSCRATCH    = 4'd0;     // no scratch registers available
    localparam          DATAACCESS  = 4'd0;     // 
    localparam [3:0]    DATASIZE    = 4'd2;
    localparam [11:0]   DATAADDR    = 12'd12;
    
    assign hartinfo_out = {
        8'd0,           // [31:24] 8 bits reserved
        NSCRATCH,       // [23:20] 4 bits nscratch = 1 (example)
        3'd0,           // [19:17] 3 bits reserved = 0
        DATAACCESS,     // [16]    1 bit dataaccess = 1 (example)
        DATASIZE,       // [15:12] 4 bits datasize = 2 (example)
        DATAADDR        // [11:0]  12 bits dataaddr = 12 (example)
    };
endmodule

