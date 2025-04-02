`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/16/2025 03:13:56 AM
// Design Name: 
// Module Name: irq_bit_sync
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


module irq_bit_sync
    #(
        parameter DEFAULT_LEVEL = 0,
        parameter DEPTH = 2
    )(
        // clock & reset
        input  wire                       aclk,
        input  wire                       areset,
        // bit to synch & synched
        input  wire                       bit_i,
        output wire                      bit_o
    );

    reg [DEPTH-1:0] sync;

    generate

    if (DEPTH<=2) begin: DEPTH_EQ_2
        always @ (posedge aclk or posedge areset) begin
            if (areset) sync <= {DEPTH{DEFAULT_LEVEL[0]}};
            else sync <= {sync[0],bit_i};
        end
    end else begin: DEPTH_GT_2
        always @ (posedge aclk or posedge areset) begin

            if (areset) sync <= {DEPTH{DEFAULT_LEVEL[0]}};
            else sync <= {sync[DEPTH-2:0],bit_i};
        end
    end
    endgenerate

    assign bit_o = sync[DEPTH-1];


endmodule
