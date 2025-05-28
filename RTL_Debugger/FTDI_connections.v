`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/17/2025 03:33:52 PM
// Design Name: 
// Module Name: FTDI_connections
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
`define DEBUG_JTAG

module FTDI_connections(
   
    `ifdef DEBUG_JTAG
        output wire TDO,
        input wire TDI,
        input wire TMS,
        input wire TCK,
    `else
        output wire TDO,
        output wire TDI,
        output wire TMS,
        output wire TCK,
    `endif
    
    input clock_100Mhz, // 100 Mhz clock source on Basys 3 FPGA
    input reset, // reset
    input [3:0] switch,     // to control what to print on seven segment
    output wire [3:0] Anode_Activate, // anode signals of the 7-segment LED display
    output wire [6:0] LED_out,// cathode patterns of the 7-segment LED display
        
    output wire [3:0] state_out, // TAP state
    output wire rst_out,
    output wire [5:0] Instr   //Instr
    );
    
    wire TDO_internal, TDI_internal, TMS_internal, TCK_internal;
    reg [15:0] seven_seg_data;
    
    always @(*) begin
        case (switch)
            4'd0:  seven_seg_data = {12'b0, state_out};   // Display 0
            4'd1:  seven_seg_data = {10'b0, Instr};   // Display 1
            4'd2:  seven_seg_data = 16'd2;   // Display 2
            4'd3:  seven_seg_data = 16'd3;   // Display 3
            4'd4:  seven_seg_data = 16'd4;   // Display 4
            4'd5:  seven_seg_data = 16'd5;   // Display 5
            4'd6:  seven_seg_data = 16'd6;   // Display 6
            4'd7:  seven_seg_data = 16'd7;   // Display 7
            4'd8:  seven_seg_data = 16'd8;   // Display 8
            4'd9:  seven_seg_data = 16'd9;   // Display 9
            4'd10: seven_seg_data = 16'd10;  // Display 10
            4'd11: seven_seg_data = 16'd11;  // Display 11
            4'd12: seven_seg_data = 16'd12;  // Display 12
            4'd13: seven_seg_data = 16'd13;  // Display 13
            4'd14: seven_seg_data = 16'd14;  // Display 14
            4'd15: seven_seg_data = 16'd15;  // Display 15
            default: seven_seg_data = 16'd0; // Default to 0
        endcase
        
    end
    
    
    
    // Assign external JTAG signals to internal wires for probing
    `ifdef DEBUG_JTAG
        assign TDO = TDO_internal;
        assign TDI_internal = TDI;
        assign TMS_internal = TMS;
        assign TCK_internal = TCK;
    `else
        // Assign external JTAG signals to internal wires for probing
        assign TDO = TDO_internal;
        assign TDI = TDI_internal;
        assign TMS = TMS_internal;
        assign TCK = TCK_internal;
    `endif
    
    
    Display display_inst (
        .clock_100Mhz(clock_100Mhz), // Connect to 100 MHz clock
        .reset(reset),               // Connect to reset signal
        .sw(seven_seg_data),                     // Connect to 4-bit switch input
        .Anode_Activate(Anode_Activate), // 4-bit anode activation output
        .LED_out(LED_out)            // 7-bit cathode pattern output
    );
    
     // Instantiate JTAG module
    JTAG #(
        .WIDTH(5)  // Set IR width
    ) jtag_inst (
        .TDO(TDO_internal),
        .TDI(TDI_internal),
        .TMS(TMS_internal),
        .TCK(TCK_internal),
        .state_out(state_out),
        .rst_out(rst_out),
        .probing_out(Instr)
    );
    
    `ifndef DEBUG_JTAG
        // BSCANE2 Instantiation
        BSCANE2 #(.JTAG_CHAIN(2)) bscane2_inst (
            .TCK(TCK_internal),
            .TDI(TDI_internal),
            .TDO(TDO_internal),
            .TMS(TMS_internal),
            .SEL()
        );
    `endif
    
    
endmodule
