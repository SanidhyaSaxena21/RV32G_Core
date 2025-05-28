`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/10/2025 12:27:03 PM
// Design Name: 
// Module Name: ID_Reg
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


module ID_Reg(
        input wire Data_in,
        input wire Shift_DR,
        input wire Clock_DR,
        input wire TCK,
        
        output wire Data_out 
    );
    
    localparam IDCODE = 32'h1234567F; // Artix-7 IDCODE
//    localparam IDCODE = 32'h00000003; // Artix-7 IDCODE
    
    reg [31:0] ID_reg = 32'h1;            // 0th bit should be hardwired to 1
    assign Data_out = ID_reg[0];


//    initial ID_reg = 32'h1;  // Use initial block for initialization
    
    always @(negedge TCK)begin //changed-toms
        if (Clock_DR)
            ID_reg <= (Shift_DR == 0) ? (IDCODE | 32'h1) : {Data_in , ID_reg[31:1]};
    end
endmodule
