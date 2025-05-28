`timescale 1ns / 1ps


//(* keep_hierarchy = "yes" *)
module Direction_Predictor
(
    input CLK,
    input RST,
    input [31:0] PC,
    
    input BPU__Stall,
    
    output Branch_Taken,
    
    output  [10:0] PHT_Read_Index, 
    output reg [1:0] PHT_Read_Data,
    
    input [10:0] PHT_Write_Index,
    input [1:0] PHT_Write_Data,
    input PHT_Write_En,
    
    input GHR_Write_Data,
    input GHR_Write_En
);

reg [4:0] GHR;
wire [4:0] PC_XOR_GHR;

reg [10:0] PHT_Write_Index__reg;
reg [1:0] PHT_Write_Data__reg;
reg PHT_Write_En__reg;

reg GHR_Write_Data__reg;
reg GHR_Write_En__reg;

assign PC_XOR_GHR = PC[12:8] ^ GHR;


always @(posedge CLK) begin
    if(RST) begin
        PHT_Write_Index__reg <= 11'b0;
        PHT_Write_Data__reg <= 2'b0;
        PHT_Write_En__reg <= 1'b0;
        GHR_Write_Data__reg <= 1'b0;
        GHR_Write_En__reg <= 1'b0;
    end 
    else if(~BPU__Stall) begin
        PHT_Write_Index__reg <= PHT_Write_Index;
        PHT_Write_Data__reg <= PHT_Write_Data;
        PHT_Write_En__reg <= PHT_Write_En;
        GHR_Write_Data__reg <= GHR_Write_Data;
        GHR_Write_En__reg <= GHR_Write_En;
    end 
end


always @(posedge CLK) begin
    if(RST) 
        GHR <= 5'b0;
    else if((GHR_Write_En__reg) & (~BPU__Stall))
        GHR <= {GHR[3:0],GHR_Write_Data__reg};
end

assign PHT_Read_Index = {PC_XOR_GHR,PC[7:2]};

//Removing RESET from the Combo Logic
/*always @(*) begin
    if(RST) 
        PHT_Read_Index = 11'b0;
    else 
        PHT_Read_Index = {PC_XOR_GHR,PC[7:2]}; //PC[12:2]; //{PC_XOR_GHR,PC[7:2]}; 
end*/

assign Branch_Taken = PHT_Read_Data[1];

//Removing RESET from the Combo Logic
/*always @(*) begin
    if(RST) 
        Branch_Taken = 1'b0;
    else 
        Branch_Taken = PHT_Read_Data[1];
end*/

reg [1:0] PHT_mem [2047:0];
integer i;

always @(posedge CLK) begin
	if(RST) begin
		for(i=0;i<2048;i=i+1) begin
			PHT_mem[i] <= 2'b00;
		end
	end
	else if(~BPU__Stall) begin
		if(PHT_Write_En__reg) PHT_mem[PHT_Write_Index__reg] <= PHT_Write_Data__reg;
	end
end

always @(posedge CLK) begin
	if(RST) PHT_Read_Data <= 2'b00;
	else if(~BPU__Stall) PHT_Read_Data <= PHT_mem[PHT_Read_Index];
end

/*TSDN65LPLLA2048X2M8M PHT (
  .AA(PHT_Write_Index__reg), 			// Address of A: Addra[6:0]
  .DA(PHT_Write_Data__reg),			// Data in of A: douta[127:0]	
  .BWEBA({2{~PHT_Write_En__reg}}),			// Bit-Write ~en of A: {128{~en}}	
  .WEBA(~PHT_Write_En__reg),.CEBA(BPU__Stall),.CLKA(~CLK),	// Write-~en, Chip-~en, CLKA	
  .AB(PHT_Read_Index),			// Address of B: Addra[6:0]
  .DB(2'd0),			// Data in of B: douta[127:0]
  .BWEBB({2{1'b1}}),			// Bit-Write ~en of B: {128{~en}}
  .WEBB(1'b1),.CEBB(BPU__Stall),.CLKB(~CLK),	// Write-~en, Chip-~en, CLKB
  .AMA(11'd0),
  .DMA(2'd0),
  .BWEBMA({2{1'b1}}),
  .WEBMA(1'b1),.CEBMA(1'b1),
  .AMB(11'd0),
  .DMB(2'd0),
  .BWEBMB({2{1'b1}}),
  .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
  .QA(PHT_Read_Data),
  .QB()
	);*/
/*
dual_port_RAM #(.DATA_WIDTH(2),.ADDR_WIDTH(11)) PHT ( .clka(CLK),
              .rsta(1'b0),
              .wea((PHT_Write_En__reg) & (~BPU__Stall)),
              .addra(PHT_Write_Index__reg),
              .dina(PHT_Write_Data__reg),
              .douta(),
              .clkb(CLK),
              .rstb(RST),
              .ena(~BPU__Stall),
              .addrb(PHT_Read_Index),
              .web(1'b0),
              .dinb(),
              .doutb(PHT_Read_Data));*/


/*PHT_mem PHT ( .clka(CLK),
              .wea((PHT_Write_En__reg) & (~BPU__Stall)),
              .addra(PHT_Write_Index__reg),
              .dina(PHT_Write_Data__reg),
              .clkb(CLK),
              .rstb(RST),
              .enb(~BPU__Stall),
              .addrb(PHT_Read_Index),
              .doutb(PHT_Read_Data));*/

endmodule








