`timescale 1ns / 1ps


(* keep_hierarchy = "yes" *)
module Branch_Target_Buff
(
    input CLK,
    input RST,
    
    input BPU__Stall,
    
    input [31:0] BTB_Read_Addr,
    output reg [31:0] BTB_Read_Data,
    output reg BTB_Hit,
    
    input [31:0] BTB_Write_Addr,
    input [31:0] BTB_Write_Data,
    input BTB_Write_En
    
);

reg [31:0] BTB_Read_Addr__reg;
reg [31:0] BTB_Write_Addr__reg;
reg [31:0] BTB_Write_Data__reg;
reg BTB_Write_En__reg;

wire [55:0] BTB_Read_Data_Set0_int;
wire [55:0] BTB_Read_Data_Set1_int;
wire [55:0] BTB_Read_Data_Set2_int;
wire [55:0] BTB_Read_Data_Set3_int;

wire [55:0] BTB_Read_Data_Set0;
wire [55:0] BTB_Read_Data_Set1;
wire [55:0] BTB_Read_Data_Set2;
wire [55:0] BTB_Read_Data_Set3;

wire BTB_Hit_Set0;
wire BTB_Hit_Set1;
wire BTB_Hit_Set2;
wire BTB_Hit_Set3;

wire [55:0] BTB_Write_Data_temp;
reg BTB_Write_En_Set0;
reg BTB_Write_En_Set1;
reg BTB_Write_En_Set2;
reg BTB_Write_En_Set3;

wire [1:0] LRU_Set; 

// BTB Valid bit logic
// We need to have this logic since we cannot read something from the memory
// until it is written 

reg [3:0] BTB_Read_Data_Valid_mem[127:0];
reg [3:0] BTB_Read_Data_Valid; 
integer i;

always @(posedge CLK or posedge RST) begin
    if(RST) begin
    	for(i=0;i<128;i=i+1) begin
	    BTB_Read_Data_Valid_mem[i] <= 4'd0;
	end
	BTB_Read_Data_Valid <= 4'd0;	
    end
    else if(~BPU__Stall) begin
	 if(BTB_Write_En__reg) begin
	     casex({BTB_Write_En_Set3,BTB_Write_En_Set2,BTB_Write_En_Set1,BTB_Write_En_Set0})
		     4'b0001: begin
			     BTB_Read_Data_Valid_mem[BTB_Write_Addr__reg[8:2]] <= 4'b0001;
		     end
		     4'b0010: begin
			     BTB_Read_Data_Valid_mem[BTB_Write_Addr__reg[8:2]] <= 4'b0010;
		     end
		     4'b0100: begin
			     BTB_Read_Data_Valid_mem[BTB_Write_Addr__reg[8:2]] <= 4'b0100;
		     end
		     4'b1000: begin
			     BTB_Read_Data_Valid_mem[BTB_Write_Addr__reg[8:2]] <= 4'b1000;
		     end
		     default: begin
			     BTB_Read_Data_Valid_mem[BTB_Write_Addr__reg[8:2]] <= 4'b0000;
		     end
	     endcase
	 end
	 else begin
		 BTB_Read_Data_Valid <= BTB_Read_Data_Valid_mem[BTB_Read_Addr[8:2]];
	 end
    end
    else begin
	   BTB_Read_Data_Valid <= 4'd0; 
    end
end




assign BTB_Hit_Set0 = ((BTB_Read_Data_Valid[0] == 1'b1) && (BTB_Read_Data_Set0[54:32] == BTB_Read_Addr__reg[31:9])) ? 1'b1 : 1'b0;
assign BTB_Hit_Set1 = ((BTB_Read_Data_Valid[1] == 1'b1) && (BTB_Read_Data_Set1[54:32] == BTB_Read_Addr__reg[31:9])) ? 1'b1 : 1'b0;
assign BTB_Hit_Set2 = ((BTB_Read_Data_Valid[2] == 1'b1) && (BTB_Read_Data_Set2[54:32] == BTB_Read_Addr__reg[31:9])) ? 1'b1 : 1'b0;
assign BTB_Hit_Set3 = ((BTB_Read_Data_Valid[3] == 1'b1) && (BTB_Read_Data_Set3[54:32] == BTB_Read_Addr__reg[31:9])) ? 1'b1 : 1'b0;

assign BTB_Write_Data_temp = {1'b1,BTB_Write_Addr__reg[31:9],BTB_Write_Data__reg};

always @(posedge CLK) begin
    if (RST) begin 
        BTB_Read_Addr__reg <= 32'h00000000;
        
        BTB_Write_Addr__reg <= 32'h00000000;
        BTB_Write_Data__reg <= 32'h00000000;
        BTB_Write_En__reg <= 1'b0;
    end
    else if(~BPU__Stall) begin
        BTB_Read_Addr__reg <= BTB_Read_Addr;
        
        BTB_Write_Addr__reg <= BTB_Write_Addr;
        BTB_Write_Data__reg <= BTB_Write_Data;
        BTB_Write_En__reg <= BTB_Write_En;
    end 
end

// Removing RESET from the Combo Logic
always @(*) begin
    /*if (RST) begin 
        BTB_Hit = 1'b0;
        BTB_Read_Data = 32'h00000000;
    end
    else*/ if (BTB_Hit_Set0 | BTB_Hit_Set1 | BTB_Hit_Set2 | BTB_Hit_Set3) begin
        BTB_Hit = 1'b1;
        case({BTB_Hit_Set3,BTB_Hit_Set2,BTB_Hit_Set1,BTB_Hit_Set0})
            4'b0001 : BTB_Read_Data = BTB_Read_Data_Set0[31:0];
            4'b0010 : BTB_Read_Data = BTB_Read_Data_Set1[31:0];
            4'b0100 : BTB_Read_Data = BTB_Read_Data_Set2[31:0];
            4'b1000 : BTB_Read_Data = BTB_Read_Data_Set3[31:0];
            default : BTB_Read_Data = 32'h00000000;        
        endcase
    end 
    else begin
        BTB_Hit = 1'b0;
        BTB_Read_Data = 32'h00000000;
    end
end

always @(*) begin
    /*if (RST | BPU__Stall) begin 
        BTB_Write_En_Set0 = 1'b0; BTB_Write_En_Set1 = 1'b0; BTB_Write_En_Set2 = 1'b0; BTB_Write_En_Set3 = 1'b0;
    end
    else*/ if (BTB_Write_En__reg) begin
        casex(LRU_Set)
            2'b00 : begin
                BTB_Write_En_Set0 = 1'b1; BTB_Write_En_Set1 = 1'b0; BTB_Write_En_Set2 = 1'b0; BTB_Write_En_Set3 = 1'b0;
            end
            2'b01 : begin
                BTB_Write_En_Set0 = 1'b0; BTB_Write_En_Set1 = 1'b1; BTB_Write_En_Set2 = 1'b0; BTB_Write_En_Set3 = 1'b0;
            end
            2'b10 : begin
                BTB_Write_En_Set0 = 1'b0; BTB_Write_En_Set1 = 1'b0; BTB_Write_En_Set2 = 1'b1; BTB_Write_En_Set3 = 1'b0;
            end
            2'b11 : begin
                BTB_Write_En_Set0 = 1'b0; BTB_Write_En_Set1 = 1'b0; BTB_Write_En_Set2 = 1'b0; BTB_Write_En_Set3 = 1'b1;
            end
            default : begin
                BTB_Write_En_Set0 = 1'b0; BTB_Write_En_Set1 = 1'b0; BTB_Write_En_Set2 = 1'b0; BTB_Write_En_Set3 = 1'b0;
            end       
        endcase
    end 
    else begin
        BTB_Write_En_Set0 = 1'b0; BTB_Write_En_Set1 = 1'b0; BTB_Write_En_Set2 = 1'b0; BTB_Write_En_Set3 = 1'b0;
    end
end


BTB_mem BTB_Set0( .clka(CLK),.rsta(RST),.wea(7'h00),.addra(BTB_Read_Addr[8:2]),.dina(56'h0),.douta(BTB_Read_Data_Set0),.ena(~BPU__Stall),
                  .clkb(CLK),.rstb(RST),.web({7{BTB_Write_En_Set0}}),.addrb(BTB_Write_Addr__reg[8:2]),.dinb(BTB_Write_Data_temp),.doutb());

BTB_mem BTB_Set1( .clka(CLK),.rsta(RST),.wea(7'h00),.addra(BTB_Read_Addr[8:2]),.dina(56'h0),.douta(BTB_Read_Data_Set1),.ena(~BPU__Stall),
                  .clkb(CLK),.rstb(RST),.web({7{BTB_Write_En_Set1}}),.addrb(BTB_Write_Addr__reg[8:2]),.dinb(BTB_Write_Data_temp),.doutb());
                        
BTB_mem BTB_Set2( .clka(CLK),.rsta(RST),.wea(7'h00),.addra(BTB_Read_Addr[8:2]),.dina(56'h0),.douta(BTB_Read_Data_Set2),.ena(~BPU__Stall),
                  .clkb(CLK),.rstb(RST),.web({7{BTB_Write_En_Set2}}),.addrb(BTB_Write_Addr__reg[8:2]),.dinb(BTB_Write_Data_temp),.doutb());                        

BTB_mem BTB_Set3( .clka(CLK),.rsta(RST),.wea(7'h00),.addra(BTB_Read_Addr[8:2]),.dina(56'h0),.douta(BTB_Read_Data_Set3),.ena(~BPU__Stall),
                  .clkb(CLK),.rstb(RST),.web({7{BTB_Write_En_Set3}}),.addrb(BTB_Write_Addr__reg[8:2]),.dinb(BTB_Write_Data_temp),.doutb()); 
             
/*
TSDN65LPLLA128X56M8F BTB_Set0 (
  .AA(BTB_Read_Addr[8:2]), 			// Address of A: Addra[6:0]
  .DA(56'd0),			// Data in of A: douta[127:0]	
  .BWEBA({56{1'b1}}),			// Bit-Write ~en of A: {128{~en}}	
  .WEBA(1'b1),.CEBA(BPU__Stall),.CLKA(CLK),	// Write-~en, Chip-~en, CLKA	
  .AB(BTB_Write_Addr__reg[8:2]),			// Address of B: Addra[6:0]
  .DB(BTB_Write_Data_temp),			// Data in of B: douta[127:0]
  .BWEBB({56{~BTB_Write_En_Set0}}),			// Bit-Write ~en of B: {128{~en}}
  .WEBB(~BTB_Write_En_Set0),.CEBB(BPU__Stall),.CLKB(CLK),	// Write-~en, Chip-~en, CLKB
  .AMA(7'd0),
  .DMA(56'd0),
  .BWEBMA({56{1'b1}}),
  .WEBMA(1'b1),.CEBMA(1'b1),
  .AMB(7'd0),
  .DMB(56'd0),
  .BWEBMB({56{1'b1}}),
  .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
  .QA(BTB_Read_Data_Set0),
  .QB()
	);
                        
TSDN65LPLLA128X56M8F BTB_Set1 (
  .AA(BTB_Read_Addr[8:2]), 			// Address of A: Addra[6:0]
  .DA(56'd0),			// Data in of A: douta[127:0]	
  .BWEBA({56{1'b1}}),			// Bit-Write ~en of A: {128{~en}}	
  .WEBA(1'b1),.CEBA(BPU__Stall),.CLKA(CLK),	// Write-~en, Chip-~en, CLKA	
  .AB(BTB_Write_Addr__reg[8:2]),			// Address of B: Addra[6:0]
  .DB(BTB_Write_Data_temp),			// Data in of B: douta[127:0]
  .BWEBB({56{~BTB_Write_En_Set1}}),			// Bit-Write ~en of B: {128{~en}}
  .WEBB(~BTB_Write_En_Set1),.CEBB(BPU__Stall),.CLKB(CLK),	// Write-~en, Chip-~en, CLKB
  .AMA(7'd0),
  .DMA(56'd0),
  .BWEBMA({56{1'b1}}),
  .WEBMA(1'b1),.CEBMA(1'b1),
  .AMB(7'd0),
  .DMB(56'd0),
  .BWEBMB({56{1'b1}}),
  .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
  .QA(BTB_Read_Data_Set1),
  .QB()
	);

TSDN65LPLLA128X56M8F BTB_Set2 (
  .AA(BTB_Read_Addr[8:2]), 			// Address of A: Addra[6:0]
  .DA(56'd0),			// Data in of A: douta[127:0]	
  .BWEBA({56{1'b1}}),			// Bit-Write ~en of A: {128{~en}}	
  .WEBA(1'b1),.CEBA(BPU__Stall),.CLKA(CLK),	// Write-~en, Chip-~en, CLKA	
  .AB(BTB_Write_Addr__reg[8:2]),			// Address of B: Addra[6:0]
  .DB(BTB_Write_Data_temp),			// Data in of B: douta[127:0]
  .BWEBB({56{~BTB_Write_En_Set2}}),			// Bit-Write ~en of B: {128{~en}}
  .WEBB(~BTB_Write_En_Set2),.CEBB(BPU__Stall),.CLKB(CLK),	// Write-~en, Chip-~en, CLKB
  .AMA(7'd0),
  .DMA(56'd0),
  .BWEBMA({56{1'b1}}),
  .WEBMA(1'b1),.CEBMA(1'b1),
  .AMB(7'd0),
  .DMB(56'd0),
  .BWEBMB({56{1'b1}}),
  .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
  .QA(BTB_Read_Data_Set2),
  .QB()
	);

TSDN65LPLLA128X56M8F BTB_Set3 (
  .AA(BTB_Read_Addr[8:2]), 			// Address of A: Addra[6:0]
  .DA(56'd0),			// Data in of A: douta[127:0]	
  .BWEBA({56{1'b1}}),			// Bit-Write ~en of A: {128{~en}}	
  .WEBA(1'b1),.CEBA(BPU__Stall),.CLKA(CLK),	// Write-~en, Chip-~en, CLKA	
  .AB(BTB_Write_Addr__reg[8:2]),			// Address of B: Addra[6:0]
  .DB(BTB_Write_Data_temp),			// Data in of B: douta[127:0]
  .BWEBB({56{~BTB_Write_En_Set3}}),			// Bit-Write ~en of B: {128{~en}}
  .WEBB(~BTB_Write_En_Set3),.CEBB(BPU__Stall),.CLKB(CLK),	// Write-~en, Chip-~en, CLKB
  .AMA(7'd0),
  .DMA(56'd0),
  .BWEBMA({56{1'b1}}),
  .WEBMA(1'b1),.CEBMA(1'b1),
  .AMB(7'd0),
  .DMB(56'd0),
  .BWEBMB({56{1'b1}}),
  .WEBMB(1'b1),.CEBMB(1'b1),.AWT(1'b0),.BIST(1'b0),.CLKM(1'b0),
  .QA(BTB_Read_Data_Set3),
  .QB()
	);*/

	/*
dual_port_RAM BTB_Set0( .clka(CLK),.rsta(RST),.wea(7'h00),.addra(BTB_Read_Addr[8:2]),.dina(56'h0),.douta(BTB_Read_Data_Set0),.ena(~BPU__Stall),
                  .clkb(CLK),.rstb(RST),.web({7{BTB_Write_En_Set0}}),.addrb(BTB_Write_Addr__reg[8:2]),.dinb(BTB_Write_Data_temp),.doutb());

dual_port_RAM BTB_Set1( .clka(CLK),.rsta(RST),.wea(7'h00),.addra(BTB_Read_Addr[8:2]),.dina(56'h0),.douta(BTB_Read_Data_Set1),.ena(~BPU__Stall),
                  .clkb(CLK),.rstb(RST),.web({7{BTB_Write_En_Set1}}),.addrb(BTB_Write_Addr__reg[8:2]),.dinb(BTB_Write_Data_temp),.doutb());
                        
dual_port_RAM BTB_Set2( .clka(CLK),.rsta(RST),.wea(7'h00),.addra(BTB_Read_Addr[8:2]),.dina(56'h0),.douta(BTB_Read_Data_Set2),.ena(~BPU__Stall),
                  .clkb(CLK),.rstb(RST),.web({7{BTB_Write_En_Set2}}),.addrb(BTB_Write_Addr__reg[8:2]),.dinb(BTB_Write_Data_temp),.doutb());                        

dual_port_RAM BTB_Set3( .clka(CLK),.rsta(RST),.wea(7'h00),.addra(BTB_Read_Addr[8:2]),.dina(56'h0),.douta(BTB_Read_Data_Set3),.ena(~BPU__Stall),
                  .clkb(CLK),.rstb(RST),.web({7{BTB_Write_En_Set3}}),.addrb(BTB_Write_Addr__reg[8:2]),.dinb(BTB_Write_Data_temp),.doutb()); 
*/               
BTB_PLRU PLRU( .CLK(CLK),
               .RST(RST),
               .BPU__Stall(BPU__Stall),         
               .BTB_Read_Addr__reg(BTB_Read_Addr__reg[8:2]),
               .BTB_Hit_Set0(BTB_Hit_Set0),
               .BTB_Hit_Set1(BTB_Hit_Set1),
               .BTB_Hit_Set2(BTB_Hit_Set2),
               .BTB_Hit_Set3(BTB_Hit_Set3),
               .Read_Access(BTB_Hit),
               .BTB_Write_Addr__reg(BTB_Write_Addr__reg[8:2]),
               .Write_Access(BTB_Write_En__reg),
               .LRU_Set(LRU_Set));                   
endmodule





































