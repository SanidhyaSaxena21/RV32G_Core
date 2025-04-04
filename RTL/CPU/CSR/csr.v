`timescale 1ns / 1ps

`include "/home/rclab/FINAL_PROJECT/RV32G_Core/RV32G_Core/RTL/DEFINES/defines.v"




module csr #( parameter XLEN = 32,
              parameter SB_LENGTH = 4*(XLEN+1)+1)
(
    input clk,
    input rst,
    // Interrupts
    input  wire                    ext_irq,
    input  wire                    sw_irq,
    input  wire                    timer_irq,
    
    input [11:0] csr_adr_wr,
    input [31:0] csr_wrdata,
    input csr_write_en,
    
    input [11:0] csr_adr_rd,
    output reg [31:0] csr_rddata,
    output reg [31:0] csr_mtvec,
    output reg [31:0] csr_mstatus,
    output reg [31:0] csr_mie,
    output reg [31:0] csr_mip,
    output reg [31:0] csr_mepc,
    input [SB_LENGTH-1:0]  sb_csr,
    output [`CSR_SB_W-1:0] csr_pmp_sb,

    //input ctrl_clr_meip,
    output mtie,
    
    input mret,
    input badaddr,
    input trap_en,
    //input mepc_res,
    input addr_exception,    
    input freeze, 
    
    input FPU_Inst,
    input [4:0] FPU_flags,
    output [2:0] frm,
    output cache_flush_csr,
    input flush_csr_clr,

    // SATP Register
    output reg [31:0] csr_satp,
    
    input [31:0] pc_id_ex
          
);


//reg [31:0] csr_mepc;   
reg [31:0] csr_mepc_shadow;   
//reg [31:0] csr_mstatus;  
reg [31:0] csr_mcause;
reg [31:0] csr_mtval;
//reg [31:0] csr_mie;
reg [31:0] csr_misa;
reg [31:0] csr_medeleg;
reg [31:0] csr_mideleg;
reg [31:0] csr_mbadaddr;
reg [1:0] cur_prev_mode; 
reg mret1;

reg [31:0] csr_fcsr;

//Custo CSR
reg [31:0] csr_cache_flush;

wire [XLEN-1:0] csr_mepc_val;
wire [XLEN-1:0] csr_mstatus_val;
wire [XLEN-1:0] csr_mcause_val;
wire [XLEN-1:0] csr_mtval_val;
wire mepc_wr;
wire mstatus_wr;
wire mcause_wr;
wire mtval_wr;
wire ctrl_clr_meip;

// Physical Memory Protection (PMP)
reg [`XLEN-1:0] pmpcfg0;       // 0x3A0    MRW
reg [`XLEN-1:0] pmpcfg1;       // 0x3A1    MRW
reg [`XLEN-1:0] pmpcfg2;       // 0x3A2    MRW
reg [`XLEN-1:0] pmpcfg3;       // 0x3A3    MRW
reg [`XLEN-1:0] pmpaddr0;      // 0x3B0    MRW
reg [`XLEN-1:0] pmpaddr1;      // 0x3B1    MRW
reg [`XLEN-1:0] pmpaddr2;      // 0x3B2    MRW
reg [`XLEN-1:0] pmpaddr3;      // 0x3B3    MRW
reg [`XLEN-1:0] pmpaddr4;      // 0x3B4    MRW
reg [`XLEN-1:0] pmpaddr5;      // 0x3B5    MRW
reg [`XLEN-1:0] pmpaddr6;      // 0x3B6    MRW
reg [`XLEN-1:0] pmpaddr7;      // 0x3B7    MRW
reg [`XLEN-1:0] pmpaddr8;      // 0x3B8    MRW
reg [`XLEN-1:0] pmpaddr9;      // 0x3B9    MRW
reg [`XLEN-1:0] pmpaddr10;     // 0x3BA    MRW
reg [`XLEN-1:0] pmpaddr11;     // 0x3BB    MRW
reg [`XLEN-1:0] pmpaddr12;     // 0x3BC    MRW
reg [`XLEN-1:0] pmpaddr13;     // 0x3BD    MRW
reg [`XLEN-1:0] pmpaddr14;     // 0x3BE    MRW
reg [`XLEN-1:0] pmpaddr15;     // 0x3BF    MRW

assign mtie = csr_mstatus[3] && csr_mie[7];                         //assign mtie = csr_mie[7];

assign frm = csr_fcsr[7:5];

assign csr_wr_en = (csr_write_en & ~freeze) ;

// IRQ Inputs to Pulse
    irq_bit_sync
    #(
        .DEPTH (2)
    )
    ext_irq_synchro
    (
        .aclk    (clk),
        .areset  (rst),
        .bit_i   (ext_irq),
        .bit_o   (ext_irq_sync)
    );

    irq_bit_sync
    #(
        .DEPTH (2)
    )
    timer_irq_synchro
    (
        .aclk    (clk),
        .areset  (rst),
        .bit_i   (timer_irq),
        .bit_o   (timer_irq_sync)
    );

    irq_bit_sync
    #(
        .DEPTH (2)
    )
    sw_irq_synchro
    (
        .aclk    (clk),
        .areset  (rst),
        .bit_i   (sw_irq),
        .bit_o   (sw_irq_sync)
    );
    

irq_pulser eirq_pulse 
    (
    .aclk    (clk),
    .areset (rst),
    .intp    (ext_irq_sync),
    .pulse   (ext_irq_pulse)
    );
irq_pulser tirq_pulse 
    (
    .aclk    (clk),
    .areset (rst),
    .intp    (timer_irq_sync),
    .pulse   (timer_irq_pulse)
    );

irq_pulser sirq_pulse 
    (
    .aclk    (clk),
    .areset (rst),
    .intp    (sw_irq_sync),
    .pulse   (sw_irq_pulse)
    );

reg sw_irq_sync_neg;
wire sw_irq_sync_neg_pulse;

assign sw_irq_sync_neg_pulse = (sw_irq_sync_neg & (~sw_irq_pulse));

always @(posedge clk or posedge rst) begin
  if(rst) begin
    sw_irq_sync_neg <= 1'b0;
  end
  else begin
    sw_irq_sync_neg <= sw_irq_pulse;
  end
end
/////////////////////////////////////////////////////////////////////

// MEPC - 0x341

////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
    if(rst) 
        csr_mepc = 32'b0;
    else if(mepc_wr) begin
        csr_mepc <= csr_mepc_val;
    end
    else if(/*trap_en || */( csr_wr_en && (csr_adr_wr == `mepc)))
        csr_mepc <= csr_wrdata;                                     // when exception happens store pc to mepd which is csr[0]
    //else if(mepc_res)          
        //csr_mepc <= csr_mepc_shadow;                                // restore MEPC to mepc_shadow value
end


always @(posedge clk) begin
    if(rst) 
        csr_mepc_shadow = 32'b0;
    else if(trap_en || ( csr_wr_en && (csr_adr_wr == `mepc)))
        csr_mepc_shadow <= csr_mepc;                                // when exception happens store pc to mepd which is csr[0]
    else if( csr_wr_en && (csr_adr_wr == `mepc_shadow))
        csr_mepc_shadow <= csr_wrdata;                              // when exception happens store pc to mepd which is csr[0]
end

/////////////////////////////////////////////////////////////////////

// MSTATUS - 0x300

////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
    if(rst) 
        csr_mstatus = `mstatus_default;
    /*else if(mret) begin
        case(csr_mstatus[12:11])
            2'b00 : csr_mstatus <= {csr_mstatus[31:13],2'b11,csr_mstatus[10:8],1'b1,csr_mstatus[6:1],csr_mstatus[7]};
            2'b01 : csr_mstatus <= {csr_mstatus[31:13],2'b11,csr_mstatus[10:8],1'b1,csr_mstatus[6:2],csr_mstatus[7],csr_mstatus[0]};
            2'b10 : csr_mstatus <= {csr_mstatus[31:13],2'b11,csr_mstatus[10:8],1'b1,csr_mstatus[6:3],csr_mstatus[7],csr_mstatus[1:0]};
            2'b11 : csr_mstatus <= {csr_mstatus[31:13],2'b11,csr_mstatus[10:8],1'b1,csr_mstatus[6:4],csr_mstatus[7],csr_mstatus[2:0]};
        endcase
    end  */
    else if(mstatus_wr) begin
        csr_mstatus <= csr_mstatus_val;
    end   
    else if( csr_wr_en && (csr_adr_wr == `mstatus))
        csr_mstatus <= csr_wrdata;  
end


/////////////////////////////////////////////////////////////////////

// MTVEC - 0x305

////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
    if(rst) 
        csr_mtvec = `mtvec_default;
    else if(csr_wr_en && (csr_adr_wr == `mtvec))
        csr_mtvec <= {csr_wrdata[31:0]};                      // lower two bits should be zero 
end


/////////////////////////////////////////////////////////////////////

// MCAUSE - 0x342

////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
    if(rst) 
        csr_mcause <= `mcause_default;
    else if(mcause_wr) begin
        csr_mcause <= csr_mcause_val;
    end
    else if(addr_exception)
            csr_mcause <= 32'b01;
    else if(csr_wr_en && (csr_adr_wr == `mcause))
        csr_mcause <= csr_wrdata;                                   // ***Mcause is WLRL not required to be updated in hardware 
end


/////////////////////////////////////////////////////////////////////

// MTVAL - 0x343

////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
    if(rst) 
        csr_mtval <= `mtval_default;
    else if(mtval_wr) begin
        csr_mtval <= csr_mtval_val;
    end
    else if(csr_wr_en && (csr_adr_wr == `mtval))
        csr_mtval <= csr_wrdata;                                   
end

/////////////////////////////////////////////////////////////////////

// MIE - 0x304

////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
    if(rst) 
        csr_mie = `mie_default;
    else if( csr_wr_en && (csr_adr_wr == `mie))
        csr_mie <= ((csr_wrdata) & (`mie_mask)) ;                  
end

/////////////////////////////////////////////////////////////////////

// MIP - 0x344

////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
    if(rst) begin
        csr_mip = `mip_default;
    end
    else begin
      if (ext_irq_pulse || timer_irq_pulse || sw_irq_pulse || ctrl_clr_meip) begin
        // external interrupt enable && external interrupt pin asserted
        if (ext_irq_pulse) begin
            csr_mip[11] <= 1'b1;
        end else if (ctrl_clr_meip) begin
            csr_mip[11] <= 1'b0;
        end
        // software interrupt enable && software interrupt pin asserted
        if (sw_irq_pulse) begin
            csr_mip[3] <= 1'b1;
        end else begin
            csr_mip[3] <= 1'b0;
        end
        // timer interrupt enable && timer interrupt pin asserted
        if (timer_irq_pulse) begin
            csr_mip[7] <= 1'b1;
        end else begin
            csr_mip[7] <= 1'b0;
        end
      end
      else if(sw_irq_sync_neg_pulse) begin
        csr_mip[3] <= 1'b0;
      end
      else if( csr_wr_en && (csr_adr_wr == `mip)) begin
          csr_mip <= ((csr_wrdata) & (`mip_mask)) ;
      end
    end      
end

/////////////////////////////////////////////////////////////////////

// MBADDR_DEFAULT = 32'h0

////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
    if(rst) 
        csr_mbadaddr = `mbadaddr_default;
    else if(badaddr || addr_exception)
        csr_mbadaddr <= pc_id_ex ;                                  
    else if( csr_wr_en && (csr_adr_wr == `mbadaddr))
        csr_mbadaddr <= csr_wrdata; 
end


/////////////////////////////////////////////////////////////////////

// MISA - 0x301

////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
    if(rst) 
        csr_misa = `misa_default;
    else if( csr_wr_en && (csr_adr_wr == `misa))
        csr_misa <= (csr_wrdata[29:0] && `misa_mask) ;          
end


/////////////////////////////////////////////////////////////////////

// MEDELEG_DEFAULT  

////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
    if(rst) 
        csr_medeleg = `medeleg_default;                             //  this will be writable as other modes will be implemented 
end


always @(posedge clk) begin
    if(rst) 
        csr_mideleg = `mideleg_default;                             //  this will be writable as other modes will be implemented 
end

/////////////////////////////////////////////////////////////////////

// FCSR - 0x003 FRM - 0x002 FFLAGS - 0x001

////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
    if(rst) 
        csr_fcsr = `mtvec_default;
    else if(csr_wr_en && (csr_adr_wr == `fcsr))
        csr_fcsr[7:0] <= csr_wrdata; 
    else if(csr_wr_en && (csr_adr_wr == `frm))
        csr_fcsr[7:5] <= csr_wrdata; 
    else if(csr_wr_en && (csr_adr_wr == `fflags))
        csr_fcsr[4:0] <= csr_wrdata; 
    else if(FPU_Inst)
        csr_fcsr[4:0] <= FPU_flags; 
end


/////////////////////////////////////////////////////////////////////

// SATP - 0x180
// MODE[31] | ASID[30:22] | PPN[21:0]
// MODE: 
// 0 --> Bare System (No Translation Supported)
// 1 --> Sv32 SUpported
//

////////////////////////////////////////////////////////////////////

//reg [31:0] csr_satp;
//wire csr_satp_mode;

always @(posedge clk) begin
    if(rst) 
        csr_satp <= `satp_default;
    else if(csr_wr_en && (csr_adr_wr == `satp))
        csr_satp <= {csr_satp_mode,csr_wrdata[30:0]};                      // lower two bits should be zero 
end

assign csr_satp_mode = (csr_wrdata[30:0] == 31'd0) ? 1'b0 : 1'b1;



/////////////////////////////////////////////////////////////////////
// CUSTOM CSR////////////////////////////////////////////////////////

always @(posedge clk) begin
    if(rst) 
        csr_cache_flush = 32'd0;
      else if( csr_wr_en && (csr_adr_wr == `cache_flush)) begin
        csr_cache_flush <= (csr_wrdata[29:0] && `cache_flush_mask) ;
      end
      else if(flush_csr_clr) csr_cache_flush <= 32'd0;  
end

assign cache_flush_csr = csr_cache_flush[0];
/////////////////////////////////////////////////////////////////////

// Physical Memory Attributes (PMA/PMP) CSR
// Parameter NB_PMP_REGION: No of PMP Region
//

////////////////////////////////////////////////////////////////////


parameter NB_PMP_REGION = 16;      // Physical memory divided into 16 Regions 
//
// MPU Support 
// 0 - Not Supported 
// 1 - Supported, fixed at synth time
parameter MPU_SUPPORT = 2;

//Max PMP Region Supported by core
parameter MAX_PMP_REGION = 16;

//PMP Value at initialisation
parameter PMPCFG0_INIT   = 32'h0;
parameter PMPCFG1_INIT   = 32'h0;
parameter PMPCFG2_INIT   = 32'h0;
parameter PMPCFG3_INIT   = 32'h0;
parameter PMPADDR0_INIT  = 32'h0;
parameter PMPADDR1_INIT  = 32'h0;
parameter PMPADDR2_INIT  = 32'h0;
parameter PMPADDR3_INIT  = 32'h0;
parameter PMPADDR4_INIT  = 32'h0;
parameter PMPADDR5_INIT  = 32'h0;
parameter PMPADDR6_INIT  = 32'h0;
parameter PMPADDR7_INIT  = 32'h0;
parameter PMPADDR8_INIT  = 32'h0;
parameter PMPADDR9_INIT  = 32'h0;
parameter PMPADDR10_INIT = 32'h0;
parameter PMPADDR11_INIT = 32'h0;
parameter PMPADDR12_INIT = 32'h0;
parameter PMPADDR13_INIT = 32'h0;
parameter PMPADDR14_INIT = 32'h0;
parameter PMPADDR15_INIT = 32'h0;

//Virtual Memory Support 
parameter MMU_SUPPORT = 1;

assign csr_pmp_sb[`CSR_SB_PMPCFG0+:XLEN] = pmpcfg0;
assign csr_pmp_sb[`CSR_SB_PMPCFG1+:XLEN] = pmpcfg1;
assign csr_pmp_sb[`CSR_SB_PMPCFG2+:XLEN] = pmpcfg2;
assign csr_pmp_sb[`CSR_SB_PMPCFG3+:XLEN] = pmpcfg3;

assign csr_pmp_sb[`CSR_SB_PMPADDR0 +:XLEN] = pmpaddr0;
assign csr_pmp_sb[`CSR_SB_PMPADDR1 +:XLEN] = pmpaddr1;
assign csr_pmp_sb[`CSR_SB_PMPADDR2 +:XLEN] = pmpaddr2;
assign csr_pmp_sb[`CSR_SB_PMPADDR3 +:XLEN] = pmpaddr3;
assign csr_pmp_sb[`CSR_SB_PMPADDR4 +:XLEN] = pmpaddr4;
assign csr_pmp_sb[`CSR_SB_PMPADDR5 +:XLEN] = pmpaddr5;
assign csr_pmp_sb[`CSR_SB_PMPADDR6 +:XLEN] = pmpaddr6;
assign csr_pmp_sb[`CSR_SB_PMPADDR7 +:XLEN] = pmpaddr7;
assign csr_pmp_sb[`CSR_SB_PMPADDR8 +:XLEN] = pmpaddr8;
assign csr_pmp_sb[`CSR_SB_PMPADDR9 +:XLEN] = pmpaddr9;
assign csr_pmp_sb[`CSR_SB_PMPADDR10+:XLEN] = pmpaddr10;
assign csr_pmp_sb[`CSR_SB_PMPADDR11+:XLEN] = pmpaddr11;
assign csr_pmp_sb[`CSR_SB_PMPADDR12+:XLEN] = pmpaddr12;
assign csr_pmp_sb[`CSR_SB_PMPADDR13+:XLEN] = pmpaddr13;
assign csr_pmp_sb[`CSR_SB_PMPADDR14+:XLEN] = pmpaddr14;
assign csr_pmp_sb[`CSR_SB_PMPADDR15+:XLEN] = pmpaddr15;

/////////////////////////////////////////////////////////////////////

// pmpcfg0/3 - 0x3A0-0x3A3

////////////////////////////////////////////////////////////////////

always @(posedge clk or posedge rst) begin
  if(rst) begin
    pmpcfg0 <= PMPCFG0_INIT;
    pmpcfg1 <= PMPCFG1_INIT;
    pmpcfg2 <= PMPCFG2_INIT;
    pmpcfg3 <= PMPCFG3_INIT;
  end
  else if (MPU_SUPPORT > 1) begin
    if (csr_wr_en) begin
      if (csr_adr_wr == `PMPCFG0 && NB_PMP_REGION >= 1) begin
        if(!pmpcfg0[0*8+7]) pmpcfg0[0+:8]  <= csr_wrdata[0+:8];
        if(!pmpcfg0[1*8+7]) pmpcfg0[8+:8]  <= csr_wrdata[8+:8];
        if(!pmpcfg0[2*8+7]) pmpcfg0[16+:8] <= csr_wrdata[16+:8];
        if(!pmpcfg0[3*8+7]) pmpcfg0[24+:8] <= csr_wrdata[24+:8];
      end

      if (csr_adr_wr == `PMPCFG0 && NB_PMP_REGION >= 5) begin
        if(!pmpcfg1[0*8+7]) pmpcfg1[0+:8]  <= csr_wrdata[0+:8];
        if(!pmpcfg1[1*8+7]) pmpcfg1[8+:8]  <= csr_wrdata[8+:8];
        if(!pmpcfg1[2*8+7]) pmpcfg1[16+:8] <= csr_wrdata[16+:8];
        if(!pmpcfg1[3*8+7]) pmpcfg1[24+:8] <= csr_wrdata[24+:8];
      end

      if (csr_adr_wr == `PMPCFG0 && NB_PMP_REGION >= 9) begin
        if(!pmpcfg2[0*8+7]) pmpcfg2[0+:8]  <= csr_wrdata[0+:8];
        if(!pmpcfg2[1*8+7]) pmpcfg2[8+:8]  <= csr_wrdata[8+:8];
        if(!pmpcfg2[2*8+7]) pmpcfg2[16+:8] <= csr_wrdata[16+:8];
        if(!pmpcfg2[3*8+7]) pmpcfg2[24+:8] <= csr_wrdata[24+:8];
      end

      if (csr_adr_wr == `PMPCFG0 && NB_PMP_REGION >= 13) begin
        if(!pmpcfg3[0*8+7]) pmpcfg3[0+:8]  <= csr_wrdata[0+:8];
        if(!pmpcfg3[1*8+7]) pmpcfg3[8+:8]  <= csr_wrdata[8+:8];
        if(!pmpcfg3[2*8+7]) pmpcfg3[16+:8] <= csr_wrdata[16+:8];
        if(!pmpcfg3[3*8+7]) pmpcfg3[24+:8] <= csr_wrdata[24+:8];
      end
    end
  end
end


/////////////////////////////////////////////////////////////////////

// PMPADDR0/15 - 0x3B0-0x3BF

////////////////////////////////////////////////////////////////////

always @(posedge clk or posedge rst) begin
  if(rst) begin
    pmpaddr0  <= PMPADDR0_INIT;
    pmpaddr1  <= PMPADDR1_INIT;
    pmpaddr2  <= PMPADDR2_INIT;
    pmpaddr3  <= PMPADDR3_INIT;
    pmpaddr4  <= PMPADDR4_INIT;
    pmpaddr5  <= PMPADDR5_INIT;
    pmpaddr6  <= PMPADDR6_INIT;
    pmpaddr7  <= PMPADDR7_INIT;
    pmpaddr8  <= PMPADDR8_INIT;
    pmpaddr9  <= PMPADDR9_INIT;
    pmpaddr10 <= PMPADDR10_INIT;
    pmpaddr11 <= PMPADDR11_INIT;
    pmpaddr12 <= PMPADDR12_INIT;
    pmpaddr13 <= PMPADDR13_INIT;
    pmpaddr14 <= PMPADDR14_INIT;
    pmpaddr15 <= PMPADDR15_INIT;
  end
  else if (MPU_SUPPORT > 1) begin
    if(csr_wr_en) begin
      if (csr_adr_wr == `PMPADDR0   &&  !pmpcfg0[0*8+7] && NB_PMP_REGION > 0   ) pmpaddr0    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR1   &&  !pmpcfg0[1*8+7] && NB_PMP_REGION > 1   ) pmpaddr1    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR2   &&  !pmpcfg0[2*8+7] && NB_PMP_REGION > 2   ) pmpaddr2    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR3   &&  !pmpcfg0[3*8+7] && NB_PMP_REGION > 3   ) pmpaddr3    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR4   &&  !pmpcfg1[0*8+7] && NB_PMP_REGION > 4   ) pmpaddr4    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR5   &&  !pmpcfg1[1*8+7] && NB_PMP_REGION > 5   ) pmpaddr5    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR6   &&  !pmpcfg1[2*8+7] && NB_PMP_REGION > 6   ) pmpaddr6    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR7   &&  !pmpcfg1[3*8+7] && NB_PMP_REGION > 7   ) pmpaddr7    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR8   &&  !pmpcfg2[0*8+7] && NB_PMP_REGION > 8   ) pmpaddr8    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR9   &&  !pmpcfg2[1*8+7] && NB_PMP_REGION > 9   ) pmpaddr9    <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR10  &&  !pmpcfg2[2*8+7] && NB_PMP_REGION > 10  ) pmpaddr10   <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR11  &&  !pmpcfg2[3*8+7] && NB_PMP_REGION > 11  ) pmpaddr11   <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR12  &&  !pmpcfg3[0*8+7] && NB_PMP_REGION > 12  ) pmpaddr12   <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR13  &&  !pmpcfg3[1*8+7] && NB_PMP_REGION > 13  ) pmpaddr13   <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR14  &&  !pmpcfg3[2*8+7] && NB_PMP_REGION > 14  ) pmpaddr14   <= csr_wrdata;
      if (csr_adr_wr == `PMPADDR15  &&  !pmpcfg3[3*8+7] && NB_PMP_REGION > 15  ) pmpaddr15   <= csr_wrdata;
    end
  end
end




///////////////////////////////////////////////////////////////////
always @(*) begin
    if(rst) begin
        csr_rddata = 32'b0;
    end   
    else if(~freeze)
    begin
        if(csr_adr_rd == `mepc) begin
            csr_rddata = csr_mepc;
        end
        else if(csr_adr_rd == `mepc_shadow) begin
            csr_rddata = csr_mepc_shadow;
        end
        else if(csr_adr_rd == `mstatus) begin
            csr_rddata = csr_mstatus;
        end
        else if(csr_adr_rd == `mtvec) begin
            csr_rddata = csr_mtvec;
        end
        else if(csr_adr_rd == `mcause) begin
            csr_rddata = csr_mcause;
        end
        else if(csr_adr_rd == `mtval) begin
            csr_rddata = csr_mtval;
        end
        else if(csr_adr_rd == `mie) begin
            csr_rddata = csr_mie;
        end
        else if(csr_adr_rd == `mip) begin
            csr_rddata = csr_mip;
        end
        else if(csr_adr_rd == `misa) begin
            csr_rddata = csr_misa;
        end
        else if(csr_adr_rd == `medeleg) begin
            csr_rddata = csr_medeleg;
        end
        else if(csr_adr_rd == `mideleg) begin
            csr_rddata = csr_mideleg;
        end
        else if(csr_adr_rd == `mbadaddr) begin
            csr_rddata = csr_mbadaddr;
        end
        else if(csr_adr_rd == `fflags) begin
            csr_rddata = csr_fcsr[4:0];
        end
        else if(csr_adr_rd == `frm) begin
            csr_rddata = csr_fcsr[7:5];
        end
        else if(csr_adr_rd == `fcsr) begin
            csr_rddata = csr_fcsr[7:0];
        end
                
        else if (csr_adr_rd == `cache_flush) begin
            csr_rddata = csr_cache_flush;
        end
        else if(csr_adr_rd == `satp) begin
            csr_rddata = csr_satp;
        end
        //PMP Register Reads
        else if(csr_adr_rd == `PMPCFG0)   csr_rddata = pmpcfg0;
        else if(csr_adr_rd == `PMPCFG1)   csr_rddata = pmpcfg1;
        else if(csr_adr_rd == `PMPCFG2)   csr_rddata = pmpcfg2;
        else if(csr_adr_rd == `PMPCFG3)   csr_rddata = pmpcfg3;
        else if(csr_adr_rd == `PMPADDR0)  csr_rddata = pmpaddr0;
        else if(csr_adr_rd == `PMPADDR1)  csr_rddata = pmpaddr1;
        else if(csr_adr_rd == `PMPADDR2)  csr_rddata = pmpaddr2;
        else if(csr_adr_rd == `PMPADDR3)  csr_rddata = pmpaddr3;
        else if(csr_adr_rd == `PMPADDR4)  csr_rddata = pmpaddr4;
        else if(csr_adr_rd == `PMPADDR5)  csr_rddata = pmpaddr5;
        else if(csr_adr_rd == `PMPADDR6)  csr_rddata = pmpaddr6;
        else if(csr_adr_rd == `PMPADDR7)  csr_rddata = pmpaddr7;
        else if(csr_adr_rd == `PMPADDR8)  csr_rddata = pmpaddr8;
        else if(csr_adr_rd == `PMPADDR9)  csr_rddata = pmpaddr9;
        else if(csr_adr_rd == `PMPADDR10) csr_rddata = pmpaddr10;
        else if(csr_adr_rd == `PMPADDR11) csr_rddata = pmpaddr11;
        else if(csr_adr_rd == `PMPADDR12) csr_rddata = pmpaddr12;
        else if(csr_adr_rd == `PMPADDR13) csr_rddata = pmpaddr13;
        else if(csr_adr_rd == `PMPADDR14) csr_rddata = pmpaddr14;
        else if(csr_adr_rd == `PMPADDR15) csr_rddata = pmpaddr15;

        //////////////////////////////////////////////////////////////////////
        else if((csr_adr_rd == `mvendorid) && (csr_adr_rd == `marchid) && (csr_adr_rd == `mimpid) && (csr_adr_rd == `mhartid)) begin
            csr_rddata = 32'b0;                                             //  to mention these registers ar enot implemented
        end

        else begin
            csr_rddata = 32'b0;
        end    
    end
end
 
    
always @(posedge clk) begin
    if(rst) 
        cur_prev_mode <= 2'b11;
end


assign { ctrl_clr_meip,
         mtval_wr,csr_mtval_val,
         mcause_wr,csr_mcause_val,
         mstatus_wr,csr_mstatus_val,
         mepc_wr,csr_mepc_val} = sb_csr;


endmodule
     
