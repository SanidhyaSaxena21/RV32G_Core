`timescale 1 ns / 1 ps

`include "/home/rclab/FINAL_PROJECT/MtechESE_RISCV_2024-25/Working_Project_New/processor_1.3/processor.srcs/sources_1/imports/processor/rtl/defines.v"
module rv32_mpu

    #(
        // Instruction length (always 32, whatever the architecture)
        parameter ILEN = 32,
        // Registers width, 32 bits for RV32i
        parameter XLEN = 32,
        // PMP / PMA supported
        parameter MPU_SUPPORT = 1,
        // Number of physical memory protection regions
        parameter NB_PMP_REGION = 16,
        // Maximum PMP regions support by the core
        parameter MAX_PMP_REGION = 16,
        // Virtual memory support
        parameter MMU_SUPPORT = 0,
        // Address bus width defined for both control and AXI4 address signals
        parameter AXI_ADDR_W = ILEN
    )(
        // clock & reset
        input  wire                        aclk,
        input  wire                        aresetn,
        // instruction memory interface
        input  wire  [XLEN    -1:0]        imem_addr,
        output logic [4             -1:0]  imem_allow, // {L, X, W, R}
        // data memory interface
        input  wire  [XLEN    -1:0]        dmem_addr,
        output logic [4             -1:0]  dmem_allow, // {L, X, W, R}
        // CSR shared bus
        input  wire  [`CSR_SB_W-1:0] csr_sb
    );


    //////////////////////////////////////////////////////////////////////////
    // Functions, parameters and variables
    //////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////
    // Get the region number matching the targeted address
    // @match: matched region, one-hot encoded
    // @returns: an unsigned integer between 0 and NB_PMP_REGION-1
    /////////////////////////////////////////////////////////////////////////////
    function automatic integer unsigned matched_ix(input [NB_PMP_REGION-1:0] match);

        matched_ix = 0;
        for (int i=0; i<NB_PMP_REGION; i++)
            if (match[i])
                matched_ix = i;

    endfunction

    // Address matching A field encoding
    typedef enum logic[1:0] {
        OFF    = 0,
        TOR    = 1,
        NA4    = 2,
        NAPOT  = 3
    } ADDR_MATCH;

    // The Sv32 page-based virtual-memory supports 34-bit physical addresses for RV32
    // The Sv39 and Sv48 page-based virtual-memory schemes support a 56-bit physical address space, 
    // so the RV64 PMP address registers impose the same limit.
    localparam RLEN = (MMU_SUPPORT) ? (XLEN == 32) ? 34 : 56 : 
                      /* NO MMU */    (XLEN == 32) ? 32 : 64;

    logic [NB_PMP_REGION-1:0] imem_pmp_matchs;
    logic [NB_PMP_REGION-1:0] dmem_pmp_matchs;
    logic [NB_PMP_REGION-1:0] pmp_active;
    logic                     mpu_off;

    logic [NB_PMP_REGION*RLEN-1:0] pmp_base;
    logic [NB_PMP_REGION*RLEN-1:0] pmp_mask;

    logic imem_match;
    logic dmem_match;

    integer imem_match_i;
    integer dmem_match_i;


    /////////////////////////////////////////////////////////////////////////////////////////
    // PMP / PMA circuits
    /////////////////////////////////////////////////////////////////////////////////////////

    generate if (MPU_SUPPORT==0) begin: MPU_OFF

    assign imem_pmp_matchs = '0;
    assign imem_match = 1'b0;
    assign imem_match_i = 0;
    assign imem_allow = 4'h7; // R/W/X but not locked

    assign dmem_pmp_matchs = '0;
    assign dmem_match = 1'b0;
    assign dmem_match_i = 0;
    assign dmem_allow = 4'h7; // R/W/X but not locked

    end else begin: MPU_ON

        // Region addres decoding from PMPCFG+PMPADDR CSRs
        for (genvar i=0; i<MAX_PMP_REGION; i++) begin: PMP_REGION_CHECKERS

            if (i<NB_PMP_REGION) begin: REGION_ACTIVE

                rv32_pmp_region 
                #(
                    .RLEN       (RLEN),
                    .XLEN       (XLEN),
                    .AXI_ADDR_W (AXI_ADDR_W)
                )
                pmp_region 
                (
                    .pmp_addr    (csr_sb[4*XLEN+i*XLEN+:XLEN]), // 4*XLEN = 4 * PMPCFG bc is on first position on the bus
                    .pmp_cfg     (csr_sb[i*8+:8]),
                    .pmp_base    (pmp_base[i*RLEN+:RLEN]),
                    .pmp_mask    (pmp_mask[i*RLEN+:RLEN])
                );

                assign pmp_active[i] = csr_sb[i*8+3+:2] != 2'b0;

            end else begin: REGION_OFF
                assign imem_pmp_matchs[i] = 1'b0;
                assign pmp_base[i*RLEN+:RLEN] = '0;
                assign pmp_mask[i*RLEN+:RLEN] = '0;
                assign pmp_active[i] = 1'b0;
            end

        end

        ////////////////////////////////
        // imem / dmem PMP address match
        ////////////////////////////////

        for (genvar i=0; i<MAX_PMP_REGION; i++) begin: PMP_ACCESS_CHECK

            if (i<NB_PMP_REGION) begin: REGION_ACTIVE

                always @ (*) begin
                    case (csr_sb[i*8+3+:2]) // pmp_cfg[4:3]

                        default: begin
                            imem_pmp_matchs[i] = '0;
                            dmem_pmp_matchs[i] = '0;
                        end

                        TOR: begin
                            if (i == 0) begin
                                imem_pmp_matchs[i] = (imem_addr < pmp_base[i*RLEN+:RLEN]);
                                dmem_pmp_matchs[i] = (dmem_addr < pmp_base[i*RLEN+:RLEN]);
                            end else begin
                                imem_pmp_matchs[i] = (imem_addr >= pmp_base[(i-1)*RLEN+:RLEN] && imem_addr < pmp_base[i*RLEN+:RLEN]);
                                dmem_pmp_matchs[i] = (dmem_addr >= pmp_base[(i-1)*RLEN+:RLEN] && dmem_addr < pmp_base[i*RLEN+:RLEN]);
                            end
                        end

                        NA4, NAPOT: begin
                            imem_pmp_matchs[i] = ((imem_addr & pmp_mask[i*RLEN+:RLEN]) == pmp_base[i*RLEN+:RLEN]);
                            dmem_pmp_matchs[i] = ((dmem_addr & pmp_mask[i*RLEN+:RLEN]) == pmp_base[i*RLEN+:RLEN]);
                        end

                    endcase
                end

            end else begin: NO_PMP_ACCESS_CHECK
                assign imem_pmp_matchs[i] = 1'b0;
                assign dmem_pmp_matchs[i] = 1'b0;
            end
        end

        // MPU OFF so make the memory fully available
        assign mpu_off = (pmp_active == '0);

        // A region matched
        assign imem_match = |imem_pmp_matchs;
        assign dmem_match = |dmem_pmp_matchs;

        // Region index taht matched
        assign imem_match_i = matched_ix(imem_pmp_matchs);
        assign dmem_match_i = matched_ix(dmem_pmp_matchs);

        ////////////////////////////////
        // PMA rights
        ////////////////////////////////

        assign imem_allow = (mpu_off)    ? 4'h7 :
                            (imem_match) ? {csr_sb[imem_match_i*8+`PMA_L],
                                            csr_sb[imem_match_i*8+`PMA_X],
                                            csr_sb[imem_match_i*8+`PMA_W],
                                            csr_sb[imem_match_i*8+`PMA_R]} : 
                                           4'b0;

        assign dmem_allow = (mpu_off)    ? 4'h7 :
                            (dmem_match) ? {csr_sb[dmem_match_i*8+`PMA_L],
                                            csr_sb[dmem_match_i*8+`PMA_X],
                                            csr_sb[dmem_match_i*8+`PMA_W],
                                            csr_sb[dmem_match_i*8+`PMA_R]} : 
                                           4'b0;

    end
    endgenerate
endmodule
