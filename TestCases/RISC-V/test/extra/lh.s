#
# TEST CODE FOR LH
#
        # -----------------------------------------
        # Program section (known as text)
        # -----------------------------------------
        .text

# Start symbol (must be present), exported as a global symbol.

# Export main as a global symbol
        .global main

# Label for entry point of test code
main:
        ### TEST CODE STARTS HERE ###
        # load halfword with zero imm
        lla  x1, data            # set x1 to data
        addi x1,x1, 8           # set x1 to data + 8
        lh  x2, 0(x1)           # load halfword from address 0x1008 to x2, x2=0x00005577
        
        # load halfword with positive imm
        lh x3, 10(x1)           # load halfword from address 0x1012 to x3, x3=0xFFFFABCD
        
        # load halfword with negative imm
        lh x4, -6(x1)           # load byte from address 0x1002 to x4, x4=0x00001234
        
        #self-check
        li x5, 0x5577           # set x5 to 0x5577 (expected value for x2)
        beqz x5, fail0          # make sure x5 has value
        li x6, -21555           # set x6 to -21555 (0xFFFFABCD) (expected value for x3)
        beqz x6, fail0          # make sure x6 has value
        li x7, 0x1234           # set x7 to 0x1234 (expected value for x4)
        beqz x7, fail0          # make sure x7 has value
        bne x2, x5, fail1       #
        bne x3, x6, fail2       # branch to fail if not equal to expected value
        bne x4, x7, fail3       #
        
        ###    END OF TEST CODE   ###

        # Exit test using RISC-V International's riscv-tests pass/fail criteria
        pass:
        li      a0, 0           # set a0 (x10) to 0 to indicate a pass code
        li      a7, 93          # set a7 (x17) to 93 (5dh) to indicate reached the end of the test
        ebreak
        
        fail0:
        li      a0, 1           # fail code
        li      a7, 93          # reached end of code
        ebreak
        
        fail1:
        li      a0, 2           # fail code
        li      a7, 93          # reached end of code
        ebreak
        
        fail2:
        li      a0, 4           # fail code
        li      a7, 93          # reached end of code
        ebreak
        
        fail3:
        li      a0, 6           # fail code
        li      a7, 93          # reached end of code
        ebreak
        
 
        # -----------------------------------------
        # Data section. Note starts at 0x1000, as 
        # set by DATAADDR variable in rv_asm.bat.
        # -----------------------------------------
        .data
 data: 
        # Data section
        .word 0x12345678    #1000
        .word 0             #1004
        .word 0x11335577    #1008
        .word 0             #100C
        .word 0xABCDEF19    #1010
