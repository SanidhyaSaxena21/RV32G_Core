#
# TEST CODE FOR SB
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
        
        # store byte with zero immediate
        li      x1, 0x12345678  # set x1 to 0x12345678 
        addi    x2,x2,-16
        #li      x2, __ram_start      # set x2 to 0x1008
        sb      x1, 0(x2)       # store byte from x1 (0x12345678) to 0x1008, mem_byte(0x1008)=0x78 
        
        # store byte with positive immediate
        li      x3, 0xAABBCCDD  # set x3 to 0xAABBCCDD
        sb      x3, 1(x2)       # store byte from x3 (0xAABBCCDD) to 0x1009, mem_byte(0x1009)=0xDD
        
        #store byte with negative immediate
        sb      x3, -4(x2)      # store byte from x3(0xAABBCCDD) to 0x1004, mem_byte(0x1004)=0xDD
         
        # self-check 
        li      x4, 0x78        # set x4 to 0x78 (expected value for mem_byte(0x1008)
        beqz    x4, fail0       # make sure x4 has value
        li      x5, 0xDD        # set x5 to 0xDD (expected value for mem_byte(0x1009) and mem_byte(0x1004) )
        beqz    x5, fail0       # make sure x5 has value
        lbu     x6, 0(x2)       # load byte from address 0x1008 to x6 (0x78)
        lbu     x7, 1(x2)       # load byte from address 0x1009 to x7 (0xDD)
        lbu     x8, -4(x2)      # load byte from address 0x1004 to x8 (0xDD)
        bne     x6, x4, fail1   #
        bne     x7, x5, fail2   # branch to fail if not equal to expected value
        bne     x8, x5, fail3   #
        

         
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

        # Data section
data:
        
