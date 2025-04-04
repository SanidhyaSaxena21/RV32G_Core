.text

csrw satp,x0
lui x3,0x00008
addi x3,x3,0x004
csrw mtvec,x3

main:
	addi x8,x0,0
    lui x8,%hi(LC0)
    addi x8,x8,%lo(LC0)
	addi a3,x0,0 
	lw a4,0(x8)  
	addi a6,x0,0x5
.loop: 
      lw a5,0(x8)
      bge a4,a5,.addr1
      lw a4,0(x8)
.addr1:
      addi a3,a3,1
      addi x8,x8,0x4
      blt a3,a6,.loop
     sw a4,40(x8)
 /* CSR Writing */
 addi x9,x0,1
 csrw 0x400,x9
 
 .end_I:
 beq x0,x0,.end_I
 /* Data Section*/
.data
LC0:
	.word 10
    .word 30
    .word 50
    .word 60
    .word 23


