/*________Address Map_____________________*/
.equ SRAM_BASE,    		0x00004000
.equ MTVEC_BASE,    	0x0000C000
.equ Page_Table_Base, 	0x00008000

.equ CLINT_MTIME_ADDRESS, 0x5F000004
.equ CLINT_MTIMECMP_ADDRESS, 0x5F00000C

/*_________CSRs__________________________*/
/* Custom CSR*/
.equiv CSR_FLUSH, 0x400

/*-----MSTATUS-----------*/
/*Global Interrupt Bit*/
.equiv MSTATUS_MIE, 0x00000008
/*Previous Interrupt enable Bit*/
.equiv MSTATUS_MPIE, 0x00000080

/*----MIE---------*/
.equiv MIE_MSIE, 0x00000008
.equiv MIE_MTIE, 0x00000080
.equiv MIE_MEIE, 0x00000800

/*-----MTVEC----------*/
.equiv trap_routine, 0x0000C001


# Handler Section
.section .handler
j trap_base
nop
nop
j isr_software
nop
nop
nop
j isr_timer
nop
nop
nop
j isr_external

trap_base:
  j trap_base


isr_timer:
	addi x15,x15,0x1
    lui a0,%hi(CLINT_MTIME_ADDRESS)
    addi a0,a0,%lo(CLINT_MTIME_ADDRESS)
    sw x0,0(a0)
    sw x0,4(a0) /* Init Timer to 0*/
    addi t0,x0,128
    csrrc zero,mip,t0
	MRET
isr_software:
	addi x10,x0,500
	MRET
isr_external:
	addi x10,x0,600
	MRET


