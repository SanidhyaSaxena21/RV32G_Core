00000000 <_start>:        nop
          	li	ra,0
          	li	sp,0
          	li	gp,0
          	li	tp,0
          	li	t0,0
          	li	t1,0
          	li	t2,0
          	li	s0,0
          	li	s1,0
          	li	a0,0
          	li	a1,0
          	li	a2,0
          	li	a3,0
          	li	a4,0
          	li	a5,0
          	li	a6,0
          	li	a7,0
          	li	s2,0
          	li	s3,0
          	li	s4,0
          	li	s5,0
          	li	s6,0
          	li	s7,0
          	li	s8,0
          	li	s9,0
          	li	s10,0
          	li	s11,0
          	li	t3,0
          	li	t4,0
          	li	t5,0
          	li	t6,0
          	auipc	sp,0x20004
          	addi	sp,sp,-132 # 20003ffc <__stack_pointer>
          	mv	gp,gp
          	csrw	mtvec,zero
          	csrw	mie,zero
          	csrw	mstatus,zero
          	csrw	mip,zero
          	lui	t0,0x4
          	srli	t0,t0,0x2
          	csrw	pmpaddr0,t0
          	lui	t0,0x20000
          	srli	t0,t0,0x2
          	csrw	pmpaddr1,t0
          	lui	t0,0x20004
          	srli	t0,t0,0x2
          	csrw	pmpaddr2,t0
          	lui	t0,0x5f000
          	srli	t0,t0,0x2
          	csrw	pmpaddr3,t0
          	lui	t0,0x5f004
          	srli	t0,t0,0x2
          	csrw	pmpaddr4,t0
          	li	t0,-1
          	srli	t0,t0,0x2
          	csrw	pmpaddr5,t0
          	lui	t0,0x70b0
          	addi	t0,t0,1805 # 70b070d <__rom_end+0x70ac70d>
          	csrw	pmpcfg0,t0
          	lui	t0,0x7070
          	addi	t0,t0,1803 # 707070b <__rom_end+0x706c70b>
          	csrw	pmpcfg1,t0
          	auipc	a4,0x20000
          	addi	a4,a4,-252 # 20000000 <__DATA_BEGIN__>
          	auipc	a5,0x20000
          	addi	a5,a5,-260 # 20000000 <__DATA_BEGIN__>
<clear_bss_loop>: bge	a4,a5,11c <clear_bss_end>
           	sw	zero,0(a4)
           	addi	a4,a4,4
           	j	10c <clear_bss_loop>
<clear_bss_end>: jal	ra,12c <main>
           	li	a0,0
           	li	a7,93
           	ebreak
0000012c <main>:          auipc	a0,0x20000
          	addi	a0,a0,-300 # 20000000 <__DATA_BEGIN__>
          	li	t0,10
          	sw	t0,0(a0)
          	li	t0,80
          	sw	t0,4(a0)
          	li	t0,30
          	sw	t0,8(a0)
          	li	t0,90
          	sw	t0,12(a0)
          	li	t0,40
          	sw	t0,16(a0)
          	li	t0,50
          	sw	t0,20(a0)
          	li	t0,70
          	sw	t0,24(a0)
          	li	a1,0
          	li	a2,6
          	jal	ra,17c <QUICKSORT>
          	jal	ra,274 <EXIT>
0000017c <QUICKSORT>:     addi	sp,sp,-20
           	sw	ra,16(sp)
           	sw	s3,12(sp)
           	sw	s2,8(sp)
           	sw	s1,4(sp)
           	sw	s0,0(sp)
           	mv	s0,a0
           	mv	s1,a1
           	mv	s2,a2
           	blt	a2,a1,1cc <START_GT_END>
           	jal	ra,1e8 <PARTITION>
           	mv	s3,a0
           	mv	a0,s0
           	mv	a1,s1
           	addi	a2,s3,-1
           	jal	ra,17c <QUICKSORT>
           	mv	a0,s0
           	addi	a1,s3,1
           	mv	a2,s2
           	jal	ra,17c <QUICKSORT>
000001cc <START_GT_END>:  lw	s0,0(sp)
          	lw	s1,4(sp)
          	lw	s2,8(sp)
          	lw	s3,12(sp)
          	lw	ra,16(sp)
          	addi	sp,sp,20
          	ret
000001e8 <PARTITION>:     addi	sp,sp,-4
          	sw	ra,0(sp)
          	slli	t0,a2,0x2
          	add	t0,t0,a0
          	lw	t0,0(t0)
          	addi	t1,a1,-1
          	mv	t2,a1
00000204 <LOOP>:          beq	t2,a2,23c <LOOP_DONE>
          	slli	t3,t2,0x2
          	add	a6,t3,a0
          	lw	t3,0(a6)
          	addi	t0,t0,1
          	blt	t0,t3,234 <CURR_ELEMENT_GTE_PIVOT>
          	addi	t1,t1,1
          	slli	t5,t1,0x2
          	add	a7,t5,a0
          	lw	t5,0(a7)
          	sw	t5,0(a6)
          	sw	t3,0(a7)
00000234 <CURR_ELEMENT_GTE_PIVOT>: addi	t2,t2,1
          	beqz	zero,204 <LOOP>
0000023c <LOOP_DONE>:     addi	t5,t1,1
          	mv	a5,t5
          	slli	t5,t5,0x2
          	add	a7,t5,a0
          	lw	t5,0(a7)
          	slli	t3,a2,0x2
          	add	a6,t3,a0
          	lw	t3,0(a6)
          	sw	t5,0(a6)
          	sw	t3,0(a7)
          	mv	a0,a5
          	lw	ra,0(sp)
          	addi	sp,sp,4
          	ret

00000274 <EXIT>:          li	a0,1
          	csrw	0x400,a0

0000027c <WHILE1>:        j	27c <WHILE1>
