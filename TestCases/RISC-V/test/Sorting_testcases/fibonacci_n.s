.text
.global main

main:
addi x4,x0,0
li x4,0x20000000
lb x1, 0(x0)
addi x4,x0,0
addi x2, x0, 0
addi x3, x0, 1
beq x1, x0, 60
addi x11, x1, -1
add x1, x0, x11
beq x1, x0, 48
addi x4, x0, 1
addi x11, x1, -1
add x1, x0, x11
add x12, x4, x0
add x11, x3, x4
add x4, x11, x0
add x3, x12, x0
addi x11, x1, -1
add x1, x0, x11
beq x1, x0, 8
beq x0, x0, -28
addi x2, x0, 1
jal x6, 8
beq x0, x0, 0
sb x4, 0(x2)
jal x6, -8
