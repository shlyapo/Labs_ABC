.globl iterative
.globl recursive

.data
n: .word 11432
m: .word 2

.text
main:
    la t0, n
    lw a0, 0(t0)
    jal ra, tester

    addi a1, a0, 0
    addi a0, x0, 1
    ecall # Print Result

    addi a1, x0, '\n'
    addi a0, x0, 11
    ecall # Print newline

    addi a0, x0, 10
    ecall # Exit

tester:
    # YOUR CODE HERE
    # вызвать итеративную и рекурсивные функции, сравнить ответ и вернуть результат, если совпал. иначе вернуть -1.
    addi sp, sp, -8
    sw a0, 0(sp)
    sw ra, 4(sp)
    jal iterative
    add t2, a0, x0 # t2 = a0
    lw a2, 0(sp)
    # addi sp, sp, 4
    
    addi a1, x0, 1 # a1 = 0 
    addi a0, x0, 0 # a0 = 0 
    jal recursive
    add t1, a0, x0 # t1 = a0
    addi a0, x0, 1 # a0 = 1
    beq t2, t1, eq
    addi a0, x0, -1 # a0 = -1
    eq:
	#jalr zero, 0(ra)
    lw ra, 4(sp)
    addi sp, sp, 8
    jr ra


iterative:
    # YOUR CODE HERE
    addi t0, x0, 1 # mask
    addi s0, x0, 0 # rang
    loop:
    and t1, a0, t0
    beq t1, x0, zzero # t1 == 0
    addi s0, s0, 1
    zzero:
    slli t0, t0, 1
    bne t0, x0, loop
    add a0, s0, x0
    jr ra
    
recursive:
    # YOUR CODE HERE
    beq a1, x0, out # a1 - mask
    and t1, a2, a1 # a2 - digit
    beq t1, x0, zzzero # t1 == 0
    addi a0, a0, 1
    zzzero:
    slli a1, a1, 1
    
    beq x0, x0, recursive
   
    out:
    add a0, s0, x0
    jr ra