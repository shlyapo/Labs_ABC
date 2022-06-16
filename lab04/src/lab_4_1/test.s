#RISC-V Assembly   Description            Address        Machine Code

lui x1, 10			#x1 = 10                 0            000000B7
lui x2, 20			#x2 = 20                 4            00001137
lui x3, 30		    #x3 = 30                 8            100001B7

add x4, x1, x3      #x4 = 10 + 30            c            00308233
add x5, x2, x1      #x5 = 20 + 10            10           001102B3