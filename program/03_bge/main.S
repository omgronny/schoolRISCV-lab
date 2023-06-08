# RISC-V sqrt program
#
# Stanislav Zhelnio, 2020
#

## unsigned isqrt (unsigned x) {
##     unsigned m, y, b;
##     m = 0x40000000;
##     y = 0;
##     while (m != 0) { // Do 16 times
##         b = y |  m;
##         y >>= 1;
##         if (x >= b) {
##             x -= b;
##             y |= m;
##         }
##         m >>= 2;
##     }
##     return y;
## }

            .text

start:      mv a0, zero

next0:      mv a0, zero
            li a1, 5
            li a2, 3
            bge a1, a2, next1
            beq zero, zero, err

next1:      addi a0, a0, 1
            li a1, 3
            li a2, 3
            bge a1, a2, next2
            beq zero, zero, err

next2:      addi a0, a0, 1
            li a1, 0
            li a2, 0
            bge a1, a2, next3
            beq zero, zero, err

next3:      addi a0, a0, 1
            li a1, 0
            li a2, 5
            bge a1, a2, err

next4:      addi a0, a0, 1
            li a1, 0
            li a2, 5
            bge a1, a2, err

next5:      addi a0, a0, 1
            li a1, 0
            li a2, 1
            bge a1, a2, err

next6:      addi a0, a0, 1
            li a1, 100
            li a2, 125
            bge a1, a2, err

next7:      addi a0, a0, 1
            li a1, 1
            li a2, 0
            bge a1, a2, return
            beq zero, zero, err

err:        li a3, 1
            li a1, 1
            li a2, 1
            beq zero, zero, err

return:     li a3, 0
            li a1, 0
            li a2, 0
            beq zero, zero, return

