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

start:      li a0, 25
            li a1, 4
            li a3, 25             # expected
            add  a2, a0, a1       # fn a2, a0, a1 00b5062b

            li a0, 3
            li a1, 4
            li a3, 5              # expected
            add  a2, a0, a1       # fn a2, a0, a1 00b5062b

            li a0, 12
            li a1, 5
            li a3, 13             # expected
            add  a2, a0, a1       # fn a2, a0, a1 00b5062b

            li a0, 0
            li a1, 0
            li a3, 0              # expected
            add  a2, a0, a1       # fn a2, a0, a1 00b5062b