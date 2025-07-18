    AREA ||i.fast_memcpy||, CODE, READONLY, ALIGN=2

fast_memcpy PROC
    EXPORT  fast_memcpy
    PUSH          {R0, R4-R7}
    ORR           r3,r0,r1
    LSLS          r3,r3,#30
    BEQ           L12
    LSLS          r3,r3,#1
    BEQ           L4
    ;EOR           R4, R0, R1
    ;ANDS          R4,R4,#3
    B             L1

    ;LDRB          r3,[r1],#0x01
    ;STRB          r3,[r0],#0x01
    ;SUB           r2,r2,#1
    SUB          R4,R4,#1
    ;BEQ           L12

    ;LDRB          r3,[r1],#0x01
    ;STRB          r3,[r0],#0x01
    ;SUBS          r2,r2,#1
    ;SUBS          R4,R4,#1
    ;BEQ           L12

    ;LDRB          r3,[r1],#0x01
    ;STRB          r3,[r0],#0x01
    ;SUBS          r2,r2,#1

    ;20字节
L13  LDM          r1!,{r3-r7}
    SUBS          r2,r2,#20
    STM           r0!,{r3-r7}
L12  CMP          r2,#20
    BCS           L13
    B             L10

    ;16字节
L11  LDM           r1!,{r3-r6}
    SUBS          r2,r2,#16
    STM           r0!,{r3-r6}
L10  CMP           r2,#16
    BCS           L11
    B             L8

    ;12字节
L9  LDM           r1!,{r3-r5}
    SUBS          r2,r2,#12
    STM           r0!,{r3-r5}
L8  CMP           r2,#12
    BCS           L9
    B             L6

    ;八字节
L7  LDM           r1!,{r3, r4}
    SUBS          r2,r2,#8
    STM           r0!,{r3, r4}
L6  CMP           r2,#0x08
    BCS           L7
    B             L0

    ;四字节
L2  LDM           r1!,{r3}
    SUBS          r2,r2,#4
    STM           r0!,{r3}
L0  CMP           r2,#0x04
    BCS           L2
    B             L4

    ;2字节
L5  LDRH          r3,[r1],#0x02
    SUBS          r2,r2,#2
    STRH          r3,[r0],#0x02
L4  CMP           r2,#0x02
    BCS           L5
    B             L1

    ;单字节
L3  LDRB          r3,[r1],#0x01
    STRB          r3,[r0],#0x01
L1  SUBS          r2,r2,#1
    BCS           L3

    POP          {R0, R4-R7}
    BX            lr

    ENDP


    AREA ||i.fast_memcpy_1||, CODE, READONLY, ALIGN=2

fast_memcpy_1 PROC
    EXPORT  fast_memcpy_1
    PUSH     {r0}

    CMP      r2,#4
    BCC      |L8.174|
    EOR      r3,r0,r1
    ANDS     r3,r3,#3
    BEQ      |L8.38|
    CMP      r3,#1
    BEQ      |L8.174|
    CMP      r3,#2
    BEQ      |L8.160|
    CMP      r3,#3
    BNE      |L8.102|
    B        |L8.174|
|L8.28|
    LDRB     r3,[r1],#1
    STRB     r3,[r0],#1
    SUBS     r2,r2,#1
|L8.38|
    LSLS     r3,r0,#30
    BNE      |L8.28|
    B        |L8.102|
|L8.44|
    LDM      r1!,{r3-r10}
    SUBS     r2,r2,#0x20
    STM      r0!,{r3-r10}
|L8.102|
    CMP      r2,#0x20
    BCS      |L8.44|
    CMP      r2,#0x10
    BCC      |L8.144|
    LDM      r1!,{r3-r6}
    SUBS     r2,r2,#0x10
    STM      r0!,{r3-r6}
    B        |L8.144|
|L8.138|
    LDM      r1!,{r3}
    SUBS     r2,r2,#4
    STM      r0!,{r3}
|L8.144|
    CMP      r2,#4
    BCS      |L8.138|
    B        |L8.160|
|L8.150|
    LDRH     r3,[r1],#2
    STRH     r3,[r0],#2
    SUBS     r2,r2,#2
|L8.160|
    CMP      r2,#2
    BCS      |L8.150|
    B        |L8.174|
|L8.166|
    LDRB     r3,[r1],#1
    STRB     r3,[r0],#1
|L8.174|
    SUBS     r2,r2,#1
    BCS      |L8.166|

    POP      {r0}
    BX       lr

    ENDP

    END