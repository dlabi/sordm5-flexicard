/* MACROS */

.macro mov32, reg, val
    movw \reg, #:lower16:\val
    movt \reg, #:upper16:\val
.endm


.macro delay val
	.rept \val
        	nop
	.endr
.endm

.macro clear_EXTI4_FTSR   //falling edge selection register
	vstr	reg_zero,[r3,FTSR]
.endm

.macro set_EXTI4_FTSR
	vstr	reg_bit4_high,[r3,FTSR]
.endm

.macro clear_EXTI4_RTSR   //rising edge selection register
	vstr	reg_zero,[r3,RTSR] //kdybychom museli jinym interuptem reagovat na nabeznou hranu, tak bychom museli pouzit AND jen na INT4
.endm

.macro set_EXTI4_RTSR
	vstr	reg_bit4_high,[r3,RTSR]
.endm


.macro unrolled_exit_EXTI4
#if defined(DEBUG_EXTI4_END)
       // delay 2
        mov     r0,#0x000F0000                          //clears bits 3-0
        str     r0,[r12,BSRR]
#endif
        dsb
	bx	lr
.endm