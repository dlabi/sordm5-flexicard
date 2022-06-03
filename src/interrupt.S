#define _VER_ 11.1

#include "main.h"

	.syntax unified
	.cpu cortex-m4
//	.fpu softvfp
        .fpu fpv4-sp-d16
	.thumb

	.global EXTI15_10_IRQHandler
	.global EXTI4_IRQHandler
        .global	init_fpu_regs
	.global rom_base
        .global low_64k_base
	.global high_64k_base
	.global main_thread_command
	.global main_thread_data
        .global mem_mode



/* REGISTERS */
#define reg_zero	s0
#define reg_bit0_high	s1
#define reg_bit1_high	s2
#define reg_gpioc_base	s3
#define reg_bit14_high	s4

#define reg_gpioa_base	s5
#define reg_high_64k_base	s6
#define reg_exti_base	s7
#define reg_moder_dataout	s8
#define reg_moder_datain	s9
#define reg_rom_base	s10
#define reg_led_on	s11
#define reg_gpiob_base	s12
#define reg_bit6_high	s13
#define reg_bit4_high	s14
#define reg_led_off	s15

// NOTE: FPU registers d8 and d9 are used somehow, so you cannot use s16,s17,s18 and s19

#define reg_bit8_high	s22
#define reg_fake_stack	s30

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
	vstr	reg_zero,[r3,RTSR]
.endm

.macro set_EXTI4_RTSR
	vstr	reg_bit4_high,[r3,RTSR]
.endm


.macro unrolled_exit_EXTI4
#if defined(DEBUG_EXTI4_END)
        delay 2
        vstr     reg_led_off,[r12,ODR]
#endif
        dsb
	bx	lr
.endm



.section .rodata
// Can put up to four 16K roms here which will get mapped to rom 12, 13, 14 and 15
rom_base:
// be careful if you add roms and later delete them. The old ones might be still in the STM32 flash
.incbin "../roms/basic-i.rom"

        .equ PERIPH_BASE     ,   0x40000000                                                                
        .equ PERIPH_BASE_APB1,   (PERIPH_BASE + 0x00000)
        .equ PERIPH_BASE_APB2,   (PERIPH_BASE + 0x10000)
        .equ PERIPH_BASE_AHB1,   (PERIPH_BASE + 0x20000)
        .equ PERIPH_BASE_AHB2,   0x50000000
        .equ PERIPH_BASE_AHB3,   0x60000000

        .equ GPIOA_BASE,   (PERIPH_BASE_AHB1 + 0x0000)                                           
        .equ GPIOB_BASE,   (PERIPH_BASE_AHB1 + 0x0400)
        .equ GPIOC_BASE,   (PERIPH_BASE_AHB1 + 0x0800)
        .equ GPIOD_BASE,   (PERIPH_BASE_AHB1 + 0x0C00)
        .equ GPIOE_BASE,   (PERIPH_BASE_AHB1 + 0x1000)
        .equ GPIOF_BASE,   (PERIPH_BASE_AHB1 + 0x1400)
        .equ GPIOG_BASE,   (PERIPH_BASE_AHB1 + 0x1800)
        .equ GPIOH_BASE,   (PERIPH_BASE_AHB1 + 0x1C00)
        .equ GPIOI_BASE,   (PERIPH_BASE_AHB1 + 0x2000)

        .equ B_TO_C_OFFSET,     (GPIOC_BASE - GPIOB_BASE)
	.equ B_TO_D_OFFSET,     (GPIOD_BASE - GPIOB_BASE)
        .equ B_TO_E_OFFSET,     (GPIOE_BASE - GPIOB_BASE)

	.equ C_TO_D_OFFSET,     (GPIOD_BASE - GPIOC_BASE)
	.equ C_TO_E_OFFSET,     (GPIOE_BASE - GPIOC_BASE)

        .equ GPIOA_MODER     ,   GPIOA_BASE + 0x00
        .equ GPIOA_OTYPER    ,   GPIOA_BASE + 0x04
        .equ GPIOA_OSPEEDR   ,   GPIOA_BASE + 0x08
        .equ GPIOA_PUPDR     ,   GPIOA_BASE + 0x0C
        .equ GPIOA_IDR       ,   GPIOA_BASE + 0x10
        .equ GPIOA_ODR       ,   GPIOA_BASE + 0x14
        .equ GPIOA_BSRR      ,   GPIOA_BASE + 0x18
        .equ GPIOA_LCKR      ,   GPIOA_BASE + 0x1C
        .equ GPIOA_AFRL      ,   GPIOA_BASE + 0x20
        .equ GPIOA_AFRH      ,   GPIOA_BASE + 0x24

        .equ GPIOB_MODER     ,   GPIOB_BASE + 0x00
        .equ GPIOB_OTYPER    ,   GPIOB_BASE + 0x04
        .equ GPIOB_OSPEEDR   ,   GPIOB_BASE + 0x08
        .equ GPIOB_PUPDR     ,   GPIOB_BASE + 0x0C
        .equ GPIOB_IDR       ,   GPIOB_BASE + 0x10
        .equ GPIOB_ODR       ,   GPIOB_BASE + 0x14
        .equ GPIOB_BSRR      ,   GPIOB_BASE + 0x18
        .equ GPIOB_LCKR      ,   GPIOB_BASE + 0x1C
        .equ GPIOB_AFRL      ,   GPIOB_BASE + 0x20
        .equ GPIOB_AFRH      ,   GPIOB_BASE + 0x24

        .equ GPIOC_MODER     ,   GPIOC_BASE + 0x00
        .equ GPIOC_OTYPER    ,   GPIOC_BASE + 0x04
        .equ GPIOC_OSPEEDR   ,   GPIOC_BASE + 0x08
        .equ GPIOC_PUPDR     ,   GPIOC_BASE + 0x0C
        .equ GPIOC_IDR       ,   GPIOC_BASE + 0x10
        .equ GPIOC_ODR       ,   GPIOC_BASE + 0x14
        .equ GPIOC_BSRR      ,   GPIOC_BASE + 0x18
        .equ GPIOC_LCKR      ,   GPIOC_BASE + 0x1C
        .equ GPIOC_AFRL      ,   GPIOC_BASE + 0x20
        .equ GPIOC_AFRH      ,   GPIOC_BASE + 0x24

        .equ GPIOD_MODER     ,   GPIOD_BASE + 0x00
        .equ GPIOD_OTYPER    ,   GPIOD_BASE + 0x04
        .equ GPIOD_OSPEEDR   ,   GPIOD_BASE + 0x08
        .equ GPIOD_PUPDR     ,   GPIOD_BASE + 0x0C
        .equ GPIOD_IDR       ,   GPIOD_BASE + 0x10
        .equ GPIOD_ODR       ,   GPIOD_BASE + 0x14
        .equ GPIOD_BSRR      ,   GPIOD_BASE + 0x18
        .equ GPIOD_LCKR      ,   GPIOD_BASE + 0x1C
        .equ GPIOD_AFRL      ,   GPIOD_BASE + 0x20
        .equ GPIOD_AFRH      ,   GPIOD_BASE + 0x24

        .equ GPIOE_MODER     ,   GPIOE_BASE + 0x00
        .equ GPIOE_OTYPER    ,   GPIOE_BASE + 0x04
        .equ GPIOE_OSPEEDR   ,   GPIOE_BASE + 0x08
        .equ GPIOE_PUPDR     ,   GPIOE_BASE + 0x0C
        .equ GPIOE_IDR       ,   GPIOE_BASE + 0x10
        .equ GPIOE_ODR       ,   GPIOE_BASE + 0x14
        .equ GPIOE_BSRR      ,   GPIOE_BASE + 0x18
        .equ GPIOE_LCKR      ,   GPIOE_BASE + 0x1C
        .equ GPIOE_AFRL      ,   GPIOE_BASE + 0x20
        .equ GPIOE_AFRH      ,   GPIOE_BASE + 0x24

        .equ MODER           ,   0x00
        .equ OTYPER          ,   0x04
        .equ OSPEEDR         ,   0x08
        .equ IDR             ,   0x10
        .equ ODR             ,   0x14

        .equ    EXTI_Line1      ,       0x00002
        .equ    EXTI_Line2      ,       0x00004
        .equ    EXTI_Line4      ,       0x00010
        .equ    EXTI_Line14      ,      0x4000
        .equ    EXTI            ,       0x40013c00
        .equ    EXTI_IMR        ,       EXTI + 0x00
        .equ    EXTI_PR         ,       EXTI + 0x14

	.equ	IMR		,	0x00
	.equ	EMR		,	0x04
	.equ	RTSR		,	0x08
        .equ	FTSR		,	0x0c
	.equ	PR		,	0x14

    .equ        SYSCFG_BASE,    0x40013800
    .equ        SYSCFG_EXTICR1, SYSCFG_BASE + 0x08


#define	DATA_OUT_MODE	0x55550020
#define	DATA_IN_MODE	0x00000020

#define _MREQ_MASK	0x0010
#define _EXIOA_MASK     0x0100
#define _MRD_MASK	0x0800
#define _MWR_MASK	0x1000
#define _IORD_MASK	0x2000
#define _IOWR_MASK	0x4000



.section .data

// main_thread_data must immediately follow main_thread_command
main_thread_command:
	.word	0x00000000			// command word
main_thread_data:
	.word	0x00000000			// command arg
mem_mode:
        .byte   0x00                            // memory mode

.section ccmram
low_64k_base:
	// low_64k_base consumes the entire 64K CCMRAM . Look in stm32_flash.ld for where the global symbol is defined
	// is defined in stm32_flash.ld
       //.lcomm low_64k_base,0x10000               

.section .bss
        .lcomm high_64k_base,0x10000                

  //      .lcomm  EXTI4_tempvar,4


//PA0-PA2 is a debug output for hooking up to a logic analyser to check timings, but is not connected normally

//PB3 - RFSH
//PB5 - WAIT
//PB6 - EXINT
//PB7 - EXCLK
//PB8 - EXIOA           &60 - &6f
//PB9 - EXIOB           &70 - &7F
//PB10 - RST
//PB11 - MRD            -RD & -MREQ
//PB12 - MWR            -WR & MREQ
//PB13 - IORD           -RD & -IORQ
//PB14 - IOWR           -WR & -IORQ
//PB15 - ROMDS

//PC0 - ROM0		&0000 - &1FFF
//PC1 - ROM1		&2000 - &3FFF
//PC2 - ROM2	 	&4000 - &5FFF	
//PC3 - EXM    	        &6000 - &6FFF
//PC4 - MRQ
//PC13 - CLK


.section .text

.type init_fpu_regs, %function
.align 4
.thumb_func
init_fpu_regs:
	movs	r0,#0
	vmov	reg_zero,r0

	movs	r0,#1
	vmov	reg_bit0_high,r0

	movs	r0,#2
	vmov	reg_bit1_high,r0

	mov32	r0,GPIOC_BASE
	vmov	reg_gpioc_base,r0

	movs	r0,#0b100000000000000
	vmov	reg_bit14_high,r0

	mov32	r0,GPIOA_BASE
	vmov	reg_gpioa_base,r0

	mov32	r0, high_64k_base
	vmov	reg_high_64k_base,r0

	mov32	r0, EXTI
	vmov	reg_exti_base,r0

	mov32	r0,0x55550020
	vmov	reg_moder_dataout,r0

	mov32	r0,0x00000020
	vmov	reg_moder_datain,r0

	mov32	r0,GPIOB_BASE
	vmov	reg_gpiob_base,r0

        movs	r0,#0x10
	vmov	reg_bit4_high,r0

	movs	r0,#0x40
	vmov	reg_bit6_high,r0

	movs	r0,#0x100
	vmov	reg_bit8_high,r0

        mov32	r0,rom_base
	vmov	reg_rom_base,r0

        movs	r0,#0x44
	vmov	reg_led_on,r0

        movs	r0,#0xc0
        vmov    reg_led_off,r0

	//mov32	r0,0x10000000
	//vmov	reg_ccmram_log,r0

	bx	lr


// _MREQ Handler. Int on falling edge

// meaning of 'global' registers, all should be pushed/popped automatically
// r0 - tmp
// r1 - address from GPIOE and also used for switch MODER to either OUTPUT or INPUT
// r2 - reg_gpiob_base
// r3 - exti_base
// r12 - reg_gpioa_base - debug register

.type EXTI4_IRQHandler, %function
.align 4
.thumb_func
EXTI4_IRQHandler:
        vmov	r12,reg_gpioa_base
#ifdef DEBUG_EXTI4_START
        vstr     reg_bit0_high,[r12,ODR]
#endif
       
clear_pending_iorq_int:                                 // musi byt jinak nezachytime vsechny edge
	vmov	r3,reg_exti_base		        // s7 = EXTI
	vstr	reg_bit4_high,[r3,PR]	                // clear interrupt

        vmov	r2,reg_gpiob_base
        ldr     r0,[r2, B_TO_C_OFFSET + IDR]            // GPIOC->IDR
        tst     r0,#_MREQ_MASK
        bne     end_EXTI4_IRQHandler                    //exit spurious IRQ

        ldr     r0,[r2, IDR]                            // GPIOB->IDR get mrd, mwr    
        tst     r0,#_MRD_MASK                           // check MRD, if log. 0 then we present data, if not write to RAM
        ldr     r1,[r2, B_TO_E_OFFSET + IDR]            // GPIOE->IDR get the address
        ldr     r0, =mem_mode
        mov     r0, #1 //[r0]            
        beq     read_memory
 
write_check:
        tst     r1,#0x8000                              // check A15. If it is high. We are interested
        bne     write_memory
        tst     r0, #1                                  //1 - RAM 0-ROM
        bne     no_rom                                  // 2000-                                                              
        b       end_EXTI4_IRQHandler   
no_rom:
        cmp     r1,#0x2000
        blo     end_EXTI4_IRQHandler                    //0-1fff exit
        cmp     r1,#0x7000
        bhs     end_EXTI4_IRQHandler                    //7000-7fff exit

write_memory:
#ifdef DEBUG_EXTI4_RAM_WRITE
        vstr     reg_bit1_high,[r12,ODR]
#endif
        vmov    r3, reg_high_64k_base
	delay 10

	ldr	r0,[r2, B_TO_D_OFFSET + IDR]            // GPIOD read databus
	lsrs	r0,r0,#8
	strb	r0,[r3,r1]
        b       end_EXTI4_IRQHandler

read_memory:
#ifdef DEBUG_EXTI4_RAM_TEST
        vstr     reg_bit0_high,[r12,ODR]
#endif
        tst     r1,#0x8000
        bne     ram
        cmp     r1,#0x7000
        bhs     end_EXTI4_IRQHandler                    //7000-7fff exit
        ldr     r0, =mem_mode
        mov     r0, #1 //[r0] 
        tst     r0, #1                                  //1-RAM 0-ROM
        bne     ram

rom:                                                    // read from 0-7fff, test if we are interested
        vmov	r3,reg_rom_base                         
        ldrb    r0,[r3]                                 // get rom type: 00: 2000-3fff, 01: 4000-5fff, 02: 2000-5fff
        sub     r1, #0x2000                             // offset address from 0
        cmp     r0,#1
        IT EQ
        subeq   r1,#0x2000                              // rom type 01 from 4000: offset address from 0
        ITE GT                                          
        movgt   r0,#0x4000                              // type 02
        movle   r0,#0x2000                              // type 00
        cmp     r1,r0                                   // test if we are in rom range
        bcs.n   end_EXTI4_IRQHandler                    // no we aren't -> exit
        b       read

ram:
        vmov    r3, reg_high_64k_base                   // read from RAM

        //musim dopsat test na odpojeny monitor, jinak muze nastat hazard v regionu 0000-1fff, zatim pro tuto oblast nebudeme emulovat RAM
        cmp     r1,#0x2000
        blo     end_EXTI4_IRQHandler                    

#ifdef DEBUG_EXTI4_RAM_READ
        vstr     reg_led_on,[r12,ODR]
#endif

read:
#ifdef DEBUG_EXTI4_MRD
        vstr     reg_bit1_high,[r12,ODR]
#endif
        ldrb    r0,[r3,r1]                              // get the byte from a 16K ROM or RAM
        lsls    r0,r0, #8
        
        vmov	r1,reg_moder_dataout

        str     r0,[r2, B_TO_D_OFFSET + ODR]            // GPIOD ODR
        str     r1,[r2, B_TO_D_OFFSET + MODER]          // MODER set to outputs

#ifdef DEBUG_EXTI4_DATA_OUT
        vstr     reg_led_on,[r12,ODR]
#endif

// Use WFE to wait until _MREQ goes high


        vmov	r3,reg_exti_base

    // disable neni potreba je automaticky, ale re-enable ji potreba je   
    //    ldr     r0,[r3,IMR]             // EXTI_IMR
     //   and     r0,#0xffffffef          // turn off  PC4 int
     //   str     r0,[r3,IMR]             // EXTI_IMR
        vstr    reg_bit4_high,[r3,PR]   // EXTI_PR  - clear the current PC4 interrupt

        set_EXTI4_RTSR                                  // povol nabeznou hranu na EXTI4, abychom tristatovali co nejdrive, pouziva r3
        vstr    reg_bit4_high,[r3,EMR]                  // EXTI_EMR - wait for an event rather than an interrupt on PC4
       
        vmov	r1,reg_moder_datain

	dsb                                             // not entirely sure if this is required
        sev                                             // set the event register
        wfe                                             // clear the event register
        // So the 2nd wfe is meant to wait for the +ve edge, but if you get a spurious edge at exactly the time these instructions execute, it can slip through without waiting
        wfe                                             // wait for the positive edge of _MREQ

        //databus_read_extra_delay

// _MREQ should have just gone high here
        str     r1,[r2, B_TO_D_OFFSET + MODER]          // tristate databus

#ifdef DEBUG_EXTI4_TRISTATED
        movs    r0,#0xc5
        str     r0,[r12, ODR]
#endif
        clear_EXTI4_RTSR                                //zakaz INT na nabeznou hranu, pouziva r3

end_EXTI4_IRQHandler:           
       
        vmov	r3,reg_exti_base
        ldr     r3,=EXTI
        ldr     r0,[r3,IMR]                             // EXTI_IMR
        orr     r0,#0x0010                              // reenable PC4 interrupts

        movs    r1,#EXTI_Line4
        str     r1,[r3,PR]                              // EXTI_PR 
        str     r0,[r3,IMR]                             // EXTI_IMR	// I think you need to reenable after the clear

unrolled_exit_EXTI4


// _IORW interrupt -ve edge
.type EXTI15_10_IRQHandler, %function
EXTI15_10_IRQHandler:
        vmov	   r12,reg_gpioa_base
#ifdef DEBUG_EXTI14_START
        vstr       reg_bit0_high,[r12,ODR]
#endif                      

clear_pending_iorq_int14:                               // musi byt jinak nezachytime vsechny edge
	vmov	r3,reg_exti_base		        // s7 = EXTI
	vstr	reg_bit14_high,[r3,PR]	                // clear interrupt

        vmov	r2,reg_gpiob_base
        ldr     r0,[r2, IDR]                            // GPIOB->IDR get mreq
        tst     r0,#_IOWR_MASK
        bne     end_EXTI15_10_IRQHandler                //exit spurious IRQ

//	tst	r0,_EXIOA_MASK
//	bne.n	end_EXTI15_10_IRQHandler

	ldr     r1,[r2, B_TO_E_OFFSET + IDR]            // GPIOE->IDR get the address
	cmp     r1, #0x6c
        bne.n	end_EXTI15_10_IRQHandler
#ifdef DEBUG_EXTI14_6C
        vstr    reg_bit1_high,[r12,ODR]
#endif
	ldr	r0,[r2, B_TO_D_OFFSET + IDR]            // GPIOD read databus
        lsrs	r0,r0,#8                                // PD8-PD15 -> shift it to 0
        ldr     r1, =mem_mode
        strb    r0,[r1]

#ifdef DEBUG_EXTI14_DATA_OUT
        tst     r0,#1
        ite     eq
        vstreq  reg_led_off,[r12,ODR] //0 ROM
        vstrne  reg_led_on,[r12,ODR]  //1 RAM
#endif

end_EXTI15_10_IRQHandler:
        vmov	r3,reg_exti_base
        ldr     r3,=EXTI
        ldr     r0,[r3,IMR]          // EXTI_IMR
        orr     r0,#EXTI_Line14      // reenable PB14 interrupts

        vstr	reg_bit14_high,[r3,PR]                     // EXTI_PR 
        str     r0,[r3,IMR]   // EXTI_IMR	// I think you need to reenable after the clear

#ifdef DEBUG_EXTI14_END
        delay 4
        vstr     reg_led_off,[r12,ODR]
#endif
        dsb
        bx      lr
