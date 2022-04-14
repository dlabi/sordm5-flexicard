#define _VER_ 3.5

#include "main.h"

	.syntax unified
	.cpu cortex-m4
//	.fpu softvfp
        .fpu fpv4-sp-d16
	.thumb

	.global EXTI1_IRQHandler
	.global EXTI4_IRQHandler
        .global	init_fpu_regs
//	.global romdis
	.global rom_base
	.global main_thread_command
	.global main_thread_data


/* REGISTERS */
#define reg_zero	s0
#define reg_bit0_high	s1
#define reg_bit1_high	s2
#define reg_gpioc_base	s3
#define reg_bit2_high	s4

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


.macro databus_read_extra_delay
	.rept 0
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

  //      .equ ROMDIS_HIGH     ,   0x0008

        .equ SCB_AIRCR       , 0xE000ED0C


        .equ    EXTI_Line1      ,       0x00002
        .equ    EXTI_Line2      ,       0x00004
        .equ    EXTI_Line4      ,       0x00010
        .equ    EXTI            ,       0x40013c00
        .equ    EXTI_IMR        ,       EXTI + 0x00
        .equ    EXTI_PR         ,       EXTI + 0x14

	.equ	IMR		,	0x00
	.equ	EMR		,	0x04
	.equ	RTSR		,	0x08
        .equ	FTSR		,	0x0c
	.equ	PR		,	0x14

	.equ	DSK_TRACKINFO_NUMBEROFSECTORS 	, 0x15
	.equ	DSK_TRACKINFO_SECTORINFOLIST 	, 0x18


    .equ        SYSCFG_BASE,    0x40013800
    .equ        SYSCFG_EXTICR1, SYSCFG_BASE + 0x08


#define	DATA_OUT_MODE	0x55550020
#define	DATA_IN_MODE	0x00000020

#define _MREQ_MASK	0x0010
#define _MRD_MASK	0x0800
#define _MWR_MASK	0x1000
#define _IORD_MASK	0x2000
#define _IOWR_MASK	0x4000


.section .data
// rom_bank must immediately follow romdis
romdis:
	.word   0x00000000
rom_bank:
        .word   0x00000000


// main_thread_data must immediately follow main_thread_command
main_thread_command:
	.word	0x00000000			// command word
main_thread_data:
	.word	0x00000000			// command arg

.section .bss
        .lcomm  EXTI1_tempvar,4
        .lcomm  EXTI2_tempvar,4
        .lcomm  EXTI4_tempvar,4



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

	movs	r0,#4
	vmov	reg_bit2_high,r0

	mov32	r0,GPIOA_BASE
	vmov	reg_gpioa_base,r0

	//mov32	r0, high_64k_base
	//vmov	reg_high_64k_base,r0

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
	vmov	reg_gpiob_base,r0

        movs	r0,#0x44
	vmov	reg_led_on,r0

        movs	r0,#0xc0
	vmov	reg_led_off,r0

	//mov32	r0,0x10000000
	//vmov	reg_ccmram_log,r0

	bx	lr


// _MREQ Handler. Int on -ve edge

.type EXTI4_IRQHandler, %function
EXTI4_IRQHandler:
        vmov	r11,reg_gpioa_base

        // assume enter on rising edge
#ifdef DEBUG_EXTI4_START
        vstr     reg_bit0_high,[r11,ODR]
        //nop
        //vstr     reg_zero,[r11,ODR]
#endif
        vmov	r2,reg_gpioc_base
        vmov	r5,reg_gpiob_base

clear_pending_iorq_int:
	vmov	r3,reg_exti_base		// s7 = EXTI
	vstr	reg_bit0_high,[r3,PR]	        // clear interrupt

        ldr     r1,[r2, C_TO_E_OFFSET + IDR]            // GPIOE->IDR get the addres
        
        ldr     r4,[r5, IDR]                            // GPIOB->IDR get mrd, mwr 
#ifdef DEBUG_EXTI4
        ror     r0, r4, #14
        and     r0, #2
        str    r0, [r11,ODR]
        //vstr     reg_bit1_high,[r11,ODR]
#endif
        
      //0x8000-0xffff

        tst     r4,#_MRD_MASK                          // check MRD, if log. 0 then we present data, if not exit INT
        bne     end_EXTI4_IRQHandler

        tst     r1,#0x8000                              // check A15. If it is high. We are interested
        beq     end_EXTI4_IRQHandler
        
        
        sub     r4, r1, #0x8000

 /*   
        sub     r4, r1, #0x2000                           // je adr mezi 0x2000-0x6fff ?
	cmp     r4, #0x2000                             //#0x5000 pro 0x6fff      
        bcs.n     end_EXTI4_IRQHandler
 */ 

#ifdef DEBUG_EXTI43
        vstr     reg_bit1_high,[r11,ODR]
#endif
        vmov	r3,reg_rom_base
        ldrb    r0,[r3,r4]              // get the byte from a 16K ROM
        lsls    r0,r0, #8
        
        vmov	r1,reg_moder_dataout

        str     r0,[r2, C_TO_D_OFFSET + ODR]    // GPIOD ODR
        str     r1,[r2, C_TO_D_OFFSET + MODER]  // MODER set to outputs

#ifdef DEBUG_EXTI4
        vstr     reg_led_on,[r11,ODR]
#endif

// Use WFE to wait until _MREQ goes high

/*
        vmov	r3,reg_exti_base
	set_EXTI4_FTSR
 */       
        vmov	r3,reg_exti_base
        movs    r1,#EXTI_Line4
        ldr     r0,[r3,IMR]     // EXTI_IMR
        and     r0,#0xffffffef  // turn off  PC4 int
        str     r0,[r3,IMR]     // EXTI_IMR
        str     r1,[r3,PR]      // EXTI_PR  - clear the current PC4 interrupt
        str     r1,[r3,EMR]

//        vstr     reg_bit4_high,[r3,EMR] // EXTI_EMR - wait for an event rather than an interrupt on PC4

        set_EXTI4_RTSR          // povol nabeznou hranu na EXTI4, abychom tristatovali co nejdrive, pouziva r3! 

        vmov	r1,reg_moder_datain

	dsb             // not entirely sure if this is required
        sev             // set the event register
        wfe             // clear the event register
        // So the 2nd wfe is meant to wait for the +ve edge, but if you get a spurious edge at exactly the time these instructions execute, it can slip through without waiting
        wfe             // wait for the positive edge of _MREQ

        //databus_read_extra_delay

// _MREQ should have just gone high here
        str     r1,[r2, C_TO_D_OFFSET + MODER]   // tristate databus

#ifdef DEBUG_EXTI45
        movs    r0,#0xc5
        str     r0,[r11, ODR]
#endif
        clear_EXTI4_RTSR //zakaz INT na nabeznou hranu, pouziva r3

end_EXTI4_IRQHandler:
#ifdef DEBUG_EXTI4
        vstr     reg_led_off,[r11,ODR]
#endif
        
//        clear_EXTI4_FTSR

        vmov	r3,reg_exti_base
        //ldr     r3,=EXTI
        ldr     r0,[r3,IMR]     // EXTI_IMR
        orr     r0,#0x0010      // reenable PC4 interrupts
        //str     r0,[r3,IMR]   // EXTI_IMR

        movs    r1,#EXTI_Line4
        str     r1,[r3,PR]                      // EXTI_PR 
        str     r0,[r3,IMR]   // EXTI_IMR	// I think you need to reenable after the clear
      //  ldr     r0,=EXTI4_tempvar               // You need to read-modify-write to prevent double interrupts
      //  str     r1,[r0]

        dsb
        bx      lr



// _IORQ interrupt -ve edge
.type EXTI1_IRQHandler, %function
EXTI1_IRQHandler:
	ldr	    r2,=GPIOC_BASE
        ldr         r12,=GPIOA_BASE

	ldr	    r1,[r2, C_TO_E_OFFSET + IDR]		// GPIOE->IDR get the address 
	ldr	    r0,[r2,IDR]	                        // get the portc control pins

	// NOTE: Parados wants to see a floppy chip
    //    tst     r1,#0x0480      // check A10/A8
    //    beq.n   fdc_chip_selected

	tst	    r1,#0x2000	// check A13 is low
	bne.n	    end_EXTI1_IRQHandler
	tst	    r1,#0x8000	// check A15 is high
	beq.n	    end_EXTI1_IRQHandler
	//tst	    r0,#Z80_WRITE_MASK	// check r/w is low
	//bne.n	    end_EXTI1_IRQHandler
// Most likely a write to the ROM select register
	ldr	    r3,=rom_bank
	ldr	    r0,[r2, C_TO_D_OFFSET + IDR] // GPIOD IDR. Get the byte written to the rom select reg
	lsrs	    r0,r0, #8
	ands	    r0,r0,#0xff
	str	    r0,[r3]		// save the rom select register
	movs	    r1,#0
	ldr	    r3,=romdis
	str	    r1,[r3]		// save potential romdis as LOW
	//subs	    r0,r0,#LOWEST_ROM_ID
	bmi.n	    end_EXTI1_IRQHandler
	//cmp	    r0,#SUPPORTED_NUMBER_OF_ROMS
	bge.n	    end_EXTI1_IRQHandler
	movs	    r1,#0x0008
	
  	str	    r1,[r3]		// save potential romdis as HIGH
#ifdef DEBUG_EXTI1
    movs    r0,#0
    str     r0,[r12,ODR]
#endif
	b.n	    end_EXTI1_IRQHandler		// consider repeating the code here instead of jumping

waitIORQHIGH_end_EXTI1_IRQHandler:
#ifdef DEBUG_EXTI1_WFE
	ldr     r12,=GPIOA_BASE
    movs    r0,#0
    str     r0,[r12,ODR]
#endif
// Use WFE to wait until _IORQ goes high
        ldr     r3,=EXTI
        movs    r0,#0
        movs    r1,#EXTI_Line1
        ldr     r0,[r3,IMR]   // EXTI_IMR
        and     r0,#0xfffffffd  // turn off  PC1 int
        str     r0,[r3,IMR]   // EXTI_IMR

        ldr     r0,[r3,RTSR]   // EXTI_RTSR
        orr     r0,#0x0002	// Add in PC1 on rising edge event
        str     r0,[r3,RTSR]   // EXTI_RTSR
	
        str     r1,[r3,PR]   // EXTI_PR  - clear the current PC1 interrupt
        str     r1,[r3,EMR]   // EXTI_EMR - wait for an event rather than an interrupt on PC4

        mov     r1,#DATA_IN_MODE
        movs    r0,#3   // set PA0/PA1 high again
6:      dsb             // not entirely sure if this is required
        sev     // set the event register
        wfe     // clear the event register
        wfe     // wait for the positive edge of _MREQ

// _IORQ should have just gone high here
#ifdef DEBUG_EXTI1_WFE
        str     r0,[r12, ODR]
#endif
        str     r1,[r2, C_TO_D_OFFSET + MODER]   // tristate databus

        ldr     r0,[r3,RTSR]   // EXTI_RTSR
        and     r0,#0xfffffffd	// remove PC1 on rising edge event
        str     r0,[r3,RTSR]   // EXTI_RTSR

        // Don't loop. The chances of two IO requests in a row are very low

end_EXTI1_IRQHandler:
#ifdef DEBUG_EXTI1
        ldr     r12,=GPIOA_BASE
	    movs	r0,#1
        str     r0,[r12, ODR]
#endif
        ldr     r3,=EXTI
        ldr     r0,[r3,IMR]   // EXTI_IMR
        orr     r0,#0x0002      // reenable PC1 interrupt
        //str     r0,[r3,IMR]   // EXTI_IMR

        movs    r1,#EXTI_Line1
        str     r1,[r3,PR]   // EXTI_PR 
        str     r0,[r3,IMR]   // EXTI_IMR	// reenable PC1
        ldr     r0,=EXTI1_tempvar             // You need to read-modify-write to prevent double interrupts
        str     r1,[r0]

        bx      lr


