#define _VER_ 1.3

#include "main.h"

	.syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.thumb

	.global EXTI1_IRQHandler
	.global EXTI4_IRQHandler
	.global romdis
	.global rom_base
	.global main_thread_command
	.global main_thread_data



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

 //       .equ C_TO_B_OFFSET,     (GPIOC_BASE - GPIOB_BASE)
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

        .equ ROMDIS_HIGH     ,   0x0008

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
	.equ	PR		,	0x14

	.equ	DSK_TRACKINFO_NUMBEROFSECTORS 	, 0x15
	.equ	DSK_TRACKINFO_SECTORINFOLIST 	, 0x18


    .equ        SYSCFG_BASE,    0x40013800
    .equ        SYSCFG_EXTICR1, SYSCFG_BASE + 0x08


#define	DATA_OUT_MODE	0x55550020
#define	DATA_IN_MODE	0x00000020

//#define Z80_WRITE_MASK	0x0001
//#define _IORQ_MASK	0x0002
//#define _ROMEN_MASK	0x0004
#define _MREQ_MASK	0x0010
#define _MRD_MASK	0x0800
#define _MWR_MASK	0x1000
#define _IORD_MASK	0x2000
#define _IOWR_MASK	0x4000

#define ROMDIS_HIGH	0x0008

#define LOWEST_ROM_ID	6
#define SUPPORTED_NUMBER_OF_ROMS	2
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


//PD15 - PD8 - connect to D7 to D0 on Amstrad
//PE15 - PE0 - connect to A15 to A0 on Amstrad
//PC0 - Amstrad /WR
//PC1 - Amstrad /IORQ
//PC2 - Amstrad /ROMEN
//PC3 - Amstrad ROMDIS
//PC4 - Amstrad /MREQ

//PA0 is a debug output for hooking up to a logic analyser to check timings, but is not connected normally

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

// _MREQ Handler. Int on +ve edge

.type EXTI4_IRQHandler, %function
EXTI4_IRQHandler:
        ldr     r12,=GPIOA_BASE
        // assume enter on rising edge
#ifdef DEBUG_EXTI41
        movs    r0,#0xc1                                  //PA0 - 0 PA1 - 0
        str     r0,[r12,ODR]
#endif

        ldr     r2,=GPIOC_BASE
        ldr     r5,=GPIOB_BASE
EXTI4_Start_Polling_For_MREQ_LOW:
3$:      ldr     r1,[r2, C_TO_E_OFFSET + IDR]            // GPIOE->IDR get the addres
        ldr     r4,[r5, IDR]                            // GPIOB->IDR get mrd, mwr
        ldr     r0,[r2,IDR]                             // GPIOC->IDR get mreq
        //tst     r0,#_MREQ_MASK                          // check _MREQ
        tst     r4,#_MRD_MASK                          // check MRD, if log. 0 then we present data, if not wait for it
        bne.n   3$
#ifdef DEBUG_EXTI42
        movs    r0,#0xc2                                   //PA0 - 0 PA1 - 1
        str     r0,[r12,ODR]
#endif
        //tst     r1,#0x8000                              // check A15. If it is high. We are interested
        sub     r4, r1, #0x2000                           // je adr mezi 0x2000-0x6fff ?
	cmp     r4, #0x5000        
        bcs.n     end_EXTI4_IRQHandler

#ifdef DEBUG_EXTI43
        movs    r0,#0xc3
        str     r0,[r12,ODR]
#endif

        //beq.n   end_EXTI4_IRQHandler
	//nop				                // You need at least one NOP here for the delay between _MREQ and _ROMEN
	//nop
        //ldr     r0,[r2,IDR]   // GPIOC->IDR             // reread _ROMEN as it goes low marginally after _MREQ
        //tst     r0,#_ROMEN_MASK                         // check _ROMEN
        //bne.n   end_EXTI4_IRQHandler
        ldr     r3,=rom_base
        ldrb    r0,[r3,r4]              // get the byte from a 16K ROM
        lsls    r0,r0, #8
        ldr     r1,=DATA_OUT_MODE
        str     r0,[r2, C_TO_D_OFFSET + ODR] // GPIOD ODR
        str     r1,[r2, C_TO_D_OFFSET + MODER]  // MODER set to outputs

#ifdef DEBUG_EXTI4
        movs    r0,#0x44
        str     r0,[r12,ODR]
#endif

// Use WFE to wait until _MREQ goes high
        ldr     r3,=EXTI
        //movs    r0,#0
        movs    r1,#EXTI_Line4
        ldr     r0,[r3,IMR]     // EXTI_IMR
        and     r0,#0xffffffef  // turn off  PC4 int
        str     r0,[r3,IMR]     // EXTI_IMR
        str     r1,[r3,PR]      // EXTI_PR  - clear the current PC4 interrupt
        str     r1,[r3,EMR]     // EXTI_EMR - wait for an event rather than an interrupt on PC4

        mov     r1,#DATA_IN_MODE

	dsb             // not entirely sure if this is required
        sev             // set the event register
        wfe             // clear the event register
        // So the 2nd wfe is meant to wait for the +ve edge, but if you get a spurious edge at exactly the time these instructions execute, it can slip through without waiting
        wfe             // wait for the positive edge of _MREQ

#ifdef DEBUG_SPURIOUS_EVENT_MREQ
        ldr     r0,[r2,IDR]                     // GPIOC->IDR
        tst     r0,#_MREQ_MASK                  // check _MREQ
        bne.n   8$
        movs    r0,#0
        str     r0,[r12,ODR]
        nop
        add     r0,#1
        str     r0,[r12,ODR]
8$:
#endif

// _MREQ should have just gone high here
        str     r1,[r2, C_TO_D_OFFSET + MODER]   // tristate databus

#ifdef DEBUG_EXTI45
        movs    r0,#0xc5
        str     r0,[r12, ODR]
#endif
       // b.n     EXTI4_Start_Polling_For_MREQ_LOW      // loop again instead of exitting the ISR

end_EXTI4_IRQHandler:
#ifdef DEBUG_EXTI4
        movs    r0,#0xc0                           //PA0 - 1 PA1 - 1
        str     r0,[r12, ODR]
#endif
        ldr     r3,=EXTI
        ldr     r0,[r3,IMR]     // EXTI_IMR
        orr     r0,#0x0010      // reenable PC4 interrupts
        //str     r0,[r3,IMR]   // EXTI_IMR

        movs    r1,#EXTI_Line4
        str     r1,[r3,PR]                      // EXTI_PR 
        str     r0,[r3,IMR]   // EXTI_IMR	// I think you need to reenable after the clear
      //  ldr     r0,=EXTI4_tempvar               // You need to read-modify-write to prevent double interrupts
      //  str     r1,[r0]

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
	subs	    r0,r0,#LOWEST_ROM_ID
	bmi.n	    end_EXTI1_IRQHandler
	cmp	    r0,#SUPPORTED_NUMBER_OF_ROMS
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


