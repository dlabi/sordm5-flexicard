#define _VER_ 15.3

#include "main.h"

	.syntax unified
	.cpu cortex-m4
//	.fpu softvfp
        .fpu fpv4-sp-d16
	.thumb

	.global EXTI0_IRQHandler
	.global EXTI4_IRQHandler
        .global	init_fpu_regs
	.global rom_base
        .global basic_f
        .global basic_g
        .global basic_i
        .global msx
        .global low_64k_base
	.global high_64k_base
	.global main_thread_command
	.global main_thread_data
        .global menu_ctrl_file_count
        .global mem_mode
        .global offset1000
        .global debug_var1
        .global debug_var2


/* REGISTERS */
#define reg_zero	        s0
#define reg_bit0_high	        s1
#define reg_bit1_high	        s2
#define reg_bit2_high           s3
#define reg_bit3_high           s4
#define reg_bit4_high           s5
#define reg_gpioa_base          s6
#define reg_gpiob_base          s7
#define reg_high_64k_base       s8
#define reg_rom_base_2000       s9
#define reg_exti_base           s10
#define reg_debug_var1	        s11   
#define reg_moder_dataout	s12
#define reg_moder_datain	s13
#define reg_led_on	        s14
#define reg_led_off	        s15

// NOTE: FPU registers d8 and d9 are used somehow, so you cannot use s16,s17,s18 and s19
// s16-s30 nejsou automaticky pushovany a nemeli byse menit

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


.section .rodata
// Can put up to four 16K roms here which will get mapped to rom 12, 13, 14 and 15

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
        .equ BSRR            ,   0x18

        .equ    EXTI_Line0      ,       0x00001
        .equ    EXTI_Line1      ,       0x00002
        .equ    EXTI_Line2      ,       0x00004
        .equ    EXTI_Line4      ,       0x00010
        .equ    EXTI_Line14     ,       0x04000
        .equ    EXTI            ,       0x40013c00

	.equ	IMR		,	0x00
	.equ	EMR		,	0x04
	.equ	RTSR		,	0x08
        .equ	FTSR		,	0x0c
	.equ	PR		,	0x14

        .equ    SYSCFG_BASE     ,       0x40013800
        .equ    SYSCFG_EXTICR1  ,       SYSCFG_BASE + 0x08


#define	DATA_OUT_MODE	0x55550020
#define	DATA_IN_MODE	0x00000020

#define _MREQ_MASK	0x0010
#define _EXIOA_MASK     0x0100
#define _MRD_MASK	0x0800
#define _MWR_MASK	0x1000
#define _IORD_MASK	0x2000
#define _IOWR_MASK	0x0001  //0x4000 PRO PB14



.section .data

//high_64k_base:
//.incbin "../roms/msx_dump.rom"
// main_thread_data must immediately follow main_thread_command
main_thread_command:
	.byte	0x00			// command byte
main_thread_data:
	.byte	0x00			// command arg
menu_ctrl_file_count:
        .word   0x00000000
mem_mode:
        .byte   0x00                    // memory mode
offset1000:
        .byte   0x00                    //non zero value offsets high_64k_base by +1000h to allow read 7000-7fff region at 8000-8fff
debug_var1:
	.word	0x00000000
//debug_var2:
//	.word	0x00000000        

rom_base:
basic_f:
.incbin "../roms/basic-f.rom"
msx:
.incbin "../roms/msx.rom"

.section ccmram
low_64k_base:
        //used as buffer for file list from SD card
	// low_64k_base consumes the entire 64K CCMRAM . Look in stm32_flash.ld for where the global symbol is defined
//.lcomm low_64k_base,0x10000  
basic_g:                    
.incbin "../roms/basic-g.rom"
basic_i:
.incbin "../roms/basic-i.rom"
.section .bss
        .lcomm high_64k_base,0x10000                

//* signs used in sw
//*PA0-PA3 is a debug output for hooking up to a logic analyser to check timings, but is not connected normally

//*PB0 = IOWR
//*PB1 = IORD
//PB3 - RFSH            nutno zrusit kvuli swo debug
//PB5 - WAIT
//PB6 - EXINT
//PB7 - EXCLK
//PB8 - EXIOA           &60 - &6f
//PB9 - EXIOB           &70 - &7F
//*PB10 - RST                           kandidat na predratovani PC3
//*PB11 - MRD            -RD & -MREQ    kandidat na predratovani PC1
//*PB12 - MWR            -WR & MREQ     kandidat na predratovani PC2
//PB13 - IORD           -RD & -IORQ     replaced by PB1
//PB14 - IOWR           -WR & -IORQ     replaced by PB0
//PB15 - ROMDS

//*PC0 - ROM0	aktivni pri	&0000 - &1FFF, pouzito pro odpojeni MONITOR ROM nastavenim vystupu na high. Musi byt upraveny Sord
//PC1 - ROM1	aktivni pri	&2000 - &3FFF
//PC2 - ROM2	aktivni pri 	&4000 - &5FFF	
//PC3 - EXM    	aktivni pri     &6000 - &6FFF
//*PC4 - MRQ
//PC13 - CLK

//*PD8-PD15  D0 - D7 
//*PE0-PE15  A0 - A15

//As mentioned, most STM32F4 boards with an SD card adapter are wired the same way. The important pins are:

//*PC11 -  /CS
//*PD2 -  MOSI
//*PC12 -  SCK 
//*PC8  - MISO

.section .text

.type init_fpu_regs, %function
.align 4
.thumb_func
init_fpu_regs:
	mov	r0,#0
	vmov	reg_zero,r0

	mov	r0,#0x01
	vmov	reg_bit0_high,r0

	mov	r0,#0x02
	vmov	reg_bit1_high,r0

        mov	r0,#0x04
	vmov	reg_bit2_high,r0

        mov	r0,#0x08
	vmov	reg_bit3_high,r0
        
        mov	r0,#0x10
	vmov	reg_bit4_high,r0

	//mov32	r0,GPIOC_BASE
	//vmov	reg_gpioc_base,r0

	mov32	r0,debug_var1
	vmov	reg_debug_var1,r0

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

        mov32	r0,rom_base - 0x2000            //neni pak treba menit adr v r1, a ROM je tak "virtualne" od 2000
	vmov	reg_rom_base_2000,r0

        mov	r0,#0x44
	vmov	reg_led_on,r0

        mov	r0,#0xc0
        vmov    reg_led_off,r0

	//mov32	r0,0x10000000
	//vmov	reg_ccmram_log,r0

	bx	lr

//************************************************************************************************************************
// _MREQ Handler. Interrupt on falling edge

// meaning of 'global' registers, all should be pushed/popped automatically
// r0 - tmp
// r1 - address from GPIOE and also used for switch MODER to either OUTPUT or INPUT
// r2 - reg_gpiob_base
// r3 - tmp, exti_base
// r12 - reg_gpioa_base - debug register

.type EXTI4_IRQHandler, %function
.align 4
.thumb_func
EXTI4_IRQHandler:
#ifdef ENABLE_SWO 
        vmov    r12,reg_debug_var1
#else
        vmov	r12,reg_gpioa_base
#endif

#ifdef DEBUG_EXTI4_START
        vstr    reg_bit3_high,[r12,BSRR]
#endif
       
clear_pending_iorq_int:                                 
	vmov	r3,reg_exti_base		        
	vstr	reg_bit4_high,[r3,PR]	                // clear pending flag of our interrupt

        vmov	r2,reg_gpiob_base
        ldr     r0,[r2, B_TO_C_OFFSET + IDR]            // GPIOC->IDR
        tst     r0,#_MREQ_MASK
        bne     end_EXTI4_IRQHandler                    // exit spurious IRQ

        ldr     r1,[r2, B_TO_E_OFFSET + IDR]            // GPIOE->IDR get the address

test_internal_RAM:             
        cmp     r1, #0x7000
        blt     read_write
        cmp     r1, #0x8000
#ifdef DEBUG_EXTI4_INTERNAL_RAM
        IT LT
        vstrlt    reg_bit0_high,[r12,BSRR]
#endif        
        blt     end_EXTI4_IRQHandler                    // internal RAM 7000-7fff get out 

read_write:
        ldr     r0,[r2, IDR]                            // GPIOB->IDR get mrd, mwr    TODO: POKUD PREDELAM MRD, MWR na PC pak toto cteni usetrime
        tst     r0,#_MRD_MASK                           // check MRD, if log. 0 then we present data, if not write to RAM

        ldr     r0, =mem_mode                           // get mem working mode
        ldrb    r0, [r0]
#ifdef  DEBUG_SWO  
        and     r0, #0x0f                               //pri swo debugu nastavuji 8. bit jako dirty indikator, podle ktereho tiskneme pouze pri zmene modu 
#endif
#ifdef  DEBUG_SWO_ADDRESS       
        str     r1, [r12]
#endif
        beq     read_memory

write_check:                                            // same as 64kbf ver 2c
        ldr     r3, =mem_map
        ldr     r3, [r3,r0, lsl #2]                     // get write starting offset from mem_map table
        cmp     r1, r3
        bge     writable
        b       end_EXTI4_IRQHandler

mem_map:                                                // mem mode write address start
    .word 0x8000                                        // mode 0
    .word 0x0000
    .word 0x0000
    .word 0x8000
    .word 0x4000
    .word 0x0000
    .word 0x0000
    .word 0x10000                                       // mode 7 - write disabled
.align 4                                                                  
writable:
        vmov    r3, reg_high_64k_base                   // begining of RAM 
	delay   18                                        //wait to safely detect if MWR is happening, 15 chodi ale semtam na hrane s MWR 0-42ns, 20: 42-83ns mezi MWR a RAM_WRITE a to chodi
#ifdef DEBUG_EXTI4_RAM_WRITE
        vstr    reg_bit1_high,[r12,BSRR]
#endif
        ldr     r0,[r2, IDR]                            // GPIOB->IDR get mrd, mwr    
        tst     r0,#_MWR_MASK                           // check MWR, if log. 0 then read databus, otherwise refreshing DRAM goin on->exit 
        bne     end_EXTI4_IRQHandler
	ldr	r0,[r2, B_TO_D_OFFSET + IDR]            // GPIOD read databus
	lsrs	r0,r0,#8                                // PD16-PD8 >> 8
	strb	r0,[r3,r1]                              // write to the RAM
#ifdef DEBUG_SWO_WRITE
        str     r1,[r12]
        str     r0,[r12, #4]
#endif
        b       end_EXTI4_IRQHandler                    // write done
.align 4
jump_table:
        .word   mode0_5                                 // mode 0
        .word   mode1_3_4
        .word   mode2
        .word   mode1_3_4
        .word   mode1_3_4
        .word   mode0_5
        .word   mode6                                   // mode 6
        .word   mode7                                   // mode 7
.align 4
read_memory:
#ifdef DEBUG_EXTI4_MODE_TEST
        vstr    reg_bit0_high,[r12,ODR]
#endif

        ldr	r3, =jump_table
	ldr	r3, [r3, r0, lsl #2]                    // writing directly to pc, didn't work for me, possible usage of dsb instruction???
        //dsb
        mov     pc, r3                                  

mode0_5:
        tst     r1,#0x8000
        bne     ram                                     // 8000-ffff read from RAM
        cmp     r1,#0x2000                              
        blt     end_EXTI4_IRQHandler                    // 0000-1fff read from Sord ROM
        vmov	r3,reg_rom_base_2000                    // rombase virtually starting at 2000
        //sub     r1, #0x2000                           // important, subtract 0x2000 'cos our rom is from 0000 at rom_base area
        b       read                                    // 2000-6fff read from ROM
mode1_3_4:                                              
        b       ram                                     // 0000-ffff read from RAM
mode2:  cmp     r1,#0x2000                              
        blt     end_EXTI4_IRQHandler                    // 0000-1fff read from Sord ROM
        b       ram                                     // 2000-ffff read from RAM
mode6:  tst     r1,#0x8000                              
        bne     ram                                     // 8000-ffff read from RAM
                                                        // 0000-1fff read from Sord ROM
mode7:  b       end_EXTI4_IRQHandler                    // 2000-6fff read from other cartridge

rom:                                                    // cokoliv mimo rom range = exit, tj. 0-1fff, >7000 a pak podle typu rom
//        vmov	r3,reg_rom_base                       
//        ldrb    r0,[r3]                               // get rom type: 00: 2000-3fff, 01: 4000-5fff, 02: 2000-5fff
//        sub     r1, #0x2000                           // offset address from 0
//        cmp     r0,#1
//        IT EQ
//        subeq   r1,#0x2000                            // rom type 01 from 4000: offset address from 0
//        ITE GT                                          
//        movgt   r0,#0x4000                            // type 02
//        movle   r0,#0x2000                            // type 00
//        cmp     r1,r0                                 // test if we are in rom range
//        bcs.n   end_EXTI4_IRQHandler                  // no we aren't -> exit
//        b       read

#ifdef  DEBUG_SWO_MONITOR_READ
        str     r1, [r12]
#endif
                                                        // address <2000, MONITOR ROM must be disabled or we have hazard on the bus
testMONITOR_ROM:
        ldr     r0,[r2, B_TO_C_OFFSET + IDR]            // read PC0
        lsrs	r0,r0,#8
        tst     r0,#1                                   // if high then MONITOR ROM must be disabled, without mod now should be low
        beq     end_EXTI4_IRQHandler                    

ram:                                                    // 0000-1fff, 2000-6fff, 8000-ffff
        ldr     r3,=offset1000                          // get state of offset
        ldrb    r3,[r3]
        cmp     r3,#0                                   // is offset1000 active?
        vmov    r3, reg_high_64k_base                   // read from RAM
	it ne
	addne  r3,#0x1000                               // yes add 0x1000   
        
         
#ifdef DEBUG_EXTI4_RAM_READ
        vstr     reg_bit0_high,[r12,BSRR]               // sets bit 0 high
#endif

read:
#ifdef DEBUG_EXTI4_MRD
        vstr     reg_bit1_high,[r12,BSRR]               // sets bit 1 high
#endif
        ldrb    r0,[r3,r1]                              // get the byte from a 16K ROM or RAM
        lsls    r0,r0, #8                               // shift to PD15-PD8
        
        vmov	r1,reg_moder_dataout

        str     r0,[r2, B_TO_D_OFFSET + ODR]            // write data to the bus -> GPIOD ODR
        str     r1,[r2, B_TO_D_OFFSET + MODER]          // MODER set to outputs

#ifdef DEBUG_EXTI4_DATA_OUT
        vstr     reg_bit2_high,[r12,BSRR]                   // sets bit 2 only
#endif

// Use WFE to wait until _MREQ goes high
        vmov	r3,reg_exti_base

        //zakaz interupt protoze jinak nam bude reagovat i INT na nabeznou hranu a my chceme jen event
        ldr     r0,[r3,IMR]             // EXTI_IMR
        and     r0,#0xffffffef          // turn off  PC4 int
        str     r0,[r3,IMR]             // EXTI_IMR
        
        vstr    reg_bit4_high,[r3,PR]                   // EXTI_PR  - clear the current PC4 interrupt
        set_EXTI4_RTSR                                  // povol nabeznou hranu na EXTI4, abychom tristatovali co nejdrive, pouziva r3
        vstr    reg_bit4_high,[r3,EMR]                  // EXTI_EMR - wait for an event rather than an interrupt on PC4
       
        vmov	r1,reg_moder_datain

	dsb                                             
        sev                                             // set the event register
        wfe                                             // clear the event register
        // So the 2nd wfe is meant to wait for the +ve edge, but if you get a spurious edge at exactly the time these instructions execute, it can slip through without waiting
        wfe                                             // wait for the positive edge of _MREQ

       // delay 3         //nevim zdali nutne, testuji nestabilitu 3 = 42ns

// _MREQ should have just gone high here
        str     r1,[r2, B_TO_D_OFFSET + MODER]          // tristate databus

#ifdef DEBUG_EXTI4_TRISTATED
        mov     r0,#0x00040000                          // clears bit 2
        str     r0,[r12,BSRR]
#endif
        clear_EXTI4_RTSR                                // zakaz INT na nabeznou hranu, pouziva r3

end_EXTI4_IRQHandler:           

        vmov	r3,reg_exti_base
        ldr     r3,=EXTI
        ldr     r0,[r3,IMR]                             // EXTI_IMR
        mov     r1,#EXTI_Line4
        orr     r0,r1                                   // reenable PC4 interrupts
        str     r1,[r3,PR]                              // clear pending EXTI_PR 
        str     r0,[r3,IMR]                             // EXTI_IMR	

unrolled_exit_EXTI4


//******************************************************************************************************************************************

#ifdef UNDEFINED

// _IORW interrupt -ve edge
.type EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

        //cpsid	i                                       // disable irq
#ifdef ENABLE_SWO 
        vmov    r12,reg_debug_var1
#else
        vmov	r12,reg_gpioa_base
#endif
#ifdef DEBUG_EXTI0_START
        vstr     reg_bit2_high,[r12,BSRR]               // sets bit 2 only, indikuje aktivni int0
#endif                      

clear_pending_iorq_int0:                                // musi byt jinak nezachytime vsechny edge
/*	vmov	r3,reg_exti_base		        // s7 = EXTI
        ldr     r0,[r3,PR]
        tst     r0,#EXTI_Line0
        beq     end_EXTI0_IRQHandler1
 */        
//	vstr	reg_bit0_high,[r3,PR]	                // clear interrupt

        vmov	r2,reg_gpiob_base
        ldr     r0,[r2, IDR]                            // GPIOB->IDR get IOWR
        tst     r0,#_IOWR_MASK
        bne     end_EXTI0_IRQHandler                    // exit spurious IRQ
     
	ldr     r1,[r2, B_TO_E_OFFSET + IDR]            // GPIOE->IDR get the address
	and     r1, #0xff                               // get the rid of upper address byte, it's important otherwise next check fails!  
        cmp     r1, #0x30
        bne	end_EXTI0_IRQHandler
	ldr	r0,[r2, B_TO_D_OFFSET + IDR]            // GPIOD read databus
        lsrs	r0,r0,#8                                // PD8-PD15 -> shift to 0

#ifdef DEBUG_EXTI0_MEM_MODE
        str     r0, [r12,ODR]                                     
#endif
 
        ldr     r1, =mem_mode
        strb    r0,[r1]                                 // set mem_mode
        mov     r3, #0b10011010                         // MONITOROM disable map: bit 7 = mode 7, bit 6=mode 6 etc., 1=disabled MONITOR, 0=enabled
        mov     r1, #1
        lsl	r1, r0
        tst     r3, r1                                  // test bit from map for current mem_mode
        ldr	r1,[r2, B_TO_C_OFFSET + MODER] 
        bne     disableMONITOR                          // if is set disable MONITOR ROM
        
#ifdef DEBUG_EXTI0_MEM_MODE_LED
        ite     eq
        vstreq  reg_led_off,[r12,ODR]                   
        vstrne  reg_led_on,[r12,ODR]                    
#endif

enableMONITOR:    
        bic     r1,#3                                   // set PC0 as input pin -> high impedance
        str     r1,[r2, B_TO_C_OFFSET + MODER]
        dsb       
        b       end_EXTI0_IRQHandler

disableMONITOR:
        mov     r0, #1
        str     r0,[r2, B_TO_C_OFFSET + BSRR]           // set PC0 to high which disable MONITOR rom         
        bic     r1, #2                                  // clears bit 1  
        orr     r1, #1                                  // sets bit 0
        str     r1,[r2, B_TO_C_OFFSET + MODER]          // set PC0 to output
        dsb                                             

end_EXTI0_IRQHandler:
        vmov	r3,reg_exti_base
        ldr     r3,=EXTI
        ldr     r0,[r3,IMR]                             // EXTI_IMR
        orr     r0,#EXTI_Line0                          // reenable PB0 interrupts

        vstr	reg_bit0_high,[r3,PR]                   // EXTI_PR 
        str     r0,[r3,IMR]                             // EXTI_IMR	// I think you need to reenable after the clear

#ifdef DEBUG_EXTI0_END
        //delay 4
        mov     r0,#0x00060000                          // clears bits 1. and 2.
        str     r0,[r12,BSRR]
#endif
        //cpsie	i                                       // enable interrupts
        dsb
        bx      lr

#endif