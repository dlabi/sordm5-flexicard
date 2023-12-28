	.syntax unified
	.cpu cortex-m4
//	.fpu softvfp
        .fpu fpv4-sp-d16
	.thumb


.section .rodata
#include "definitions.S"
#include "macros.S"

	.global EXTI0_IRQHandler
	.global EXTI4_IRQHandler
        .global	init_fpu_regs
	.global rom_base
        .global basic_f
        .global basic_g
        .global basic_i
        .global basic_f_end
        .global basic_g_end
        .global basic_i_end
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


//rom repository
basic_f:
.incbin "../roms/basic-f.rom"
basic_f_end:

#ifdef MODDED_SORD                                   
msx:
.incbin "../roms/msx.rom"
msx_end:
#endif
#ifdef OTHER_BASICS
basic_g:                    
.incbin "../roms/basic-g.rom"
basic_g_end:
basic_i:
.incbin "../roms/basic-i.rom"
basic_i_end:
#endif

.section .data

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
 

//this part of ram is used for simulating rom area, by default basic-f is here
rom_base:
.incbin "../roms/basic-f.rom"

.section ccmram
low_64k_base:

.section .bss
        .lcomm high_64k_base,0x10000                

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

	mov32	r0,GPIOC_BASE
	vmov	reg_gpioc_base,r0

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

        mov32	r0,rom_base - 0x2000            //makes rom_base "virtually" from 2000h so r1 need not to be adjusted
	vmov	reg_rom_base_2000,r0

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

        vmov	r2,reg_gpioc_base
        ldr     r0,[r2, IDR]                            // GPIOC->IDR
        tst     r0,#_MREQ_MASK
        bne     end_EXTI4_IRQHandler                    // exit spurious IRQ

        ldr     r1,[r2, C_TO_E_OFFSET + IDR]            // GPIOE->IDR get the address

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
        //ldr     r0,[r2, IDR]                            // GPIOB->IDR get mrd, mwr    TODO: POKUD PREDELAM MRD, MWR na PC pak toto cteni usetrime
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
#ifdef DEBUG_EXTI4_RAM_WRITECHECK
        vstr    reg_bit1_high,[r12,BSRR]
#endif        
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
        vmov    r3, reg_high_64k_base                   // beginning of RAM 
	delay   18                                      // wait to safely detect if MWR is happening, value 15 works but time to time fails with MWR 0-42ns, 20: 42-83ns between MWR and RAM_WRITE and this works well
#ifdef DEBUG_EXTI4_RAM_WRITE
        vstr    reg_bit1_high,[r12,BSRR]
#endif
        ldr     r0,[r2, IDR]                            // GPIOC->IDR get mrd, mwr    
        tst     r0,#_MWR_MASK                           // check MWR, if log. 0 then read databus, otherwise refreshing DRAM goin on->exit 
        bne     end_EXTI4_IRQHandler
	ldr	r0,[r2, C_TO_D_OFFSET + IDR]            // GPIOD read databus
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

rom:                                                    // anything outside rom area = exit, i.e 0-1fff, >7000 and then by rom type

#ifdef  DEBUG_SWO_MONITOR_READ
        str     r1, [r12]
#endif
                                                        // address <2000, MONITOR ROM must be disabled or we have hazard on the bus
testMONITOR_ROM:
        ldr     r0,[r2]                                 // read PC0
        lsrs	r0,r0,#8
        tst     r0,#_ROM0_MASK                          // if high then MONITOR ROM must be disabled, without mod now should be low
        beq     end_EXTI4_IRQHandler                    

ram:                                                    // 0000-1fff, 2000-6fff, 8000-ffff
        ldr     r3,=offset1000                          // get state of offset
        ldrb    r3,[r3]
        cmp     r3,#0                                   // is offset1000 active?
        vmov    r3, reg_high_64k_base                   // read from RAM
        beq     read 
	cmp     r1,#0x9000                              // reading 8000-8fff?
        bge     read                                    // no
	sub     r3,#0x1000                              // yes sub 0x1000 , eg. reading from 8000 actually reads from 7000  
        
         
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

        str     r0,[r2, C_TO_D_OFFSET + ODR]            // write data to the bus -> GPIOD ODR
        str     r1,[r2, C_TO_D_OFFSET + MODER]          // MODER set to outputs

#ifdef DEBUG_EXTI4_DATA_OUT
        vstr     reg_bit2_high,[r12,BSRR]               // sets bit 2 only
#endif

// Use WFE to wait until _MREQ goes high
        vmov	r3,reg_exti_base

        //disable interrupt otherwise it would be triggered by incoming rising edge too and we want just event
        ldr     r0,[r3,IMR]             // EXTI_IMR
        and     r0,#0xffffffef          // turn off  PC4 int
        str     r0,[r3,IMR]             // EXTI_IMR
        
        vstr    reg_bit4_high,[r3,PR]                   // EXTI_PR  - clear the current PC4 interrupt
        set_EXTI4_RTSR                                  // enable rising edge on EXTI4, to tristatate bus ASAP, uses r3
        vstr    reg_bit4_high,[r3,EMR]                  // EXTI_EMR - wait for an event rather than an interrupt on PC4
       
        vmov	r1,reg_moder_datain

	dsb                                             
        sev                                             // set the event register
        wfe                                             // clear the event register
        // So the 2nd wfe is meant to wait for the +ve edge, but if you get a spurious edge at exactly the time these instructions execute, it can slip through without waiting
        wfe                                             // wait for the positive edge of _MREQ

       // delay 3                                       //don't know if necessary, testing instability 3 = 42ns

// _MREQ should have just gone high here
        str     r1,[r2, C_TO_D_OFFSET + MODER]          // tristate databus

#ifdef DEBUG_EXTI4_TRISTATED
        mov     r0,#0x00040000                          // clears bit 2
        str     r0,[r12,BSRR]
#endif
        clear_EXTI4_RTSR                                // disable rising edge for INT, uses r3

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
// now we are handling IOWR in main.c
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
        vstr     reg_bit2_high,[r12,BSRR]               // sets bit 2 only, indicates int0 is active
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
        vstreq  reg_bit1_high,[r12,ODR]                   
        vstrne  reg_zero,[r12,ODR]                    
#endif

enableMONITOR:    
        bic     r1,#3                                   // set PC0 as input pin -> high impedance
        str     r1,[r2, B_TO_C_OFFSET + MODER]
        dsb       
        b       end_EXTI0_IRQHandler

disableMONITOR:
        mov     r0, #_ROM0_MASK
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