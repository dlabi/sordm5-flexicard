#include <stdio.h>
#include <string.h>
#include "main.h"
#include "util.h"
#include "init.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "ff.h"
#include "diskio.h"
#include "ver.h"
#include "protocol.h"
#include "loader.h"


//is defined in the Makefile
#ifndef _BUILD_
  #define _BUILD_ "no_build"
#endif
//used for binary identification
const char *ver = _VER_;
const char *build = _BUILD_;
const char *compiled = "compiled: " __DATE__ "-" __TIME__;

// FATFS stuff
FATFS fs32;
/* Debugger counters: IRQ serviced after the corresponding bus strobe ended. */
volatile uint32_t late_iowr_count, late_iord_count;

GPIO_InitTypeDef  GPIO_InitStructure;

#ifdef ENABLE_SEMIHOSTING
extern void initialise_monitor_handles(void);   /*rtt*/
#endif

#ifdef ENABLE_SWO
extern volatile uint32_t debug_var1;
extern volatile uint32_t debug_var2;

int _write(int32_t file, uint8_t *ptr, int32_t len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}

/*!
 * \brief Sends a character over the SWO channel
 * \param c Character to be sent
 * \param portNo SWO channel number, value in the range of 0 to 31
 */
void SWO_PrintChar(char c, uint8_t portNo) {
  volatile int timeout;
 
  /* Check if Trace Control Register (ITM->TCR at 0xE0000E80) is set */
  if ((ITM->TCR&ITM_TCR_ITMENA_Msk) == 0) { /* check Trace Control Register if ITM trace is enabled*/
    return; /* not enabled? */
  }
  /* Check if the requested channel stimulus port (ITM->TER at 0xE0000E00) is enabled */
  if ((ITM->TER & (1ul<<portNo))==0) { /* check Trace Enable Register if requested port is enabled */
    return; /* requested port not enabled? */
  }
  
  timeout = 5000; // arbitrary timeout value
  while (ITM->PORT[portNo].u32 == 0) {
    // Wait until STIMx is ready, then send data 
    timeout--;
    if (timeout==0) {
      return; // not able to send 
    }
  }
  //ITM->PORT[0].u16 = 0x08 | (c<<8);
  
  ITM->PORT[portNo].u8 = (uint8_t) c;
}

/*!
 * \brief Sends a string over SWO to the host
 * \param s String to send
 * \param portNumber Port number, 0-31, use 0 for normal debug strings
 */
void SWO_PrintString(const char *s, uint8_t portNumber) {
  while (*s!='\0') {
    SWO_PrintChar(*s++, portNumber);
  }
}

/* Formatting through hard-float newlib can destroy the bus's FPU registers. */
static void SWO_PrintHex32(uint32_t value) {
  static const char digits[] = "0123456789ABCDEF";
  for (int shift = 28; shift >= 0; shift -= 4) {
    SWO_PrintChar(digits[(value >> shift) & 15], 0);
  }
}

#endif


//IOWR
void EXTI0_IRQHandler(void) {
  
  uint8_t data;
  
  //__disable_irq();
  // Make sure that interrupt flag is set 
  if ((EXTI->PR & EXTI_Line0) != 0) {
        int address = GPIOE->IDR & 0xff;
        if (address == 0x30) {        
                mem_mode = (GPIOD->IDR >> 8) & 0x0f;

                switch (mem_mode){

                case 1:
                case 3:
                case 4:
                case 7:
                        GPIOC->BSRRL = GPIO_Pin_0;                // set PC0 to high, switch MONITOR ROM off
                        GPIOC->MODER |= GPIO_MODER_MODER0_0;      // PC0 - as output
                        break;
                default:
                        GPIOC->MODER &= ~( GPIO_MODER_MODER0 );   // PC0 - as input, switch MONITOR ROM on 
                        break;
                }
        }
        else 
        if ((address & 0xfe) == 0x80) {
                /* Clear early only for our protocol, preserving its next edge.
                 * Other I/O cycles retain the original acknowledge-at-exit path.
                 */
                EXTI->PR = EXTI_Line0;
                if (GPIOB->IDR & GPIO_Pin_0) {
                    ++late_iowr_count;
                    return;
                }
                data = GPIOD->IDR >> 8;
                if (GPIOB->IDR & GPIO_Pin_0) {
                    ++late_iowr_count;
                    return;
                }
                switch (address) {
                case DATA_PORT: 
                    protocol_write(data);
                    break;
                case CMD_PORT: 
                    protocol_command(data, (const uint8_t *)CCMRAM_BASE);
                    break;
                }                             
                return;
        }
         
    EXTI->PR = EXTI_Line0;
  }
  
  //__enable_irq();
}

//IORD
void EXTI1_IRQHandler(void) {
  
  uint8_t data;

  //__disable_irq();
  // Make sure that interrupt flag is set 
  if ((EXTI->PR & EXTI_Line1) != 0) {
        int address = GPIOE->IDR & 0xff;
        if ((address & 0xfe) == 0x80) {                                         //is it either port 80 or 81?
                EXTI->PR = EXTI_Line1;
                if (GPIOB->IDR & GPIO_Pin_1) {
                    ++late_iord_count;
                    return;
                }
                /* No stream parsing or function call before the Z80 samples data. */
                data = address == DATA_PORT ? main_thread_data : main_thread_command;
                /* MREQ may have preempted us since the first strobe check. */
                if (GPIOB->IDR & GPIO_Pin_1) {
                    ++late_iord_count;
                    return;
                }

              GPIOD->ODR = data << 8;
              GPIOD->MODER = DATA_OUT_MODE;

              while (!(GPIOB->IDR & GPIO_Pin_1) );      // wait for rising edge of IORD and ...
              GPIOD->MODER = DATA_IN_MODE;              // ... tristate bus
              if (address == DATA_PORT) (void)protocol_read();
              return;
      
        }

    EXTI->PR = EXTI_Line1;

  }
  //__enable_irq();
}


void init_fc(void) {

        // You have to disable lazy stacking BEFORE initialising the scratch fpu registers
	enable_fpu_and_disable_lazy_stacking();
	init_fpu_regs();
	rcc_set_frequency(SYSCLK_240_MHZ);

	// switch on compensation cell
	RCC->APB2ENR |= 0 |  RCC_APB2ENR_SYSCFGEN ;
	SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD; // enable compensation cell
	while ((SYSCFG->CMPCR & SYSCFG_CMPCR_READY) == 0);  // wait until ready

	//__disable_irq();
	
	// Enable CCMRAM clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CCMDATARAMEN, ENABLE); 
        config_backup_sram();

	/* PD{8..15}  and PD2 for SD card MOSI*/
	config_gpio_data();
	/* PE{0..15} */
	config_gpio_addr();
        /* PB{11..12} */
	config_gpio_portb();
	/* PC{0..2} and PC8 and PC{10..12} */
	config_gpio_portc();
        config_gpio_dbg();
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 

	SysTick->CTRL  = 0;
	config_PB0_int(); //IOWR
        config_PB1_int(); //IORD
	config_PC4_int(); //MREQ

        SD_NVIC_Configuration(); 

#ifdef ENABLE_SEMIHOSTING
        initialise_monitor_handles();   /*rtt*/
	printf("Semi hosting on\n");
#endif



}

// probably dont need to turn the optimiser off, but it kept on annoying me at the time
int __attribute__((optimize("O0")))  main(void) {

        FRESULT res;
        TCHAR root_directory[15];
        DIR dir;
        uint16_t selected_index;
        uint32_t request_id;

        init_fc();
        
        mem_mode = 0;
	main_thread_data = 0;
        offset1000 = 0;

	//__enable_irq();
        blink_pa1(3);

        memset(&fs32, 0, sizeof(FATFS));
        res = f_mount(&fs32, "",0);
        if (res != FR_OK) {
                blink_debug_led(250);
        }

        strcpy(root_directory, "sordm5");
        res = f_opendir(&dir, root_directory);
        if (res == FR_OK) {
               
                // attempt to load the menu ROM from the root of the SD card
                res = load_rom("menu.rom", high_64k_base + ROM_STAGE_ADDRESS, &file_size);
                if (res == FR_OK) memcpy(rom_base, high_64k_base + ROM_STAGE_ADDRESS, ROM_CAPACITY);
                f_closedir(&dir);
                menu_ctrl_file_count = load_directory(root_directory, (uint8_t *)(CCMRAM_BASE), MENU_MAX_DIRECTORY_ITEMS ); //(uint16_t *)(CCMRAM_BASE)
        }
        res = load_cas("debug.cas", high_64k_base, &cas_header);



#ifdef ENABLE_SWO
	//printf("%d\n", mem_mode);
        SWO_PrintString("hello world with SWO\r\n", 0);
#endif
        while (1)
        {
                request_id = protocol_request_id;
                switch (main_thread_command & 0x7f) {
                case LOAD_FILE:
                    if (!protocol_take_load_index(&selected_index)) break;
                    res = load_selected_file(selected_index, (const uint8_t *)CCMRAM_BASE);
                    last_load_result = res;
#ifdef ENABLE_SWO
                    SWO_PrintString("Load index=0x", 0);
                    SWO_PrintHex32(selected_index);
                    SWO_PrintString(" ", 0);
                    SWO_PrintString(last_load_filename, 0);
                    SWO_PrintString(" result=0x", 0);
                    SWO_PrintHex32((uint32_t)res);
                    SWO_PrintString(" size=0x", 0);
                    SWO_PrintHex32(file_size);
                    SWO_PrintString("\r\n", 0);
#endif
                    if (request_id == protocol_request_id) {
                        main_thread_data = (uint8_t)res;
                        main_thread_command = 0;
                    } else {
                        cancel_load();
                    }
                    break;
                case COMMIT_ROM:
                    res = commit_rom();
                    main_thread_data = (uint8_t)res;
                    main_thread_command = 0;
                    break;
                case DIR_SORD:
                    cancel_load();
                    menu_ctrl_file_count = load_directory(root_directory,
                        (uint8_t *)CCMRAM_BASE, MENU_MAX_DIRECTORY_ITEMS);
                    if (request_id == protocol_request_id) main_thread_command = 0;
                    break;
                default:
                    break;
                }

#ifdef ENABLE_SWO
                if ((mem_mode & 0x10) == 0)
                {
                SWO_PrintString("Mem mode is :\n", 0);
                SWO_PrintChar(mem_mode + 0x30, 0);
                SWO_PrintChar('\n', 0);
                mem_mode |= 0x10;
                }
                /*
                if ((debug_var2 & 0x10000) == 0)
                {
                  printf("Write to %04lx:%02lx\n", debug_var1, debug_var2);
                  debug_var1 = 0x10000; debug_var2 = 0x10000;
                }
                else
                if ((debug_var1 & 0x10000) == 0)
                {
                        printf("Read adr :%04lx\n", debug_var1);
                        debug_var1 = 0x10000;
                }
                */

#endif
                int count = 0;

                while ((GPIOB->IDR & GPIO_Pin_10) == 0)
                        count += 1;

                // longer reset resets mem_mode
                // if (count > 10000000) mem_mode = 0;

                // holding RESET for more than ~3sec will also make STM32 reset
                if (count > 10000000)
                {
#ifdef ENABLE_SWO
              SWO_PrintString("STM32 reset.\n", 0);
#endif
              NVIC_SystemReset();
                }

                if ((mem_mode & 0xf) == 0 || (mem_mode & 0xf) == 2 || (mem_mode & 0xf) == 5 || (mem_mode & 0xf) == 6)
                {
                        GPIOA->BSRRL = 0x02; // LED OFF
                }
                else
                {
                        GPIOA->BSRRH = 0x02; // LED ON
                }
        }
}
