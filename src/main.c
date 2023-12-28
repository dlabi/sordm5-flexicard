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

// FATFS stuff
FATFS fs32;

unsigned int counter, file_size;
uint16_t file_num;
SORD_HEADER cas_header;
uint8_t *p_cas_flag = (((uint8_t*)&rom_base)+0xb);
uint16_t *p_cas_start = (((uint8_t*)&rom_base)+0x9);
uint16_t *p_cas_head = (((uint8_t*)&rom_base)+0xc);
uint16_t *p_cas_size = (((uint8_t*)&rom_base)+0xe);

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

#endif


//IOWR
void EXTI0_IRQHandler(void) {
  
  uint8_t data;
  
  //__disable_irq();
  // Make sure that interrupt flag is set 
  if ((EXTI->PR & EXTI_Line0) != 0) {
        int address = GPIOE->IDR & 0xff;
        if (address == 0x30) {        
                mem_mode = ( GPIOD->IDR >> 8 ) & 0x0f;

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
                data = GPIOD->IDR >> 8;
                switch (address) {
                case DATA_PORT: 
                           switch(main_thread_command & 0x7f){
                                case SET_INDEX:
                                case LOAD_FILE:
                                        if (main_thread_command & 0x80) {
                                        counter--;
                                        }
                                        break;
                           }
                           main_thread_data = data;
                           break;
                case CMD_PORT: 
                           switch(data){
                                case BREAK:
                                        main_thread_data = 0;
                                        data = 0;
                                        break;
                                case FIRST_FILE:
                                        file_num = 0;
                                        data = 0;               //main_thread_command = 0
                                        main_thread_data = file_num;
                                        break;
                                case NEXT_FILE: 
                                        file_num++;             //TODO: 2 byte value
                                        data = 0;               //main_thread_command = 0
                                        main_thread_data = file_num;
                                        break;
                                case PREV_FILE: 
                                        file_num--;
                                        data = 0;
                                        main_thread_data = file_num;
                                        break;
                                case RESET_SORD:
                                        reset_sord(100);
                                        break;
                                case OFFSET_RAM_ON:
                                        offset1000 = 1;
                                        data = 0;               //main_thread_command = 0
                                        break;
                                case OFFSET_RAM_OFF:
                                        offset1000 = 0;
                                        data = 0;               //main_thread_command = 0
                                        break;

                           }
                           main_thread_command = data;
                           break;
                }                             
        }
         
    // Clear interrupt flag 
    EXTI->PR = EXTI_Line0;
  }
  
  //__enable_irq();
}

//IORD
void EXTI1_IRQHandler(void) {
  
  uint8_t data;
  int cmd_active;

  //__disable_irq();
  // Make sure that interrupt flag is set 
  if ((EXTI->PR & EXTI_Line1) != 0) {
        int address = GPIOE->IDR & 0xff;
        if ((address & 0xfe) == 0x80) {                                         //is it either port 80 or 81?
                cmd_active = main_thread_command >> 7;       
                switch(address){
                        case (DATA_PORT):                                       //data port reading
                                switch (main_thread_command & 0x7f){            //ignore status bit
                                    case GET_COUNT:
                                                if (cmd_active) {               //still active?                                             
                                                        data = main_thread_data;
                                                        counter--;
                                                }
                                                else {
                                                        data = 0xff;            //shouldn't get here        
                                                }                
                                                break;

                                    case GET_FILENAME:
                                                if (cmd_active) {               //still active?                                              
                                                        data = main_thread_data;
                                                        counter++;
                                                        #ifdef ENABLE_SWO
                                                        SWO_PrintString("Get_filename\r\n", 0);
                                                        #endif
                                                }
                                                else {
                                                        data = 0xff;            //shouldn't get here        
                                                }                
                                                break;
                                    default:    data = main_thread_data;

                                }
                                
                break;
                case (CMD_PORT): data = main_thread_command;        //STM32 cmd port
                break;

        }       
              GPIOD->ODR = data << 8;
              GPIOD->MODER = DATA_OUT_MODE;

              while (!(GPIOB->IDR & GPIO_Pin_1) );      // wait for rising edge of IORD and ...
              GPIOD->MODER = DATA_IN_MODE;              // ... tristate bus
      
        }

    // Clear interrupt flag 
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
        TCHAR sord_folder[] = "sordm5/";
        TCHAR full_filename[128];
        TCHAR root_directory[15];
        DIR dir;
        int cmd_active;

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
                res = load_rom("menu.rom",(unsigned char *) &rom_base, &file_size);
                if (res != FR_OK) {
                //blink_debug_led(500);
                }
                menu_ctrl_file_count = load_directory(root_directory, (uint8_t *)(CCMRAM_BASE), MENU_MAX_DIRECTORY_ITEMS ); //(uint16_t *)(CCMRAM_BASE)
        }
        res = load_cas("debug.cas", (unsigned char *) &high_64k_base, (SORD_HEADER *)&cas_header);


char buff[128];
int length;

#ifdef ENABLE_SWO
	//printf("%d\n", mem_mode);
        SWO_PrintString("hello world with SWO\r\n", 0);
#endif
        while (1)
        {
                cmd_active = main_thread_command >> 7;
                switch (main_thread_command & 0x7f)
                {
                // returns num of files
                case GET_COUNT:
              if (!cmd_active)
              {
                main_thread_command |= 0x80; //set active status high
                counter = 1;
              }
              else
              {
                if (counter < 0)
                        main_thread_command = 0; //we sent both bytes
                else
                        main_thread_data = (uint8_t)(menu_ctrl_file_count >> (8 * counter));
              }
              break;
                // set file index
                case SET_INDEX:
              if (!cmd_active)
              {
                main_thread_command |= 0x80; //set active status high
                counter = 2;
                file_num = 0;
                main_thread_data = 0;  
              }
              else
              {
                file_num |= main_thread_data << (8 * counter);
                if (counter == 0)
                        main_thread_command = 0; // we received both bytes
              }
              break;
                // return file name at index
                case GET_FILENAME:
              if (!cmd_active)
              {
                if (file_num > menu_ctrl_file_count)
                {
                        main_thread_command = 0;
                        break;
                }
                main_thread_command |= 0x80; //set active status high
                length = get_filename((uint8_t *)(CCMRAM_BASE), buff, file_num);
                counter = 0;
                break;
              }
              else
              {
                if (counter < length + 1)
                        main_thread_data = buff[counter];
                else
                        main_thread_command = 0;
              }
              break;
                case LOAD_FILE:
              if (!cmd_active)
              {
                main_thread_command |= 0x80; //set active status high
                counter = 2;
                file_num = 0;
                main_thread_data = 0;
              }
              else
              {
                file_num |= main_thread_data << (8 * counter);
                if (counter == 0)
                {

                        length = get_filename((uint8_t *)(CCMRAM_BASE), buff, file_num);
                        strcpy(full_filename, sord_folder);
                        strcat(full_filename, buff);
                        if (suffix_match(full_filename, ".rom") || suffix_match(full_filename, ".bin"))
                        {
                                /* I was thinking setting wait state on the bus is good idea how to stop z80 for a while
                                but it seems it crashes Sord. Need to be investigated more */
                                //GPIOC->BSRRH = GPIO_Pin_5; // log. 0 to WAIT
                                res = load_rom(full_filename, (unsigned char *)&rom_base, &file_size); // high_64k_base+0x9000
                                //GPIOC->BSRRL = GPIO_Pin_5;       // log. 1 to WAIT
                        }
                        else if (suffix_match(full_filename, ".cas"))
                        {
                                res = load_cas(full_filename, (unsigned char *)&high_64k_base, (SORD_HEADER *)&cas_header);

                                switch (cas_header.Attr) {
                                        case 1:
                                        case 3:  //memcpy( (unsigned char *) &rom_base+9, &cas_header.Start, 2);
                                                 *p_cas_start = cas_header.Start;
                                                 *p_cas_flag = 2;
                                                 break;
                                        #ifdef OTHER_BASICS         
                                        case 0x20: //BI
                                                 memcpy((unsigned char *) &high_64k_base + 0x2000,&basic_i, (char *)&basic_i_end - (char *)&basic_i);
                                                 *p_cas_flag = 3;
                                                 offset1000 = 1; // 7000h->8000h
                                                 break;
                                        case 0x60: //BG    
                                                 memcpy((unsigned char *) &high_64k_base + 0x2000,&basic_g, (char *)&basic_g_end - (char *)&basic_g); 
                                                 *p_cas_flag = 3;
                                                 offset1000 = 1; // 7000h->8000h
                                                 break;
                                        case 0x80: //BF
                                        case 0x82:    
                                                 memcpy((unsigned char *) &high_64k_base + 0x2000,&basic_f, (char *)&basic_f_end - (char *)&basic_f);
                                                 *p_cas_flag = 3;
                                                 offset1000 = 1; // 7000h->8000h 
                                                 break; 
                                        #endif                                             
                                }
                                 *p_cas_head = cas_header.Head;
                                 *p_cas_size = cas_header.Size;
                                 
                        }
                        #ifdef MODDED_SORD
                        else if (suffix_match(full_filename, ".msx"))
                        {
                                //GPIOC->BSRRH = GPIO_Pin_5; // log. 0 to WAIT
                                res = load_cas(full_filename, (unsigned char *)&high_64k_base,   (SORD_HEADER *)&cas_header);
                                
                                if (res == FR_OK)
                                {
                                        memcpy(&high_64k_base,&msx, 0x8000);
                                        *p_cas_start = cas_header.Start;
                                        *p_cas_flag = 1; //MSX
                                        offset1000 = 1; // 7000h->8000h
                                        main_thread_data = 0;
                                }
                                else
                                        main_thread_data = 0xff; // error status during file read
                        }
                        #endif
                        if (res == FR_OK)
                        {
                                main_thread_data = 0;
                        }
                        else
                                main_thread_data = 0xff; // error status during file read
                        main_thread_command = 0;         // we received both bytes and rom loaded
                        //GPIOC->BSRRL = GPIO_Pin_5;       // log. 1 to WAIT
                }
              }
              break;
                case DIR_SORD:
              if (!cmd_active)
              {
                main_thread_command |= 0x80; //set active status high
                res = f_opendir(&dir, root_directory);

                if (res == FR_OK)
                {
                        menu_ctrl_file_count = load_directory(root_directory, (uint8_t *)(CCMRAM_BASE), MENU_MAX_DIRECTORY_ITEMS);
                }
                else
                        menu_ctrl_file_count = 0;
              }
              else
              {
                main_thread_command = 0;
              }
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
