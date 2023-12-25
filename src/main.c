#include <stdio.h>
#include <string.h>
#include "main.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "util.h"

//#include "defines.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "ff.h"
#include "diskio.h"

#define	DATA_OUT_MODE	0x55550020
#define	DATA_IN_MODE	0x00000020
#define MENU_MAX_DIRECTORY_ITEMS	500 //1024 jinak musim zmensit max delku jmena souboru ze 128 na 64
#define CCMRAM_BASE	0x10000000

// FLEXI CARD PORTS
#define DATA_PORT       0x80
#define CMD_PORT        0x81

// FLEXI CARD COMMANDS
#define BREAK           0
#define GET_COUNT       1
#define SET_INDEX       2
#define GET_INDEX       3
#define NEXT_FILE       4
#define PREV_FILE       5
#define FIRST_FILE      6
#define GET_FILENAME    7
#define LOAD_FILE       8
#define RESET_SORD      9
#define DIR_SORD        10
#define OFFSET_RAM_ON   11
#define OFFSET_RAM_OFF  12

// FATFS stuff
FATFS fs32;

#if _USE_LFN
    static char lfn[_MAX_LFN + 1];
        fno.lfname = lfn;
            fno.lfsize = sizeof lfn;
#endif

GPIO_InitTypeDef  GPIO_InitStructure;


extern void init_fpu_regs(void);


// Must be volatile to prevent optimiser doing stuff
extern volatile BYTE main_thread_command;
extern volatile BYTE main_thread_data;
extern volatile BYTE mem_mode;
extern volatile BYTE offset1000;
extern volatile BYTE rst5;
extern volatile uint32_t menu_ctrl_file_count;

unsigned int counter, file_size;
uint16_t file_num;
SORD_HEADER cas_header;
BYTE *p_cas_flag = (((BYTE*)&rom_base)+0xb);
uint16_t *p_cas_start = (((BYTE*)&rom_base)+0x9);
uint16_t *p_cas_head = (((BYTE*)&rom_base)+0xc);
uint16_t *p_cas_size = (((BYTE*)&rom_base)+0xe);



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


// Enable the FPU (Cortex-M4 - STM32F4xx and higher)
// http://infocenter.arm.com/help/topic/com.arm.doc.dui0553a/BEHBJHIG.html
// Also make sure lazy stacking is disabled
void enable_fpu_and_disable_lazy_stacking() {
  __asm volatile
  (
    "  ldr.w r0, =0xE000ED88    \n"  /* The FPU enable bits are in the CPACR. */
    "  ldr r1, [r0]             \n"  /* read CAPCR */
    "  orr r1, r1, #( 0xf << 20 )\n" /* Set bits 20-23 to enable CP10 and CP11 coprocessors */
    "  str r1, [r0]              \n" /* Write back the modified value to the CPACR */
    "  dsb                       \n" /* wait for store to complete */
    "  isb                       \n" /* reset pipeline now the FPU is enabled */
    // Disable lazy stacking (the default) and effectively have no stacking since we're not really using the FPU for anything other than a fast register store
    "  ldr.w r0, =0xE000EF34    \n"  /* The FPU FPCCR. */
    "  ldr r1, [r0]             \n"  /* read FPCCR */
    "  bfc r1, #30,#2\n" /* Clear bits 30-31. ASPEN and LSPEN. This disables lazy stacking */
    "  str r1, [r0]              \n" /* Write back the modified value to the FPCCR */
    "  dsb                       \n" /* wait for store to complete */
    "  isb"                          /* reset pipeline  */
    :::"r0","r1"
    );
}

enum sysclk_freq {
    SYSCLK_42_MHZ=0,
    SYSCLK_84_MHZ,
    SYSCLK_168_MHZ,
    SYSCLK_200_MHZ,
    SYSCLK_240_MHZ,
};
 
void rcc_set_frequency(enum sysclk_freq freq)
{
    int freqs[]   = {42, 84, 168, 200, 240};
 
    /* USB freqs: 42MHz, 42Mhz, 48MHz, 50MHz, 48MHz */
    int pll_div[] = {2, 4, 7, 10, 10}; 
 
    /* PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N */
    /* SYSCLK = PLL_VCO / PLL_P */
    /* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
    uint32_t PLL_P = 2;
    uint32_t PLL_N = freqs[freq] * 2;
    uint32_t PLL_M = (HSE_VALUE/1000000);
    uint32_t PLL_Q = pll_div[freq];
 
    RCC_DeInit();
 
    /* Enable HSE osscilator */
    RCC_HSEConfig(RCC_HSE_ON);
 
    if (RCC_WaitForHSEStartUp() == ERROR) {
        return;
    }
 
    /* Configure PLL clock M, N, P, and Q dividers */
    RCC_PLLConfig(RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q);
 
    /* Enable PLL clock */
    RCC_PLLCmd(ENABLE);
 
    /* Wait until PLL clock is stable */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
 
    /* Set PLL_CLK as system clock source SYSCLK */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
 
    /* Set AHB clock divider */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
 
    //FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

    /* Set APBx clock dividers */
    switch (freq) {
        /* Max freq APB1: 42MHz APB2: 84MHz */
        case SYSCLK_42_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div1); /* 42MHz */
            RCC_PCLK2Config(RCC_HCLK_Div1); /* 42MHz */
            break;
        case SYSCLK_84_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div2); /* 42MHz */
            RCC_PCLK2Config(RCC_HCLK_Div1); /* 84MHz */
            break;
        case SYSCLK_168_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div4); /* 42MHz */
            RCC_PCLK2Config(RCC_HCLK_Div2); /* 84MHz */
            break;
        case SYSCLK_200_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div4); /* 50MHz */
            RCC_PCLK2Config(RCC_HCLK_Div2); /* 100MHz */
            break;
        case SYSCLK_240_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div4); /* 60MHz */
            RCC_PCLK2Config(RCC_HCLK_Div2); /* 120MHz */
            break;
    }
 
    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();
}

void SD_NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure;

        /* Configure the NVIC Preemption Priority Bits */
        //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
        //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

        NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
        //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;    // This must be a lower priority (ie. higher number) than the _MREQ and _IORQ interrupts
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        // DMA2 STREAMx Interrupt ENABLE
	NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_Init(&NVIC_InitStructure);

}

void SDIO_IRQHandler(void)
{
	/* Process All SDIO Interrupt Sources */
	SD_ProcessIRQSrc();
}

void SD_SDIO_DMA_IRQHANDLER(void)
{
	SD_ProcessDMAIRQ();
}

// EXTI0_IRQn 	EXTI0_IRQHandler
// https://stm32f4-discovery.net/2014/08/stm32f4-external-interrupts-tutorial/
// _IOWR interrupt
void config_PB0_int(void) {
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;

        /* Enable clock for SYSCFG */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Tell system that you will use PB0 for EXTI_Line0 */
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

        /* PB0 is connected to EXTI_Line0 */
        EXTI_InitStruct.EXTI_Line = EXTI_Line0;
        /* Enable interrupt */
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        /* Interrupt mode */
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        /* Triggers on rising and falling edge */
        //EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        /* Add to EXTI */
        EXTI_Init(&EXTI_InitStruct);

        /* Add IRQ vector to NVIC */
        /* PB0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
        NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
        /* Set priority */
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
        /* Set sub priority */
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
        /* Enable interrupt */
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        /* Add to NVIC */
        NVIC_Init(&NVIC_InitStruct);
}

// EXTI1_IRQn 	EXTI1_IRQHandler
// https://stm32f4-discovery.net/2014/08/stm32f4-external-interrupts-tutorial/
// _IORD interrupt
void config_PB1_int(void) {
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;

        /* Enable clock for SYSCFG */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Tell system that you will use PB1 for EXTI_Line1 */
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

        /* PB1 is connected to EXTI_Line1 */
        EXTI_InitStruct.EXTI_Line = EXTI_Line1;
        /* Enable interrupt */
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        /* Interrupt mode */
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        /* Triggers on rising and falling edge */
        //EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        /* Add to EXTI */
        EXTI_Init(&EXTI_InitStruct);

        /* Add IRQ vector to NVIC */
        /* PB1 is connected to EXTI_Line1, which has EXTI1_IRQn vector */
        NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
        /* Set priority */
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
        /* Set sub priority */
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
        /* Enable interrupt */
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        /* Add to NVIC */
        NVIC_Init(&NVIC_InitStruct);
}


//MREQ INT
void config_PC4_int(void) {
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;

        /* Enable clock for SYSCFG */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);

        /* PC4 is connected to EXTI_Line4 */
        EXTI_InitStruct.EXTI_Line = EXTI_Line4;
        /* Enable interrupt */
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        /* Interrupt mode */
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        /* Triggers on rising and falling edge */
        //EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        /* Add to EXTI */
        EXTI_Init(&EXTI_InitStruct);

        /* Add IRQ vector to NVIC */
        /* PC4 is connected to EXTI_Line4, which has EXTI4_IRQn vector */
        NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
        /* Set priority */
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
        /* Set sub priority */
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
        /* Enable interrupt */
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        /* Add to NVIC */
        NVIC_Init(&NVIC_InitStruct);
}

/*  RST -> PB10 */
void config_gpio_portb(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOB Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* Input Signals GPIO pins on MRQ -> PC4 */
/* Output Signals GPIO pins on ROM0 -> PC0, ROMDS -> PC13, need to make it open collector with a pullup, it disables MONITOR ROM */
void config_gpio_portc(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOC Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	//input pins
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


        //input/output pin
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //GPIO_OType_OD
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_DOWN; //GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
        //ROMDS
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //GPIO_OType_OD
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_DOWN; //GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIOC->BSRRH = GPIO_Pin_13; // log. 0 to ROMDS, blocks ROM inside Sord

        //WAIT
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_DOWN; //GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIOC->BSRRL = GPIO_Pin_5; // log. 1 to WAIT
}

/* Input/Output data GPIO pins on PD{8..15}. Also PD2 is used fo MOSI on the STM32F407VET6 board I have */
void config_gpio_data(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | 
		GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

}

/* Input Address GPIO pins on PE{0..15} */
void config_gpio_addr(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOE Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/* Debug GPIO pins on PA0-PA3, PA1 LED */
void config_gpio_dbg(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,DISABLE);


	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}


void config_backup_sram(void) {

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
        PWR_BackupAccessCmd(ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
        PWR_BackupRegulatorCmd(ENABLE);
}

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
  
  BYTE data;
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



// probably dont need to turn the optimiser off, but it kept on annoying me at the time
int __attribute__((optimize("O0")))  main(void) {

        FRESULT res;
        TCHAR sord_folder[] = "sordm5/";
        TCHAR full_filename[128];
        TCHAR root_directory[15];
        DIR dir;
        int cmd_active;

        // You have to disable lazy stacking BEFORE initialising the scratch fpu registers
	enable_fpu_and_disable_lazy_stacking();
	init_fpu_regs();

	main_thread_data = 0;
        offset1000 = 0;

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

        mem_mode = 0;

        SD_NVIC_Configuration(); 

#ifdef ENABLE_SEMIHOSTING
        initialise_monitor_handles();   /*rtt*/
	printf("Semi hosting on\n");
#endif

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
