
#ifndef __INIT_H
#define __INIT_H

extern void init_fpu_regs(void);

enum sysclk_freq {
    SYSCLK_42_MHZ=0,
    SYSCLK_84_MHZ,
    SYSCLK_168_MHZ,
    SYSCLK_200_MHZ,
    SYSCLK_240_MHZ,
};

void enable_fpu_and_disable_lazy_stacking(void);
void rcc_set_frequency(enum sysclk_freq);
void config_backup_sram(void);
void config_gpio_data(void);
void config_gpio_addr(void);
void config_gpio_portb(void);
void config_gpio_portc(void);
void config_gpio_dbg(void);
void config_PB0_int(void);
void config_PB1_int(void);
void config_PC4_int(void);
void SD_NVIC_Configuration(void);


#endif