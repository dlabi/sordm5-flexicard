#ifndef __MAIN_H
#define __MAIN_H

extern volatile uint8_t *rom_base;
extern volatile uint8_t *high_64k_base;
extern volatile uint8_t *low_64k_base;
extern volatile uint8_t *basic_f;
extern volatile uint8_t *basic_f_end;


#ifdef OTHER_BASICS 
extern volatile uint8_t *basic_i;
extern volatile uint8_t *basic_i_end;
extern volatile uint8_t *basic_g;
extern volatile uint8_t *basic_g_end;
#endif

#ifdef MODDED_SORD
extern volatile uint8_t *msx;
extern volatile uint8_t *msx_end;
#endif

#endif





