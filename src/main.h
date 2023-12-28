#ifndef __MAIN_H
#define __MAIN_H

extern volatile uint8_t *rom_base;
extern volatile uint8_t *high_64k_base;
extern volatile uint8_t *low_64k_base;
extern volatile uint8_t *basic_f;
extern volatile uint8_t *basic_f_end;
extern volatile uint8_t main_thread_command;
extern volatile uint8_t main_thread_data;
extern volatile uint8_t mem_mode;
extern volatile uint8_t offset1000;
extern volatile uint8_t rst5;
extern volatile uint32_t menu_ctrl_file_count;


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


#if _USE_LFN
    static char lfn[_MAX_LFN + 1];
        fno.lfname = lfn;
            fno.lfsize = sizeof lfn;
#endif


#endif





