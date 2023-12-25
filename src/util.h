#ifndef __UTIL_H
#define __UTIL_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"

#include "ff.h"
#include "main.h"

//#include "defines.h"

#define GPIO_DEBUG_LED		GPIO_Pin_1
// true/false
#define TRUE 1
#define FALSE 0

typedef char			CHAR;
typedef unsigned char	UCHAR;
typedef unsigned char	BYTE;

typedef struct
{	
  unsigned char Attr;
  char  Name[9];
  unsigned short Head;
  unsigned short Size;
  unsigned short Start;
  char  E_attr;
  char  Blank[14];
  unsigned char	Checksum;
} SORD_HEADER;


void delay_us(const uint16_t us);
void delay_ms(const uint16_t ms);
void blink_debug_led(int delay);
void blink_pa1(int n);

uint32_t suffix_match(char *name, char *suffix);
FRESULT load_rom(char *fname, unsigned char* buffer, unsigned int* file_size );
FRESULT load_binary(char *fname, unsigned char* buffer, int size);
FRESULT load_cas(char *fname, unsigned char* buffer, SORD_HEADER* cas_header);
// void load_rom_and_grom_and_disk_name(char *app_directory, unsigned char*rom_buffer, unsigned char *grom_buffer, DSK *dsk);

uint32_t load_directory(char *dirname, uint8_t *index_buffer, uint32_t max_files);
uint8_t get_filename(uint8_t *index_buffer, char *buffer, int file_num );
void send_files(uint32_t file_number, uint16_t *index_buffer );
void reset_sord(int delay);
void my_memcpy(unsigned char *dest, unsigned char *from, int length);

#endif
