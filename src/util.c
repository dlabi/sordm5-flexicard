#include "util.h"

// probably have to tweak these if the processor speed changes. Just have to be rough
void delay_us(const uint16_t us)
{
   uint32_t i = us * 60;
   while (i-- > 0) {
      __asm volatile ("nop");
   }
}

void delay_ms(const uint16_t ms)
{
   //uint32_t i = ms * 27778;
   uint32_t i = ms * 30000 *2;
   while (i-- > 0) {
      __asm volatile ("nop");
   }
}

void blink_pa1(int n) {
        int i=0;
        while(n) {
                GPIOA->ODR = 0x0002+(i++ % 3)+1;
                delay_ms(50);
                GPIOA->ODR = 0x0000+(i++ % 3)+1; 
                delay_ms(50);
                n -= 1;
        }
        GPIOA->ODR = 0x0002;
}

void reset_sord(int delay) {
		return;
		/*
		takhle to nejde protoze na rst pin je zapojen vystup inventoru a vznikal by hazard.
		je treba udelat to tak ze pri cteni kontrolovane oblasti pameti budeme posilat instrukci rst0 dokud nedojde k resetu
		GPIOB->BSRRH = GPIO_Pin_10;                // ground PB10 -> reset
		GPIOB->MODER |= GPIO_MODER_MODER10_0;      // PB10 - as output
		delay_ms(delay);
		GPIOB->MODER &= ~(GPIO_MODER_MODER10);     // PB10 - as input     
		*/
}

void blink_debug_led(int delay) {
        while(1) {
                GPIOA->ODR |= GPIO_DEBUG_LED;
                delay_ms(delay);
                GPIOA->ODR &= ~(GPIO_DEBUG_LED);
                delay_ms(delay);
        }
}

uint32_t suffix_match(char *name, char *suffix) {
	if (strlen(name)>strlen(suffix)) {
		if (strncasecmp (&name[strlen(name)-strlen(suffix)],suffix,strlen(suffix)) == 0) {
			return TRUE;
		}
	}
	return FALSE;
}
/*
// Find the rom and grom and dsk files in dir and load them into the buffers provided. If a dsk is found, we just return its name in *dsk_name
void load_rom_and_grom_and_disk_name(char *app_directory, unsigned char*rom_buffer, unsigned char *grom_buffer, DSK *dsk) {
	DIR dir;
        FIL     fil;
	FILINFO fno;
	TCHAR fname[128];
	FRESULT res;
        UINT BytesRead;
	UINT c_size=0;
	UINT d_size=0;
	UINT g_size=0;
	UINT other_size=0;

	// Assume there are no disks
	//dsk_names[0][0]=0;		//DSK1
	//dsk_names[1][0]=0;		//DSK2
	//dsk_names[2][0]=0;		//DSK2

	//dsk[0].id=1;
	//dsk[1].id=2;
	//dsk[2].id=3;

	dsk[0].disk_filename[0]=0;
	dsk[1].disk_filename[0]=0;
	dsk[2].disk_filename[0]=0;

	// zero the rom and grom memory before we load anything
	memset(rom_buffer,0,0x8000);
	memset(grom_buffer,0,0xA000);

	res = f_opendir(&dir, app_directory);
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno);
			if (res != FR_OK || fno.fname[0] == 0) break;
			if (suffix_match(fno.fname,"c.bin")) {
				strcpy(fname,app_directory);
				strcat(fname,"/");
				strcat(fname,fno.fname);
				res =  f_open(&fil, fname, FA_READ);
				if (res == FR_OK) {
					// try to read 32K. ROMS that end in c.bin are usually 8K, but if they arent a C and D set, then its possible the
					// ROMS name simply ends in a C. So if its a non C and D set then we could load up to 32K here. If it is a C and
					// D set then the rom is probably 8K and we'll only read 8K
					res = f_read(&fil,rom_buffer,0x8000,&BytesRead);
					f_close(&fil);
					c_size = BytesRead;
					//if (BytesRead == 0x2000) {
					//	memcpy(&rom_buffer[0x4000],rom_buffer,0x2000);
					//}
				}
			} else if (suffix_match(fno.fname,"d.bin")) {
				strcpy(fname,app_directory);
				strcat(fname,"/");
				strcat(fname,fno.fname);
				res =  f_open(&fil, fname, FA_READ);
				if (res == FR_OK) {
					// TODO. This wont work if you just happen to have a game that ends in D (eg. stupid.bin)
					// Load the D ROM into the 2nd 8K block, but we'll allow it to be bigger, up to 24K
					res = f_read(&fil,&rom_buffer[0x2000],0x6000,&BytesRead);
					f_close(&fil);
					d_size = BytesRead;
					//if (BytesRead == 0x2000) {
					//	memcpy(&rom_buffer[0x6000],&rom_buffer[0x2000],0x2000);
					//}

				}
			} else if (suffix_match(fno.fname,"g.bin")) {
				strcpy(fname,app_directory);
				strcat(fname,"/");
				strcat(fname,fno.fname);
				res =  f_open(&fil, fname, FA_READ);
				if (res == FR_OK) {
					res = f_read(&fil,grom_buffer,0xA000,&BytesRead);
					f_close(&fil);
					g_size = BytesRead;
				}
			} else if ( (suffix_match(fno.fname,".dsk")) || (suffix_match(fno.fname,".dsk1"))) {
				strcpy(dsk[0].disk_filename, app_directory);
				strcat(dsk[0].disk_filename, "/");
				strcat(dsk[0].disk_filename, fno.fname);
			} else if (suffix_match(fno.fname,".dsk2")) {
				strcpy(dsk[1].disk_filename, app_directory);
				strcat(dsk[1].disk_filename, "/");
				strcat(dsk[1].disk_filename, fno.fname);
			} else if (suffix_match(fno.fname,".dsk3")) {
				strcpy(dsk[2].disk_filename, app_directory);
				strcat(dsk[2].disk_filename, "/");
				strcat(dsk[2].disk_filename, fno.fname);
			} else {
				// Assume its a normal ROM up to 32K in size
				//
				strcpy(fname,app_directory);
				strcat(fname,"/");
				strcat(fname,fno.fname);
				res =  f_open(&fil, fname, FA_READ);
				if (res == FR_OK) {
					res = f_read(&fil,rom_buffer,0x8000,&BytesRead);
					f_close(&fil);
					other_size = BytesRead;
				}

			}

		}
		f_closedir(&dir);
		// Sort out replication
		if (other_size == 0) {
			if (d_size == 0) {
				if (c_size == 0x2000) {
					for (int i=0;i<0x2000;i++) {
						rom_buffer[0x2000+i] = rom_buffer[i];
						rom_buffer[0x4000+i] = rom_buffer[i];
						rom_buffer[0x6000+i] = rom_buffer[i];
					}
				}
			} else if (d_size == 0x2000) {
				if (c_size == 0x2000) {
					for (int j=0;j<0x2000;j++) {
						rom_buffer[0x4000+j] = rom_buffer[j];
						rom_buffer[0x6000+j] = rom_buffer[0x2000+j];
					}
				}
			}
		}

	}

}

*/


void my_memcpy(unsigned char *dest, unsigned char *from, int length) {
	unsigned char *q = dest;
	unsigned char *p = from;

	while (p < from + length) {
		*q++ = *p++;
	}

}

// Load fname rom file into a buffer of size max_size. If blank_remaining=1 then fill up to max_size with zeros
FRESULT load_rom(char *fname, unsigned char* buffer, unsigned int* file_size) {
	FRESULT res;
    FIL     fil;
    UINT BytesRead;

	memset(&fil, 0, sizeof(FIL));
	(*file_size)=0;

	res =  f_open(&fil, fname, FA_READ);

	if (res == FR_OK) {
/*
00 - 8K ROM OD ADRESY 2000H
01 - 8K ROM OD ADRESY 4000H
02 - 16K ROM OD ADRESY 2000H
*/
		res = f_read(&fil,buffer, 0x5000, &BytesRead);
		if (res == FR_OK) {
			if(buffer[0] == 1) {
				memcpy(&buffer + 0x2000, &buffer, BytesRead);
				memset(&buffer, 0xff, 0x2000);			 
			}
			(*file_size)=BytesRead;
		}		
	}
	//else blink_debug_led(3000);

	f_close(&fil);
	return res;
}

FRESULT load_binary(char *fname, unsigned char* buffer, int size) {
	FRESULT res;
    FIL     fil;
    UINT BytesRead;

	memset(&fil, 0, sizeof(FIL));

	res =  f_open(&fil, fname, FA_READ);

	if (res == FR_OK) {

		res = f_read(&fil,buffer, size, &BytesRead);
		
	}
	f_close(&fil);
	return res;
}

FRESULT load_cas(char *fname, unsigned char* buffer) {
	FRESULT res;
    FIL     fil;
    UINT BytesRead, length, offset;
	SORD_HEADER sord_header;
	unsigned char byte[1];
	char buff[0x101];
	char tmp[1];

	memset(&fil, 0, sizeof(FIL));
	res =  f_open(&fil, fname, FA_READ);

	if (res == FR_OK) 
	{

		res = f_read(&fil,buff, 7, &BytesRead);
		if (strcmp(buff,"SORDM5")!=0) {f_close(&fil); return -1;}
		f_lseek(&fil,16);
		res = f_read(&fil,byte,1, &BytesRead); //read header type
		if (res != FR_OK) {f_close(&fil); return -1;}

		while (!f_eof(&fil) && res !=-1)
		{
			if (*byte!='H') {res = -1; break;}

			res = f_read(&fil,tmp,1, &BytesRead);
			length = (UINT)*tmp + 1;
			res |= f_read(&fil, &sord_header,sizeof(sord_header), &BytesRead);
			if (BytesRead != length  || res != FR_OK) {res = -1; break;}

			offset = 0;
			while (*(f_gets((TCHAR *)byte,2,&fil))!='H' && !f_eof(&fil) )
			{
				res = f_read(&fil,tmp,1, &BytesRead);
				length=(UINT)*tmp;
				if (length==0) length=256;
				res |= f_read(&fil,buff,length, &BytesRead);
				if (BytesRead != length || res != FR_OK) {res = -1; break;}

				//zahod crc
				res = f_read(&fil,tmp,1, &BytesRead);
				if (res != FR_OK) {res = -1; break;}
				if (*byte=='D' || *byte=='F')
				{
					memcpy(&buffer[sord_header.Head + offset], &buff, length);
					offset += length;
				}
			}
		}
	}
	f_close(&fil);
	return res;
}

uint32_t load_directory(char *dirname, uint8_t *index_buffer, uint32_t max_files) {
	FRESULT res;
        DIR dir;
        static FILINFO fno;
	uint32_t file_index;
	int i;
	//unsigned char *buffer;

	// make the string buffer start straight after the array
	//buffer = (unsigned char *) &index_buffer[max_files];
	memset(index_buffer, 0, 128 * max_files);
	memset(&dir, 0, sizeof(DIR));
        res = f_opendir(&dir, (TCHAR *) dirname);
        if (res != FR_OK) {
                blink_debug_led(2000);
        }

	file_index=0;
	
	while (file_index<max_files) {
		
		res = f_readdir(&dir, &fno);
		if (res != FR_OK || fno.fname[0] == 0) {
			break;
		}
		i=0;
		do {
			index_buffer[file_index * 128 + i] = fno.fname[i];
			if (i>126) {
				index_buffer[file_index * 128 + i]=0;
				break;
			}
		} while (fno.fname[i++]!=0);
		file_index++;
	}

	res = f_closedir(&dir);
	return file_index;
}

uint8_t get_filename(uint8_t *index_buffer, char *buffer, int file_num ) {
	
	int i=0;
	do {
			buffer[i]=index_buffer[file_num * 128 + i];
		} while (buffer[i++]!=0);	
	
	return strlen((char *)buffer);
}

void send_files(uint32_t file_number, uint16_t *index_buffer ) {
	
}
