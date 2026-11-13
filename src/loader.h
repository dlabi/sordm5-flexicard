#ifndef FC_LOADER_H
#define FC_LOADER_H

#include "util.h"

extern unsigned int file_size;
extern SORD_HEADER cas_header;
extern volatile uint16_t last_load_index;
extern volatile FRESULT last_load_result;
extern char last_load_filename[DIRECTORY_NAME_SIZE + sizeof("sordm5/")];

FRESULT load_selected_file(uint16_t index, const uint8_t *directory);
FRESULT commit_rom(void);
void cancel_load(void);

#endif
