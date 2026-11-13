#include "loader.h"

unsigned int file_size;
static int rom_pending;
SORD_HEADER cas_header;
volatile uint16_t last_load_index;
volatile FRESULT last_load_result;
char last_load_filename[DIRECTORY_NAME_SIZE + sizeof("sordm5/")];

static void launch_info(uint8_t flag, uint16_t head, uint16_t size, uint16_t start)
{
    rom_base[9] = (uint8_t)start;
    rom_base[10] = (uint8_t)(start >> 8);
    rom_base[11] = flag;
    rom_base[12] = (uint8_t)head;
    rom_base[13] = (uint8_t)(head >> 8);
    rom_base[14] = (uint8_t)size;
    rom_base[15] = (uint8_t)(size >> 8);
}

FRESULT load_selected_file(uint16_t index, const uint8_t *directory)
{
    char name[DIRECTORY_NAME_SIZE];
    FRESULT result;
    uint32_t end;
    uint8_t name_length;
    static const char folder[] = "sordm5/";

    last_load_index = index;
    last_load_filename[0] = 0;
    file_size = 0;
    rom_pending = 0;
    offset1000 = 0;
    launch_info(0, 0, 0, 0);
    name_length = get_filename(directory, name, index, menu_ctrl_file_count);
    if (!name_length || sizeof(folder) + name_length > sizeof(last_load_filename))
        return FR_INVALID_PARAMETER;
    /* Do not use printf-family functions here. Prebuilt hard-float newlib may
     * clobber s0-s15 even for a %s format; the bus ISR keeps live state there.
     * -ffixed-sN only applies to sources compiled with those flags, not libc.
     */
    memcpy(last_load_filename, folder, sizeof(folder) - 1);
    memcpy(last_load_filename + sizeof(folder) - 1, name, name_length + 1);
    if (suffix_match(name, ".rom") || suffix_match(name, ".bin")) {
        result = load_rom(last_load_filename, high_64k_base + ROM_STAGE_ADDRESS,
                          &file_size);
        if (result == FR_OK) {
            rom_pending = 1;
            launch_info(4, 0, 0, 0);
        }
        return result;
    }
    if (!suffix_match(name, ".cas")
#ifdef MODDED_SORD
        && !suffix_match(name, ".msx")
#endif
        ) return FR_INVALID_PARAMETER;

    result = load_cas(last_load_filename, high_64k_base, &cas_header);
    if (result != FR_OK) return result;
    end = (uint32_t)cas_header.Head + cas_header.Size;
    /* Only single-part machine code with an entry inside the loaded data.
     * 7000-72ff holds monitor state and the safe launch stack below 7300.
     */
    if ((cas_header.Attr != 1 && cas_header.Attr != 3) ||
        cas_header.Head < 0x7300 || cas_header.Start < cas_header.Head ||
        cas_header.Start >= end) return FR_INVALID_PARAMETER;
#ifdef MODDED_SORD
    if (suffix_match(name, ".msx")) {
        uint16_t launch_address;
        /* MSX BIOS occupies 0000-7fff; reserve 8000 for its RAM probe sentinel. */
        if (cas_header.Head <= 0x8000) return FR_INVALID_PARAMETER;
        /* The mapping switch must execute outside the menu it disconnects.
         * Prefer the gap before the game; otherwise use space just after it.
         */
        if (cas_header.Head >= 0x8001 + MSX_LAUNCH_SIZE) {
            launch_address = 0x8001;
        } else if (end <= SORD_MEMORY_SIZE - MSX_LAUNCH_SIZE) {
            launch_address = (uint16_t)end;
        } else {
            return FR_INVALID_PARAMETER;
        }
        memcpy(high_64k_base, msx, 0x8000);
        /* For MSX, HEAD publishes the free RAM address for the launch stub. */
        launch_info(1, launch_address, cas_header.Size, cas_header.Start);
        offset1000 = 1;
        file_size = cas_header.Size;
        return FR_OK;
    }
#endif
    file_size = cas_header.Size;
    launch_info(2, cas_header.Head, cas_header.Size, cas_header.Start);
    return FR_OK;
}

FRESULT commit_rom(void)
{
    if (!rom_pending) return FR_INVALID_OBJECT;
    rom_pending = 0;
    memcpy(rom_base, high_64k_base + ROM_STAGE_ADDRESS, ROM_CAPACITY);
    return FR_OK;
}

void cancel_load(void)
{
    rom_pending = 0;
    offset1000 = 0;
    launch_info(0, 0, 0, 0);
}
