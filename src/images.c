#include "util.h"
#include <strings.h>

uint32_t suffix_match(char *name, char *suffix)
{
    size_t n = strlen(name), s = strlen(suffix);
    return n > s && strncasecmp(name + n - s, suffix, s) == 0;
}

static FRESULT read_exact(FIL *file, void *buffer, UINT length)
{
    UINT count;
    FRESULT result = f_read(file, buffer, length, &count);
    return result != FR_OK ? result : count == length ? FR_OK : FR_INT_ERR;
}

static FRESULT close_result(FIL *file, FRESULT result)
{
    FRESULT closed = f_close(file);
    return result == FR_OK ? closed : result;
}

FRESULT load_rom(char *name, unsigned char *buffer, unsigned int *file_size)
{
    FIL file;
    FRESULT result;
    FSIZE_t size;
    uint8_t type;
    UINT offset;

    *file_size = 0;
    result = f_open(&file, name, FA_READ);
    if (result != FR_OK) return result;
    size = f_size(&file);
    if (size < 16 || size > ROM_CAPACITY) {
        return close_result(&file, FR_INVALID_PARAMETER);
    }
    result = read_exact(&file, &type, 1);
    if (result != FR_OK) return close_result(&file, result);
    if (type > 2) {
        return close_result(&file, FR_INVALID_PARAMETER);
    }
    offset = type == 1 ? 0x2000 : 0;
    /* Header type selects the mapping, not a maximum file length. Homebrew
     * type-0 cartridges (e.g. Ascend/Battlot) extend beyond 8 KiB. Bound the
     * relocated image by the full 2000h-6fffh cartridge window instead.
     */
    if (size > ROM_CAPACITY - offset) {
        return close_result(&file, FR_INVALID_PARAMETER);
    }
    result = f_lseek(&file, 0);
    if (result == FR_OK) {
        memset(buffer, 0xff, ROM_CAPACITY);
        result = read_exact(&file, buffer + offset, (UINT)size);
    }
    result = close_result(&file, result);
    if (result == FR_OK) *file_size = (unsigned int)size;
    return result;
}

FRESULT load_binary(char *name, unsigned char *buffer, int size)
{
    FIL file;
    FRESULT result;
    if (size < 0) return FR_INVALID_PARAMETER;
    result = f_open(&file, name, FA_READ);
    if (result != FR_OK) return result;
    result = read_exact(&file, buffer, (UINT)size);
    return close_result(&file, result);
}

static uint16_t little_word(const uint8_t *bytes)
{
    return (uint16_t)(bytes[0] | ((uint16_t)bytes[1] << 8));
}

/* SORDM5: 16-byte signature area, then type/length/payload/checksum blocks.
 * Length zero means 256. Support one H header followed by D/F data blocks.
 * Checksums are consumed (as in the original loader), not validated.
 * Reference: mamedev/mame, src/lib/formats/sord_cas.cpp.
 */
FRESULT load_cas(char *name, unsigned char *buffer, SORD_HEADER *header)
{
    FIL file;
    FRESULT result;
    SORD_HEADER parsed;
    uint8_t bytes[256], block[2], checksum;
    uint32_t offset = 0;
    UINT length, copy_length;

    memset(header, 0, sizeof(*header));
    memset(&parsed, 0, sizeof(parsed));
    result = f_open(&file, name, FA_READ);
    if (result != FR_OK) return result;
    result = read_exact(&file, bytes, 16);
    if (result != FR_OK) return close_result(&file, result);
    if (memcmp(bytes, "SORDM5", 6) != 0) {
        return close_result(&file, FR_INVALID_PARAMETER);
    }
    result = read_exact(&file, block, 2);
    if (result != FR_OK) return close_result(&file, result);
    if (block[0] != 'H' || block[1] != 31) {
        return close_result(&file, FR_INVALID_PARAMETER);
    }
    result = read_exact(&file, bytes, 32); /* 31-byte header plus checksum */
    if (result != FR_OK) return close_result(&file, result);
    parsed.Attr = bytes[0];
    memcpy(parsed.Name, bytes + 1, sizeof(parsed.Name));
    parsed.Head = little_word(bytes + 10);
    parsed.Size = little_word(bytes + 12);
    parsed.Start = little_word(bytes + 14);
    parsed.E_attr = bytes[16];
    memcpy(parsed.Blank, bytes + 17, sizeof(parsed.Blank));
    parsed.Checksum = bytes[31];
    if (!parsed.Size || (uint32_t)parsed.Head + parsed.Size > SORD_MEMORY_SIZE) {
        return close_result(&file, FR_INVALID_PARAMETER);
    }

    while (f_tell(&file) < f_size(&file)) {
        result = read_exact(&file, block, 2);
        if (result != FR_OK) break;
        if (block[0] != 'D' && block[0] != 'F') {
            result = FR_INVALID_PARAMETER;
            break;
        }
        length = block[1] ? block[1] : 256;
        copy_length = length;
        if (length > (uint32_t)parsed.Size - offset) {
            /* Exporters may pad the final 256-byte block. Read the complete
             * block/checksum, but never copy padding past the declared image.
             * Extra blocks or another header still require multipart support.
             */
            if (offset == parsed.Size || length != 256 ||
                f_size(&file) - f_tell(&file) != length + 1) {
                result = FR_INVALID_PARAMETER;
                break;
            }
            copy_length = parsed.Size - offset;
        }
        result = read_exact(&file, bytes, length);
        if (result != FR_OK) break;
        result = read_exact(&file, &checksum, 1);
        if (result != FR_OK) break;
        memcpy(buffer + parsed.Head + offset, bytes, copy_length);
        offset += copy_length;
    }
    if (result == FR_OK && offset != parsed.Size) result = FR_INT_ERR;
    result = close_result(&file, result);
    if (result == FR_OK) *header = parsed;
    return result;
}

static int supported_image(char *name)
{
    return suffix_match(name, ".rom") || suffix_match(name, ".bin") ||
           suffix_match(name, ".cas")
#ifdef MODDED_SORD
           || suffix_match(name, ".msx")
#endif
           ;
}

uint32_t load_directory(char *dirname, uint8_t *index_buffer, uint32_t max_files)
{
    DIR dir;
    FILINFO info;
    FRESULT result;
    uint32_t count = 0;
    char *name;
    size_t length;

    if (max_files > MENU_MAX_DIRECTORY_ITEMS) max_files = MENU_MAX_DIRECTORY_ITEMS;
    memset(index_buffer, 0, DIRECTORY_NAME_SIZE * max_files);
    result = f_opendir(&dir, dirname);
    if (result != FR_OK) return 0;
    while (count < max_files) {
        result = f_readdir(&dir, &info);
        if (result != FR_OK || !info.fname[0]) break;
        if (info.fattrib & (AM_DIR | AM_HID | AM_SYS | 0x08)) continue;
        name = info.fname;
        if (!supported_image(name)) continue;
        length = strlen(name);
        if (length >= DIRECTORY_NAME_SIZE) {
#if FF_USE_LFN
            name = info.altname; /* Keep a usable path instead of truncating an LFN. */
            length = strlen(name);
#endif
            if (!length || length >= DIRECTORY_NAME_SIZE) continue;
        }
        memcpy(index_buffer + count * DIRECTORY_NAME_SIZE, name, length + 1);
        ++count;
    }
    if (f_closedir(&dir) != FR_OK || result != FR_OK) return 0;
    return count;
}

uint8_t get_filename(const uint8_t *index_buffer, char *buffer,
                     uint32_t file_num, uint32_t file_count)
{
    size_t length = 0;
    buffer[0] = 0;
    if (file_num >= file_count || file_num >= MENU_MAX_DIRECTORY_ITEMS) return 0;
    index_buffer += file_num * DIRECTORY_NAME_SIZE;
    while (length < DIRECTORY_NAME_SIZE - 1 && index_buffer[length]) {
        buffer[length] = index_buffer[length];
        ++length;
    }
    buffer[length] = 0;
    return (uint8_t)length;
}
