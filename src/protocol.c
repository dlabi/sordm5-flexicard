#include "protocol.h"

/* IORD must return a byte within a Z80 bus cycle even in an -O0 firmware build. */
#if defined(__GNUC__)
#pragma GCC optimize ("O2")
#endif

static uint16_t file_index;
static volatile uint16_t received_index;
static uint8_t receive_remaining;
static volatile uint8_t load_ready;
static uint16_t transmit_value;
static uint8_t transmit_remaining;
static const uint8_t *filename;
static uint8_t filename_offset;
volatile uint32_t protocol_request_id;

void protocol_command(uint8_t command, const uint8_t *directory)
{
    ++protocol_request_id;
    receive_remaining = 0;
    transmit_remaining = 0;
    load_ready = 0;
    filename = 0;
    main_thread_data = 0;

    switch (command) {
    case SET_INDEX:
    case LOAD_FILE:
        received_index = 0;
        receive_remaining = 2;
        main_thread_data = 0xff;
        break;
    case GET_COUNT:
    case GET_INDEX:
        transmit_value = command == GET_COUNT ? menu_ctrl_file_count : file_index;
        transmit_remaining = 2;
        main_thread_data = (uint8_t)(transmit_value >> 8);
        break;
    case GET_FILENAME:
        if (file_index < menu_ctrl_file_count) {
            filename = directory + file_index * DIRECTORY_NAME_SIZE;
            main_thread_data = filename[0];
            filename_offset = 1;
        }
        break;
    case FIRST_FILE:
        file_index = 0;
        command = BREAK;
        break;
    case NEXT_FILE:
        if (file_index < menu_ctrl_file_count) ++file_index;
        main_thread_data = (uint8_t)file_index;
        command = BREAK;
        break;
    case PREV_FILE:
        if (file_index) --file_index;
        main_thread_data = (uint8_t)file_index;
        command = BREAK;
        break;
    case OFFSET_RAM_ON:
    case OFFSET_RAM_OFF:
        offset1000 = command == OFFSET_RAM_ON;
        command = BREAK;
        break;
    case DIR_SORD:
    case COMMIT_ROM:
        break; /* Work is performed by main, never by a bus interrupt. */
    case BREAK:
    case RESET_SORD:
        command = BREAK;
        break;
    default:
        main_thread_data = 0xff;
        command = BREAK;
        break;
    }
    /* Publish ready only after all state is initialized. */
    main_thread_command = command ? command | 0x80 : 0;
}

void protocol_write(uint8_t data)
{
    uint8_t command = main_thread_command & 0x7f;
    if (!(main_thread_command & 0x80) || !receive_remaining ||
        (command != SET_INDEX && command != LOAD_FILE)) return;

    received_index = (uint16_t)((received_index << 8) | data);
    if (--receive_remaining == 0) {
        if (command == SET_INDEX) {
            file_index = received_index;
            main_thread_data = file_index < menu_ctrl_file_count ? 0 : 0xff;
            main_thread_command = 0;
        } else {
            load_ready = 1;
        }
    }
}

uint8_t protocol_read(void)
{
    /* The ISR has already driven this prefetched byte onto the data bus.
     * Advance only after IORD rises and the bus has been released.
     */
    uint8_t result = main_thread_data;
    if (transmit_remaining) {
        if (--transmit_remaining) {
            main_thread_data = (uint8_t)transmit_value;
        } else {
            main_thread_data = 0;
            main_thread_command = 0;
        }
        return result;
    }
    if ((main_thread_command & 0x7f) == GET_FILENAME) {
        main_thread_data = result && filename && filename_offset < DIRECTORY_NAME_SIZE - 1 ?
                          filename[filename_offset++] : 0;
        if (!result) main_thread_command = 0;
        return result;
    }
    return result;
}

int protocol_take_load_index(uint16_t *index)
{
    if (main_thread_command != (LOAD_FILE | 0x80) || !load_ready) return 0;
    *index = received_index;
    load_ready = 0;
    return 1;
}
