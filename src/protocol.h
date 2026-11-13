#ifndef FC_PROTOCOL_H
#define FC_PROTOCOL_H

#include "main.h"

extern volatile uint32_t protocol_request_id;

/* Command/data functions run only in the I/O ISRs. Main consumes complete loads. */
void protocol_command(uint8_t command, const uint8_t *directory);
void protocol_write(uint8_t data);
uint8_t protocol_read(void);
/* main_thread_data is the prefetched DATA_PORT response. The hardware ISR
 * drives it first, releases the bus, then calls protocol_read() to advance.
 */
int protocol_take_load_index(uint16_t *index);

#endif
