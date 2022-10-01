#ifndef _FILE_TRANSMIT_H_
#define _FILE_TRANSMIT_H_

#include <stdint.h>
#include "fatfs_handler.h"


extern uint16_t am_i_ready_to_send_chunk;

extern u16_to_u8 which_chunk_to_send; 

uint8_t init_file_transmit(uint8_t *transmission_info_message);
uint8_t create_chunk(uint8_t *chunk_to_send, uint8_t was_the_transmission_successful);

#endif // _FILE_TRANSMIT_H_
