#ifndef _FILE_TRANSMIT_H_
#define _FILE_TRANSMIT_H_

#include <stdint.h>
#include "fatfs_handler.h"


#define CHUNK_SIZE 1024
#define OTHER_DATA_OF_CHUNK 8 //CRC, CHUNK NUM etc. 
#define FIRST_FILE_DATA_INDEX 5 //received_chunk[5] = first file data

#define FILE_SIZE_ON_CHUNK (CHUNK_SIZE - OTHER_DATA_OF_CHUNK)

#define NAME_OF_RECEIVED_FILE_FROM_PC "received_video_from_pc.mp4"
#define NAME_OF_TRANSMITTED_FILE_SKY NAME_OF_RECEIVED_FILE_FROM_PC
#define NAME_OF_RECEIVED_FILE_FROM_SKY "received_video_from_sky.mp4"

extern uint16_t am_i_ready_to_send_chunk;


uint8_t init_file_transmit(uint8_t *transmission_info_message);
uint8_t create_chunk(uint8_t *chunk_to_send, uint8_t was_the_transmission_successful);

#endif // _FILE_TRANSMIT_H_
