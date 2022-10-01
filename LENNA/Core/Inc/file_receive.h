#ifndef _FILE_RECEIVE_H_
#define _FILE_RECEIVE_H_

#include <stdint.h>
#include "fatfs_handler.h"




extern uint8_t is_ready_to_parse;
extern uint8_t receiving_status;

extern u16_to_u8 received_chunk_number;
extern u16_to_u8 total_chunk_num;


/*
ALINAN HAM VERININ YAPISI

START_KEY
CHUNK NUM BYTE_1
CHUNK NUM BYTE_2

VERI
..
..
..
VERI

CRC BYTE_1
CRC BYTE_2
END_KEY

*/

/*
REQUEST YAPISI

requsted_chunk BYTE_1
requsted_chunk BYTE_2
is_ready_to_parse

*/

uint8_t init_file_receive();
uint8_t receive_chunk();
uint8_t gimme_THE_BYTE(uint8_t the_byte);
void loop_count_reset();
void free_received_chunk();





#endif // _FILE_RECEIVE_H_