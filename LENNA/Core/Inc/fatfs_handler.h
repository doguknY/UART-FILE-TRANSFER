#ifndef _FATFS_HANDLER_H_
#define _FATFS_HANDLER_H_

#include "app_fatfs.h"
#include <stdint.h>

#define CHUNK_SIZE 1024
#define OTHER_DATA_OF_CHUNK 8 //CRC, CHUNK NUM etc. 
#define FIRST_FILE_DATA_INDEX 5 //received_chunk[5] = first file data

#define FILE_SIZE_ON_CHUNK (CHUNK_SIZE - OTHER_DATA_OF_CHUNK)


extern FATFS fs;
extern FATFS *pfs;
extern FIL fp; //file pointer
extern FRESULT fr;
extern DWORD fre_clust;
extern uint32_t totalSpace;
extern uint32_t freeSpace;


FRESULT mount_sd();
unsigned int get_file_size(const char* path);
uint16_t get_total_chunk_number(const char* path);
FRESULT create_file(const char* path);
FRESULT append_to_file(uint8_t *write_buffer, uint16_t write_size, const char* path);
FRESULT read_from_anywhere(uint8_t *read_buffer ,uint16_t read_size, unsigned int where_to_read, const char* path);
FRESULT append_to_anywhere(uint8_t *write_buffer, uint16_t write_size, unsigned int where_to_write, const char* path);
FRESULT close();
void writeSD(char *data);
void hello_world();


typedef union {
    uint16_t u16;
    int16_t i16;
    uint8_t u8[2];
} u16_to_u8;

#endif // _FATFS_HANDLER_H_
