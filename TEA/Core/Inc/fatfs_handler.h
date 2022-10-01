#ifndef _FATFS_HANDLER_H_
#define _FATFS_HANDLER_H_

#include "app_fatfs.h"
#include <stdint.h>




extern FATFS fs;
extern FATFS *pfs;
extern FIL fp; //file pointer
extern FRESULT fr;
extern DWORD fre_clust;
extern uint32_t totalSpace;
extern uint32_t freeSpace;


FRESULT mount_sd();
unsigned int get_file_size(const char* path);
uint16_t get_total_chunk_number(const char* path, int file_size_on_chunk);
FRESULT create_file(const char* path);
FRESULT append_to_file(uint8_t *write_buffer, uint16_t write_size, const char* path);
FRESULT read_from_anywhere(uint8_t *read_buffer ,uint16_t read_size, unsigned int where_to_read, const char* path);
FRESULT append_to_anywhere(uint8_t *write_buffer, uint16_t write_size, unsigned int where_to_write, const char* path);
FRESULT close();
void writeSD(char *data);
void hello_world();


#endif // _FATFS_HANDLER_H_
