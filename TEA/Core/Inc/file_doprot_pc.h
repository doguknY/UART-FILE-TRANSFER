#ifndef __FILE_DOPROT_PC_H__
#define __FILE_DOPROT_PC_H__

#include "usart.h"
#include "stdint.h"
#include "fatfs_handler.h"
#include "main.h"


void initFileReceivePc(UART_HandleTypeDef *__uart_handler,int __bufferSize, uint8_t *__fileName, uint8_t *receiveInterruptByte);    
uint8_t saveBufferToSD();
uint8_t gimmeTheBytePC(uint8_t theByte);

#endif // __FILE_DOPROT_PC_H__
