

#include "file_doprot_pc.h"


UART_HandleTypeDef *uart_handler;
int bufferSize;
uint8_t *fileBuffer;
uint8_t *fileName;


int _index_i;
uint8_t _flag;
uint8_t _bufferHalf = 3;
int _timeoutPc;
int _fileSize;

uint8_t *_interruptBytePointer;


typedef enum {
    IDLE,
    LISTENING_FROM_PC,
    RECEIVING_FROM_PC

};

uint8_t _transmissionState = IDLE;




static void _led(uint8_t state){
    if(state == 1){
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }else if(state == 0){
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    } else {
        for(uint8_t i = 0; i < state; i++){
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            HAL_Delay(40);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            HAL_Delay(40);
        }
    }
}



static void _saveToSD(uint8_t which_half){
  // append_to_file(receive_buffer + (((POINTER_SIZE / 2) - 1) * (which_half - 1) ), POINTER_SIZE / 2, "received_video_pc.mp4");
  if (which_half == 1){
    append_to_file(fileBuffer, bufferSize / 2, fileName); 
  }
  else if(which_half == 2){
    append_to_file(fileBuffer + (bufferSize / 2), bufferSize / 2, fileName);
  }
  _flag = 0;
}

static void _saveToSDWithSize(uint8_t which_half, uint16_t size){
  append_to_file(fileBuffer + ((bufferSize / 2) * (which_half - 1) ), size, fileName);
  _flag = 0;
}









void initFileReceivePc(UART_HandleTypeDef *__uart_handler,int __bufferSize, uint8_t *__fileName, uint8_t *receiveInterruptByte)
{

    uart_handler = __uart_handler;
    bufferSize = __bufferSize;
    fileName = __fileName;
    create_file(fileName);

    fileBuffer = (uint8_t *)malloc(bufferSize * sizeof(uint8_t));

    _transmissionState = LISTENING_FROM_PC;


    

    HAL_UART_Receive_IT(uart_handler, receiveInterruptByte, 1);

    _interruptBytePointer = receiveInterruptByte;

    _led(8);
}


uint8_t saveBufferToSD(){
    switch (_transmissionState) {
        
        case LISTENING_FROM_PC:        
            HAL_UART_Receive_IT(uart_handler, _interruptBytePointer, 1);
            break;

        case RECEIVING_FROM_PC:
            if (_flag > 0){
                _saveToSD(_flag);
            }
            int32_t time_for_finish;
            time_for_finish = HAL_GetTick() - _timeoutPc;

            if (time_for_finish > 500 && _bufferHalf < 3 && _fileSize > 33300){
                
                if(_bufferHalf == 1){
                    _saveToSDWithSize(1, _index_i + 1);
                } else if(_bufferHalf == 2){
                    _saveToSDWithSize(2, _index_i - (bufferSize / 2) + 1);
                }
                free(fileBuffer);
                _transmissionState = LISTENING_FROM_PC;
                return 2;

            }

            
            break;
    }
    return 1;
    
}






uint8_t gimmeTheBytePC(uint8_t theByte)
{
    switch(_transmissionState){

        case LISTENING_FROM_PC:
          if (theByte == (uint8_t)'D' && (_bufferHalf == 3)){
            _transmissionState = RECEIVING_FROM_PC;
            _index_i = 0 - 1;  
            _timeoutPc = HAL_GetTick();
            _bufferHalf = 1;
            _led(1);   
            return 2;
          }
          break;


        case RECEIVING_FROM_PC:
          _index_i++;
          fileBuffer[_index_i] = theByte;

          if (_bufferHalf == 1){

            if(_index_i == (bufferSize / 2) - 1){  
              _bufferHalf = 2;
              _flag = 1;   

            }

          } else if(_bufferHalf == 2){

            if(_index_i == (bufferSize - 1)){ 
              _bufferHalf = 1;
              _index_i = 0 - 1;
              _flag = 2; 


            }
          }
          _timeoutPc = HAL_GetTick();
          _fileSize++;

        default:
          break;  
      }
      return 1;
    
}
