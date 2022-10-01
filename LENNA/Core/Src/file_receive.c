#include "file_receive.h"
#include "fatfs_handler.h"
#include "crc.h"
#include "def.h"


uint8_t is_ready_to_parse;

uint8_t *received_chunk; 

u16_to_u8 received_chunk_number;
u16_to_u8 total_chunk_num;

uint8_t info_message[6];

u16_to_u8 last_chunk_size; // file data

#define INFO_START_KEY '\r'
#define INFO_END_KEY '\n'

#define CHUNK_START_KEY '\r'
#define CHUNK_END_KEY '\n'





/* @brief: dosya alimi icin degiskenleri falan tanimlar.
 * @param: void
 * @return: void
 */
uint8_t init_file_receive() {


    received_chunk = (uint8_t *)calloc(CHUNK_SIZE, sizeof(uint8_t));

    fr = create_file(NAME_OF_RECEIVED_FILE_FROM_GROUND);
    if (fr != FR_OK) {
        return fr; // sd hatasi
    }
    
    is_ready_to_parse = 0;
    return FR_OK;
}

void free_received_chunk() {
    free(received_chunk);
}


/* @brief: Gelen chunkin crc'ini kontrol eder.
 * @param: Gelen chunk
 * @return: 1: crc ok, 0: crc hatali
 */
uint8_t received_chunk_control(uint8_t chunk[CHUNK_SIZE]) {

    u16_to_u8 crc;
    crc.u16 = CRCCalculator(chunk, (CHUNK_SIZE - 3));

    u16_to_u8 embedded_crc;
    embedded_crc.u8[0] = chunk[CHUNK_SIZE - 3];
    embedded_crc.u8[1] = chunk[CHUNK_SIZE - 2];

    return (embedded_crc.u16 == crc.u16) ? 1 : 0;
}

/* @brief: Dongu icerisinde olmalidir. Chunk alinip 256 byte dolunca is_ready_to_parse 1 olur ve chunki parse eder.
 * is_ready_to_parse global degiskendir.
 * @param: void
 * @return: chunk bozuksa 2, islem basarili ise 1
 * Daha sonrasinda buna gore mainde islem yapilir, geri bildirim gonderilir.
 */
uint8_t receive_chunk() {

    if (info_message[0] == '\0' && info_message[1] == '\0') return 5;

    if(is_ready_to_parse == 0) return 0;

    is_ready_to_parse = 0;

    uint8_t crc_check = received_chunk_control(received_chunk);
    if (crc_check != 1) {
        // memset(received_chunk, 0, CHUNK_SIZE);
        return 2;  // crc error bir daha ayni paketi iste
    }

    received_chunk_number.u8[0] = received_chunk[1];
    received_chunk_number.u8[1] = received_chunk[2]; 
    if(received_chunk_number.u16 > total_chunk_num.u16) return 4;


    u16_to_u8 data_to_write;
    data_to_write.u8[0] = received_chunk[3];
    data_to_write.u8[1] = received_chunk[4];

    append_to_anywhere(received_chunk + FIRST_FILE_DATA_INDEX, data_to_write.u16, (received_chunk_number.u16 * FILE_SIZE_ON_CHUNK), NAME_OF_RECEIVED_FILE_FROM_GROUND);

    if (received_chunk_number.u16 == total_chunk_num.u16) return 3;
    
    // memset(received_chunk, 0, CHUNK_SIZE);

    return 1;
}




// --------------------------------------------------------------------------------------------------------------------


enum {
    LISTENING_FOR_INFO_MESSAGE,
    LISTENING_FOR_CHUNKS
};
uint8_t receiving_status = LISTENING_FOR_INFO_MESSAGE;



uint16_t loop_count = CHUNK_SIZE + 10;


uint32_t start_time;

uint32_t receive_timeout;  // Bytelarin yarida kesilmesine karsi timeout.


void loop_count_reset() {
    loop_count = CHUNK_SIZE + 10;
}

uint8_t gimme_THE_BYTE(uint8_t the_byte) {


    switch (receiving_status) {

        case LISTENING_FOR_INFO_MESSAGE: // info mesaji surekli gonderilse iyi olur 
            loop_count++;
            if(loop_count <= 5) info_message[loop_count] = the_byte;                
        
            if (the_byte == (uint8_t)INFO_START_KEY && (loop_count > 5) ) {
                info_message[0] = the_byte;
                loop_count = 0;

            }

            if (loop_count == 5) {
                if(the_byte == (uint8_t)INFO_END_KEY) { // info_message[5] = INFO_END_KEY; mesaji alindi.
                    
                    total_chunk_num.u8[0] = info_message[1];
                    total_chunk_num.u8[1] = info_message[2];

                    last_chunk_size.u8[0] = info_message[3];
                    last_chunk_size.u8[1] = info_message[4];

                    receiving_status = LISTENING_FOR_CHUNKS;
                    loop_count = CHUNK_SIZE + 10;
                    received_chunk[23] = 23;
                    received_chunk[25] = the_byte;       // yanlis parcalama icin sacma degerler
                    received_chunk[26] = info_message[1]; 
                    received_chunk[27] = info_message[2];
                    is_ready_to_parse = 1; // bos bir sekilde received_chunk parse edilsin. Bassarisiz olunca sifirinci chunki ister. Yani hizlica baslatilmis olur aktarim.  
                    return 5; 
                }   
            }
            return 0;                        
            break;





        case LISTENING_FOR_CHUNKS:

            loop_count++;
            if(loop_count < CHUNK_SIZE) 
                received_chunk[loop_count] = the_byte;

            if (the_byte == (uint8_t)CHUNK_START_KEY && (loop_count > CHUNK_SIZE) ) {
                received_chunk[0] = the_byte;
                loop_count = 0;
            }

            if (loop_count == CHUNK_SIZE - 1) { // received_chunk[255]
                if(the_byte == (uint8_t)CHUNK_END_KEY) { 
                    received_chunk[CHUNK_SIZE - 1] = the_byte;
                    is_ready_to_parse = 1;  // while dongusunde receive fonksiyonu calismaya baslayacak ve received_chunki parse edecek.
                    loop_count = CHUNK_SIZE + 10;  // loop_count CHUNK_SIZE'dan buyukse start keyi dinleniyor.
                    return 1;
                }
                else {
                    is_ready_to_parse = 1;  
                    loop_count = CHUNK_SIZE + 10;
                    return 2; // CHUNK dogru degil. Kaydı veya sacma seyler oldu. bir daha iste. while'daki receive_chunk fonksiyonu oto yapicaktir.
                }
            }
            return 0;
            break;
        
        default:
            break;
    }

}
