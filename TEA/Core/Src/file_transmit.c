#include "file_transmit.h"
#include "crc.h"


uint16_t am_i_ready_to_send_chunk;

u16_to_u8 which_chunk_to_send; 
uint16_t total_chunk_number;

uint64_t file_size; //byte 

uint16_t last_chunk_file_data_size;

#define INFO_START_KEY '\r'
#define INFO_END_KEY '\n'

#define CHUNK_START_KEY '\r'
#define CHUNK_END_KEY '\n'

/**
 * @brief  Dosya aktarim oncesi ilk mesaj. Toplam chunk sayisi ve dosya boyutu gonderilir.
 * Olumlu cevap gelince am_i_ready_to_send_chunk'i 1 yapip aktarimi baslat.
 */
void create_info_message(uint8_t *info_message){
    info_message[0] = (uint8_t)INFO_START_KEY;

    u16_to_u8 tot_chunk;
    tot_chunk.u16 = total_chunk_number; 
    info_message[1] = tot_chunk.u8[0];
    info_message[2] = tot_chunk.u8[1];

    u16_to_u8 last_chunk_s;
    last_chunk_s.u16 = last_chunk_file_data_size;
    info_message[3] = last_chunk_s.u8[0];
    info_message[4] = last_chunk_s.u8[1];

    info_message[5] = (uint8_t)INFO_END_KEY;
}


/* @brief: File_transmit islemini baslatir. Degiskenleri falan tanimlar.
 * Daha sonra create_chunk fonksiyonu calistirilmalidir.
 * 
 * @param transmission_info_message: gonderilecek ilk bilgi paketi. 6 bytedir.
 * bundan olumlu cevap gelince am_i_ready_to_send_chunk degiskeni 1 yapilmalidir.
 * 
 * @return: basarisizsa 0 doner. (Dosyaya ulasilamadi)
 */
uint8_t init_file_transmit(uint8_t *transmission_info_message) {

   
    file_size = get_file_size(NAME_OF_TRANSMITTED_FILE_SKY);
    if (file_size == 0 || fr != FR_OK) {
        return fr;  // Dosya okunurken hata
    }
    total_chunk_number = get_total_chunk_number(NAME_OF_TRANSMITTED_FILE_SKY, FILE_SIZE_ON_CHUNK);
    last_chunk_file_data_size = file_size - (total_chunk_number * FILE_SIZE_ON_CHUNK);

    which_chunk_to_send.u16 = 0; 
    am_i_ready_to_send_chunk = 0; //no

    create_info_message(transmission_info_message);

    return FR_OK;
}


/* @brief: Dosyayi SD carddan okuyup paketler
 * @param: void
 * @return: void    
 */
void pack_chunk(uint8_t *chunk_to_send) {

    chunk_to_send[0] = (uint8_t)CHUNK_START_KEY;

    chunk_to_send[1] = which_chunk_to_send.u8[0];
    chunk_to_send[2] = which_chunk_to_send.u8[1];

    uint16_t data_size_to_read;

    if (which_chunk_to_send.u16 == total_chunk_number){
    	data_size_to_read = last_chunk_file_data_size;
    } else if(which_chunk_to_send.u16 < total_chunk_number ){
        data_size_to_read = FILE_SIZE_ON_CHUNK;
    } else {
        data_size_to_read = 0;
    }

    u16_to_u8 data_size_to_read_u16;
    data_size_to_read_u16.u16 = data_size_to_read;
    chunk_to_send[3] = data_size_to_read_u16.u8[0];
    chunk_to_send[4] = data_size_to_read_u16.u8[1];
    
    fr = read_from_anywhere(chunk_to_send + FIRST_FILE_DATA_INDEX, data_size_to_read, which_chunk_to_send.u16 * FILE_SIZE_ON_CHUNK, NAME_OF_TRANSMITTED_FILE_SKY);


    u16_to_u8 crc;
    crc.u16 = CRCCalculator(chunk_to_send, (CHUNK_SIZE - 3));
    chunk_to_send[CHUNK_SIZE - 3] = crc.u8[0];
    chunk_to_send[CHUNK_SIZE - 2] = crc.u8[1];

    chunk_to_send[CHUNK_SIZE - 1] = (uint8_t)CHUNK_END_KEY;
}



/* @brief: Surekli donen dongude calismasi lazim
 *
 * @param chunk_to_send: gonderilecek chunk. mainde calloc ile olusturup buraya aktar.
 * 
 * @param was_the_transmission_successful: receiver aygittan gelen geri bildirim mesajinda basari durumu olur. Aktarim basarili ise bu 1 olmali.
 * Bozuksa 0 olur ve aynisini paketler.
 * 
 * @return: bos dongu ise 0, paketleme yapmissa 1, daha paketleme yapmayacaksa 2(tamamlandi).
 */
uint8_t create_chunk(uint8_t *chunk_to_send, uint8_t was_the_transmission_successful) {

    if (am_i_ready_to_send_chunk == 1) {

        am_i_ready_to_send_chunk = 0;

        if (was_the_transmission_successful == 1)
            which_chunk_to_send.u16++;
            
        if (which_chunk_to_send.u16 > total_chunk_number) return 2; // WHYYY
       

        memset(chunk_to_send, 0, CHUNK_SIZE);
        pack_chunk(chunk_to_send);

        return 1;

    } else return 0;


}
