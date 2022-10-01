#include "fatfs_handler.h"



/**
 * http://elm-chan.org/fsw/ff/doc/open.html
 * http://irtos.sourceforge.net/FAT32_ChaN/doc/en/lseek.html
 * Fonksiyonlarin wikisi
 */

/**
 * SD Karti kullanmak icin https://www.youtube.com/watch?v=spVIZO-jbxE 
 */



FATFS fs;
FATFS *pfs;
FIL fp; //file pointer
FRESULT fr; // file process result
DWORD fre_clust;
uint32_t totalSpace;
uint32_t freeSpace;



FRESULT mount_sd(){
    fr = f_mount(&fs, "", 0);
    return fr;
}


/**
 * @brief : Dosya boyutunu byte olarak doner.
 * @param  path: dosya ismi. Uzantisi ile
 * @return dosya boyutu
 */
unsigned int get_file_size(const char* path) {

    fr = f_open(&fp, path, FA_READ | FA_OPEN_EXISTING);
    if (fr != FR_OK){
        fr = close();
        return 0;
    }
    unsigned int re_v;
    re_v =  ((unsigned int) f_size(&fp));
    fr = close();
    return re_v;

}

void hello_world(){
    fr = f_open(&fp, "hello.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ);
    f_puts("Hello World from Receiver LENNA\n", &fp);
    fr = close();
}

/**
 * @brief : Veriyi sd txt dosyasina kaydeder
 * @param  data: veri
 * @return void
 */
void writeSD(char *data) {

	f_open(&fp, "qr_sky_lenna.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ);

	f_puts(data, &fp);

	fr = f_close(&fp);
	
}

/**
 * @brief : CHUNK_SIZE ve OTHER_DATA_OF_CHUNK a gore toplam chunk sayisi. Last chunk sayisini ogrenmek onemli.
 * @param path: dosya ismi. Uzantisi ile
 * @return chunk sayisi
 */
uint16_t get_total_chunk_number(const char* path) {

    unsigned int file_size = get_file_size(path);
    uint16_t total_chunk_number = file_size / FILE_SIZE_ON_CHUNK;
    return total_chunk_number;

}
   

/**
 * @brief : Dosyayi sifirdan olusturur. Var olan dosya silinir.
 * @param  path: dosya ismi. Uzantisi ile
 * @return: FRESULT: status
 */
FRESULT create_file(const char* path) {

    fr = f_open(&fp, path, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    fr = close(&fp);
    return fr;
}


/**
 * @brief : Veriyi dosyaya ekler.
 * @param write_buffer: verinin oldugu array. Arrayin baslangicina dikkat
 * @param write_size: verinin boyutu. Son chunkda degistirmek gerekir
 * @param path: dosya ismi. Uzantisi ile
 * @return: FRESULT: status
 */
FRESULT append_to_file(uint8_t *write_buffer, uint16_t write_size, const char* path) {

    fr = f_open(&fp, path, FA_OPEN_APPEND | FA_WRITE); 
    if (fr != FR_OK){
        fr = close();
        return fr;
    }

    // Write 
    uint32_t written_bytes; /*  Number of bytes written */
    fr = f_write(&fp, write_buffer, write_size, &written_bytes);

    fr  = close();

    return fr;

}

FRESULT append_to_anywhere(uint8_t *write_buffer, uint16_t write_size, unsigned int where_to_write, const char* path) {
    fr = f_open(&fp, path, FA_OPEN_EXISTING | FA_WRITE); 
    if (fr != FR_OK){
        fr = close();
        return fr;
    }
    fr = f_lseek(&fp, where_to_write);
    if (fr != FR_OK){
        fr = close();
        return fr;
    }
    uint32_t written_bytes; /*  Number of bytes written */
    fr = f_write(&fp, write_buffer, write_size, &written_bytes);
    fr = close();
    return fr;
}

/**
 * @brief : Dosyayi okur.
 * @param read_buffer: Verinin yazilcagi array. Arrayin baslangicina dikkat
 * @param read_size: okunacak verinin boyutu. Son chunkda degistirmek gerekir
 * @param where_to_read: dosyadan verinin okunacagi bolum. Dosya pointerini hareket ettirmis oluyorsun.
 * @param path: dosya ismi. Uzantisi ile
 * @return FRESULT: status
 */
FRESULT read_from_anywhere(uint8_t *read_buffer ,uint16_t read_size, unsigned int where_to_read, const char* path) {

    fr = f_open(&fp, path, FA_READ | FA_OPEN_EXISTING);
    if (fr != FR_OK){
        fr = close();
        return fr;
    }

    fr = f_lseek(&fp, where_to_read);
    

    uint32_t br;	
    fr = f_read(&fp, read_buffer,  read_size, &br);

    fr = close();
    return fr;
}


/**
 * @brief : Dosya pointeri globaldir ve onu kapatir.
 * @param : 
 * @return: FRESULT: status
 */
FRESULT close() {
    fr = f_close(&fp);
    return fr;
}

