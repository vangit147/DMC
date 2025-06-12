/**
  *****************************************************************************
  * FILENAME   : spi.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.24 23:46:23 Sunday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.24                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _SPI_H_2023_09_24_23_46_23_150_
#define _SPI_H_2023_09_24_23_46_23_150_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/



/******************************** Functions **********************************/
int32_t spi0_init(void);
int32_t spi1_init(void);
int32_t spi2_init(void);
void spi2_register_end_transfer_cb(void (*cb)(void));
int32_t spi0_cs0_transfer(uint8_t* send_buff, uint8_t* receive_buff, uint32_t len);
int32_t spi0_cs1_transfer(uint8_t* send_buff, uint8_t* receive_buff, uint32_t len);
int32_t spi0_cs2_transfer(uint8_t* send_buff, uint8_t* receive_buff, uint32_t len);

#ifdef __cplusplus
}
#endif
#endif

