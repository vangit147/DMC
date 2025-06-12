/**
  *****************************************************************************
  * FILENAME   : lpuart.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.23 23:54:30 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.23                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _LPUART_H_2023_09_23_23_54_30_917_
#define _LPUART_H_2023_09_23_23_54_30_917_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/



/******************************** Functions **********************************/
int32_t LPUART0_init(void (*tx_cb)(uint32_t), void (*rx_cb)(uint32_t, uint32_t));
int32_t LPUART0_send(uint8_t* data, uint32_t len);

int32_t LPUART1_init(void (*tx_cb)(uint32_t), void (*rx_cb)(uint32_t, uint32_t));
int32_t LPUART1_send(uint8_t* data, uint32_t len);

int32_t LPUART2_init(void (*tx_cb)(uint32_t), void (*rx_cb)(uint32_t, uint32_t));
int32_t LPUART2_deinit(void);
int32_t LPUART2_send(uint8_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif
#endif

