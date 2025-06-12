/**
  *****************************************************************************
  * FILENAME   : uart_service.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.10.07 22:36:11 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.10.07                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _UART_SERVICE_H_2023_10_07_22_36_11_580_
#define _UART_SERVICE_H_2023_10_07_22_36_11_580_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"
#include "ie_task.h"  // 添加ie_task.h头文件，用于访问fitting函数


/******************************** Functions **********************************/
void handle_uart_msg(uint8_t* msg, uint32_t *msg_len);
void send_msg(void);


#ifdef __cplusplus
}
#endif
#endif

