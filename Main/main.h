/**
  *****************************************************************************
  * FILENAME   : main.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.09 15:29:54 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.09                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _MAIN_H_2023_09_09_15_29_54_521_
#define _MAIN_H_2023_09_09_15_29_54_521_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "../version.h"
#include "device_registers.h"
#include "system_S32K144.h"
#include "cpu.h"

#include "S32K144_it.h"
#include "freertos.h"
#include "task.h"
#include "timers.h"

#include "init_calls.h"

#include "lpuart.h"
#include "i2c.h"
#include "spi.h"
#include "ads1278_imu.h"
#include "IAM_20680HT.h"
#include "gpio_port.h"
#include "timer.h"
#include "rtc_pca8565.h"
#include "IS25LP032_flash.h"

#include "uart_service.h"
#include "main_task.h"
#include "ie_task.h"

#include "common.h"
#include "quick_sort.h"

#include "fir_config.h"
typedef float real32_T;

#define MAJOR_VERSION       1
#define MINOR_VERSION       0

/******************************** Functions **********************************/
void DMA0_IRQHandler(void);
void DMA1_IRQHandler(void);
void DMA2_IRQHandler(void);
void DMA3_IRQHandler(void);
void DMA4_IRQHandler(void);
void DMA5_IRQHandler(void);
void DMA6_IRQHandler(void);
void DMA7_IRQHandler(void);
void DMA8_IRQHandler(void);
void DMA9_IRQHandler(void);
void DMA10_IRQHandler(void);
void DMA11_IRQHandler(void);
void DMA12_IRQHandler(void);
void DMA13_IRQHandler(void);
void DMA14_IRQHandler(void);
void DMA15_IRQHandler(void);

uint32_t CRC16(void* data, uint32_t len);
int print_buff(const void * v_addr, unsigned int size, const char* func, int line);
#define printm(v_addr, size) print_buff(v_addr, size, __func__, __LINE__)
void (*get_DMA_IRQHandler(uint32_t channel))(void);

#ifdef __cplusplus
}
#endif
#endif

