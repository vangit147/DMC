/**
  *****************************************************************************
  * FILENAME   : ads1278.h
  * COPYRIGHT  : MD.Tec(ShangHai) Co.,Ltd2024.
  * CREATEDDATE: 2025.04.06 22:04:20 Monday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * NickYang                  2024.02.19                    CREATE
  * AI Assistant              2024.06.01                    重构代码，添加全局变量结构体
  *****************************************************************************
  */
#ifndef __ADS1278_H__
#define __ADS1278_H__

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "main.h"

// ADS1278原始数据结构体
typedef struct {
    // ADS1278数据
    int32_t t_raw;
    int32_t ax_raw;
    int32_t ay_raw;
    int32_t az_raw;
    int32_t gx_raw;
    int32_t gy_raw;
    int32_t gz_raw;
} ads1278_global_raw_data_t;

// 全局变量声明
extern ads1278_global_raw_data_t s_ads1278_global_raw_data;


// 函数声明
void ads1278_reset(void);
uint8_t ads1278_read_reg(uint8_t reg);
void ads1278_write_reg(uint8_t reg, uint8_t val);
void ads1278_read_data(uint8_t *data);
void ads1278_data_ready_isr(uint32_t int_flag);

#endif /* __ADS1278_H__ */

