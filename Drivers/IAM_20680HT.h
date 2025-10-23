/**
  *****************************************************************************
  * FILENAME   : IAM_20680HT.h
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
/******************************************************************************/
#ifndef __IAM_20680HT_H__
#define __IAM_20680HT_H__

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "main.h"
#include "IS25LP032_flash.h"
//#include "filtering_functions.h"

// IAM-20680HT原始数据结构体
typedef struct {
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    int16_t t_raw;
} iam_global_raw_data_t;

// 全局变量声明
extern iam_global_raw_data_t s_iam_global_raw_data;
extern TaskHandle_t iam_20680ht_task_handle;

// 添加iam_20680ht_read_reg函数声明
uint8_t iam_20680ht_read_reg(uint32_t reg);

// 函数声明
int32_t iam_20680ht_init(void);
void iam_20680ht_task(void *pvParameters);

#endif /* __IAM_20680HT_H__ */

