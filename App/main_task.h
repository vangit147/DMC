/**
  *****************************************************************************
  * FILENAME   : main_task.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.24 22:22:05 Sunday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.24                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _MAIN_TASK_H_2023_09_24_22_4_5_702_
#define _MAIN_TASK_H_2023_09_24_22_4_5_702_

#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"

// 外部变量声明
extern int8_t downhole;  // 井下状态标志
extern float vSupply;  // 36V电源电压监测值

/******************************** Functions **********************************/
void get_acc(float *x, float *y, float *z);
void get_ko_value(float *k0, float *k1, float *k2);
float get_rp_ie_357(void);
void get_rp(float *e0, float *e1278, float *r1278, float *p1278, float *e357, float *r357, float *p357);
void set_downhole(int val);
int8_t get_downhole(void);
void start_and_get_adc_result(void);
void load_algorithm_setting_from_flash(void);
void record_log_to_flash(void);
void calculate_pump_off_inc(void);
void accumulate_pump_off_inc(void);
#ifdef __cplusplus
}
#endif
#endif

