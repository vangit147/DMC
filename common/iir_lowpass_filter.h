/**
  *****************************************************************************
  * FILENAME   : iir_lowpass_filter.h
  * COPYRIGHT  : ModingTech(ShangHai) Co.,Ltd2025.
  * CREATEDDATE: 2025.01.24 13:21:08 Saturday
  * DESCRIPTION: IIR低通滤波器头文件，实现4阶IIR低通滤波器，等效于硬件RC同向低通滤波
  *              截止频率为0.12Hz，采样频率50Hz，用于对FIR低通滤波器后的信号进行进一步振动滤波
  *              增加一个低通滤波器 Fc=0.12Hz，采样频率=50Hz，用于实现低通滤波测斜
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  *   Gordon Li                  2025.01.24                    CREATE
  *   Gordon Li                  2025.08.27                    UPDATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _IIR_LOWPASS_FILTER_H_2025_01_24_13_21_8_678_
#define _IIR_LOWPASS_FILTER_H_2025_01_24_13_21_8_678_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "arm_math.h"

// IIR滤波器阶数
#define IIR_FILTER_ORDER_4TH 4  // 4阶滤波器


// IIR低通滤波器结构体（4阶，级联2个2阶节）
typedef struct {
    const float32_t *b1;            // 指向第1个2阶节的分子系数（指针，节省12字节）
    const float32_t *a1;            // 指向第1个2阶节的分母系数（指针，节省12字节）
    const float32_t *b2;            // 指向第2个2阶节的分子系数（指针，节省12字节）
    const float32_t *a2;            // 指向第2个2阶节的分母系数（指针，节省12字节）
    float32_t x1_delay[2];          // 第1个2阶节的输入延迟线
    float32_t y1_delay[2];          // 第1个2阶节的输出延迟线
    float32_t x2_delay[2];          // 第2个2阶节的输入延迟线
    float32_t y2_delay[2];          // 第2个2阶节的输出延迟线
    bool is_initialized;             // 初始化标志
} iir_lowpass_filter_4th_t;

// 函数声明

// 4阶IIR滤波器函数
void iir_lowpass_filter_4th_init(iir_lowpass_filter_4th_t *filter);
float32_t iir_lowpass_filter_4th_process(iir_lowpass_filter_4th_t *filter, float32_t input);
void iir_lowpass_filter_4th_reset(iir_lowpass_filter_4th_t *filter);


#ifdef __cplusplus
}
#endif
#endif
