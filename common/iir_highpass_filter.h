/**
  *****************************************************************************
  * FILENAME   : iir_highpass_filter.h
  * COPYRIGHT  : ModingTech(ShangHai) Co.,Ltd2025.
  * CREATEDDATE: 2025.09.10 20:36
  * DESCRIPTION: IIR高通滤波器头文件，实现16阶IIR高通巴特沃斯滤波器
  *              支持100Hz采样率，多种截止频率，用于对低速旋转的加速度计信号进行离心力分离
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  *   Gordon Li                  2025.09.10                    CREATE
  *****************************************************************************
  */

  /*
 * 离散时间 IIR 滤波器(实数)
 * ----------------
 * 滤波器结构  : 直接 II 型，二阶节
 * 阶数       : 16
 * 节数     : 8
 * 稳定     : 是
 * 线性相位   : 否
 */
/******************************************************************************/
#ifndef _IIR_HIGHPASS_FILTER_H_2025_09_10_20_36_8_678_
#define _IIR_HIGHPASS_FILTER_H_2025_09_10_20_36_8_678_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

// IIR滤波器阶数
#define IIR_HIGHPASS_FILTER_ORDER_16TH 16  // 16阶滤波器
#define IIR_HIGHPASS_SECTIONS 8            // 8个二阶节

// 100Hz采样率滤波器类型枚举
typedef enum {
    IIR_HIGHPASS_100_000_HZ = -1,  // 无需高通滤波
    IIR_HIGHPASS_100_005_HZ = 0,  // 0.05Hz截止频率
    IIR_HIGHPASS_100_020_HZ,      // 0.2Hz截止频率
    IIR_HIGHPASS_100_030_HZ,      // 0.3Hz截止频率
    IIR_HIGHPASS_100_050_HZ,      // 0.5Hz截止频率
    IIR_HIGHPASS_100_070_HZ,      // 0.7Hz截止频率
    IIR_HIGHPASS_100_120_HZ,      // 1.2Hz截止频率
    IIR_HIGHPASS_100_170_HZ,      // 1.7Hz截止频率
    IIR_HIGHPASS_100_220_HZ,      // 2.2Hz截止频率
    IIR_HIGHPASS_100_270_HZ,      // 2.7Hz截止频率
    IIR_HIGHPASS_100_320_HZ,      // 3.2Hz截止频率
    IIR_HIGHPASS_100_TYPE_COUNT   // 100Hz采样率滤波器类型总数
} iir_highpass_100_filter_type_t;

// IIR高通滤波器结构体（16阶，8个二阶节）
typedef struct {
    float32_t x_delay[IIR_HIGHPASS_SECTIONS][2];  // 各节的输入延迟线
    float32_t y_delay[IIR_HIGHPASS_SECTIONS][2];  // 各节的输出延迟线
    iir_highpass_100_filter_type_t filter_type;   // 滤波器类型
    bool is_initialized;                          // 初始化标志
} iir_highpass_filter_16th_t;

// 函数声明

// 16阶IIR高通滤波器函数
void iir_highpass_filter_16th_init(iir_highpass_filter_16th_t *filter, iir_highpass_100_filter_type_t type);
float32_t iir_highpass_filter_16th_process(iir_highpass_filter_16th_t *filter, float32_t input);
void iir_highpass_filter_16th_reset(iir_highpass_filter_16th_t *filter);
void iir_highpass_filter_16th_set_type(iir_highpass_filter_16th_t *filter, iir_highpass_100_filter_type_t type);

// 批量初始化和重置函数
void iir_highpass_filter_16th_init_all_100hz(iir_highpass_filter_16th_t filters[IIR_HIGHPASS_100_TYPE_COUNT]);
void iir_highpass_filter_16th_reset_all_100hz(iir_highpass_filter_16th_t filters[IIR_HIGHPASS_100_TYPE_COUNT]);

// 获取滤波器类型对应的截止频率
float32_t iir_highpass_get_cutoff_freq(iir_highpass_100_filter_type_t type);

#ifdef __cplusplus
}
#endif
#endif
