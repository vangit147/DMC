/**
  *****************************************************************************
  * FILENAME   : iir_lowpass_filter.h
  * COPYRIGHT  : ModingTech(ShangHai) Co.,Ltd2025.
  * CREATEDDATE: 2025.01.24 13:21:08 Saturday
  * DESCRIPTION: IIR低通滤波器头文件，采样率100Hz
  *             实现8阶IIR低通滤波器，截止频率为0.15Hz
  *             实现16阶IIR低通滤波器，截止频率为0.12Hz
  *             实现100Hz采样率多种截止频率的16阶IIR低通滤波器(butterworth)
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  *   Gordon Li                  2025.01.24                    CREATE
  *   Gordon Li                  2025.08.27                    UPDATE
  *   Gordon Li                  2025.09.10                    ADD 100Hz多种截止频率支持
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
#define IIR_FILTER_ORDER_8TH 8  // 8阶滤波器
#define IIR_FILTER_ORDER_16TH 16 // 16阶滤波器
#define IIR_LOWPASS_SECTIONS_8TH 4  // 8阶滤波器的二阶节数量
#define IIR_LOWPASS_SECTIONS_16TH 8 // 16阶滤波器的二阶节数量

// 100Hz采样率低通滤波器类型枚举
typedef enum {
    IIR_LOWPASS_100_015_HZ = 0,  // 0.15Hz截止频率
    IIR_LOWPASS_100_070_HZ,      // 0.7Hz截止频率
    IIR_LOWPASS_100_120_HZ,      // 1.2Hz截止频率
    IIR_LOWPASS_100_170_HZ,      // 1.7Hz截止频率
    IIR_LOWPASS_100_220_HZ,      // 2.2Hz截止频率
    IIR_LOWPASS_100_270_HZ,      // 2.7Hz截止频率
    IIR_LOWPASS_100_320_HZ,      // 3.2Hz截止频率
    IIR_LOWPASS_100_370_HZ,      // 3.7Hz截止频率
    IIR_LOWPASS_100_TYPE_COUNT   // 100Hz采样率滤波器类型总数
} iir_lowpass_100_filter_type_t;


// IIR低通滤波器结构体（8阶，级联4个2阶节），0.15Hz截止频率
typedef struct {
    float32_t x_delay[IIR_LOWPASS_SECTIONS_8TH][2];  // 各节的输入延迟线
    float32_t y_delay[IIR_LOWPASS_SECTIONS_8TH][2];  // 各节的输出延迟线
    bool is_initialized;                             // 初始化标志
} iir_lowpass_filter_8th_t;

// IIR低通滤波器结构体（16阶，级联8个2阶节）
typedef struct {
    float32_t x_delay[IIR_LOWPASS_SECTIONS_16TH][2];  // 各节的输入延迟线
    float32_t y_delay[IIR_LOWPASS_SECTIONS_16TH][2];  // 各节的输出延迟线
    bool is_initialized;                              // 初始化标志
} iir_lowpass_filter_16th_t;


// IIR低通滤波器结构体（16阶，8个二阶节，支持多种截止频率）
typedef struct {
    float32_t x_delay[IIR_LOWPASS_SECTIONS_16TH][2];  // 各节的输入延迟线
    float32_t y_delay[IIR_LOWPASS_SECTIONS_16TH][2];  // 各节的输出延迟线
    iir_lowpass_100_filter_type_t filter_type;        // 滤波器类型
    bool is_initialized;                              // 初始化标志
} iir_lowpass_filter_16th_100hz_t;

// 函数声明

// 8阶IIR滤波器函数（100Hz采样率，0.15Hz截止频率）
void iir_lowpass_filter_8th_init(iir_lowpass_filter_8th_t *filter);
float32_t iir_lowpass_filter_8th_process(iir_lowpass_filter_8th_t *filter, float32_t input);
void iir_lowpass_filter_8th_reset(iir_lowpass_filter_8th_t *filter);

// 16阶IIR滤波器函数（100Hz采样率，0.12Hz截止频率）
void iir_lowpass_filter_16th_init(iir_lowpass_filter_16th_t *filter);
float32_t iir_lowpass_filter_16th_process(iir_lowpass_filter_16th_t *filter, float32_t input);
void iir_lowpass_filter_16th_reset(iir_lowpass_filter_16th_t *filter);

// 16阶IIR低通滤波器函数（100Hz采样率，多种截止频率）
void iir_lowpass_filter_16th_100hz_init(iir_lowpass_filter_16th_100hz_t *filter, iir_lowpass_100_filter_type_t type);
float32_t iir_lowpass_filter_16th_100hz_process(iir_lowpass_filter_16th_100hz_t *filter, float32_t input);
void iir_lowpass_filter_16th_100hz_reset(iir_lowpass_filter_16th_100hz_t *filter);
void iir_lowpass_filter_16th_100hz_set_type(iir_lowpass_filter_16th_100hz_t *filter, iir_lowpass_100_filter_type_t type);

// 批量初始化和重置函数
void iir_lowpass_filter_16th_100hz_init_all(iir_lowpass_filter_16th_100hz_t filters[IIR_LOWPASS_100_TYPE_COUNT]);
void iir_lowpass_filter_16th_100hz_reset_all(iir_lowpass_filter_16th_100hz_t filters[IIR_LOWPASS_100_TYPE_COUNT]);

// 获取滤波器类型对应的截止频率
float32_t iir_lowpass_get_cutoff_freq_100hz(iir_lowpass_100_filter_type_t type);


#ifdef __cplusplus
}
#endif
#endif
