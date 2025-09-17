/**
  *****************************************************************************
  * FILENAME   : fir_config.h
  * COPYRIGHT  : ModingTech(ShangHai) Co.,Ltd2025.
  * CREATEDDATE: 2025.01.24 13:21:08 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * Gordon Li                  2025.01.24                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _FIR_CONFIG_H_2024_11_16_13_21_8_678_
#define _FIR_CONFIG_H_2024_11_16_13_21_8_678_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "arm_math.h"
#include "iir_lowpass_filter.h"
#include "iir_highpass_filter.h"

// 滤波器定义
#define FILTER_TAP_NUM 513  // 滤波器阶数:抽头数 = 阶数 + 1 = 512 + 1
#define NUM_FILTERS 9  // 总滤波器数量


// 100Hz采样率FIR低通滤波器类型枚举
typedef enum {
    FIR_LOW_100_512_010_CHEB = 0,    // 100Hz低通滤波器（0.1Hz，切比雪夫窗，512阶）
    FIR_LOW_100_512_070_CHEB = 1,    // 100Hz低通滤波器（0.7Hz，切比雪夫窗，512阶）
    FIR_LOW_100_512_120_CHEB = 2,    // 100Hz低通滤波器（1.2Hz，切比雪夫窗，512阶）
    FIR_LOW_100_512_170_CHEB = 3,    // 100Hz低通滤波器（1.7Hz，切比雪夫窗，512阶）
    FIR_LOW_100_512_220_CHEB = 4,    // 100Hz低通滤波器（2.2Hz，切比雪夫窗，512阶）
    FIR_LOW_100_512_270_CHEB = 5,    // 100Hz低通滤波器（2.7Hz，切比雪夫窗，512阶）
    FIR_LOW_100_512_320_CHEB = 6,    // 100Hz低通滤波器（3.2Hz，切比雪夫窗，512阶）
    FIR_LOW_100_512_370_CHEB = 7,    // 100Hz低通滤波器（3.7Hz，切比雪夫窗，512阶）
    FIR_LOW_100_512_420_CHEB = 8,    // 100Hz低通滤波器（4.2Hz，切比雪夫窗，512阶）
    FIR_LOW_100_512_TYPE_COUNT       // 100Hz采样率低通滤波器类型总数
} FIR_Lowpass_100_FilterType;


// 滤波器配置结构体
typedef struct {
    FIR_Lowpass_100_FilterType type;      // 滤波器类型
    const float32_t *coeffs;  // 滤波器系数指针
    uint32_t blockSize;       // 块大小
    uint32_t numTaps;         // 抽头数
} FIR_Config;

// 声明存储在flash中的系数数组
extern const float32_t * const fir_coeffs_flash[NUM_FILTERS];

//---------------- 独立资源声明（加速度Z轴和陀螺仪z轴）----------------
// 注释掉Z轴加速度计FIR滤波器实例，改用IIR滤波器
extern arm_fir_instance_f32 fir_acc_x_instance;  // 加速度计X轴实例
extern arm_fir_instance_f32 fir_acc_y_instance;  // 加速度计Y轴实例

// 公共系数矩阵
extern float32_t fir_shared_coeffs[FILTER_TAP_NUM];  // 所有滤波器共享的系数矩阵

// 状态缓冲区
extern float32_t fir_acc_x_state[FILTER_TAP_NUM + 3];//加速度计X状态缓冲
extern float32_t fir_acc_y_state[FILTER_TAP_NUM + 3];//加速度计Y状态缓冲


// 100Hz FIR滤波器相关声明
extern const float32_t *fir_get_100hz_coeffs(FIR_Lowpass_100_FilterType type);  // 获取100Hz滤波器系数的函数

// 函数声明
void fir_init_all(void);                                // 初始化所有滤波器
void fir_switch_type(FIR_Lowpass_100_FilterType new_type);          // 动态切换滤波器类型
const float32_t *fir_get_coeffs(FIR_Lowpass_100_FilterType type);   // 获取滤波器系数的函数

// 高通滤波器选择函数
iir_highpass_100_filter_type_t select_highpass_filter_type(float32_t rotation_freq);  // 根据旋转频率选择高通滤波器类型

// FIR低通滤波器选择函数
FIR_Lowpass_100_FilterType select_fir_lowpass_filter_type(float32_t rotation_freq);  // 根据旋转频率选择FIR低通滤波器类型

// FIR滤波器迟滞阈值函数
float32_t fir_get_upgrade_threshold(FIR_Lowpass_100_FilterType current_type);    // 获取升级阈值
float32_t fir_get_downgrade_threshold(FIR_Lowpass_100_FilterType current_type);  // 获取降级阈值
#ifdef __cplusplus
}
#endif
#endif

