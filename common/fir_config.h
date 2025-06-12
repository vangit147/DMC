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

// 滤波器定义
#define FILTER_TAP_NUM 257  // 滤波器阶数:抽头数 = 阶数 + 1 = 256 + 1
#define NUM_FILTERS 17  // 总滤波器数量

// 滤波器类型枚举
typedef enum {
    FIR_LOW_20_000_980 = 0,  // 低通滤波器（9.8Hz）
    FIR_LOW_20_030_045 = 1,  // 低通滤波器（0.3-0.45Hz）
    FIR_LOW_20_050_BMW = 2,  // 低通滤波器（0.5Hz，布莱克曼窗）
    FIR_LOW_20_010_KBW = 3,  // 低通滤波器（0.1Hz，凯泽窗）
    FIR_BAND_20_012_085 = 4, // 带通滤波器（0.12-0.85Hz）
    FIR_BAND_20_025_110 = 5, // 带通滤波器（0.25-1.10Hz）
    FIR_BAND_20_055_145 = 6, // 带通滤波器（0.55-1.45Hz）
    FIR_BAND_20_085_175 = 7, // 带通滤波器（0.85-1.75Hz）
    FIR_BAND_20_115_205 = 8, // 带通滤波器（1.15-2.05Hz）
    FIR_BAND_20_145_235 = 9, // 带通滤波器（1.45-2.35Hz）
    FIR_BAND_20_175_265 = 10,// 带通滤波器（1.75-2.65Hz）
    FIR_BAND_20_205_295 = 11,// 带通滤波器（2.05-2.95Hz）
    FIR_BAND_20_235_325 = 12,// 带通滤波器（2.35-3.25Hz）
    FIR_BAND_20_265_355 = 13,// 带通滤波器（2.65-3.55Hz）
    FIR_BAND_20_295_385 = 14,// 带通滤波器（2.95-3.85Hz）
    FIR_BAND_20_325_415 = 15,// 带通滤波器（3.25-4.15Hz）
    FIR_BAND_20_355_445 = 16,// 带通滤波器（3.55-4.45Hz）
} FIR_FilterType;

// 滤波器配置结构体
typedef struct {
    FIR_FilterType type;      // 滤波器类型
    const float32_t *coeffs;  // 滤波器系数指针
    uint32_t blockSize;       // 块大小
    uint32_t numTaps;         // 抽头数
} FIR_Config;

// 声明存储在flash中的系数数组
extern const float32_t * const fir_coeffs_flash[NUM_FILTERS];

//---------------- 独立资源声明（加速度Z轴和陀螺仪z轴）----------------
extern arm_fir_instance_f32 fir_acc_z_instance;  // 加速度计Z轴实例
extern arm_fir_instance_f32 fir_acc_x_instance;  // 加速度计X轴实例
extern arm_fir_instance_f32 fir_acc_y_instance;  // 加速度计Y轴实例
extern arm_fir_instance_f32 fir_gyro_z_instance; // 陀螺仪Z轴实例
extern arm_fir_instance_f32 fir_gyro_z_instance_for_compensation; // 陀螺仪Z轴标定用低通滤波实例
//extern arm_fir_instance_f32 fir_gyro_x_instance; // 陀螺仪X轴实例
//extern arm_fir_instance_f32 fir_gyro_y_instance; // 陀螺仪Y轴实例

// 公共系数矩阵
extern float32_t fir_shared_coeffs[FILTER_TAP_NUM];  // 所有滤波器共享的系数矩阵

// 状态缓冲区
extern float32_t fir_acc_z_state[FILTER_TAP_NUM + 3];//加速度计Z状态缓冲
extern float32_t fir_acc_x_state[FILTER_TAP_NUM + 3];//加速度计X状态缓冲
extern float32_t fir_acc_y_state[FILTER_TAP_NUM + 3];//加速度计Y状态缓冲
extern float32_t fir_gyro_z_state[FILTER_TAP_NUM + 3];//陀螺仪Z状态缓冲
//extern float32_t fir_gyro_z_state_for_compensation[FILTER_TAP_NUM + 3];//陀螺仪Z标定用低通滤波状态缓冲
//extern float32_t fir_gyro_x_state[FILTER_TAP_NUM + 3];//陀螺仪X状态缓冲
//extern float32_t fir_gyro_y_state[FILTER_TAP_NUM + 3];//陀螺仪Y状态缓冲

// 函数声明
void fir_init_all(void);                                // 初始化所有滤波器
void fir_switch_type(FIR_FilterType new_type);          // 动态切换滤波器类型
const float32_t *fir_get_coeffs(FIR_FilterType type);   // 获取滤波器系数的函数

#ifdef __cplusplus
}
#endif
#endif

