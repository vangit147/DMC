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
#define FILTER_TAP_NUM 513  // 滤波器阶数:抽头数 = 阶数 + 1 = 512 + 1
#define NUM_FILTERS 9  // 总滤波器数量

// 滤波器类型枚举
typedef enum {
    FIR_LOW_50_512_200_CHEB = 0,    // 低通滤波器（200Hz，切比雪夫窗，512阶，陀螺仪）
    FIR_LOW_50_512_040_CHEB = 1,    // 低通滤波器（0.4Hz，切比雪夫窗，512阶，加速度计）
    FIR_BAND_50_512_025_120_CHEB = 2, //带通滤波器(0.25-1.2Hz, 切比雪夫窗，512阶,用于x/y轴加速度计滤波，旁瓣衰减80dB)
    FIR_BAND_50_512_070_170_CHEB = 3, //带通滤波器(0.7-1.7Hz, 切比雪夫窗，512阶,用于x/y轴加速度计滤波，旁瓣衰减100db)
    FIR_BAND_50_512_120_220_CHEB = 4, //带通滤波器(1.2-2.2Hz, 切比雪夫窗，512阶,用于x/y轴加速度计滤波，旁瓣衰减100db)
    FIR_BAND_50_512_170_270_CHEB = 5, //带通滤波器(1.7-2.7Hz, 切比雪夫窗，512阶,用于x/y轴加速度计滤波，旁瓣衰减100db)
    FIR_BAND_50_512_220_320_CHEB = 6, //带通滤波器(2.2-3.2Hz, 切比雪夫窗，512阶,用于x/y轴加速度计滤波，旁瓣衰减100db)
    FIR_BAND_50_512_270_370_CHEB = 7, //带通滤波器(2.7-3.7Hz, 切比雪夫窗，512阶,用于x/y轴加速度计滤波，旁瓣衰减100db)
    FIR_BAND_50_512_320_420_CHEB = 8 //带通滤波器(3.2-4.2Hz, 切比雪夫窗，512阶,用于x/y轴加速度计滤波，旁瓣衰减100db)
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
// 注释掉Z轴加速度计FIR滤波器实例，改用IIR滤波器
// extern arm_fir_instance_f32 fir_acc_z_instance;  // 加速度计Z轴实例
extern arm_fir_instance_f32 fir_acc_x_instance;  // 加速度计X轴实例
extern arm_fir_instance_f32 fir_acc_y_instance;  // 加速度计Y轴实例
//用中位数滤波+延时滤波代替Z轴陀螺仪FIR滤波器
//extern arm_fir_instance_f32 fir_gyro_z_instance; // 陀螺仪Z轴实例

// 公共系数矩阵
extern float32_t fir_shared_coeffs[FILTER_TAP_NUM];  // 所有滤波器共享的系数矩阵

// 状态缓冲区
// Z轴加速度计，改用IIR滤波器
extern float32_t fir_acc_x_state[FILTER_TAP_NUM + 3];//加速度计X状态缓冲
extern float32_t fir_acc_y_state[FILTER_TAP_NUM + 3];//加速度计Y状态缓冲
//用中位数滤波+延时滤波代替Z轴陀螺仪FIR滤波器
//extern float32_t fir_gyro_z_state[FILTER_TAP_NUM + 3];//陀螺仪Z状态缓冲

// 函数声明
void fir_init_all(void);                                // 初始化所有滤波器
void fir_switch_type(FIR_FilterType new_type);          // 动态切换滤波器类型
const float32_t *fir_get_coeffs(FIR_FilterType type);   // 获取滤波器系数的函数

#ifdef __cplusplus
}
#endif
#endif

