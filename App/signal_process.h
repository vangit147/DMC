/**
 *****************************************************************************
 * FILENAME   : signal_process.h
 * COPYRIGHT  : MD.Tec(ShangHai) Co.,Ltd2024.
 * CREATEDDATE: 2025.04.06 22:04:20 Monday
 * DESCRIPTION: 传感器信号处理模块头文件
 *
 *
 * EDIT HISTORY:
 *   NAME                        DATE                      CONTENT
 * Gordon                       2025.05.05                    CREATE
 * Gordon               2025.06.30                  修改#include "../drivers/ads1278.h"为#include "../drivers/ads1278_imu.h"  
 * Gordon                       2025.08.27                    UPDATE

 *****************************************************************************
 */
#ifndef __SIGNAL_PROCESS_H__
#define __SIGNAL_PROCESS_H__

/* 系统头文件包含 */
#include <stdint.h>

/* 项目头文件包含 */
#include "IAM_20680HT.h"
#include "ads1278_imu.h"  //2025.06.30
//#include "ie_task.h"
#include "../common/fir_config.h"
#include "../common/iir_lowpass_filter.h"

/* 宏定义 */
// 角度转弧度系数 (π/180)，保留7位有效数字
#define DEG_TO_RAD 0.0174533f

// 带通滤波器中心频率定义
#define FIR_BAND_50_512_025_120_CENTER 0.8f  // 0.25-1.2Hz带通中心频率
#define FIR_BAND_50_512_070_170_CENTER 1.2f  // 0.7-1.7Hz带通中心频率
#define FIR_BAND_50_512_120_220_CENTER 1.7f  // 1.2-2.2Hz带通中心频率
#define FIR_BAND_50_512_170_270_CENTER 2.2f  // 1.7-2.7Hz带通中心频率
#define FIR_BAND_50_512_220_320_CENTER 2.7f  // 2.2-3.2Hz带通中心频率
#define FIR_BAND_50_512_270_370_CENTER 3.2f  // 2.7-3.7Hz带通中心频率
#define FIR_BAND_50_512_320_420_CENTER 3.7f  // 3.2-4.2Hz带通中心频率

// 低通滤波器截止频率定义
#define FIR_LOW_50_512_200_CHEB_CENTER 20.0f // 20Hz低通截止频率
#define FIR_LOW_50_512_040_CHEB_CENTER 0.4f // 0.4Hz低通截止频率
#define LPF_TO_BPF 0.3f // 低通向带通切换的边界频率
#define BPF_TO_LPF 0.3f // 带通向低通切换的边界频率

// 低通-带通切换迟滞
#define HYSTERESIS_HIGH 0.02f        // 低通向带通切换的迟滞
#define HYSTERESIS_LOW 0.05f         // 带通向低通切换的迟滞
#define HYSTERESIS_BAND 0.05f         // 带通滤波器之间的迟滞

#define BAND_SWITCH_THRESHOLD 0.25f        // 离开带通中心频率的距离阈值


/* 类型定义 */
// 传感器类型枚举
typedef enum {
    SIGNAL_PROCESS_ACC_HT20680 = 0,
    SIGNAL_PROCESS_ACC_VS10XX,
    SIGNAL_PROCESS_ACC_MINIQ,
    SIGNAL_PROCESS_ACC_TOTAL
} signal_process_acc_type_t;

typedef enum {
    SIGNAL_PROCESS_GYRO_HT20680 = 0,
    SIGNAL_PROCESS_GYRO_ADXRS645,
    SIGNAL_PROCESS_GYRO_TOTAL
} signal_process_gyro_type_t;

// 传感器信号结构体
typedef struct {
    float ax_raw;
    float ay_raw;
    float az_raw;
    float gx_raw;
    float gy_raw;
    float gz_raw;
    float t_raw;
    float ax_raw_fit;                    // ax轴:温度补偿后的原始值
    float ay_raw_fit;                    // ay轴:温度补偿后的原始值
    float az_raw_fit;                    // az轴:温度补偿后的原始值
    float gx_raw_fit;                    // gx轴:温度补偿后的原始值
    float gy_raw_fit;                    // gy轴:温度补偿后的原始值
    float gz_raw_fit;                    // gz轴:温度补偿后的原始值
    float ax_raw_offset;                 // ax轴:温度补偿+装配误差补偿后的x轴加速度原始值
    float ay_raw_offset;                 // ay轴:温度补偿+装配误差补偿后的y轴加速度原始值
    float az_raw_offset;                 // az轴:温度补偿+装配误差补偿后的z轴加速度原始值
    FIR_FilterType current_filter_type;  // 当前使用的滤波器类型
} sensor_signal_t;

// 滤波后的信号结构体
typedef struct {
    float ax_raw;
    float ay_raw;
    float az_raw;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_raw;
    float gy_raw;
    float gz_raw;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float t_C;

        // 新增：始终通过IIR低通滤波的ax和ay信号 25/08/27 Gordon Li
    float ax_raw_lpf;    // ax轴：始终通过IIR低通滤波的原始信号
    float ay_raw_lpf;    // ay轴：始终通过IIR低通滤波的原始信号
    float ax_g_lpf;      // ax轴：始终通过IIR低通滤波的g值信号
    float ay_g_lpf;      // ay轴：始终通过IIR低通滤波的g值信号
} filtered_signal_t;

/* 全局变量声明 */
extern sensor_signal_t sensor_signal;
extern filtered_signal_t filtered_sensor_signal;
extern TaskHandle_t signal_process_task_handle;

/* 函数声明 */
void signal_process_init(void);
void signal_process_fir_filter(void);
void signal_process_task(void *p);  // 添加任务函数声明

#endif /* __SIGNAL_PROCESS_H__ */ 
