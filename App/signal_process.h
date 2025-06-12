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
 *****************************************************************************
 */
#ifndef __SIGNAL_PROCESS_H__
#define __SIGNAL_PROCESS_H__

/* 系统头文件包含 */
#include <stdint.h>

/* 项目头文件包含 */
#include "IAM_20680HT.h"
#include "ads1278-2.h"
//#include "ie_task.h"
#include "../common/fir_config.h"

/* 宏定义 */
// 角度转弧度系数 (π/180)，保留7位有效数字
#define DEG_TO_RAD 0.0174533f

// 带通滤波器中心频率定义
#define FIR_BAND_20_012_085_CENTER 0.4f  // 0.12-0.85Hz带通中心频率
#define FIR_BAND_20_025_110_CENTER 0.7f  // 0.25-1.10Hz带通中心频率
#define FIR_BAND_20_055_145_CENTER 1.0f  // 0.55-1.45Hz带通中心频率
#define FIR_BAND_20_085_175_CENTER 1.3f  // 0.85-1.75Hz带通中心频率
#define FIR_BAND_20_115_205_CENTER 1.6f  // 1.15-2.05Hz带通中心频率
#define FIR_BAND_20_145_235_CENTER 1.9f  // 1.45-2.35Hz带通中心频率
#define FIR_BAND_20_175_265_CENTER 2.2f  // 1.75-2.65Hz带通中心频率
#define FIR_BAND_20_205_295_CENTER 2.5f  // 2.05-2.95Hz带通中心频率
#define FIR_BAND_20_235_325_CENTER 2.8f  // 2.35-3.25Hz带通中心频率
#define FIR_BAND_20_265_355_CENTER 3.1f  // 2.65-3.55Hz带通中心频率
#define FIR_BAND_20_295_385_CENTER 3.4f  // 2.95-3.85Hz带通中心频率
#define FIR_BAND_20_325_415_CENTER 3.7f  // 3.25-4.15Hz带通中心频率
#define FIR_BAND_20_355_445_CENTER 4.0f  // 3.55-4.45Hz带通中心频率

// 低通滤波器截止频率定义
#define FIR_LOW_20_000_980_CENTER 9.8f   // 9.8Hz低通截止频率
#define FIR_LOW_20_030_045_CENTER 0.375f // 0.3-0.45Hz低通截止频率
#define FIR_LOW_20_050_BMW_CENTER 0.375f // 0.5Hz低通截止频率（布莱克曼窗）
#define FIR_LOW_20_010_KBW_CENTER 0.1f   // 0.1Hz低通截止频率（凯塞窗）

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
