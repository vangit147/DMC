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
#include "../common/iir_highpass_filter.h"

/* 宏定义 */
// 角度转弧度系数 (π/180)，保留7位有效数字
#define DEG_TO_RAD 0.0174533f



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
    FIR_Lowpass_100_FilterType current_filter_type;  // 当前使用的低通滤波器类型
    iir_highpass_100_filter_type_t current_hpf_filter_type;  // 当前使用的高通滤波器类型
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
