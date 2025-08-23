#ifndef __ADXL357_H__
#define __ADXL357_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

// ADXL357振动标志宏定义
#define LOG_FLAG_VIBRATING_FOR_ADXL357      0x10     // ADXL357振动标志

typedef struct {
    float avg_vibration;
    float min_vibration;
    float max_vibration;
    float vibration;
    uint32_t account;

    // 新增：用于日志记录的ADXL357振动检测详细数据
    uint32_t delta_count_total;     // 日志周期内总差值计数
    uint32_t rms_over_count_total;  // 日志周期内总RMS超过次数
    float current_rms_value;        // 当前RMS值
    float max_delta_value_in_period; // 日志周期内差值最大值

    // 新增：ADXL357三轴加速度数据
    int32_t acc_x_raw;              // X轴加速度原始值（LSB单位）
    int32_t acc_y_raw;              // Y轴加速度原始值（LSB单位）
    int32_t acc_z_raw;              // Z轴加速度原始值（LSB单位）
    float acc_magnitude;            // 三轴加速度模值（LSB单位）
    float acc_x_g;                  // X轴重力加速度值（g单位）
    float acc_y_g;                  // Y轴重力加速度值（g单位）
    float acc_z_g;                  // Z轴重力加速度值（g单位）
} adxl357_vibration_data_t;

extern TaskHandle_t     adx357_task_handle;
extern void adxl357_task(void* p);

extern float g_norm;

// ============================================================================
// ADXL357 ODR配置值和对应的时间周期
// 这些宏定义建立了ODR寄存器值与时间周期的对应关系
// ============================================================================

// ADXL357 ODR配置值（Register 0x28, Bits[3:0]）
#define SET_ODR_4000        0x00        // 4000 Hz
#define SET_ODR_2000        0x01        // 2000 Hz
#define SET_ODR_1000        0x02        // 1000 Hz
#define SET_ODR_500         0x03        // 500 Hz
#define SET_ODR_250         0x04        // 250 Hz
#define SET_ODR_125         0x05        // 125 Hz
#define SET_ODR_62_5        0x06        // 62.5 Hz
#define SET_ODR_31_25       0x07        // 31.25 Hz
#define SET_ODR_15_625      0x08        // 15.625 Hz
#define SET_ODR_7_813       0x09        // 7.813 Hz
#define SET_ODR_3_906       0x0A        // 3.906 Hz

// ODR对应的时间周期（微秒）
#define ODR_PERIOD_4000     250         // 4000 Hz = 250us
#define ODR_PERIOD_2000     500         // 2000 Hz = 500us
#define ODR_PERIOD_1000     1000        // 1000 Hz = 1000us
#define ODR_PERIOD_500      2000        // 500 Hz = 2000us
#define ODR_PERIOD_250      4000        // 250 Hz = 4000us
#define ODR_PERIOD_125      8000        // 125 Hz = 8000us
#define ODR_PERIOD_62_5     16000       // 62.5 Hz = 16000us
#define ODR_PERIOD_31_25    32000       // 31.25 Hz = 32000us
#define ODR_PERIOD_15_625   64000       // 15.625 Hz = 64000us
#define ODR_PERIOD_7_813    128000      // 7.813 Hz = 128000us
#define ODR_PERIOD_3_906    256000      // 3.906 Hz = 256000us

// ============================================================================
// ODR配置与时间周期的对应关系宏
// 这些宏可以根据ODR配置值自动获取对应的时间周期
// ============================================================================

// 根据ODR配置值获取对应的时间周期（微秒）
#define GET_ODR_PERIOD(odr_value) \
    ((odr_value) == SET_ODR_4000 ? ODR_PERIOD_4000 : \
     (odr_value) == SET_ODR_2000 ? ODR_PERIOD_2000 : \
     (odr_value) == SET_ODR_1000 ? ODR_PERIOD_1000 : \
     (odr_value) == SET_ODR_500 ? ODR_PERIOD_500 : \
     (odr_value) == SET_ODR_250 ? ODR_PERIOD_250 : \
     (odr_value) == SET_ODR_125 ? ODR_PERIOD_125 : \
     (odr_value) == SET_ODR_62_5 ? ODR_PERIOD_62_5 : \
     (odr_value) == SET_ODR_31_25 ? ODR_PERIOD_31_25 : \
     (odr_value) == SET_ODR_15_625 ? ODR_PERIOD_15_625 : \
     (odr_value) == SET_ODR_7_813 ? ODR_PERIOD_7_813 : \
     (odr_value) == SET_ODR_3_906 ? ODR_PERIOD_3_906 : \
     ODR_PERIOD_1000)  // 默认值

// 根据ODR配置值获取对应的频率名称字符串
#define GET_ODR_NAME(odr_value) \
    ((odr_value) == SET_ODR_4000 ? "4000Hz" : \
     (odr_value) == SET_ODR_2000 ? "2000Hz" : \
     (odr_value) == SET_ODR_1000 ? "1000Hz" : \
     (odr_value) == SET_ODR_500 ? "500Hz" : \
     (odr_value) == SET_ODR_250 ? "250Hz" : \
     (odr_value) == SET_ODR_125 ? "125Hz" : \
     (odr_value) == SET_ODR_62_5 ? "62.5Hz" : \
     (odr_value) == SET_ODR_31_25 ? "31.25Hz" : \
     (odr_value) == SET_ODR_15_625 ? "15.625Hz" : \
     (odr_value) == SET_ODR_7_813 ? "7.813Hz" : \
     (odr_value) == SET_ODR_3_906 ? "3.906Hz" : \
     "1000Hz")  // 默认值

// ============================================================================
// ADXL357核心配置参数 - 只需修改这三个宏即可控制ADXL357振动检测系统
// 修改这三个参数后，所有相关配置会自动更新
// ============================================================================

// ================================================================================
// 【重要】ADXL357振动检测系统核心控制参数 - 修改这三个宏即可调整ADXL357振动检测系统行为
// ================================================================================

// 1. ADXL357采样频率配置
// 可选值：SET_ODR_4000(4000Hz), SET_ODR_2000(2000Hz), SET_ODR_1000(1000Hz),
//         SET_ODR_500(500Hz), SET_ODR_250(250Hz), SET_ODR_125(125Hz),
//         SET_ODR_62_5(62.5Hz), SET_ODR_31_25(31.25Hz), SET_ODR_15_625(15.625Hz),
//         SET_ODR_7_813(7.813Hz), SET_ODR_3_906(3.906Hz)
// 推荐值：SET_ODR_1000 (1000Hz) - 平衡响应性和功耗
#define ADXL357_SAMPLE_RATE_ODR      SET_ODR_1000    // 采样频率ODR配置

// 2. ADXL357 FIFO样本数配置（控制中断频率）
// 可选值：9, 18, 27, 36, 45, 54, 63, 72, 81, 90, 96 (必须是9的倍数)
// 限制：最大96字节（硬件限制）
// 推荐值：90字节（10组数据，每10ms中断）
#define ADXL357_SAMPLES_PER_READ    90      // FIFO样本数（字节）

// 3. ADXL357 FIFO清空次数限制配置
// 说明：当ADXL357重新使能中断时，为了获取到稳定的数据，等待指定次数的INT_MAP_FIFO_FULL中断后再获取数据
// 推荐值：50次（约0.5秒，基于1000Hz采样率和90字节FIFO配置）
// 计算公式：等待时间 = ADXL357_FIFO_CLEAR_LIMIT × (ADXL357_SAMPLES_PER_READ / 9) / ADXL357_SAMPLE_RATE_HZ
// 当前配置：50 × (90/9) / 1000 = 50 × 10 / 1000 = 0.5秒
#define ADXL357_FIFO_CLEAR_LIMIT    50      // FIFO清空次数限制

// ============================================================================
// 自动计算参数 - 无需手动修改，会根据上述三个参数自动计算
// ============================================================================

// 4. 采样频率数值（Hz）- 根据ADXL357_SAMPLE_RATE_ODR自动计算
#define ADXL357_SAMPLE_RATE_HZ       (1000000 / GET_ODR_PERIOD(ADXL357_SAMPLE_RATE_ODR))    // 自动计算采样频率数值

// 5. 数据组数计算（每次中断处理的数据组数）
#define ADXL357_DATA_GROUPS_PER_READ    ADXL357_SAMPLES_PER_READ / 9    // 每次读取的数据组数

// ============================================================================
// 配置说明和使用指南
// ============================================================================

// 【快速配置指南】
// 只需修改上述三个核心参数即可调整ADXL357振动检测系统：
//
// 1. 修改采样频率：更改 ADXL357_SAMPLE_RATE_ODR 的值
// 2. 修改中断频率：更改 ADXL357_SAMPLES_PER_READ 的值
// 3. 修改FIFO清空等待时间：更改 ADXL357_FIFO_CLEAR_LIMIT 的值
//
// 【配置示例】
// 高响应性配置：SET_ODR_1000 + 90字节 = 1000Hz采样，每10ms中断
// 低功耗配置：  SET_ODR_250  + 72字节 = 250Hz采样，每32ms中断
// 高精度配置：  SET_ODR_2000 + 90字节 = 2000Hz采样，每5ms中断
//
// 【自动连锁更新】
// 修改上述三个参数后，以下ADXL357振动检测系统相关参数会自动更新：
// - ADXL357_SAMPLE_RATE_HZ: 采样频率数值（Hz）
// - ADXL357_DATA_GROUPS_PER_READ: 数据组数
// - VIBRATION_DETECTION_FREQUENCY_MS: 振动检测频率
// - VIBRATION_SLIDING_WINDOW_SIZE: 滑动窗口大小
// - VIBRATION_SLIDING_WINDOW_SEGMENTS: 分段数量
//
// 【计算公式】
// 中断频率 = ADXL357_SAMPLES_PER_READ ÷ 9 ÷ ADXL357_SAMPLE_RATE_HZ × 1000ms
// 当前配置：90字节 ÷ 9字节/组 ÷ 1000Hz × 1000ms = 10ms中断周期

void Reset_Vibration_Stats(void);
void adxl357_reconfigure_interrupts(void);

#endif /* __ADXL357_H__ */

