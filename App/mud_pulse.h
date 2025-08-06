/**
 ******************************************************************************
 * @file    mud_pulse.h
 * @author  Gordon Li
 * @version V1.16
 * @date    2024-04-25
 * @brief   泥浆脉冲控制模块头文件
 *
 * @note    实现了泥浆脉冲的发送控制，支持配置参数和状态管理
 ******************************************************************************
 */
#ifndef __MUD_PULSE_H
#define __MUD_PULSE_H

#include <stdint.h>
#include "signal_process.h"
#include "ie_task.h"
#include "cpu.h"

// 泥浆脉冲引脚定义
#define MUD_PULSE_PORT_LED PTE
#define MUD_PULSE_PIN_LED 11
#define MUD_PULSE_PORT PTA
#define MUD_PULSE_PIN 13

// 泥浆脉冲配置宏定义
#define MUD_PULSE_VIBRATION_COOLDOWN_SECONDS 5  // 振动冷却期（秒）

// 泥浆脉冲配置默认值宏定义
#define MUD_PULSE_DEFAULT_RETRY_INTERVAL        30   // 默认静态脉冲数据重传时间间隔（秒）
#define MUD_PULSE_DEFAULT_MAX_RETRY_COUNT       1    // 默认静态脉冲数据重传次数
#define MUD_PULSE_DEFAULT_STATIC_COLLECTION_TIME 60  // 默认静态脉冲数据采集时间（秒）
#define MUD_PULSE_DEFAULT_AUTO_SEND_PERIOD      1800 // 默认动态脉冲数据周期性上传时间（秒）

// 泥浆脉冲配置结构体
typedef __packed struct
{
    uint32_t timer_hz;               // 定时器频率
    uint32_t max_retry_count;        // 静态脉冲数据重传次数
    uint32_t retry_interval;         // 静态脉冲数据重传时间间隔（秒）
    uint32_t static_collection_time; // 静态脉冲数据采集时间
    uint32_t auto_send_period;       // 动态脉冲数据周期性上传时间
} mud_pulse_config_t;

// 泥浆脉冲状态结构体
typedef __packed struct
{
    uint8_t double_stage;           // 双脉冲阶段
    uint8_t tx_request;             // 发送请求
    uint32_t current_retry_count;   // 当前静态脉冲重传计数
    uint32_t retry_interval_counter; // 静态脉冲重传间隔计时器
    uint32_t period_counter;        // 动态脉冲周期性发送计数
    uint8_t vibration_triggered;    // 振动触发标志
    uint32_t vibration_cooldown;    // 振动冷却计时器
    uint8_t tx_started;             // 发送启动标志
    uint8_t timer_triggered;        // 动态脉冲定时发送触发标志
} mud_pulse_state_t;

// 泥浆脉冲数据结构体
typedef struct
{
    int32_t tx_buffer[64];           // 统一发送缓冲区
    int32_t static_data_buffer[64];  // 静态脉冲数据准备缓冲区
    int32_t dynamic_data_buffer[64]; // 动态脉冲数据准备缓冲区
    uint32_t buffer_len;             // 缓冲区长度
    uint32_t curr_index;             // 当前索引
    uint32_t curr_duration;          // 当前持续时间
} mud_pulse_data_t;

// 静态脉冲数据采集结构体
typedef struct
{
    float min_ie;      // 最小井斜
    float min_temp;    // 最小温度
    float min_hs;      // 最小高边
    float min_voltage; // 最小电压
    float avg_ie;      // 平均井斜
    float avg_temp;    // 平均温度
    float avg_hs;      // 平均高边
    float avg_voltage; // 平均电压
    uint8_t count;     // 静态数据采集计数
} mud_pulse_data_collect_t;

// 泥浆脉冲控制结构体
typedef struct
{
    mud_pulse_config_t config;        // 配置参数
    mud_pulse_state_t state;          // 状态管理
    mud_pulse_data_t data;            // 数据管理
    mud_pulse_data_collect_t collect; // 数据采集
} mud_pulse_t;

// 初始化函数
void mud_pulse_init(mud_pulse_t *pulse);

// 启动发送
int32_t mud_pulse_start_tx(mud_pulse_t *pulse);

// 更新状态
void mud_pulse_update_state(mud_pulse_t *pulse);

// 定时器中断处理
void mud_pulse_timer_isr(mud_pulse_t *pulse);

// 更新泥浆脉冲数据（包括静态和动态脉冲）
void mud_pulse_update_data(mud_pulse_t *pulse, uint8_t currentMotionState);

// 初始化静态脉冲数据采集
void mud_pulse_init_collect(mud_pulse_t *pulse);

// 设置泥浆脉冲数据格式
void mud_pulse_set_data(mud_pulse_t *pulse);

// 全局变量声明
extern mud_pulse_t mud_pulse;

#endif /* __MUD_PULSE_H */
