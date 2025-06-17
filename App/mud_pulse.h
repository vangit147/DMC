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

// 泥浆脉冲配置结构体
typedef __packed struct
{
    uint32_t timer_hz;               // 定时器频率
    uint32_t no_vibration_time;      // 无振动时间
    uint32_t group_interval;         // 组间隔
    uint32_t send_delay;             // 发送延迟
    uint32_t max_retry_count;        // 最大重试次数
    uint32_t number_of_groups;       // 脉冲组数量
    uint32_t static_collection_time; // 静态数据采集时间
    uint32_t auto_send_period;       // 定时发送时间
} mud_pulse_config_t;

// 泥浆脉冲状态结构体
typedef __packed struct
{
    uint8_t double_stage;           // 双脉冲阶段
    uint8_t tx_request;             // 发送请求
    uint32_t current_retry_count;   // 当前重试计数
    uint32_t no_vibration_period;   // 无振动周期
    uint32_t period_counter;        // 周期计数
    uint8_t last_motion_state;      // 上次运动状态
    uint8_t remaining_groups;       // 剩余要发送的组数
    uint8_t tx_started;             // 发送启动标志
    uint8_t vibration_persist_flag; // 持续振动标志，振动中静止状态不会超过60s
} mud_pulse_state_t;

// 泥浆脉冲数据结构体
typedef struct
{
    int32_t tx_buffer[64];  // 发送缓冲区
    uint32_t buffer_len;    // 缓冲区长度
    uint32_t curr_index;    // 当前索引
    uint32_t curr_duration; // 当前持续时间
} mud_pulse_data_t;

// 泥浆脉冲数据采集结构体
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
    uint8_t count;     // 数据采集计数
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

// 更新泥浆脉冲数据
void mud_pulse_update_data(mud_pulse_t *pulse, uint8_t currentMotionState);

// 初始化泥浆脉冲数据采集
void mud_pulse_init_collect(mud_pulse_t *pulse);

// 设置泥浆脉冲数据
void mud_pulse_set_data(mud_pulse_t *pulse);

// 全局变量声明
extern mud_pulse_t mud_pulse;

#endif /* __MUD_PULSE_H */
