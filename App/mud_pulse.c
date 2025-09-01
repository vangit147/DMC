/**
 ******************************************************************************
 * @file    mud_pulse.c
 * @author  Gordon Li
 * @version V1.15
 * @date    2025-04-05
 * @brief   泥浆脉冲控制模块实现
 *
 * @note
 *          - 实现了泥浆脉冲的发送控制
 *          - 支持动态脉冲定时发送和振动触发静态脉冲发送
 *          - 采用并列关系，避免相互阻塞
 *          - 提供完整的接口函数
 ******************************************************************************
 */
#include "mud_pulse.h"
#include "IS25LP032_flash.h"
#include "cpu.h"

// 声明外部变量
extern interval_info_t interval_info;
extern sensor_data_t sensor_data; // 用于存储传感器数据

// 全局变量定义
mud_pulse_t mud_pulse;

// 启动发送
int32_t mud_pulse_start_tx(mud_pulse_t *pulse)
{
    if (!pulse || pulse->data.buffer_len == 0)
        return -1;

    __disable_irq();

    pulse->state.tx_started = 1;
    pulse->data.curr_index = 0;
    PINS_DRV_WritePin(MUD_PULSE_PORT, MUD_PULSE_PIN, 0);
    PINS_DRV_WritePin(MUD_PULSE_PORT_LED, MUD_PULSE_PIN_LED, 0);
    // 统一发送缓冲区
    pulse->data.curr_duration = pulse->data.tx_buffer[pulse->data.curr_index];

    __enable_irq();

    return 0;
}

// 更新状态
void mud_pulse_update_state(mud_pulse_t *pulse)
{
    static uint8_t led;
    static uint32_t counter;

    if (!pulse || pulse->state.tx_started == 0)
        return;

    if(pulse->state.timer_triggered || pulse->state.tx_request)
    {
        counter++;
        // 默认pulse->config.timer_hz = 100
        if (counter >= 100 / 10)
        {
            counter = 0;
            led ^= 1;
            PINS_DRV_WritePin(MUD_PULSE_PORT_LED, MUD_PULSE_PIN_LED, led);
        }
    }

    /*当前脉冲未发送完毕*/
    if (pulse->data.curr_duration > 1)
    {
        pulse->data.curr_duration--;
        return;
    }

    // 发送下一个脉冲
    pulse->data.curr_index++;
    pulse->data.curr_duration = pulse->data.tx_buffer[pulse->data.curr_index];

    // 所有脉冲发送完毕
    if (pulse->data.curr_index == pulse->data.buffer_len)
    {
        pulse->state.double_stage = 0;
        // 发送完成后，立即将引脚设置为低电平，确保解码软件能正确解码最后一个脉冲
        // 这个低电平状态将在重传间隔期间保持不变
        pulse->data.curr_index = 0;

        if (pulse->state.timer_triggered)
            // 动态脉冲数据：不重传，直接完成
            // 重置动态脉冲定时发送标志（在完整发送完成后）
            pulse->state.timer_triggered = 0;

        if (pulse->state.tx_request)
        {
            // 静态脉冲数据：需要重传机制
            pulse->state.current_retry_count--;
            if (pulse->state.current_retry_count == 0)
                // 静态脉冲重传次数用完，发送完成
                pulse->state.tx_request = 0;
            else
                // 还有静态脉冲重传次数，启动重传间隔
                // 静态脉冲重传间隔：每次完整脉冲传输完毕后等待指定时间再重传
                pulse->data.curr_duration = pulse->config.retry_interval * pulse->config.timer_hz;
        }

        if (!pulse->state.timer_triggered && !pulse->state.tx_request)
        {
            pulse->data.curr_duration = INT_MAX;
            // 启动振动冷却期（防止立即重新触发）
            pulse->state.vibration_cooldown = MUD_PULSE_VIBRATION_COOLDOWN_SECONDS; // 振动冷却期（每秒递减1）
        }
    }
    // 更新引脚状态
    PINS_DRV_WritePin(MUD_PULSE_PORT, MUD_PULSE_PIN, pulse->data.curr_index & 0x1);
}

// 定时器中断处理
void mud_pulse_timer_isr(mud_pulse_t *pulse)
{
    if (!pulse)
        return;

    mud_pulse_update_state(pulse);
}

// 设置数据
void mud_pulse_set_data(mud_pulse_t *pulse)
{
    if (!pulse)
        return;

    __disable_irq();

    // 准备发送缓冲区
    uint32_t index = 0;
    pulse->data.tx_buffer[index++] = pulse->config.timer_hz * 1; // LOW
    pulse->data.tx_buffer[index++] = pulse->config.timer_hz * 1; // HIGH
    pulse->data.tx_buffer[index++] = pulse->config.timer_hz * 4; // LOW
    pulse->data.tx_buffer[index++] = pulse->config.timer_hz * 1; // HIGH

    // 设置双脉冲阶段
    pulse->state.double_stage = 1;

    // 设置高电平时间
    for (int i = 0; i < 16; i++)
        pulse->data.tx_buffer[i * 2 + 1] = pulse->config.timer_hz * 1;

    pulse->data.buffer_len = index;

    __enable_irq();
}

// 初始化泥浆脉冲数据采集
void mud_pulse_init_collect(mud_pulse_t *pulse)
{
    if (!pulse)
        return;

    // 初始化数据采集结构体
    pulse->collect.min_ie = INT_MAX;
    pulse->collect.min_temp = INT_MAX;
    pulse->collect.min_hs = INT_MAX;
    pulse->collect.min_voltage = INT_MAX;
    pulse->collect.avg_ie = 0.0f;
    pulse->collect.avg_temp = 0.0f;
    pulse->collect.avg_hs = 0.0f;
    pulse->collect.avg_voltage = 0.0f;
    pulse->collect.count = 0;
}

// 静态数据收集函数（以100ms为单位）
void mud_pulse_collect_static_data(mud_pulse_t *pulse, uint8_t is_static_state)
{
    if (!pulse)
        return;

    // 仅在静止状态下进行静态数据采集
    if (is_static_state && pulse->collect.count < pulse->config.static_collection_time * 10) // 100ms定时器调用，所以乘以10
    {
        // 前20秒（200次100ms调用）等待数据稳定，只计数
        if (pulse->collect.count > 200)
        {
            // 数据稳定后开始收集和计算
            // 更新最小值
            if (interval_info.good_inc_avg < pulse->collect.min_ie)
                pulse->collect.min_ie = interval_info.good_inc_avg;
            if (sensor_data.t_C < pulse->collect.min_temp)
                pulse->collect.min_temp = sensor_data.t_C;

            inclination_hs_t hs;
            get_inc_hs(&hs);
            if (hs.hs < pulse->collect.min_hs)
                pulse->collect.min_hs = hs.hs;

            float temp_voltage = get_36V_voltage();
            if (temp_voltage < pulse->collect.min_voltage)
                pulse->collect.min_voltage = temp_voltage;

            // 计算均值（从第201次100ms调用开始计算）
            uint32_t valid_count = pulse->collect.count - 200; // 有效数据计数（从1开始）
            pulse->collect.avg_ie = (pulse->collect.avg_ie * (valid_count - 1) + pulse->collect.min_ie) / valid_count;
            pulse->collect.avg_temp = (pulse->collect.avg_temp * (valid_count - 1) + pulse->collect.min_temp) / valid_count;
            pulse->collect.avg_hs = (pulse->collect.avg_hs * (valid_count - 1) + pulse->collect.min_hs) / valid_count;
            pulse->collect.avg_voltage = (pulse->collect.avg_voltage * (valid_count - 1) + pulse->collect.min_voltage) / valid_count;
        }
        pulse->collect.count++;
    }
}

// 初始化泥浆脉冲
void mud_pulse_init(mud_pulse_t *pulse)
{
    if (!pulse)
        return;

    // 从Flash读取所有配置参数
    uint32_t retry_count = is25pl032_flash_get_retry_count();
    uint32_t retry_interval = is25pl032_flash_get_retry_interval();
    uint32_t auto_send_period = is25pl032_flash_get_Pulse_auto_send();
    uint32_t static_collection_time = is25pl032_flash_get_Static_data_collection();

    // 设置固定值
    pulse->config.timer_hz = 100; // 固定值，不需要从Flash读取

    // 检查并设置静态脉冲数据重传次数
    if (retry_count >= 1 && retry_count <= 50)
    {
        pulse->config.max_retry_count = retry_count;
    }
    else
    {
        pulse->config.max_retry_count = MUD_PULSE_DEFAULT_MAX_RETRY_COUNT; // 默认值
    }

    // 检查并设置静态脉冲数据重传时间间隔
    if (retry_interval >= 10 && retry_interval <= 300)
    {
        pulse->config.retry_interval = retry_interval;
    }
    else
    {
        pulse->config.retry_interval = MUD_PULSE_DEFAULT_RETRY_INTERVAL; // 默认值
    }

    // 检查并设置动态脉冲数据周期性上传时间
    if (auto_send_period >= 300 && auto_send_period <= 3600)
    {
        pulse->config.auto_send_period = auto_send_period;
    }
    else
    {
        pulse->config.auto_send_period = MUD_PULSE_DEFAULT_AUTO_SEND_PERIOD; // 默认值（30分钟）
    }

    // 检查并设置静态脉冲数据采集时间
    if (static_collection_time >= 21 && static_collection_time <= 120)
    {
        pulse->config.static_collection_time = static_collection_time;
    }
    else
    {
        pulse->config.static_collection_time = MUD_PULSE_DEFAULT_STATIC_COLLECTION_TIME; // 默认值（1分钟）
    }

    // 初始化状态
    pulse->state.double_stage = 0;
    pulse->state.tx_request = 0;
    pulse->state.current_retry_count = 0;  // 静态脉冲重传计数
    pulse->state.period_counter = pulse->config.auto_send_period;  // 动态脉冲周期性发送计数
    pulse->state.vibration_cooldown = 0;
    pulse->state.tx_started = 0;
    pulse->state.timer_triggered = 0;  // 动态脉冲定时发送触发标志
    pulse->state.last_motion_state = 0;  // 初始化为静止状态

    // 初始化数据
    pulse->data.curr_duration = 0;
    pulse->data.buffer_len = 0;
    pulse->data.curr_index = 0;

    // 初始化数据采集
    mud_pulse_init_collect(pulse);

    // 初始化发送数据
    mud_pulse_set_data(pulse);
}

// 更新泥浆脉冲数据
void mud_pulse_update_data(mud_pulse_t *pulse, uint8_t currentMotionState)
{
    if (!pulse)
        return;

    __disable_irq(); // 开始关键数据更新，禁用中断

    // ===== 第二阶段：定时发送条件检查 =====
    // 检查发送周期
    if (pulse->state.period_counter)
        pulse->state.period_counter--;
    else
    {
        // 定时发送：检查三个条件
        // 1. 定时周期到了（已满足）
        // 2. 当前振动状态是振动
        // 3. 没有正在进行的发送（包括动态脉冲和静态脉冲）
        // 注意：动态脉冲发送可能被振动触发中断，优先级低于静态脉冲
        if (currentMotionState && !pulse->state.tx_request && !pulse->state.timer_triggered)
        {
            // 三个条件都满足，直接执行动态脉冲发送
            // --- 1. 设置发送参数 ---
            // 动态脉冲定时发送：不设置重传机制，直接发送一次

            // --- 2. 获取实时传感器数据 ---
            float real_time_ie = interval_info.good_inc_avg;
            float real_time_temp = sensor_data.t_C;
            inclination_hs_t real_time_hs;
            get_inc_hs(&real_time_hs);
            float real_time_voltage = get_36V_voltage();

            // --- 3. 数据范围限制（确保数据在有效范围内） ---
            if (real_time_ie < 0.0f) real_time_ie = 0.0f;
            if (real_time_ie > 180.0f) real_time_ie = 180.0f;
            if (real_time_temp < 0.0f) real_time_temp = 0.0f;
            if (real_time_temp > 180.0f) real_time_temp = 180.0f;
            if (real_time_voltage < 20.0f) real_time_voltage = 20.0f;
            if (real_time_voltage > 36.0f) real_time_voltage = 36.0f;

            // --- 4. 准备动态脉冲数据到动态缓冲区 ---
            uint32_t dynamic_index = 0;

            // 设置发送缓冲区前两位为200
            pulse->data.dynamic_data_buffer[dynamic_index] = 200;
            dynamic_index++;
            pulse->data.dynamic_data_buffer[dynamic_index] = 200;
            dynamic_index++;

            // 同步脉冲
            pulse->data.dynamic_data_buffer[dynamic_index] = 34 * pulse->config.timer_hz;
            dynamic_index += 2;
            pulse->data.dynamic_data_buffer[dynamic_index] = 34 * pulse->config.timer_hz;
            dynamic_index += 2;
            // 井斜（实时数据）
            pulse->data.dynamic_data_buffer[dynamic_index] = 23 * pulse->config.timer_hz + (real_time_ie * pulse->config.timer_hz) / 2;
            dynamic_index += 2;
            // 温度（实时数据）
            pulse->data.dynamic_data_buffer[dynamic_index] = 19 * pulse->config.timer_hz + 8.0f * pulse->config.timer_hz * real_time_temp / 180.0f;
            dynamic_index += 2;
            // 高边（实时数据）
            pulse->data.dynamic_data_buffer[dynamic_index] = 19 * pulse->config.timer_hz + 8.0f * pulse->config.timer_hz * (360.0f - real_time_hs.hs) / 360.0f;
            dynamic_index += 2;
            // 电压（实时数据）
            pulse->data.dynamic_data_buffer[dynamic_index] = 19 * pulse->config.timer_hz + pulse->config.timer_hz * (real_time_voltage - 20.0f) / 2.0f;
            dynamic_index += 2;

            // --- 5. 设置高电平时间 ---
            for (int i = 1; i < dynamic_index; i++)
                pulse->data.dynamic_data_buffer[i * 2 + 1] = pulse->config.timer_hz * 1;

            // --- 6. 复制动态脉冲数据到发送缓冲区 ---
            // 从第0位开始复制动态脉冲数据（前两位已在第四步设置为200）
            for (uint32_t i = 0; i < dynamic_index; i++)
            {
                pulse->data.tx_buffer[i] = pulse->data.dynamic_data_buffer[i];
            }
            pulse->data.buffer_len = dynamic_index;

            // --- 7. 启动动态脉冲发送 ---
            pulse->data.curr_index = 0;
            pulse->data.curr_duration = pulse->data.tx_buffer[0];
            pulse->state.timer_triggered = 1;  // 设置动态脉冲发送中标志
        }
        // 重置定时周期计数器（无论是否设置标志都要重置）
        pulse->state.period_counter = pulse->config.auto_send_period;
    }

    // ===== 第三阶段：振动冷却期更新 =====
    // 更新振动冷却计时器
    if (pulse->state.vibration_cooldown > 0)
    {
        pulse->state.vibration_cooldown--;
    }

    // ===== 第四阶段：发送执行（并列关系） =====
    // ===== 振动触发静态脉冲发送：独立于动态脉冲定时发送 =====

    // 检测由静止到振动的状态变化
    uint8_t motion_state_changed = (pulse->state.last_motion_state == 0 && currentMotionState == 1);

    // 更新上一次的运动状态
    pulse->state.last_motion_state = currentMotionState;

    if (motion_state_changed)
    {
        // --- 振动触发条件检查 ---
        // 振动触发静态脉冲发送优先级最高，需要检查三个条件：
        // 1. 检测到由静止到振动的状态变化（motion_state_changed为真）
        // 2. 振动冷却期结束（vibration_cooldown == 0）
        // 3. 没有静态脉冲正在发送（!tx_request）
        // 注意：静态脉冲数据采集完成后直接发送，会覆盖任何正在进行的动态脉冲发送
        if (!pulse->state.tx_request && pulse->state.vibration_cooldown == 0)
        {
            // --- 1. 检查静态脉冲数据是否采集完成 ---
            if (pulse->collect.count >= pulse->config.static_collection_time * 10) // 100ms定时器调用，所以乘以10
            {
                // --- 2. 设置发送参数 ---
                pulse->state.current_retry_count = pulse->config.max_retry_count;  // 静态脉冲数据重传次数

                // --- 3. 准备静态脉冲数据到静态缓冲区 ---
                uint32_t static_index = 0;

                // 设置发送缓冲区前两位为200
                pulse->data.static_data_buffer[static_index] = 200;
                static_index++;
                pulse->data.static_data_buffer[static_index] = 200;
                static_index++;

                // 同步脉冲
                pulse->data.static_data_buffer[static_index] = 34 * pulse->config.timer_hz;
                static_index += 2;
                pulse->data.static_data_buffer[static_index] = 34 * pulse->config.timer_hz;
                static_index += 2;

                // 井斜（静态脉冲数据）
                float avg_ie = pulse->collect.avg_ie;
                if (avg_ie < 0.0f) avg_ie = 0.0f;
                if (avg_ie > 180.0f) avg_ie = 180.0f;
                pulse->data.static_data_buffer[static_index] = 23 * pulse->config.timer_hz + (avg_ie * pulse->config.timer_hz) / 2;
                static_index += 2;

                // 温度（静态脉冲数据）
                float avg_temp = pulse->collect.avg_temp;
                if (avg_temp < 0.0f) avg_temp = 0.0f;
                if (avg_temp > 180.0f) avg_temp = 180.0f;
                pulse->data.static_data_buffer[static_index] = 19 * pulse->config.timer_hz + 8.0f * pulse->config.timer_hz * avg_temp / 180.0f;
                static_index += 2;

                // 高边（静态脉冲数据）
                float avg_hs = pulse->collect.avg_hs;
                pulse->data.static_data_buffer[static_index] = 19 * pulse->config.timer_hz + 8.0f * pulse->config.timer_hz * (360.0f - avg_hs) / 360.0f;
                static_index += 2;

                // 电压（静态脉冲数据）
                float avg_voltage = pulse->collect.avg_voltage;
                if (avg_voltage < 20.0f) avg_voltage = 20.0f;
                if (avg_voltage > 36.0f) avg_voltage = 36.0f;
                pulse->data.static_data_buffer[static_index] = 19 * pulse->config.timer_hz + pulse->config.timer_hz * (avg_voltage - 20.0f) / 2.0f;
                static_index += 2;

                // --- 4. 设置高电平时间 ---
                for (int i = 1; i < static_index; i++)
                    pulse->data.static_data_buffer[i * 2 + 1] = pulse->config.timer_hz * 1;

                // --- 5. 复制静态脉冲数据到发送缓冲区 ---
                // 从第0位开始复制静态脉冲数据（前两位已在第三步设置为200）
                for (uint32_t i = 0; i < static_index; i++)
                {
                    pulse->data.tx_buffer[i] = pulse->data.static_data_buffer[i];
                }
                pulse->data.buffer_len = static_index;

                // --- 6. 启动静态脉冲发送 ---
                pulse->data.curr_index = 0;
                pulse->data.curr_duration = pulse->data.tx_buffer[0];
                pulse->state.tx_request = 1;

            }
        }
        // --- 重置静态脉冲数据采集 ---
        // 只有在检测到由静止到振动的状态变化时，才重置数据采集
        // 这样可以避免在持续振动过程中重复重置数据采集
        mud_pulse_init_collect(pulse);
        // 如果静态脉冲数据未采集完成，继续采集但不发送
        // 这样可以确保振动检测立即响应，但发送需要等待数据准备完成
    }

    __enable_irq(); // 关键数据更新完成，启用中断
}
