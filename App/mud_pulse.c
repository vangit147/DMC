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
 *          - 支持配置参数和状态管理
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
    pulse->data.curr_duration = pulse->data.tx_buffer[pulse->data.curr_index];

    __enable_irq();

    return 0;
}

// 更新led状态
void update_led_state()
{
    static uint8_t led;
    static uint32_t counter;

    counter++;
    //默认pulse->config.timer_hz = 100
    if (counter >= 100 / 10)
    {
        counter = 0;
        led ^= 1;
        PINS_DRV_WritePin(MUD_PULSE_PORT_LED, MUD_PULSE_PIN_LED, led);
    }
}

// 更新状态
void mud_pulse_update_state(mud_pulse_t *pulse)
{
    if (!pulse || pulse->state.tx_started == 0)
        return;

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
        pulse->data.curr_index = 0;
        pulse->data.curr_duration = INT_MAX;

        if (pulse->state.tx_request)
        {
            pulse->state.current_retry_count--;
            if (pulse->state.current_retry_count == 0)
                pulse->state.tx_request = 0;
            else
            {
                pulse->data.curr_index = 2;
                pulse->data.curr_duration = pulse->data.tx_buffer[pulse->data.curr_index];
            }
            if (pulse->state.remaining_groups != 0 && pulse->state.current_retry_count == 0)
            {
                // 延迟60s再发送下一次的retry_count组数据
                pulse->state.tx_request = 1;
                pulse->state.remaining_groups--;
                pulse->state.current_retry_count = pulse->config.max_retry_count;
                // 持续振动中，每组泥浆脉冲数据发送的时间间隔
                pulse->data.tx_buffer[pulse->data.curr_index] = (pulse->config.group_interval / 2) * pulse->config.timer_hz;
                pulse->data.tx_buffer[pulse->data.curr_index + 1] = (pulse->config.group_interval / 2) * pulse->config.timer_hz;
                pulse->data.curr_duration = pulse->data.tx_buffer[pulse->data.curr_index];
            }
        }
    }
    PINS_DRV_WritePin(MUD_PULSE_PORT, MUD_PULSE_PIN, pulse->data.curr_index & 0x1);
}

// 定时器中断处理
void mud_pulse_timer_isr(mud_pulse_t *pulse)
{
    if(get_downhole()==2)
        update_led_state();

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
static void mud_pulse_init_collect(mud_pulse_t *pulse)
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

// 初始化泥浆脉冲
void mud_pulse_init(mud_pulse_t *pulse)
{
    if (!pulse)
        return;

    // 从Flash读取配置参数
    uint32_t retry_count = is25pl032_flash_get_retry_count();
    uint32_t group_interval = is25pl032_flash_get_Pulse_group_interval();
    uint32_t send_delay = is25pl032_flash_get_Pulse_send_delay();
    uint32_t number_of_groups = is25pl032_flash_get_Number_of_pluse_group();
    uint32_t auto_send_period = is25pl032_flash_get_Pulse_auto_send();
    uint32_t static_collection_time = is25pl032_flash_get_Static_data_collection();

    // 检查配置合理性并设置
    pulse->config.timer_hz = 100; // 固定值，不需要从Flash读取

    // 检查重试次数
    if (retry_count >= 1 && retry_count <= 10)
    {
        pulse->config.max_retry_count = retry_count;
    }
    else
    {
        pulse->config.max_retry_count = 1; // 默认值
    }

    // 检查组间隔
    if (group_interval >= 60 && group_interval <= 600)
    {
        pulse->config.group_interval = group_interval;
    }
    else
    {
        pulse->config.group_interval = 60; // 默认值
    }

    // 检查发送延迟
    if (send_delay >= 60 && send_delay <= 600)
    {
        pulse->config.send_delay = send_delay;
    }
    else
    {
        pulse->config.send_delay = 60; // 默认值
    }

    // 检查脉冲组数量
    if (number_of_groups >= 1 && number_of_groups <= 5)
    {
        pulse->config.number_of_groups = number_of_groups;
    }
    else
    {
        pulse->config.number_of_groups = 3; // 默认值
    }

    // 检查定时发送时间
    if (auto_send_period >= 60 && auto_send_period <= 36000)
    {
        pulse->config.auto_send_period = auto_send_period;
    }
    else
    {
        pulse->config.auto_send_period = 900; // 默认值
    }

    // 检查静态数据采集时间
    if (static_collection_time >= 10 && static_collection_time <= 60)
    {
        pulse->config.static_collection_time = static_collection_time;
    }
    else
    {
        pulse->config.static_collection_time = 30; // 默认值
    }

    // 设置无振动时间默认值
    pulse->config.no_vibration_time = 60; // 默认值60秒

    // 初始化状态
    pulse->state.double_stage = 0;
    pulse->state.tx_request = 0;
    pulse->state.current_retry_count = 0;
    pulse->state.no_vibration_period = 0;
    pulse->state.period_counter = pulse->config.auto_send_period;
    pulse->state.last_motion_state = 0;
    pulse->state.remaining_groups = 0;
    pulse->state.tx_started = 0;

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

    uint32_t index = 0;
    float temp_voltage;
    inclination_hs_t hs;

    // 只获取静态后30秒内井斜、温度、高边、电压数据的平均值
    // 默认情况下，在每次振动前都会静止一分钟
    // 更新数据采集
    if (currentMotionState == 0 && pulse->collect.count < pulse->config.static_collection_time)
    {
        // 更新最小值
        if (interval_info.good_inc_avg < pulse->collect.min_ie)
            pulse->collect.min_ie = interval_info.good_inc_avg;
        if (sensor_data.t_C < pulse->collect.min_temp)
            pulse->collect.min_temp = sensor_data.t_C;
        get_inc_hs(&hs);
        if (hs.hs < pulse->collect.min_hs)
            pulse->collect.min_hs = hs.hs;
        temp_voltage = get_36V_voltage();
        if (temp_voltage < pulse->collect.min_voltage)
            pulse->collect.min_voltage = temp_voltage;

        // 更新平均值
        pulse->collect.avg_ie = (pulse->collect.avg_ie * pulse->collect.count + pulse->collect.min_ie) / (pulse->collect.count + 1);
        pulse->collect.avg_temp = (pulse->collect.avg_temp * pulse->collect.count + pulse->collect.min_temp) / (pulse->collect.count + 1);
        pulse->collect.avg_hs = (pulse->collect.avg_hs * pulse->collect.count + pulse->collect.min_hs) / (pulse->collect.count + 1);
        pulse->collect.avg_voltage = (pulse->collect.avg_voltage * pulse->collect.count + pulse->collect.min_voltage) / (pulse->collect.count + 1);
        pulse->collect.count++;
    }

    // 检查发送周期
    if (pulse->state.period_counter)
        pulse->state.period_counter--;
    else
    {
        // 默认情况下，在每次振动前都会静止一分钟，所以currentMotionState为1时，account只能为0
        if (currentMotionState == 0 && pulse->collect.count < 30)
            pulse->state.period_counter = 30 - pulse->collect.count;
        else
        {
            // 1. currentMotionState==0 && count == 30
            // 2. currentMotionState==1 && count == 0
            pulse->state.period_counter = pulse->config.auto_send_period;
            currentMotionState = 1;
        }
    }

    __disable_irq(); // 开始关键数据更新，禁用中断
    if (currentMotionState)
    {
        //! mud_pulse.tx_request 可以过滤掉震动后频繁的静止与震动
        if (!pulse->state.last_motion_state && !pulse->state.tx_request)
        {
            // 持续振动中，只会发送三组泥浆脉冲数据
            pulse->state.remaining_groups = pulse->config.number_of_groups;
            // 一组泥浆脉冲数据发送的次数
            pulse->state.current_retry_count = pulse->config.max_retry_count;
            // 振动后，连续的静止时间。若振动后连续静止的时间超过此数据，则认为目前静止了
            pulse->state.no_vibration_period = pulse->config.no_vibration_time;

            // 准备发送缓冲区
            // 开泵后等待泥浆脉冲压强准备好的时间
            pulse->data.tx_buffer[index++] = (pulse->config.send_delay / 2) * pulse->config.timer_hz;
            pulse->data.tx_buffer[index++] = (pulse->config.send_delay / 2) * pulse->config.timer_hz;
            pulse->data.curr_duration = pulse->data.tx_buffer[index - 2];
            pulse->state.tx_request = 1;

            if (pulse->collect.count == 30)
            {
                pulse->data.tx_buffer[index] = 34 * pulse->config.timer_hz;
                index += 2;
                pulse->data.tx_buffer[index] = 34 * pulse->config.timer_hz;
                index += 2;

                // 井斜
                float avg_ie = pulse->collect.avg_ie;
                if (avg_ie < 0.0f)
                    avg_ie = 0.0f;
                if (avg_ie > 180.0f)
                    avg_ie = 180.0f;
                pulse->data.tx_buffer[index] = 23 * pulse->config.timer_hz + (avg_ie * pulse->config.timer_hz) / 2;
                index += 2;

                // 温度
                float avg_temp = pulse->collect.avg_temp;
                if (avg_temp < 0.0f)
                    avg_temp = 0.0f;
                if (avg_temp > 180.0f)
                    avg_temp = 180.0f;
                pulse->data.tx_buffer[index] = 19 * pulse->config.timer_hz + 8.0f * pulse->config.timer_hz * avg_temp / 180.0f + 5;
                index += 2;

                // 高边
                float avg_hs = pulse->collect.avg_hs;
                pulse->data.tx_buffer[index] = 19 * pulse->config.timer_hz + 8.0f * pulse->config.timer_hz * (360.0f - avg_hs) / 360.0f;
                index += 2;

                // 电压
                float avg_voltage = pulse->collect.avg_voltage;
                if (avg_voltage < 20.0f)
                    avg_voltage = 20.0f;
                if (avg_voltage > 36.0f)
                    avg_voltage = 36.0f;
                pulse->data.tx_buffer[index] = 19 * pulse->config.timer_hz + pulse->config.timer_hz * (avg_voltage - 20.0f) / 2.0f + 5;
                index += 2;

                pulse->data.buffer_len = index;

                // 重置数据采集
                mud_pulse_init_collect(pulse);
            }
        }
    }
    else
    {
        if (!pulse->state.last_motion_state)
        {
            // 在当连续静止一分钟时，则认为已经处于非振动状态
            if (pulse->state.no_vibration_period == 0)
                pulse->state.remaining_groups = 0;
            else
                pulse->state.no_vibration_period--;
        }
        else
            pulse->state.no_vibration_period = pulse->config.no_vibration_time;
    }
    pulse->state.last_motion_state = currentMotionState;
    __enable_irq(); // 关键数据更新完成，启用中断
}
