/**
 ******************************************************************************
 * @file    vibration_detector.c
 * @author  Gordon Li
 * @version V1.0
 * @date    2024-12-19
 * @brief   振动检测器驱动实现
 *
 * @note    实现了基于ADXL357的振动检测功能
 *          - 支持多种检测模式
 *          - 提供滤波和数据处理
 *          - 支持配置参数管理
 ******************************************************************************
 */
#include "main.h"

#define EVENT_VIBRATION_TIMER    0X40000000
vibration_detector_t vibration_detector;
vibration_sensor_data_t adxl357_sensor_data;  // 当前传感器数据
static uint8_t s_u8_adxl357_vibrating_flag = 0;
static uint8_t s_u8_gpio_vibrating_flag = 0;  // GPIO振动检测标志
TaskHandle_t vibration_monitor_task_handle;

/**
 *******************************************************************************
 * @Description: 获取ADXL357振动标志
 * @Parameters : 无
 * @RetValue   : 振动标志（1-检测到振动，0-未检测到振动）
 * @Note       : 获取当前振动检测状态
 *               此函数返回全局振动标志，供其他模块查询振动状态
 *******************************************************************************
 */
uint32_t get_adxl357_vibrating_flag(void)
{
    return s_u8_adxl357_vibrating_flag;
}

/**
 *******************************************************************************
 * @Description: 获取GPIO振动标志
 * @Parameters : 无
 * @RetValue   : 振动标志（1-检测到振动，0-未检测到振动）
 * @Note       : 获取当前GPIO振动检测状态
 *               此函数返回GPIO振动标志，供其他模块查询振动状态
 *******************************************************************************
 */
uint32_t get_gpio_vibrating_flag(void)
{
    return s_u8_gpio_vibrating_flag;
}

/**
 *******************************************************************************
 * @Description: GPIO振动检测处理
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 处理GPIO振动检测，更新振动标志
 *               使用移位寄存器方式检测GPIO状态变化
 *******************************************************************************
 */
void checking_vibrating_gpio(void)
{
    static uint8_t s_u8_gpio_state;

    s_u8_gpio_state <<= 1;
    if (PINS_DRV_ReadPins(PTA) & (0x1 << 12))
        s_u8_gpio_state |= 1;
    if (s_u8_gpio_state == 0xff)
        s_u8_gpio_vibrating_flag = 1;
    else if (s_u8_gpio_state == 0)
        s_u8_gpio_vibrating_flag = 0;
}

/**
 *******************************************************************************
 * @Description: 获取统一振动标志
 * @Parameters : 无
 * @RetValue   : 振动标志（1-检测到振动，0-未检测到振动）
 * @Note       : 根据vibration_source参数选择振动检测源
 *               0=GPIO振动检测，1=ADXL357传感器振动检测
 *******************************************************************************
 */
uint32_t get_vibrating_flag(void)
{
    if (vibration_detector.config.vibration_source == VIBRATION_SOURCE_GPIO)
    {
        // 使用GPIO振动检测
        return s_u8_gpio_vibrating_flag;
    }
    else
    {
        // 使用ADXL357传感器振动检测
        return s_u8_adxl357_vibrating_flag;
    }
}

/**
 *******************************************************************************
 * @Description: 滑动窗口初始化
 * @Parameters : window - 滑动窗口指针
 * @RetValue   : 无
 * @Note       : 初始化滑动窗口的所有成员变量
 *               设置初始状态，为滑动窗口机制做准备
 *               使用分段存储方式实现8秒响应时间
 *******************************************************************************
 */
void vibration_sliding_window_init(vibration_sliding_window_t *window)
{
    if (!window)
        return;

    // 初始化滑动窗口状态
    window->segment_index = 0;            // 当前段索引
    window->data_index = 0;               // 当前段内数据索引
    window->total_count = 0;              // 总有效数据数量
    window->sum_squared = 0.0f;           // 平方和
    window->delta_count = 0;              // 差值计数
    window->rms_over_threshold_count = 0; // RMS超过阈值计数
    window->is_full = 0;                  // 窗口未满标志
    window->oldest_segment = 0;           // 最旧段索引

    // 初始化每段的统计信息
    for (uint32_t i = 0; i < VIBRATION_SLIDING_WINDOW_SEGMENTS; i++) {
        window->segment_delta_counts[i] = 0;
        window->segment_sum_squared[i] = 0.0f;
    }
}

/**
 *******************************************************************************
 * @Description: 滑动窗口数据更新（分段存储机制）
 * @Parameters : window - 滑动窗口指针
 *               new_value - 新的振动数据
 *               threshold - 振动阈值
 *               delta_threshold - 差值阈值
 *               rms_threshold - RMS阈值
 * @RetValue   : 无
 * @Note       : 更新滑动窗口数据，使用分段存储方式
 *               实现8秒滑动窗口的实时更新机制
 *               与GPIO振动开关响应时间保持一致：1600个数据点（约8秒）
 *******************************************************************************
 */
void vibration_sliding_window_update(vibration_sliding_window_t *window,
                                   float new_value,
                                   float threshold,
                                   float delta_threshold,
                                   float rms_threshold)
{
    if (!window)
        return;

    // 更新当前段的平方和统计
    window->segment_sum_squared[window->segment_index] += new_value * new_value;

    // 检查新数据是否超过差值阈值
    float new_diff = fabs(threshold - new_value);
    if (new_diff > delta_threshold) {
        window->segment_delta_counts[window->segment_index]++;
        window->delta_count++;
    }

    // 更新索引
    window->data_index++;
    if (window->data_index >= VIBRATION_SEGMENT_SIZE) {
        // 当前段已满，切换到下一段
        window->data_index = 0;
        window->segment_index = (window->segment_index + 1) % VIBRATION_SLIDING_WINDOW_SEGMENTS;

        // 如果窗口已满，需要移除最旧段的统计信息
        if (window->is_full) {
            window->oldest_segment = (window->oldest_segment + 1) % VIBRATION_SLIDING_WINDOW_SEGMENTS;

            // 移除最旧段的统计信息
            window->sum_squared -= window->segment_sum_squared[window->oldest_segment];
            window->delta_count -= window->segment_delta_counts[window->oldest_segment];
            window->total_count -= VIBRATION_SEGMENT_SIZE;  // 减去最旧段的数据点数

            // 重置最旧段的统计信息
            window->segment_sum_squared[window->oldest_segment] = 0.0f;
            window->segment_delta_counts[window->oldest_segment] = 0;
        }
    }

    // 更新总统计信息
    window->sum_squared += new_value * new_value;
    window->total_count++;

    // 检查窗口是否已满
    if (!window->is_full && window->total_count >= VIBRATION_SLIDING_WINDOW_SIZE) {
        window->is_full = 1;
    }

    // 计算当前RMS值
    float current_rms = 0.0f;
    if (window->total_count > 0) {
        current_rms = sqrtf(window->sum_squared / window->total_count);
    }

    // 检查RMS是否超过阈值
    if (fabs(current_rms - threshold) > threshold * rms_threshold) {
        window->rms_over_threshold_count++;
    } else {
        // 如果当前RMS不超过阈值，重置计数器
        window->rms_over_threshold_count = 0;
    }
}

/**
 *******************************************************************************
 * @Description: 滑动窗口振动检测（8秒响应时间）
 * @Parameters : window - 滑动窗口指针
 *               detector - 振动检测器指针
 * @RetValue   : 无
 * @Note       : 基于滑动窗口数据进行振动检测判断
 *               使用双条件检测：频率检测（差值计数）+ 振幅检测（RMS）
 *               需要同时满足频率和振幅两个条件
 *               与GPIO振动开关响应时间保持一致：1600个数据点窗口（约8秒）
 *               直接在函数内部设置振动状态
 *******************************************************************************
 */
void vibration_sliding_window_detect(vibration_sliding_window_t *window,
                                   vibration_detector_t *detector)
{
    if (!window || !detector)
        return;

    // 窗口未满时，不进行检测
    if (!window->is_full) {
        detector->state.is_triggered = 0;
        return;
    }

    // 条件1：频率检测 - 差值计数超过阈值（基于1600个数据点）
    uint8_t delta_condition = 0;
    if (window->delta_count > VIBRATION_TRIGGER_COUNT_THRESHOLD_INT) {
        detector->state.delta_flag = VIBRATION_FLAG_ENABLED;
        delta_condition = 1;
    } else {
        detector->state.delta_flag = VIBRATION_FLAG_DISABLED;
    }

    // 条件2：振幅检测 - RMS超过阈值（基于1600个数据点）
    uint8_t rms_condition = 0;
    if (window->rms_over_threshold_count >= VIBRATION_SUB_WINDOW_THRESHOLD_COUNT) {
        detector->state.rms_flag = VIBRATION_FLAG_ENABLED;
        rms_condition = 1;
    } else {
        detector->state.rms_flag = VIBRATION_FLAG_DISABLED;
    }

    // 振动检测触发条件：频率检测 + 振幅检测
    // 需要同时满足频率和振幅两个条件
    if (delta_condition && rms_condition) {
        detector->state.is_triggered = 1;  // 检测到振动
    } else {
        detector->state.is_triggered = 0;  // 未检测到振动
    }
}

/**
 *******************************************************************************
 * @Description: 振动检测器完整初始化
 * @Parameters : detector - 振动检测器指针
 * @RetValue   : 无
 * @Note       : 执行振动检测器的完整初始化流程
 *               包括：状态初始化、滑动窗口初始化、Flash参数读取
 *               此函数在定时器开始前调用一次，完成所有初始化工作
 *******************************************************************************
 */
void vibration_detector_init(vibration_detector_t *detector)
{
    if (!detector)
        return;

    // 第一步：初始化振动检测状态标志
    detector->state.is_triggered = 0;                    // 振动触发状态：0=未触发，1=已触发
    detector->state.delta_flag = VIBRATION_FLAG_DISABLED; // 差值检测标志：DISABLED=未启用，ENABLED=已启用
    detector->state.rms_flag = VIBRATION_FLAG_DISABLED;   // RMS检测标志：DISABLED=未启用，ENABLED=已启用

    // 第二步：初始化滑动窗口
    vibration_sliding_window_init(&detector->sliding_window);

    // 第三步：从Flash读取配置参数，确保系统启动时配置正确
    vibration_detector_load_config(detector);
}

/**
 *******************************************************************************
 * @Description: 振动检测器配置参数读取
 * @Parameters : detector - 振动检测器指针
 * @RetValue   : 无
 * @Note       : 从Flash读取最新的振动检测配置参数
 *               每次都从Flash读取配置参数，保证上位机修改后立即生效
 *               此函数可在运行时调用，用于更新配置参数
 *******************************************************************************
 */
void vibration_detector_load_config(vibration_detector_t *detector)
{
    if (!detector)
        return;

    // 从Flash读取最新的振动检测配置参数，保证上位机修改后立即生效
    detector->config.threshold = is25pl032_flash_get_vibration_threshold();        // 基础振动阈值
    detector->config.delta_threshold = is25pl032_flash_get_vibration_delta_threshold(); // 差值检测阈值
    detector->config.rms_threshold = is25pl032_flash_get_vibration_rms();          // RMS检测阈值
    detector->config.norm = is25pl032_flash_get_norm();                            // NORM值
    detector->config.vibration_source = is25pl032_flash_get_vibration_source();    // 振动检测源选择
}

/**
 *******************************************************************************
 * @Description: 振动检测器数据处理（滑动窗口机制）
 * @Parameters : detector - 振动检测器指针
 * @RetValue   : 无
 * @Note       : 处理ADXL357传感器数据，进行振动检测
 *               每5ms调用一次，使用8秒滑动窗口进行实时检测
 *               使用双条件检测：频率检测（差值计数）+ 振幅检测（RMS）
 *               频率检测：统计超过阈值的次数
 *               振幅检测：使用RMS值表示振动振幅
 *               与GPIO振动开关响应时间保持一致：1600个数据点窗口（约8秒）
 *               直接在滑动窗口检测函数中设置振动状态
 *******************************************************************************
 */
void vibration_detector_process(vibration_detector_t *detector)
{
    if (!detector) {
        return;
    }

    // ADXL357数据处理始终运行，无论选择哪种振动检测源
    // 这样可以进行数据对比和分析，通过上位机查看两种检测方式的差异
    adxl357_get_adc_data(&adxl357_sensor_data.x, &adxl357_sensor_data.y, &adxl357_sensor_data.z);

    // 计算当前加速度模长
    float current_value = sqrtf(adxl357_sensor_data.x * adxl357_sensor_data.x +
                               adxl357_sensor_data.y * adxl357_sensor_data.y +
                               adxl357_sensor_data.z * adxl357_sensor_data.z);

    // 更新滑动窗口数据
    vibration_sliding_window_update(&detector->sliding_window,
                                  current_value,
                                  detector->config.threshold,
                                  detector->config.delta_threshold,
                                  detector->config.rms_threshold);

    // 基于滑动窗口进行振动检测
    vibration_sliding_window_detect(&detector->sliding_window, detector);
}

// 定时器回调函数
static void vibration_timer_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(vibration_monitor_task_handle, EVENT_VIBRATION_TIMER, eSetBits, NULL);
}

// 振动监控任务
void VibrationMonitor_Task(void *pvParameters)
{
    uint32_t notify;

    // 执行振动检测器完整初始化
    vibration_detector_init(&vibration_detector);

    // 创建并立即启动定时器，减少启动延迟
    TimerHandle_t vibration_timer = xTimerCreate("vibration_timer", VIBRATION_DETECTION_FREQUENCY, 1, 0, vibration_timer_cb);
    xTimerStart(vibration_timer, 0); // 立即启动，不延迟

    for(;;)
    {
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if(notify & EVENT_VIBRATION_TIMER)
        {
            // 重新读取配置参数，确保上位机修改后立即生效
            vibration_detector_load_config(&vibration_detector);
            // 执行振动检测算法，处理传感器数据并更新检测结果
            vibration_detector_process(&vibration_detector);
            // 根据检测结果更新全局振动标志，供其他模块使用
            if(vibration_detector.state.is_triggered)
                s_u8_adxl357_vibrating_flag = 1;  // 检测到振动，设置标志
            else
                s_u8_adxl357_vibrating_flag = 0;  // 未检测到振动，清除标志
        }
    }
}
