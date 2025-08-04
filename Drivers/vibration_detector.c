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
// 全局静态变量
uint32_t delta = 0;

// 测试用结构体（仅用于调试和测试，出厂时不需要）
#if ENABLE_DATA_INTEGRITY_VERIFICATION
vibration_data_integrity_t test_data_integrity;
#endif

#if ENABLE_TIME_MEASUREMENT
vibration_time_measurement_t test_time_measurement;
#endif

// 当前周期的时间测量全局变量（用于debug时清楚显示当前周期数据）
#if ENABLE_TIME_MEASUREMENT
static uint32_t current_cycle_total_processing_time_us = 0;  // 当前周期累计处理时间（微秒）
static uint32_t current_cycle_max_processing_time_us = 0;    // 当前周期最大处理时间（微秒）
static uint32_t current_cycle_min_processing_time_us = 0xFFFFFFFF; // 当前周期最小处理时间（微秒）
static uint32_t current_cycle_processing_count = 0;          // 当前周期处理次数
static uint32_t current_cycle_over_1ms_count = 0;           // 当前周期超过1ms次数
static uint32_t current_cycle_total_time_us = 0;             // 当前周期总时间（微秒）
#endif

// 子窗口统计变量 - 只保留条件3和条件4相关
float sub_sum_squared = VIBRATION_INIT_ZERO;
uint32_t sub_count = 0;
uint32_t rms_over_threshold_count = 0;

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
    if (vibration_detector.config.vibration_source == 0)
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
 * @Description: 更新当前周期时间统计
 * @Parameters : task_start_tick - 任务开始时间（FreeRTOS tick）
 * @RetValue   : 无
 * @Note       : 计算任务处理时间并更新当前周期的时间统计信息，包括累计时间、最大最小值等
 *******************************************************************************
 */
#if ENABLE_TIME_MEASUREMENT
static void update_current_cycle_time_stats(uint32_t task_start_tick)
{
    uint32_t task_end_tick, task_processing_ticks;
    uint32_t task_processing_time_ms;

    // 时间测量：记录任务处理结束时间并计算处理时间
    task_end_tick = xTaskGetTickCount();
    task_processing_ticks = task_end_tick - task_start_tick;
    task_processing_time_ms = (uint32_t)(task_processing_ticks * portTICK_PERIOD_MS);

    // 更新当前周期累计处理时间（微秒级别）
    // 将毫秒转换为微秒并累加到当前周期的总处理时间中
    current_cycle_total_processing_time_us += task_processing_time_ms * 1000;

    // 增加当前周期的处理次数统计
    current_cycle_processing_count++;

    // 更新当前周期最大处理时间（微秒级别）
    // 如果本次处理时间超过历史最大值，则更新最大值
    if (task_processing_time_ms * 1000 > current_cycle_max_processing_time_us) {
        current_cycle_max_processing_time_us = task_processing_time_ms * 1000;
    }

    // 更新当前周期最小处理时间（微秒级别）
    // 如果本次处理时间小于历史最小值，则更新最小值
    if (task_processing_time_ms * 1000 < current_cycle_min_processing_time_us) {
        current_cycle_min_processing_time_us = task_processing_time_ms * 1000;
    }

    // 统计超过1ms的处理次数
    // 用于分析系统性能，识别可能的性能瓶颈
    if (task_processing_time_ms >= 1) {
        current_cycle_over_1ms_count++;
    }
}
#endif

/**
 *******************************************************************************
 * @Description: 打印时间测量数据
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 打印当前周期的时间测量统计信息
 *******************************************************************************
 */
#if ENABLE_TIME_MEASUREMENT
static void print_time_measurement_data(void)
{
    printf("=== 振动检测任务时间测量数据（基于FreeRTOS tick）===\n");
    printf("理论周期时间: %lu 微秒\n", VIBRATION_DATA_SIZE * VIBRATION_DETECTION_FREQUENCY * 1000);
    printf("基于调用次数的理论时间: %lu 微秒\n", test_time_measurement.processing_time_count * VIBRATION_DETECTION_FREQUENCY * 1000);
    printf("实际周期时间: %lu 微秒\n", test_time_measurement.last_cycle_total_time_us);
    printf("时间偏差: %ld 微秒\n", test_time_measurement.time_deviation_us);
    printf("平均处理时间: %.2f 微秒\n", test_time_measurement.average_processing_time_us);
    printf("最大处理时间: %lu 微秒\n", test_time_measurement.max_processing_time_us);
    printf("最小处理时间: %lu 微秒\n", test_time_measurement.min_processing_time_us);
    printf("处理次数: %lu\n", test_time_measurement.processing_time_count);
    printf("大于等于1ms次数: %lu\n", test_time_measurement.over_1ms_count);
    printf("处理时间占周期比例: %.2f%%\n", test_time_measurement.processing_time_ratio_percent);
    printf("本周期累计处理时间: %lu 微秒\n", test_time_measurement.last_cycle_total_processing_time_us);
    printf("============================\n");
}
#endif

/**
 *******************************************************************************
 * @Description: 重置当前周期变量
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 重置当前周期的时间测量局部变量，为下一个周期做准备
 *******************************************************************************
 */
#if ENABLE_TIME_MEASUREMENT
static void reset_current_cycle_variables(void)
{
    // 重置当前周期累计处理时间
    current_cycle_total_processing_time_us = 0;

    // 重置当前周期最大处理时间
    current_cycle_max_processing_time_us = 0;

    // 重置当前周期最小处理时间（初始化为最大值，便于后续比较）
    current_cycle_min_processing_time_us = 0xFFFFFFFF;

    // 重置当前周期处理次数统计
    current_cycle_processing_count = 0;

    // 重置当前周期超过1ms的处理次数统计
    current_cycle_over_1ms_count = 0;

    // 重置当前周期总时间
    current_cycle_total_time_us = 0;
}
#endif

/**
 *******************************************************************************
 * @Description: 处理8秒周期结束时的统计
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 当完成一个完整的8秒周期时，更新全局统计并打印结果
 *******************************************************************************
 */
#if ENABLE_TIME_MEASUREMENT
static void process_cycle_completion(void)
{
    // 计算实际周期时间（基于系统时钟，微秒级别）
    // 获取当前系统时钟，计算与周期开始时间的差值
    uint32_t current_tick = xTaskGetTickCount();
    uint32_t cycle_ticks = current_tick - test_time_measurement.cycle_start_tick;
    // 将时钟节拍转换为微秒，得到实际周期时间
    current_cycle_total_time_us = (uint32_t)(cycle_ticks * portTICK_PERIOD_MS * 1000);

    // 将当前周期局部变量赋值给对应的全局变量
    // 保存当前周期的统计数据到全局结构体，供打印和分析使用
    test_time_measurement.last_cycle_total_time_us = current_cycle_total_time_us;
    test_time_measurement.last_cycle_total_processing_time_us = current_cycle_total_processing_time_us;
    test_time_measurement.max_processing_time_us = current_cycle_max_processing_time_us;
    test_time_measurement.min_processing_time_us = current_cycle_min_processing_time_us;
    test_time_measurement.processing_time_count = current_cycle_processing_count;
    test_time_measurement.over_1ms_count = current_cycle_over_1ms_count;

    // 计算平均处理时间（微秒）
    // 避免除零错误，当处理次数为0时，平均处理时间为0
    test_time_measurement.average_processing_time_us = (test_time_measurement.processing_time_count > 0) ?
        ((float)test_time_measurement.last_cycle_total_processing_time_us / test_time_measurement.processing_time_count) : 0.0f;

    // 计算处理时间占用周期比例（百分比）
    // 避免除零错误，当周期时间为0时，比例为0
    test_time_measurement.processing_time_ratio_percent = (test_time_measurement.last_cycle_total_time_us > 0) ?
        ((float)test_time_measurement.last_cycle_total_processing_time_us / test_time_measurement.last_cycle_total_time_us * 100.0f) : 0.0f;

    // 计算时间偏差（微秒）：实际周期时间与理论周期时间的差值
    // 理论周期时间 = 1600个数据点 * 5ms * 1000 = 8000000微秒
    test_time_measurement.time_deviation_us = (int32_t)test_time_measurement.last_cycle_total_time_us -
        (int32_t)(VIBRATION_DATA_SIZE * VIBRATION_DETECTION_FREQUENCY * 1000);

    // 打印时间测量数据（使用全局变量）
    // 输出当前周期的详细时间统计信息，便于调试和性能分析
    print_time_measurement_data();

    // 重置当前周期时间测量变量（为下一个周期做准备）
    // 清空当前周期的统计数据，为下一个8秒周期做准备
    reset_current_cycle_variables();

    // 重新开始计时
    // 记录下一个周期的开始时间，用于计算下一个周期的实际耗时
    test_time_measurement.cycle_start_tick = xTaskGetTickCount();
}
#endif

/**
 *******************************************************************************
 * @Description: 测试结构体初始化函数
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 初始化测试用结构体，确保正确的初始值
 *               此函数在系统启动时调用一次
 *******************************************************************************
 */
static void test_structures_init(void)
{
#if ENABLE_DATA_INTEGRITY_VERIFICATION
    // 初始化数据完整性验证结构体
    // 实际调用次数：记录当前周期实际执行的数据采集次数
    test_data_integrity.actual_call_count = 0;
    // 期望调用次数：理论应该执行的数据采集次数（固定为1600次）
    test_data_integrity.expected_call_count = VIBRATION_DATA_SIZE;
    // 数据采集错误总数：累计缺少或多余的数据点数量
    test_data_integrity.call_count_errors = 0;
    // 最小调用次数：历史记录中的最小调用次数（初始化为最大值，便于后续比较）
    test_data_integrity.min_call_count = 0xFFFFFFFF;
    // 最大调用次数：历史记录中的最大调用次数
    test_data_integrity.max_call_count = 0;
#endif

#if ENABLE_TIME_MEASUREMENT
    // 初始化时间测量结构体
    // 最大单次处理时间：当前周期内单次处理的最大时间（微秒）
    test_time_measurement.max_processing_time_us = 0;
    // 最小单次处理时间：当前周期内单次处理的最小时间（微秒，初始化为最大值）
    test_time_measurement.min_processing_time_us = 0xFFFFFFFF;
    // 处理时间统计次数：当前周期内已统计的处理次数
    test_time_measurement.processing_time_count = 0;
    // 周期开始时的系统时钟：记录当前8秒周期的开始时间
    test_time_measurement.cycle_start_tick = 0;
    // 上一个周期总时间：上一个完整8秒周期的实际耗时（微秒）
    test_time_measurement.last_cycle_total_time_us = 0;
    // 超过1ms的次数统计：当前周期内处理时间超过1ms的次数
    test_time_measurement.over_1ms_count = 0;
    // 当前8秒周期的累计处理时间：当前周期内所有处理时间的累计（微秒）
    test_time_measurement.last_cycle_total_processing_time_us = 0;
    // 平均处理时间：当前周期内单次处理的平均时间（微秒）
    test_time_measurement.average_processing_time_us = 0.0f;
    // 处理时间占用周期比例：处理时间占总周期时间的百分比
    test_time_measurement.processing_time_ratio_percent = 0.0f;
    // 时间偏差：实际周期时间与理论周期时间的差值（微秒）
    test_time_measurement.time_deviation_us = 0;
#endif
}

/**
 *******************************************************************************
 * @Description: 振动检测器配置初始化
 * @Parameters : detector - 振动检测器指针
 * @RetValue   : 无
 * @Note       : 初始化检测器的基本配置参数
 *               设置默认的检测状态和数据结构
 *               此函数在系统启动时调用一次
 *******************************************************************************
 */
void vibration_detector_config_init(vibration_detector_t *detector)
{
    if (!detector)
        return;

    // 初始化振动检测状态标志
    // 振动触发状态：0=未触发，1=已触发
    detector->state.is_triggered = 0;
    // 峰值检测标志：DISABLED=未启用，ENABLED=已启用 - 注释掉
    // detector->state.peak_flag = VIBRATION_FLAG_DISABLED;
    // 方差检测标志：DISABLED=未启用，ENABLED=已启用 - 注释掉
    // detector->state.variance_flag = VIBRATION_FLAG_DISABLED;
    // 差值检测标志：DISABLED=未启用，ENABLED=已启用
    detector->state.delta_flag = VIBRATION_FLAG_DISABLED;
    // RMS检测标志：DISABLED=未启用，ENABLED=已启用
    detector->state.rms_flag = VIBRATION_FLAG_DISABLED;
    // 平均值检测标志：DISABLED=未启用，ENABLED=已启用 - 注释掉
    // detector->state.avg_flag = VIBRATION_FLAG_DISABLED;

    // 初始化振动数据统计
    // 数据点计数：当前周期已采集的数据点数量
    detector->data.count = 0;
    // 最大值：当前周期内加速度的最大值 - 注释掉
    // detector->data.max = 0;
    // 最小值：当前周期内加速度的最小值 - 注释掉
    // detector->data.min = 0;
    // 峰峰值：当前周期内加速度的最大值与最小值之差 - 注释掉
    // detector->data.peak = 0;
    // 方差：当前周期内加速度的方差值 - 注释掉
    // detector->data.variance = 0;
    // 差值计数：当前周期内超过阈值的次数
    detector->data.delta = 0;
    // RMS值：当前周期内加速度的均方根值
    detector->data.rms = 0;
    // 平均值：当前周期内加速度的平均值 - 注释掉
    // detector->data.avg = 0;

    // 初始化检测状态机
    // 检测状态：初始状态
    detector->detection_state = DETECTION_STATE_INIT;
    // 上次触发状态：记录上一次的振动触发状态
    detector->last_trigger_state = 0;
    // 稳定计数：连续检测到振动的次数
    detector->stable_count = 0;
    // 无振动计数：连续未检测到振动的次数（用于滞后机制）
    detector->no_vibration_count = 0;
}

/**
 *******************************************************************************
 * @Description: 振动检测器初始化
 * @Parameters : detector - 振动检测器指针
 * @RetValue   : 无
 * @Note       : 从Flash读取配置参数，初始化数据结构
 *               每次都从Flash读取配置参数，保证上位机修改后立即生效
 *               此函数在每次检测周期开始时调用
 *******************************************************************************
 */
void vibration_detector_init(vibration_detector_t *detector)
{
    if (!detector)
        return;

    // 每次都从Flash读取配置参数，保证上位机修改后立即生效
    // 基础振动阈值：振动检测的基础阈值参数
    detector->config.threshold = is25pl032_flash_get_vibration_threshold();
    // 峰值阈值系数：峰值检测的阈值系数（相对于基础阈值的倍数）- 注释掉
    // detector->config.peak_threshold = is25pl032_flash_get_vibration_peak_threshold();
    // 方差阈值系数：方差检测的阈值系数（相对于基础阈值的倍数）- 注释掉
    // detector->config.variance_threshold = is25pl032_flash_get_vibration_variance_threshold();
    // 差值阈值系数：差值检测的阈值系数（相对于基础阈值的倍数）
    detector->config.delta_threshold = is25pl032_flash_get_vibration_delta_threshold();
    // RMS阈值系数：RMS检测的阈值系数（相对于基础阈值的倍数）
    detector->config.rms_threshold = is25pl032_flash_get_vibration_rms();
    // 平均值阈值系数：平均值检测的阈值系数（相对于基础阈值的倍数）- 注释掉
    // detector->config.avg_threshold = is25pl032_flash_get_vibration_avg_threshold();
    // NORM值：用于归一化处理的参数
    detector->config.norm = is25pl032_flash_get_norm();
    // 最小条件数量：触发振动检测所需满足的最小条件数量（1-5）
    detector->config.min_conditions_count = is25pl032_flash_get_vibration_min_conditions_count();
}

/**
 *******************************************************************************
 * @Description: 振动检测器数据处理
 * @Parameters : detector - 振动检测器指针
 * @RetValue   : 无
 * @Note       : 处理ADXL357传感器数据，进行振动检测
 *               每5ms调用一次，8秒（1600个数据点）进行一次完整检测
 *               使用双条件检测：频率检测（条件3）+ 振幅检测（条件4-RMS）
 *               频率检测：统计超过阈值的次数
 *               振幅检测：使用RMS值表示振动振幅
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

    detector->data.count++;
#if ENABLE_DATA_INTEGRITY_VERIFICATION
    test_data_integrity.actual_call_count++;
#endif

    // 计算触发次数 - 频率检测
    float diff = fabs(detector->config.threshold - current_value);
    if(diff > detector->config.delta_threshold) {
        delta++;
    }

    // 新增：每20个数据点的统计 - 只保留RMS计算
    sub_sum_squared += current_value * current_value;
    sub_count++;

    // 每20个数据点计算一次RMS值（用于振幅检测）
    if(sub_count >= VIBRATION_SUB_WINDOW_SIZE) {
        detector->data.rms = sqrtf(sub_sum_squared / VIBRATION_SUB_WINDOW_SIZE);

        // 检查是否超过阈值
        // 方差检测 - 注释掉
        /*
        if(detector->data.variance > detector->config.threshold * detector->config.variance_threshold) {
            variance_over_threshold_count++;
        }
        */

        // 平均值检测 - 注释掉
        /*
        if(fabs(detector->data.avg - detector->config.threshold) > detector->config.threshold * detector->config.avg_threshold) {
            avg_over_threshold_count++;
        }
        */

        // RMS检测 - 振幅检测
        if(fabs(detector->data.rms - detector->config.threshold) > detector->config.threshold * detector->config.rms_threshold) {
            rms_over_threshold_count++;
        }

        // 峰峰值检测 - 注释掉
        /*
        if(detector->data.peak > detector->config.threshold * detector->config.peak_threshold) {
            peak_over_threshold_count++;
        }
        */

        // 重置子窗口统计变量 - 只保留RMS相关
        sub_sum_squared = VIBRATION_INIT_ZERO;
        sub_count = 0;
    }

    // 每8秒（1600个数据点）进行一次完整的振动检测
    if(detector->data.count >= VIBRATION_DATA_SIZE) {
#if ENABLE_DATA_INTEGRITY_VERIFICATION
        // 验证数据采集完整性：计算少了多少个数据
        if (test_data_integrity.actual_call_count < detector->data.count) {
            test_data_integrity.call_count_errors += (detector->data.count - test_data_integrity.actual_call_count);
        } else if (test_data_integrity.actual_call_count > detector->data.count) {
            test_data_integrity.call_count_errors += (test_data_integrity.actual_call_count - detector->data.count);
        }

        // 更新统计信息
        if (test_data_integrity.actual_call_count < test_data_integrity.min_call_count) {
            test_data_integrity.min_call_count = test_data_integrity.actual_call_count;
        }
        if (test_data_integrity.actual_call_count > test_data_integrity.max_call_count) {
            test_data_integrity.max_call_count = test_data_integrity.actual_call_count;
        }
#endif

        detector->data.delta = delta;

        // 振动检测判断
        uint8_t trigger_conditions = 0; // 重置条件计数
        uint8_t current_trigger = 0;

        // 条件1：峰峰值超过阈值（基于80次子窗口计算，70%以上超过阈值）- 注释掉
        /*
        if(peak_over_threshold_count >= VIBRATION_SUB_WINDOW_THRESHOLD_COUNT) {
            detector->state.peak_flag = VIBRATION_FLAG_ENABLED;
            trigger_conditions++;
        }
        else
            detector->state.peak_flag = VIBRATION_FLAG_DISABLED;
        */

         // 条件2：方差超过阈值（基于80次子窗口计算，70%以上超过阈值）- 注释掉
        /*
        if(variance_over_threshold_count >= VIBRATION_SUB_WINDOW_THRESHOLD_COUNT) {
            detector->state.variance_flag = VIBRATION_FLAG_ENABLED;
            trigger_conditions++;
        }
        else
            detector->state.variance_flag = VIBRATION_FLAG_DISABLED;
        */

        // 条件3：频率检测 - 超过阈值的次数（使用动态阈值）
        if(detector->data.delta > VIBRATION_TRIGGER_COUNT_THRESHOLD_INT) {
            detector->state.delta_flag = VIBRATION_FLAG_ENABLED;
        }
        else
            detector->state.delta_flag = VIBRATION_FLAG_DISABLED;

        // 条件4：RMS超过阈值（基于80次子窗口计算，70%以上超过阈值）- 振幅检测
        if(rms_over_threshold_count >= VIBRATION_SUB_WINDOW_THRESHOLD_COUNT) {
            detector->state.rms_flag = VIBRATION_FLAG_ENABLED;
            trigger_conditions++;
        }
        else
            detector->state.rms_flag = VIBRATION_FLAG_DISABLED;

        // 条件5：平均值超过阈值（基于80次子窗口计算，70%以上超过阈值）- 注释掉
        /*
        if(avg_over_threshold_count >= VIBRATION_SUB_WINDOW_THRESHOLD_COUNT) {
            detector->state.avg_flag = VIBRATION_FLAG_ENABLED;
            trigger_conditions++;
        }
        else
            detector->state.avg_flag = VIBRATION_FLAG_DISABLED;
        */

        // 振动检测触发条件：频率检测（条件3）+ 振幅检测（条件4）
        // 需要同时满足频率和振幅两个条件
        if(detector->state.delta_flag == VIBRATION_FLAG_ENABLED && detector->state.rms_flag == VIBRATION_FLAG_ENABLED) {
            current_trigger = 1;
        } else {
            current_trigger = 0;
        }

        // 状态机处理（增加滞后机制，防止频繁切换）
        // 滞后机制：振动消失后需要等待下一个8秒周期结束才确认静止
        // 这样可以避免振动开关频繁切换导致的静态数据采集问题
        switch(detector->detection_state) {
            case DETECTION_STATE_INIT: // 初始状态
                if(current_trigger && !detector->last_trigger_state) {
                    // 从非触发到触发，进入等待确认状态
                    detector->detection_state = DETECTION_STATE_WAIT;
                    detector->stable_count = 1;
                    detector->state.is_triggered = 0; // 先不触发
                }
                break;

            case DETECTION_STATE_WAIT: // 等待确认状态
                if(current_trigger) {
                    detector->stable_count++;
                    if(detector->stable_count >= VIBRATION_STABLE_COUNT_THRESHOLD) { // 连续2次检测到振动
                        detector->detection_state = DETECTION_STATE_CONFIRMED;
                        detector->state.is_triggered = 1;
                    }
                } else {
                    // 振动消失，回到初始状态
                    detector->detection_state = DETECTION_STATE_INIT;
                }
                break;

            case DETECTION_STATE_CONFIRMED: // 确认状态
                if(!current_trigger) {
                    // 振动消失，开始滞后计数
                    detector->no_vibration_count++;
                    // 滞后一个完整的8秒周期才回到初始状态
                    // 需要等待下一个8秒周期结束才回到初始状态
                    if(detector->no_vibration_count >= VIBRATION_HYSTERESIS_COUNT) {
                        detector->detection_state = DETECTION_STATE_INIT;
                        detector->state.is_triggered = 0;
                        detector->no_vibration_count = 0; // 重置滞后计数
                    }
                } else {
                    // 重新检测到振动，重置滞后计数
                    detector->no_vibration_count = 0;
                }
                break;
        }

        // 保存当前触发状态供下次使用
        detector->last_trigger_state = current_trigger;

        // 重置全局变量和计数器，为下一个8秒周期做准备
        delta = 0;
        detector->data.count = 0;

#if ENABLE_DATA_INTEGRITY_VERIFICATION
        // 重置数据采集验证变量
        test_data_integrity.actual_call_count = 0;
#endif

        // 重置子窗口统计计数器 - 只保留RMS相关
        rms_over_threshold_count = 0;
    }
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
#if ENABLE_TIME_MEASUREMENT
    uint32_t task_start_tick;
#endif

    // 启动定时器前，先做一次配置初始化
    vibration_detector_config_init(&vibration_detector);

#if ENABLE_DATA_INTEGRITY_VERIFICATION || ENABLE_TIME_MEASUREMENT
    // 初始化测试结构体
    test_structures_init();
#endif

    // 首次从Flash读取配置参数，确保系统启动时配置正确
    vibration_detector_init(&vibration_detector);

    // 创建并立即启动定时器，减少启动延迟
    TimerHandle_t vibration_timer = xTimerCreate("vibration_timer", VIBRATION_DETECTION_FREQUENCY, 1, 0, vibration_timer_cb);
    xTimerStart(vibration_timer, 0); // 立即启动，不延迟

#if ENABLE_TIME_MEASUREMENT
    // 初始化时间测量标志
    static uint8_t first_timer_trigger = 1;
#endif

    for(;;)
    {
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if(notify & EVENT_VIBRATION_TIMER)
        {
#if ENABLE_TIME_MEASUREMENT
            // 第一次定时器触发时开始计时
            if(first_timer_trigger) {
                test_time_measurement.cycle_start_tick = xTaskGetTickCount();
                first_timer_trigger = 0;
            }

            // 时间测量：记录任务处理开始时间
            task_start_tick = xTaskGetTickCount();
#endif

            // 执行振动检测算法，处理传感器数据并更新检测结果
            vibration_detector_process(&vibration_detector);
            // 根据检测结果更新全局振动标志，供其他模块使用
            if(vibration_detector.state.is_triggered)
                s_u8_adxl357_vibrating_flag = 1;  // 检测到振动，设置标志
            else
                s_u8_adxl357_vibrating_flag = 0;  // 未检测到振动，清除标志

#if ENABLE_TIME_MEASUREMENT
            // 计算本次任务处理时间并更新当前8秒周期的时间统计
            // 包括累计处理时间、最大最小值、处理次数、超时次数等统计信息
            update_current_cycle_time_stats(task_start_tick);

            // 检查是否完成一个完整的8秒周期（1600次数据采集）
            // 当detector->data.count为0时，表示刚刚完成了一个完整的8秒周期
            if(vibration_detector.data.count == 0) {
                // 从Flash读取最新的振动检测配置参数，确保上位机修改后立即生效
                vibration_detector_init(&vibration_detector);

                // 完成8秒周期后的统计处理：计算实际周期时间、保存统计数据、打印结果、重置变量
                process_cycle_completion();
            }
#endif
        }
    }
}
