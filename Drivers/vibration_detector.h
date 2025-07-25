/**
  *****************************************************************************
  * FILENAME   : vibration_detector.h
  * COPYRIGHT  : Copyright (c) 2024
  * DESCRIPTION: 振动检测器驱动头文件
  *
  * 功能说明：
  * - 基于ADXL357加速度计的振动检测
  * - 使用多条件实时检测（峰值、方差、计数、RMS、均值）
  * - 使用循环缓冲区优化内存使用
  * - 每8秒基于1600个数据点进行完整检测
  *
  * 振动检测配置参数和上位机命令对应关系：
  *
  * 【基础配置命令】
  * - VT=?: 获取振动阈值 - 基础振动检测阈值，推荐范围：0.5-1.5g
  * - VT=数值: 设置振动阈值 - 参数范围：0.1-5.0g，<50.0g验证
  * - VS=?: 获取振动检测源选择 - 返回值：0或1，0=GPIO，1=ADXL357
  * - VS=数值: 设置振动检测源选择 - 参数：0或1
  *
  * 【多条件检测参数命令】
  * - DP=?: 获取振动峰值检测阈值 - 推荐范围：0.2-1.0g
  * - DP=数值: 设置振动峰值检测阈值 - 参数范围：0.1-2.0g，<50.0g验证
  * - DV=?: 获取振动方差检测阈值 - 推荐范围：0.02-0.06
  * - DV=数值: 设置振动方差检测阈值 - 参数范围：0.01-0.1，<50.0g验证
  * - DT=?: 获取振动差值检测阈值 - 推荐范围：0.06-0.15g
  * - DT=数值: 设置振动差值检测阈值 - 参数范围：0.01-0.5g，<50.0g验证
  * - DR=?: 获取振动RMS检测阈值 - 推荐范围：0.15-0.5g
  * - DR=数值: 设置振动RMS检测阈值 - 参数范围：0.1-1.0g，<50.0g验证
  * - DA=?: 获取振动平均值检测阈值 - 推荐范围：0.03-0.1g
  * - DA=数值: 设置振动平均值检测阈值 - 参数范围：0.01-0.2g，<50.0g验证
  *
  * 【高级配置命令】
  * - VN=?: 获取NORM值 - 通常为49526.6797，用于三轴加速度合成计算
  * - VN=数值: 设置NORM值 - 参数：大于0的浮点数
  * - VC=?: 获取振动检测最小条件数量 - 返回值：1-5之间的整数
  * - VC=数值: 设置振动检测最小条件数量 - 参数范围：1-5，控制检测严格程度
  *
  * 【数据查询命令】
  * - VD=?: 获取振动综合数据 - 返回83字节振动数据包，包含配置、数据、状态等信息
  *
  * 应用场景说明：
  * - 软地层：降低各阈值，提高检测灵敏度
  * - 中等地层：使用标准设置
  * - 硬地层：提高各阈值，减少误报
  * - 调试设置：使用1个条件，最宽松设置
  * - 高精度检测：使用5个条件，最严格设置
  *****************************************************************************
  */
#ifndef __VIBRATION_DETECTOR_H
#define __VIBRATION_DETECTOR_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

// 振动检测配置参数
// 振动检测周期，单位s
#define VIBRATION_PERIOD 8
// 振动检测频率，单位ms
#define VIBRATION_DETECTION_FREQUENCY 5
// 窗口大小（用于滤波）- 暂时注释掉，防止以后使用
// #define WINDOW_SIZE 5  // 使用固定的小窗口大小，减少内存占用

// 数据个数常量（进一步优化内存使用）
#define VIBRATION_DATA_SIZE (VIBRATION_PERIOD * 1000 / VIBRATION_DETECTION_FREQUENCY)  // 总的数据个数：8秒 * 1000ms / 5ms = 1600个数据点

// 子窗口大小和计数常量（由振动周期和频率衍生）
#define VIBRATION_SUB_WINDOW_SIZE 20  // 子窗口大小：每20个数据点计算一次（100ms）
#define VIBRATION_SUB_WINDOW_COUNT (VIBRATION_DATA_SIZE / VIBRATION_SUB_WINDOW_SIZE)  // 子窗口数量：1600/20 = 80次

// 子窗口检测阈值比例（70%）
#define VIBRATION_SUB_WINDOW_THRESHOLD_RATIO 7
#define VIBRATION_SUB_WINDOW_THRESHOLD_DENOMINATOR 10

// 子窗口检测阈值计算宏（70%阈值）
#define VIBRATION_SUB_WINDOW_THRESHOLD_COUNT (VIBRATION_SUB_WINDOW_COUNT * VIBRATION_SUB_WINDOW_THRESHOLD_RATIO / VIBRATION_SUB_WINDOW_THRESHOLD_DENOMINATOR)

// 动态触发次数阈值（使用整数计算，避免浮点数宏定义问题）
// 基于1600个数据点的70%作为触发阈值
#define VIBRATION_TRIGGER_COUNT_THRESHOLD_INT ((VIBRATION_DATA_SIZE * VIBRATION_SUB_WINDOW_THRESHOLD_RATIO) / VIBRATION_SUB_WINDOW_THRESHOLD_DENOMINATOR)

// 默认参数定义（按照代码使用顺序排列）
#define DEFAULT_VIBRATION_THRESHOLD      1.0f    // 默认振动阈值 1.0g - 上位机命令: VT=?
                                                   // 推荐范围：0.5-1.5g，根据钻井环境调整
                                                   // 软地层：0.5-0.8g，中等地层：0.8-1.2g，硬地层：1.2-1.5g

#define VIBRATION_DEFAULT_PEAK_THRESHOLD    0.85f     // 默认峰值检测阈值 - 上位机命令: DP=?
                                                       // 推荐范围：0.2-1.0g，用于检测冲击振动
                                                       // 软地层：0.2-0.4g，中等地层：0.4-0.6g，硬地层：0.6-1.0g

#define VIBRATION_DEFAULT_VARIANCE_THRESHOLD 0.34f   // 默认方差检测阈值 - 上位机命令: DV=?
                                                      // 推荐范围：0.02-0.06，用于检测振动变化
                                                      // 软地层：0.02-0.06g，中等地层：0.06-0.1g，硬地层：0.1-0.15g

#define DEFAULT_VIBRATION_DELTA_THRESHOLD    0.2f    // 默认振动差值阈值 - 上位机命令: DT=?
                                                      // 推荐范围：0.06-0.15g，用于检测振动频率
                                                      // 软地层：0.06-0.1g，中等地层：0.1-0.12g，硬地层：0.12-0.15g

#define VIBRATION_DEFAULT_RMS_THRESHOLD     0.186f     // 默认RMS检测阈值 - 上位机命令: DR=?
                                                        // 推荐范围：0.15-0.5g，用于检测振动能量
                                                        // 软地层：0.15-0.25g，中等地层：0.25-0.35g，硬地层：0.35-0.5g

#define VIBRATION_DEFAULT_AVG_THRESHOLD     0.09f    // 默认平均值阈值 - 上位机命令: DA=?
                                                      // 推荐范围：0.03-0.1g，用于检测振动偏移
                                                      // 软地层：0.03-0.06g，中等地层：0.06-0.09g，硬地层：0.09-0.12g

#define VIBRATION_DEFAULT_SOURCE            0        // 默认使用GPIO振动检测 - 上位机命令: VS=?
                                                       // 0=GPIO振动检测（传统方式），1=ADXL357传感器振动检测（推荐方式）

#define DEFAULT_NORM_VALUE               49526.6797f  // 默认NORM值 - 上位机命令: VN=?
                                                       // 用于计算三轴加速度的合成值，通常不需要修改

#define VIBRATION_DEFAULT_MIN_CONDITIONS_COUNT 2     // 默认最小条件数量 - 上位机命令: VC=?
                                                       // 1=最宽松（调试），2=较宽松（软地层），3=标准（推荐），4=较严格（硬地层），5=最严格（高精度）

// 检测状态机状态定义
#define DETECTION_STATE_INIT       0  // 初始状态：等待振动触发
#define DETECTION_STATE_WAIT       1  // 等待确认状态：已检测到潜在振动，等待确认
#define DETECTION_STATE_CONFIRMED  2  // 确认振动状态：振动已确认

// 振动检测状态机相关常量
#define VIBRATION_STABLE_COUNT_THRESHOLD    2        // 连续检测到振动的次数阈值
#define VIBRATION_HYSTERESIS_COUNT          2        // 滞后计数阈值：2个8秒周期 = 16秒（振动消失后等待下一个周期）
#define VIBRATION_MAX_INIT_VALUE            -999999.0f  // 最大值初始化值
#define VIBRATION_MIN_INIT_VALUE            999999.0f   // 最小值初始化值

// 振动检测状态标志常量
#define VIBRATION_FLAG_DISABLED             0.0f     // 标志禁用状态
#define VIBRATION_FLAG_ENABLED              1.0f     // 标志启用状态

// 振动检测初始化常量
#define VIBRATION_INIT_ZERO                 0.0f     // 初始化零值

// 传感器数据结构体
// 用于存储ADXL357三轴加速度计的原始数据
typedef struct
{
    float x;  // X轴加速度值
    float y;  // Y轴加速度值
    float z;  // Z轴加速度值
} vibration_sensor_data_t;

// 振动检测配置结构体
// 存储振动检测的各种配置参数，支持上位机动态修改
// 所有参数都支持通过UART命令实时修改，修改后立即生效
typedef __packed struct
{
    float threshold;      // 振动阈值：基础振动检测阈值，作为其他参数的基准值
                         // 推荐范围：0.5-1.5g，根据钻井环境调整
                         // 上位机命令：VT=?（查询），VT=数值（设置）

    uint8_t vibration_source;     // 振动检测源选择：0-GPIO振动检测，1-ADXL357传感器振动检测
                                  // 用于控制振动检测的源选择，影响所有使用振动检测的功能
                                  // 0=GPIO振动检测（传统方式），1=ADXL357传感器振动检测（推荐方式）
                                  // 上位机命令：VS=?（查询），VS=数值（设置）

    // 各检测条件的专用阈值参数
    float peak_threshold;      // 峰值检测阈值：峰峰值超过此阈值时触发
                               // 用于检测冲击振动，数值越大检测越严格
                               // 推荐范围：0.2-1.0g，上位机命令：DP=?（查询），DP=数值（设置）

    float variance_threshold;  // 方差检测阈值：方差超过此阈值时触发
                               // 用于检测振动变化，数值越大检测越严格
                               // 推荐范围：0.02-0.06，上位机命令：DV=?（查询），DV=数值（设置）

    float delta_threshold;     // 差值检测阈值：与基准值的差值超过此阈值时触发
                               // 用于检测振动频率，数值越大检测越严格
                               // 推荐范围：0.06-0.15g，上位机命令：DT=?（查询），DT=数值（设置）

    float rms_threshold;       // RMS检测阈值：RMS值超过此阈值时触发
                               // 用于检测振动能量，数值越大检测越严格
                               // 推荐范围：0.15-0.5g，上位机命令：DR=?（查询），DR=数值（设置）

    float avg_threshold;       // 平均值检测阈值：平均值超过此阈值时触发
                               // 用于检测振动偏移，数值越大检测越严格
                               // 推荐范围：0.03-0.1g，上位机命令：DA=?（查询），DA=数值（设置）

    float norm;                // ADXL357三轴加速度模长NORM值
                               // 用于计算三轴加速度的合成值，通常为49526.6797
                               // 上位机命令：VN=?（查询），VN=数值（设置）

    uint8_t min_conditions_count;  // 最小条件数量：至少满足的条件数量（1-5）
                                   // 控制振动检测的严格程度，数值越大检测越严格
                                   // 1个条件：最宽松，容易误报，仅用于调试
                                   // 2个条件：较宽松，适合软地层钻井
                                   // 3个条件：标准设置，推荐用于正常钻井
                                   // 4个条件：较严格，适合关键检测或硬地层
                                   // 5个条件：最严格，几乎无误报，适合高精度检测
                                   // 上位机命令：VC=?（查询），VC=数值（设置）
} vibration_config_t;

// 振动检测状态结构体
// 存储振动检测的状态标志，用于多条件检测
typedef __packed struct
{
    uint8_t is_triggered;      // 触发标志：1-已触发振动检测，0-未触发
    float peak_flag;           // 峰值标志：1-峰值条件满足，0-不满足
    float variance_flag;       // 方差标志：1-方差条件满足，0-不满足
    float delta_flag;          // 差值标志：1-差值条件满足，0-不满足
    float rms_flag;            // RMS标志：1-RMS条件满足，0-不满足
    float avg_flag;            // 平均值标志：1-平均值条件满足，0-不满足
} vibration_state_t;

// 振动检测数据结构体
// 存储振动检测的原始数据、统计数据和计算结果
typedef struct
{
    uint32_t count;                // 数据计数：当前已收集的数据点数量
    float max;                     // 最大值：当前检测周期内的最大值
    float min;                     // 最小值：当前检测周期内的最小值
    float peak;                    // 峰峰值：最大值与最小值的差值
    float variance;                // 方差：数据的离散程度
    uint32_t delta;                // 差值计数：与基准值的差值统计
    float rms;                     // 均方根值：数据的有效值
    float avg;                     // 平均值：数据的算术平均值
} vibration_data_t;

/*******************************************************/
/* 调试和测试功能控制宏定义 */
/*******************************************************/

// 数据完整性验证功能控制
// 设置为1启用数据完整性验证，设置为0禁用（出厂时建议禁用）
#define ENABLE_DATA_INTEGRITY_VERIFICATION    0

// 时间测量功能控制
// 设置为1启用时间测量功能，设置为0禁用（出厂时建议禁用）
#define ENABLE_TIME_MEASUREMENT               0

// 数据完整性验证结构体
// 用于验证振动数据采集的完整性和准确性
typedef struct
{
    uint32_t actual_call_count;    // 实际调用次数：当前周期实际执行的数据采集次数
    uint32_t expected_call_count;  // 期望调用次数：理论应该执行的数据采集次数
    uint32_t call_count_errors;    // 数据采集错误总数：累计缺少或多余的数据点数量
    uint32_t min_call_count;       // 最小调用次数：历史记录中的最小调用次数
    uint32_t max_call_count;       // 最大调用次数：历史记录中的最大调用次数
} vibration_data_integrity_t;

// 时间测量结构体
// 用于测量和统计振动检测任务的时间性能
typedef struct
{
    uint32_t max_processing_time_us;      // 最大单次处理时间（微秒）：当前周期内单次处理的最大时间
    uint32_t min_processing_time_us;      // 最小单次处理时间（微秒）：当前周期内单次处理的最小时间
    uint32_t processing_time_count;       // 处理时间统计次数：当前周期内已统计的处理次数
    uint32_t cycle_start_tick;           // 周期开始时的系统时钟：记录当前8秒周期的开始时间
    uint32_t last_cycle_total_time_us;   // 上一个周期总时间（微秒）：上一个完整8秒周期的实际耗时
    uint32_t over_1ms_count;             // 超过1ms的次数统计：当前周期内处理时间超过1ms的次数
    uint32_t last_cycle_total_processing_time_us;  // 当前8秒周期的累计处理时间（微秒）：当前周期内所有处理时间的累计
    float average_processing_time_us;      // 平均处理时间（微秒）：当前周期内单次处理的平均时间
    float processing_time_ratio_percent;  // 处理时间占用周期比例（百分比）：处理时间占总周期时间的百分比
    int32_t time_deviation_us;           // 时间偏差（微秒）：实际周期时间与理论周期时间的差值
} vibration_time_measurement_t;

// 振动检测控制结构体
// 整合配置、状态、数据，形成完整的振动检测器
typedef struct
{
    vibration_config_t config;     // 配置参数：存储所有检测配置
    vibration_state_t state;       // 状态管理：存储检测状态和标志
    vibration_data_t data;         // 数据管理：存储原始数据和统计结果
    uint8_t detection_state;      // 检测状态机状态：当前处于哪个检测阶段
    uint8_t last_trigger_state;   // 上一次的触发状态：用于状态机判断
    uint32_t stable_count;        // 稳定计数：连续检测到振动的次数
    uint32_t no_vibration_count;  // 无振动计数：用于滞后机制，防止频繁切换
} vibration_detector_t;

/*******************************************************/

/* 函数声明 */

/**
 * @brief 获取ADXL357振动标志
 * @return 振动标志：1-检测到振动，0-未检测到振动
 */
uint32_t get_adxl357_vibrating_flag(void);

/**
 * @brief 获取GPIO振动标志
 * @return 振动标志：1-检测到振动，0-未检测到振动
 */
uint32_t get_gpio_vibrating_flag(void);

/**
 * @brief GPIO振动检测处理
 * @note 处理GPIO振动检测，更新振动标志
 */
void checking_vibrating_gpio(void);

/**
 * @brief 获取统一振动标志
 * @return 振动标志：1-检测到振动，0-未检测到振动
 * @note 根据vibration_source参数选择振动检测源
 *       0=GPIO振动检测，1=ADXL357传感器振动检测
 */
uint32_t get_vibrating_flag(void);

/**
 * @brief 振动检测器初始化
 * @param detector 振动检测器指针
 * @note 从Flash读取配置参数，初始化数据结构
 */
void vibration_detector_init(vibration_detector_t *detector);

/**
 * @brief 振动检测器配置初始化
 * @param detector 振动检测器指针
 * @note 初始化检测器的基本配置参数
 */
void vibration_detector_config_init(vibration_detector_t *detector);

/**
 * @brief 振动检测器数据处理
 * @param detector 振动检测器指针
 * @note 处理ADXL357传感器数据，进行振动检测
 *       每5ms调用一次，8秒（1600个数据点）进行一次完整检测
 *       使用多条件检测：峰值、方差、计数、RMS、均值
 *       注意：时间测量已在VibrationMonitor_Task任务中实现，使用detector->data.count判断周期
 *       count为0时表示刚刚完成一个完整的8秒周期
 *       时间测量会显示理论时间（8000ms）和实际时间的对比
 */
void vibration_detector_process(vibration_detector_t *detector);

// FreeRTOS任务相关声明
extern void VibrationMonitor_Task(void *pvParameters);  // 振动监控任务
extern TaskHandle_t vibration_monitor_task_handle;       // 振动监控任务句柄

#ifdef __cplusplus
}
#endif
#endif /* __VIBRATION_DETECTOR_H */
