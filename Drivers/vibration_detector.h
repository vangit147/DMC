/**
  *****************************************************************************
  * FILENAME   : vibration_detector.h
  * COPYRIGHT  : Copyright (c) 2024
  * DESCRIPTION: 振动检测器驱动头文件
  *
  * 功能说明：
  * - 基于ADXL357加速度计的振动检测
  * - 使用双条件实时检测（差值检测、RMS检测）
  * - 使用循环缓冲区优化内存使用
  * - 每8秒基于1600个数据点进行完整检测
  *
  * 振动检测配置参数和上位机命令对应关系：
  *
  * 【基础配置命令】
  * - VT=?: 获取振动阈值 - 基础振动检测阈值，推荐范围：0.5-1.5g
  * - VT=数值: 设置振动阈值 - 参数范围：0.1-5.0g，<50.0g验证
   * - VS=?: 获取振动检测源选择 - 返回值：0.0f或1.0f，0.0f=GPIO，1.0f=ADXL357
 * - VS=数值: 设置振动检测源选择 - 参数：0或1（内部转换为0.0f或1.0f）
  *
  * 【双条件检测参数命令】
  * - DT=?: 获取振动差值检测阈值 - 推荐范围：0.06-0.15g
  * - DT=数值: 设置振动差值检测阈值 - 参数范围：0.01-0.5g，<50.0g验证
  * - DR=?: 获取振动RMS检测阈值 - 推荐范围：0.15-0.5g
  * - DR=数值: 设置振动RMS检测阈值 - 参数范围：0.1-1.0g，<50.0g验证
  *
  * 【触发次数比例命令】
  * - DTR=?: 获取差值触发次数比例 - 范围：1-10，对应10%-100%
  * - DTR=数值: 设置差值触发次数比例 - 参数：1-10（对应1600个数据点的10%-100%）
  * - RTR=?: 获取RMS触发次数比例 - 范围：1-10，对应10%-100%
  * - RTR=数值: 设置RMS触发次数比例 - 参数：1-10（对应1600个数据点的10%-100%）
  *
  * 【高级配置命令】
  * - VN=?: 获取NORM值 - 通常为49526.6797，用于三轴加速度合成计算
  * - VN=数值: 设置NORM值 - 参数：大于0的浮点数
  *
  * 【数据查询命令】
  * - VD=?: 获取振动综合数据 - 返回83字节振动数据包，包含配置、数据、状态等信息
  *
  * 应用场景说明：
  * - 软地层：降低各阈值，提高检测灵敏度
  * - 中等地层：使用标准设置
  * - 硬地层：提高各阈值，减少误报
  * - 调试设置：使用标准双条件检测
  * - 高精度检测：使用严格阈值设置
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
// 滑动窗口大小（使用振动检测周期和频率计算，实现8秒响应时间）
// 由于RAM限制，我们使用分段存储的方式
#define VIBRATION_SLIDING_WINDOW_SIZE (VIBRATION_PERIOD * 1000 / VIBRATION_DETECTION_FREQUENCY)  // 8秒 * 1000ms / 5ms = 1600个数据点
#define VIBRATION_SEGMENT_SIZE 200           // 每段200个数据点
#define VIBRATION_SLIDING_WINDOW_SEGMENTS (VIBRATION_SLIDING_WINDOW_SIZE / VIBRATION_SEGMENT_SIZE)  // 分段数量：1600/200 = 8段

// 默认参数定义（按照代码使用顺序排列）
#define DEFAULT_VIBRATION_THRESHOLD      1.0f    // 默认振动阈值 1.0g - 上位机命令: VT=?
                                                   // 推荐范围：0.5-1.5g，根据钻井环境调整
                                                   // 软地层：0.5-0.8g，中等地层：0.8-1.2g，硬地层：1.2-1.5g

#define DEFAULT_VIBRATION_DELTA_THRESHOLD    0.2f    // 默认振动差值阈值 - 上位机命令: DT=?
                                                      // 推荐范围：0.06-0.15g，用于检测振动频率
                                                      // 软地层：0.06-0.1g，中等地层：0.1-0.12g，硬地层：0.12-0.15g

#define VIBRATION_DEFAULT_RMS_THRESHOLD     0.186f     // 默认RMS检测阈值 - 上位机命令: DR=?
                                                        // 推荐范围：0.15-0.5g，用于检测振动能量
                                                        // 软地层：0.15-0.25g，中等地层：0.25-0.35g，硬地层：0.35-0.5g

#define VIBRATION_SOURCE_GPIO               0.0f     // GPIO振动检测源 - 上位机命令: VS=?
                                                      // 0=GPIO振动检测（传统方式）
#define VIBRATION_SOURCE_ADXL357            1.0f     // ADXL357传感器振动检测源 - 上位机命令: VS=?
                                                      // 1=ADXL357传感器振动检测（推荐方式）
#define VIBRATION_DEFAULT_SOURCE            VIBRATION_SOURCE_GPIO  // 默认使用GPIO振动检测

#define DEFAULT_NORM_VALUE               49526.6797f  // 默认NORM值 - 上位机命令: VN=?
                                                       // 用于计算三轴加速度的合成值，通常不需要修改

// 新增：触发次数比例默认值
#define DEFAULT_DELTA_TRIGGER_RATIO      7            // 默认差值触发次数比例：7（70%）
                                                       // 对应1600个数据点的70%（1120次）
                                                       // 上位机命令：DTR=?（查询），DTR=数值（设置）

#define DEFAULT_RMS_TRIGGER_RATIO        7            // 默认RMS触发次数比例：7（70%）
                                                       // 对应1600个数据点的70%（1120次）
                                                       // 上位机命令：RTR=?（查询），RTR=数值（设置）

// 振动检测状态标志常量
#define VIBRATION_FLAG_DISABLED             0.0f     // 标志禁用状态
#define VIBRATION_FLAG_ENABLED              1.0f     // 标志启用状态

// 滑动窗口结构体
// 用于实现8秒滑动窗口机制，提供实时振动检测
// 与GPIO振动开关响应时间保持一致：1600个数据点（约8秒）
// 使用分段存储方式解决RAM限制问题
typedef struct
{
    uint32_t segment_index;              // 当前段索引（0-7）
    uint32_t data_index;                 // 当前段内数据索引
    uint32_t total_count;                // 总有效数据数量
    float sum_squared;                   // 平方和（用于RMS计算）
    uint32_t delta_count;                // 差值计数（频率检测）
    uint32_t rms_over_threshold_count;   // RMS超过阈值计数
    uint8_t is_full;                     // 窗口是否已满标志（1600个数据点）
    uint32_t oldest_segment;             // 最旧段索引
    uint32_t segment_delta_counts[VIBRATION_SLIDING_WINDOW_SEGMENTS];  // 每段的差值计数
    float segment_sum_squared[VIBRATION_SLIDING_WINDOW_SEGMENTS];       // 每段的平方和
} vibration_sliding_window_t;

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

    float vibration_source;       // 振动检测源选择：0.0f=GPIO振动检测，1.0f=ADXL357传感器振动检测
                                  // 用于控制振动检测的源选择，影响所有使用振动检测的功能
                                  // 0.0f=GPIO振动检测（传统方式），1.0f=ADXL357传感器振动检测（推荐方式）
                                  // 上位机命令：VS=?（查询），VS=数值（设置）

    // 各检测条件的专用阈值参数
    float delta_threshold;     // 差值检测阈值：与基准值的差值超过此阈值时触发
                               // 用于检测振动频率，数值越大检测越严格
                               // 推荐范围：0.06-0.15g，上位机命令：DT=?（查询），DT=数值（设置）

    float rms_threshold;       // RMS检测阈值：RMS值超过此阈值时触发
                               // 用于检测振动能量，数值越大检测越严格
                               // 推荐范围：0.15-0.5g，上位机命令：DR=?（查询），DR=数值（设置）

    float norm;                // ADXL357三轴加速度模长NORM值
                               // 用于计算三轴加速度的合成值，通常为49526.6797
                               // 上位机命令：VN=?（查询），VN=数值（设置）

    // 新增：触发次数比例参数（范围[1,10]，对应10%-100%）
    uint32_t delta_trigger_ratio;  // 差值触发次数比例：1-10，对应1600个数据点的10%-100%
                                   // 上位机命令：DTR=?（查询），DTR=数值（设置）
                                   // 1=10%（160次），2=20%（320次），...，10=100%（1600次）

    uint32_t rms_trigger_ratio;    // RMS超过阈值次数比例：1-10，对应1600个数据点的10%-100%
                                   // 上位机命令：RTR=?（查询），RTR=数值（设置）
                                   // 1=10%（160次），2=20%（320次），...，10=100%（1600次）
} vibration_config_t;

// 振动检测状态结构体
// 存储振动检测的状态标志，用于多条件检测
typedef __packed struct
{
    uint8_t is_triggered;      // 触发标志：1-已触发振动检测，0-未触发
    float delta_flag;          // 差值标志：1-差值条件满足，0-不满足
    float rms_flag;            // RMS标志：1-RMS条件满足，0-不满足
} vibration_state_t;

// 振动检测控制结构体
// 整合配置、状态、数据，形成完整的振动检测器
typedef struct
{
    vibration_config_t config;     // 配置参数：存储所有检测配置
    vibration_state_t state;       // 状态管理：存储检测状态和标志
    vibration_sliding_window_t sliding_window;  // 滑动窗口：实现8秒滑动窗口机制
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
 *       0.0f=GPIO振动检测，1.0f=ADXL357传感器振动检测
 */
uint32_t get_vibrating_flag(void);

/**
 * @brief 振动检测器完整初始化
 * @param detector 振动检测器指针
 * @note 执行振动检测器的完整初始化流程
 *       包括：状态初始化、滑动窗口初始化、Flash参数读取
 *       此函数在定时器开始前调用一次，完成所有初始化工作
 */
void vibration_detector_init(vibration_detector_t *detector);

/**
 * @brief 振动检测器配置参数读取
 * @param detector 振动检测器指针
 * @note 从Flash读取最新的振动检测配置参数
 *       每次都从Flash读取配置参数，保证上位机修改后立即生效
 *       此函数可在运行时调用，用于更新配置参数
 */
void vibration_detector_load_config(vibration_detector_t *detector);

/**
 * @brief 振动检测器数据处理
 * @param detector 振动检测器指针
 * @note 处理ADXL357传感器数据，进行振动检测
 *       每5ms调用一次，使用8秒滑动窗口进行实时检测
 *       使用双条件检测：频率检测（差值计数）+ 振幅检测（RMS）
 *       频率检测：统计超过阈值的次数
 *       振幅检测：使用RMS值表示振动振幅
 *       与GPIO振动开关响应时间保持一致：1600个数据点窗口（约8秒）
 */
void vibration_detector_process(vibration_detector_t *detector);

/**
 * @brief 滑动窗口初始化
 * @param window 滑动窗口指针
 * @note 初始化滑动窗口的所有成员变量
 */
void vibration_sliding_window_init(vibration_sliding_window_t *window);

/**
 * @brief 滑动窗口数据更新
 * @param window 滑动窗口指针
 * @param new_value 新的振动数据
 * @param threshold 振动阈值
 * @param delta_threshold 差值阈值
 * @param rms_threshold RMS阈值
 * @note 更新滑动窗口数据，移除最旧数据，添加新数据
 *       同时更新统计信息（差值计数、RMS计算等）
 */
void vibration_sliding_window_update(vibration_sliding_window_t *window,
                                   float new_value,
                                   float threshold,
                                   float delta_threshold,
                                   float rms_threshold);

/**
 * @brief 滑动窗口振动检测
 * @param window 滑动窗口指针
 * @param detector 振动检测器指针
 * @note 基于滑动窗口数据进行振动检测判断
 *       直接在函数内部设置振动状态
 */
void vibration_sliding_window_detect(vibration_sliding_window_t *window,
                                   vibration_detector_t *detector);

// FreeRTOS任务相关声明
extern void VibrationMonitor_Task(void *pvParameters);  // 振动监控任务
extern TaskHandle_t vibration_monitor_task_handle;       // 振动监控任务句柄

#ifdef __cplusplus
}
#endif
#endif /* __VIBRATION_DETECTOR_H */
