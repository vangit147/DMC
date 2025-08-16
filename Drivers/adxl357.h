#ifndef __ADXL357_H__
#define __ADXL357_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

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
} adxl357_vibration_data_t;

extern TaskHandle_t     adx357_task_handle;
extern void adxl357_task(void* p);

extern float g_norm;

/* 函数声明 */
void adxl357_task(void* p);
void adxl357_get_adc_norm(float* norm);
void Get_Vibration_Data(void);
void Reset_Vibration_Stats(void);

/* 新增：Flash操作状态管理 */
void set_flash_operation_status(bool in_progress);

#endif /* __ADXL357_H__ */

