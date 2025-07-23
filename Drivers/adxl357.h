#ifndef __ADXL357_H__
#define __ADXL357_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#define NORM 49526.6797f

typedef struct {
    float avg_vibration;
    float min_vibration;
    float max_vibration;
    float vibration;
    uint32_t account;
} adxl357_vibration_data_t;

typedef struct {
    float x;
    float y;
    float z;
} SensorData;

extern TaskHandle_t     adx357_task_handle;
extern void adxl357_task(void* p);

extern float g_norm;

void Reset_Vibration_Stats(void);
void adxl357_get_adc_data(float* x, float* y, float* z);
void adxl357_get_adc_raw_data(float* x, float* y, float* z);
void adxl357_get_adc_norm(float* norm);

#endif /* __ADXL357_H__ */

