/**
  *****************************************************************************
  * FILENAME   : vibration_detector.h
  * COPYRIGHT  : Copyright (c) 2024
  * DESCRIPTION: 振动检测器驱动头文件
  *****************************************************************************
  */
#ifndef __VIBRATION_DETECTOR_H
#define __VIBRATION_DETECTOR_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

/*******************************************************/
// 振动配置参数
//ADXL357 采样频率，单位ms
#define ADXL357_COLLECTION_FREQUENCY 100
//振动检测周期，单位s
#define VIBRATION_PERIOD 8
//振动检测频率，单位ms
#define VIBRATION_DETECTION_FREQUENCY 100
//振动阈值 1.0g
#define THRESHOLD 1.0f
#define WINDOW_SIZE VIBRATION_PERIOD*1000/VIBRATION_DETECTION_FREQUENCY/2
/*******************************************************/

/* 函数声明 */
uint32_t get_adxl357_vibrating_flag(void);

extern void VibrationMonitor_Task(void *pvParameters);
extern TaskHandle_t    vibration_monitor_task_handle;

#ifdef __cplusplus
}
#endif
#endif /* __VIBRATION_DETECTOR_H */
