#include "main.h"

#define EVENT_VIBRATION_TIMER    0X40000000
TaskHandle_t              vibration_monitor_task_handle;
static uint8_t                   s_u8_adxl357_vibrating_flag;

typedef struct {
    float threshold;          // 振动阈值
    uint8_t enabled;          // 使能标志
    uint8_t is_triggered;     // 触发标志
    uint32_t trigger_count;   // 触发计数
    uint32_t count;           // 计数
    float vibrationdata[VIBRATION_PERIOD*1000/VIBRATION_DETECTION_FREQUENCY];
    float vibrationfilterdata[VIBRATION_PERIOD*1000/VIBRATION_DETECTION_FREQUENCY];
    float sum;           // 振动数值总和
} VibrationDetector_t;

SensorData data;
SensorData DATA[VIBRATION_PERIOD*1000/VIBRATION_DETECTION_FREQUENCY];

static VibrationDetector_t vibration_config = {
    .threshold = THRESHOLD,
    .enabled = 1,
    .is_triggered = 0,
    .trigger_count = 0,
    .count = 0,
    .sum = 0
};

uint32_t get_adxl357_vibrating_flag(void)
{
    return s_u8_adxl357_vibrating_flag;
}

static void VibrationDetector_Config_Init()
{
    vibration_config.threshold = is25pl032_flash_get_vibration_threshold();
}

// 中值滤波（窗口大小=5）
float median_filter(float *window, int size) {
    // 复制窗口数据以避免修改原数组
    float temp[size];
    for (int i = 0; i < size; i++) {
        temp[i] = window[i];
    }

    // 冒泡排序
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }

    // 返回中值
    return temp[size / 2];
}

// 滑动窗口均值滤波（窗口大小=5）
float moving_average_filter(float *window, int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += window[i];
    }
    return sum / size;
}

void remove_spikes(float *input, float *output, int n) {
    float window[WINDOW_SIZE];

    // 初始化窗口
    for (int i = 0; i < WINDOW_SIZE; i++) {
        window[i] = input[i];
    }

    // 处理数据
    for (int i = WINDOW_SIZE; i < n; i++) {
        // 滑动窗口更新
        for (int j = 0; j < WINDOW_SIZE - 1; j++) {
            window[j] = window[j + 1];
        }
        window[WINDOW_SIZE - 1] = input[i];

        // 先中值滤波去异常
        float median = median_filter(window, WINDOW_SIZE);

        // 再均值滤波平滑
        output[i] = moving_average_filter(window, WINDOW_SIZE);
    }
}

static void VibrationDetector_Process()
{
    if (!vibration_config.enabled) {
        return;
    }

//    uint8_t temp_data[200];


    DATA[vibration_config.count-1].x = data.x;
    DATA[vibration_config.count-1].y = data.y;
    DATA[vibration_config.count-1].z = data.z;
    vibration_config.vibrationdata[vibration_config.count-1] = sqrtf(DATA[vibration_config.count-1].x * DATA[vibration_config.count-1].x + DATA[vibration_config.count-1].y * DATA[vibration_config.count-1].y + DATA[vibration_config.count-1].z * DATA[vibration_config.count-1].z);

	  if(vibration_config.count >= (VIBRATION_PERIOD*1000/VIBRATION_DETECTION_FREQUENCY)) {
        remove_spikes(vibration_config.vibrationdata, vibration_config.vibrationfilterdata, sizeof(vibration_config.vibrationdata) / sizeof(vibration_config.vibrationdata[0]));
        for(uint32_t i=1;i<=vibration_config.count;i++)
		        vibration_config.sum += vibration_config.vibrationfilterdata[i];
        vibration_config.sum /= (float)vibration_config.count;
/*
        memset(temp_data, 0xFF, sizeof(temp_data));
        int n = sprintf((char*)temp_data,"sum=%.2f g\r\n", sum);
        LPUART1_send(temp_data, n);
        //printf("sum=%.2f g\r\n", sum);
*/
        if(vibration_config.sum > vibration_config.threshold)
		        vibration_config.is_triggered = 1;
        else
		        vibration_config.is_triggered = 0;
        vibration_config.count = 0;
        vibration_config.sum = 0;
        for(uint32_t i=1;i<=VIBRATION_PERIOD*1000/VIBRATION_DETECTION_FREQUENCY;i++) {
		        vibration_config.vibrationdata[i] = 0.0f;
		        vibration_config.vibrationfilterdata[i] = 0.0f;
        }
		}
}

static void vibration_timer_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(vibration_monitor_task_handle, EVENT_VIBRATION_TIMER, eSetBits, NULL);
}

void VibrationMonitor_Task(void *pvParameters)
{
    uint32_t notify;
    uint8_t temp_data[200];

    for(uint32_t i=1;i<=VIBRATION_PERIOD*1000/VIBRATION_DETECTION_FREQUENCY;i++) {
        vibration_config.vibrationdata[i] = 0.0f;
        vibration_config.vibrationfilterdata[i] = 0.0f;
		}

    xTimerStart(xTimerCreate("vibration_timer", VIBRATION_DETECTION_FREQUENCY, 1, 0, vibration_timer_cb), 1000);

    for(;;)
    {
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if(notify & EVENT_VIBRATION_TIMER)
        {
		        VibrationDetector_Config_Init();
		        vibration_config.count++;
		        adxl357_get_adc_data(&data.x, &data.y, &data.z);
		        VibrationDetector_Process();
		        if(vibration_config.is_triggered)
			          s_u8_adxl357_vibrating_flag = 1;
		        else
			          s_u8_adxl357_vibrating_flag = 0;
        }
    }
}

//START_TASK(VibrationMonitor_Task, "VibrationMonitor_Task", 256, NULL, TASK_PRIORITY_IAM, &vibration_monitor_task_handle);
