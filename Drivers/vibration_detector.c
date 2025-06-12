#include "main.h"

#ifdef ADXL357_VIBRATION_TEST
#define EVENT_VIBRATION_TIMER    0X40000000
static TaskHandle_t              vibration_monitor_task_handle;
#endif
static uint8_t                   s_u8_adxl357_vibrating_flag;

typedef struct {
    float threshold;          // 振动阈值
    uint8_t enabled;          // 使能标志
    uint8_t sensitivity;      // 灵敏度设置
    uint8_t is_triggered;     // 触发标志
    uint32_t trigger_count;   // 触发计数
    uint32_t count;           // 计数
} VibrationDetector_t;

SensorData data;
SensorData DATA[VIBRATION_PERIOD*1000/VIBRATION_DETECTION_FREQUENCY];

static VibrationDetector_t vibration_config = {
    .threshold = THRESHOLD,
    .enabled = 1,
    .sensitivity = SENSITIVITY,
    .is_triggered = 0,
    .trigger_count = 0,
    .count = 0
};

uint32_t get_adxl357_vibrating_flag(void)
{
    return s_u8_adxl357_vibrating_flag;
}

static void VibrationDetector_Config_Init()
{
    float threshold;
    uint32_t sensitivity;
    threshold = is25pl032_flash_get_vibration_threshold();
    sensitivity = is25pl032_flash_get_vibration_sensitivity();
    vibration_config.threshold = threshold;
    vibration_config.sensitivity = (uint8_t)sensitivity;
}

static void VibrationDetector_Process()
{
    if (!vibration_config.enabled) {
        return;
    }

    float threshold;
//    uint8_t temp_data[200];
	  float sum=0.0f;

    DATA[vibration_config.count-1].x = data.x;
    DATA[vibration_config.count-1].y = data.y;
    DATA[vibration_config.count-1].z = data.z;
    threshold = vibration_config.threshold * ((float)vibration_config.sensitivity / 5.0f);

	  if(vibration_config.count >= (VIBRATION_PERIOD*1000/VIBRATION_DETECTION_FREQUENCY)) {
        for(uint32_t i=1;i<=vibration_config.count;i++)
		        sum += sqrtf(DATA[i].x * DATA[i].x + DATA[i].y * DATA[i].y + DATA[i].z * DATA[i].z);
        sum /= (float)vibration_config.count;
/*
        memset(temp_data, 0xFF, sizeof(temp_data));
        int n = sprintf((char*)temp_data,"sum=%.2f g\r\n", sum);
        LPUART1_send(temp_data, n);
        //printf("sum=%.2f g\r\n", sum);
*/
        if(sum > threshold)
		        vibration_config.is_triggered = 1;
        else
		        vibration_config.is_triggered = 0;
        vibration_config.count = 0;
		}
}

#ifdef ADXL357_VIBRATION_TEST

static void vibration_timer_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(vibration_monitor_task_handle, EVENT_VIBRATION_TIMER, eSetBits, NULL);
}

static void VibrationMonitor_Task(void *pvParameters)
{
    uint32_t notify;
    uint8_t temp_data[200];

    xTimerStart(xTimerCreate("vibration_timer", VIBRATION_DETECTION_FREQUENCY, 1, 0, vibration_timer_cb), 1000);

    for(;;)
    {
        VibrationDetector_Config_Init();
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if(notify & EVENT_VIBRATION_TIMER)
        {
		        vibration_config.count++;
		        adxl357_get_adc_data(&data.x, &data.y, &data.z);
		        VibrationDetector_Process();
/*
		        checking_vibrating_gpio();
		        if(get_vibrating_flag()) {
							  memset(temp_data, 0xFF, sizeof(temp_data));
                int n = sprintf((char*)temp_data,"vibrating_flag=%d\r\n", get_vibrating_flag());
		            //printf("vibrating_flag=%d\r\n", get_vibrating_flag());
							if(vibration_config.count%20==0)
		            LPUART1_send(temp_data, n);
						}
*/
		        if(vibration_config.is_triggered)
						{
			          s_u8_adxl357_vibrating_flag = 1;/*
							  //printf("vibration_config.is_triggered=%d", vibration_config.is_triggered);
							  memset(temp_data, 0xFF, sizeof(temp_data));
                int n = sprintf((char*)temp_data,"status.is_triggered=%d, vibration_config.threshold=%.2fg vibration_config.sensitivity=%d\r\n", 
								vibration_config.is_triggered, vibration_config.threshold, vibration_config.sensitivity);				
							if(vibration_config.count%20==0)
		            LPUART1_send(temp_data, n);*/
						}
		        else
			          s_u8_adxl357_vibrating_flag = 0;
        }
    }
}

START_TASK(VibrationMonitor_Task, "VibrationMonitor_Task", 256, NULL, TASK_PRIORITY_IAM, &vibration_monitor_task_handle);
#else
void checking_vibrating_adxl357(void)
{
    VibrationDetector_Config_Init();
    vibration_config.count++;
    adxl357_get_adc_data(&data.x, &data.y, &data.z);
    VibrationDetector_Process();
    if(vibration_config.is_triggered)
        s_u8_adxl357_vibrating_flag=1;
    else
        s_u8_adxl357_vibrating_flag=0;
}
#endif
