/**
  *****************************************************************************
  * FILENAME   : ads1278.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2025.04.07 10:00:00 Monday
  * DESCRIPTION: ADS1278 ADC驱动模块，用于采集加速度计、陀螺仪和温度数据
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.25                    CREATE
  * Grodon Li                  2025.04.06                    MODIFY - 精简为4通道
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#include "ads1278-2.h"

#define ADC_SCAN_CHANNELS      4   //需采样4个ADC通道

#define SYNC_GPIO_GROUP         PTC
#define SYNC_PINS               (1 << 16)

#define READY_PORT_GROUP        PORTC
#define READY_GPIO_GROUP        PTC
#define READY_PIN_NUMBER        17
#define READY_GPIO_PINS         (1 << READY_PIN_NUMBER)
#define EVENT_ADS1278_10MS_TIMER   0X20000000

static TaskHandle_t     ads1278_task_handle;

static uint8_t                  transfer_data_buffer[32];
static int32_t                  adc_raw_data[ADC_SCAN_CHANNELS];

// 全局变量，供其他文件直接访问
ads1278_global_raw_data_t s_ads1278_global_raw_data = {0};
/**
  *******************************************************************************
  * @Description: 更新全局变量
  * @Parameters : 无
  * @RetValue   : 无
  * @Note       : 将ADC原始数据转换为物理量并更新全局变量，包括加速度和温度数据
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07 10:00:00 Monday
  *******************************************************************************
  */
static void ads1278_update_global_vars(void)
{
#define     MV_PER_G        480     //1g加速度对应的毫伏值
    double    adc_scale = 2.0 * 2500.0 / 0x1000000 / MV_PER_G;

    
    // 更新原始数据结构体
    s_ads1278_global_raw_data.ay_raw = adc_raw_data[0];
    s_ads1278_global_raw_data.ax_raw = adc_raw_data[1];
    s_ads1278_global_raw_data.az_raw = adc_raw_data[3];
    s_ads1278_global_raw_data.t_raw = adc_raw_data[2];

}

/**
  *******************************************************************************
  * @Description: 结束数据传输
  * @Parameters : 无
  * @RetValue   : 无
  * @Note       : 将传输缓冲区数据转换为ADC原始数据，处理字节序并更新全局变量
  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2025.04.07 10:00:00 Monday
  *******************************************************************************
  */
static void ads1278_end_data_transfer(void)
{
    uint8_t     convert_buffer[32];
    uint8_t     *p_uint8;

    PINS_DRV_SetPinIntSel(READY_PORT_GROUP, READY_PIN_NUMBER, PORT_INT_FALLING_EDGE);

    //32bit
    for(int i = 0; i < 8; i++)
    {
        convert_buffer[4 * i + 0] = transfer_data_buffer[4 * i + 3];
        convert_buffer[4 * i + 1] = transfer_data_buffer[4 * i + 2];
        convert_buffer[4 * i + 2] = transfer_data_buffer[4 * i + 1];
        convert_buffer[4 * i + 3] = transfer_data_buffer[4 * i + 0];
    }

    p_uint8 = convert_buffer;
    for(int i = 0; i < ADC_SCAN_CHANNELS; i++)
    {
        int32_t     ret;

        ret = (*p_uint8++) << 16;
        ret |= (*p_uint8++) << 8;
        ret |= (*p_uint8++);
        if(ret & 0x800000)
            ret |= 0xff000000;

        adc_raw_data[i] = ret;
    }
    

}

/**
  *******************************************************************************
  * @Description: 数据就绪中断处理函数
  * @Parameters : int_flag - 中断标志，用于判断是哪个引脚触发了中断
  * @RetValue   : 无
  * @Note       : 当数据就绪时，启动SPI传输，接收ADS1278的数据
  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2025.04.07 10:00:00 Monday
  *******************************************************************************
  */
static void ads1278_data_ready_isr(uint32_t int_flag)
{
    if(int_flag & READY_GPIO_PINS)
    {
        LPSPI_DRV_MasterTransfer(LPSPICOM2, NULL, transfer_data_buffer, (ADC_SCAN_CHANNELS * 3 + 3) & ~0x3);
        PINS_DRV_SetPinIntSel(READY_PORT_GROUP, READY_PIN_NUMBER, PORT_DMA_INT_DISABLED);
    }
}

/**
  *******************************************************************************
  * @Description: 启动同步信号
  * @Parameters : 无
  * @RetValue   : 0-成功，其他-失败
  * @Note       : 发送同步信号启动ADC采样，通过控制GPIO引脚产生脉冲信号
  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2025.04.07 10:00:00 Monday
  *******************************************************************************
  */
static int32_t ads1278_start_SYNC(void)
{
    volatile    uint32_t loop = 10;

    PINS_DRV_ClearPins(SYNC_GPIO_GROUP, SYNC_PINS);
    while(loop--);
    PINS_DRV_SetPins(SYNC_GPIO_GROUP, SYNC_PINS);

    return 0;
}

/**
  *******************************************************************************
  * @Description: 初始化ADS1278
  * @Parameters : 无
  * @RetValue   : 0-成功，其他-失败
  * @Note       : 注册回调函数并启动同步信号，设置SPI和GPIO中断
  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2025.04.07 10:00:00 Monday
  *******************************************************************************
  */
static int32_t ads1278_init(void)
{
    spi2_register_end_transfer_cb(ads1278_end_data_transfer);
    gpio_portc_register_cb(ads1278_data_ready_isr);

    ads1278_start_SYNC();
    return 0;
}

/**
  *******************************************************************************
  * @Description: 定时器回调函数
  * @Parameters : xTimer - 定时器句柄，用于标识是哪个定时器触发了回调
  * @RetValue   : 无
  * @Note       : 通知任务定时器事件，每10ms触发一次，用于定期处理ADS1278数据
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07 10:00:00 Monday
  *******************************************************************************
  */
static void ads1278_task_timer_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(ads1278_task_handle, EVENT_ADS1278_10MS_TIMER, eSetBits, NULL);
}
/**
  *******************************************************************************
  * @Description: ADS1278任务
  * @Parameters : p - 任务参数，FreeRTOS任务创建时传入的参数
  * @RetValue   : 无
  * @Note       : 初始化ADS1278并处理定时器事件，创建10ms定时器并等待事件通知
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07 10:00:00 Monday
  *******************************************************************************
  */
static void ads1278_task(void* p)
{
    ads1278_init();

    xTimerStart(xTimerCreate("ads1278_task_timer_cb", 20, 1, 0, ads1278_task_timer_cb), 1000);
    for(;;)
    {
        uint32_t notify;
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if(notify & EVENT_ADS1278_10MS_TIMER)
        {
            ads1278_update_global_vars();
        }
    }
}
START_TASK(ads1278_task, "ads1278_task", 128, NULL, TASK_PRIORITY_ADS1278, &ads1278_task_handle);

