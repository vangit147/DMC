/**
 ******************************************************************************
 * @file    main_task.c
 * @author  Gordon Li
 * @version V1.13
 * @date    2025-04-05
 * @brief   主任务模块实现
 *
 * @note    实现了系统的主要任务调度和功能控制
 ******************************************************************************
 */
/******************************* 头文件包含区 ********************************/
#include "main.h"
#include "cpu.h"
#include "main_task.h"
#include "ie_task.h"
#include "IS25LP032_flash.h"
#include "signal_process.h" // 用于访问sensor_data_t结构体定义
#include "pins_driver.h"  // 用于PINS_DRV_ReadPins函数

/******************************* 宏定义区 **********************************/
// 事件定义宏
#define EVENT_TIMER_100MS 0X1               // 100ms定时器事件 
#define EVENT_UART2_RX 0X4                  // UART2接收事件 
#define EVENT_UART2_TIMEOUT 0X08            // UART2超时事件 
#define EVENT_UART1_RX 0X10                 // UART1接收事件 
#define EVENT_SEND_DBG_DATA_TIMER 0X2       // 发送调试数据定时器事件 

// 泥浆脉冲相关宏定义
#define MUD_PULSE_PORT_LED              PTE
#define MUD_PULSE_PIN_LED               11
#define MUD_PULSE_PORT                  PTA
#define MUD_PULSE_PIN                   13
#define MUD_PULSE_TIMER_HZ              100    // 泥浆脉冲定时器频率
#define DELAY_COUNT2                   120    // 停泵状态下的静态井斜延迟计数

/******************************* 外部变量声明区 ******************************/
extern sensor_data_t sensor_data;    // 可直接引用访问传感器数据 

/******************************* 内部变量定义区 ******************************/
// 任务句柄相关变量 
static TaskHandle_t main_task_handle;        // 主任务句柄 

// 串口通信相关变量
// UART1-调试/上位机通信端口
static uint8_t UART1_rx_buffer[128];    // UART1接收缓冲区
static uint32_t UART1_rx_data_len;      // UART1接收数据长度
static TimerHandle_t lpuart1_rx_timer;  // UART1接收定时器句柄

// UART2-井下通信/Modbus端口
static uint8_t UART2_rx_buffer[128];    // UART2接收缓冲区
static uint32_t UART2_rx_data_len;      // UART2接收数据长度
static TimerHandle_t lpuart2_rx_timer;  // UART2接收定时器句柄

// 陀螺仪数据统计变量
static float gz_avg = 0;                 // Z轴角速度平均值 
static uint32_t avg_count = 0;           // 角速度采样计数 

// 设备状态相关变量
// 井下状态相关变量：开机20秒后uart1端口无通信则进入井下模式
int8_t downhole;                  // 井下状态标志 1:井下（记录日志，不响应上位机通信），0:地面（响应上位机通信，不记录日志）
float vSupply;                  // ADC0_SE2 PTA6 36V电源电压监测值
static bool pumping = false;            // 开泵状态:1.开泵中，0.停泵中
static bool previous_pumping = false;   // 前一状态开泵状态:1.开泵中，0.停泵中
static uint8_t current_vibration_status = 0;  // 当前振动状态，从on_100ms_timer_event中的局部变量赋值

// 静态井斜相关变量
float pump_off_inc = -1;                // 静态井斜，停泵状态下的静态井斜 -1表示没有有效静止井斜
float pump_off_inc_sum = 0;             // 停泵状态下的静态井斜累计值
float invalid_pump_off_inc_sum1 = 0;    // 停泵状态下的静态井斜前置无效累计值
float invalid_pump_off_inc_sum2 = 0;    // 停泵状态下的静态井斜后置无效累计值
uint16_t invalid_pump_off_inc_count1 = 0;  // 停泵状态下的静态井斜前置无效计数
uint16_t invalid_pump_off_inc_count2 = 0;  // 停泵状态下的静态井斜后置无效计数
int32_t inc_count = 0;                   // 停泵状态下的静态井斜总计数

// 泥浆脉冲控制变量
static int32_t  pulser_tx_buffer[64];        // 发送缓冲区
static uint32_t pulser_tx_buffer_len;       // 缓冲区长度
static uint8_t  pulser_tx_started;          // 发送启动标志
static uint32_t pulser_curr_tx_index;        // 当前发送索引
static uint32_t pulser_current_duration;     // 当前持续时间

// 定时发送控制变量
static uint32_t mud_pulse_timer_counter = 0;  // 定时发送计数器（100ms为单位）
static uint32_t mud_pulse_send_interval = 36000; // 发送间隔（3600秒 = 36000 * 100ms）

// 静态数据重传控制变量
static uint32_t  pump_off_data_send_retry_count = 3;  // 静态数据重传次数
static uint32_t pump_off_data_send_interval = 12000; // 静态数据重传间隔（1200秒 = 12000 * 100ms）

/******************************* 函数声明区 ********************************/
// 泥浆脉冲相关函数声明
static void update_mud_pulser_state(void);
static int32_t pulser_start_tx(int32_t * data, uint32_t len);
static void prepare_data_transmission(int type);

// 振动状态检测函数
static uint8_t check_gpio_vibration(void);

// 定时器中断处理函数
static int32_t flextimer_mc1_isr(void);

// 串口回调函数
static void lpuart1_tx_cb(uint32_t uartHandle);
static void lpuart1_rx_cb(uint32_t uartHandle, uint32_t data);
static void lpuart2_tx_cb(uint32_t uartHandle);
static void lpuart2_rx_cb(uint32_t uartHandle, uint32_t data);

// 定时器回调函数
static void lpuart2_rx_timer_cb(TimerHandle_t timer);

// 事件处理函数
static void on_100ms_timer_event(void);
static void on_send_debug_data_event(void);

// 主任务函数
void main_task(void *pvParameters);

// 系统钩子函数
#if (configUSE_TICK_HOOK > 0)
void vApplicationTickHook(void);
#endif

/******************************* 函数实现区 ********************************/
// ==================== 系统工具函数 ====================
/**
 *******************************************************************************
 * @Description: 获取36V电源电压
 * @Parameters : 无
 * @RetValue   : 36V电源电压值
 * @Note       : 返回ADC转换后的电压值
 * @CreatedBy  : Gordon Li
 * @CreatedDate: 2025-01-27
 *******************************************************************************
 */

void start_and_get_adc_result(void)
{
    uint16_t u16_ADC_raw_result;

    //ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig0); 配置ADC通道，只需执行一次
    ADC_DRV_WaitConvDone(INST_ADCONV1);//等待转换完成
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &u16_ADC_raw_result);//获取转换结果

    // 电压缩放系数:21
    vSupply = (float)u16_ADC_raw_result * 3.300f * 21.0f / 4096.0f;
}

/**
  *******************************************************************************
  * @Description: 应用层钩子函数
  * @Parameters :
  * @RetValue   :
  * @Note       :FreeRTOS 系统钩子函数（Hook Function）每个系统时钟节拍（Tick）都会自动调用
  * 主要用途： 系统监控， 性能统计，看门狗等
  * 只有当 configUSE_TICK_HOOK > 0 时才启用，根据FreeRTOSConfig.h 中配置，当前未启用
  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.01.27 14:29:03 Saturday
  *******************************************************************************
  */
#if (configUSE_TICK_HOOK > 0)
void vApplicationTickHook(void)
{
    //当前为空实现，无具体功能
}
#endif

// ==================== 中断处理函数 ====================
/**
  *******************************************************************************
  * @Description: 定时器通道1中断处理函数，中断类型：硬件定时器中断（FlexTimer）
  * @Parameters :
  * @RetValue   :
  * @Note       : 泥浆脉冲定时器中断服务，用于更新泥浆脉冲数据
  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.01.28 00:19:16 Sunday
  *******************************************************************************
  */
static int32_t flextimer_mc1_isr(void)
{
    update_mud_pulser_state();
    return 0;
}

// ==================== 串口回调函数 ====================
/**
  *******************************************************************************
  * @Description: 串口1发送完成回调函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 22:07:08 Sunday
  *******************************************************************************
  */
static void lpuart1_tx_cb(uint32_t uartHandle)
{
}
/**
  *******************************************************************************
  * @Description: 串口1接收回调函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 23:59:08 Sunday
  *******************************************************************************
  */
static void lpuart1_rx_cb(uint32_t uartHandle, uint32_t data)
{

    if (data == '\n')  // 忽略换行符
        return;
    // Convert to Uppercase
    if (data >= 'a' && data <= 'z')  // 转换为大写字母
        data -= 32;
    if (UART1_rx_data_len < sizeof(UART1_rx_buffer)) // 检查缓冲区大小
        UART1_rx_buffer[UART1_rx_data_len++] = (uint8_t)data;
    if ((data == '\r' || UART1_rx_data_len >= sizeof(UART1_rx_buffer)) && main_task_handle) //接收到回车符或者缓冲区已满，通知主任务处理
        xTaskGenericNotifyFromISR(main_task_handle, EVENT_UART1_RX, eSetBits, NULL, NULL);
    else if (lpuart1_rx_timer)
    {
        // xTimerChangePeriodFromISR(lpuart1_rx_timer, 50, 0);
    }
}
/**
  *******************************************************************************
  * @Description: 串口2发送完成回调函数
  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 22:07:21 Sunday
  *******************************************************************************
  */
static void lpuart2_tx_cb(uint32_t uartHandle)
{
}
/**
  *******************************************************************************
  * @Description: 串口2接收回调函数
  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 23:59:08 Sunday
  *******************************************************************************
  */
static void lpuart2_rx_cb(uint32_t uartHandle, uint32_t data)
{
    if (UART2_rx_data_len < sizeof(UART2_rx_buffer))
        UART2_rx_buffer[UART2_rx_data_len++] = (uint8_t)data;
    if (UART2_rx_data_len >= sizeof(UART2_rx_buffer))
        xTaskGenericNotifyFromISR(main_task_handle, EVENT_UART2_RX, eSetBits, NULL, NULL);
    else
        xTimerChangePeriodFromISR(lpuart2_rx_timer, 10, 0);
}
/**
  *******************************************************************************
  * @Description: 100ms定时器回调函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 22:05:03 Sunday
  *******************************************************************************
  */
static void timer_100ms_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(main_task_handle, EVENT_TIMER_100MS, eSetBits, NULL);
}
// ==================== 定时器回调函数 ====================
/**
  *******************************************************************************
  * @Description: 发送调试数据定时器回调函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.22 23:30:32 Thursday
  *******************************************************************************
  */
static void timer_send_dbg_data_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(main_task_handle, EVENT_SEND_DBG_DATA_TIMER, eSetBits, NULL);
}

/**
  *******************************************************************************
  * @Description: 串口2接收超时定时器回调函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.07 22:32:11 Saturday
  *******************************************************************************
  */
static void timer_waiting_for_uart2_timeout_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(main_task_handle, EVENT_UART2_TIMEOUT, eSetBits, NULL);
}
/**
  *******************************************************************************
  * @Description: 串口1接收超时定时器回调函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.01 22:21:29 Friday
  *******************************************************************************
  */
static void timer_lpuart1_rx_cb(TimerHandle_t timer)
{
    xTaskGenericNotify(main_task_handle, EVENT_UART1_RX, eSetBits, NULL);
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NY
  * @CreatedDate: 2024.08.03 12:05:55 Saturday
  *******************************************************************************
  */
static void lpuart2_rx_timer_cb(TimerHandle_t timer)
{
    xTaskGenericNotify(main_task_handle, EVENT_UART2_RX, eSetBits, NULL);
}

// ==================== 泥浆脉冲相关函数 ====================
/**
 *******************************************************************************
 * @Description: 泥浆脉冲状态机
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 每10ms调用一次，控制泥浆脉冲的发送状态
 * @CreatedBy  : Gordon Li
 * @CreatedDate: 2025-01-27
 *******************************************************************************
 */
static void update_mud_pulser_state(void)
{
    static uint8_t  led;
    static uint32_t counter;

    if(pulser_tx_started == 0)
        return;

    counter++;
    if(counter >= MUD_PULSE_TIMER_HZ / 10)  // 100Hz定时器，每10次切换LED
    {
        counter = 0;
        led ^= 1;
        PINS_DRV_WritePin(MUD_PULSE_PORT_LED, MUD_PULSE_PIN_LED, led);
    }

    /*当前脉冲未发送完毕*/
    if(pulser_current_duration > 1)
    {
        pulser_current_duration--;
        return;
    }

    //发送下一个脉冲
    pulser_curr_tx_index++;
    pulser_current_duration = pulser_tx_buffer[pulser_curr_tx_index];

    //所有脉冲发送完毕
    if(pulser_curr_tx_index == pulser_tx_buffer_len)
    {
        pulser_curr_tx_index = 0;
        PINS_DRV_WritePin(MUD_PULSE_PORT_LED, MUD_PULSE_PIN_LED, 0); // 脉冲发送完毕后，LED熄灭
        pulser_current_duration = pulser_tx_buffer[pulser_curr_tx_index];  // 加载第一段时间
    }

    PINS_DRV_WritePin(MUD_PULSE_PORT, MUD_PULSE_PIN, pulser_curr_tx_index & 0x1);
}

/**
 *******************************************************************************
 * @Description: 启动泥浆脉冲发送（参考Origin实现）
 * @Parameters : data - 发送数据缓冲区，len - 数据长度
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 启动泥浆脉冲发送
 * @CreatedBy  : Gordon Li
 * @CreatedDate: 2025-01-27
 *******************************************************************************
 */
static int32_t pulser_start_tx(int32_t * data, uint32_t len)
{
    if(len == 0) return -1;

    __disable_irq();
    pulser_tx_started = 1;                    // 标记发送开始
    pulser_curr_tx_index = 0;                 // 重置索引
    PINS_DRV_WritePin(MUD_PULSE_PORT, MUD_PULSE_PIN, 0);  // 初始输出低电平
    PINS_DRV_WritePin(MUD_PULSE_PORT_LED, MUD_PULSE_PIN_LED, 0);
    pulser_current_duration = pulser_tx_buffer[pulser_curr_tx_index];  // 加载第一段时间
    __enable_irq();

    return 0;
}

/**
  *******************************************************************************
  * @Description: 初始化片内外设
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.21 23:45:08 Wednesday
  *******************************************************************************
  */
static int32_t init_on_chip_peripheral(void)
{
    LPUART1_init(lpuart1_tx_cb, lpuart1_rx_cb);    // 初始化LPUART1，设置发送和接收回调函数
    LPUART2_init(lpuart2_tx_cb, lpuart2_rx_cb);    // 初始化LPUART2，设置发送和接收回调函数

    i2c_init();                                     // 初始化I2C接口

    /*对于V1.2
    SPI0 CS0 ---- IS25LP032D
    SPI0 CS1 ---- IAM 20680HT
    SPI0 CS2 ---- ADXL357BEZ
    SPI1     ---- Reserved
    SPI2     ---- ADS1278HPAP
    */
    spi0_init();                                    // 初始化SPI0接口，用于Flash、IMU和加速度计
    spi1_init();                                    // 初始化SPI1接口，预留备用
    spi2_init();                                    // 初始化SPI2接口，用于ADS1278高精度ADC

    gpio_port_init();                               // 初始化GPIO端口配置
    flextime_mc1_init(flextimer_mc1_isr);          // 初始化FlexTimer1，设置中断服务函数
    return 0;                                       // 返回成功状态
}
INIT_CALL_1(init_on_chip_peripheral);

/**
  *******************************************************************************
  * @Description: 初始化板级外设
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.21 23:47:24 Wednesday
  *******************************************************************************
  */
static int32_t init_on_board_peripheral(void)
{
    is25pl032_flash_init();                         // 初始化IS25LP032 Flash存储器

    pca8565_init();                                 // 初始化PCA8565 RTC实时时钟芯片

    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);    // 配置ADC转换器参数
    ADC_DRV_AutoCalibration(INST_ADCONV1);                          // 执行ADC自动校准

    // 初始化ADC通道配置，用于采集电池供电电压
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig0);

    return 0;                                        // 返回成功状态
}

// ==================== 事件处理函数 ====================
/**
  *******************************************************************************
  * @Description: 发送调试数据
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.22 22:23:37 Thursday
  *******************************************************************************
  */
static void on_send_debug_data_event(void)
{
#if 0
    // 调试数据相关局部变量 25/08/31 Gordon
    uint32_t tail_flag = 0x7f800000;        // 调试数据尾部标识 25/08/31 Gordon
    float debug_data[20];                    // 调试数据数组 25/08/31 Gordon
    int32_t  counter = 0;                    // 数据计数器 25/08/31 Gordon

    //debug_data[counter + 6] = ads1278_get_raw_acc_gyro_temp(&debug_data[counter + 0], &debug_data[counter + 1],
    //    &debug_data[counter + 2], &debug_data[counter + 3], &debug_data[counter + 4], &debug_data[counter + 5]);
    //counter += 7;

    debug_data[counter + 6] = iam_20680ht_get_raw_acc_gyro_temp(&debug_data[counter + 0], &debug_data[counter + 1],
                              &debug_data[counter + 2], &debug_data[counter + 3], &debug_data[counter + 4], &debug_data[counter + 5]);
    counter += 7;

    // 使用全局变量inc_hs_data
    extern inclination_hs_t inc_hs_data;
    debug_data[counter + 0] = inc_hs_data.inc1;
    debug_data[counter + 1] = inc_hs_data.inc2;
    debug_data[counter + 2] = inc_hs_data.inc3;
    counter += 3;

    LPUART1_send((uint8_t*)debug_data, counter << 2);
    LPUART1_send((uint8_t*)&tail_flag, 4);
#endif
}

/**
  *******************************************************************************
  * @Description: 定时器事件处理函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.22 14:31:49 Thursday
  *******************************************************************************
  */
static void on_100ms_timer_event(void)
{
    // RTC相关静态变量 25/08/31 Gordon
    static uint32_t device_usage_time = 0;   // 设备运行时间计数器
    static uint32_t log_period = 0;         // 日志记录周期计数器，当其递增到algorithm_setting.log_period_time时，记录一条日志

    gz_avg += sensor_data.gz_dps;
    avg_count++;

    // 启动ADC，获取36V电压
    start_and_get_adc_result();

    // 获取震动状态 - 使用新的GPIO振动检测函数
    uint8_t vibration_status = check_gpio_vibration();
    
    // 将局部变量值赋给全局变量，供其他函数使用
    current_vibration_status = vibration_status;

    // 注释掉原来的调用
    // checking_vibrating_gpio();
    
    // 根据振动状态更新开泵状态
    // 振动状态为1表示开泵中，振动状态为0表示停泵中
    pumping = (vibration_status == 1) ? true : false;

    //1.记录日志
    log_period++; 
    if (log_period >= algorithm_setting.log_period_time*10)  //100ms为单位，所以需要乘以10
    {
        //井下模式才实际上记录日志
        if(downhole==1)
            record_log_to_flash();
        //无论记录日志与否，都需要重置日志记录周期计数器
        log_period = 0; 
        reset_interval_info();
    }

    
    // 2.更新硬件设备的使用情况（按秒）
    device_usage_time++;
    if (device_usage_time >= 10) // 10次100ms = 1秒
    {
        // 清除计数
        device_usage_time = 0;
        // 更新各个温度下运行时间
        update_total_time_per_temp(sensor_data.t_C);
    }

    //泥浆脉冲处理
    /*状态管理
    1. 动态->静态：previous_pumping != pumping，并且当前pump_status==off
    2. 静态->动态：previous_pumping != pumping，并且当前pump_status==on
    3. 静态->静态：previous_pumping == pumping，并且当前pump_status==off
    4. 动态->动态：previous_pumping == pumping，并且当前pump_status==on
    
    状态1：知晓自己进入了静态，将上次pum_staus为on，结束；
    状态2：知晓自己进入了动态，判断静态时长足够，计算有效静态井斜，并设置待传输状态，结束；
        判断逻辑： (inc_count-invalid_inc_count_1-invalid_inc_count_2)>0, 则认定有效
        静态井斜计算：累计数值除以有效计数（总计数-前置无效计数-后置无效计数）
    状态3：知晓自己进入了静态，将上次pum_staus为off，结束；
        获取inc_hs_data中的inc_lpf 并累计到静态变量：pump_off_inc_sum 并累加总计数
        计算停泵最初100个静态井斜的累计（10秒），invalid_pump_off_inc_sum1 并记录前置无效计数
        计算停泵最后100个静态井斜的累计（10秒），invalid_pump_off_inc_sum2 并记录后置无效计数
    状态4：知晓自己进入了动态，进行传输，结束；
         静态井斜传输没完成就继续静态井斜传输， 静态井斜传输完成了的话，就传输动态井斜
    */
    if(previous_pumping != pumping) //开泵状态发生变化
    {
        // 状态2：静态->动态
        if(pumping == true) 
        {
            //更新静态井斜：静态状态有效（静止了足够的时间），计算有效静态井斜并赋值
            calculate_pump_off_inc();
        }
        // 状态1：动态->静态
        else 
        {
            // 重置静态井斜相关变量
            pump_off_inc_sum = 0;
            invalid_pump_off_inc_sum1 = 0;
            invalid_pump_off_inc_sum2 = 0;
            inc_count = 0;
            pump_off_inc = -1; // 重置为无效值
            pump_off_data_send_retry_count = is25pl032_flash_get_pulse_retry_for_pump_off_data();
        }
    }
    // 状态4：动态->动态
    else if(pumping == true) 
    {
        // 泥浆脉冲定时发送控制
        mud_pulse_timer_counter++;

        // 静态传输未完成 并且 达到间隔时间 并且 未在传输中
        if(pump_off_data_send_retry_count >0 && 
            mud_pulse_timer_counter >= pump_off_data_send_interval &&  
            pulser_tx_started==0 ) 
        {
            prepare_data_transmission(1); // 1 for static data
            pulser_start_tx(pulser_tx_buffer, pulser_tx_buffer_len);
            mud_pulse_timer_counter = 0; // 重置计数器
            pump_off_data_send_retry_count--; // 重传次数减一
        }    
        // 静态传输完成 并且 达到间隔时间 并且 未在传输中    
        else if(pump_off_data_send_retry_count ==0 && 
            mud_pulse_timer_counter >= mud_pulse_send_interval &&  
            pulser_tx_started==0 ) 
        {
            prepare_data_transmission(0); // 0 for dynamic data
            pulser_start_tx(pulser_tx_buffer, pulser_tx_buffer_len);
            mud_pulse_timer_counter = 0; // 重置计数器
        }
    }
    // 状态3：静态->静态：previous_pumping == pumping，并且当前pump_status==off
    else 
    {   
        //累计静态井斜数据，准备在状态2时计算有效静态井斜
        accumulate_pump_off_inc();
    }
    previous_pumping = pumping;
}

/**
 *******************************************************************************
 * @Description: 更新静态井斜函数
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 当从静态状态切换到动态状态时，计算有效静态井斜，没有有效井斜时为-1
 * @CreatedBy  : Gordon Li
 * @CreatedDate: 2025-01-27
 *******************************************************************************
 */
void calculate_pump_off_inc(void)
{
    // 判断静态状态是否有效
    // 要求：至少持续20秒的数据才有效，所以至少需要200次调用
    int32_t valid_count = inc_count - invalid_pump_off_inc_count1 - invalid_pump_off_inc_count2;

    if (inc_count >= invalid_pump_off_inc_count1 + invalid_pump_off_inc_count2 && valid_count > 0)
    {
        // 计算有效静态井斜：累计数值除以有效计数（总计数-前置无效计数-后置无效计数） 掐头（8秒）去尾（12秒）
        pump_off_inc = (pump_off_inc_sum - invalid_pump_off_inc_sum1 - invalid_pump_off_inc_sum2) / valid_count;
    }
    else
    {
        // 静态状态无效：要么总时间不足invalid_pump_off_inc_count1 + invalid_pump_off_inc_count2秒，要么没有有效数据
        pump_off_inc = -1;
    }
}

/**
 *******************************************************************************
 * @Description: 累计静态井斜函数
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 在停泵状态下，计算静态井斜的累计值
 * @CreatedBy  : Gordon Li
 * @CreatedDate: 2025-01-27
 *******************************************************************************
 */
void accumulate_pump_off_inc(void)
{
    // 获取inc_hs_data中的inc1（低通滤波井斜）并累计到静态变量：pump_off_inc_sum 并累加总计数
    pump_off_inc_sum += inc_hs_data.inc1;
    inc_count++;

    // 计算停泵最后invalid_pump_off_inc_count2个静态井斜的累计（12秒），invalid_pump_off_inc_sum2 并记录后置无效计数
    // 使用滑动窗口逻辑：维护最近DELAY_COUNT2个值的累计
    static float recent_values[DELAY_COUNT2] = {0}; // 最近DELAY_COUNT2个值的数组
    static int32_t window_index = 0;       // 滑动窗口索引    
    
    // 计算停泵最初invalid_pump_off_inc_count1个静态井斜的累计（8秒），invalid_pump_off_inc_sum1 并记录前置无效计数
    if (inc_count <= invalid_pump_off_inc_count1) // 前8秒（80次100ms调用）
    {
        invalid_pump_off_inc_sum1 += inc_hs_data.inc1;
    }
    else //累计前invalid_pump_off_inc_count1个数字以后
    {
        // 更新滑动窗口
        if (window_index >= invalid_pump_off_inc_count2)
        {
            // 移除最旧的值
            invalid_pump_off_inc_sum2 -= recent_values[window_index % DELAY_COUNT2];
        }
        // 添加新值
        recent_values[window_index % DELAY_COUNT2] = inc_hs_data.inc1;
        invalid_pump_off_inc_sum2 += inc_hs_data.inc1;
        window_index++;
    }
}

/**
 *******************************************************************************
 * @Description: 记录日志到Flash函数
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 将传感器数据、统计信息等记录到Flash中
 * @CreatedBy  : Gordon Li
 * @CreatedDate: 2025-01-27
 *******************************************************************************
 */
void record_log_to_flash(void)
{
    // 日志记录相关局部变量
    int32_t ret;
    log_t log = {0};
    uint32_t timestamp;
    uint8_t rtc_data[8];
    struct tm time_data;

    // 计算时间戳
    PCA8565_on_timer_event();
    PCA8565_get_data(rtc_data, 7);
    memset(&time_data, 0, sizeof(time_data));
    time_data.tm_sec = (rtc_data[0] >> 4 & 0x7) * 10 + (rtc_data[0] & 0xf);
    time_data.tm_min = (rtc_data[1] >> 4 & 0x7) * 10 + (rtc_data[1] & 0xf);
    time_data.tm_hour = (rtc_data[2] >> 4 & 0x3) * 10 + (rtc_data[2] & 0xf);
    time_data.tm_mday = (rtc_data[3] >> 4 & 0x3) * 10 + (rtc_data[3] & 0xf);
    time_data.tm_mon = (rtc_data[5] >> 4 & 0x1) * 10 + (rtc_data[5] & 0xf) - 1;
    time_data.tm_year = 2000 - 1900 + (rtc_data[6] >> 4 & 0xf) * 10 + (rtc_data[6] & 0xf);

    timestamp = mktime(&time_data) - 3600 * 8; // ZONE 8

    // 从sensor_data 全局变量中获取传感器数据
    log.timestamp = timestamp;
    log.ax = sensor_data.ax_g;
    log.ay = sensor_data.ay_g;
    log.az = sensor_data.az_g;
    log.gx = sensor_data.gx_dps;
    log.gy = sensor_data.gy_dps;
    log.gz = sensor_data.gz_dps;
    log.temp = sensor_data.t_C;

    // 将interval_info中的数据赋值给log结构体
    log.gz_max = interval_info.gyro_z_dps_max;
    log.gz_min = interval_info.gyro_z_dps_min;
    log.gz_avg = interval_info.gyro_z_dps_avg;

    log.inc1_max = interval_info.inc1_max;
    log.inc1_min = interval_info.inc1_min;
    log.inc1_avg = interval_info.inc1_avg;

    log.inc2_max = interval_info.inc2_max;
    log.inc2_min = interval_info.inc2_min;
    log.inc2_avg = interval_info.inc2_avg;

    log.inc6_max = interval_info.good_inc_max;
    log.inc6_min = interval_info.good_inc_min;
    log.inc6_avg = interval_info.good_inc_avg;

    // 将inc_hs_data中的数据赋值给log结构体
    log.hs = inc_hs_data.hs_lpf;

    // 从interval_info_data中获取虚拟半径数据
    log.virtual_x_radius_min = interval_info.radius_x_min;
    log.virtual_x_radius_max = interval_info.radius_x_max;

    log.virtual_y_radius_min = interval_info.radius_y_min;
    log.virtual_y_radius_max = interval_info.radius_y_max;

    // 从interval_info_data中获取标准差数据
    log.gz_dps_sdv_max = interval_info.sdv_gyro_z_max;
    log.gz_dps_sdv_min = interval_info.sdv_gyro_z_min;

    // 振动检测标准差统计
    log.std_v_norm_g_max = interval_info.std_v_norm_g_max;
    log.std_v_norm_g_min = interval_info.std_v_norm_g_min;
    log.std_v_norm_g_avg = interval_info.std_v_norm_g_avg;

    // 电压数据
    log.s_f32_36V = vSupply;

    // 振动标志位：编码钻进状态、旋转状态和GPIO振动状态
    // bit 0: 钻进状态 (1=钻进中, 0=静态)
    // bit 1: 旋转状态 (1=旋转中, 0=不旋转)
    // bit 2: GPIO振动状态 (1=检测到振动, 0=未检测到振动)
    // bit 3-31: 保留位
    log.flag = 0;
    if (algorithm_data.drilling) {
        SET_DRILLING_FLAG(log.flag);  // 设置钻进状态位
    }
    if (algorithm_data.rotating) {
        SET_ROTATING_FLAG(log.flag);  // 设置旋转状态位
    }

    // 使用全局变量中的GPIO振动状态设置标志位（避免重复调用振动检测函数）
    if (current_vibration_status == 1) {
        SET_GPIO_VIBRATION_FLAG(log.flag);  // 设置GPIO振动状态位
    }

    reset_interval_info();

    // 计算转速
    gz_avg = 0;
    avg_count = 0;

    // 写入日志到Flash，返回值：0-成功，-1-日志写入失败，-2-日志上下文更新失败，-3-FLASH_INITED_FLAG恢复失败
    if ((ret = is25pl032_flash_write_one_log(&log)) != 0)
        printf("Writing ONE log failed! ret=%d\r\n", ret);
}


// 加载配置
void load_algorithm_setting_from_flash(void)
{
    /* 初始化算法设置参数 */
    // 加速度计类型
    algorithm_setting.acc_sensor_type = get_acc_sensor_type();

    // 陀螺仪类型
    algorithm_setting.gyro_sensor_type = get_gyro_sensor_type();

    // 从Flash中读取虚拟半径限制参数
    // Flash参数读取相关局部变量 25/08/31 Gordon
    double xr_limit, yr_limit;              // X轴和Y轴虚拟半径限制 25/08/31 Gordon
    is25pl032_flash_get_param(&xr_limit, &yr_limit);
    algorithm_setting.xr_limit = xr_limit;
    algorithm_setting.yr_limit = yr_limit;
    // 确保虚拟半径限制参数有效
    if (algorithm_setting.yr_limit <= 0)
    {
        algorithm_setting.yr_limit = 0.01f; // 默认Y轴限制为0.1米，按照g要缩小100倍
    }
    if (algorithm_setting.xr_limit <= 0)
    {
        algorithm_setting.xr_limit = 0.001f; // 默认X轴限制为0.001米，按照g要缩小10倍
    }

    // 确保日志周期默认至少为60秒
    algorithm_setting.log_period_time = is25pl032_flash_get_log_period();
    /* 如果Flash中没有参数，使用默认值 */
    if (algorithm_setting.log_period_time <= 0)
    {
        algorithm_setting.log_period_time = 60;
    }

    /* 稳定时间阈值设置 */
    algorithm_setting.max_peace_time_threshold = 5; // 稳定时间阈值，默认值为5

    // IMU参数读取相关局部变量 25/08/31 Gordon
    double imu[9];                          // IMU校准参数数组 25/08/31 Gordon
    is25pl032_flash_get_imu(imu);
    /* 加速度计MS矩阵偏差设置 */
    algorithm_setting.ms_xx = imu[3]; // X-X轴MS矩阵系数
    algorithm_setting.ms_xy = imu[4]; // X-Y轴MS矩阵系数
    algorithm_setting.ms_xz = imu[5]; // X-Z轴MS矩阵系数
    algorithm_setting.ms_yy = imu[6]; // Y-Y轴MS矩阵系数
    algorithm_setting.ms_yz = imu[7]; // Y-Z轴MS矩阵系数
    algorithm_setting.ms_zz = imu[8]; // Z-Z轴MS矩阵系数

    /* 加速度计零偏设置 */
    algorithm_setting.ax_bias = imu[0]; // X轴加速度计零偏，单位：g
    algorithm_setting.ay_bias = imu[1]; // Y轴加速度计零偏，单位：g
    algorithm_setting.az_bias = imu[2]; // Z轴加速度计零偏，单位：g

    /* 加速度计装配误差设置 */
    algorithm_setting.acc_x_offset = is25pl032_flash_get_offset(0); // X轴加速度计装配误差，单位：g
    algorithm_setting.acc_y_offset = is25pl032_flash_get_offset(1); // Y轴加速度计装配误差，单位：g
    algorithm_setting.acc_z_offset = 0.0f;                          // Z轴加速度计装配误差，单位：g

    // 设置陀螺仪零偏和scale
    if (algorithm_setting.gyro_sensor_type == SIGNAL_PROCESS_GYRO_HT20680)
    {
        // IAM-20680HT陀螺仪设置
        algorithm_setting.gx_bias = get_gx_offset();
        algorithm_setting.gy_bias = get_gy_offset();
        algorithm_setting.gz_bias = get_gz_offset();
        algorithm_setting.gx_scale = 4000.0f / 65536.0f; // ±2000 dps量程
        algorithm_setting.gy_scale = 4000.0f / 65536.0f;
        algorithm_setting.gz_scale = 4000.0f / 65536.0f;
    }
    else
    {
        // ADXRS645陀螺仪设置
        algorithm_setting.gx_bias = 0.0f;
        algorithm_setting.gy_bias = 0.0f;
        algorithm_setting.gz_bias = 4023000.0f;       // 零偏值
        algorithm_setting.gx_scale = 1.0f / 16110.0f; // scale系数
        algorithm_setting.gy_scale = 1.0f / 16110.0f;
        algorithm_setting.gz_scale = 1.0f / 16110.0f;
    }

    /* 多项式拟合参数初始化 */
    // 读取所有加速度计和陀螺仪的五阶拟合系数 (共6个轴，每个轴6个系数)
    // 多项式系数读取相关局部变量 25/08/31 Gordon
    double all_coefficients[36];             // 所有轴的多项式拟合系数数组 25/08/31 Gordon
    int8_t degree;                           // 多项式阶数（虽然不再使用，但为了匹配函数接口需要声明） 25/08/31 Gordon

    is25pl032_flash_get_degree(all_coefficients, &degree);

    // 将读取到的系数分别映射到 algorithm_setting 中
    // 加速度计系数 (XYZ轴)
    memcpy(algorithm_setting.degrees_acc, all_coefficients, 3 * 6 * sizeof(double));
    // 陀螺仪系数 (XYZ轴)
    memcpy(algorithm_setting.degrees_gyro, all_coefficients + 3 * 6, 3 * 6 * sizeof(double));

    /* 温度补偿范围设置 */
    algorithm_setting.t_comp_lower_limit = get_temp_comp_lower_limit(); // 温度补偿下限，默认值为-20°C
    algorithm_setting.t_comp_upper_limit = get_temp_comp_upper_limit(); // 温度补偿上限，默认值为150°C

    if (algorithm_setting.acc_sensor_type == SIGNAL_PROCESS_ACC_HT20680)
    { // IAM-20680HT:(sensor_signal.t_raw - 0) / 326.8f + 25.0f
        algorithm_setting.t_scale = 0.003059976f;
        algorithm_setting.t_intercept = 25.0f;
    }
    else if (algorithm_setting.acc_sensor_type == SIGNAL_PROCESS_ACC_MINIQ)
    { // mini-Q: (sensor_signal.t_raw * 2500.0 / 8388608) * 1.27 * 1.0052 - 276.35;
        algorithm_setting.t_scale = 0.0003804576f;
        algorithm_setting.t_intercept = -276.35f;
    }
    else
    { // VS1005或其他型号: (sensor_signal.t_raw * (-0.00015421537061f) +330.f) * 1.06557377f;
        algorithm_setting.t_scale = -0.000164327854f;
        algorithm_setting.t_intercept = 321.9705002;
    }

}

/**
  *******************************************************************************
  * @Description: 主任务
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 22:06:02 Sunday
  *******************************************************************************
  */
// ==================== 主任务函数 ====================
void main_task(void *p)
{
    downhole = 1;                           // 设置井下状态为1 25/08/31 Gordon
    TimerHandle_t waiting_for_uart2_timeout_tmr;  // UART2超时等待定时器句柄 25/08/31 Gordon

    main_task_handle = xTaskGetCurrentTaskHandle();
    uint32_t start_ticket = xTaskGetTickCount();

    lpuart1_rx_timer = xTimerCreate("lpuart_rx_timer", 10, 0, 0, timer_lpuart1_rx_cb);
    lpuart2_rx_timer = xTimerCreate("lpuart2_rx_timer", 10, 0, 0, lpuart2_rx_timer_cb);

    init_on_board_peripheral();

    // 加载配置
    load_algorithm_setting_from_flash();
    // 从Flash配置中加载泥浆脉冲相关参数
    pump_off_data_send_retry_count = is25pl032_flash_get_pulse_retry_for_pump_off_data();
    pump_off_data_send_interval = is25pl032_flash_get_pulse_interval_for_pump_off_data() * 10 ; // 转换为100ms单位
    mud_pulse_send_interval = is25pl032_flash_get_pulse_interval() * 10; // 转换为100ms单位
    invalid_pump_off_inc_count1 = is25pl032_flash_get_pump_delay1() * 10; // 转换为100ms单位
    invalid_pump_off_inc_count2 = is25pl032_flash_get_pump_delay2() * 10; // 转换为100ms单位

    // 如果使用20680陀螺仪芯片，则启动20680任务
    if (algorithm_setting.acc_sensor_type == SIGNAL_PROCESS_ACC_HT20680 || algorithm_setting.gyro_sensor_type == SIGNAL_PROCESS_GYRO_HT20680)
    {
        xTaskCreate(iam_20680ht_task, "iam_20680ht_task", 128, NULL, TASK_PRIORITY_IAM, &iam_20680ht_task_handle);
    }
    // 启动信号处理任务
    signal_process_init(); //滤波器初始化处理也在其中
    xTaskCreate(signal_process_task, "signal_process_task", 512, NULL, TASK_PRIORITY_SIGNAL_PROCESS, &signal_process_task_handle);
    // 启动算法任务
    xTaskCreate(ie_task, "ie_task", 640, NULL, TASK_PRIORITY_IE, &ie_task_handle);

    prepare_data_transmission(0); // 0 for dynamic data
    pulser_start_tx(pulser_tx_buffer, pulser_tx_buffer_len);
    // 处理上位机事件
    waiting_for_uart2_timeout_tmr = xTimerCreate("uart2_timeout_tmr", 20 * 1000, 0, 0, timer_waiting_for_uart2_timeout_cb);
    xTimerStart(waiting_for_uart2_timeout_tmr, 1000);
    xTimerStart(xTimerCreate("second_timer", 100, 1, 0, timer_100ms_cb), 1000);
    for (;;)
    {
        uint32_t notify;
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if (notify & EVENT_UART2_TIMEOUT)
        {
            if (waiting_for_uart2_timeout_tmr)
            {
                xTimerDelete(waiting_for_uart2_timeout_tmr, 0);
                waiting_for_uart2_timeout_tmr = 0;
            }
            // LPUART2_deinit();
            // break;
        }
        if (downhole > 0 && xTaskGetTickCount() - start_ticket > 20 * 1000)
        {
            break; // 20秒后退出等待循环，进入井下模式
        }

        if (notify & EVENT_UART2_RX)
        {
            if (waiting_for_uart2_timeout_tmr)
            {
                xTimerDelete(waiting_for_uart2_timeout_tmr, 0);
                waiting_for_uart2_timeout_tmr = 0;
            }

            handle_uart_msg(UART2_rx_buffer, &UART2_rx_data_len);
        }

        if (notify & EVENT_TIMER_100MS)
        {
            PCA8565_on_timer_event();
            prepare_data_transmission(0); // 0 for dynamic data
        }
        if (notify & EVENT_UART1_RX)
            handle_uart_msg(UART1_rx_buffer, &UART1_rx_data_len);
    }

    xTimerStart(xTimerCreate("timer_send_dbg_data_cb", 100, 1, 0, timer_send_dbg_data_cb), 1000);

    // 初始化泥浆脉冲，执行双脉冲阶段设置传输
    pulser_tx_buffer_len = 0;
    pulser_tx_buffer[pulser_tx_buffer_len++] = MUD_PULSE_TIMER_HZ * 1;   //LOW
    pulser_tx_buffer[pulser_tx_buffer_len++] = MUD_PULSE_TIMER_HZ * 1;   //HIGH
    pulser_tx_buffer[pulser_tx_buffer_len++] = MUD_PULSE_TIMER_HZ * 4;   //LOW
    pulser_tx_buffer[pulser_tx_buffer_len++] = MUD_PULSE_TIMER_HZ * 1;   //HIGH
    for(int i = 0; i < 16; i++)
        pulser_tx_buffer[i * 2 + 1] = MUD_PULSE_TIMER_HZ * 1;  //设置高电平的时间
    pulser_start_tx(pulser_tx_buffer, pulser_tx_buffer_len);

    // Enter main loop（井下模式）
    printf("Enter main loop!\r\n");
    for (;;)
    {
        uint32_t notify;
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if (notify & EVENT_TIMER_100MS)
            on_100ms_timer_event();

        if (notify & EVENT_SEND_DBG_DATA_TIMER)
        {
            on_send_debug_data_event();
        }
    }
}

// ==================== 泥浆脉冲相关函数 ====================
/**
 *******************************************************************************
 * @Description: 准备动态数据发送（参考Origin实现）
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 准备动态井斜、温度、高边、电压等数据到发送缓冲区
 * @CreatedBy  : Gordon Li
 * @CreatedDate: 2025-01-27
 *******************************************************************************
 */
static void prepare_data_transmission(int type) //type: 0.动态数据 1.静态数据
{
    uint32_t index = 0;
    float temp_var;

    // 井斜数据（实时数据）
    if(type == 1 && pump_off_inc != -1)
        temp_var = fabs(pump_off_inc);
    else 
        temp_var = fabs(inc_hs_data.good_inc);

    if(temp_var > 8.0f)
        temp_var = 8.0f;
    
    // 脉冲检测头：2秒低电平 + 2秒高电平（用于检测脉冲组开始，区别于原来的连续循环发送）
    pulser_tx_buffer[index++] = 100 * 2;  // 检测头1：2秒低电平
    pulser_tx_buffer[index++] = 100 * 2;  // 检测头2：2秒高电平

    // 脉冲同步头：两个35秒（34秒低 + 1秒高）
    pulser_tx_buffer[index++] = 100 * 34;  // 同步头1：34秒低电平
    pulser_tx_buffer[index++] = 100 * 1;   // 同步头1：1秒高电平
    pulser_tx_buffer[index++] = 100 * 34;  // 同步头2：34秒低电平
    pulser_tx_buffer[index++] = 100 * 1;   // 同步头2：1秒高电平
    pulser_tx_buffer[index++] = 100 * 23 + (temp_var * 100) / 2;  // 井斜
    pulser_tx_buffer[index++] = 100 * 1;   // HIGH

    // 温度数据（实时数据）
    temp_var = sensor_data.t_C;
    if(temp_var < 0.0f)
        temp_var = 0.0f;
    if(temp_var > 180.0f)
        temp_var = 180.0f;
    pulser_tx_buffer[index++] = 100 * 19 + 8.0f * 100 * temp_var / 180.0f;  // 温度
    pulser_tx_buffer[index++] = 100 * 1;   // HIGH

    // 高边数据（实时数据）
    inclination_hs_t hs;
    get_inc_hs(&hs);
    pulser_tx_buffer[index++] = 100 * 19 + 8.0f * 100 * (360.0f - hs.hs_lpf) / 360.0f;  // 高边
    pulser_tx_buffer[index++] = 100 * 1;   // HIGH

    // 电压数据（实时数据）
    temp_var = vSupply;
    if(temp_var < 20.0f)
        temp_var = 20.0f;
    if(temp_var > 36.0f)
        temp_var = 36.0f;
    pulser_tx_buffer[index++] = 100 * 19 + 100 * (temp_var - 20.0f) / 2.0f;  // 电压
    pulser_tx_buffer[index++] = 100 * 1;   // HIGH

    pulser_tx_buffer_len = index;

}

// ==================== 振动检测相关函数 ====================
/**
 *******************************************************************************
 * @Description: GPIO振动检测函数
 * @Parameters : 无
 * @RetValue   : 振动状态（1-检测到振动，0-未检测到振动）
 * @Note       : 使用移位寄存器方式检测GPIO状态变化
 *               检测PTA12引脚状态，连续8次高电平表示振动
 * @CreatedBy  : Assistant
 * @CreatedDate: 2025.01.27
 *******************************************************************************
 */
static uint8_t check_gpio_vibration(void)
{
    static uint8_t gpio_state = 0;
    
    // 移位寄存器方式检测GPIO状态变化
    gpio_state <<= 1;
    if (PINS_DRV_ReadPins(PTA) & (0x1 << 12))
        gpio_state |= 1;
    
    // 连续8次高电平表示振动，连续8次低电平表示无振动
    if (gpio_state == 0xff)
        return 1;  // 检测到振动
    else if (gpio_state == 0)
        return 0;  // 未检测到振动
    
    // 保持之前的状态
    return (gpio_state & 0x80) ? 1 : 0;
}

