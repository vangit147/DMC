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
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#include "mud_pulse.h"
#include "main_task.h"
#include "ie_task.h"
#include "IS25LP032_flash.h"
#include "signal_process.h" // 用于访问sensor_data_t结构体定义

static TaskHandle_t main_task_handle;
#define EVENT_TIMER_100MS 0X1
#define EVENT_SEND_DBG_DATA_TIMER 0X2
#define EVENT_UART2_RX 0X4
#define EVENT_UART2_TIMEOUT 0X08

#define EVENT_UART1_RX 0X10

#define IS_TEMPERATURE_OFFSET 1
#define IS_ROLL_PITCH 1
#define OFFSET_LEN 8

// 声明外部变量
extern interval_info_t interval_info;
extern sensor_data_t sensor_data;    // 用于存储传感器数据
extern inclination_hs_t inc_hs_data; // 倾角高边结构体变量
extern adxl357_vibration_data_t vibration_data;

// 串口
static uint8_t UART1_rx_buffer[128];
static uint32_t UART1_rx_data_len;
static TimerHandle_t lpuart1_rx_timer;

static uint8_t UART2_rx_buffer[128];
static uint32_t UART2_rx_data_len;
static TimerHandle_t lpuart2_rx_timer;

static int8_t downhole;

static float sum_roll;
float trans_ie;
float rpm;

static float s_f32_36V;             // ADC0_SE2  PTA6  36V监测
static uint8_t s_u8_vibrating_flag; // PTA12 1--震动 0--非震动

/******************************** Functions **********************************/
float get_36V_voltage(void)
{
    return s_f32_36V;
}

uint32_t get_vibrating_flag(void)
{
    return s_u8_vibrating_flag;
}

void start_and_get_adc_result(void)
{
    uint16_t u16_ADC_raw_result;

    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig0);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &u16_ADC_raw_result);

    // 电压缩放系数:21
    s_f32_36V = (float)u16_ADC_raw_result * 3.300f * 21.0f / 4096.0f;
}

void checking_vibrating_gpio(void)
{
    static uint8_t s_u8_gpio_state;

    s_u8_gpio_state <<= 1;
    if (PINS_DRV_ReadPins(PTA) & (0x1 << 12))
        s_u8_gpio_state |= 1;
    if (s_u8_gpio_state == 0xff)
        s_u8_vibrating_flag = 1;
    else if (s_u8_gpio_state == 0)
        s_u8_vibrating_flag = 0;
}

void set_downhole(int val)
{
    downhole = val;
}

int8_t get_downhole()
{
    return downhole;
}
/**
  *******************************************************************************
  * @Description: 应用层钩子函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.01.27 14:29:03 Saturday
  *******************************************************************************
  */
#if (configUSE_TICK_HOOK > 0)
void vApplicationTickHook(void)
{
}
#endif
/**
  *******************************************************************************
  * @Description: 定时器通道1中断处理函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.01.28 00:19:16 Sunday
  *******************************************************************************
  */
static int32_t flextimer_mc1_isr(void)
{
    mud_pulse_timer_isr(&mud_pulse);
    return 0;
}

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

    if (data == '\n')
        return;
    // Convert to Uppercase
    if (data >= 'a' && data <= 'z')
        data -= 32;
    if (UART1_rx_data_len < sizeof(UART1_rx_buffer))
        UART1_rx_buffer[UART1_rx_data_len++] = (uint8_t)data;
    if ((data == '\r' || UART1_rx_data_len >= sizeof(UART1_rx_buffer)) && main_task_handle)
        xTaskGenericNotifyFromISR(main_task_handle, EVENT_UART1_RX, eSetBits, NULL, NULL);
    else if (lpuart1_rx_timer)
    {
        // xTimerChangePeriodFromISR(lpuart1_rx_timer, 50, 0);
    }
}
/**
  *******************************************************************************
  * @Description: 串口2发送完成回调函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
    LPUART1_init(lpuart1_tx_cb, lpuart1_rx_cb);
    LPUART2_init(lpuart2_tx_cb, lpuart2_rx_cb);

    i2c_init();

    /*对于V1.2
    SPI0 CS0 ---- IS25LP032D
    SPI0 CS1 ---- IAM 20680HT
    SPI0 CS2 ---- ADXL357BEZ
    SPI1     ---- Reserved
    SPI2     ---- ADS1278HPAP
    */
    spi0_init();
    spi1_init();
    spi2_init();

    gpio_port_init();
    flextime_mc1_init(flextimer_mc1_isr);
    return 0;
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
    is25pl032_flash_init();
    pca8565_init();

    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);
    ADC_DRV_AutoCalibration(INST_ADCONV1);

    return 0;
}
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
    uint32_t tail_flag = 0x7f800000;
    float debug_data[20];
    int32_t  counter = 0;

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
    static uint32_t rtc_timeout = 0;
    static uint32_t last_timestamp = 0;
    static uint32_t log_period = 5;
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

    // 启动ADC，获取36V电压
    start_and_get_adc_result();

    // 获取震动状态
    checking_vibrating_gpio();
#ifndef ADXL357_VIBRATION_TEST
    checking_vibrating_adxl357();
#endif

    // 每秒更新一次数据
    rtc_timeout++;
    if (last_timestamp != timestamp || rtc_timeout > 10)
    {
        int32_t ret;
        // 清除计数
        rtc_timeout = 0;
        last_timestamp = timestamp;

        // 更新各个温度下运行时间
        update_total_time_per_temp(sensor_data.t_C);

        // 更新泥浆脉冲数据
        if (!mud_pulse.state.double_stage)
        {
            uint8_t currentMotionState = 0;

            // 在震动时才发送泥浆脉冲
            if (get_vibrating_flag())
                currentMotionState = 1;

            mud_pulse_update_data(&mud_pulse, currentMotionState);
        }

        // 保存LOG
        if (log_period)
            log_period--;
        if (log_period == 0)
        {
            log_t log = {0};
            log_period = algorithm_setting.log_period_time;

            // 调用get_sensor_data函数获取传感器数据
            get_sensor_data(&sensor_data);

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
            log.hs = inc_hs_data.hs;

            // log.roll = sum_roll / 1000;
            // log.diff_t = 0;

            // 从interval_info_data中获取虚拟半径数据
            log.virtual_x_radius_min = interval_info.radius_x_min;
            log.virtual_x_radius_max = interval_info.radius_x_max;
            // log.virtual_x_radius_avg = 0; // 如果interval_info中没有平均值，则设为0

            log.virtual_y_radius_min = interval_info.radius_y_min;
            log.virtual_y_radius_max = interval_info.radius_y_max;
            // log.virtual_y_radius_avg = 0; // 如果interval_info中没有平均值，则设为0

            // 从interval_info_data中获取标准差数据
            log.gz_dps_sdv_max = interval_info.sdv_gyro_z_max;
            log.gz_dps_sdv_min = interval_info.sdv_gyro_z_min;
            log.std_dev_ax_ay_max = interval_info.std_dev_ax_ay_max; // 如果interval_info中没有这些数据，则设为0
            if (get_vibrating_flag())
                log.flag |= LOG_FLAG_VIBRATING;
            if (get_adxl357_vibrating_flag())
                log.flag |= LOG_FLAG_VIBRATING_FOR_ADXL357;

            log.vibration_data_min = vibration_data.min_vibration;
            log.vibration_data_max = vibration_data.max_vibration;
            log.vibration_data_avg = vibration_data.avg_vibration;
            printf("vibration_data.min_vibration=%f,vibration_data.max_vibration=%f,vibration_data.avg_vibration=%f\r\n", vibration_data.min_vibration, vibration_data.max_vibration, vibration_data.avg_vibration);
            Reset_Vibration_Stats();

            log.s_f32_36V = get_36V_voltage();
            log.max_peace_time_max = interval_info.peace_time_max;
            log.peace_time_count = interval_info.peace_time_count;
            // log.max_peace_time_min = 0; // 如果interval_info中没有最小值，则设为0

            // 从interval_info_data中获取数据质量等级统计
            log.c0_num_max = interval_info.c0_num_count;
            // log.c0_num_min = 0; // 如果interval_info中没有最小值，则设为0
            log.c1_num_max = interval_info.c1_num_count;
            // log.c1_num_min = 0;
            log.c2_num_max = interval_info.c2_num_count;
            // log.c2_num_min = 0;

            if (log.inc6_avg > 8)
            {
                trans_ie = 8;
            }
            else
            {
                trans_ie = log.inc6_avg;
            }

            reset_interval_info();

            sum_roll = 0;

            if ((ret = is25pl032_flash_write_one_log(&log)) != 0)
                printf("Writing ONE log failed! ret=%d\r\n", ret);
        }
    }
}

/**
 *******************************************************************************
 * @Description: 查找是否收到VD下发的命令帧
 * @Parameters :
 * @RetValue   :
 * @Note       :
 * @CreatedBy  : NY
 * @CreatedDate: 2024.09.25 06:54:08 Wednesday
 *******************************************************************************
 */
static bool check_vd_cmd(uint8_t *rx_data, uint32_t len, uint32_t *be_cleared_data)
{
    static const uint8_t modbus_rd_data_cmd[] = {0x10, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc7, 0x4a};
    static uint8_t data_buffer[32];
    static uint8_t data_len;

    bool found = false;
    uint8_t *p_head = data_buffer;

    __disable_irq();
    if (len > sizeof(data_buffer) - data_len)
        len = sizeof(data_buffer) - data_len;
    memcpy(data_buffer + data_len, rx_data, len);
    data_len += len;
    if (be_cleared_data)
        *be_cleared_data = 0;
    __enable_irq();

    while (data_len >= sizeof(modbus_rd_data_cmd))
    {
        p_head = data_buffer + data_len - sizeof(modbus_rd_data_cmd);
        if (memcmp(p_head, modbus_rd_data_cmd, sizeof(modbus_rd_data_cmd)) == 0)
        {
            found = true;
            data_len -= sizeof(modbus_rd_data_cmd);
            p_head += sizeof(modbus_rd_data_cmd);
            break;
        }
        data_len--;
        p_head++;
    }
    if (p_head != data_buffer && data_len)
        memmove(data_buffer, p_head, data_len);

    return found;
}

void algorithm_setting_for_Calibration()
{
    // 设置传感器类型
    // 加速度计类型
    algorithm_setting.acc_sensor_type = get_acc_sensor_type();

    // 陀螺仪类型
    algorithm_setting.gyro_sensor_type = get_gyro_sensor_type();

    // 设置虚拟半径限制
    algorithm_setting.xr_limit = 0.0015f;
    algorithm_setting.yr_limit = 0.015f;
    is25pl032_flash_set_param(0, 0, algorithm_setting.xr_limit, algorithm_setting.yr_limit, 0);

    // 设置日志周期60s
    algorithm_setting.log_period_time = 60;
    is25pl032_flash_set_log_period(algorithm_setting.log_period_time);

    // 设置稳定时间阈值
    algorithm_setting.max_peace_time_threshold = 5;

    double imu[9] = {0, 0, 0, 1, 0, 0, 1, 0, 1};
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
    is25pl032_flash_set_imu(imu);

    /* 加速度计装配误差设置 */
    algorithm_setting.acc_x_offset = 0.0f; // X轴加速度计装配误差，单位：g
    algorithm_setting.acc_y_offset = 0.0f; // Y轴加速度计装配误差，单位：g
    algorithm_setting.acc_z_offset = 0.0f; // Z轴加速度计装配误差，单位：g
    is25pl032_flash_set_offset(0, algorithm_setting.acc_x_offset);
    is25pl032_flash_set_offset(1, algorithm_setting.acc_y_offset);

    // 设置多项式拟合参数
    memset(algorithm_setting.degrees_acc, 0, sizeof(algorithm_setting.degrees_acc));
    is25pl032_flash_set_degree(0, algorithm_setting.degrees_acc, 5);
    is25pl032_flash_set_degree(1, algorithm_setting.degrees_acc + 6, 5);
    is25pl032_flash_set_degree(2, algorithm_setting.degrees_acc + 6, 5);

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

    // 设置多项式拟合参数
    memset(algorithm_setting.degrees_gyro, 0, sizeof(algorithm_setting.degrees_gyro));
    is25pl032_flash_set_degree(3, algorithm_setting.degrees_gyro, 5);
    is25pl032_flash_set_degree(4, algorithm_setting.degrees_gyro + 6, 5);
    is25pl032_flash_set_degree(5, algorithm_setting.degrees_gyro + 6, 5);

    // 设置温度补偿范围
    algorithm_setting.t_comp_lower_limit = -20.0f;
    set_temp_comp_lower_limit(algorithm_setting.t_comp_lower_limit);
    algorithm_setting.t_comp_upper_limit = 150.0f;
    set_temp_comp_upper_limit(algorithm_setting.t_comp_upper_limit);

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
        //        algorithm_setting.t_intercept = 351.64f;
        algorithm_setting.t_intercept = 321.9705002;
    }
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
    uint8_t accfir, gyrofir;
    double gain;
    double xr_limit, yr_limit;
    is25pl032_flash_get_param(&accfir, &gyrofir, &xr_limit, &yr_limit, &gain);
    algorithm_setting.xr_limit = xr_limit;
    algorithm_setting.xr_limit = yr_limit;
    // 确保虚拟半径限制参数有效
    if (algorithm_setting.yr_limit <= 0)
    {
        algorithm_setting.yr_limit = 0.1f; // 默认Y轴限制为1米，按照g
    }
    if (algorithm_setting.xr_limit <= 0)
    {
        algorithm_setting.xr_limit = 0.01f; // 默认X轴限制为0.002米，按照g要缩小10被
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

    double imu[9];
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
    double all_coefficients[36];
    int8_t degree; // 虽然不再使用，但为了匹配函数接口需要声明

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

#if 0
    /* 加速度计零偏设置 */
    algorithm_setting.acc_x_bias = 0.0005133800f;  // X轴加速度计零偏，单位：g
    algorithm_setting.acc_y_bias = 0.0323904000f;  // Y轴加速度计零偏，单位：g
    algorithm_setting.acc_z_bias = -0.0024774900f; // Z轴加速度计零偏，单位：g

    /* 加速度计MS矩阵偏差设置 */
    algorithm_setting.ms_xx = 0.000000629696f;  // X-X轴MS矩阵系数
    algorithm_setting.ms_xy = 0.0f;  // X-Y轴MS矩阵系数
    algorithm_setting.ms_xz = 0.0f; // X-Z轴MS矩阵系数
    algorithm_setting.ms_yy = 0.000000630076f;  // Y-Y轴MS矩阵系数
    algorithm_setting.ms_yz = 0.0f;  // Y-Z轴MS矩阵系数
    algorithm_setting.ms_zz = 0.000000628612f;  // Z-Z轴MS矩阵系数
#endif
}

/**
 *******************************************************************************
 * @Description: 测试用配置设置函数：misc_config中的数据硬编码写入algorithm_setting
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 用于测试，将misc_config中的数据硬编码写入algorithm_setting
 *******************************************************************************
 */
static void setting_for_test(void)
{
    // 设置传感器类型
    algorithm_setting.acc_sensor_type = SIGNAL_PROCESS_ACC_VS10XX;
    algorithm_setting.gyro_sensor_type = SIGNAL_PROCESS_GYRO_HT20680;

    // 设置虚拟半径限制
    algorithm_setting.xr_limit = 0.00015f;
    algorithm_setting.yr_limit = 0.0015f;

    // 设置日志周期60s
    algorithm_setting.log_period_time = 60;

    // 设置稳定时间阈值
    algorithm_setting.max_peace_time_threshold = 5;

    // 设置加速度计MS矩阵
    algorithm_setting.ms_xx = 5.5587055e-7;
    algorithm_setting.ms_xy = 0.0f;
    algorithm_setting.ms_xz = 0.0f;
    algorithm_setting.ms_yy = 5.5587055e-7;
    algorithm_setting.ms_yz = 0.0f;
    algorithm_setting.ms_zz = 5.5587055e-7;

    // 设置加速度计零偏
    algorithm_setting.ax_bias = 0.0f;
    algorithm_setting.ay_bias = 0.0f;
    algorithm_setting.az_bias = 0.0f;

    // 设置加速度计装配误差
    algorithm_setting.acc_x_offset = 0.0f;
    algorithm_setting.acc_y_offset = 0.0f;
    algorithm_setting.acc_z_offset = 0.0f;

    // 设置陀螺仪零偏和scale
    if (algorithm_setting.gyro_sensor_type == SIGNAL_PROCESS_GYRO_HT20680)
    {
        // IAM-20680HT陀螺仪设置
        algorithm_setting.gx_bias = 0.0f;
        algorithm_setting.gy_bias = 0.0f;
        algorithm_setting.gz_bias = 0.0f;
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

    // 设置多项式拟合参数
    memset(algorithm_setting.degrees_acc, 0, sizeof(algorithm_setting.degrees_acc));

    // 设置温度补偿范围
    algorithm_setting.t_comp_lower_limit = -20.0f;
    algorithm_setting.t_comp_upper_limit = 150.0f;

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
        //        algorithm_setting.t_intercept = 351.64f;
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
void main_task(void *p)
{
    sum_roll = 0;
    downhole = 1;
    TimerHandle_t waiting_for_uart2_timeout_tmr;

    main_task_handle = xTaskGetCurrentTaskHandle();
    uint32_t start_ticket = xTaskGetTickCount();

    lpuart1_rx_timer = xTimerCreate("lpuart_rx_timer", 10, 0, 0, timer_lpuart1_rx_cb);
    lpuart2_rx_timer = xTimerCreate("lpuart2_rx_timer", 10, 0, 0, lpuart2_rx_timer_cb);

    init_on_board_peripheral();

    // 加载配置
    load_algorithm_setting_from_flash();
    // 测试用配置设置
    // setting_for_test();

    // 初始化泥浆脉冲
    mud_pulse_init(&mud_pulse);

    if (algorithm_setting.acc_sensor_type == SIGNAL_PROCESS_ACC_HT20680 || algorithm_setting.gyro_sensor_type == SIGNAL_PROCESS_GYRO_HT20680)
    {
        xTaskCreate(iam_20680ht_task, "iam_20680ht_task", 256, NULL, TASK_PRIORITY_IAM, &iam_20680ht_task_handle);
    }
    // 启动信号处理任务
    signal_process_init();
    xTaskCreate(signal_process_task, "signal_process_task", 256, NULL, TASK_PRIORITY_SIGNAL_PROCESS, &signal_process_task_handle);
    // 启动算法任务
    xTaskCreate(ie_task, "ie_task", 640, NULL, TASK_PRIORITY_IE, &ie_task_handle);

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
            set_downhole(2);
            break;
        }

        if (notify & EVENT_UART2_RX)
        {
            if (waiting_for_uart2_timeout_tmr)
            {
                xTimerDelete(waiting_for_uart2_timeout_tmr, 0);
                waiting_for_uart2_timeout_tmr = 0;
            }

            // 检查是否收到命令
            if (check_vd_cmd(UART2_rx_buffer, UART2_rx_data_len, NULL))
            {
                printf("Received MODBUS read command!\r\n");
                UART2_rx_data_len = 0;
                if (waiting_for_uart2_timeout_tmr)
                {
                    xTimerDelete(waiting_for_uart2_timeout_tmr, 0);
                    waiting_for_uart2_timeout_tmr = 0;
                }
                break;
            }

            handle_uart_msg(UART2_rx_buffer, &UART2_rx_data_len);
        }

        if (notify & EVENT_TIMER_100MS)
        {
            // 井上跑这个函数会导致在井上时，日志也会存储
            // on_100ms_timer_event();
            PCA8565_on_timer_event();
        }
        if (notify & EVENT_UART1_RX)
            handle_uart_msg(UART1_rx_buffer, &UART1_rx_data_len);
    }

    xTaskCreate(adxl357_task, "adxl357_task", 256, NULL, TASK_PRIORITY_ADXL357, &adx357_task_handle);
    // 启动泥浆脉冲
    mud_pulse_start_tx(&mud_pulse);
    xTimerStart(xTimerCreate("timer_send_dbg_data_cb", 100, 1, 0, timer_send_dbg_data_cb), 1000);

    // Enter main loop
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

        if (notify & EVENT_UART2_RX)
        {
            uint8_t temp[10];
            int16_t temp_v;
            uint16_t crc;

            if (check_vd_cmd(UART2_rx_buffer, UART2_rx_data_len, &UART2_rx_data_len))
            {
                temp[0] = 0x10;
                temp[1] = 0x03;
                temp[2] = 4;
                temp_v = trans_ie * 100;
                temp[3] = temp_v >> 8;
                temp[4] = temp_v & 0xff;

                temp_v = rpm * 100 / 6;
                if (temp_v < 0)
                    temp_v = -temp_v;
                if (temp_v > 10000)
                    temp_v = 10000;
                temp[5] = temp_v >> 8;
                temp[6] = temp_v & 0xff;

                crc = CRC16(temp, 7);
                temp[7] = crc;
                temp[8] = crc >> 8;
                LPUART2_send(temp, 9);
            }
        }
    }
}
