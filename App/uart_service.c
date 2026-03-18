/**
 *******************************************************************************
 * @file    uart_service.c
 * @brief   UART服务模块，负责处理串口通信、数据发送和接收
 * @author  YangHaifeng
 * @date    2023.10.07
 * @version v1.0
 * @note    该模块实现了以下功能：
 *          1. 串口数据发送和接收
 *          2. 设备信息查询和设置
 *          3. 传感器数据采集和传输
 *          4. 日志记录和管理
 *          5. 参数配置和校准
 *******************************************************************************
 */

#include "main.h"
#include "uart_service.h"
#include "main_task.h"
#include "ie_task.h"

// 外部函数声明
extern void send_cutter_valve_test_pulse(uint8_t test_type);
#include "ads1278_imu.h"
#include "IAM_20680HT.h"
#include "signal_process.h"

// 外部变量声明
extern sensor_data_t sensor_data;
extern ads1278_global_raw_data_t s_ads1278_global_raw_data;
extern iam_global_raw_data_t s_iam_global_raw_data;
extern inclination_hs_t inc_hs_data;
extern int8_t downhole;
extern float vSupply;

#define RECEIVE_MSG_LEN 120
static uint8_t receiveMsg[RECEIVE_MSG_LEN];
static uint8_t isSend;
static uint8_t output_buffer[100];


/******************************** Functions **********************************/
/**
 * @brief  向可视化工具发送数据
 * @param  data: 要发送的数据缓冲区
 * @param  len: 数据长度
 * @note   通过LPUART0发送数据到可视化工具
 */
static void send_data_to_vd_tool(void *data, uint32_t len)
{
    LPUART0_send(data, len);
}

/**
 * @brief  处理接收到的UART消息
 * @param  msg: 接收到的消息缓冲区
 * @param  msg_len: 消息长度
 * @note   处理各种命令，包括：
 *         1. MZA=? - 查询设备信息
 *         2. MZB=? - 查询原始传感器数据
 *         3. UD= - 设置RTC时间
 *         4. FS - 重置Flash读取索引
 *         5. FL - 删除所有日志
 *         6. FU - 设置日志周期
 *         7. FA=? - 读取一条日志
 *         8. KP= - 设置偏移量
 *         9. UX= - 设置参数
 *         10. AXB/GXB等 - 设置多项式拟合系数
 */
static void uart_proc_received_msg(uint8_t *msg, uint32_t msg_len)
{
    /* printf("%s", msg); */
    // 消息长度至少为2个有效字节，且以回车结束。
    if (msg_len < 3 || msg[msg_len - 1] != '\r')
        //  goto error;
        return;
    downhole = 0;
    msg[msg_len - 1] = 0;
    msg_len--;
    uint8_t k = 0;
    for (; k < RECEIVE_MSG_LEN; k++)
        receiveMsg[k] = 0;
    memcpy(receiveMsg, msg, msg_len);
    isSend = 1;
    send_msg();
}

/**
 * @brief  将字节数组转换为double类型
 * @param  bytes: 字节数组
 * @return 转换后的double值
 */
double bytesToDouble(unsigned char *bytes)
{
    double num;
    memcpy(&num, bytes, sizeof(double));
    return num;
}

uint8_t *toOutPutBuffer(uint8_t *p_uint8, uint8_t *byteArr, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        *p_uint8++ = *byteArr++;
    }
    return p_uint8;
}
char *findEndByFlag(char *input, char *output, uint8_t size, char flag)
{
    for (uint8_t i = 0; i < size; i++)
    {
        if (*input == flag)
        {
            break;
        }
        *output++ = *input++;
    }
    return input;
}
char *findEnd(char *input, char *output, uint8_t size)
{

    return findEndByFlag(input, output, size, 0x00);
}

void convert(char *input, uint8_t startIndex, uint8_t size, char *output)
{
    char *value = input + startIndex;
    for (uint8_t i = 0; i < size; i++)
    {
        *output++ = *value++;
    }
}
int32_t toInt32(char *input, uint8_t startIndex, uint8_t size)
{
    char temp[size];
    convert(input, startIndex, size, temp);
    return atoi((const char *)temp);
}

float toFloat(char *input, uint8_t startIndex, uint8_t size)
{
    char temp[size];
    convert(input, startIndex, size, temp);
    return atof((const char *)temp);
}
void cleanArray(uint8_t *array, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        *array++ = 0;
    }
}

void send_msg(void)
{
    uint8_t *p_uint8 = output_buffer;
    uint8_t temp_data[15+VERSION_MAX_LENGTH]; // 在函数开始处定义
    float temp = 0.0f; // 添加全局temp变量用于占位符
    uint32_t sendLength = 0;
    
    if (isSend < 1)
    {
        return;
    }
    
    if (receiveMsg[0] == 'M' && receiveMsg[1] == 'Z' && receiveMsg[2] == 'A' && receiveMsg[3] == '=' && receiveMsg[4] == '?')
    {

        char pro_id[15];
        char version[VERSION_MAX_LENGTH];
        uint8_t rtc_data[8];
        uint32_t sec, min, hours, day, mon, year;
        float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temperature;
        float ie1, ie2, ie6, hs;
        float y_radius, ofx, ofy, x_radius;
        double yr_limit, xr_limit;
        float inc1_r, inc1_p, inc3_r, inc3_p;

        PCA8565_get_data(rtc_data, 7);
        sec = (rtc_data[0] >> 4 & 0x7) * 10 + (rtc_data[0] & 0xf);
        min = (rtc_data[1] >> 4 & 0x7) * 10 + (rtc_data[1] & 0xf);
        hours = (rtc_data[2] >> 4 & 0x3) * 10 + (rtc_data[2] & 0xf);
        day = (rtc_data[3] >> 4 & 0x3) * 10 + (rtc_data[3] & 0xf);
        mon = (rtc_data[5] >> 4 & 0x1) * 10 + (rtc_data[5] & 0xf);
        year = (rtc_data[6] >> 4 & 0xf) * 10 + (rtc_data[6] & 0xf);

        // 从sensor_data结构体获取数据，替换原来的iam_filtered_data
        temperature = sensor_data.t_C;
        acc_x = sensor_data.ax_g;
        acc_y = sensor_data.ay_g;
        acc_z = - sensor_data.az_g;
        gyro_x = sensor_data.gx_dps;
        gyro_y = sensor_data.gy_dps;
        gyro_z = sensor_data.gz_dps;

        // 使用全局变量inc_hs_data
        ie1 = inc_hs_data.inc1;
        ie2 = inc_hs_data.inc2;
        ie6 = inc_hs_data.good_inc;
        hs = inc_hs_data.hs_lpf;

        y_radius = algorithm_data.ry;
        ofx = algorithm_setting.acc_x_offset;
        ofy = algorithm_setting.acc_y_offset;
        x_radius = algorithm_data.rx;
        is25pl032_flash_get_param(&xr_limit, &yr_limit);

        inc1_r = inc_hs_data.inc1_roll;
        inc1_p = inc_hs_data.inc1_pitch;
        inc3_r = inc_hs_data.inc3_roll;
        inc3_p = inc_hs_data.inc3_pitch;
        uint32_t fu = is25pl032_flash_get_log_period();
        is25pl032_flash_get_proid(pro_id);
        is25pl032_flash_get_version(version);
        for (uint8_t k = 0; k < 100; k++)
        {
            output_buffer[k] = 0;
        }

        memset(temp_data, 0, sizeof(temp_data)); // 使用前清零
        int n = sprintf((char *)temp_data, "%s;%s", pro_id, version);

        *p_uint8++ = 0x55;
        *p_uint8++ = 0x01;
        *p_uint8++ = 0x54;

        *p_uint8++ = 0x01;
        *p_uint8++ = 0x01;

        output_buffer[3] = (121 + n) >> 16 & 0xff;
        output_buffer[4] = (121 + n) & 0xff;
        *p_uint8++ = year >> 24 & 0xff;
        *p_uint8++ = year >> 16 & 0xff;
        *p_uint8++ = year >> 8 & 0xff;
        *p_uint8++ = year >> 0 & 0xff;
        *p_uint8++ = mon >> 24 & 0xff;
        *p_uint8++ = mon >> 16 & 0xff;
        *p_uint8++ = mon >> 8 & 0xff;
        *p_uint8++ = mon >> 0 & 0xff;
        *p_uint8++ = day >> 24 & 0xff;
        *p_uint8++ = day >> 16 & 0xff;
        *p_uint8++ = day >> 8 & 0xff;
        *p_uint8++ = day >> 0 & 0xff;
        *p_uint8++ = hours >> 24 & 0xff;
        *p_uint8++ = hours >> 16 & 0xff;
        *p_uint8++ = hours >> 8 & 0xff;
        *p_uint8++ = hours >> 0 & 0xff;
        *p_uint8++ = min >> 24 & 0xff;
        *p_uint8++ = min >> 16 & 0xff;
        *p_uint8++ = min >> 8 & 0xff;
        *p_uint8++ = min >> 0 & 0xff;
        *p_uint8++ = sec >> 24 & 0xff;
        *p_uint8++ = sec >> 16 & 0xff;
        *p_uint8++ = sec >> 8 & 0xff;
        *p_uint8++ = sec >> 0 & 0xff;

        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&fu, 4);

        *p_uint8++ = n & 0xff; // 版本号字符串长度

        memcpy(p_uint8, temp_data, n); // 版本号
        p_uint8 += n;                  // 34+n

        send_data_to_vd_tool(output_buffer, 34 + n);

        p_uint8 = (uint8_t *)output_buffer;
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&acc_x, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&acc_y, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&acc_z, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&gyro_x, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&gyro_y, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&gyro_z, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temperature, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&ie1, 4);//井斜辅1
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&ie2, 4);//井斜辅2
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&ie6, 4);//井斜主
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&x_radius, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&y_radius, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&ofx, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&ofy, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&xr_limit, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&yr_limit, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&inc1_r, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&inc1_p, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&inc3_r, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&inc3_p, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&hs, 4);

        *p_uint8++ = 0x54;
        *p_uint8++ = 0x01;
        *p_uint8++ = 0x55;

        send_data_to_vd_tool(output_buffer, 87);
        isSend = 0;
    }

    if (receiveMsg[0] == 'M' && receiveMsg[1] == 'Z' && receiveMsg[2] == 'B' && receiveMsg[3] == '=' && receiveMsg[4] == '?')
    {
        double xr_limit, yr_limit;
        double gain = 1.0f;

        float org_ax = filtered_sensor_signal.ax_raw;
        float org_ay = filtered_sensor_signal.ay_raw;
        float org_az = - filtered_sensor_signal.az_raw;
        float org_gx = filtered_sensor_signal.gx_raw;
        float org_gy = filtered_sensor_signal.gy_raw;
        float org_gz = filtered_sensor_signal.gz_raw;
        float org_temp = sensor_signal.t_raw;

        is25pl032_flash_get_param(&xr_limit, &yr_limit);

        float x_offset = is25pl032_flash_get_offset(0);
        float y_offset = is25pl032_flash_get_offset(1);
        uint32_t fu = is25pl032_flash_get_log_period();

        *p_uint8++ = 0x55;
        *p_uint8++ = 0x09;
        *p_uint8++ = 0x54;

        *p_uint8++ = 0x01;
        *p_uint8++ = 0x01;
        output_buffer[3] = 126 >> 16 & 0xff;
        output_buffer[4] = 126 & 0xff;

        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&org_ax, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&org_ay, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&org_az, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&org_gx, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&org_gy, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&org_gz, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&org_temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);

        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 4);
        send_data_to_vd_tool(output_buffer, 89);

        p_uint8 = (uint8_t *)output_buffer;
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 1);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&temp, 1);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&xr_limit, 8);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&yr_limit, 8);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&gain, 8);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&x_offset, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&y_offset, 4);

        *p_uint8++ = 0x54;
        *p_uint8++ = 0x09;
        *p_uint8++ = 0x55;

        send_data_to_vd_tool(output_buffer, 37);
        isSend = 0;
    }

    if (receiveMsg[0] == 'U' && receiveMsg[1] == 'D' && receiveMsg[2] == '=' && receiveMsg[3] != '?')
    {
        char *actual_data = (char *)receiveMsg + 3;

        if (strlen((const char *)actual_data) != 16 || actual_data[2] != '.' || actual_data[5] != '.')
            return;
        uint32_t day = toInt32(actual_data, 0, 2);
        uint32_t mon = toInt32(actual_data, 3, 2);
        uint32_t year = toInt32(actual_data, 6, 2);
        uint32_t hours = toInt32(actual_data, 8, 2);
        uint32_t min = toInt32(actual_data, 11, 2);
        uint32_t sec = toInt32(actual_data, 14, 2);
        PCA8565_set_data(year, mon, day, hours, min, sec);
        goto ok;
    }

    if (receiveMsg[0] == 'F' && receiveMsg[1] == 'S')
    {
        is25pl032_flash_reset_rd_index();
        goto ok;
    }
    if (receiveMsg[0] == 'F' && receiveMsg[1] == 'L')
    {
        is25pl032_flash_delete_all_log();
        goto ok;
    }
    if (receiveMsg[0] == 'F' && receiveMsg[1] == 'U')
    {
        uint32_t val = atoi((const char *)&receiveMsg[3]);
        if (val == 0 || val > 120)
            goto error;
        is25pl032_flash_set_log_period(val);
        goto ok;
    }
    if (receiveMsg[0] == 'F' && receiveMsg[1] == 'A' && receiveMsg[2] == '=' && receiveMsg[3] == '?')
    {
        log_t log;
        int32_t ret = is25pl032_flash_read_one_log(&log);
        if (!ret)
        {
            goto ok;
        }
        struct tm *tm_now;
        uint32_t timestamp = log.timestamp + 3600 * 8;
        tm_now = localtime((time_t *)&timestamp);
        int year = tm_now->tm_year + 1900;
        int mon = tm_now->tm_mon + 1;
        int day = tm_now->tm_mday;
        int hour = tm_now->tm_hour;
        int min = tm_now->tm_min;
        int sec = tm_now->tm_sec;

        *p_uint8++ = 0x55;
        *p_uint8++ = 0x02;
        *p_uint8++ = 0x54;

        *p_uint8++ = 0x01;
        *p_uint8++ = 0x01;

        output_buffer[3] = 156 >> 16 & 0xff;
        output_buffer[4] = 156 & 0xff;

        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&year, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&mon, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&day, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&hour, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&min, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&sec, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.ax, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.ay, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.az, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.gx, 4);

        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.gy, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.gz, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.gz_max, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.gz_min, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.gz_avg, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.temp, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.inc1_max, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.inc1_min, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.inc1_avg, 4);

        send_data_to_vd_tool(output_buffer, 81);

        p_uint8 = (uint8_t *)output_buffer;
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.inc2_max, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.inc2_min, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.inc2_avg, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.inc6_max, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.inc6_min, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.inc6_avg, 4);

        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.virtual_x_radius_min, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.virtual_x_radius_max, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.virtual_y_radius_min, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.virtual_y_radius_max, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.gz_dps_sdv_max, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.gz_dps_sdv_min, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&vSupply, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.hs, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.flag, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.std_v_norm_g_max, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.std_v_norm_g_min, 4);
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&log.std_v_norm_g_avg, 4);
        *p_uint8++ = 0x54;
        *p_uint8++ = 0x02;
        *p_uint8++ = 0x55;

        send_data_to_vd_tool(output_buffer, 75);
        isSend = 0;
    }

    if (receiveMsg[0] == 'K' && receiveMsg[1] == 'P' && receiveMsg[3] == '=' && receiveMsg[4] != '?')
    {
        uint32_t val0;
        char *p_val;
        val0 = receiveMsg[2] - '0';
        p_val = (char *)&receiveMsg[4];
        float fValue = atof(p_val);
        is25pl032_flash_set_offset(val0, fValue);
        goto ok;
    }
    if (receiveMsg[0] == 'U' && receiveMsg[1] == 'X' && receiveMsg[2] == '=' && receiveMsg[3] != '?')
    {
        memset(temp_data, 0, sizeof(temp_data)); // 使用前清零
        // uint8_t n, m;
        float yRadiusUpper, yRadiusLower;

        char *param_data = (char *)receiveMsg + 3;

        param_data += 2;
        cleanArray(temp_data, sizeof(temp_data));
        param_data = findEndByFlag(param_data, (char *)temp_data, 20, ':');
        yRadiusUpper = atof((const char *)temp_data);
        param_data++;
        cleanArray(temp_data, sizeof(temp_data));
        param_data = findEndByFlag(param_data, (char *)temp_data, 20, ':');
        yRadiusLower = atof((const char *)temp_data);
        param_data++;
        cleanArray(temp_data, sizeof(temp_data));
        findEnd(param_data, (char *)temp_data, 20);

        is25pl032_flash_set_param(yRadiusUpper, yRadiusLower);
        goto ok;
    }
    if ((receiveMsg[0] == 'A' || receiveMsg[0] == 'G') && receiveMsg[2] == 'B' && receiveMsg[3] == '=')
    {

        if (receiveMsg[4] != '?')
        {
            // AXB=xxxxxxxxx
            // AYB=xxxxxxxxx
            // AZB=xxxxxxxxx
            // GXB=xxxxxxxxx
            // GYB=xxxxxxxxx
            // GZB=xxxxxxxxx

            int8_t degree = 0;
            uint8_t bIndex = 0;
            uint8_t k = 6;
            double b[degree + 1];
            // unsigned char bytes[8];
            degree = receiveMsg[4] - '0';
            if (receiveMsg[4] == '-')
            {
                degree = receiveMsg[5] - '0';
                degree = -degree;
                k = 7;
            }
            memset(temp_data, 0, sizeof(temp_data)); // 使用前清零
            uint8_t tempIndex = 0;
            for (; k < RECEIVE_MSG_LEN; k++)
            {

                if (receiveMsg[k] == 0x2C || receiveMsg[k] == 0x00)
                {
                    tempIndex = 0;
                    b[bIndex++] = atof((const char *)temp_data);
                    for (uint8_t i = 0; i < 24; i++)
                    {
                        temp_data[i] = 0x00;
                    }
                    if (receiveMsg[k] == 0x00)
                    {
                        break;
                    }
                    continue;
                }
                temp_data[tempIndex++] = receiveMsg[k];
            }

            if (receiveMsg[0] == 'A')
            {
                if (receiveMsg[1] == 'X')
                {
                    is25pl032_flash_set_degree(0, b, degree);
                }
                if (receiveMsg[1] == 'Y')
                {
                    is25pl032_flash_set_degree(1, b, degree);
                }
                if (receiveMsg[1] == 'Z')
                {
                    is25pl032_flash_set_degree(2, b, degree);
                }
            }

            if (receiveMsg[0] == 'G')
            {
                if (receiveMsg[1] == 'X')
                {
                    is25pl032_flash_set_degree(3, b, degree);
                }
                if (receiveMsg[1] == 'Y')
                {
                    is25pl032_flash_set_degree(4, b, degree);
                }
                if (receiveMsg[1] == 'Z')
                {
                    is25pl032_flash_set_degree(5, b, degree);
                }
            }
            goto ok;
        }

        if (receiveMsg[4] == '?')
        {
            uint8_t s = 0;
            uint8_t flag = 0x03;
            if (receiveMsg[0] == 'A')
            {
                if (receiveMsg[1] == 'X')
                {
                    s = 0;
                    flag = 0x03;
                }
                if (receiveMsg[1] == 'Y')
                {
                    s += 6;
                    flag = 0x04;
                }
                if (receiveMsg[1] == 'Z')
                {
                    s += 12;
                    flag = 0x05;
                }
            }
            if (receiveMsg[0] == 'G')
            {
                if (receiveMsg[1] == 'X')
                {
                    s += 18;
                    flag = 0x06;
                }
                if (receiveMsg[1] == 'Y')
                {
                    s += 24;
                    flag = 0x07;
                }
                if (receiveMsg[1] == 'Z')
                {
                    s += 30;
                    flag = 0x08;
                }
            }
            double degrees[36];
            int8_t degree;
            is25pl032_flash_get_degree(degrees, &degree);

            *p_uint8++ = 0x55;
            *p_uint8++ = flag;
            *p_uint8++ = 0x54;

            *p_uint8++ = 0x01;
            *p_uint8++ = 0x01;

            uint8_t *byteArr = (uint8_t *)&degree;
            *p_uint8++ = *byteArr++;

            for (uint8_t k = 0; k < 6; k++)
            {
                p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&degrees[k + s], 8);
            }

            *p_uint8++ = 0x54;
            *p_uint8++ = flag;
            *p_uint8++ = 0x55;
            sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
            isSend = 0;
        }
    }

    if (receiveMsg[0] == 'U' && receiveMsg[1] == 'P' && receiveMsg[2] == '=' && receiveMsg[3] != '?')
    {
        memset(temp_data, 0, sizeof(temp_data)); // 使用前清零
        uint8_t k = 0;
        for (; k < 15; k++)
        {
            if (receiveMsg[3 + k] == 0x00)
            {
                break;
            }
            temp_data[k] = receiveMsg[3 + k];
        }
        is25pl032_flash_set_proid((char *)temp_data, k);
        goto ok;
    }
    if (receiveMsg[0] == 'A' && receiveMsg[1] == 'T' && receiveMsg[2] == '=')
    {
        if (receiveMsg[3] != '?')
        {
            char temp[8];
            findEnd((char *)receiveMsg + 3, temp, 8);
            set_acc_sensor_type(toInt32(temp, 0, 4));
            goto ok;
        }
        else
        {
            uint8_t acc_type = get_acc_sensor_type();
            uint8_t flag = 0x10;
            *p_uint8++ = 0x55;
            *p_uint8++ = flag; // 区分的是哪个指令
            *p_uint8++ = 0x54;
            *p_uint8++ = 0x01;
            *p_uint8++ = 0x01;
            p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&acc_type, 1);
            *p_uint8++ = 0x54;
            *p_uint8++ = flag;
            *p_uint8++ = 0x55;
            sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
            isSend = 0;
        }
    }
    if (receiveMsg[0] == 'G' && receiveMsg[1] == 'T' && receiveMsg[2] == '=')
    {
        if (receiveMsg[3] != '?')
        {
            char temp[8];
            findEnd((char *)receiveMsg + 3, temp, 8);
            set_gyro_sensor_type(toInt32(temp, 0, 4));
            goto ok;
        }
        else
        {
            uint8_t gyro_type = get_gyro_sensor_type();
            uint8_t flag = 0x11;
            *p_uint8++ = 0x55;
            *p_uint8++ = flag; // 区分的是哪个指令
            *p_uint8++ = 0x54;
            *p_uint8++ = 0x01;
            *p_uint8++ = 0x01;
            p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&gyro_type, 1);
            *p_uint8++ = 0x54;
            *p_uint8++ = flag;
            *p_uint8++ = 0x55;
            sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
            isSend = 0;
        }
    }

    if (receiveMsg[0] == 'T' && receiveMsg[1] == 'L' && receiveMsg[2] == '=')
    {
        if (receiveMsg[3] != '?')
        {
            char temp[8];
            findEnd((char *)receiveMsg + 3, temp, 8);
            set_temp_comp_lower_limit(toFloat(temp, 0, 4));
            goto ok;
        }
        else
        {
            float lower = get_temp_comp_lower_limit();
            uint8_t flag = 0x12;
            *p_uint8++ = 0x55;
            *p_uint8++ = flag; // 区分的是哪个指令
            *p_uint8++ = 0x54;
            *p_uint8++ = 0x01;
            *p_uint8++ = 0x01;
            p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&lower, 4);
            *p_uint8++ = 0x54;
            *p_uint8++ = flag;
            *p_uint8++ = 0x55;
            sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
            isSend = 0;
        }
    }
    if (receiveMsg[0] == 'T' && receiveMsg[1] == 'U' && receiveMsg[2] == '=')
    {
        if (receiveMsg[3] != '?')
        {
            char temp[8];
            findEnd((char *)receiveMsg + 3, temp, 8);
            set_temp_comp_upper_limit(toFloat(temp, 0, 4));
            goto ok;
        }
        else
        {
            float upper = get_temp_comp_upper_limit();
            uint8_t flag = 0x13;
            *p_uint8++ = 0x55;
            *p_uint8++ = flag; // 区分的是哪个指令
            *p_uint8++ = 0x54;
            *p_uint8++ = 0x01;
            *p_uint8++ = 0x01;
            p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&upper, 4);
            *p_uint8++ = 0x54;
            *p_uint8++ = flag;
            *p_uint8++ = 0x55;
            sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
            isSend = 0;
        }
    }
    if (receiveMsg[0] == 'I' && receiveMsg[1] == 'M' && receiveMsg[3] == '=')
    {

        if (receiveMsg[4] != '?')
        {
            // SET IMU=xxxxxxxxx
            uint8_t bIndex = 0;
            uint8_t k = 4;
            double b[9];
            uint8_t temp[24];
            uint8_t tempIndex = 0;
            for (; k < RECEIVE_MSG_LEN; k++)
            {
                if (receiveMsg[k] == 0x2C || receiveMsg[k] == 0x00)
                {
                    tempIndex = 0;
                    b[bIndex++] = atof((const char *)temp);
                    for (uint8_t i = 0; i < 24; i++)
                    {
                        temp[i] = 0x00;
                    }
                    if (receiveMsg[k] == 0x00)
                    {
                        break;
                    }
                    continue;
                }
                temp[tempIndex++] = receiveMsg[k];
            }

            is25pl032_flash_set_imu(b);

            goto ok;
        }

        if (receiveMsg[4] == '?')
        {
            uint8_t flag = 0x0A;
            double b[9];
            is25pl032_flash_get_imu(b);
            *p_uint8++ = 0x55;
            *p_uint8++ = flag;
            *p_uint8++ = 0x54;
            *p_uint8++ = 0x01;
            *p_uint8++ = 0x01;
            for (uint8_t k = 0; k < 9; k++)
            {
                p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&b[k], 8);
            }

            *p_uint8++ = 0x54;
            *p_uint8++ = flag;
            *p_uint8++ = 0x55;
            sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
            isSend = 0;
        }
    }
    if (receiveMsg[0] == 'P' && receiveMsg[1] != 'O' && receiveMsg[2] == '=')
    {
        float b[5];
        if (receiveMsg[3] != '?')
        {
            uint8_t bIndex = 0;
            uint8_t k = 3;
            uint8_t temp[24];
            uint8_t tempIndex = 0;
            for (; k < RECEIVE_MSG_LEN; k++)
            {
                if (receiveMsg[k] == 0x2C || receiveMsg[k] == 0x00)
                {
                    tempIndex = 0;
                    b[bIndex++] = atof((const char *)temp);
                    for (uint8_t i = 0; i < 24; i++)
                    {
                        temp[i] = 0x00;
                    }
                    if (receiveMsg[k] == 0x00)
                    {
                        break;
                    }
                    continue;
                }
                temp[tempIndex++] = receiveMsg[k];
            }
            switch (receiveMsg[1])
            {
            case 'X':
                is25pl032_flash_set_px(b);
                break;
            case 'Y':
                is25pl032_flash_set_py(b);
                break;
            case 'Z':
                is25pl032_flash_set_pz(b);
                break;
            }
            goto ok;
        }
        else
        {
            uint8_t flag = 0x14;
            switch (receiveMsg[1])
            {
            case 'X':
                flag = 0x14;
                is25pl032_flash_get_px(b);
                break;
            case 'Y':
                flag = 0x15;
                is25pl032_flash_get_py(b);
                break;
            case 'Z':
                flag = 0x16;
                is25pl032_flash_get_pz(b);
                break;
            }
            *p_uint8++ = 0x55;
            *p_uint8++ = flag;
            *p_uint8++ = 0x54;
            *p_uint8++ = 0x01;
            *p_uint8++ = 0x01;
            for (uint8_t k = 0; k < 5; k++)
            {
                p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&b[k], 4);
            }
            *p_uint8++ = 0x54;
            *p_uint8++ = flag;
            *p_uint8++ = 0x55;
            sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
            isSend = 0;
        }
    }

    if (receiveMsg[0] == 'G' && receiveMsg[1] == 'V' && receiveMsg[2] == '=' && receiveMsg[3] == '?')
    {
        // 获取电池电压命令GV=?
        float voltage = 0.0f;
        uint8_t flag = 0x0B;
        *p_uint8++ = 0x55;
        *p_uint8++ = flag;
        *p_uint8++ = 0x54;
        *p_uint8++ = 0x01;
        *p_uint8++ = 0x01;
        start_and_get_adc_result();
        voltage = vSupply;
        uint8_t *byteArr = (uint8_t *)&voltage;
        *p_uint8++ = *byteArr++;
        *p_uint8++ = *byteArr++;
        *p_uint8++ = *byteArr++;
        *p_uint8++ = *byteArr++;
        *p_uint8++ = 0x54;
        *p_uint8++ = flag;
        *p_uint8++ = 0x55;
        sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
        isSend = 0;
    }

    if (receiveMsg[0] == 'R' && receiveMsg[1] == 'E' && receiveMsg[2] == '=' && receiveMsg[3] != '?')
    {
        // 设置静态脉冲数据重传次数
        uint32_t val = atoi((const char *)&receiveMsg[3]);
        if (val > 50)
            goto error;
        is25pl032_flash_set_pulse_retry_for_pump_off_data(val);
        isSend = 0;
        goto ok;
    }

    if (receiveMsg[0] == 'R' && receiveMsg[1] == 'E' && receiveMsg[2] == '=' && receiveMsg[3] == '?')
    {
        // 获取静态脉冲数据重传次数
        uint32_t retry_count = 0;
        uint8_t flag = 0x0C;
        *p_uint8++ = 0x55;
        *p_uint8++ = flag;
        *p_uint8++ = 0x54;
        *p_uint8++ = 0x01;
        *p_uint8++ = 0x01;
        retry_count = is25pl032_flash_get_pulse_retry_for_pump_off_data();
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&retry_count, 4);
        *p_uint8++ = 0x54;
        *p_uint8++ = flag;
        *p_uint8++ = 0x55;
        sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
        isSend = 0;
    }

    if (receiveMsg[0] == 'P' && receiveMsg[1] == 'R' && receiveMsg[2] == 'I' && receiveMsg[3] == '=' && receiveMsg[4] != '?')
    {
        // 设置静态脉冲数据重传时间间隔
        uint32_t val = atoi((const char *)&receiveMsg[4]);
        if (val > 3600)
            goto error;
        is25pl032_flash_set_pulse_interval_for_pump_off_data(val);
        isSend = 0;
        goto ok;
    }

    if (receiveMsg[0] == 'P' && receiveMsg[1] == 'R' && receiveMsg[2] == 'I' && receiveMsg[3] == '=' && receiveMsg[4] == '?')
    {
        // 获取静态脉冲数据重传时间间隔
        uint32_t retry_interval = 0;
        uint8_t flag = 0x0D;
        *p_uint8++ = 0x55;
        *p_uint8++ = flag;
        *p_uint8++ = 0x54;
        *p_uint8++ = 0x01;
        *p_uint8++ = 0x01;
        retry_interval = is25pl032_flash_get_pulse_interval_for_pump_off_data();
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&retry_interval, 4);
        *p_uint8++ = 0x54;
        *p_uint8++ = flag;
        *p_uint8++ = 0x55;
        sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
        isSend = 0;
    }

    if (receiveMsg[0] == 'P' && receiveMsg[1] == 'A' && receiveMsg[2] == 'S' && receiveMsg[3] == '=' && receiveMsg[4] != '?')
    {
        // 设置动态脉冲数据周期性上传时间
        uint32_t val = atoi((const char *)&receiveMsg[4]);
        if (val > 36000)
            goto error;
        is25pl032_flash_set_pulse_interval(val);
        isSend = 0;
        goto ok;
    }

    if (receiveMsg[0] == 'P' && receiveMsg[1] == 'A' && receiveMsg[2] == 'S' && receiveMsg[3] == '=' && receiveMsg[4] == '?')
    {
        // 获取动态脉冲数据周期性上传时间
        uint32_t Pulse_auto_send = 0;
        uint8_t flag = 0x0F;
        *p_uint8++ = 0x55;
        *p_uint8++ = flag;
        *p_uint8++ = 0x54;
        *p_uint8++ = 0x01;
        *p_uint8++ = 0x01;
        Pulse_auto_send = is25pl032_flash_get_pulse_interval();
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&Pulse_auto_send, 4);
        *p_uint8++ = 0x54;
        *p_uint8++ = flag;
        *p_uint8++ = 0x55;
        sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
        isSend = 0;
    }

    if (receiveMsg[0] == 'P' && receiveMsg[1] == 'O' && receiveMsg[2] == '=' && receiveMsg[3] != '?')
    {
        // 设置低功耗状态
        uint32_t val = atoi((const char *)&receiveMsg[3]);
        if (val > 1)
            goto error;
        is25pl032_flash_set_idle_hook_enable(val);
        isSend = 0;
        goto ok;
    }

    if (receiveMsg[0] == 'P' && receiveMsg[1] == 'O' && receiveMsg[2] == '=' && receiveMsg[3] == '?')
    {
        // 获取低功耗状态
        uint32_t idle_hook_enable = 0;
        uint8_t flag = 0x19; // 更新命令标识
        *p_uint8++ = 0x55;
        *p_uint8++ = flag;
        *p_uint8++ = 0x54;
        *p_uint8++ = 0x01;
        *p_uint8++ = 0x01;
        idle_hook_enable = is25pl032_flash_get_idle_hook_enable();
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&idle_hook_enable, 4);
        *p_uint8++ = 0x54;
        *p_uint8++ = flag;
        *p_uint8++ = 0x55;
        sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
        isSend = 0;
    }

    // 读取校准数据命令 CHS=?
    if (receiveMsg[0] == 'C' && receiveMsg[1] == 'H' && receiveMsg[2] == 'S' && receiveMsg[3] == '=' && receiveMsg[4] == '?')
    {
        uint8_t flag = 0x1A; // 校准数据读取命令标识
        *p_uint8++ = 0x55;
        *p_uint8++ = flag;
        *p_uint8++ = 0x54;
        *p_uint8++ = 0x01;
        *p_uint8++ = 0x01;

        // 读取校准数据
        float calibration_data = is25pl032_flash_get_calibration_data();
        p_uint8 = toOutPutBuffer(p_uint8, (uint8_t *)&calibration_data, 4);

        *p_uint8++ = 0x54;
        *p_uint8++ = flag;
        *p_uint8++ = 0x55;
        sendLength = (uint32_t)p_uint8 - (uint32_t)output_buffer;
        isSend = 0;
    }

    // 写入校准数据命令 CHS=*
    if (receiveMsg[0] == 'C' && receiveMsg[1] == 'H' && receiveMsg[2] == 'S' && receiveMsg[3] == '=' && receiveMsg[4] != '?')
    {
        char *data = (char *)receiveMsg + 4;
        uint8_t temp[24];
        uint8_t tempIndex = 0;
        uint8_t dataIndex = 0;

        // 解析校准数据
        tempIndex = 0;
        while (data[dataIndex] != '\0')
        {
            temp[tempIndex++] = data[dataIndex++];
        }
        temp[tempIndex] = '\0';

        // 写入校准数据
        float calibration_data = atof((const char *)temp);
        is25pl032_flash_set_calibration_data(calibration_data);
        isSend = 0;
        goto ok;
    }

    // 剪切阀通信测试命令 CT=*
    if (receiveMsg[0] == 'C' && receiveMsg[1] == 'T' && receiveMsg[2] == '=' && receiveMsg[3] != '?')
    {
        char *data = (char *)receiveMsg + 3;
        uint8_t temp[8];
        uint8_t tempIndex = 0;
        uint8_t dataIndex = 0;

        // 解析测试参数
        tempIndex = 0;
        while (data[dataIndex] != '\0' && tempIndex < 7)
        {
            temp[tempIndex++] = data[dataIndex++];
        }
        temp[tempIndex] = '\0';

        // 发送测试脉冲
        uint8_t test_type = atoi((const char *)temp);
        send_cutter_valve_test_pulse(test_type);

        // 直接返回成功响应
        isSend = 0;
        goto ok;
    }

    if (sendLength > 0)
    {
        output_buffer[3] = sendLength >> 16 & 0xff;
        output_buffer[4] = sendLength & 0xff;
        send_data_to_vd_tool(output_buffer, sendLength);
        return;
    }

    if (sendLength == 0)
    {
        isSend = 0;
        return;
    }

ok:
    isSend = 0;
    output_buffer[0] = 0x55;
    output_buffer[1] = 0xaa;
    output_buffer[2] = 0x54;
    send_data_to_vd_tool(output_buffer, 3);
    return;
error:
    isSend = 0;
    output_buffer[0] = 0x55;
    output_buffer[1] = 0xab;
    output_buffer[2] = 0x54;
    send_data_to_vd_tool(output_buffer, 3);
    return;
}

/**
 * @brief  处理UART消息
 * @param  rx_msg: 接收到的消息缓冲区
 * @param  rx_msg_len: 消息长度指针
 * @note   1. 禁用中断
 *         2. 复制消息
 *         3. 转换为大写
 *         4. 处理消息
 */
void handle_uart_msg(uint8_t *rx_msg, uint32_t *rx_msg_len)
{
    static uint8_t msg[128];
    uint32_t msg_len;

    __disable_irq();
    msg_len = *rx_msg_len;
    if (msg_len > sizeof(msg) - 1)
        msg_len = sizeof(msg) - 1;
    memcpy(msg, rx_msg, msg_len);
    *rx_msg_len = 0;
    __enable_irq();

    // Convert to Uppercase
    for (int i = 0; i < msg_len; i++)
    {
        if (msg[i] >= 'a' && msg[i] <= 'z')
            msg[i] -= 32;
    }
    msg[msg_len] = 0;
    uart_proc_received_msg(msg, msg_len);
}
