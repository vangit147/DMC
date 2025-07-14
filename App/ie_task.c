/**
 ******************************************************************************
 * @file    ie_task.c
 * @author  Gordon Li
 * @version V1.2
 * @date    2025-04-05 00:00:07 Saturday
 * @brief   倾角计算任务实现
 *
 * @note
 *          - 实现了基于加速度计和陀螺仪的倾角融合计算
 *          - 包含数据质量评估和虚拟半径计算
 *          - 支持多种倾角计算方法
 *
 * @update_history
 *          - 2025-04-05: 将algorithm_data改为全局变量
 *          - 2025-04-05: 增加dsp/fast_math_functions.h头文件包含
            - 2025-05-23: 修改虚拟半径计算逻辑中的bug，将360°一圈时长计算中漏掉的定时器频率加入
 ******************************************************************************
 */

/* 头文件保护宏 ----------------------------------------------------------------*/
#ifndef IE_TASK_C
#define IE_TASK_C

/* 系统头文件包含 ----------------------------------------------------------------*/
//#include <stdint.h>
//#include <stdbool.h>
//#include <string.h>
//#include "arm_math.h"

/* 项目头文件包含 ----------------------------------------------------------------*/
#include "ie_task.h"

/* 宏定义 ----------------------------------------------------------------*/
/* 基础计算宏 */
#define power2(x) ((x) * (x))             /* 求平方 */
#define MIN(a, b) ((a) < (b) ? (a) : (b)) /* 取最小值 */
#define _180_Div_Pi 57.295779513082320876798154814105f

/* 连续计数阈值 */
#define MAX_CONTINUOUS_BEFORE 120         /* 最大前向连续计数值 */
#define MAX_CONTINUOUS_AFTER 30           /* 最大后向连续计数值 */
#define MIN_CONTINUOUS_BEFORE 50          /* 最小前向连续计数阈值 */
#define MIN_CONTINUOUS_AFTER 15           /* 最小后向连续计数阈值 */

/* 虚拟半径计算相关 */
#define CIRCLE_SIZE 30                    /* 存储用于计算虚拟半径的数组大小 */
#define Gz_COUNT_THRESHOLD 200           /* 计算虚拟半径时，Gz的计数值阈值 */

/* 权重值定义 */
#define WEIGHT_MIN 1   /* 最小权重 */
#define WEIGHT_LOW 8   /* 低权重 */
#define WEIGHT_MED 24  /* 中等权重 */
#define WEIGHT_MAX 254 /* 最大权重 */

/* 时间相关定义 */
#define TIMER_10MS 10   /* 毫秒 */
#define TIMER_20MS 20   /* 毫秒 */
#define TIMER_50MS 50   /* 毫秒 */
#define TIMER_100MS 100 /* 毫秒 */

/* 事件定义 */
#define EVENT_TIMER_10MS (0X1)
#define EVENT_TIMER_20MS (0X1 << 1)
#define EVENT_TIMER_50MS (0X1 << 2)
#define EVENT_TIMER_100MS (0X1 << 3)

/* 算法参数配置 */
#define HISTORY_MAX_LEN 15      /* 二级矩阵的每级长度 */
#define SNAPSHOT_HISTORY_LEN 20 /* 供丢弃的回溯数据的长度 */
#define BEST_RECORD_LEN 51      /* 最佳记录长度 */
#define HIGH_WEIGHT_INC_LEN 180 /* 高权重倾角环形队列长度 */

/* EKF参数定义 */
#define EKF_DT 0.05f        /* EKF采样周期，与TIMER_50MS对应 */
#define EKF_Q_ANGLE 0.0002f /* 过程噪声 */
#define EKF_Q_GYRO 0.001f   /* 陀螺仪过程噪声 */
#define EKF_R_ANGLE 0.08f   /* 测量噪声 */
#define MAX_R_MULTIPLIER 10000.0f /* 最大R_ANGLE倍数 */
#define MAX_Q_MULTIPLIER 2.0f     /* 最大Q倍数 */
#define GZ_STD_THRESHOLD_EKF 3.0f /* 用于EKF的Gz标准差阈值 */

/* 可调配参数 */
#define VARIANCE_SAMPLE_LEN 20         /* 计算标准差的样本长度 */
#define VARIANCE_THRESHOLD_Gz_C2 20.0f /* 标准差阈值 */
#define VARIANCE_THRESHOLD_Ax_y 0.2f   /* ax与ay的标准差阈值之和 */
#define VARIANCE_THRESHOLD_Gz_C1 10.0f /* 入选C1-leavel的转速标准差阈值 */
#define RATATION_THRESHOLD_Gz_C0 36.0f /* 旋转阈值 */

/* 类型定义 ----------------------------------------------------------------*/
/* EKF状态结构体 */
typedef struct {
    float angle;     /* 角度 */
    float bias;      /* 陀螺仪偏差 */
    float P[2][2];   /* 误差协方差矩阵 */
} EKF_State;



/* 全局变量声明 ----------------------------------------------------------------*/
/* 任务句柄 */
TaskHandle_t ie_task_handle;

/* 倾角变量 */
float inc1_roll, inc1_pitch, inc3_roll, inc3_pitch;

/* 传感器数据结构体 */
sensor_data_t sensor_data;

/* 周期性信息结构体 */
interval_info_t interval_info;

/* 倾角高边结构体 */
inclination_hs_t inc_hs_data;

/* 算法信息结构体 */
algorithm_info_t algorithm_data;

/* EKF状态结构体 */
static EKF_State ekf_state;

/* 算法设置默认值 */
algorithm_setting_t algorithm_setting = {
    .log_period_time = 60, /* 日志周期默认60秒 */
    .gx_bias = 0.0f,       /* 陀螺仪零偏默认值 */
    .gy_bias = 0.0f,
    .gz_bias = 0.0f,
    .ax_bias = 0.0f,       /* 加速度计零偏默认值 */
    .ay_bias = 0.0f,
    .az_bias = 0.0f,
    .ms_xx = 1.0f,         /* 加速度计MS矩阵偏差默认值 */
    .ms_xy = 0.0f,
    .ms_xz = 0.0f,
    .ms_yy = 1.0f,
    .ms_yz = 0.0f,
    .ms_zz = 1.0f,
    .acc_x_offset = 0.0f,  /* 加速度计装配误差默认值 */
    .acc_y_offset = 0.0f,
    .acc_z_offset = 0.0f,
    .max_peace_time_threshold = 5, /* 稳定时间阈值默认值 */
    .t_comp_lower_limit = -20.0f,  /* 温度补偿范围初始值 */
    .t_comp_upper_limit = 150.0f,
    .xr_limit = 0.005f,
    .yr_limit = 0.02f
};

/* 函数声明 ----------------------------------------------------------------*/
/* 私有函数声明 */
static int16_t calculate_std_dev_welford(int16_t m, uint16_t n);
static void calculate_weight(void);
static void ekf_filter(float *acc_meas, float *gyro_meas, float *angle_out);
static void calculate_rotation_radius(float ax, float ay, float gz_dps, float gz_rad);
static void qualify_by_variance(void);
static void calibrate(void);
static void ie_task_timer_50ms_cb(TimerHandle_t xTimer);

/* 公共函数声明 */
void getRadiusOffset(float *ry, float *offsetX, float *offsetY, float *rx, float *yrLimit, float *xrLimit);
void compute_ie();
void interval_info_deal(bool period_end);
int32_t get_inc_hs(inclination_hs_t* inc_hs);
void get_interval_info(interval_info_t* info);
void reset_interval_info(void);
void get_sensor_data(sensor_data_t* data);
void ie_task(void *p);

/**
 *******************************************************************************
  * @Description: 20ms定时器回调函数，用于触发ie_task任务
  * @Parameters : xTimer - 定时器句柄
  * @RetValue   : 无
  * @Note       : 通过xTaskGenericNotify向ie_task发送EVENT_TIMER_20MS事件

  * @CreatedBy  : Gordon Li
  * @CreatedDate: 2024.02.06 14:30:00 Tuesday
  *******************************************************************************
  */
/*
static void ie_task_timer_20ms_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(ie_task_handle, EVENT_TIMER_20MS, eSetBits, NULL);
}
*/
/**
  *******************************************************************************
  * @Description: 50ms定时器回调函数，用于触发ie_task任务
  * @Parameters : xTimer - 定时器句柄
  * @RetValue   : 无
  * @Note       : 通过xTaskGenericNotify向ie_task发送EVENT_TIMER_50MS事件

  * @CreatedBy  : Gordon Li
  * @CreatedDate: 2024.02.06 14:30:00 Tuesday
  *******************************************************************************
  */
static void ie_task_timer_50ms_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(ie_task_handle, EVENT_TIMER_50MS, eSetBits, NULL);
}
/**
  *******************************************************************************
  * @Description: 100ms定时器回调函数，用于触发ie_task任务
  * @Parameters : xTimer - 定时器句柄
  * @RetValue   : 无
  * @Note       : 通过xTaskGenericNotify向ie_task发送EVENT_TIMER_100MS事件

  * @CreatedBy  : Gordon Li
  * @CreatedDate: 2024.02.22 22:01:12 Thursday
  *******************************************************************************

static void ie_task_timer_100ms_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(ie_task_handle, EVENT_TIMER_100MS, eSetBits, NULL);
}
 */

/*
float fround(float x, int n)
{
    x = ((float)((int)(((x + (pow(0.1, n) * 0.5)) * pow(10, n))))) / pow(10, n);
    return x;
}
*/

/**
  *******************************************************************************
 * @Description: 根据加速度计和陀螺仪数据的方差计算数据质量等级
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 通过分析加速度计和陀螺仪数据的方差来判断设备运动状态
 *               将数据质量分为C0(静止)、C1(匀速)、C2(变速)三个等级
  * @CreatedBy  : Gordon
 * @CreatedDate: 2024.02.26 12:19:26 Monday
  *******************************************************************************
  */

static void qualify_by_variance(void)
{
    static uint32_t last_timestick; // 上次时间
    static float time_slot; // 两次调用之间的时间

    // 计算每次调用间隔的时间
    time_slot = (xTaskGetTickCount() - last_timestick) / 1000.0f;
    last_timestick = xTaskGetTickCount();

    // 将当前数据存入环形数组
    int current_pos = algorithm_data.tail;
    algorithm_data.b_record[0][current_pos] = sensor_data.ax_cf_g;
    algorithm_data.b_record[1][current_pos] = sensor_data.ay_cf_g;
    algorithm_data.b_record[2][current_pos] = sensor_data.az_cf_g;
    algorithm_data.b_record[3][current_pos] = sensor_data.gz_dps;

    // 计算标准差并存储 - 使用Welford算法
    int16_t target_pos = calculate_std_dev_welford(VARIANCE_SAMPLE_LEN, SNAPSHOT_HISTORY_LEN);

    // 验证target_pos的有效性
    if (target_pos < 0 || target_pos >= BEST_RECORD_LEN)
    {
        // 如果返回的位置无效，使用当前位置
        target_pos = current_pos;
    }

    // 获取标准差值
    float std_dev_ax_ay = algorithm_data.b_record_stdv[0][target_pos];
    float std_dev_gz = algorithm_data.b_record_stdv[1][target_pos];
		
    algorithm_data.gz_dps_sdv = std_dev_gz;
    algorithm_data.std_dev_ax_ay = std_dev_ax_ay;

    // 计算grade
    int current_grade;
    float abs_gz = fabs(sensor_data.gz_dps);

    // 根据标准差和旋转阈值判断数据质量等级
    if (std_dev_gz <= VARIANCE_THRESHOLD_Gz_C1 &&
        std_dev_ax_ay <= VARIANCE_THRESHOLD_Ax_y &&
        abs_gz <= RATATION_THRESHOLD_Gz_C0)
    {
        current_grade = 0; // C0级：静止状态
    }
    else if (std_dev_gz <= VARIANCE_THRESHOLD_Gz_C1 &&
             std_dev_ax_ay <= VARIANCE_THRESHOLD_Ax_y)
    {
        current_grade = 1; // C1级：匀速状态
    }
    else if (std_dev_gz <= VARIANCE_THRESHOLD_Gz_C2)
    {
        current_grade = 2; // C2级：准匀速状态
    }
    else
    {
        current_grade = -1; // 无效数据
    }

    if(!(current_grade==0))//peace_time重开,reset
    {
        algorithm_data.peace_time_break = true;
        algorithm_data.peace_time = 0;
        algorithm_data.peacetime_threshod_write_done=false;
    }
    else
    {
        algorithm_data.peace_time_break = false;
        algorithm_data.peace_time += time_slot;
    }

    // 存储grade
    algorithm_data.b_record_grade[target_pos] = current_grade;
    algorithm_data.current_signal_level = current_grade;

    // 更新环形队列状态
    if (algorithm_data.record_num <= BEST_RECORD_LEN)
    {
        // 队列未满，增加计数
        algorithm_data.record_num++;
        algorithm_data.tail = (algorithm_data.tail + 1) % BEST_RECORD_LEN;
    }
    else
    {
        // 队列已满，先处理head位置的数据
        int removed_idx = algorithm_data.head;
        int removed_grade = (int)algorithm_data.b_record_grade[removed_idx];

        // 处理被移除的数据
        if (removed_grade == 0 || removed_grade == 1)
        {
            // 处理高权重数据
            int pre_pos;
            if (algorithm_data.high_weight_incs_head == -1)
            {
                // 空队列初始化
                algorithm_data.high_weight_incs_head = 0;
                algorithm_data.high_weight_incs_tail = 0;
                pre_pos = -1;
            }
            else
            {
                // 计算前一个位置
                pre_pos = (algorithm_data.high_weight_incs_tail - 1 + HIGH_WEIGHT_INC_LEN) % HIGH_WEIGHT_INC_LEN;

                // 检查队列是否已满
                int next_tail = (algorithm_data.high_weight_incs_tail + 1) % HIGH_WEIGHT_INC_LEN;
                if (next_tail == algorithm_data.high_weight_incs_head)
                {
                    // 队列已满，移动head
                    algorithm_data.high_weight_incs_head =
                        (algorithm_data.high_weight_incs_head + 1) % HIGH_WEIGHT_INC_LEN;
                }
            }

            // 存储数据到high_weight_incs
            algorithm_data.high_weight_incs_data[algorithm_data.high_weight_incs_tail] =
                fabs(atan2(sqrt(power2(sensor_data.ax_cf_g) + power2(sensor_data.ay_cf_g)), sensor_data.az_cf_g) * 180 / PI);

            // 计算加速度误差（使用当前加速度与标准重力加速度的差值）
            float acc_error = sqrtf(power2(algorithm_data.b_record[0][target_pos]) +
                                          power2(algorithm_data.b_record[1][target_pos]) +
                                          power2(algorithm_data.b_record[2][target_pos]));

            // 直接使用acc_error，不再存储到数组中
            algorithm_data.high_weight_incs_grade[algorithm_data.high_weight_incs_tail] = removed_grade;

            // 处理连续性信息
            if (pre_pos == -1 ||
                algorithm_data.high_weight_incs_grade[pre_pos] != removed_grade)
            {
                // 新的连续数据块
                algorithm_data.high_weight_incs_info[0][algorithm_data.high_weight_incs_tail] = 1;
                algorithm_data.high_weight_incs_info[1][algorithm_data.high_weight_incs_tail] = 1;

                if (pre_pos != -1)
                {
                    // 标记前一个数据块结束
                    algorithm_data.high_weight_incs_info[0][pre_pos] =
                        -abs(algorithm_data.high_weight_incs_info[0][pre_pos]);
                }
            }
            else
            {
                // 继续现有数据块
                algorithm_data.high_weight_incs_info[0][algorithm_data.high_weight_incs_tail] =
                    MIN(algorithm_data.high_weight_incs_info[0][pre_pos] + 1, MAX_CONTINUOUS_BEFORE);

                // 更新前面数据的backward_count
                int curr_pos = pre_pos;
                int count = 1;
                while (curr_pos != algorithm_data.high_weight_incs_head &&
                       algorithm_data.high_weight_incs_grade[curr_pos] == removed_grade &&
                       algorithm_data.high_weight_incs_info[0][curr_pos] > 0)
                {
                    algorithm_data.high_weight_incs_info[1][curr_pos] = MIN(count + 1, MAX_CONTINUOUS_AFTER);
                    curr_pos = (curr_pos - 1 + HIGH_WEIGHT_INC_LEN) % HIGH_WEIGHT_INC_LEN;
                    count++;
                }
                algorithm_data.high_weight_incs_info[1][algorithm_data.high_weight_incs_tail] = 1;
            }

            // 计算权重
            calculate_weight();

            // 移动tail指针
            algorithm_data.high_weight_incs_tail =
                (algorithm_data.high_weight_incs_tail + 1) % HIGH_WEIGHT_INC_LEN;
        }

        // 更新b_record的指针
        algorithm_data.head = (algorithm_data.head + 1) % BEST_RECORD_LEN;
        algorithm_data.tail = (algorithm_data.tail + 1) % BEST_RECORD_LEN;
    }
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.11.09 11:59:26 Saturday
  *******************************************************************************

*/

static void calibrate(void)
{

    // 计算离心力补偿后的加速度
    sensor_data.ax_cf_g = sensor_data.ax_g - power2(sensor_data.gz_rad) * algorithm_data.rx;
    sensor_data.ay_cf_g = sensor_data.ay_g - power2(sensor_data.gz_rad) * algorithm_data.ry;
    sensor_data.az_cf_g = sensor_data.az_g;
}

/**
 *******************************************************************************
 * @Description: 计算井斜角(inc)和方位角(az)等姿态参数
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 根据加速度计和陀螺仪数据计算实时井斜角、方位角等姿态参数
 *               并进行数据质量评估和滤波处理
 * @CreatedBy  : Gordon Li
 * @CreatedDate: 2024.04.04 15:45:00 Tuesday
 *******************************************************************************
 */
void compute_ie()
{
    //在这里旋转半径
    calculate_rotation_radius(sensor_data.ax_g, sensor_data.ay_g, sensor_data.gz_dps, sensor_data.gz_rad);

    float inc;
    float ax_g_square = sensor_data.ax_g * sensor_data.ax_g;
    float ay_g_square = sensor_data.ay_g * sensor_data.ay_g;
    float az_g_square = sensor_data.az_g * sensor_data.az_g;
    
    // 计算inc1- 加速度计实时井斜
    inc = atan2(sqrt(ax_g_square + ay_g_square), sensor_data.az_g) * _180_Div_Pi;
    inc = inc < 0 ? -inc : inc;
    inc_hs_data.inc1_prefilter = inc;
		algorithm_data.inc1=inc;
    
    // 计算roll和pitch角度
    // Roll (绕X轴旋转) - 使用原始加速度值
    inc1_roll = atan2(sensor_data.ay_g, 
                      sqrt(ax_g_square + 
                           az_g_square)) * _180_Div_Pi;
    
    // Pitch (绕Y轴旋转) - 使用原始加速度值
    inc1_pitch = atan2(-sensor_data.ax_g,
                       sqrt(ay_g_square + 
                            az_g_square)) * _180_Div_Pi;
    
    // 更新到inc_hs_data中
    inc_hs_data.inc1_roll = inc1_roll;
    inc_hs_data.inc1_pitch = inc1_pitch;

    // 计算Inc2- 加速度经过陀螺仪的数据离心力补偿后的实时井斜
    float acc_meas[3] = {sensor_data.ax_cf_g, sensor_data.ay_cf_g, sensor_data.az_cf_g};
    float gyro_meas[3] = {sensor_data.gx_rad, sensor_data.gy_rad, sensor_data.gz_rad};
    float angle_out;

    ekf_filter(acc_meas, gyro_meas, &angle_out);
    inc = fabsf(angle_out); // 取绝对值
    algorithm_data.inc2 = inc;
	inc_hs_data.inc2_prefilter = inc;

    // 计算inc3 -- 使用权重计算最优倾角
    if (algorithm_data.high_weight_incs_head != -1)
    {
        // 计算high_weight_incs中的有效数据数量
        int16_t valid_count = (algorithm_data.high_weight_incs_tail - algorithm_data.high_weight_incs_head + HIGH_WEIGHT_INC_LEN) % HIGH_WEIGHT_INC_LEN;

        if (valid_count > 0)
        {
            // 计算加权平均倾角
            float total_weight = 0.0f;
            float weighted_sum = 0.0f;

            // 遍历整个high_weight_incs数组计算加权和
            int16_t curr_pos = algorithm_data.high_weight_incs_head;
            do
            {
                uint8_t weight = algorithm_data.high_weight[curr_pos];
                float inc = algorithm_data.high_weight_incs_data[curr_pos];

                weighted_sum += weight * inc;
                total_weight += weight;

                curr_pos = (curr_pos + 1) % HIGH_WEIGHT_INC_LEN;

            } while (curr_pos != algorithm_data.high_weight_incs_tail);

            // 计算加权平均值
            if (total_weight > 0.0f)
            {
                algorithm_data.inc3 = weighted_sum / total_weight;
                algorithm_data.good_inc = algorithm_data.inc3;
                inc_hs_data.inc3_prefilter = algorithm_data.inc3;
                
                // 计算 inc3 的 roll 和 pitch 角度
                // 使用与 inc1 相同的计算方法，但使用补偿后的加速度值
                inc_hs_data.inc3_roll = atan2(sensor_data.ay_cf_g, 
                                             sqrt(sensor_data.ax_cf_g * sensor_data.ax_cf_g + 
                                                  sensor_data.az_cf_g * sensor_data.az_cf_g)) * 180.0f / PI;
                
                inc_hs_data.inc3_pitch = atan2(-sensor_data.ax_cf_g,
                                              sqrt(sensor_data.ay_cf_g * sensor_data.ay_cf_g + 
                                                   sensor_data.az_cf_g * sensor_data.az_cf_g)) * 180.0f / PI;
    }
    else
    {
                algorithm_data.good_inc = algorithm_data.inc2; // 如果没有有效权重，使用inc2作用good_inc
                // 当没有有效权重时，inc3_roll 和 inc3_pitch 保持原值
            }
        }
    }
    else
    {
        // 当high_weight_incs为空时，使用当前传感器数据
        algorithm_data.good_inc = algorithm_data.inc2; // 使用EKF计算的倾角

			//inc_hs_data.inc3 此时不再更新
    }
    
    
    inc_hs_data.good_inc_prefilter = algorithm_data.good_inc;

    // 计算高边
    double local_hs;
    float x = (is25pl032_flash_get_hs_direction() == 1) ? sensor_data.ax_g : -sensor_data.ax_g;
    float y = sensor_data.ay_g;

    // 使用atan2直接计算角度，返回值范围(-π, π]
    local_hs = atan2(y, x);

    // 转换为角度
    local_hs *= 180.0f / PI;

    // 转换为[0, 360)范围
    if (local_hs < 0)
    {
        local_hs += 360.0;
    }

    inc_hs_data.hs = local_hs;
//    excel中计算高边的公式：
//    =IF(ATAN2(-X轴数据,Y轴数据)*180/PI()<0,360+ATAN2(-X轴数据,Y轴数据)*180/PI(),ATAN2(-X轴数据,Y轴数据)*180/PI())
    
    // 中位数滤波处理
    // 将当前值存入缓冲区
    inc_hs_data.inc1_buffer[inc_hs_data.buffer_index] = inc_hs_data.inc1_prefilter;
    inc_hs_data.inc2_buffer[inc_hs_data.buffer_index] = inc_hs_data.inc2_prefilter;
    inc_hs_data.inc3_buffer[inc_hs_data.buffer_index] = inc_hs_data.inc3_prefilter;
    inc_hs_data.good_inc_buffer[inc_hs_data.buffer_index] = inc_hs_data.good_inc_prefilter;
    
    // 更新缓冲区索引
    inc_hs_data.buffer_index = (inc_hs_data.buffer_index + 1) % 10;
    
    // 如果缓冲区已填满，进行中位数滤波
    if (inc_hs_data.buffer_index == 0)
    {
        // 创建临时数组用于排序
        float temp_inc1[10], temp_inc2[10], temp_inc3[10], temp_good_inc[10];
        
        // 复制数据到临时数组
        for (int i = 0; i < 10; i++)
        {
            temp_inc1[i] = inc_hs_data.inc1_buffer[i];
            temp_inc2[i] = inc_hs_data.inc2_buffer[i];
            temp_inc3[i] = inc_hs_data.inc3_buffer[i];
            temp_good_inc[i] = inc_hs_data.good_inc_buffer[i];
        }
        
        // 对临时数组进行排序
        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 9 - i; j++)
            {
                if (temp_inc1[j] > temp_inc1[j + 1])
                {
                    float temp = temp_inc1[j];
                    temp_inc1[j] = temp_inc1[j + 1];
                    temp_inc1[j + 1] = temp;
                }
                
                if (temp_inc2[j] > temp_inc2[j + 1])
                {
                    float temp = temp_inc2[j];
                    temp_inc2[j] = temp_inc2[j + 1];
                    temp_inc2[j + 1] = temp;
                }
                
                if (temp_inc3[j] > temp_inc3[j + 1])
                {
                    float temp = temp_inc3[j];
                    temp_inc3[j] = temp_inc3[j + 1];
                    temp_inc3[j + 1] = temp;
                }
                
                if (temp_good_inc[j] > temp_good_inc[j + 1])
                {
                    float temp = temp_good_inc[j];
                    temp_good_inc[j] = temp_good_inc[j + 1];
                    temp_good_inc[j + 1] = temp;
                }
            }
        }
        
        // 取中位数（第5个元素，索引为4）
        inc_hs_data.inc1 = (temp_inc1[2]+temp_inc1[3]+temp_inc1[4])/3.0f;
        inc_hs_data.inc2 = (temp_inc2[2]+temp_inc2[3]+temp_inc2[4])/3.0f;
        inc_hs_data.inc3 = (temp_inc3[2]+temp_inc3[3]+temp_inc3[4])/3.0f;
        inc_hs_data.good_inc = (temp_good_inc[2]+temp_good_inc[3]+temp_good_inc[4])/3.0f;
    }
}


/**
 * @brief 处理interval_info数据
 * @param period_end 是否是周期结束
 */
void interval_info_deal(bool period_end)
{
    static uint32_t period_timer = 0;  // 静态变量用于计时
    
    // 如果达到日志周期时间或手动触发重置，则执行重置
    if (period_end || period_timer >= algorithm_setting.log_period_time*50)
    {
        reset_interval_info();
        period_timer = 0;  // 重置计时器
    }
    else
    {
        period_timer++;  // 增加计时器
    }

    // 更新interval_info中的统计数据
    // 更新虚拟半径的最大最小值
    if (fabs(algorithm_data.rx) > fabs(interval_info.radius_x_max))
        interval_info.radius_x_max = algorithm_data.rx;
    if (fabs(algorithm_data.rx) < fabs(interval_info.radius_x_min))
        interval_info.radius_x_min = algorithm_data.rx;

    if (fabs(algorithm_data.ry) > fabs(interval_info.radius_y_max))
        interval_info.radius_y_max = algorithm_data.ry;
    if (fabs(algorithm_data.ry) < fabs(interval_info.radius_y_min))
        interval_info.radius_y_min = algorithm_data.ry;

    // 更新标准差的最大值
    if (algorithm_data.std_dev_ax_ay > interval_info.std_dev_ax_ay_max)
        interval_info.std_dev_ax_ay_max = algorithm_data.std_dev_ax_ay;

    if (algorithm_data.gz_dps_sdv > interval_info.sdv_gyro_z_max)
        interval_info.sdv_gyro_z_max = algorithm_data.gz_dps_sdv;
    if (algorithm_data.gz_dps_sdv < interval_info.sdv_gyro_z_min)
        interval_info.sdv_gyro_z_min = algorithm_data.gz_dps_sdv;

    // 更新稳定时间的最大值
    if (algorithm_data.peace_time > interval_info.peace_time_max)
        interval_info.peace_time_max = algorithm_data.peace_time;
    
    // 注意：peace_time_count的增加并不在qualify_by_variance函数中处理
    // 当peace_time_break为true且peace_time达到阈值时，会增加peace_time_count
    if(algorithm_data.peace_time>=algorithm_setting.max_peace_time_threshold&&(!algorithm_data.peacetime_threshod_write_done))
    {
      interval_info.peace_time_count++;
			algorithm_data.peacetime_threshod_write_done=true;
    }

    // 更新倾角的最大最小值和均值
    if (inc_hs_data.inc1 > interval_info.inc1_max)
        interval_info.inc1_max = inc_hs_data.inc1;
    if (inc_hs_data.inc1 < interval_info.inc1_min)
        interval_info.inc1_min = inc_hs_data.inc1;
    interval_info.inc1_avg = (interval_info.inc1_avg * interval_info.acc_count + inc_hs_data.inc1) / (interval_info.acc_count + 1);

    if (inc_hs_data.inc2 > interval_info.inc2_max)
        interval_info.inc2_max = inc_hs_data.inc2;
    if (inc_hs_data.inc2 < interval_info.inc2_min)
        interval_info.inc2_min = inc_hs_data.inc2;
    interval_info.inc2_avg = (interval_info.inc2_avg * interval_info.acc_count + inc_hs_data.inc2) / (interval_info.acc_count + 1);

    if (inc_hs_data.inc3 > interval_info.inc3_max)
        interval_info.inc3_max = inc_hs_data.inc3;
    if (inc_hs_data.inc3 < interval_info.inc3_min)
        interval_info.inc3_min = inc_hs_data.inc3;
    interval_info.inc3_avg = (interval_info.inc3_avg * interval_info.acc_count + inc_hs_data.inc3) / (interval_info.acc_count + 1);

    // 更新good_inc统计信息
    if (inc_hs_data.good_inc > interval_info.good_inc_max)
        interval_info.good_inc_max = inc_hs_data.good_inc;
    if (inc_hs_data.good_inc < interval_info.good_inc_min)
        interval_info.good_inc_min = inc_hs_data.good_inc;
    interval_info.good_inc_avg = (interval_info.good_inc_avg * interval_info.acc_count + inc_hs_data.good_inc) / (interval_info.acc_count + 1);

    // 更新陀螺仪z轴的最大最小值和均值
    if (sensor_data.gz_dps > interval_info.gyro_z_dps_max)
        interval_info.gyro_z_dps_max = sensor_data.gz_dps;
    if (sensor_data.gz_dps < interval_info.gyro_z_dps_min)
        interval_info.gyro_z_dps_min = sensor_data.gz_dps;
    interval_info.gyro_z_dps_avg = (interval_info.gyro_z_dps_avg * interval_info.acc_count + sensor_data.gz_dps) / (interval_info.acc_count + 1);

    // 更新数据质量等级统计
    if (algorithm_data.current_signal_level == 0 && interval_info.c0_num_count < UINT16_MAX)
        interval_info.c0_num_count++;
    else if (algorithm_data.current_signal_level == 1 && interval_info.c1_num_count < UINT16_MAX)
        interval_info.c1_num_count++;
    else if (algorithm_data.current_signal_level == 2 && interval_info.c2_num_count < UINT16_MAX)
        interval_info.c2_num_count++;

    // 增加计数，并防止溢出
    if (interval_info.acc_count < UINT32_MAX) {
        interval_info.acc_count++;
    }
}

/**
 * @brief  使用Welford在线算法计算环形数组中指定位置前后数据的标准差
 * @param  m   - 需要计算的前m个数据
 * @param  n    - 需要计算的后n个数据
 * @return int16_t    - 返回target_pos
 * @note   使用Welford在线算法，只需一次遍历即可同时计算均值和方差，提高计算效率
 */
static int16_t calculate_std_dev_welford(int16_t m, uint16_t n)
{
    float mean_ax = 0.0f, mean_ay = 0.0f, mean_gz = 0.0f;
    float M2_ax = 0.0f, M2_ay = 0.0f, M2_gz = 0.0f;
    int16_t tail = algorithm_data.tail;
    uint16_t total_samples = m + n;
    int16_t target_pos = 0;

    if (algorithm_data.record_num <= 1)
    {
        algorithm_data.b_record_stdv[0][tail] = 0;
        algorithm_data.b_record_stdv[1][tail] = 0;
        return tail;
    }

    // 确保采样数量不超过数组长度
    if (total_samples > algorithm_data.record_num)
    {
        total_samples = algorithm_data.record_num;
        target_pos = tail;
    }
    else
    {
        target_pos = (tail - n + BEST_RECORD_LEN) % BEST_RECORD_LEN;
    }

    uint32_t start_idx = (tail - total_samples + BEST_RECORD_LEN) % BEST_RECORD_LEN;

    // 使用Welford在线算法，单次遍历计算均值和方差
    for (uint32_t i = 0; i < total_samples; i++)
    {
        uint32_t idx = (start_idx + i) % BEST_RECORD_LEN;
        float x_ax = algorithm_data.b_record[0][idx];
        float x_ay = algorithm_data.b_record[1][idx];
        float x_gz = algorithm_data.b_record[3][idx];
        
        // 更新均值
        float delta_ax = x_ax - mean_ax;
        float delta_ay = x_ay - mean_ay;
        float delta_gz = x_gz - mean_gz;
        
        mean_ax += delta_ax / (i + 1);
        mean_ay += delta_ay / (i + 1);
        mean_gz += delta_gz / (i + 1);
        
        // 更新方差
        M2_ax += delta_ax * (x_ax - mean_ax);
        M2_ay += delta_ay * (x_ay - mean_ay);
        M2_gz += delta_gz * (x_gz - mean_gz);
    }
    
    // 计算最终方差
    float variance_ax = M2_ax / (total_samples - 1);
    float variance_ay = M2_ay / (total_samples - 1);
    float variance_gz = M2_gz / (total_samples - 1);
    
    // 计算标准差
    float std_dev_ax_ay = sqrtf(variance_ax + variance_ay);
    float std_dev_gz = sqrtf(variance_gz);

    // 存储标准差结果
    algorithm_data.b_record_stdv[0][target_pos] = std_dev_ax_ay;
    algorithm_data.b_record_stdv[1][target_pos] = std_dev_gz;

    return target_pos;
}

/**
 * @brief  计算数据权重并写入到指定位置
 * @return None
 */
static void calculate_weight(void)
{
    // 检查数组是否为空或无效
    if (algorithm_data.high_weight_incs_head < 0 ||
        algorithm_data.high_weight_incs_tail < 0 ||
        algorithm_data.high_weight_incs_head >= HIGH_WEIGHT_INC_LEN ||
        algorithm_data.high_weight_incs_tail >= HIGH_WEIGHT_INC_LEN ||
        algorithm_data.high_weight_incs_head == algorithm_data.high_weight_incs_tail)
    {
        return;
    }

    uint8_t weight = 0;
    int curr_pos = algorithm_data.high_weight_incs_head;

    do
    {
        if (curr_pos < 0 || curr_pos >= HIGH_WEIGHT_INC_LEN)
        {
            break;
        }

        // 获取forward和backward计数
        int forward_count = abs(algorithm_data.high_weight_incs_info[0][curr_pos]);
        int backward_count = algorithm_data.high_weight_incs_info[1][curr_pos];

        // 检查计数值的有效性
        if (forward_count < 0 || backward_count < 0)
        {
            weight = 0;
        }
        else
        {
            // 根据前向和后向计数分别判断权重
            if (forward_count >= MAX_CONTINUOUS_BEFORE &&
                backward_count >= MAX_CONTINUOUS_AFTER)
            {
                // 前后向都达到最大连续计数要求
                weight = WEIGHT_MAX;
            }
            else if ((forward_count >= MAX_CONTINUOUS_BEFORE &&
                      backward_count >= MIN_CONTINUOUS_AFTER) ||
                     (forward_count >= MIN_CONTINUOUS_BEFORE &&
                      backward_count >= MAX_CONTINUOUS_AFTER))
            {
                // 一个方向达到最大连续计数，另一个方向达到最小连续计数
                weight = WEIGHT_MED;
            }
            else if (forward_count >= MIN_CONTINUOUS_BEFORE &&
                     backward_count >= MIN_CONTINUOUS_AFTER)
            {
                // 两个方向都达到最小连续计数
                weight = WEIGHT_LOW;
            }
            else
            {
                weight = WEIGHT_MIN;
            }
        }

        algorithm_data.high_weight[curr_pos] = weight;
        curr_pos = (curr_pos + 1) % HIGH_WEIGHT_INC_LEN;
    } while (curr_pos != algorithm_data.high_weight_incs_tail);
}

/**
 * @brief  扩展卡尔曼滤波器实现
 * @param  acc_meas - 加速度计测量值数组 [ax, ay, az]
 * @param  gyro_meas - 陀螺仪测量值数组 [gx, gy, gz]
 * @param  angle_out - 输出角度指针
 * @return None
 * @note   使用EKF融合加速度计和陀螺仪数据计算倾角，并增加中位数均值滤波
 */
static void ekf_filter(float *acc_meas, float *gyro_meas, float *angle_out)
{
    // 获取当前Gz的标准差
    float gz_std_dev = algorithm_data.gz_dps_sdv;
    
    // 设置阈值和调整系数

    // 计算调整系数
    float r_multiplier = 1.0f;
    float q_multiplier = 1.0f;
    
    if (gz_std_dev > GZ_STD_THRESHOLD_EKF) {
        // 线性增加倍数
        float normalized_std = (gz_std_dev - GZ_STD_THRESHOLD_EKF) / GZ_STD_THRESHOLD_EKF;
        r_multiplier = 1.0f + normalized_std * (MAX_R_MULTIPLIER - 1.0f);
        q_multiplier = 1.0f + normalized_std * (MAX_Q_MULTIPLIER - 1.0f);
        
        // 限制最大值
        if (r_multiplier > MAX_R_MULTIPLIER) r_multiplier = MAX_R_MULTIPLIER;
        if (q_multiplier > MAX_Q_MULTIPLIER) q_multiplier = MAX_Q_MULTIPLIER;
    }
    
    // 动态调整参数
    float dynamic_R_ANGLE = EKF_R_ANGLE * r_multiplier;
    float dynamic_Q_ANGLE = EKF_Q_ANGLE * q_multiplier;
    float dynamic_Q_GYRO = EKF_Q_GYRO * q_multiplier;
    
    // 1. 从加速度计数据计算倾角
    float acc_angle = atan2f(sqrtf(acc_meas[0] * acc_meas[0] + acc_meas[1] * acc_meas[1]), acc_meas[2]);
    acc_angle = acc_angle * 180.0f / PI; // 转换为角度

    // 2. 预测步骤
    // 2.1 状态预测
    float gyro_rate = sqrtf(gyro_meas[0] * gyro_meas[0] + gyro_meas[1] * gyro_meas[1]); // 合成角速度
    float angle_pred = ekf_state.angle + (gyro_rate - ekf_state.bias) * EKF_DT;
    float bias_pred = ekf_state.bias; // 假设偏差保持不变

    // 2.2 误差协方差预测
    float F[2][2] = {{1.0f, -EKF_DT}, {0.0f, 1.0f}};           // 状态转移矩阵
    float Q[2][2] = {{dynamic_Q_ANGLE, 0.0f}, {0.0f, dynamic_Q_GYRO}}; // 使用动态调整的过程噪声协方差

    // P = F*P*F' + Q
    float P_temp[2][2];
    P_temp[0][0] = ekf_state.P[0][0] + (-EKF_DT * ekf_state.P[1][0]) + dynamic_Q_ANGLE;
    P_temp[0][1] = ekf_state.P[0][1] + (-EKF_DT * ekf_state.P[1][1]);
    P_temp[1][0] = ekf_state.P[1][0] + (-EKF_DT * ekf_state.P[1][0]);
    P_temp[1][1] = ekf_state.P[1][1] + dynamic_Q_GYRO;

    // 3. 更新步骤
    // 3.1 计算卡尔曼增益
    float H[2] = {1.0f, 0.0f};            // 观测矩阵
    float S = P_temp[0][0] + dynamic_R_ANGLE; // 使用动态调整的测量噪声协方差
    float K[2];                           // 卡尔曼增益
    K[0] = P_temp[0][0] / S;
    K[1] = P_temp[1][0] / S;

    // 3.2 更新状态估计
    float innovation = acc_angle - angle_pred; // 测量残差
    ekf_state.angle = angle_pred + K[0] * innovation;
    ekf_state.bias = bias_pred + K[1] * innovation;

    // 3.3 更新误差协方差
    ekf_state.P[0][0] = P_temp[0][0] * (1.0f - K[0]);
    ekf_state.P[0][1] = P_temp[0][1] * (1.0f - K[0]);
    ekf_state.P[1][0] = P_temp[1][0] - K[1] * P_temp[0][0];
    ekf_state.P[1][1] = P_temp[1][1] - K[1] * P_temp[0][1];

    // 4. 输出结果
    *angle_out = ekf_state.angle;
}

/**
 *******************************************************************************
 * @Description: 获取倾角、高边和姿态角数据
 * @Parameters : inc_hs - 倾角高边结构体指针
 * @RetValue   : 0 - 成功
 * @Note       : 从inc_hs_data结构体中获取倾角、高边和姿态角数据
 * @CreatedBy  : Gordon Li
 * @CreatedDate: 2024.05.01
 *******************************************************************************
 */
int32_t get_inc_hs(inclination_hs_t* inc_hs)
{
    if (inc_hs == NULL)
        return -1;

    // 复制inc_hs_data中的所有数据到inc_hs
    inc_hs->inc1 = inc_hs_data.inc1;
    inc_hs->inc2 = inc_hs_data.inc2;
    inc_hs->inc3 = inc_hs_data.inc3;
    inc_hs->hs = inc_hs_data.hs;
    inc_hs->inc1_roll = inc_hs_data.inc1_roll;
    inc_hs->inc1_pitch = inc_hs_data.inc1_pitch;
    inc_hs->inc3_roll = inc_hs_data.inc3_roll;
    inc_hs->inc3_pitch = inc_hs_data.inc3_pitch;
    inc_hs->good_inc = inc_hs_data.good_inc;

    return 0;
}

/**
 * @brief 从interval_info中获取日志信息
 * @param log 日志结构体指针
 */
void get_interval_info(interval_info_t* info)
{
    if (info == NULL)
        return;

    // 复制interval_info中的数据到info
    info->acc_count = interval_info.acc_count;
    
    // 陀螺仪数据统计
    info->gyro_z_dps_max = interval_info.gyro_z_dps_max;
    info->gyro_z_dps_min = interval_info.gyro_z_dps_min;
    info->gyro_z_dps_avg = interval_info.gyro_z_dps_avg;
    
    // 倾角数据统计
    info->inc1_max = interval_info.inc1_max;
    info->inc1_min = interval_info.inc1_min;
    info->inc1_avg = interval_info.inc1_avg;
    
    info->inc2_max = interval_info.inc2_max;
    info->inc2_min = interval_info.inc2_min;
    info->inc2_avg = interval_info.inc2_avg;
    
    info->inc3_max = interval_info.inc3_max;
    info->inc3_min = interval_info.inc3_min;
    info->inc3_avg = interval_info.inc3_avg;
    
    info->good_inc_max = interval_info.good_inc_max;
    info->good_inc_min = interval_info.good_inc_min;
    info->good_inc_avg = interval_info.good_inc_avg;
    
    // 虚拟半径统计
    info->radius_x_max = interval_info.radius_x_max;
    info->radius_x_min = interval_info.radius_x_min;
    info->radius_y_max = interval_info.radius_y_max;
    info->radius_y_min = interval_info.radius_y_min;
    
    // 加速度标准差统计
    info->std_dev_ax_ay_max = interval_info.std_dev_ax_ay_max;
    
    // 陀螺仪标准差统计
    info->sdv_gyro_z_max = interval_info.sdv_gyro_z_max;
    info->sdv_gyro_z_min = interval_info.sdv_gyro_z_min;
    
    // 稳定时间统计
    info->peace_time_max = interval_info.peace_time_max;
    info->peace_time_count = interval_info.peace_time_count;
    
    // 数据质量等级统计
    info->c0_num_count = interval_info.c0_num_count;
    info->c1_num_count = interval_info.c1_num_count;
    info->c2_num_count = interval_info.c2_num_count;
}

/**
 * @brief 重置interval_info中的所有数据
 */
void reset_interval_info(void)
{
    // 重置累加次数
    interval_info.acc_count = 0;

    // 重置陀螺仪数据
    interval_info.gyro_z_dps_max = 0;
    interval_info.gyro_z_dps_min = 99999;
    interval_info.gyro_z_dps_avg = 0;

    // 重置倾角数据
    interval_info.inc1_max = 0;
    interval_info.inc1_min = 999;
    interval_info.inc1_avg = 0;
    
    interval_info.inc2_max = 0;
    interval_info.inc2_min = 999;
    interval_info.inc2_avg = 0;
    
    interval_info.inc3_max = 0;
    interval_info.inc3_min = 999;
    interval_info.inc3_avg = 0;
    
    // 重置good_inc统计信息
    interval_info.good_inc_max = 0;
    interval_info.good_inc_min = 999;
    interval_info.good_inc_avg = 0;

    // 重置虚拟半径数据
    interval_info.radius_x_max = 0;
    interval_info.radius_x_min = 999;
    interval_info.radius_y_max = 0;
    interval_info.radius_y_min = 999;

    // 重置标准差数据
    interval_info.std_dev_ax_ay_max = 0;
    interval_info.sdv_gyro_z_max = 0;
    interval_info.sdv_gyro_z_min = 999;

    // 重置稳定时间数据
    interval_info.peace_time_max = 0;
    interval_info.peace_time_count = 0;
    algorithm_data.peacetime_threshod_write_done=false;
    algorithm_data.peace_time_break = false;
    algorithm_data.peace_time = 0;

    // 重置数据质量等级统计
    interval_info.c0_num_count = 0;
    interval_info.c1_num_count = 0;
    interval_info.c2_num_count = 0;
}

/**
  *******************************************************************************
  * @Description: 获取传感器数据
  * @Parameters : data - 用于存储传感器数据的结构体指针
  * @RetValue   : 无
  * @Note       : 将sensor_data中的数据复制到传入的结构体中
  *******************************************************************************
  */
void get_sensor_data(sensor_data_t* data)
{
    if (data == NULL)
        return;
    
    // 复制sensor_data中的数据到data
    data->ax_g = sensor_data.ax_g;
    data->ay_g = sensor_data.ay_g;
    data->az_g = sensor_data.az_g;
    data->ax_cf_g = sensor_data.ax_cf_g;
    data->ay_cf_g = sensor_data.ay_cf_g;
    data->az_cf_g = sensor_data.az_cf_g;
    data->gx_rad = sensor_data.gx_rad;
    data->gy_rad = sensor_data.gy_rad;
    data->gz_rad = sensor_data.gz_rad;
    data->gx_dps = sensor_data.gx_dps;
    data->gy_dps = sensor_data.gy_dps;
    data->gz_dps = sensor_data.gz_dps;
    data->t_C = sensor_data.t_C;
}

/**
  *******************************************************************************
 * @brief 计算并更新虚拟旋转半径rx、ry（用于离心力补偿和信号质量分析）
 *
 * 本函数用于在每个采样周期内，累计加速度计x/y轴和陀螺仪z轴的相关数据，自动检测是否完成一圈旋转，
 * 并在每圈结束时通过滑动窗口法计算虚拟半径。虚拟半径rx、ry用于后续的离心力补偿和信号质量判据。
 *
 * 主要流程：
 * 1. 累加本圈内的ax、ay、gz_dps、gz_rad^2等数据。
 * 2. 判断是否达到一圈（gz_dps累计绝对值达到360°且采样点数不超过阈值），
 *    若未达到一圈但采样点数超限，则丢弃本圈数据并重置。
 * 3. 每完成一圈，将本圈数据存入环形缓冲区，并更新累计和（满窗口时先减去最老一圈的数据）。
 * 4. 用窗口内所有圈的累计和计算rx、ry（分别为x、y方向的虚拟半径），并根据配置对其进行限幅。
 * 5. 更新全局算法状态中的rx、ry，供后续离心力补偿等算法使用。
 *
 * @param ax     当前采样周期的x轴加速度（单位：g）
 * @param ay     当前采样周期的y轴加速度（单位：g）
 * @param gz_dps 当前采样周期的z轴角速度（单位：度/秒）
 * @param gz_rad 当前采样周期的z轴角速度（单位：弧度/秒）
 *
 * @note
 * - 本函数应在每次采样周期调用，通常由倾角融合主任务（如ie_task）驱动。
 * - 虚拟半径的窗口长度、采样阈值等参数可根据实际需求调整。
 * - 计算得到的rx、ry会被用于离心力补偿（如ax_cf_g、ay_cf_g的计算）。
 *******************************************************************************
 */
void calculate_rotation_radius(float ax, float ay, float gz_dps, float gz_rad)
{
    // 累加当前圈的ax和ay
    algorithm_data.ax_sum_in_circle += ax;
    algorithm_data.ay_sum_in_circle += ay;
    
    // 累加gz的弧度平方和
    algorithm_data.gz_r_squared_sum_in_circle += gz_rad * gz_rad;
    
    // 累加gz用于判断是否转完一圈
    algorithm_data.gz_dps_sum_in_circle += gz_dps;
    algorithm_data.gz_count++;
    
    // 判断是否达到最大计数次数:50ms定时器运行的话，需要小于等于200次;判断是否转完一圈（通过gz的累加值判断）
		// 当定时器按照50ms时，需要0.05f，当采样频率变化时，需要相应调整 2025-05-23
    if (algorithm_data.gz_count >= Gz_COUNT_THRESHOLD && fabs(algorithm_data.gz_dps_sum_in_circle)*0.05f < 360.0f)
    {
        // 如果总里程未达到360度，重置所有累加变量
        algorithm_data.ax_sum_in_circle = 0.0f;
        algorithm_data.ay_sum_in_circle = 0.0f;
        algorithm_data.gz_r_squared_sum_in_circle = 0.0f;
        algorithm_data.gz_dps_sum_in_circle = 0.0f;
        algorithm_data.gz_count = 0.0f;
    }
    else if (fabs(algorithm_data.gz_dps_sum_in_circle)*0.05f >= 360.0f && algorithm_data.gz_count <= Gz_COUNT_THRESHOLD)
    {
        // 完成一圈，更新滑动窗口队列
        
        if ( algorithm_data.circle_full  )// 如果已经填满20圈，先要减去最古老的数值
        {   
            algorithm_data.total_ax_sum_circle -= algorithm_data.ax_sum_circle[algorithm_data.circles_index];
            algorithm_data.total_ay_sum_circle -= algorithm_data.ay_sum_circle[algorithm_data.circles_index];
            algorithm_data.total_gz_r_sq_sum_circle -= algorithm_data.gz_r_sq_sum_circle[algorithm_data.circles_index];
        }

        // 2. 将当前圈的数据存入队列
        algorithm_data.ax_sum_circle[algorithm_data.circles_index] = algorithm_data.ax_sum_in_circle;
        algorithm_data.ay_sum_circle[algorithm_data.circles_index] = algorithm_data.ay_sum_in_circle;
        algorithm_data.gz_r_sq_sum_circle[algorithm_data.circles_index] = algorithm_data.gz_r_squared_sum_in_circle;
     
        // 3. 更新累计和
        algorithm_data.total_ax_sum_circle += algorithm_data.ax_sum_in_circle;
        algorithm_data.total_ay_sum_circle += algorithm_data.ay_sum_in_circle;
        algorithm_data.total_gz_r_sq_sum_circle += algorithm_data.gz_r_squared_sum_in_circle;
        
        // 4. 更新索引
        algorithm_data.circles_index = (algorithm_data.circles_index + 1) % CIRCLE_SIZE;
        if (algorithm_data.circles_index == 0)
        {
            algorithm_data.circle_full = true;
        }
        
        // 5. 重置当前圈的累加值
        algorithm_data.ax_sum_in_circle = 0.0f;
        algorithm_data.ay_sum_in_circle = 0.0f;
        algorithm_data.gz_r_squared_sum_in_circle = 0.0f;
        algorithm_data.gz_dps_sum_in_circle = 0.0f;
        algorithm_data.gz_count = 0.0f;
        
        // 6. 使用20圈累计和计算虚拟半径
        if (algorithm_data.total_gz_r_sq_sum_circle > 0.0f)
        {
            // 计算x轴和y轴的虚拟半径
            float rx = algorithm_data.total_ax_sum_circle / algorithm_data.total_gz_r_sq_sum_circle;
            float ry = algorithm_data.total_ay_sum_circle / algorithm_data.total_gz_r_sq_sum_circle;
            
            // 限制x轴虚拟半径的上限
            if (rx > algorithm_setting.xr_limit)
            {
                rx = algorithm_setting.xr_limit;
            }
            else if (rx < -algorithm_setting.xr_limit)
            {
                rx = -algorithm_setting.xr_limit;
            }
            
            // 限制y轴虚拟半径的上限
            if (ry > algorithm_setting.yr_limit)
            {
                ry = algorithm_setting.yr_limit;
            }
            else if (ry < -algorithm_setting.yr_limit)
            {
                ry = -algorithm_setting.yr_limit;
            }
            
            // 更新算法数据中的虚拟半径
            algorithm_data.rx = rx;
            algorithm_data.ry = ry;
        }
    }
}

/**
  *******************************************************************************
 * @brief 倾角融合与数据质量评估主任务
 *
 * 主要功能：
 * 1. 周期性（50ms定时）接收事件，完成一次倾角融合、数据质量评估、虚拟半径计算等核心算法流程。
 * 2. 任务启动时初始化环形队列、权重队列、缓冲区等算法状态，并启动50ms定时器。
 * 3. 每次定时事件到来时，依次执行：
 *    - calibrate()：对加速度计数据进行离心力补偿，得到ax_cf_g、ay_cf_g等。
 *    - qualify_by_variance()：根据加速度计和陀螺仪的方差，评估当前数据质量等级（C0静止、C1匀速、C2变速）。
 *    - compute_ie()：融合加速度计、陀螺仪数据，计算多种倾角（inc1/2/3）、roll/pitch、高边等姿态参数，并进行中位数滤波。
 *    - interval_info_deal(false)：统计本周期内的最大/最小/均值等区间信息。
 *    - 记录任务执行时间。
 *    - send_msg()：将最新计算结果通过消息队列或串口发送到上位机或其他模块。
 *
 * 任务特点与注意事项：
 * - 该任务为FreeRTOS任务，需通过xTaskCreate在main_task中启动。
 * - 任务内部通过xTaskNotifyWait等待定时器事件，事件由50ms定时器回调函数触发。
 * - 任务涉及大量环形队列、权重、方差等算法状态变量，适合高频实时信号处理。
 * - 任务执行流程高度结构化，便于后续扩展如EKF、更多数据质量判据等。
 * - 任务执行时间已做统计，便于性能分析和优化。
 *
 * @param p 任务参数（未使用，保留接口一致性）
 * @return 无（任务函数不会返回）
   *******************************************************************************
 **/

void ie_task(void *p)
{
    algorithm_data.head = 0;                   // 初始化环形队列头指针
    algorithm_data.tail = 0;                   // 初始化环形队列尾指针
    algorithm_data.high_weight_incs_head = -1; // 初始化高权重倾角环形队列头指针
    algorithm_data.high_weight_incs_tail = -1; // 初始化高权重倾角环形队列尾指针
    inc_hs_data.buffer_index = 0;              // 初始化缓冲区索引

    reset_interval_info();

//    xTaskCreate(virtual_radius_task, "virtual_radius_task", 512, NULL, TASK_PRIORITY_VIRTUAL_RADIUS, &virtual_radius_task_handle);

    xTimerStart(xTimerCreate("ie_task_timer_50", TIMER_50MS, 1, 0, ie_task_timer_50ms_cb), 1000);

    for (;;)
    {
        uint32_t notify;

        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if (notify & EVENT_TIMER_50MS)
        {
                        // 记录开始时间
            sensor_data.start = xTaskGetTickCount();

            calibrate();
            qualify_by_variance();
            compute_ie();
            interval_info_deal(false);

             // 计算执行时间（微秒）
            sensor_data.end = xTaskGetTickCount();
            // 将tick转换为微秒 (假设tick为1ms)
            sensor_data.timelost = (sensor_data.end-sensor_data.start) ;
					

        }
//        send_msg();
    }
}
//START_TASK(ie_task, "ie_task", 640, NULL, TASK_PRIORITY_IE, &ie_task_handle);
#endif /* IE_TASK_C */