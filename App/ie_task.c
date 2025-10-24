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
#include "signal_process.h"

/* 宏定义 ----------------------------------------------------------------*/
/* 基础计算宏 */
#define power2(x) ((x) * (x))             /* 求平方 */
#define MIN(a, b) ((a) < (b) ? (a) : (b)) /* 取最小值 */
#define _180_Div_Pi 57.2957795f


/* 虚拟半径计算相关 */
#define CIRCLE_SIZE 30                    /* 存储用于计算虚拟半径的数组大小 */
#define Gz_COUNT_THRESHOLD 200           /* 计算虚拟半径时，Gz的计数值阈值 */

/* 时间相关定义 */
#define TIMER_10MS 10   /* 毫秒 */
#define TIMER_20MS 20   /* 毫秒 */
#define TIMER_50MS 50   /* 毫秒 */
#define TIMER_100MS 100 /* 毫秒 */
#define DELTA_TIME 0.05f /* 时间间隔  20Hz采样率*/

/* 事件定义 */
#define EVENT_TIMER_10MS (0X1)
#define EVENT_TIMER_20MS (0X1 << 1)
#define EVENT_TIMER_50MS (0X1 << 2)
#define EVENT_TIMER_100MS (0X1 << 3)

/* EKF参数定义 */
#define EKF_DT 0.05f        /* EKF采样周期，与TIMER_50MS对应 */
#define EKF_Q_ANGLE 0.0002f /* 过程噪声 */
#define EKF_Q_GYRO 0.001f   /* 陀螺仪过程噪声 */
#define EKF_R_ANGLE 0.08f   /* 测量噪声 */
#define MAX_R_MULTIPLIER 20000.0f /* 最大R_ANGLE倍数 */
#define MAX_Q_MULTIPLIER 2.0f     /* 最大Q倍数 */
#define GZ_STD_THRESHOLD_EKF 3.0f /* 用于EKF的Gz标准差阈值 */

/* 可调配参数 */
#define VARIANCE_THRESHOLD_Gz_C2 10.0f /* 标准差阈值 */
#define VARIANCE_THRESHOLD_Gz_C1 5.0f /* 入选C1-leavel的转速标准差阈值 */
#define RATATION_THRESHOLD_Gz_C0 36.0f /* 旋转阈值 */

/* 简化权重计算参数 */
#define HARSH_THRESHOLD 150    /* 标准差阈值 >150 */
#define MILD_THRESHOLD 100     /* 标准差阈值 >100 */
#define LESS_THRESHOLD 10      /* 标准差阈值 >10 */
#define NO_THRESHOLD 10        /* 标准差阈值 <10 */

/* 距离阈值定义 */
#define HARSH_DISTANCE 180     /* HARSH阈值对应的距离150 ，此数值修改时需小于GZ_STD_SNAPSHOT_LEN*/
#define MILD_DISTANCE 120      /* MILD阈值对应的距离 100*/
#define LESS_DISTANCE 60      /* LESS阈值对应的距离 50*/
#define NO_DISTANCE 40         /* NO阈值对应的距离20 */

/* 权重值定义 */
#define STD_DEV_PROPAGATION_FACTOR 0.005f /* 标准差传播因子 ，在线计算标准差时用于传播*/
#define HARSH_WEIGHT_NEAR 0.00000001f  /* HARSH阈值近距离权重 */
#define HARSH_WEIGHT_FAR 0.0001f        /* HARSH阈值远距离权重 */
#define MILD_WEIGHT_NEAR 0.0000001f    /* MILD阈值近距离权重 */
#define MILD_WEIGHT_FAR 0.0001f         /* MILD阈值远距离权重 */
#define LESS_WEIGHT_NEAR 0.000001f     /* LESS阈值近距离权重 */
#define LESS_WEIGHT_FAR 0.001f         /* LESS阈值远距离权重 */
#define NO_WEIGHT 0.01f           /* NO阈值远距离权重 */

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

/* 振动检测相关变量 */
#define VIBRATION_BUFFER_SIZE 20  // 20个元素，对应1秒钟的数据（20Hz）
static float vibration_buffer[VIBRATION_BUFFER_SIZE];  // 振动数据缓冲区
static uint8_t vibration_buffer_index = 0;             // 缓冲区索引
static bool vibration_buffer_full = false;             // 缓冲区是否已满
static uint32_t vibration_sample_count = 0;            // 振动采样计数

/* 钻进状态判断相关变量 */
#define DRILLING_HISTORY_SIZE 10  // 10秒历史记录
static float gz_avg_history[DRILLING_HISTORY_SIZE];    // z轴陀螺仪均值历史记录
static float std_v_norm_g_history[DRILLING_HISTORY_SIZE]; // 振动标准差历史记录
static uint8_t drilling_history_index = 0;             // 历史记录索引
static bool drilling_history_full = false;              // 历史记录是否已满

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
static float calculate_std_dev_welford(int16_t sample_count, uint16_t latest_index);
static void ekf_filter(float *acc_meas, float *gyro_meas, float *angle_out);
static void calculate_rotation_radius(float ax, float ay, float gz_dps, float gz_rad);
static void qualify_by_variance(void);
static void calibrate(void);
static void ie_task_timer_50ms_cb(TimerHandle_t xTimer);

/* 振动检测相关函数声明 */
static void vibration_data_collect(void);
static float calculate_vibration_std_dev(void);

/* 钻进状态判断相关函数声明 */
static void update_drilling_status(void);

/* 公共函数声明 */
void getRadiusOffset(float *ry, float *offsetX, float *offsetY, float *rx, float *yrLimit, float *xrLimit);
void compute_ie(void);
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
 * @Description: 根据加速度计和陀螺仪数据的方差计算数据质量等级并确认权重
 * @Parameters : 无
 * @RetValue   : 无
 * @Note       : 通过分析陀螺仪数据的方差来判断设备运动状态和手续数据的可靠性，并确认权重
 *               此方法要点在于
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

    // 将当前数据存入环形数组,sample_num和gz_std_dev_num用于判断是否需要重置
    int current_gz_sample_pos = algorithm_data.gz_dps_tail;
    int current_gz_std_dev_pos = algorithm_data.gz_std_dev_tail;

    algorithm_data.gz_dps_sample_num++;
    algorithm_data.gz_std_dev_num++;

    if(algorithm_data.gz_dps_sample_num > SAMPLE_LEN)
    {
        algorithm_data.gz_dps_sample_num = SAMPLE_LEN;
    }
    if(algorithm_data.gz_std_dev_num > GZ_STD_SNAPSHOT_LEN)
    {
        algorithm_data.gz_std_dev_num = GZ_STD_SNAPSHOT_LEN;
    }

    // 1. 存储转速用于样本计算（总共SAMPLE_LEN个样本）
    algorithm_data.gz_dps_sample[current_gz_sample_pos] = sensor_data.gz_dps;

    // 2. 计算转速标准差（总共GZ_STD_SNAPSHOT_LEN个样本）用于计算权重
    algorithm_data.gz_dps_sdv = calculate_std_dev_welford(SAMPLE_LEN, current_gz_sample_pos);
    algorithm_data.gz_std_dev_sample[current_gz_std_dev_pos] = algorithm_data.gz_dps_sdv;


    // 3. 根据简化规则计算最新数据的权重
    float weight = 0.0f;
    float min_influence = 1.0f; // 初始化为最大值，用于寻找最小影响度

    // 以最新数据为起点，遍历历史数据gz-std计算影响度，选择最小影响度作为权重
    // 注意gz-std是基于历史数据向前推的，而且大小可以和sample_len不同
    for (int i = 0; i < algorithm_data.gz_std_dev_num && i < GZ_STD_SNAPSHOT_LEN; i++)
    {
        int data_pos = (algorithm_data.gz_std_dev_head + i) % GZ_STD_SNAPSHOT_LEN;
        float influence = 0.0f;

        // 获取该位置的标准差
        float std_dev = algorithm_data.gz_std_dev_sample[data_pos];

        // 计算距离（相对于最新数据tail的位置）
        // 最新数据在tail，距离tail最近的是(tail-1)，head是最远的
        int distance_from_tail;
        if (data_pos == algorithm_data.gz_std_dev_tail)
        {
            distance_from_tail = 0; // 当前位置就是最新数据
        }
        else
        {
            // 计算从data_pos到tail的距离
            if (data_pos <= algorithm_data.gz_std_dev_tail)
            {
                distance_from_tail = algorithm_data.gz_std_dev_tail - data_pos;
            }
            else
            {
                // 处理环形队列的边界情况
                distance_from_tail = GZ_STD_SNAPSHOT_LEN - data_pos + algorithm_data.gz_std_dev_tail;
            }
        }

        // 根据标准差阈值和距离计算影响度
        if (std_dev > HARSH_THRESHOLD)
        {
            // HARSH_THRESHOLD >150: 距离HARSH_DISTANCE以内影响度HARSH_WEIGHT_NEAR，距离以上影响度HARSH_WEIGHT_FAR
            if (distance_from_tail <= HARSH_DISTANCE)
            {
                influence = HARSH_WEIGHT_NEAR;
            }
            else
            {
                influence = HARSH_WEIGHT_FAR;
            }
        }
        else if (std_dev > MILD_THRESHOLD)
        {
            // MILD_THRESHOLD >100: 距离MILD_DISTANCE以内影响度MILD_WEIGHT_NEAR，距离以上影响度MILD_WEIGHT_FAR
            if (distance_from_tail <= MILD_DISTANCE)
            {
                influence = MILD_WEIGHT_NEAR;
            }
            else
            {
                influence = MILD_WEIGHT_FAR;
            }
        }
        else if (std_dev > LESS_THRESHOLD)
        {
            // LESS_THRESHOLD >10: 距离LESS_DISTANCE以内影响度LESS_WEIGHT_NEAR，距离以上影响度LESS_WEIGHT_FAR
            if (distance_from_tail <= LESS_DISTANCE)
            {
                influence = LESS_WEIGHT_NEAR;
            }
            else
            {
                influence = LESS_WEIGHT_FAR;
            }
        }
        else
        {
               influence = NO_WEIGHT;

        }

        // 寻找最小影响度
        if (influence < min_influence)
        {
            min_influence = influence;
        }
    }

    // 将最小影响度作为最新数据的权重
    weight = min_influence;

    // 存储最新数据的权重
    algorithm_data.current_weight = weight;


    // 简化peace_time处理逻辑，基于标准差判断
    if (algorithm_data.gz_dps_sdv > VARIANCE_THRESHOLD_Gz_C1)
    {
        // 标准差较大，重置peace_time
        algorithm_data.peace_time_break = true;
        algorithm_data.peace_time = 0;
        algorithm_data.peacetime_threshod_write_done = false;
    }
    else
    {
        // 标准差较小，继续累积peace_time
        algorithm_data.peace_time_break = false;
        algorithm_data.peace_time += time_slot;
    }

    // 更新snapshot的指针
    // 增加两个数组中head的更新处理
    if(algorithm_data.gz_dps_sample_num >= SAMPLE_LEN)
    {
        algorithm_data.gz_dps_head = (algorithm_data.gz_dps_head + 1) % SAMPLE_LEN;
    }
    if(algorithm_data.gz_std_dev_num >= GZ_STD_SNAPSHOT_LEN)
    {
        algorithm_data.gz_std_dev_head = (algorithm_data.gz_std_dev_head + 1) % GZ_STD_SNAPSHOT_LEN;
    }


    algorithm_data.gz_dps_tail = (algorithm_data.gz_dps_tail + 1) % SAMPLE_LEN;
    algorithm_data.gz_std_dev_tail = (algorithm_data.gz_std_dev_tail + 1) % GZ_STD_SNAPSHOT_LEN;


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
    float local_hs;
    float inc_lpf_roll;
    float inc_lpf_pitch;
    float ax_lpf_g_square = sensor_data.ax_lpf_g * sensor_data.ax_lpf_g;
    float ay_lpf_g_square = sensor_data.ay_lpf_g * sensor_data.ay_lpf_g;
    float az_g_square = sensor_data.az_g * sensor_data.az_g;
    

    //1.计算井斜
    // 1.1 inc1:采用完全低通滤波井斜

    // 计算inc_lpf和inc_hpf 完全采用低通滤波，不考虑离心力影响
    inc = atan2(sqrt(ax_lpf_g_square + ay_lpf_g_square), sensor_data.az_g) * _180_Div_Pi;
    inc = inc < 0 ? -inc : inc;
		algorithm_data.inc_lpf=inc;
    inc_hs_data.inc1 = algorithm_data.inc_lpf;

    // 计算roll和pitch角度,NED坐标系，绕东轴旋转为pitch，绕北轴旋转为roll
    inc_lpf_pitch = atan2(sensor_data.ay_lpf_g,
                      sqrt(ax_lpf_g_square +
                           az_g_square)) * _180_Div_Pi;
    
    inc_lpf_roll = atan2(-sensor_data.ax_lpf_g,
                       sensor_data.ay_lpf_g) * _180_Div_Pi;
    
    // 更新到inc_hs_data中
    inc_hs_data.inc1_roll = inc_lpf_roll;
    inc_hs_data.inc1_pitch = inc_lpf_pitch;

    local_hs = atan2(sensor_data.ax_lpf_g,sensor_data.ay_lpf_g)*_180_Div_Pi;

    // 转换为[0, 360)范围
    if (local_hs < 0)
    {
        local_hs += 360.0f;
    }

    algorithm_data.hs_lpf = local_hs;
    inc_hs_data.hs_lpf = local_hs;


    // 1.2计算Inc2- 原inc2，根据转速以低通+带通滤波动态过滤xy轴加速度，考虑陀螺仪的数据离心力补偿后，并采用EKF卡尔曼平滑的实时井斜
    float acc_meas[3] = {sensor_data.ax_cf_g, sensor_data.ay_cf_g, sensor_data.az_cf_g};
    float gyro_meas[3] = {sensor_data.gx_rad, sensor_data.gy_rad, sensor_data.gz_rad};
    float angle_out;

    ekf_filter(acc_meas, gyro_meas, &angle_out);
    inc = fabsf(angle_out); // 取绝对值
    algorithm_data.inc2 = inc;
	inc_hs_data.inc2 = inc;

    // 1.3使用权重计算方式更新inc3：inc3 = (1-a)*inc3 + a*当前inc
    // 获取当前inc的权重（从snapshot中获取最新的权重）

    float inc_bpf =
    atan2(sqrt(power2(sensor_data.ax_cf_g) + power2(sensor_data.ay_cf_g)), sensor_data.az_cf_g) * _180_Div_Pi;
    inc_bpf = inc_bpf < 0 ? -inc_bpf : inc_bpf;
    

    local_hs = atan2(sensor_data.ax_cf_g, sensor_data.ay_cf_g) * _180_Div_Pi;
    if (local_hs < 0) {
        local_hs += 360.0f;
    }
//    excel中计算高边的公式：
//    =IF(ATAN2(-X轴数据,Y轴数据)*180/PI()<0,360+ATAN2(-X轴数据,Y轴数据)*180/PI(),ATAN2(-X轴数据,Y轴数据)*180/PI())
    algorithm_data.inc3 = (1.0f - algorithm_data.current_weight) * algorithm_data.inc3 + algorithm_data.current_weight * inc;
    algorithm_data.hs_bpf = (1.0f - algorithm_data.current_weight) *algorithm_data.hs_bpf + algorithm_data.current_weight*local_hs;
     algorithm_data.good_inc = algorithm_data.inc3;

    inc_hs_data.inc3 = algorithm_data.inc3;
    inc_hs_data.hs_bpf = algorithm_data.hs_bpf;
	inc_hs_data.good_inc=algorithm_data.good_inc;
    inc_hs_data.hs=inc_hs_data.hs_lpf;
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

    // 更新数据质量统计（基于标准差）
    if (algorithm_data.gz_dps_sdv <= VARIANCE_THRESHOLD_Gz_C1 && interval_info.c0_num_count < UINT16_MAX)
        interval_info.c0_num_count++;
    else if (algorithm_data.gz_dps_sdv <= VARIANCE_THRESHOLD_Gz_C2 && interval_info.c1_num_count < UINT16_MAX)
        interval_info.c1_num_count++;
    else if (interval_info.c2_num_count < UINT16_MAX)
        interval_info.c2_num_count++;

    // 更新振动标准差统计（每秒计算一次）
    static uint32_t vibration_std_calc_count = 0;
    vibration_std_calc_count++;

    // 每20次（1秒）计算一次振动标准差
    if (vibration_std_calc_count >= 20) {
        float current_std = calculate_vibration_std_dev();

        if (current_std > 0.0f) {  // 只有当缓冲区满时才更新统计
            // 更新最大值
            if (current_std > interval_info.std_v_norm_g_max)
                interval_info.std_v_norm_g_max = current_std;

            // 更新最小值（注意初始值的处理）
            if (interval_info.std_v_norm_g_min == 0.0f || current_std < interval_info.std_v_norm_g_min)
                interval_info.std_v_norm_g_min = current_std;

            // 更新平均值
            static uint32_t vibration_std_sample_count = 0;
            vibration_std_sample_count++;
            interval_info.std_v_norm_g_avg = (interval_info.std_v_norm_g_avg * (vibration_std_sample_count - 1) + current_std) / vibration_std_sample_count;
        }

        vibration_std_calc_count = 0;  // 重置计数器

        // 更新钻进状态判断（每秒调用一次）
        update_drilling_status();
    }

    // 增加计数，并防止溢出
    if (interval_info.acc_count < UINT32_MAX) {
        interval_info.acc_count++;
    }
}


/**
 * @brief  使用Welford在线算法计算环形数组中指定位置前后数据的标准差
 * @param  sample_count   - 需要计算的样本数量
 * @param  latest_index   - 环形数组中的最新数据位置索引
 * @return float          - 返回计算好的标准差
 * @note   使用Welford在线算法，只需一次遍历即可同时计算均值和方差，提高计算效率
 */
static float calculate_std_dev_welford(int16_t sample_count, uint16_t latest_index)
{
    float mean_gz = 0.0f;
    float M2_gz = 0.0f;

    // 检查参数有效性
    if (sample_count <= 0 || latest_index >= SAMPLE_LEN)
    {
        // 如果参数无效，返回0
        return 0.0f;
    }

    // 确保采样数量不超过可用数据量
    uint16_t available_samples = algorithm_data.gz_dps_sample_num;
    if (sample_count > available_samples)
    {
        sample_count = available_samples;
    }

    // 如果数据不足，返回0
    if (sample_count <= 1)
    {
        return 0.0f;
    }

    // 从最新位置开始，回溯环形数组计算标准差
    uint32_t start_idx = (latest_index - sample_count + 1 + SAMPLE_LEN) % SAMPLE_LEN;

    // 使用Welford在线算法，单次遍历计算均值和方差
    for (uint32_t i = 0; i < sample_count; i++)
    {
        uint32_t idx = (start_idx + i) % SAMPLE_LEN;
        
        // 从b_record[3]中获取gz数据
        float x_gz = algorithm_data.gz_dps_sample[idx];
        
        // Welford算法：计算增量均值
        float delta_gz = x_gz - mean_gz;
        mean_gz += delta_gz / (i + 1);
        
        // Welford算法：累积方差
        M2_gz += delta_gz * (x_gz - mean_gz);
    }

    // 计算最终方差（使用n-1作为分母，这是样本方差的无偏估计）
    float variance_gz = M2_gz / (sample_count - 1);

    // 计算标准差
    float std_dev_gz = sqrtf(variance_gz);

    // 时间加权标准差计算：由于gz_std_dev的波动性，需要进行时间加权平滑处理
    // 用于避免ax/ay的长效波动对inc3计算的影响
    if (algorithm_data.gz_dps_sdv > std_dev_gz)
    {
        std_dev_gz = STD_DEV_PROPAGATION_FACTOR * std_dev_gz + (1.0f - STD_DEV_PROPAGATION_FACTOR) * algorithm_data.gz_dps_sdv;
    }

    // 仅返回计算好的标准差，不进行数据插入
    return std_dev_gz;
}



/**
 * @brief  初始化EKF状态
 * @return None
 */
void ekf_init(void) {
    // 初始化状态
    ekf_state.angle = 0.0f;        // 假设初始倾角为0°
    ekf_state.bias = 0.0f;         // 假设初始偏差为0°/s

    // 初始化协方差矩阵
    ekf_state.P[0][0] = 1.0f;     // 角度初始不确定性：1°
    ekf_state.P[0][1] = 0.0f;     // 初始无相关性
    ekf_state.P[1][0] = 0.0f;     // 初始无相关性
    ekf_state.P[1][1] = 0.1f;     // 偏差初始不确定性：0.1°/s
}


/**
 * @brief  扩展卡尔曼滤波器实现
 * @param  acc_meas - 加速度计测量值数组 [ax, ay, az]
 * @param  gyro_meas - 陀螺仪测量值数组 [gx, gy, gz]
 * @param  angle_out - 输出角度指针
 * @return None
 * @note   使用EKF融合加速度计和陀螺仪数据计算倾角，并增加中位数均值滤波
*  转动时ax，ay的重力分量信号会随着转动形成一个与转动频率同频的正弦波信号，
*  最影响ay，ax的是gz的变化，如果gz变化小，ax没有角加速度，ay的离心力也可以被带通滤波清除掉，
*  问题是gz的不均匀性，一会儿大，一会儿小，也就是gz_std_dev数据太大，这个就会影响到加速度计，
*  在ekf卡尔曼滤波中如何就现有的代码，尽量少改动的前提下，针对gz_std_dev(转速变化的标准差）来进行建模平滑
*  1. 首先，需要对gz_std_dev进行平滑处理，以减少其对加速度计的影响。
*  2. 其次，需要对gz_std_dev进行建模，以预测其对加速度计的影响。

 */
static void ekf_filter(float *acc_meas, float *gyro_meas, float *angle_out)
{
    // 获取当前Gz的标准差
    float gz_std_dev = algorithm_data.gz_dps_sdv;

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
    acc_angle = acc_angle * _180_Div_Pi; // 转换为角度

    // 2. 预测步骤
    // 2.1 状态预测
    float gyro_rate = sqrtf(gyro_meas[0] * gyro_meas[0] + gyro_meas[1] * gyro_meas[1]); // 合成角速度
    float angle_pred = ekf_state.angle + (gyro_rate - ekf_state.bias) * EKF_DT;
    float bias_pred = ekf_state.bias; // 假设偏差保持不变

    // 2.2 误差协方差预测
//    float F[2][2] = {{1.0f, -EKF_DT}, {0.0f, 1.0f}};           // 状态转移矩阵
//    float Q[2][2] = {{dynamic_Q_ANGLE, 0.0f}, {0.0f, dynamic_Q_GYRO}}; // 使用动态调整的过程噪声协方差

    // P = F*P*F' + Q
    float P_temp[2][2];
    P_temp[0][0] = ekf_state.P[0][0] + (-EKF_DT * ekf_state.P[1][0]) + dynamic_Q_ANGLE;
    P_temp[0][1] = ekf_state.P[0][1] + (-EKF_DT * ekf_state.P[1][1]);
    P_temp[1][0] = ekf_state.P[1][0] + (-EKF_DT * ekf_state.P[0][0]);
    P_temp[1][1] = ekf_state.P[1][1] + dynamic_Q_GYRO;

    // 3. 更新步骤
    // 3.1 计算卡尔曼增益
//    float H[2] = {1.0f, 0.0f};            // 观测矩阵
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
    inc_hs->good_inc_prefilter = inc_hs_data.good_inc_prefilter;
    inc_hs->inc1 = inc_hs_data.inc1;
    inc_hs->inc2 = inc_hs_data.inc2;
    inc_hs->inc3 = inc_hs_data.inc3;
    inc_hs->hs = inc_hs_data.hs;
    inc_hs->hs_lpf = inc_hs_data.hs_lpf;
    inc_hs->hs_bpf = inc_hs_data.hs_bpf;
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

    // 振动标准差统计
    info->std_v_norm_g_max = interval_info.std_v_norm_g_max;
    info->std_v_norm_g_min = interval_info.std_v_norm_g_min;
    info->std_v_norm_g_avg = interval_info.std_v_norm_g_avg;
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
    if (algorithm_data.gz_count >= Gz_COUNT_THRESHOLD && fabs(algorithm_data.gz_dps_sum_in_circle)*DELTA_TIME < 360.0f)
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
    algorithm_data.gz_dps_head = 0;                   // 初始化gz-dps样本数组头指针
    algorithm_data.gz_dps_tail = 0;                   // 初始化gz-dps样本数组尾指针
    algorithm_data.gz_std_dev_head = 0;                     // 初始化gz-std样本数组头指针
    algorithm_data.gz_std_dev_tail = 0;                     // 初始化gz-std样本数组尾指针
    algorithm_data.gz_dps_sample_num = 0;                          // 初始化gz-dps样本数量
    algorithm_data.gz_std_dev_num = 0;                  // 初始化gz-std样本数量



    reset_interval_info();
	        ekf_init();

//    xTaskCreate(virtual_radius_task, "virtual_radius_task", 512, NULL, TASK_PRIORITY_VIRTUAL_RADIUS, &virtual_radius_task_handle);
    // 延迟启动定时器，确保ie_task_handle已经被设置
    vTaskDelay(pdMS_TO_TICKS(100));
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

            // 振动数据收集（每50ms收集一次，相当于20Hz）
            vibration_data_collect();

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

/**
 * @brief 振动数据收集函数
 * @note 每50ms调用一次，收集v_norm_g数据到缓冲区
 */
static void vibration_data_collect(void)
{
    // 将最新的v_norm_g数据放入缓冲区
    vibration_buffer[vibration_buffer_index] = sensor_data.v_norm_g;

    // 更新缓冲区索引
    vibration_buffer_index++;
    vibration_sample_count++;

    // 检查缓冲区是否已满
    if (vibration_buffer_index >= VIBRATION_BUFFER_SIZE) {
        vibration_buffer_full = true;
        vibration_buffer_index = 0;  // 循环使用缓冲区
        vibration_sample_count = 0;  // 重置振动采样计数
    }
}

/**
 * @brief 计算振动标准差
 * @return 标准差值，如果缓冲区未满则返回0
 */
static float calculate_vibration_std_dev(void)
{
    if (!vibration_buffer_full && vibration_sample_count < VIBRATION_BUFFER_SIZE) {
        return 0.0f;  // 缓冲区未满，返回0
    }

    // 计算均值
    float sum = 0.0f;
    int count = vibration_buffer_full ? VIBRATION_BUFFER_SIZE : vibration_sample_count;

    for (int i = 0; i < count; i++) {
        sum += vibration_buffer[i];
    }
    float mean = sum / count;

    // 计算方差
    float variance_sum = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = vibration_buffer[i] - mean;
        variance_sum += diff * diff;
    }
    float variance = variance_sum / count;

    // 返回标准差
    return sqrt(variance);
}

/**
 * @brief 更新钻进状态判断
 * @note 每1秒调用一次，判断是否在钻进状态
 *       滞后机制：进入条件宽松，退出条件严格
 *       进入：连续10秒中8秒满足条件 OR 连续10秒旋转
 *       退出：连续10秒中所有10秒都不满足条件 AND 连续10秒不旋转
 */
static void update_drilling_status(void)
{
    // 计算当前秒的z轴陀螺仪均值（使用已有的gz_dps_avg）
    float current_gz_avg = sensor_data.gz_dps;
    float current_std_v_norm_g = 0.0f;

    // 获取当前秒的振动标准差（如果缓冲区满的话）
    if (vibration_buffer_full) {
        current_std_v_norm_g = calculate_vibration_std_dev();
    }

    // 更新历史记录
    gz_avg_history[drilling_history_index] = current_gz_avg;
    std_v_norm_g_history[drilling_history_index] = current_std_v_norm_g;

    // 更新索引
    drilling_history_index++;
    if (drilling_history_index >= DRILLING_HISTORY_SIZE) {
        drilling_history_full = true;
        drilling_history_index = 0;  // 循环使用
    }

    // 只有历史记录满了才开始判断
    if (!drilling_history_full) {
        algorithm_data.drilling = false;  // 历史记录未满，设为静态
        return;
    }

    // 判断旋转条件：所有10秒的z轴陀螺仪均值都大于10dps(0.17rad/s)(0.6rpm)
    bool is_rotating = true;
    bool is_not_rotating = true;
    for (int i = 0; i < DRILLING_HISTORY_SIZE; i++) {
        if (gz_avg_history[i] <= 10.0f) {
            is_rotating = false;
        }
        if (gz_avg_history[i] > 10.0f) {
            is_not_rotating = false;
        }
    }

    // 判断振动条件：10秒中有8秒的振动标准差大于0.1
    int vibration_count = 0;
    int no_vibration_count = 0;
    for (int i = 0; i < DRILLING_HISTORY_SIZE; i++) {
        if (std_v_norm_g_history[i] > 0.1f) {
            vibration_count++;
        }
        if (std_v_norm_g_history[i] <= 0.1f) {
            no_vibration_count++;
        }
    }
    bool is_vibrating = (vibration_count >= 8);
    bool is_not_vibrating = (no_vibration_count >= 10);  // 所有10秒都不振动

    // 更新旋转状态（直接保存，供其他任务调用）
    algorithm_data.rotating = is_rotating;

    // ==================== 钻进状态判断逻辑 ====================
    // 基于滞后机制：进入宽松，退出严格，避免频繁切换
    //
    // 状态转换表：
    // 当前状态 | 旋转 | 振动 | 不旋转 | 不振动 | 结果
    // 不在钻进 | ✓ | ✗ | ✗ | ✗ | 进入钻进（旋转满足）
    // 不在钻进 | ✗ | ✓ | ✗ | ✗ | 进入钻进（振动满足）
    // 不在钻进 | ✗ | ✗ | ✓ | ✓ | 保持静态
    // 在钻进   | ✓ | ✗ | ✗ | ✗ | 保持钻进（旋转持续）
    // 在钻进   | ✗ | ✓ | ✗ | ✗ | 保持钻进（振动持续）
    // 在钻进   | ✗ | ✗ | ✓ | ✗ | 保持钻进（振动仍存在）
    // 在钻进   | ✗ | ✗ | ✗ | ✓ | 保持钻进（旋转仍存在）
    // 在钻进   | ✗ | ✗ | ✓ | ✓ | 退出钻进（两个条件都满足）
    // =========================================================

    static bool was_drilling = false;  // 记录上一秒的钻进状态

    if (!was_drilling) {
        // 进入条件（宽松）：旋转 OR 振动，只要有一个满足就进入
        algorithm_data.drilling = (is_rotating || is_vibrating);
    } else {
        // 退出条件（严格）：必须连续10秒不旋转 AND 连续10秒不振动
        algorithm_data.drilling = !(is_not_rotating && is_not_vibrating);
    }

    // 更新状态记录
    was_drilling = algorithm_data.drilling;
}
#endif /* IE_TASK_C */
