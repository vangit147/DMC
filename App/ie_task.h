/**
 ******************************************************************************
 * @file    ie_task.h
 * @author  Gordon Li
 * @version V1.2
 * @date    2025-04-05 00:00:07 Saturday
 * @brief   倾角计算任务头文件
 *
 * @note
 *          - 定义了倾角计算相关的数据结构和函数接口
 *          - 包含算法参数配置和状态管理
 *
 * @update_history
 *          - 2025-04-05: 将algorithm_data改为全局变量
 *          - 2025-04-05: 增加dsp/fast_math_functions.h头文件包含
            - 2025-05-18：修改algorithm_setting的变量成员，把加速度计和陀螺仪数据成员变量，方便现行的flashDB按照key-value进行简化存储
 ******************************************************************************
 */

#ifndef _IE_TASK_H_2024_01_12_22_3_29_566_
#define _IE_TASK_H_2024_01_12_22_3_29_566_

#ifdef __cplusplus
extern "C"
{
#endif

/* 系统头文件包含 */

/* 项目头文件包含 */
#include "main.h"
#include "dsp/fast_math_functions.h"

/* 宏定义 */
#define DEG_TO_RAD 0.0174533f  // 角度转弧度系数 (π/180)，保留7位有效数字
#define GZ_STD_SNAPSHOT_LEN 200 /* 高边数据快照长度 */
#define SAMPLE_LEN 40 /* 标准差样本距离 */

/* 类型定义 */
// 算法信息结构体
typedef struct {
    /* 虚拟半径相关 */
    float ax_sum_in_circle;           /* 旋转一周内累计的ax */
    float ay_sum_in_circle;           /* 旋转一周内累计的ay */
    float gz_dps_sum_in_circle;       /* 一圈内累计的gz */
    float gz_r_squared_sum_in_circle; /* 旋转一周内累计的gz弧度平方和 */
    float ax_sum_circle[30];          /* ax累计圈数 */
    float ay_sum_circle[30];          /* ay累计圈数 */
    float gz_r_sq_sum_circle[30];     /* gz弧度平方累计圈数 */
    uint8_t gz_count;                 /* gz_count的计数 */
    int8_t circles_index;             /* 当前circle的index */
    float total_ax_sum_circle;        /* 整个circle数组的ax累加值 */
    float total_ay_sum_circle;        /* 整个circle数组的ay累加值 */
    float total_gz_r_sq_sum_circle;   /* 整个circle数组的gz弧度平方累加值 */
    bool circle_full;                 /* 是否已经填满30圈 */

    /* 倾角相关 */
    float rx;                    /* x轴虚拟半径 */
    float ry;                    /* y轴虚拟半径 */
    float inc_lpf;               /* 加速度解算倾斜角:完全采用加速度计的x，y轴低通滤波0.4Hz信号，不考虑离心力影响*/
    float inc2;                  /* 加速度解算倾斜角:采用加速度计的x，y轴低通+带通滤波，不考虑离心力影响* */
    float inc3;                  /* 信号质量权重倾斜角 */
    float good_inc;              /* 可信inc ,目前为inc3*/

    float hs_lpf;                /* 低通滤波高边 */
    float hs_bpf;                /* 带通滤波高边 */


    /* 数据质量因子 */
    float gz_dps_sdv;           /* 方差 */

    /* 数据记录 */
    float current_weight; /* 当前数据权重 */

    float gz_dps_sample[SAMPLE_LEN]; /* 数组用于存储转速 ，计算方差*/
    float gz_std_dev_sample[GZ_STD_SNAPSHOT_LEN]; /* 数组用于存储历史方差，计算权重 */

    int16_t gz_dps_sample_num; /* gz-dps样本数量 */
    int16_t gz_std_dev_num;    /* gz-标准差样本数量 */
    int16_t gz_dps_head;       /* gz-dps样本数组头指针 */
    int16_t gz_std_dev_head;   /* gz-标准差样本数组头指针 */
    int16_t gz_dps_tail;       /* gz-dps样本数组尾指针 */
    int16_t gz_std_dev_tail;   /* gz-标准差样本数组尾指针 */

    int16_t log_period_time;                /* 日志周期 */

    /* 稳定时间相关 */
    float peace_time;                    /* 持续静止或匀速转动的最长时间 */
    bool peace_time_break;               /* 稳定时间中断标志 */
    bool peacetime_threshod_write_done;  /* 超过threshold的peatime计数标志 */

    /* 钻进状态相关 */
    bool drilling;                       /* 钻进状态标志：1表示在钻进，0表示静态（既不振动也不旋转） */
    bool rotating;                       /* 独立旋转状态标志：1表示在旋转，0表示不旋转，可配合第三方振动开关来判断*/
} algorithm_info_t;

// 算法设置结构体
typedef struct
{
    /* sensor_type */
    uint8_t acc_sensor_type;  // 加速度计类型
    uint8_t gyro_sensor_type;  // 陀螺仪类型
    uint8_t mag_sensor_type;  // 磁力计类型
    /* 虚拟半径限 */
    float xr_limit;
    float yr_limit;
    uint16_t log_period_time; // 日志周期，单位：秒

    /* 稳定时间阈值设置 */
    int8_t max_peace_time_threshold; // 稳定时间阈值，用于统计在一个interval内超过该阈值的peace_time次数，默认值为5


    /* 加速度计相关设置（顺序参考板间通信holdings 寄存器表格顺序，） */
    float ms_xx; // X-X轴MS矩阵系数
    float ms_xy; // X-Y轴MS矩阵系数
    float ms_xz; // X-Z轴MS矩阵系数
    float ms_yy; // Y-Y轴MS矩阵系数
    float ms_yz; // Y-Z轴MS矩阵系数
    float ms_zz; // Z-Z轴MS矩阵系数
    float ax_bias; // X轴加速度计零偏, raw_data
    float ay_bias; // Y轴加速度计零偏，raw_data
    float az_bias; // Z轴加速度计零偏，raw_data
    float acc_x_offset; // X轴加速度计装配误差，单位：g
    float acc_y_offset; // Y轴加速度计装配误差，单位：g
    //加速度计温补矩阵
    double degrees_acc[18]; //加速度计温漂数组

    float acc_z_offset; // Z轴加速度计装配误差，单位：g  //方便holdings 寄存器 flashDB key-value数据修改


    /* 陀螺仪相关设置 */
    float gx_scale; // X轴陀螺仪比例系数，raw_data -> dps
    float gy_scale; // Y轴陀螺仪比例系数，raw_data -> dps
    float gz_scale; // Z轴陀螺仪比例系数，raw_data -> dps
    float gx_bias; // X轴陀螺仪零偏，raw_data
    float gy_bias; // Y轴陀螺仪零偏，raw_data
    float gz_bias; // Z轴陀螺仪零偏，raw_data
    /* 多项式拟合参数 */
    double degrees_gyro[18]; // 多项式拟合系数数组: 默认5阶


    /* 温度补偿范围设置 */
    float t_comp_lower_limit; // 温度补偿下限，默认-20°C
    float t_comp_upper_limit;
    float t_scale;           // 温度比例系数
    float t_intercept;       // 温度截距

} algorithm_setting_t;

// 区间信息结构体
typedef struct
{
    uint32_t acc_count; /* 累加次数 用于计算各种均值 约1000天@20ms执行一次*/

    /* 陀螺仪数据统计 */
    float gyro_z_dps_max; /* gz_dps最大值 */
    float gyro_z_dps_min; /* gz_dps最小值 */
    float gyro_z_dps_avg; /* gz_dps平均值 */

    /* 倾角数据统计 */
    float inc1_max, inc1_min, inc1_avg; /* inc1统计信息 */
    float inc2_max, inc2_min, inc2_avg; /* inc2统计信息 */
    float inc3_max, inc3_min, inc3_avg; /* inc3统计信息 */
    float good_inc_max, good_inc_min, good_inc_avg; /* good_inc统计信息 */

    /* 虚拟半径统计 */
    float radius_x_max, radius_x_min; /* x轴虚拟半径统计 */
    float radius_y_max, radius_y_min; /* y轴虚拟半径统计 */


    /* 陀螺仪标准差统计 */
    float sdv_gyro_z_max, sdv_gyro_z_min; /* z轴陀螺仪标准差统计 */

    /* 稳定时间统计 */
    float peace_time_max;      /* 稳定时间最长数值 */
    uint16_t peace_time_count; /* 满足条件的次数 */

    /* 数据质量等级统计 */
    uint16_t c0_num_count;
    uint16_t c1_num_count;
    uint16_t c2_num_count;

    /* 振动检测标准差统计 */
    float std_v_norm_g_max;    /* 振动标准差最大值 */
    float std_v_norm_g_min;    /* 振动标准差最小值 */
    float std_v_norm_g_avg;    /* 振动标准差平均值 */
} interval_info_t;

// 倾角高边结构体
typedef struct
{
    float hs;   // 高边
    float good_inc_prefilter; // 滤波前综合井斜
    float inc1; // 低通滤波井斜
    float inc2; // 离心力补偿倾斜角2以及EKF卡尔曼计算后的倾斜角
    float inc3; // 信号质量权重倾斜角
    float hs_lpf; // 低通滤波高边
    float hs_bpf; // 带通滤波高边

    float good_inc; //综合井斜，优先使用inc3，如果inc3在整个周期内不变则使用inc2
    float inc1_roll;  // inc1的横滚角
    float inc1_pitch; // inc1的俯仰角
    float inc3_roll;  // inc3的横滚角
    float inc3_pitch; // inc3的俯仰角

} inclination_hs_t;

// 传感器数据结构体
typedef struct
{
    // imu sensor
    float ax_g;          // X-Axis Acc （G为单位）
    float ay_g;          // Y-Axis Acc（G为单位）
    float az_g;          // Z-Axis Acc（G为单位）
    float ax_cf_g;       // 补偿离心力后的X-Axis加速度计数值
    float ay_cf_g;       // 补偿离心力后的Y-Axis加速度计数值
    float az_cf_g;       // 补偿离心力后的Y-Axis加速度计数值
    float gx_rad;        // X-Axis Gryroscope
    float gy_rad;        // Y-Axis Gryroscope
    float gz_rad;        // Z-Axis Gryroscope
    float gx_dps;        // X-Axis Gryroscope
    float gy_dps;        // Y-Axis Gryroscope
    float gz_dps;        // Z-Axis Gryroscope
    float t_C;          // 温度
    /* 任务执行时间相关变量 */
    uint32_t timelost;  // 信号处理任务执行时间（微秒）
    uint32_t start;
    uint32_t end;

    // 新增：X/Y轴加速度计专用4阶IIR低通滤波器实例
    float ax_lpf_g;    // ax轴：始终通过IIR低通滤波的原始信号
    float ay_lpf_g;    // ay轴：始终通过IIR低通滤波的原始信号

    // 振动检测相关
    float v_norm_g;    // 三轴加速度平方和的根（用于振动检测）
} sensor_data_t;

/* 全局变量声明 */
extern interval_info_t interval_info;
extern inclination_hs_t inc_hs_data;
extern algorithm_setting_t algorithm_setting;
extern TaskHandle_t ie_task_handle;
extern algorithm_info_t algorithm_data;  // 添加algorithm_data全局变量声明

/* 函数声明 */
int32_t get_inc_hs(inclination_hs_t* inc_hs);
void getRadiusOffset(float *r, float *offsetX, float *offsetY, float *xr, float *yrLimit, float *xrLimit);
void get_interval_info(interval_info_t* info);
void get_sensor_data(sensor_data_t* data);
void reset_interval_info(void);
void ie_task(void *p);

#ifdef __cplusplus
}
#endif

#endif /* _IE_TASK_H_2024_01_12_22_3_29_566_ */
