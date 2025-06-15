/**
 *****************************************************************************
 * FILENAME   : IS25LP032_flash.h
 * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
 * CREATEDDATE: 2023.10.04 09:58:02 Wednesday
 * DESCRIPTION:
 *
 *
 * EDIT HISTORY:
 *   NAME                        DATE                      CONTENT
 * YangHaifeng               2023.10.04                    CREATE
 *****************************************************************************
 */
/******************************************************************************/
#ifndef _IS25LP032_FLASH_H_2023_10_04_09_58_2_759_
#define _IS25LP032_FLASH_H_2023_10_04_09_58_2_759_
#ifdef __cplusplus
extern "C"
{
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#define LOG_FLAG_VIBRATING 0X1              // 震动标志
#define LOG_FLAG_VIBRATING_FOR_ADXL357 0X10 // ADXL357震动标志

    typedef __packed struct _log_
    {
        uint32_t timestamp;         // 时间戳
        uint32_t reserved0;         // 保留字段0
        float ax;                   // X轴加速度
        float ay;                   // Y轴加速度
        float az;                   // Z轴加速度
        float gx;                   // X轴角速度
        float gy;                   // Y轴角速度
        float gz;                   // Z轴角速度
        float inc1_max;             // 倾角1最大值
        float inc1_min;             // 倾角1最小值
        float inc1_avg;             // 倾角1平均值
        float inc2_max;             // 倾角2最大值
        float inc2_min;             // 倾角2最小值
        float inc2_avg;             // 倾角2平均值
        float virtual_x_radius_min; // 虚拟X半径最小值
        float virtual_x_radius_max; // 虚拟X半径最大值
        float virtual_y_radius_min; // 虚拟Y半径最小值
        float virtual_y_radius_max; // 虚拟Y半径最大值
        float inc6_min;             // 倾角6最小值
        float inc6_max;             // 倾角6最大值
        float inc6_avg;             // 倾角6平均值
        float hs;                   // 高边
        float gz_max;               // Z轴角速度最大值
        float gz_min;               // Z轴角速度最小值
        float gz_avg;               // Z轴角速度平均值
        float temp;                 // 温度
        float roll;                 // 日志间隔内转速（没有使用）
        uint32_t diff_t;            // 时间差
        float virtual_x_radius_avg; // 虚拟X半径平均值（没有使用）
        float virtual_y_radius_avg; // 虚拟Y半径平均值（没有使用）
        float gz_dps_sdv_max;       // Z轴角速度标准差最大值
        float gz_dps_sdv_min;       // Z轴角速度标准差最小值
        float std_dev_ax_ay_max;    // XY轴加速度标准差之和的最大值
        float vibration_data_min;
        float vibration_data_max;
        float vibration_data_avg;
        uint32_t flag;             // 振动标志
        int c0_num_max;            // 计数器0最大值
        int c0_num_min;            // 计数器0最小值
        int c1_num_max;            // 计数器1最大值
        int c1_num_min;            // 计数器1最小值
        int c2_num_max;            // 计数器2最大值
        int c2_num_min;            // 计数器2最小值
        float max_peace_time_max;  // 最大平静时间最大值
        float max_peace_time_min;  // 最大平静时间最小值
        uint16_t peace_time_count; // 满足条件的次数
        float s_f32_36V;           // 电池电压值
        int16_t reserved1[1];      // 保留字段1
        uint16_t crc16;            // CRC16校验值
    } log_t;

    /******************************** Functions **********************************/
    /**
     *******************************************************************************
     * @Description: 初始化Flash存储器和相关配置
     * @Parameters : 无
     * @RetValue   : 0-成功，非0-失败
     * @Note       : 系统启动时必须调用此函数初始化Flash
     *******************************************************************************
     */
    int32_t is25pl032_flash_init(void);
    void is25pl032_flash_set_imu(double b[]);
    void is25pl032_flash_get_imu(double *b);
    void is25pl032_flash_set_px(float p[]);
    void is25pl032_flash_set_py(float p[]);
    void is25pl032_flash_set_pz(float p[]);
    void is25pl032_flash_get_px(float *b);
    void is25pl032_flash_get_py(float *b);
    void is25pl032_flash_get_pz(float *b);
    /**
     *******************************************************************************
     * @Description: 向Flash写入数据
     * @Parameters : flash_address - Flash地址
     *               data_buffer - 数据缓冲区
     *               len - 数据长度
     * @RetValue   : 0-成功，非0-失败
     * @Note       : 写入前会自动擦除对应扇区
     *******************************************************************************
     */
    int32_t is25pl032_flash_normal_write(uint32_t flash_address, uint8_t *data_buffer, uint32_t len);

    /**
     *******************************************************************************
     * @Description: 从Flash读取数据
     * @Parameters : flash_address - Flash地址
     *               data_buffer - 数据缓冲区
     *               len - 数据长度
     * @RetValue   : 0-成功，非0-失败
     * @Note       : 读取指定地址的数据到缓冲区
     *******************************************************************************
     */
    int32_t is25pl032_flash_normal_read(uint32_t flash_address, uint8_t *data_buffer, uint32_t len);

    /**
     *******************************************************************************
     * @Description: 擦除Flash扇区
     * @Parameters : address - 扇区地址
     *               check_empty - 是否检查扇区是否为空
     * @RetValue   : 0-成功，非0-失败
     * @Note       : 擦除4KB大小的扇区
     *******************************************************************************
     */
    int32_t is25pl032_flash_erase_sector(uint32_t address, uint32_t check_empty);

    /**
     *******************************************************************************
     * @Description: 读取一条日志记录
     * @Parameters : log - 日志结构体指针
     * @RetValue   : 1-成功读取，0-无日志可读
     * @Note       : 按顺序读取下一条日志记录
     *******************************************************************************
     */
    int32_t is25pl032_flash_read_one_log(log_t *log);

    /**
     *******************************************************************************
     * @Description: 写入一条日志记录
     * @Parameters : log - 日志结构体指针
     * @RetValue   : 0-成功，非0-失败
     * @Note       : 将日志记录写入Flash存储
     *******************************************************************************
     */
    int32_t is25pl032_flash_write_one_log(log_t *log);

    /**
     *******************************************************************************
     * @Description: 重置日志读取索引
     * @Parameters : 无
     * @RetValue   : 0-成功，非0-失败
     * @Note       : 将日志读取位置重置到最新记录
     *******************************************************************************
     */
    int32_t is25pl032_flash_reset_rd_index(void);

    /**
     *******************************************************************************
     * @Description: 删除所有日志记录
     * @Parameters : 无
     * @RetValue   : 0-成功，非0-失败
     * @Note       : 清空所有日志数据
     *******************************************************************************
     */
    int32_t is25pl032_flash_delete_all_log(void);

    /**
     *******************************************************************************
     * @Description: 获取日志保存周期
     * @Parameters : 无
     * @RetValue   : 日志保存周期（秒）
     * @Note       : 返回当前设置的日志保存时间间隔
     *******************************************************************************
     */
    uint32_t is25pl032_flash_get_log_period(void);

    /**
     *******************************************************************************
     * @Description: 设置日志保存周期
     * @Parameters : period - 日志保存周期（秒）
     * @RetValue   : 0-成功，非0-失败
     * @Note       : 设置日志记录的时间间隔
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_log_period(uint8_t period);

    /**
     *******************************************************************************
     * @Description: 重置设备配置为默认值
     * @Parameters : 无
     * @RetValue   : 0-成功，非0-失败
     * @Note       : 将所有配置参数恢复为出厂默认值
     *******************************************************************************
     */
    uint32_t is25pl032_flash_reset_cfg(void);

    /**
     *******************************************************************************
     * @Description: 获取各温度区间的运行时间
     * @Parameters : addr - 运行时间数组指针
     * @RetValue   : 温度区间数量
     * @Note       : 返回不同温度区间的累计运行时间
     *******************************************************************************
     */
    int32_t get_total_time_per_temp(uint16_t **addr);

    /**
     *******************************************************************************
     * @Description: 更新温度区间运行时间
     * @Parameters : temp - 当前温度
     * @RetValue   : 无
     * @Note       : 根据当前温度更新对应区间的运行时间
     *******************************************************************************
     */
    void update_total_time_per_temp(float temp);

    /**
     *******************************************************************************
     * @Description: 设置偏移值
     * @Parameters : index - 偏移索引
     *               d - 偏移值
     * @RetValue   : 无
     * @Note       : 设置传感器数据的偏移补偿值
     *******************************************************************************
     */
    void is25pl032_flash_set_offset(uint32_t index, float d);

    /**
     *******************************************************************************
     * @Description: 获取偏移值
     * @Parameters : index - 偏移索引
     * @RetValue   : 偏移值
     * @Note       : 获取传感器数据的偏移补偿值
     *******************************************************************************
     */
    float is25pl032_flash_get_offset(uint32_t index);

    // void is25pl032_flash_set_roll_pitch(uint32_t index, double d);
    // double is25pl032_flash_get_roll_pitch(uint32_t index);
    // void is25pl032_flash_set_roll_pitch_357(uint32_t index, double d);
    // double is25pl032_flash_get_roll_pitch_357(uint32_t index);

    // float get_iam_20680ht_b2_kp(void);
    // float get_iam_20680ht_b3_ki(void);
    // float get_iam_20680ht_b4_limiting(void);
    // float get_iam_20680ht_b5_max_rating(void);
    // float get_iam_20680ht_acc_zero_offset_x(void);
    // float get_iam_20680ht_acc_zero_offset_y(void);
    // float get_iam_20680ht_acc_zero_offset_z(void);

    /**
     *******************************************************************************
     * @Description: 设置设备参数
     * @Parameters :
     *               xr_limit - 设置半径
     *               yr_limit - 半径余量
     * @RetValue   : 无
     * @Note       : 设置设备的基本参数
     *******************************************************************************
     */
    void is25pl032_flash_set_param(uint8_t accfir, uint8_t gyrofir, double xr_limit, double yr_limit, double gain);

    /**
     *******************************************************************************
     * @Description: 获取设备参数
     * @Parameters :
     *               xr_limit - 设置半径指针
     *               rMargin - 半径余量指针
     * @RetValue   : 无
     * @Note       : 获取设备的基本参数
     *******************************************************************************
     */
    void is25pl032_flash_get_param(uint8_t *accfir, uint8_t *gyrofir, double *xr_limit, double *yr_limit, double *gain);

    /**
     *******************************************************************************
     * @Description: 获取陀螺仪X轴零偏
     * @Parameters : 无
     * @RetValue   : X轴零偏值
     * @Note       : 获取陀螺仪X轴的零偏补偿值
     *******************************************************************************
     */
    float get_gx_offset(void);

    /**
     *******************************************************************************
     * @Description: 获取陀螺仪Y轴零偏
     * @Parameters : 无
     * @RetValue   : Y轴零偏值
     * @Note       : 获取陀螺仪Y轴的零偏补偿值
     *******************************************************************************
     */
    float get_gy_offset(void);

    /**
     *******************************************************************************
     * @Description: 获取陀螺仪Z轴零偏
     * @Parameters : 无
     * @RetValue   : Z轴零偏值
     * @Note       : 获取陀螺仪Z轴的零偏补偿值
     *******************************************************************************
     */
    float get_gz_offset(void);

    /**
     *******************************************************************************
     * @Description: 设置多项式拟合参数
     * @Parameters : t - 参数类型（0-ax, 1-ay, 2-az, 3-gx, 4-gy, 5-gz）
     *               b - 多项式系数数组
     *               degree - 多项式阶数
     * @RetValue   : 无
     * @Note       : 设置传感器数据的多项式拟合参数
     *******************************************************************************
     */
    void is25pl032_flash_set_degree(uint8_t t, double b[], int8_t degree);

    /**
     *******************************************************************************
     * @Description: 获取多项式拟合参数
     * @Parameters : b - 多项式系数数组指针
     *               degree - 多项式阶数指针
     * @RetValue   : 无
     * @Note       : 获取传感器数据的多项式拟合参数
     *******************************************************************************
     */
    void is25pl032_flash_get_degree(double *b, int8_t *degree);

    /**
     *******************************************************************************
     * @Description: 设置产品ID
     * @Parameters : pro_id - 产品ID字符串
     *               len - 字符串长度
     * @RetValue   : 无
     * @Note       : 设置设备的产品标识符
     *******************************************************************************
     */
    void is25pl032_flash_set_proid(char pro_id[], uint8_t len);

    /**
     *******************************************************************************
     * @Description: 获取产品ID
     * @Parameters : pro_id - 产品ID字符串指针
     * @RetValue   : 无
     * @Note       : 获取设备的产品标识符
     *******************************************************************************
     */
    void is25pl032_flash_get_proid(char *pro_id);

    /**
     *******************************************************************************
     * @Description: 获取固件版本
     * @Parameters : version - 版本字符串指针
     * @RetValue   : 无
     * @Note       : 获取当前固件的版本号
     *******************************************************************************
     */
    void is25pl032_flash_get_version(char *version);

    /**
     *******************************************************************************
     * @Description: 获取加速度计型号
     * @Parameters : 无
     * @RetValue   : 加速度计型号代码
     * @Note       : 获取当前使用的加速度计型号
     *               10 = IAM-20680HT
     *               20 = VS1002
     *               21 = VS1005
     *               22 = VS1010
     *               23 = TS1002
     *               24 = TS1005
     *               25 = TS1010
     *               30 = MiniQ
     *******************************************************************************
     */
    uint8_t get_acc_sensor_type(void);

    /**
     *******************************************************************************
     * @Description: 获取陀螺仪型号
     * @Parameters : 无
     * @RetValue   : 陀螺仪型号代码
     * @Note       : 获取当前使用的陀螺仪型号
     *               10 = IAM-20680HT
     *               20 = ADXRS645HDYZ
     *******************************************************************************
     */
    uint8_t get_gyro_sensor_type(void);

    /**
     *******************************************************************************
     * @Description: 设置加速度计型号
     * @Parameters : type - 加速度计型号代码
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置当前使用的加速度计型号
     *               10 = IAM-20680HT
     *               20 = VS1002
     *               21 = VS1005
     *               22 = VS1010
     *               23 = TS1002
     *               24 = TS1005
     *               25 = TS1010
     *               30 = MiniQ
     *******************************************************************************
     */
    int32_t set_acc_sensor_type(uint8_t type);

    /**
     *******************************************************************************
     * @Description: 设置陀螺仪型号
     * @Parameters : type - 陀螺仪型号代码
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置当前使用的陀螺仪型号
     *               10 = IAM-20680HT
     *               20 = ADXRS645HDYZ
     *******************************************************************************
     */
    int32_t set_gyro_sensor_type(uint8_t type);

    /**
     *******************************************************************************
     * @Description: 获取温度补偿下限值
     * @Parameters : 无
     * @RetValue   : 温度补偿下限值（°C）
     * @Note       : 获取温度补偿有效范围的下限值
     *******************************************************************************
     */
    float get_temp_comp_lower_limit(void);

    /**
     *******************************************************************************
     * @Description: 获取温度补偿上限值
     * @Parameters : 无
     * @RetValue   : 温度补偿上限值（°C）
     * @Note       : 获取温度补偿有效范围的上限值
     *******************************************************************************
     */
    float get_temp_comp_upper_limit(void);

    /**
     *******************************************************************************
     * @Description: 设置温度补偿下限值
     * @Parameters : limit - 温度补偿下限值（°C）
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置温度补偿有效范围的下限值
     *******************************************************************************
     */
    int32_t set_temp_comp_lower_limit(float limit);

    /**
     *******************************************************************************
     * @Description: 设置温度补偿上限值
     * @Parameters : limit - 温度补偿上限值（°C）
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置温度补偿有效范围的上限值
     *******************************************************************************
     */
    int32_t set_temp_comp_upper_limit(float limit);
    /**
     *******************************************************************************
     * @Description: 设置泥浆脉冲数据重传次数
     * @Parameters : count - 重传次数
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置泥浆脉冲数据有效范围的重传次数
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_retry_count(uint32_t count);

    /**
     *******************************************************************************
     * @Description: 获取泥浆脉冲数据重传次数
     * @Parameters : 无
     * @RetValue   : 泥浆脉冲数据重传次数
     * @Note       : 获取泥浆脉冲数据有效范围的重传次数
     *******************************************************************************
     */
    uint32_t is25pl032_flash_get_retry_count(void);

    /**
     *******************************************************************************
     * @Description: 设置每组泥浆脉冲数据的间隔
     * @Parameters : Pulse_group_interval - 时间间隔
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置每组泥浆脉冲数据的有效间隔
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_Pulse_group_interval(uint32_t Pulse_group_interval);

    /**
     *******************************************************************************
     * @Description: 获取每组泥浆脉冲数据的间隔
     * @Parameters : 无
     * @RetValue   : 每组泥浆脉冲数据的间隔
     * @Note       : 获取每组泥浆脉冲数据的间隔
     *******************************************************************************
     */
    uint32_t is25pl032_flash_get_Pulse_group_interval(void);
    /**
     *******************************************************************************
     * @Description: 设置泥浆脉冲数据发送延时时间
     * @Parameters : Pulse_send_delay - 延时时间
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置泥浆脉冲数据发送延时时间
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_Pulse_send_delay(uint32_t Pulse_send_delay);
    /**
     *******************************************************************************
     * @Description: 获取泥浆脉冲数据发送延时时间
     * @Parameters : 无
     * @RetValue   : 泥浆脉冲数据发送延时时间
     * @Note       : 获取泥浆脉冲数据发送延时时间
     *******************************************************************************
     */
    uint32_t is25pl032_flash_get_Pulse_send_delay(void);
    /**
     *******************************************************************************
     * @Description: 设置泥浆脉冲数据定时发送时间
     * @Parameters : Pulse_auto_send - 定时发送时间
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置泥浆脉冲数据定时发送时间
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_Pulse_auto_send(uint32_t Pulse_auto_send);
    /**
     *******************************************************************************
     * @Description: 获取泥浆脉冲数据定时发送时间
     * @Parameters : 无
     * @RetValue   : 泥浆脉冲数据定时发送时间
     * @Note       : 获取泥浆脉冲数据定时发送时间
     *******************************************************************************
     */
    uint32_t is25pl032_flash_get_Pulse_auto_send(void);
    /**
     *******************************************************************************
     * @Description: 设置静态数据收集的时间
     * @Parameters : Static_data_collection - 静态数据收集的时间
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置静态数据收集的时间
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_Static_data_collection(uint32_t Static_data_collection);
    /**
     *******************************************************************************
     * @Description: 获取静态数据收集的时间
     * @Parameters : 无
     * @RetValue   : 静态数据收集的时间
     * @Note       : 获取静态数据收集的时间
     *******************************************************************************
     */
    uint32_t is25pl032_flash_get_Static_data_collection(void);
    /**
     *******************************************************************************
     * @Description: 设置泥浆脉冲发送的组数
     * @Parameters : Number_of_pluse_group - 泥浆脉冲发送的组数
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置泥浆脉冲发送的组数
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_Number_of_pluse_group(uint32_t Number_of_pluse_group);
    /**
     *******************************************************************************
     * @Description: 获取泥浆脉冲发送的组数
     * @Parameters : 无
     * @RetValue   : 泥浆脉冲发送的组数
     * @Note       : 获取泥浆脉冲发送的组数
     *******************************************************************************
     */
    uint32_t is25pl032_flash_get_Number_of_pluse_group(void);
    /**
     *******************************************************************************
     * @Description: 设置ADXL357振动阈值数据
     * @Parameters : vibration_threshold - ADXL357振动阈值
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置ADXL357振动阈值数据
     *******************************************************************************
     */

    uint32_t is25pl032_flash_set_vibration_threshold(float vibration_threshold);

    /**
     *******************************************************************************
     * @Description: 获取ADXL357振动阈值数据
     * @Parameters : 无
     * @RetValue   : ADXL357振动阈值数据
     * @Note       : 获取ADXL357振动阈值数据
     *******************************************************************************
     */
    float is25pl032_flash_get_vibration_threshold(void);

    /**
     *******************************************************************************
     * @Description: 设置ADXL357振动灵敏度
     * @Parameters : vibration_sensitivity - 灵敏值
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置ADXL357振动灵敏度
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_vibration_sensitivity(uint32_t vibration_sensitivity);

    /**
     *******************************************************************************
     * @Description: 获取ADXL357振动灵敏度
     * @Parameters : 无
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 获取ADXL357振动灵敏度
     *******************************************************************************
     */
    uint32_t is25pl032_flash_get_vibration_sensitivity(void);

    /**
     *******************************************************************************
     * @Description: 设置低功耗状态
     * @Parameters : enable - 使能状态（0-禁用，1-启用）
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置低功耗状态
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_idle_hook_enable(uint32_t enable);

    /**
     *******************************************************************************
     * @Description: 获取低功耗状态
     * @Parameters : 无
     * @RetValue   : 低功耗状态（0-禁用，1-启用）
     * @Note       : 获取低功耗状态
     *******************************************************************************
     */
    uint32_t is25pl032_flash_get_idle_hook_enable(void);

    /**
     *******************************************************************************
     * @Description: 获取校准数据
     * @Parameters : 无
     * @RetValue   : 校准数据值
     * @Note       : 获取校准数据
     *******************************************************************************
     */
    float is25pl032_flash_get_calibration_data(void);

    /**
     *******************************************************************************
     * @Description: 设置校准数据
     * @Parameters : calibration_data - 要设置的校准数据值
     * @RetValue   : 0-成功，-1-失败
     * @Note       : 设置校准数据
     *******************************************************************************
     */
    uint32_t is25pl032_flash_set_calibration_data(float calibration_data);
    /**
     *******************************************************************************
     * @Description: 获取校准高边后的数据，用于debug跟踪调试
     * @Parameters : hs - 要校准的高边数据
     * @RetValue   : 无
     * @Note       : 获取校准高边后的数据，用于debug跟踪调试
     *******************************************************************************
     */
    void is25pl032_flash_get_calibrated_high_data(double hs);

#ifdef __cplusplus
}
#endif
#endif
