/**
 *****************************************************************************
 * FILENAME   : signal_process.c
 * COPYRIGHT  : MD.Tec(ShangHai) Co.,Ltd2024.
 * CREATEDDATE: 2025.04.06 22:04:20 Monday
 * DESCRIPTION: 传感器信号处理模块，负责根据配置选择和处理不同传感器的信号
 *
 *
 * EDIT HISTORY:
 *   NAME                        DATE                      CONTENT
 * Gordon              2025.04.07                   重构代码，添加滤波、传感器选择和迟滞逻辑
 * Gordon              2025.05.18                   修改algorithm_setting中加速度计和陀螺仪数据成员变量定义顺序，方便现行的flashDB按照key-value进行简化存储
                                                    取消algorithm_setting中degrees阶数定义，通过下发高阶为0参数进行处理，当阶数小于5时，高阶系数为0，不影响计算结果

 *****************************************************************************
 */
/************* Included files, Macros, Various and Declarations ***************/
#include "signal_process.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
//#include "main.h"  // 这个已经包含了spi.h
#include "ie_task.h"  // 添加ie_task.h头文件，用于访问sensor_data结构体
#include "ads1278-2.h"  // 使用新版本的ADS1278头文件
//#include "../Common/fir_config.h"  // 添加FIR滤波器配置头文件
//#include "IS25LP032_flash.h"  // 添加Flash存储头文件


// 任务句柄
TaskHandle_t signal_process_task_handle;                        // 将static改为全局变量

// 事件定义
#define EVENT_SIGNAL_PROCESS_50MS_TIMER 0x02000000              //50ms定时器事件

// 声明外部变量
extern sensor_data_t sensor_data;                               //传感器数据结构体（ie_task.c）
extern iam_global_raw_data_t s_iam_global_raw_data;             //IAM-20680HT传感器数据结构体（iam_20680ht.c）
extern ads1278_global_raw_data_t s_ads1278_global_raw_data;     //VS1005传感器数据结构体（ads1278-2.c）
extern algorithm_setting_t algorithm_setting;                   // 算法设置 （ie_task.c）
sensor_signal_t sensor_signal ={0};                     //用于保存所选原始传感器数据
filtered_signal_t filtered_sensor_signal={0} ;          //用于保存滤波后的传感器数据


//声明内部全局变量
static float temp_buffer[3] = {0};                              // 温度中值滤波缓冲区

//用于测试z轴陀螺仪的零偏
//static float gz_dps_average_BMW = 0.0f;                             // gz_dps的平均值
//static float gz_dps_average_KBW = 0.0f;                             // gz_dps的平均值，凯赛窗，和布莱克曼窗差别很小
static float gz_dps_average_LPF = 0.0f;                             // gz_dps的平均值
static int gz_count;
//static float gz_dps_average_RAW = 0.0f;                             // gz_dps的平均值

//static float gz_dps_BMW;
//static float gz_dps_KBW;

//static float gz_dps_average_RAW_array[10] = {0};  // 大小为10的数组
//static float gz_dps_average_LPF_array[10] = {0};  // 大小为10的数组
//static float gz_dps_average_BMW_array[10] = {0};  // 大小为10的数组
//static float gz_dps_average_KBW_array[10] = {0};  // 大小为10的数组 和布莱克曼窗差别很小，零偏差几十

//static uint8_t gz_dps_average_index = 0;   											// 当前写入位置索引
//static uint32_t gz_dps_count = 0;                               // 计数器
//#define GZ_DPS_AVERAGE_MAX_COUNT 2400                           // 最大计算次数120秒 50ms*2400/1000 =120 

// 低通-带通切换迟滞
#define HYSTERESIS_HIGH 0.2f        // 低通向带通切换的迟滞
#define HYSTERESIS_LOW 0.15f         // 带通向低通切换的迟滞

 /*
  *******************************************************************************
 * @brief 对传感器数据进行温度补偿
 * @param ax - 加速度计X轴数据指针
 * @param ay - 加速度计Y轴数据指针
 * @param az - 加速度计Z轴数据指针
 * @param gx - 陀螺仪X轴数据指针
 * @param gy - 陀螺仪Y轴数据指针
 * @param gz - 陀螺仪Z轴数据指针
 * @param t - 当前温度值
 * @note 该函数实现了基于温度的多项式补偿算法，用于消除传感器数据随温度变化的漂移
 *       补偿系数存储在algorithm_setting.degrees数组中，支持0-5阶多项式拟合
 *       每个轴都有独立的温度补偿系数
  *******************************************************************************
 */
void fitting(float t)
{
    // 限制温度在设定的上下限范围内
    if (t < algorithm_setting.t_comp_lower_limit) {
        t = algorithm_setting.t_comp_lower_limit;
    } else if (t > algorithm_setting.t_comp_upper_limit) {
        t = algorithm_setting.t_comp_upper_limit;
    }
    

    // 计算加速度计每轴的补偿值
    for (int axis = 0; axis < 3; axis++) {
        float compensation = 0.0f;
        float t_power = 1.0f;  // t^0 = 1
        
			// 计算5阶的补偿值,当实际阶数小于5时，上位机下发的高阶系数为0
        for (int order = 0; order <= 5; order++) {
            compensation += algorithm_setting.degrees_acc[axis * 6 + order] * t_power;
            t_power *= t;  // 计算下一个幂次
        }
        
        // 应用补偿
        sensor_signal.ax_raw_fit = sensor_signal.ax_raw - compensation;
        sensor_signal.ay_raw_fit = sensor_signal.ay_raw - compensation;
        sensor_signal.az_raw_fit = sensor_signal.az_raw - compensation;
			}
		
		//计算陀螺仪每轴的补偿值
    for (int axis = 0; axis < 3; axis++) {
        float compensation = 0.0f;
        float t_power = 1.0f;  // t^0 = 1
        
			// 计算5阶的补偿值,当实际阶数小于5时，上位机下发的高阶系数为0
        for (int order = 0; order <= 5; order++) {
            compensation += algorithm_setting.degrees_gyro[axis * 6 + order] * t_power;
            t_power *= t;  // 计算下一个幂次
        }
        				
        sensor_signal.gx_raw_fit = sensor_signal.gx_raw - compensation;
        sensor_signal.gy_raw_fit = sensor_signal.gy_raw - compensation;
        sensor_signal.gz_raw_fit = sensor_signal.gz_raw - compensation;
    }
}

/**
  *******************************************************************************
 * @brief 对加速度计三轴数据进行MS矩阵和零偏补偿，并对陀螺仪原始数据进行类型适配与零偏补偿
 * 
 * 该函数主要实现两部分功能：
 * 1. 对加速度计三轴（ax、ay、az）进行零偏（Bias）和MS矩阵（Misalignment & Scale）补偿，输出补偿后的 ax_g_offset、ay_g_offset、az_g_offset。
 *    - 补偿公式：
 *        ax_g_offset = ms_xx * ax_raw_fit + ms_xy * ay_raw_fit + ms_xz * az_raw_fit
 *        ay_g_offset = ms_yy * ay_raw_fit + ms_yz * az_raw_fit
 *        az_g_offset = ms_zz * az_raw_fit
 *    - 其中 ms_xx、ms_xy、ms_xz、ms_yy、ms_yz、ms_zz 为MS矩阵参数，ax_raw_fit等为温度补偿后的原始加速度。
 * 
 * 2. 对陀螺仪三轴原始数据（gx_raw、gy_raw、gz_raw）根据传感器类型进行单位换算和零偏补偿，输出 gx_dps、gy_dps、gz_dps。
 *    - 若为IAM-20680HT（SIGNAL_PROCESS_GYRO_HT20680），则原始值需乘以4000/65536后减去零偏；
 *    - 若为ADXRS645，则原始值需先减去4023000再除以16110后减去零偏。
 *    - 零偏参数分别为 gyro_x_bias、gyro_y_bias、gyro_z_bias。
 * 
 * @note
 * - 本函数不带输入参数，直接操作全局变量 sensor_signal 及 algorithm_setting。
 * - 调用前需确保 sensor_signal 的 *_raw_fit、*_raw 等成员已完成温度补偿和原始数据更新。
 * - 本函数为信号处理流程中的核心补偿步骤，需在滤波和温度补偿后调用。
   *******************************************************************************
 */
void imu()
{
    //1.对加速度计：M*S*(b-B)：
    filtered_sensor_signal.ax_g = algorithm_setting.ms_xx * (filtered_sensor_signal.ax_raw - algorithm_setting.ax_bias) + algorithm_setting.ms_xy * (filtered_sensor_signal.ay_raw - algorithm_setting.ay_bias) + algorithm_setting.ms_xz * (filtered_sensor_signal.az_raw - algorithm_setting.az_bias);
    filtered_sensor_signal.ay_g = algorithm_setting.ms_yy * (filtered_sensor_signal.ay_raw - algorithm_setting.ay_bias) + algorithm_setting.ms_yz * (filtered_sensor_signal.az_raw - algorithm_setting.az_bias);
    filtered_sensor_signal.az_g = algorithm_setting.ms_zz * (filtered_sensor_signal.az_raw - algorithm_setting.az_bias);

    //2.对陀螺仪： 通过algorithm_setting.gx_scale，algorithm_setting.gy_scale，algorithm_setting.gz_scale进行scale补偿
    filtered_sensor_signal.gz_dps = (float)(sensor_signal.gz_raw - algorithm_setting.gz_bias)* algorithm_setting.gz_scale;
	  filtered_sensor_signal.gx_dps = 0.0f;
    filtered_sensor_signal.gy_dps = 0.0f;
}

/**
  *******************************************************************************
	* @Description: 计算固定size为3的数组的中位数
  * @Parameters : array - 输入数组
  * @RetValue   : 返回数组的中位数
  * @Note       : 用于温度信号的中值滤波
  * @CreatedBy  : Gordon
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
static float calculate_median(float *array)
{
    // 直接使用数组元素进行比较，不创建局部变量
    if (array[0] <= array[1]) {
        if (array[1] <= array[2]) {
            return array[1];  // array[0] <= array[1] <= array[2]
        } else if (array[0] <= array[2]) {
            return array[2];  // array[0] <= array[2] < array[1]
        } else {
            return array[0];  // array[2] < array[0] <= array[1]
        }
    } else {  // array[1] < array[0]
        if (array[0] <= array[2]) {
            return array[0];  // array[1] < array[0] <= array[2]
        } else if (array[1] <= array[2]) {
            return array[2];  // array[1] <= array[2] < array[0]
        } else {
            return array[1];  // array[2] < array[1] < array[0]
        }
    }
}

// 初始化信号处理模块
void signal_process_init(void)
{
    // 初始化ARM FIR滤波器
    fir_init_all();
    // 初始化当前滤波器类型为默认的0.3-0.45Hz低通
    sensor_signal.current_filter_type = FIR_LOW_20_030_045;
}

// 选择信号源
void select_signal_source_and_update(void)
{
    vTaskSuspendAll();
    // 根据传感器类型选择信号源
    switch(algorithm_setting.acc_sensor_type) {
        case SIGNAL_PROCESS_ACC_HT20680:
            // 直接访问IAM数据
            sensor_signal.ax_raw = s_iam_global_raw_data.ax_raw;
            sensor_signal.ay_raw = s_iam_global_raw_data.ay_raw;
            sensor_signal.az_raw = s_iam_global_raw_data.az_raw;
            sensor_signal.t_raw = s_iam_global_raw_data.t_raw;
            break;
        case SIGNAL_PROCESS_ACC_VS10XX:
        case SIGNAL_PROCESS_ACC_MINIQ:
            // 直接访问ADS1278数据
            sensor_signal.ax_raw = s_ads1278_global_raw_data.ax_raw;
            sensor_signal.ay_raw = s_ads1278_global_raw_data.ay_raw;
            sensor_signal.az_raw = s_ads1278_global_raw_data.az_raw;
            sensor_signal.t_raw = s_ads1278_global_raw_data.t_raw;
            break;
        default:
            // 默认使用ADS1278数据
            sensor_signal.ax_raw = s_ads1278_global_raw_data.ax_raw;
            sensor_signal.ay_raw = s_ads1278_global_raw_data.ay_raw;
            sensor_signal.az_raw = s_ads1278_global_raw_data.az_raw;
            sensor_signal.t_raw = s_ads1278_global_raw_data.t_raw;
            break;
    }

    switch(algorithm_setting.gyro_sensor_type) {
        case SIGNAL_PROCESS_GYRO_HT20680:
            // 直接访问IAM数据
            sensor_signal.gx_raw = s_iam_global_raw_data.gx_raw;
            sensor_signal.gy_raw = s_iam_global_raw_data.gy_raw;
            sensor_signal.gz_raw = s_iam_global_raw_data.gz_raw;
            break;
        case SIGNAL_PROCESS_GYRO_ADXRS645:
            // 直接访问ADS1278数据
            sensor_signal.gx_raw = s_ads1278_global_raw_data.gx_raw;
            sensor_signal.gy_raw = s_ads1278_global_raw_data.gy_raw;
            sensor_signal.gz_raw = s_ads1278_global_raw_data.gz_raw;
            break;
        default:
            // 默认使用ADS1278数据
            sensor_signal.gx_raw = s_ads1278_global_raw_data.gx_raw;
            sensor_signal.gy_raw = s_ads1278_global_raw_data.gy_raw;
            sensor_signal.gz_raw = s_ads1278_global_raw_data.gz_raw;
            break;
    }
    xTaskResumeAll();
    // 更新温度缓冲区
    temp_buffer[0] = temp_buffer[1];
    temp_buffer[1] = temp_buffer[2];
    temp_buffer[2] = sensor_signal.t_raw;

    // 更新为温度中值
    sensor_signal.t_raw = calculate_median(temp_buffer);
}

/**
  *******************************************************************************
  * @Description: 对传感器数据进行calibrate和FIR滤波处理
  * @Parameters : 无
  * @RetValue   : 无
  * @Note       : 包括温度、加速度和陀螺仪数据的滤波处理
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
void signal_process_fir_filter(void)
{

    //前提：目前只支持单套传感器参数存储，即：fitting，imu，零偏等在cfg结构体中只能有一个传感器参数

    //0.根据传感器类型计算温度参数,单位为℃
    
    float t_compensation_param = 0;
		t_compensation_param = sensor_signal.t_raw*algorithm_setting.t_scale + algorithm_setting.t_intercept ;
   
    // 限制温度补偿参数在设定的上下限范围内
    // 默认范围为-20°C到150°C，可通过algorithm_setting中的temp_comp_lower_limit和temp_comp_upper_limit配置
    if (t_compensation_param < algorithm_setting.t_comp_lower_limit) {
        t_compensation_param = algorithm_setting.t_comp_lower_limit;
    } else if (t_compensation_param > algorithm_setting.t_comp_upper_limit) {
        t_compensation_param = algorithm_setting.t_comp_upper_limit;
    }

    //1. 对加速度计与陀螺仪信号进行1.1温度补偿，1.2 misalignment补偿，1.3 scale补偿，单位为g与dps
    
    //1.1调用fitting函数，对sensor_signal结构体中的加速度计和陀螺仪进行补偿
    //包括ax_raw_fit,ay_raw_fit,az_raw_fit,gx_raw_fit,gy_raw_fit,gz_raw_fit成员变量
    fitting(t_compensation_param);

    //1.2 note.在带通滤波中，基准装配偏差等直流分量会被滤掉，所以带通滤波后在对装配误差补偿会因为重复补偿而失准
    //而没有旋转时的低通滤波则不会滤掉装配误差等直流分量，所以正确选择是在滤波前把直流分量（装配偏差等）都先补偿，避免放在滤波后补偿 
    //而algorithm_setting.acc_x_offset,algorithm_setting.acc_y_offset,algorithm_setting.acc_z_offset是经过imu补偿后的以g为单位的装配偏差等直流分量
    //所以需要把装配偏差等直流分量转换为原来的raw为单位，需要对上三角函数（MS矩阵）进行逆矩阵变换
    //为简单起见，就直接除以MS矩阵的对角元素，忽略xy，xz，yz的轴交误差矩阵系数
    float ax_offset_in_raw = algorithm_setting.acc_x_offset / algorithm_setting.ms_xx ;
    float ay_offset_in_raw = algorithm_setting.acc_y_offset / algorithm_setting.ms_yy ;
    float az_offset_in_raw = algorithm_setting.acc_z_offset / algorithm_setting.ms_zz ;

    sensor_signal.ax_raw_offset = sensor_signal.ax_raw_fit - ax_offset_in_raw;
    sensor_signal.ay_raw_offset = sensor_signal.ay_raw_fit - ay_offset_in_raw;
    sensor_signal.az_raw_offset = sensor_signal.az_raw_fit - az_offset_in_raw;
 
    // 2.处理陀螺仪和加速度计的滤波
    fir_switch_type(FIR_LOW_20_000_980);
    arm_fir_f32(&fir_gyro_z_instance, &sensor_signal.gz_raw, &filtered_sensor_signal.gz_raw, 1);//陀螺仪z轴滤波
		/*
		gz_count++;
		gz_dps_average_LPF = (float)(gz_dps_average_LPF*gz_count+filtered_sensor_signal.gz_raw)/(gz_count+1);
		*/
 
    fir_switch_type(FIR_LOW_20_050_BMW);
    arm_fir_f32(&fir_acc_z_instance, &sensor_signal.az_raw_offset, &filtered_sensor_signal.az_raw, 1);//加速度计z轴滤波

    // 3.动态选择加速度计X/Y的滤波器
    float rotation_freq = fabs(filtered_sensor_signal.gz_dps / 360.0f);
    FIR_FilterType target_filter = FIR_LOW_20_050_BMW; // 默认0.5Hz布莱克曼窗低通

    // 3.1 选出目标滤波器
    if (rotation_freq > 0.2f) {
        if (rotation_freq <= FIR_BAND_20_012_085_CENTER) {
            target_filter = FIR_BAND_20_012_085;
        } else if (rotation_freq <= FIR_BAND_20_025_110_CENTER) {
            target_filter = FIR_BAND_20_025_110;
        } else if (rotation_freq <= FIR_BAND_20_055_145_CENTER) {
            target_filter = FIR_BAND_20_055_145;
        } else if (rotation_freq <= FIR_BAND_20_085_175_CENTER) {
            target_filter = FIR_BAND_20_085_175;
        } else if (rotation_freq <= FIR_BAND_20_115_205_CENTER) {
            target_filter = FIR_BAND_20_115_205;
        } else if (rotation_freq <= FIR_BAND_20_145_235_CENTER) {
            target_filter = FIR_BAND_20_145_235;
        } else if (rotation_freq <= FIR_BAND_20_175_265_CENTER) {
            target_filter = FIR_BAND_20_175_265;
        } else if (rotation_freq <= FIR_BAND_20_205_295_CENTER) {
            target_filter = FIR_BAND_20_205_295;
        } else if (rotation_freq <= FIR_BAND_20_235_325_CENTER) {
            target_filter = FIR_BAND_20_235_325;
        } else if (rotation_freq <= FIR_BAND_20_265_355_CENTER) {
            target_filter = FIR_BAND_20_265_355;
        } else if (rotation_freq <= FIR_BAND_20_295_385_CENTER) {
            target_filter = FIR_BAND_20_295_385;
        } else if (rotation_freq <= FIR_BAND_20_325_415_CENTER) {
            target_filter = FIR_BAND_20_325_415;
        } else {
            target_filter = FIR_BAND_20_355_445;
        }
    }

    bool switch_or_not = true;
    // 3.2 迟滞机制
    if (target_filter >= FIR_BAND_20_012_085 && sensor_signal.current_filter_type <= FIR_LOW_20_050_BMW)
    { // 低通向带通的切换
        if (rotation_freq < HYSTERESIS_HIGH)
        {
            switch_or_not = false;
        }
    }
    else if (target_filter <= FIR_LOW_20_050_BMW && sensor_signal.current_filter_type >= FIR_BAND_20_012_085)
    { // 带通向低通的切换
        if (rotation_freq > HYSTERESIS_LOW)
        {
            switch_or_not = false;
        }
    }
    // 带通滤波器之间的迟滞机制
    else if (target_filter != sensor_signal.current_filter_type)
    {

        // 计算当前滤波器的中心频率
        float current_center_freq = 0.0f;
        switch(sensor_signal.current_filter_type) {
            case FIR_BAND_20_012_085: current_center_freq = FIR_BAND_20_012_085_CENTER; break;
            case FIR_BAND_20_025_110: current_center_freq = FIR_BAND_20_025_110_CENTER; break;
            case FIR_BAND_20_055_145: current_center_freq = FIR_BAND_20_055_145_CENTER; break;
            case FIR_BAND_20_085_175: current_center_freq = FIR_BAND_20_085_175_CENTER; break;
            case FIR_BAND_20_115_205: current_center_freq = FIR_BAND_20_115_205_CENTER; break;
            case FIR_BAND_20_145_235: current_center_freq = FIR_BAND_20_145_235_CENTER; break;
            case FIR_BAND_20_175_265: current_center_freq = FIR_BAND_20_175_265_CENTER; break;
            case FIR_BAND_20_205_295: current_center_freq = FIR_BAND_20_205_295_CENTER; break;
            case FIR_BAND_20_235_325: current_center_freq = FIR_BAND_20_235_325_CENTER; break;
            case FIR_BAND_20_265_355: current_center_freq = FIR_BAND_20_265_355_CENTER; break;
            case FIR_BAND_20_295_385: current_center_freq = FIR_BAND_20_295_385_CENTER; break;
            case FIR_BAND_20_325_415: current_center_freq = FIR_BAND_20_325_415_CENTER; break;
            case FIR_BAND_20_355_445: current_center_freq = FIR_BAND_20_355_445_CENTER; break;
            default: current_center_freq = 0.0f; break;
        }

        // 只有信号频率比当前滤波器的中心频率>0.3Hz时才切换
        if ( fabs(rotation_freq - current_center_freq) <= 0.3f)
        {
            switch_or_not = false;
        }
    }

    // 4 动态切换X/Y轴滤波器（仅在需要时切换）
    if (target_filter != sensor_signal.current_filter_type && switch_or_not)
    {
        fir_switch_type(target_filter);
        sensor_signal.current_filter_type = target_filter;
    }
    fir_switch_type(sensor_signal.current_filter_type);

    // 5. 对x，y轴加速度计进行滤波
    arm_fir_f32(&fir_acc_x_instance, &sensor_signal.ax_raw_offset, &filtered_sensor_signal.ax_raw, 1);
    arm_fir_f32(&fir_acc_y_instance, &sensor_signal.ay_raw_offset, &filtered_sensor_signal.ay_raw, 1);

   // 6.调用imu函数，对加速度计进行misalignment补偿和scale计算
    imu();   


    // 6. 更新温度
    filtered_sensor_signal.t_C = t_compensation_param;
}

// 更新sensor_data结构体
void update_sensor_data(void)
{
    // 将滤波后的数据更新到sensor_data结构体中
    sensor_data.ax_g = filtered_sensor_signal.ax_g;
    sensor_data.ay_g = filtered_sensor_signal.ay_g;
    sensor_data.az_g = filtered_sensor_signal.az_g;
    
    // 陀螺仪数据，单位转换为dps
    sensor_data.gx_dps = filtered_sensor_signal.gx_dps;
    sensor_data.gy_dps = filtered_sensor_signal.gy_dps;
    sensor_data.gz_dps = filtered_sensor_signal.gz_dps;
    
    // 陀螺仪数据，单位转换为rad/s
    sensor_data.gx_rad = filtered_sensor_signal.gx_dps * DEG_TO_RAD;
    sensor_data.gy_rad = filtered_sensor_signal.gy_dps * DEG_TO_RAD;
    sensor_data.gz_rad = filtered_sensor_signal.gz_dps * DEG_TO_RAD;
    
    // 温度
    sensor_data.t_C = filtered_sensor_signal.t_C;
   

    // 补偿离心力后的加速度（这里暂时与原始加速度相同，实际应用中可能需要根据旋转半径进行补偿）
//    sensor_data.ax_cf_g = filtered_sensor_signal.ax_g;
//    sensor_data.ay_cf_g = filtered_sensor_signal.ay_g;
//    sensor_data.az_cf_g = filtered_sensor_signal.az_g;

#if 0
    uint32_t tail_flag = 0x7f800000;
    float debug_data[7];


    debug_data[0] = sensor_data.ax_g;
    debug_data[1] = sensor_data.ax_cf_g;
		debug_data[2] = sensor_data.ay_g;
    debug_data[3] = sensor_data.ay_cf_g;
		//debug_data[4] = algorithm_data.inc1;
    //debug_data[5] = algorithm_data.inc2;
    //debug_data[6] = algorithm_data.inc3;



    LPUART0_send((uint8_t*)debug_data, 28);
    LPUART0_send((uint8_t*)&tail_flag, 4);
#endif
}



/**
  *******************************************************************************
  * @Description: 50ms定时器回调函数
  * @Parameters : xTimer - 定时器句柄
  * @RetValue   : 无
  * @Note       : 通知任务处理定时事件
  * @CreatedBy  : AI Assistant
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
static void signal_process_50ms_timer_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(signal_process_task_handle, EVENT_SIGNAL_PROCESS_50MS_TIMER, eSetBits, NULL);
}

/**
  *******************************************************************************
  * @Description: 信号处理任务
  * @Parameters : p - 任务参数
  * @RetValue   : 无
  * @Note       : 负责信号处理、滤波和数据更新
  * @CreatedBy  : AI Assistant
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
void signal_process_task(void *p)
{
    TimerHandle_t signal_process_50ms_timer = NULL;
    
    // 创建50ms定时器
    signal_process_50ms_timer = xTimerCreate("signal_process_50ms_timer",
                                            pdMS_TO_TICKS(50),
                                            pdTRUE,
                                            (void *)0,
                                            signal_process_50ms_timer_cb);
    
    if(signal_process_50ms_timer != NULL)
    {
        xTimerStart(signal_process_50ms_timer, 0);
    }
    
    for(;;)
    {
        // 等待事件
        uint32_t event = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &event, portMAX_DELAY);
        
        // 处理50ms定时器事件
        if(event & EVENT_SIGNAL_PROCESS_50MS_TIMER)
        {
            select_signal_source_and_update();
            signal_process_fir_filter();
            update_sensor_data();
        }
    }
}

// 创建信号处理任务，在main_task中被启动
//START_TASK(signal_process_task, "signal_process_task", 256, NULL, TASK_PRIORITY_SIGNAL_PROCESS, &signal_process_task_handle);
