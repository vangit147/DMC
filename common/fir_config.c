/**
  *****************************************************************************
  * FILENAME   : fir_config.c
  * COPYRIGHT  : ModintTech(ShangHai) Co.,Ltd2024.
  * CREATEDDATE: 2025.01.24 13:21:34 Friday
  * DESCRIPTION: FIR滤波器配置实现（复用状态缓冲区和系数RAM）
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * Gordon                 		2025.01.24                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
// 系统头文件
#include <string.h>

// 项目头文件
#include "fir_config.h"
//#include "filtering_functions.h"

// 100Hz采样率FIR滤波器（用于ax/ay的FIR低通+IIR高通组合滤波）
#include "FIR_LOW_100_512_010_CHEB.h"  //100Hz低通滤波器（0.1Hz截止）
#include "FIR_LOW_100_512_070_CHEB.h"  //100Hz低通滤波器（0.7Hz截止）
#include "FIR_LOW_100_512_120_CHEB.h"  //100Hz低通滤波器（1.2Hz截止）
#include "FIR_LOW_100_512_170_CHEB.h"  //100Hz低通滤波器（1.7Hz截止）
#include "FIR_LOW_100_512_220_CHEB.h"  //100Hz低通滤波器（2.2Hz截止）
#include "FIR_LOW_100_512_270_CHEB.h"  //100Hz低通滤波器（2.7Hz截止）
#include "FIR_LOW_100_512_320_CHEB.h"  //100Hz低通滤波器（3.2Hz截止）
#include "FIR_LOW_100_512_370_CHEB.h"  //100Hz低通滤波器（3.7Hz截止）
#include "FIR_LOW_100_512_420_CHEB.h"  //100Hz低通滤波器（4.2Hz截止）

// 所有滤波器的系数定义（存储在Flash中）
const float32_t * const fir_coeffs_flash[NUM_FILTERS] = {
    fir_low_100_512_010_CHEB_coeffs,
    fir_low_100_512_070_CHEB_coeffs,
    fir_low_100_512_120_CHEB_coeffs,
    fir_low_100_512_170_CHEB_coeffs,
    fir_low_100_512_220_CHEB_coeffs,
    fir_low_100_512_270_CHEB_coeffs,
    fir_low_100_512_320_CHEB_coeffs,
    fir_low_100_512_370_CHEB_coeffs,
    fir_low_100_512_420_CHEB_coeffs
};

//---------------- 独立资源定义（加速度Z轴和陀螺仪z轴）----------------
// 注释掉Z轴加速度计FIR滤波器实例，改用8阶IIR滤波器（0.15Hz)
// arm_fir_instance_f32 fir_acc_z_instance;  // 加速度计Z轴实例
arm_fir_instance_f32 fir_acc_x_instance;  // 加速度计X轴实例
arm_fir_instance_f32 fir_acc_y_instance;  // 加速度计Y轴实例
//用中位数滤波+延时滤波代替Z轴陀螺仪FIR滤波器
//arm_fir_instance_f32 fir_gyro_z_instance; // 陀螺仪Z轴实例


// 公共系数矩阵
float32_t fir_shared_coeffs[FILTER_TAP_NUM];  // 所有滤波器共享的系数矩阵

// 状态缓冲区
// 注释掉Z轴加速度计FIR状态缓冲区，改用IIR滤波器
float32_t fir_acc_x_state[FILTER_TAP_NUM + 3];//加速度计X状态缓冲
float32_t fir_acc_y_state[FILTER_TAP_NUM + 3];//加速度计Y状态缓冲
//用中位数滤波+延时滤波代替Z轴陀螺仪FIR滤波器
//float32_t fir_gyro_z_state[FILTER_TAP_NUM + 3];//陀螺仪Z状态缓冲

uint32_t start_ticks = 0;
uint32_t end_ticks = 0;
uint32_t ticks_used = 0;

/******************************* 函数实现 ********************************/
/**
 * @brief 初始化所有FIR滤波器实例
 */
void fir_init_all(void) {

    // 初始化加速度计X轴滤波器
    arm_fir_init_f32(&fir_acc_x_instance, 
                     FILTER_TAP_NUM,
                     (float32_t *)fir_shared_coeffs,
                     fir_acc_x_state, 
                     1);

    // 初始化加速度计Y轴滤波器
    arm_fir_init_f32(&fir_acc_y_instance, 
                     FILTER_TAP_NUM,
                     (float32_t *)fir_shared_coeffs,
                     fir_acc_y_state, 
                     1);
                     

    // 设置默认滤波器类型
    fir_switch_type(FIR_LOW_100_512_010_CHEB);
}


/**
 * @brief 切换滤波器类型
 * @param new_type 新的滤波器类型
 */
void fir_switch_type(FIR_Lowpass_100_FilterType new_type) {
    // 获取新的滤波器系数
    const float32_t *coeffs = fir_get_coeffs(new_type);
    if (coeffs != NULL) {
        // 复制新的滤波器系数到共享系数矩阵
        start_ticks = xTaskGetTickCount();
        memcpy(fir_shared_coeffs, coeffs, FILTER_TAP_NUM * sizeof(float32_t));
        end_ticks = xTaskGetTickCount();
        ticks_used = end_ticks - start_ticks;
    }
} 

/**
 * @brief 获取指定类型的滤波器系数
 * @param type 滤波器类型
 * @return 滤波器系数数组的指针
 */
const float32_t *fir_get_coeffs(FIR_Lowpass_100_FilterType type) {
   switch(type) {
        case FIR_LOW_100_512_010_CHEB:
            return fir_low_100_512_010_CHEB_coeffs;
        case FIR_LOW_100_512_070_CHEB:
            return fir_low_100_512_070_CHEB_coeffs;
        case FIR_LOW_100_512_120_CHEB:
            return fir_low_100_512_120_CHEB_coeffs;
        case FIR_LOW_100_512_170_CHEB:
            return fir_low_100_512_170_CHEB_coeffs;
        case FIR_LOW_100_512_220_CHEB:
            return fir_low_100_512_220_CHEB_coeffs;
        case FIR_LOW_100_512_270_CHEB:
            return fir_low_100_512_270_CHEB_coeffs;
        case FIR_LOW_100_512_320_CHEB:
            return fir_low_100_512_320_CHEB_coeffs;
        case FIR_LOW_100_512_370_CHEB:
            return fir_low_100_512_370_CHEB_coeffs;
        case FIR_LOW_100_512_420_CHEB:
            return fir_low_100_512_420_CHEB_coeffs;
        default:
            return fir_low_100_512_010_CHEB_coeffs;
    }

}

/**
 * @brief 根据旋转频率选择合适的高通滤波器类型
 * @param rotation_freq 旋转频率 (Hz)
 * @return 高通滤波器类型，如果不需要高通滤波则返回-1
 */
iir_highpass_100_filter_type_t select_highpass_filter_type(float32_t rotation_freq)
{
    // 高通滤波器的通过频率必须比rotation_freq小
    // 找到比rotation_freq小的所有高通滤波器中最大的那个
    // 如果两个频率的差距<0.2Hz，为了避免边缘衰减，应该再找下一档的高通滤波通过频率

    if(rotation_freq < 0.15f) {
        return IIR_HIGHPASS_100_000_HZ;  // 不需要高通滤波
    }
    else if(rotation_freq <= 0.3f) {
        return IIR_HIGHPASS_100_005_HZ;  // 0.05Hz
    }
    else if(rotation_freq <= 0.5f) {  //注意避开下一档的0.3Hz
        return IIR_HIGHPASS_100_020_HZ;  // 0.2Hz
    }
    else if(rotation_freq <= 0.6f) { //注意避开下一档的0.5Hz
        return IIR_HIGHPASS_100_030_HZ;  // 0.3Hz
    }
    else if(rotation_freq <= 0.9f) { //注意避开下一档的0.7Hz
        return IIR_HIGHPASS_100_050_HZ;  // 0.5Hz
    }
    else if(rotation_freq <= 1.5f) {//注意避开下一档的1.2Hz（需要跨过1.2Hz避免边缘衰减）
        return IIR_HIGHPASS_100_070_HZ;  // 0.7Hz
    }
    else if(rotation_freq <= 2.0f) {//注意避开下一档的1.7Hz（需要跨过1.7Hz避免边缘衰减）
        return IIR_HIGHPASS_100_120_HZ;  // 1.2Hz
    }
    else if(rotation_freq <= 2.5f) {//注意避开下一档的2.2Hz（需要跨过2.2Hz避免边缘衰减）
        return IIR_HIGHPASS_100_170_HZ;  // 1.7Hz
    }
    else if(rotation_freq <= 3.0f) {//注意避开下一档的2.2Hz（需要跨过2.2Hz避免边缘衰减）
        return IIR_HIGHPASS_100_220_HZ;  // 2.2Hz
    }
    else if(rotation_freq <= 3.5f) {//注意避开下一档的3.2Hz（需要跨过3.2Hz避免边缘衰减）
        return IIR_HIGHPASS_100_270_HZ;  // 2.7Hz
    }
    else if(rotation_freq <= 4.2f) {
        return IIR_HIGHPASS_100_320_HZ;  // 3.2Hz
    }
    else {
        return IIR_HIGHPASS_100_000_HZ;  // 超出范围，不需要高通滤波
    }
}

/**
 * @brief 根据旋转频率选择合适的FIR低通滤波器类型
 * @param rotation_freq 旋转频率 (Hz)
 * @return FIR低通滤波器类型
 */
FIR_Lowpass_100_FilterType select_fir_lowpass_filter_type(float32_t rotation_freq)
{
    // FIR低通滤波器截止频率表（Hz）
//    static const float32_t fir_lowpass_100hz_cutoff_freqs[FIR_LOW_100_512_TYPE_COUNT] = {
//        0.1f, 0.7f, 1.2f, 1.7f, 2.2f, 2.7f, 3.2f, 3.7f, 4.2f
//    };

    // 低通滤波器阈值和距离定义

    // 根据旋转频率选择FIR低通滤波器
    if (rotation_freq <= 0.11f) {
        return FIR_LOW_100_512_010_CHEB;  // 0.1Hz截止频率，适合极低转速
    }
    else if (rotation_freq <= 0.4f) {
        return FIR_LOW_100_512_070_CHEB;  // 0.7Hz截止频率，适合低转速
    }
    else if (rotation_freq <= 0.9f) {
        return FIR_LOW_100_512_120_CHEB;  // 1.2Hz截止频率，适合中低转速
    }
    else if (rotation_freq <= 1.4f) {
        return FIR_LOW_100_512_170_CHEB;  // 1.7Hz截止频率，适合中转速
    }
    else if (rotation_freq <= 1.9f) {
        return FIR_LOW_100_512_220_CHEB;  // 2.2Hz截止频率，适合中高转速
    }
    else if (rotation_freq <= 2.4f) {
        return FIR_LOW_100_512_270_CHEB;  // 2.7Hz截止频率，适合高转速
    }
    else if (rotation_freq <= 2.9f) {
        return FIR_LOW_100_512_320_CHEB;  // 3.2Hz截止频率，适合很高转速
    }
    else if (rotation_freq <= 3.4f) {
        return FIR_LOW_100_512_370_CHEB;  // 3.7Hz截止频率，适合极高转速
    }
    else {
        return FIR_LOW_100_512_420_CHEB;  // 4.2Hz截止频率，适合最高转速
    }
}

/**
 * @brief 获取FIR低通滤波器的截止频率
 * @param type FIR低通滤波器类型
 * @return 截止频率（Hz）
 */
float32_t fir_get_cutoff_freq(FIR_Lowpass_100_FilterType type)
{
    // FIR低通滤波器截止频率表（Hz）
    static const float32_t fir_lowpass_100hz_cutoff_freqs[FIR_LOW_100_512_TYPE_COUNT] = {
        0.1f, 0.7f, 1.2f, 1.7f, 2.2f, 2.7f, 3.2f, 3.7f, 4.2f
    };

    if (type >= FIR_LOW_100_512_TYPE_COUNT) {
        return 0.1f;  // 默认返回0.1Hz
    }

    return fir_lowpass_100hz_cutoff_freqs[type];
}

/**
 * @brief 获取当前滤波器的升级阈值（切换到更高档位的rotation_freq阈值）
 * @param current_type 当前滤波器类型
 * @return 升级阈值（Hz），如果已是最高档位则返回无穷大
 */
float32_t fir_get_upgrade_threshold(FIR_Lowpass_100_FilterType current_type)
{
    // 升级阈值表：对应select_fir_lowpass_filter_type函数中的判断条件
    // 当rotation_freq超过这些阈值时，会切换到下一档更高的滤波器
    static const float32_t upgrade_thresholds[FIR_LOW_100_512_TYPE_COUNT] = {
        0.11f,  // FIR_LOW_100_512_010_CHEB -> FIR_LOW_100_512_070_CHEB
        0.4f,   // FIR_LOW_100_512_070_CHEB -> FIR_LOW_100_512_120_CHEB
        0.9f,   // FIR_LOW_100_512_120_CHEB -> FIR_LOW_100_512_170_CHEB
        1.4f,   // FIR_LOW_100_512_170_CHEB -> FIR_LOW_100_512_220_CHEB
        1.9f,   // FIR_LOW_100_512_220_CHEB -> FIR_LOW_100_512_270_CHEB
        2.4f,   // FIR_LOW_100_512_270_CHEB -> FIR_LOW_100_512_320_CHEB
        2.9f,   // FIR_LOW_100_512_320_CHEB -> FIR_LOW_100_512_370_CHEB
        3.4f,   // FIR_LOW_100_512_370_CHEB -> FIR_LOW_100_512_420_CHEB
        1000.0f // FIR_LOW_100_512_420_CHEB 已是最高档位，设为无穷大
    };

    if (current_type >= FIR_LOW_100_512_TYPE_COUNT) {
        return 1000.0f;  // 无效类型，返回无穷大
    }

    return upgrade_thresholds[current_type];
}

/**
 * @brief 获取当前滤波器的降级阈值（切换到更低档位的rotation_freq阈值）
 * @param current_type 当前滤波器类型
 * @return 降级阈值（Hz），如果已是最低档位则返回0
 */
float32_t fir_get_downgrade_threshold(FIR_Lowpass_100_FilterType current_type)
{
    // 降级阈值表：当rotation_freq低于这些阈值时，会切换到上一档更低的滤波器
    // 基于select_fir_lowpass_filter_type函数的逆向逻辑
    static const float32_t downgrade_thresholds[FIR_LOW_100_512_TYPE_COUNT] = {
        0.0f,   // FIR_LOW_100_512_010_CHEB 已是最低档位
        0.11f,  // FIR_LOW_100_512_070_CHEB -> FIR_LOW_100_512_010_CHEB
        0.4f,   // FIR_LOW_100_512_120_CHEB -> FIR_LOW_100_512_070_CHEB
        0.9f,   // FIR_LOW_100_512_170_CHEB -> FIR_LOW_100_512_120_CHEB
        1.4f,   // FIR_LOW_100_512_220_CHEB -> FIR_LOW_100_512_170_CHEB
        1.9f,   // FIR_LOW_100_512_270_CHEB -> FIR_LOW_100_512_220_CHEB
        2.4f,   // FIR_LOW_100_512_320_CHEB -> FIR_LOW_100_512_270_CHEB
        2.9f,   // FIR_LOW_100_512_370_CHEB -> FIR_LOW_100_512_320_CHEB
        3.4f    // FIR_LOW_100_512_420_CHEB -> FIR_LOW_100_512_370_CHEB
    };

    if (current_type >= FIR_LOW_100_512_TYPE_COUNT) {
        return 0.0f;  // 无效类型，返回0
    }

    return downgrade_thresholds[current_type];
}

