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
#include "FIR_LOW_100_512_015_CHEB.h"  //100Hz低通滤波器（0.15Hz截止）
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
    fir_low_100_512_015_CHEB_coeffs,
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
    fir_switch_type(FIR_LOW_100_512_015_CHEB);
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
        case FIR_LOW_100_512_015_CHEB:
            return fir_low_100_512_015_CHEB_coeffs;
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
            return fir_low_100_512_015_CHEB_coeffs;
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

    if(rotation_freq < 0.12f) {
        return IIR_HIGHPASS_100_000_HZ;  // 不需要高通滤波
    }
    else if(rotation_freq <= 0.3f) {
        return IIR_HIGHPASS_100_005_HZ;  // 0.05Hz
    }
    else if(rotation_freq <= 0.5f) {  //注意避开下一档的0.3Hz
        return IIR_HIGHPASS_100_020_HZ;  // 0.2Hz
    }
    else if(rotation_freq <= 0.7f) { //注意避开下一档的0.5Hz
        return IIR_HIGHPASS_100_030_HZ;  // 0.3Hz
    }
    else if(rotation_freq <= 1.2f) { //注意避开下一档的0.7Hz， 高通滤波也要注意避开0.7Hz的高通过渡带衰减
        return IIR_HIGHPASS_100_050_HZ;  // 0.5Hz
    }
    else if(rotation_freq <= 1.7f) {//注意避开下一档的1.2Hz（需要跨过1.2Hz避免边缘衰减）, 1.6Hz时下一档1.2Hz高通仍然有衰减，需要提高到1.6（20680传感器）
        return IIR_HIGHPASS_100_070_HZ;  // 0.7Hz

    }
    else if(rotation_freq <= 2.2f) {//注意避开下一档的1.7Hz（需要跨过1.7Hz避免边缘衰减），2.1Hz时下一档1.7Hz高通仍然有衰减，需要提高到2.2（20680传感器）
        return IIR_HIGHPASS_100_120_HZ;  // 1.2Hz
    }
    else if(rotation_freq <= 2.7f) {//注意避开下一档的2.2Hz（需要跨过2.2Hz避免边缘衰减），2.5Hz时下一档2.2Hz高通仍然有衰减，需要提高到2.6（20680传感器）
        return IIR_HIGHPASS_100_170_HZ;  // 1.7Hz
    }
    else if(rotation_freq <= 3.2f) {//注意避开下一档的2.7Hz（需要跨过2.7Hz避免边缘衰减），3.0Hz时下一档2.7Hz高通仍然有衰减，需要提高到3.1（20680传感器）
        return IIR_HIGHPASS_100_220_HZ;  // 2.2Hz
    }
    else if(rotation_freq <= 3.7f) {//注意避开下一档的3.2Hz（需要跨过3.2Hz避免边缘衰减），3.5Hz时下一档3.2Hz高通仍然有衰减，需要提高到3.2（20680传感器）
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
    if (rotation_freq <= 0.06f) {
        return FIR_LOW_100_512_015_CHEB;  // 0.1Hz截止频率，适合极低转速
    }
    else if (rotation_freq <= 0.35f) {
        return FIR_LOW_100_512_070_CHEB;  // 0.7Hz截止频率，适合低转速
    }
    else if (rotation_freq <= 0.7f) {
        return FIR_LOW_100_512_120_CHEB;  // 1.2Hz截止频率，适合中低转速
    }
    else if (rotation_freq <= 1.2f) {
        return FIR_LOW_100_512_170_CHEB;  // 1.7Hz截止频率，适合中转速
    }
    else if (rotation_freq <= 1.6f) {
        return FIR_LOW_100_512_220_CHEB;  // 2.2Hz截止频率，适合中高转速
    }
    else if (rotation_freq <= 2.1f) {
        return FIR_LOW_100_512_270_CHEB;  // 2.7Hz截止频率，适合高转速
    }
    else if (rotation_freq <= 2.6f) {
        return FIR_LOW_100_512_320_CHEB;  // 3.2Hz截止频率，适合很高转速
    }
    else if (rotation_freq <= 3.1f) {
        return FIR_LOW_100_512_370_CHEB;  // 3.7Hz截止频率，适合极高转速
    }
    else {
        return FIR_LOW_100_512_420_CHEB;  // 4.2Hz截止频率，适合最高转速
    }
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
        0.07f,  // FIR_LOW_100_512_015_CHEB -> FIR_LOW_100_512_070_CHEB
        0.4f,   // FIR_LOW_100_512_070_CHEB -> FIR_LOW_100_512_120_CHEB
        0.8f,   // FIR_LOW_100_512_120_CHEB -> FIR_LOW_100_512_170_CHEB
        1.3f,   // FIR_LOW_100_512_170_CHEB -> FIR_LOW_100_512_220_CHEB
        1.7f,   // FIR_LOW_100_512_220_CHEB -> FIR_LOW_100_512_270_CHEB
        2.2f,   // FIR_LOW_100_512_270_CHEB -> FIR_LOW_100_512_320_CHEB
        2.7f,   // FIR_LOW_100_512_320_CHEB -> FIR_LOW_100_512_370_CHEB
        3.2f,   // FIR_LOW_100_512_370_CHEB -> FIR_LOW_100_512_420_CHEB
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
        0.0f,   // FIR_LOW_100_512_015_CHEB 已是最低档位
        0.04f,  // FIR_LOW_100_512_070_CHEB -> FIR_LOW_100_512_015_CHEB
        0.3f,   // FIR_LOW_100_512_120_CHEB -> FIR_LOW_100_512_070_CHEB
        0.6f,   // FIR_LOW_100_512_170_CHEB -> FIR_LOW_100_512_120_CHEB
        1.1f,   // FIR_LOW_100_512_220_CHEB -> FIR_LOW_100_512_170_CHEB
        1.5f,   // FIR_LOW_100_512_270_CHEB -> FIR_LOW_100_512_220_CHEB
        2.0f,   // FIR_LOW_100_512_320_CHEB -> FIR_LOW_100_512_270_CHEB
        2.5f,   // FIR_LOW_100_512_370_CHEB -> FIR_LOW_100_512_320_CHEB
        3.0f    // FIR_LOW_100_512_420_CHEB -> FIR_LOW_100_512_370_CHEB
    };

    if (current_type >= FIR_LOW_100_512_TYPE_COUNT) {
        return 0.0f;  // 无效类型，返回0
    }

    return downgrade_thresholds[current_type];
}

/**
 * @brief 获取高通滤波器的升级阈值（切换到更高档位的rotation_freq阈值）
 * @param current_type 当前高通滤波器类型
 * @return 升级阈值（Hz），如果已是最高档位则返回无穷大
 */
float32_t hpf_get_upgrade_threshold(iir_highpass_100_filter_type_t current_type)
{
    // 高通滤波器升级阈值表：基于select_highpass_filter_type函数的切换点
    // 升级阈值应该高于原始切换点，避免过早升级
    static const float32_t upgrade_thresholds[IIR_HIGHPASS_100_TYPE_COUNT] = {
        0.35f,  // IIR_HIGHPASS_100_005_HZ -> IIR_HIGHPASS_100_020_HZ (0.3f -> 0.35f)
        0.6f,  // IIR_HIGHPASS_100_020_HZ -> IIR_HIGHPASS_100_030_HZ (0.5f -> 0.6f)
        0.8f,  // IIR_HIGHPASS_100_030_HZ -> IIR_HIGHPASS_100_050_HZ (0.7f -> 0.8f)
        1.3f,  // IIR_HIGHPASS_100_050_HZ -> IIR_HIGHPASS_100_070_HZ (1.2f -> 1.3f)
        1.8f,   // IIR_HIGHPASS_100_070_HZ -> IIR_HIGHPASS_100_120_HZ (1.7f -> 1.8f) 关键死区
        2.3f,   // IIR_HIGHPASS_100_120_HZ -> IIR_HIGHPASS_100_170_HZ (2.2f -> 2.3f)
        2.8f,   // IIR_HIGHPASS_100_170_HZ -> IIR_HIGHPASS_100_220_HZ (2.7f -> 2.8f)
        3.3f,   // IIR_HIGHPASS_100_220_HZ -> IIR_HIGHPASS_100_270_HZ (3.2f -> 3.3f)
        3.8f,   // IIR_HIGHPASS_100_270_HZ -> IIR_HIGHPASS_100_320_HZ (3.7f -> 3.8f)
        1000.0f // IIR_HIGHPASS_100_320_HZ 已是最高档位，设为无穷大
    };

    if (current_type >= IIR_HIGHPASS_100_TYPE_COUNT) {
        return 1000.0f;  // 无效类型，返回无穷大
    }

    if(current_type == IIR_HIGHPASS_100_000_HZ) {
        return 0.15f;   //当前类型为无需高通滤波时，此时下一档切换频率为0.15Hz
    }

    return upgrade_thresholds[current_type];
}

/**
 * @brief 获取高通滤波器的降级阈值（切换到更低档位的rotation_freq阈值）
 * @param current_type 当前高通滤波器类型
 * @return 降级阈值（Hz），如果已是最低档位则返回0
 */
float32_t hpf_get_downgrade_threshold(iir_highpass_100_filter_type_t current_type)
{
    // 高通滤波器降级阈值表：降级阈值应该低于原始切换点，避免过早降级
    static const float32_t downgrade_thresholds[IIR_HIGHPASS_100_TYPE_COUNT] = {
        0.05f,   // IIR_HIGHPASS_100_005_HZ 已是最低档位
        0.25f,  // IIR_HIGHPASS_100_020_HZ -> IIR_HIGHPASS_100_005_HZ (0.3f -> 0.25f)
        0.45f,  // IIR_HIGHPASS_100_030_HZ -> IIR_HIGHPASS_100_020_HZ (0.5f -> 0.45f)
        0.65f,  // IIR_HIGHPASS_100_050_HZ -> IIR_HIGHPASS_100_030_HZ (0.7f -> 0.65f)
        1.15f,  // IIR_HIGHPASS_100_070_HZ -> IIR_HIGHPASS_100_050_HZ (1.2f -> 1.15f)
        1.6f,   // IIR_HIGHPASS_100_120_HZ -> IIR_HIGHPASS_100_070_HZ (1.7f -> 1.6f) 关键死区
        2.15f,  // IIR_HIGHPASS_100_170_HZ -> IIR_HIGHPASS_100_120_HZ (2.2f -> 2.15f)
        2.65f,  // IIR_HIGHPASS_100_220_HZ -> IIR_HIGHPASS_100_170_HZ (2.7f -> 2.65f)
        3.15f,  // IIR_HIGHPASS_100_270_HZ -> IIR_HIGHPASS_100_220_HZ (3.2f -> 3.15f)
        3.65f   // IIR_HIGHPASS_100_320_HZ -> IIR_HIGHPASS_100_270_HZ (3.7f -> 3.65f)
    };

    if (current_type >= IIR_HIGHPASS_100_TYPE_COUNT) {
        return 0.0f;  // 无效类型，返回0
    }
    if(current_type == IIR_HIGHPASS_100_005_HZ) {
        return 0.1f;   //当前类型为0.05Hz高通滤波时，此时下一档切换频率为0.1f,低于0.1f时会降级到无需高通滤波
    }

    return downgrade_thresholds[current_type];
}

