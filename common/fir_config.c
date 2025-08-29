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

// 本地头文件（滤波器系数）
#include "FIR_LOW_50_512_200_CHEB.h"  //用于陀螺仪Z轴滤波
#include "FIR_LOW_50_512_040_CHEB.h"  //用于x/y轴加速度计滤波
#include "FIR_BAND_50_512_025_120_CHEB.h"  //用于x/y轴加速度计滤波
#include "FIR_BAND_50_512_070_170_CHEB.h"  //用于x/y轴加速度计滤波
#include "FIR_BAND_50_512_120_220_CHEB.h"  //用于x/y轴加速度计滤波
#include "FIR_BAND_50_512_170_270_CHEB.h"  //用于x/y轴加速度计滤波
#include "FIR_BAND_50_512_220_320_CHEB.h"  //用于x/y轴加速度计滤波
#include "FIR_BAND_50_512_270_370_CHEB.h"  //用于x/y轴加速度计滤波
#include "FIR_BAND_50_512_320_420_CHEB.h"  //用于x/y轴加速度计滤波

// 所有滤波器的系数定义（存储在Flash中）
const float32_t * const fir_coeffs_flash[NUM_FILTERS] = {
    // FIR_LOW_50_512_200_CHEB (20Hz低通)
    fir_low_50_512_200_CHEB_coeffs,
    
    // FIR_LOW_50_512_040_CHEB (0.4Hz低通)
    fir_low_50_512_040_CHEB_coeffs,
    
    // FIR_BAND_50_512_025_120_CHEB (0.25-1.2Hz带通)
    fir_band_50_512_025_120_CHEB_coeffs,

    // FIR_BAND_50_512_070_170_CHEB (0.7-1.7Hz带通)
    fir_band_50_512_070_170_CHEB_coeffs,
    
    // FIR_BAND_50_512_120_220_CHEB (1.2-2.2Hz带通)
    fir_band_50_512_120_220_CHEB_coeffs,

    // FIR_BAND_50_512_170_270_CHEB (1.7-2.7Hz带通)
    fir_band_50_512_170_270_CHEB_coeffs,
    
    // FIR_BAND_50_512_220_320_CHEB (2.2-3.2Hz带通)
    fir_band_50_512_220_320_CHEB_coeffs,
    
    // FIR_BAND_50_512_270_370_CHEB (2.7-3.7Hz带通)
    fir_band_50_512_270_370_CHEB_coeffs,
    
    // FIR_BAND_50_512_320_420_CHEB (3.2-4.2Hz带通)
    fir_band_50_512_320_420_CHEB_coeffs
};

//---------------- 独立资源定义（加速度Z轴和陀螺仪z轴）----------------
// 注释掉Z轴加速度计FIR滤波器实例，改用4阶IIR滤波器（0.12Hz)
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
                     
    // 初始化陀螺仪Z轴滤波器
    /*
    arm_fir_init_f32(&fir_gyro_z_instance, 
                     FILTER_TAP_NUM,
                     (float32_t *)fir_shared_coeffs,
                     fir_gyro_z_state, 
                     1);

     */
    // 设置默认滤波器类型
    fir_switch_type(FIR_LOW_50_512_040_CHEB);
}

/**
 * @brief 获取指定类型的滤波器系数
 * @param type 滤波器类型
 * @return 滤波器系数数组的指针
 */
const float32_t *fir_get_coeffs(FIR_FilterType type) {
    switch(type) {
        case FIR_LOW_50_512_200_CHEB:
            return fir_low_50_512_200_CHEB_coeffs;;
        case FIR_LOW_50_512_040_CHEB:
            return fir_low_50_512_040_CHEB_coeffs;
        case FIR_BAND_50_512_025_120_CHEB:
            return fir_band_50_512_025_120_CHEB_coeffs;
        case FIR_BAND_50_512_070_170_CHEB:
            return fir_band_50_512_070_170_CHEB_coeffs;
        case FIR_BAND_50_512_120_220_CHEB:
            return fir_band_50_512_120_220_CHEB_coeffs;
        case FIR_BAND_50_512_170_270_CHEB:
            return fir_band_50_512_170_270_CHEB_coeffs;
        case FIR_BAND_50_512_220_320_CHEB:
            return fir_band_50_512_220_320_CHEB_coeffs;
        case FIR_BAND_50_512_270_370_CHEB:
            return fir_band_50_512_270_370_CHEB_coeffs;
        case FIR_BAND_50_512_320_420_CHEB:
            return fir_band_50_512_320_420_CHEB_coeffs;
        default:
            return fir_low_50_512_040_CHEB_coeffs;
    }
}

/**
 * @brief 切换滤波器类型
 * @param new_type 新的滤波器类型
 */
void fir_switch_type(FIR_FilterType new_type) {
    // 获取新的滤波器系数
    const float32_t *coeffs = fir_get_coeffs(new_type);
    if (coeffs != NULL) {
        // 复制新的滤波器系数到共享系数矩阵
        memcpy(fir_shared_coeffs, coeffs, FILTER_TAP_NUM * sizeof(float32_t));
    }
} 



