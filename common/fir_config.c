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
#include "FIR_LOW_20_000_980.h"
#include "FIR_LOW_20_030_045.h"
#include "FIR_LOW_20_050_BMW.h"
#include "FIR_LOW_20_010_KBW.h"
#include "FIR_BAND_20_012_085.h"
#include "FIR_BAND_20_025_110.h"
#include "FIR_BAND_20_055_145.h"
#include "FIR_BAND_20_085_175.h"
#include "FIR_BAND_20_115_205.h"
#include "FIR_BAND_20_145_235.h"
#include "FIR_BAND_20_175_265.h"
#include "FIR_BAND_20_205_295.h"
#include "FIR_BAND_20_235_325.h"
#include "FIR_BAND_20_265_355.h"
#include "FIR_BAND_20_295_385.h"
#include "FIR_BAND_20_325_415.h"
#include "FIR_BAND_20_355_445.h"

// 所有滤波器的系数定义（存储在Flash中）
const float32_t * const fir_coeffs_flash[NUM_FILTERS] = {
    // FIR_LOW_20_000_980 (9.8Hz低通)
    fir_low_20_000_980_coeffs,
    
    // FIR_LOW_20_030_045 (0.3-0.45Hz低通)
    fir_low_20_030_045_coeffs,
    
    // FIR_LOW_20_050_BMW (0.5Hz低通，布莱克曼窗)
    fir_low_20_050_BMW_coeffs,
    
    // FIR_LOW_20_010_KBW (0.1Hz低通，凯泽窗)
    fir_low_20_010_KBW_coeffs,
    
    // FIR_BAND_20_012_085 (0.12-0.85Hz带通)
    fir_band_20_012_085_coeffs,
    
    // FIR_BAND_20_025_110 (0.25-1.10Hz带通)
    fir_band_20_025_110_coeffs,
    
    // FIR_BAND_20_055_145 (0.55-1.45Hz带通)
    fir_band_20_055_145_coeffs,
    
    // FIR_BAND_20_085_175 (0.85-1.75Hz带通)
    fir_band_20_085_175_coeffs,
    
    // FIR_BAND_20_115_205 (1.15-2.05Hz带通)
    fir_band_20_115_205_coeffs,
    
    // FIR_BAND_20_145_235 (1.45-2.35Hz带通)
    fir_band_20_145_235_coeffs,
    
    // FIR_BAND_20_175_265 (1.75-2.65Hz带通)
    fir_band_20_175_265_coeffs,
    
    // FIR_BAND_20_205_295 (2.05-2.95Hz带通)
    fir_band_20_205_295_coeffs,
    
    // FIR_BAND_20_235_325 (2.35-3.25Hz带通)
    fir_band_20_235_325_coeffs,
    
    // FIR_BAND_20_265_355 (2.65-3.55Hz带通)
    fir_band_20_265_355_coeffs,
    
    // FIR_BAND_20_295_385 (2.95-3.85Hz带通)
    fir_band_20_295_385_coeffs,
    
    // FIR_BAND_20_325_415 (3.25-4.15Hz带通)
    fir_band_20_325_415_coeffs,
    
    // FIR_BAND_20_355_445 (3.55-4.45Hz带通)
    fir_band_20_355_445_coeffs
};

//---------------- 独立资源定义（加速度Z轴和陀螺仪z轴）----------------
arm_fir_instance_f32 fir_acc_z_instance;  // 加速度计Z轴实例
arm_fir_instance_f32 fir_acc_x_instance;  // 加速度计X轴实例
arm_fir_instance_f32 fir_acc_y_instance;  // 加速度计Y轴实例
arm_fir_instance_f32 fir_gyro_z_instance; // 陀螺仪Z轴实例
//arm_fir_instance_f32 fir_gyro_z_instance_for_compensation; // 陀螺仪Z轴标定用低通滤波实例
//arm_fir_instance_f32 fir_gyro_x_instance; // 陀螺仪X轴实例
//arm_fir_instance_f32 fir_gyro_y_instance; // 陀螺仪Y轴实例

// 公共系数矩阵
float32_t fir_shared_coeffs[FILTER_TAP_NUM];  // 所有滤波器共享的系数矩阵

// 状态缓冲区
float32_t fir_acc_z_state[FILTER_TAP_NUM + 3];//加速度计Z状态缓冲
float32_t fir_acc_x_state[FILTER_TAP_NUM + 3];//加速度计X状态缓冲
float32_t fir_acc_y_state[FILTER_TAP_NUM + 3];//加速度计Y状态缓冲
float32_t fir_gyro_z_state[FILTER_TAP_NUM + 3];//陀螺仪Z状态缓冲
//float32_t fir_gyro_z_state_for_compensation[FILTER_TAP_NUM + 3];//陀螺仪Z标定用低通滤波状态缓冲
//float32_t fir_gyro_x_state[FILTER_TAP_NUM + 3];//陀螺仪X状态缓冲
//float32_t fir_gyro_y_state[FILTER_TAP_NUM + 3];//陀螺仪Y状态缓冲


/******************************* 函数实现 ********************************/
/**
 * @brief 初始化所有FIR滤波器实例
 */
void fir_init_all(void) {
    // 初始化加速度计Z轴滤波器
    arm_fir_init_f32(&fir_acc_z_instance, 
                     FILTER_TAP_NUM,
                     (float32_t *)fir_shared_coeffs,
                     fir_acc_z_state, 
                     1);

                     
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
    arm_fir_init_f32(&fir_gyro_z_instance, 
                     FILTER_TAP_NUM,
                     (float32_t *)fir_shared_coeffs,
                     fir_gyro_z_state, 
                     1);

 /*   // 初始化陀螺仪Z轴标定用低通滤波器
    arm_fir_init_f32(&fir_gyro_z_instance_for_compensation, 
                     FILTER_TAP_NUM,
                     (float32_t *)fir_shared_coeffs,
                     fir_gyro_z_state_for_compensation, 
                     1);
 */                    
    // 设置默认滤波器类型
    fir_switch_type(FIR_LOW_20_050_BMW);
}

/**
 * @brief 获取指定类型的滤波器系数
 * @param type 滤波器类型
 * @return 滤波器系数数组的指针
 */
const float32_t *fir_get_coeffs(FIR_FilterType type) {
    switch(type) {
        case FIR_LOW_20_000_980:
            return fir_low_20_000_980_coeffs;
        case FIR_LOW_20_030_045:
            return fir_low_20_030_045_coeffs;
        case FIR_LOW_20_050_BMW:
            return fir_low_20_050_BMW_coeffs;
        case FIR_LOW_20_010_KBW:
            return fir_low_20_010_KBW_coeffs;
        case FIR_BAND_20_012_085:
            return fir_band_20_012_085_coeffs;
        case FIR_BAND_20_025_110:
            return fir_band_20_025_110_coeffs;
        case FIR_BAND_20_055_145:
            return fir_band_20_055_145_coeffs;
        case FIR_BAND_20_085_175:
            return fir_band_20_085_175_coeffs;
        case FIR_BAND_20_115_205:
            return fir_band_20_115_205_coeffs;
        case FIR_BAND_20_145_235:
            return fir_band_20_145_235_coeffs;
        case FIR_BAND_20_175_265:
            return fir_band_20_175_265_coeffs;
        case FIR_BAND_20_205_295:
            return fir_band_20_205_295_coeffs;
        case FIR_BAND_20_235_325:
            return fir_band_20_235_325_coeffs;
        case FIR_BAND_20_265_355:
            return fir_band_20_265_355_coeffs;
        case FIR_BAND_20_295_385:
            return fir_band_20_295_385_coeffs;
        case FIR_BAND_20_325_415:
            return fir_band_20_325_415_coeffs;
        case FIR_BAND_20_355_445:
            return fir_band_20_355_445_coeffs;
        default:
            return NULL;
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



