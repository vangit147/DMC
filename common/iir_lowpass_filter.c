/**
  *****************************************************************************
  * FILENAME   : iir_lowpass_filter.c
  * COPYRIGHT  : ModingTech(ShangHai) Co.,Ltd2025.
  * CREATEDDATE: 2025.01.24 13:21:34 Friday
   * DESCRIPTION: IIR低通滤波器实现，实现4阶IIR低通滤波器，等效于硬件RC同向4阶低通滤波
 *              截止频率为0.12Hz，采样频率50Hz，用于Z轴加速度计进行振动滤波

 *              适用于需要稳定低通滤波的场景，如测斜应用
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  *   Gordon Li                  2025.01.24                    CREATE
  *   Gordon Li                  2025.08.27                    UPDATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "iir_lowpass_filter.h"
#include <string.h>

// 系统参数定义
#define SAMPLING_FREQ 50.0f        // 采样频率 50Hz (20ms周期)
#define CUTOFF_FREQ_4TH 0.12f     // 4阶滤波器截止频率 0.12Hz
#define SLOW_ROTATION_FREQ_4th 


// 4阶低通滤波 - 双二阶节格式（MATLAB生成）
// 4阶Butterworth低通滤波器，截止频率0.12Hz，采样频率50Hz
// 采用双二阶节级联结构，双线性变换，确保数值稳定性
static const float32_t IIR_4TH_GAIN1 = 3.928896331e-05f;        // 第1个增益节
static const float32_t IIR_4TH_GAIN2 = 3.902483877e-05f;        // 第2个增益节

// 第1个2阶节系数
static const float32_t IIR_4TH_B1_COEFFS[3] = {
    1.0f,              // b0 = 1.0
    2.0f,              // b1 = 2.0
    1.0f               // b2 = 1.0
};

static const float32_t IIR_4TH_A1_COEFFS[3] = {
    1.0f,              // a0 = 1.0
    -1.990271211f,     // a1 = -1.990271211
    0.9904283881f      // a2 = 0.9904283881
};

// 第2个2阶节系数
static const float32_t IIR_4TH_B2_COEFFS[3] = {
    1.0f,              // b0 = 1.0
    2.0f,              // b1 = 2.0
    1.0f               // b2 = 1.0
};

static const float32_t IIR_4TH_A2_COEFFS[3] = {
    1.0f,              // a0 = 1.0
    -1.976891398f,     // a1 = -1.976891398
    0.9770474434f      // a2 = 0.9770474434
};


/******************************* 函数实现 ********************************/

/**
 * @brief 初始化4阶IIR低通滤波器（双二阶节格式）
 * @param filter 4阶滤波器结构体指针
 */
void iir_lowpass_filter_4th_init(iir_lowpass_filter_4th_t *filter)
{
    if (filter == NULL) {
        return;
    }
    
    // 直接指向静态系数，不复制
    filter->b1 = IIR_4TH_B1_COEFFS;
    filter->a1 = IIR_4TH_A1_COEFFS;
    filter->b2 = IIR_4TH_B2_COEFFS;
    filter->a2 = IIR_4TH_A2_COEFFS;
    
    // 清零所有延迟线
    memset(filter->x1_delay, 0, sizeof(filter->x1_delay));
    memset(filter->y1_delay, 0, sizeof(filter->y1_delay));
    memset(filter->x2_delay, 0, sizeof(filter->x2_delay));
    memset(filter->y2_delay, 0, sizeof(filter->y2_delay));
    
    filter->is_initialized = true;
}


/**
 * @brief 处理一个输入样本（4阶IIR滤波器 - 双二阶节格式）
 * @param filter 4阶滤波器结构体指针
 * @param input 输入样本
 * @return 滤波后的输出样本
 */
float32_t iir_lowpass_filter_4th_process(iir_lowpass_filter_4th_t *filter, float32_t input)  // 修复参数类型
{
    if (filter == NULL || !filter->is_initialized) {
        return input;
    }
    
    // 第1个增益节
    float32_t gain1_output = input * IIR_4TH_GAIN1;
    
    // 第1个2阶节处理（使用指针访问系数）
    float32_t output1 = filter->b1[0] * gain1_output;
    
    // 添加第1个2阶节的输入延迟项
    for (int i = 0; i < 2; i++) {
        output1 += filter->b1[i + 1] * filter->x1_delay[i];
    }
    
    // 减去第1个2阶节的输出延迟项
    for (int i = 0; i < 2; i++) {
        output1 -= filter->a1[i + 1] * filter->y1_delay[i];
    }
    
    // 更新第1个2阶节的延迟线
    for (int i = 1; i > 0; i--) {
        filter->x1_delay[i] = filter->x1_delay[i - 1];
    }
    filter->x1_delay[0] = gain1_output;
    
    for (int i = 1; i > 0; i--) {
        filter->y1_delay[i] = filter->y1_delay[i - 1];
    }
    filter->y1_delay[0] = output1;
    
    // 第2个增益节
    float32_t gain2_output = output1 * IIR_4TH_GAIN2;
    
    // 第2个2阶节处理
    float32_t output2 = filter->b2[0] * gain2_output;           // b0*x[n]
    
    // 添加第2个2阶节的输入延迟项
    for (int i = 0; i < 2; i++) {
        output2 += filter->b2[i + 1] * filter->x2_delay[i];     // b1*x[n-1] + b2*x[n-2]
    }
    
    // 减去第2个2阶节的输出延迟项
    for (int i = 0; i < 2; i++) {
        output2 -= filter->a2[i + 1] * filter->y2_delay[i];     // -a1*y[n-1] - a2*y[n-2]
    }
    
    // 更新第2个2阶节的延迟线
    for (int i = 1; i > 0; i--) {
        filter->x2_delay[i] = filter->x2_delay[i - 1];
    }
    filter->x2_delay[0] = gain2_output;
    
    for (int i = 1; i > 0; i--) {
        filter->y2_delay[i] = filter->y2_delay[i - 1];
    }
    filter->y2_delay[0] = output2;
    
    // 返回最终输出
    return output2;
}


/**
 * @brief 重置4阶IIR滤波器状态
 * @param filter 4阶滤波器结构体指针
 */
void iir_lowpass_filter_4th_reset(iir_lowpass_filter_4th_t *filter)
{
    if (filter == NULL) {
        return;
    }
    
    // 清零所有延迟线
    memset(filter->x1_delay, 0, sizeof(filter->x1_delay));
    memset(filter->y1_delay, 0, sizeof(filter->y1_delay));
    memset(filter->x2_delay, 0, sizeof(filter->x2_delay));
    memset(filter->y2_delay, 0, sizeof(filter->y2_delay));
}