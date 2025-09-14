/**
  *****************************************************************************
  * FILENAME   : iir_lowpass_filter.c
  * COPYRIGHT  : ModingTech(ShangHai) Co.,Ltd2025.
  * CREATEDDATE: 2025.01.24 13:21:34 Friday
   * DESCRIPTION: IIR低通滤波器头文件，采样率100Hz
  *             实现8阶IIR低通滤波器，截止频率为0.15Hz
  *             实现16阶IIR低通滤波器，截止频率为0.12Hz
  *             实现100Hz采样率多种截止频率的16阶IIR低通滤波器
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  *   Gordon Li                  2025.01.24                    CREATE
  *   Gordon Li                  2025.08.27                    UPDATE
  *   Gordon Li                  2025.09.10                    ADD 100Hz多种截止频率支持
*****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "iir_lowpass_filter.h"
#include <string.h>

// 系统参数定义
#define SAMPLING_FREQ 100.0f        // 采样频率 100Hz (10ms周期)
#define CUTOFF_FREQ_8TH 0.15f //    8阶滤波器截止频率 0.15Hz
#define CUTOFF_FREQ_16TH 0.12f // 16阶滤波器截止频率 0.12Hz
#define LOW_PASS_SECTIONS 8       // 低通滤波器节数

// 共享的分子系数（所有IIR低通滤波器都使用相同的分子系数）
static const float32_t iir_shared_b_coeffs[3] = {1.0f, 2.0f, 1.0f};

// 8阶低通滤波 - 4个二阶节格式（MATLAB生成）
// 8阶Chebyshev低通滤波器，截止频率0.15Hz，采样频率100Hz
// 采用4个二阶节级联结构，双线性变换，确保数值稳定性
// 从IIR_LOW_100_8_015_CHEB.h提取的系数

// 增益系数
static const float32_t iir_8th_100hz_gain[5] = {
    2.21656901e-05f,   // 第1个增益节
    2.209077684e-05f,  // 第2个增益节
    2.203378244e-05f,  // 第3个增益节
    2.200305971e-05f,  // 第4个增益节
    1.0f               // 第5个增益节（直通）
};

// 分子系数（所有节都使用[1, 2, 1]系数）
static const float32_t iir_8th_100hz_b_coeffs[4][3] = {
    {1.0f, 2.0f, 1.0f},  // 第1个2阶节
    {1.0f, 2.0f, 1.0f},  // 第2个2阶节
    {1.0f, 2.0f, 1.0f},  // 第3个2阶节
    {1.0f, 2.0f, 1.0f}   // 第4个2阶节
};

// 分母系数
static const float32_t iir_8th_100hz_a_coeffs[4][3] = {
    {1.0f, -1.996240735f, 0.9963294268f},  // 第1个2阶节
    {1.0f, -1.989494085f, 0.989582479f},   // 第2个2阶节
    {1.0f, -1.984361172f, 0.9844492674f},  // 第3个2阶节
    {1.0f, -1.981594205f, 0.981682241f}    // 第4个2阶节
};

// 16阶低通滤波 - 8个二阶节格式（MATLAB生成）
// 16阶Chebyshev低通滤波器，截止频率0.12Hz，采样频率100Hz
// 采用8个二阶节级联结构，双线性变换，确保数值稳定性
// 从IIR_LOW_100_16_012_CHEB.h提取的系数

// 增益系数
static const float32_t iir_16th_100hz_gain[9] = {
    1.420166791e-05f,  // 第1个增益节
    1.418112515e-05f,  // 第2个增益节
    1.41618284e-05f,   // 第3个增益节
    1.414450708e-05f,  // 第4个增益节
    1.412981055e-05f,  // 第5个增益节
    1.411828362e-05f,  // 第6个增益节
    1.411035555e-05f,  // 第7个增益节
    1.410631739e-05f,  // 第8个增益节
    1.0f                // 第9个增益节（直通）
};

// 分子系数（所有节都使用[1, 2, 1]系数）
static const float32_t iir_16th_100hz_b_coeffs[8][3] = {
    {1.0f, 2.0f, 1.0f},  // 第1个2阶节
    {1.0f, 2.0f, 1.0f},  // 第2个2阶节
    {1.0f, 2.0f, 1.0f},  // 第3个2阶节
    {1.0f, 2.0f, 1.0f},  // 第4个2阶节
    {1.0f, 2.0f, 1.0f},  // 第5个2阶节
    {1.0f, 2.0f, 1.0f},  // 第6个2阶节
    {1.0f, 2.0f, 1.0f},  // 第7个2阶节
    {1.0f, 2.0f, 1.0f}   // 第8个2阶节
};

// 分母系数
static const float32_t iir_16th_100hz_a_coeffs[8][3] = {
    {1.0f, -1.998466253f, 0.9985230565f},  // 第1个2阶节
    {1.0f, -1.995575428f, 0.9956322312f},  // 第2个2阶节
    {1.0f, -1.992860079f, 0.9929167628f},  // 第3个2阶节
    {1.0f, -1.990422606f, 0.9904792309f},  // 第4个2阶节
    {1.0f, -1.988354445f, 0.9884109497f},  // 第5个2阶节
    {1.0f, -1.986732483f, 0.9867889285f},  // 第6个2阶节
    {1.0f, -1.985616803f, 0.9856731892f},  // 第7个2阶节
    {1.0f, -1.985048413f, 0.9851048589f}   // 第8个2阶节
};

// ==================== 100Hz采样率16阶低通滤波器系数 ====================

// 0.15Hz截止频率滤波器系数
static const float32_t iir_015hz_gain[9] = {
    2.218595e-05f,     // 第1个增益节
    2.214585766e-05f,  // 第2个增益节
    2.210822458e-05f,  // 第3个增益节
    2.207446414e-05f,  // 第4个增益节
    2.204583325e-05f,  // 第5个增益节
    2.202339238e-05f,  // 第6个增益节
    2.200796007e-05f,  // 第7个增益节
    2.200010022e-05f,  // 第8个增益节
    1.0f               // 第9个增益节（直通）
};


static const float32_t iir_015hz_a_coeffs[8][3] = {
    {1.0f, -1.998065352f, 0.9981541634f},  // 第1个2阶节
    {1.0f, -1.994454741f, 0.9945432544f},  // 第2个2阶节
    {1.0f, -1.991065383f, 0.9911538363f},  // 第3个2阶节
    {1.0f, -1.98802495f, 0.9881132245f},   // 第4个2阶节
    {1.0f, -1.985446572f, 0.985534668f},   // 第5个2阶节
    {1.0f, -1.983425379f, 0.9835134745f},  // 第6个2阶节
    {1.0f, -1.982035518f, 0.9821236134f},  // 第7个2阶节
    {1.0f, -1.981327772f, 0.9814158082f}   // 第8个2阶节
};

// 0.7Hz截止频率滤波器系数
static const float32_t iir_070hz_gain[9] = {
    0.0004814577696f,  // 第1个增益节
    0.0004774389672f,  // 第2个增益节
    0.000473714259f,   // 第3个增益节
    0.0004704114399f,  // 第4个增益节
    0.0004676386307f,  // 第5个增益节
    0.0004654829099f,  // 第6个增益节
    0.0004640095285f,  // 第7个增益节
    0.0004632619966f,  // 第8个增益节
    1.0f               // 第9个增益节（直通）
};



static const float32_t iir_070hz_a_coeffs[8][3] = {
    {1.0f, -1.98949194f, 0.991417706f},    // 第1个2阶节
    {1.0f, -1.97288537f, 0.9747951627f},   // 第2个2阶节
    {1.0f, -1.95749402f, 0.9593888521f},   // 第3个2阶节
    {1.0f, -1.943845987f, 0.9457276464f},  // 第4个2阶节
    {1.0f, -1.932388186f, 0.934258759f},   // 第5个2阶节
    {1.0f, -1.923480153f, 0.9253421426f},  // 第6个2阶节
    {1.0f, -1.917391896f, 0.9192479253f},  // 第7个2阶节
    {1.0f, -1.914302826f, 0.9161559343f}   // 第8个2阶节
};

// 1.2Hz截止频率滤波器系数
static const float32_t iir_120hz_gain[9] = {
    0.001410138328f,   // 第1个增益节
    0.001390152494f,   // 第2个增益节
    0.00137183757f,    // 第3个增益节
    0.001355762244f,   // 第4个增益节
    0.00134238496f,    // 第5个增益节
    0.001332058222f,   // 第6个增益节
    0.001325036865f,   // 第7个增益节
    0.001321485848f,   // 第8个增益节
    1.0f               // 第9个增益节（直通）
};


static const float32_t iir_120hz_a_coeffs[8][3] = {
    {1.0f, -1.979701042f, 0.9853416085f},  // 第1个2阶节
    {1.0f, -1.951642752f, 0.9572033882f},  // 第2个2阶节
    {1.0f, -1.925930262f, 0.931417644f},   // 第3个2阶节
    {1.0f, -1.903362155f, 0.908785224f},   // 第4个2阶节
    {1.0f, -1.884581566f, 0.8899511695f},  // 第5个2阶节
    {1.0f, -1.870083809f, 0.8754120469f},  // 第6个2阶节
    {1.0f, -1.860226631f, 0.8655267358f},  // 第7个2阶节
    {1.0f, -1.855241299f, 0.860527277f}    // 第8个2阶节
};

// 1.7Hz截止频率滤波器系数
static const float32_t iir_170hz_gain[9] = {
    0.002820135094f,   // 第1个增益节
    0.002764063654f,   // 第2个增益节
    0.002713247435f,   // 第3个增益节
    0.002669085516f,   // 第4个增益节
    0.002632644493f,   // 第5个增益节
    0.002604703652f,   // 第6个增益节
    0.002585800132f,   // 第7个增益节
    0.002576268511f,   // 第8个增益节
    1.0f               // 第9个增益节（直通）
};



static const float32_t iir_170hz_a_coeffs[8][3] = {
    {1.0f, -1.968036175f, 0.9793167114f},  // 第1个2阶节
    {1.0f, -1.92890656f, 0.9399628639f},   // 第2个2阶节
    {1.0f, -1.8934443f, 0.9042973518f},    // 第3个2阶节
    {1.0f, -1.862625837f, 0.8733022213f},  // 第4个2阶节
    {1.0f, -1.837195516f, 0.847726047f},   // 第5个2阶节
    {1.0f, -1.817696929f, 0.8281157017f},  // 第6个2阶节
    {1.0f, -1.804504991f, 0.8148482442f},  // 第7个2阶节
    {1.0f, -1.797853351f, 0.8081583977f}   // 第8个2阶节
};

// 2.2Hz截止频率滤波器系数
static const float32_t iir_220hz_gain[9] = {
    0.004705732223f,   // 第1个增益节
    0.004585859831f,   // 第2个增益节
    0.004478397779f,   // 第3个增益节
    0.004385901149f,   // 第4个增益节
    0.004310193937f,   // 第5个增益节
    0.004252519924f,   // 第6个增益节
    0.004213683307f,   // 第7个增益节
    0.004194155801f,   // 第8个增益节
    1.0f               // 第9个增益节（直通）
};



static const float32_t iir_220hz_a_coeffs[8][3] = {
    {1.0f, -1.954525352f, 0.9733483195f},  // 第1个2阶节
    {1.0f, -1.9047364f, 0.9230798483f},    // 第2个2阶节
    {1.0f, -1.860101938f, 0.8780155778f},  // 第3个2阶节
    {1.0f, -1.821683645f, 0.8392271996f},  // 第4个2阶节
    {1.0f, -1.790238619f, 0.807479322f},   // 第5个2阶节
    {1.0f, -1.766283751f, 0.7832937837f},  // 第6个2阶节
    {1.0f, -1.750152826f, 0.7670075297f},  // 第7个2阶节
    {1.0f, -1.742042184f, 0.7588188052f}   // 第8个2阶节
};

// 2.7Hz截止频率滤波器系数
static const float32_t iir_270hz_gain[9] = {
    0.007060855161f,   // 第1个增益节
    0.0068423599f,     // 第2个增益节
    0.006648560055f,   // 第3个增益节
    0.006483297329f,   // 第4个增益节
    0.006349083502f,   // 第5个增益节
    0.006247468293f,   // 第6个增益节
    0.006179345306f,   // 第7个增益节
    0.006145185325f,   // 第8个增益节
    1.0f               // 第9个增益节（直通）
};



static const float32_t iir_270hz_a_coeffs[8][3] = {
    {1.0f, -1.939198256f, 0.967441678f},   // 第1个2阶节
    {1.0f, -1.879190564f, 0.9065599442f},  // 第2个2阶节
    {1.0f, -1.825965166f, 0.8525593877f},  // 第3个2阶节
    {1.0f, -1.780577302f, 0.8065104485f},  // 第4个2阶节
    {1.0f, -1.743716717f, 0.7691130042f},  // 第5个2阶节
    {1.0f, -1.715808988f, 0.7407988906f},  // 第6个2阶节
    {1.0f, -1.697099805f, 0.7218171358f},  // 第7个2阶节
    {1.0f, -1.687718034f, 0.7122987509f}   // 第8个2阶节
};

// 3.2Hz截止频率滤波器系数
static const float32_t iir_320hz_gain[9] = {
    0.009879090823f,   // 第1个增益节
    0.009520541877f,   // 第2个增益节
    0.009205814451f,   // 第3个增益节
    0.008939843625f,   // 第4个增益节
    0.008725455962f,   // 第5个增益节
    0.00856409315f,    // 第6个增益节
    0.008456371725f,   // 第7个增益节
    0.008402492851f,   // 第8个增益节
    1.0f               // 第9个增益节（直通）
};



static const float32_t iir_320hz_a_coeffs[8][3] = {
    {1.0f, -1.922085285f, 0.9616016746f},  // 第1个2阶节
    {1.0f, -1.852325678f, 0.8904078603f},  // 第2个2阶节
    {1.0f, -1.791092038f, 0.8279152513f},  // 第3个2阶节
    {1.0f, -1.739344478f, 0.7751038074f},  // 第4个2阶节
    {1.0f, -1.697633028f, 0.7325348854f},  // 第5个2阶节
    {1.0f, -1.66623807f, 0.7004944682f},   // 第6个2阶节
    {1.0f, -1.645279765f, 0.6791052818f},  // 第7个2阶节
    {1.0f, -1.634796977f, 0.6684069037f}   // 第8个2阶节
};

// 3.7Hz截止频率滤波器系数
static const float32_t iir_370hz_gain[9] = {
    0.01315370575f,    // 第1个增益节
    0.01260756887f,    // 第2个增益节
    0.01213303767f,    // 第3个增益节
    0.01173551101f,    // 第4个增益节
    0.01141738333f,    // 第5个增益节
    0.0111792786f,     // 第6个增益节
    0.01102096401f,    // 第7个增益节
    0.01094196923f,    // 第8个增益节
    1.0f               // 第9个增益节（直通）
};


static const float32_t iir_370hz_a_coeffs[8][3] = {
    {1.0f, -1.903218389f, 0.955833137f},   // 第1个2阶节
    {1.0f, -1.824197412f, 0.8746276498f},  // 第2个2阶节
    {1.0f, -1.755537271f, 0.8040693402f},  // 第3个2阶节
    {1.0f, -1.698018789f, 0.7449608445f},  // 第4个2阶节
    {1.0f, -1.651988626f, 0.6976582408f},  // 第5个2阶节
    {1.0f, -1.617537141f, 0.6622542739f},  // 第6个2阶节
    {1.0f, -1.59463048f, 0.6387143731f},   // 第7个2阶节
    {1.0f, -1.583200693f, 0.6269685626f}   // 第8个2阶节
};

// 100Hz采样率16阶低通滤波器系数表
static const float32_t* const iir_lowpass_100hz_gain_coeffs[IIR_LOWPASS_100_TYPE_COUNT] = {
    iir_015hz_gain,   // 0.15Hz
    iir_070hz_gain,   // 0.7Hz
    iir_120hz_gain,   // 1.2Hz
    iir_170hz_gain,   // 1.7Hz
    iir_220hz_gain,   // 2.2Hz
    iir_270hz_gain,   // 2.7Hz
    iir_320hz_gain,   // 3.2Hz
    iir_370hz_gain    // 3.7Hz
};



// 100Hz采样率16阶低通滤波器分子系数表（所有类型都使用相同的分子系数）
static const float32_t (* const iir_lowpass_100hz_b_coeffs[IIR_LOWPASS_100_TYPE_COUNT])[3] = {
    (const float32_t(*)[3])iir_shared_b_coeffs,  // 0.15Hz
    (const float32_t(*)[3])iir_shared_b_coeffs,  // 0.7Hz
    (const float32_t(*)[3])iir_shared_b_coeffs,  // 1.2Hz
    (const float32_t(*)[3])iir_shared_b_coeffs,  // 1.7Hz
    (const float32_t(*)[3])iir_shared_b_coeffs,  // 2.2Hz
    (const float32_t(*)[3])iir_shared_b_coeffs,  // 2.7Hz
    (const float32_t(*)[3])iir_shared_b_coeffs,  // 3.2Hz
    (const float32_t(*)[3])iir_shared_b_coeffs   // 3.7Hz
};

static const float32_t (* const iir_lowpass_100hz_a_coeffs[IIR_LOWPASS_100_TYPE_COUNT])[3] = {
    iir_015hz_a_coeffs,   // 0.15Hz
    iir_070hz_a_coeffs,   // 0.7Hz
    iir_120hz_a_coeffs,   // 1.2Hz
    iir_170hz_a_coeffs,   // 1.7Hz
    iir_220hz_a_coeffs,   // 2.2Hz
    iir_270hz_a_coeffs,   // 2.7Hz
    iir_320hz_a_coeffs,   // 3.2Hz
    iir_370hz_a_coeffs    // 3.7Hz
};

// 截止频率表
static const float32_t iir_lowpass_100hz_cutoff_freqs[IIR_LOWPASS_100_TYPE_COUNT] = {
    0.15f,  // IIR_LOWPASS_100_015_HZ
    0.7f,   // IIR_LOWPASS_100_070_HZ
    1.2f,   // IIR_LOWPASS_100_120_HZ
    1.7f,   // IIR_LOWPASS_100_170_HZ
    2.2f,   // IIR_LOWPASS_100_220_HZ
    2.7f,   // IIR_LOWPASS_100_270_HZ
    3.2f,   // IIR_LOWPASS_100_320_HZ
    3.7f    // IIR_LOWPASS_100_370_HZ
};


/******************************* 函数实现 ********************************/


/**
 * @brief 初始化8阶IIR低通滤波器（4个二阶节格式）
 * @param filter 8阶滤波器结构体指针
 */
void iir_lowpass_filter_8th_init(iir_lowpass_filter_8th_t *filter)
{
    if (filter == NULL) {
        return;
    }
    
    // 清零所有延迟线
    memset(filter->x_delay, 0, sizeof(filter->x_delay));
    memset(filter->y_delay, 0, sizeof(filter->y_delay));
    
    filter->is_initialized = true;
}

/**
 * @brief 处理一个输入样本（8阶IIR低通滤波器 - 4个二阶节格式）
 * @param filter 8阶滤波器结构体指针
 * @param input 输入样本
 * @return 滤波后的输出样本
 */
float32_t iir_lowpass_filter_8th_process(iir_lowpass_filter_8th_t *filter, float32_t input)
{
    if (filter == NULL || !filter->is_initialized) {
        return input;
    }
    
    float32_t current_input = input;
    float32_t current_output = 0.0f;

    // 处理4个二阶节
    for (int section = 0; section < IIR_LOWPASS_SECTIONS_8TH; section++) {
        // 计算当前节的输出：y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        current_output = iir_8th_100hz_b_coeffs[section][0] * current_input;

        // 添加输入延迟项
        for (int i = 0; i < 2; i++) {
            current_output += iir_8th_100hz_b_coeffs[section][i + 1] * filter->x_delay[section][i];
        }

        // 减去输出延迟项
        for (int i = 0; i < 2; i++) {
            current_output -= iir_8th_100hz_a_coeffs[section][i + 1] * filter->y_delay[section][i];
        }

        // 更新当前节的延迟线
        for (int i = 1; i > 0; i--) {
            filter->x_delay[section][i] = filter->x_delay[section][i - 1];
        }
        filter->x_delay[section][0] = current_input;

        for (int i = 1; i > 0; i--) {
            filter->y_delay[section][i] = filter->y_delay[section][i - 1];
        }
        filter->y_delay[section][0] = current_output;

        // 应用当前节的增益
        current_output *= iir_8th_100hz_gain[section];

        // 当前节的输出作为下一节的输入
        current_input = current_output;
    }

    // 应用最后的增益
    current_output *= iir_8th_100hz_gain[IIR_LOWPASS_SECTIONS_8TH];

    // 返回最终输出
    return current_output;
}

/**
 * @brief 重置8阶IIR低通滤波器状态
 * @param filter 8阶滤波器结构体指针
 */
void iir_lowpass_filter_8th_reset(iir_lowpass_filter_8th_t *filter)
{
    if (filter == NULL) {
        return;
    }

    // 清零所有延迟线
    memset(filter->x_delay, 0, sizeof(filter->x_delay));
    memset(filter->y_delay, 0, sizeof(filter->y_delay));
}

/**
 * @brief 初始化16阶IIR低通滤波器（8个二阶节格式）
 * @param filter 16阶滤波器结构体指针
 */
void iir_lowpass_filter_16th_init(iir_lowpass_filter_16th_t *filter)
{
    if (filter == NULL) {
        return;
    }

    // 清零所有延迟线
    memset(filter->x_delay, 0, sizeof(filter->x_delay));
    memset(filter->y_delay, 0, sizeof(filter->y_delay));

    filter->is_initialized = true;
}

/**
 * @brief 处理一个输入样本（16阶IIR低通滤波器 - 8个二阶节格式）
 * @param filter 16阶滤波器结构体指针
 * @param input 输入样本
 * @return 滤波后的输出样本
 */
float32_t iir_lowpass_filter_16th_process(iir_lowpass_filter_16th_t *filter, float32_t input)
{
    if (filter == NULL || !filter->is_initialized) {
        return input;
    }

    float32_t current_input = input;
    float32_t current_output = 0.0f;
    
    // 处理8个二阶节
    for (int section = 0; section < IIR_LOWPASS_SECTIONS_16TH; section++) {
        // 计算当前节的输出：y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        current_output = iir_16th_100hz_b_coeffs[section][0] * current_input;

        // 添加输入延迟项
        for (int i = 0; i < 2; i++) {
            current_output += iir_16th_100hz_b_coeffs[section][i + 1] * filter->x_delay[section][i];
        }

        // 减去输出延迟项
        for (int i = 0; i < 2; i++) {
            current_output -= iir_16th_100hz_a_coeffs[section][i + 1] * filter->y_delay[section][i];
        }

        // 更新当前节的延迟线
        for (int i = 1; i > 0; i--) {
            filter->x_delay[section][i] = filter->x_delay[section][i - 1];
        }
        filter->x_delay[section][0] = current_input;

        for (int i = 1; i > 0; i--) {
            filter->y_delay[section][i] = filter->y_delay[section][i - 1];
        }
        filter->y_delay[section][0] = current_output;

        // 应用当前节的增益
        current_output *= iir_16th_100hz_gain[section];

        // 当前节的输出作为下一节的输入
        current_input = current_output;
    }

    // 应用最后的增益
    current_output *= iir_16th_100hz_gain[IIR_LOWPASS_SECTIONS_16TH];

    // 返回最终输出
    return current_output;
}

/**
 * @brief 重置16阶IIR低通滤波器状态
 * @param filter 16阶滤波器结构体指针
 */
void iir_lowpass_filter_16th_reset(iir_lowpass_filter_16th_t *filter)
{
    if (filter == NULL) {
        return;
    }

    // 清零所有延迟线
    memset(filter->x_delay, 0, sizeof(filter->x_delay));
    memset(filter->y_delay, 0, sizeof(filter->y_delay));
}

// ==================== 100Hz采样率16阶低通滤波器函数实现 ====================

/**
 * @brief 初始化16阶IIR低通滤波器（100Hz采样率，多种截止频率）
 * @param filter 16阶滤波器结构体指针
 * @param type 滤波器类型
 */
void iir_lowpass_filter_16th_100hz_init(iir_lowpass_filter_16th_100hz_t *filter, iir_lowpass_100_filter_type_t type)
{
    if (filter == NULL || type >= IIR_LOWPASS_100_TYPE_COUNT) {
        return;
    }

    // 清零所有延迟线
    memset(filter->x_delay, 0, sizeof(filter->x_delay));
    memset(filter->y_delay, 0, sizeof(filter->y_delay));
    
    filter->filter_type = type;
    filter->is_initialized = true;
}

/**
 * @brief 处理一个输入样本（16阶IIR低通滤波器 - 100Hz采样率，多种截止频率）
 * @param filter 16阶滤波器结构体指针
 * @param input 输入样本
 * @return 滤波后的输出样本
 */
float32_t iir_lowpass_filter_16th_100hz_process(iir_lowpass_filter_16th_100hz_t *filter, float32_t input)
{
    if (filter == NULL || !filter->is_initialized || filter->filter_type >= IIR_LOWPASS_100_TYPE_COUNT) {
        return input;
    }

    // 获取当前滤波器类型的系数
    const float32_t* gain_coeffs = iir_lowpass_100hz_gain_coeffs[filter->filter_type];
    const float32_t (*b_coeffs)[3] = iir_lowpass_100hz_b_coeffs[filter->filter_type];
    const float32_t (*a_coeffs)[3] = iir_lowpass_100hz_a_coeffs[filter->filter_type];

    // 检查系数是否已加载
    if (gain_coeffs == NULL || b_coeffs == NULL || a_coeffs == NULL) {
        return input;  // 系数未加载，直接返回输入
    }

    float32_t current_input = input;
    float32_t current_output = 0.0f;

    // 处理8个二阶节
    for (int section = 0; section < LOW_PASS_SECTIONS; section++) {
        // 应用增益
        float32_t gain_output = current_input * gain_coeffs[section];

        // 计算当前节的输出
        current_output = b_coeffs[section][0] * gain_output;

        // 添加输入延迟项
        for (int i = 0; i < 2; i++) {
            current_output += b_coeffs[section][i + 1] * filter->x_delay[section][i];
        }

        // 减去输出延迟项
        for (int i = 0; i < 2; i++) {
            current_output -= a_coeffs[section][i + 1] * filter->y_delay[section][i];
        }

        // 更新当前节的延迟线
        for (int i = 1; i > 0; i--) {
            filter->x_delay[section][i] = filter->x_delay[section][i - 1];
        }
        filter->x_delay[section][0] = gain_output;

        for (int i = 1; i > 0; i--) {
            filter->y_delay[section][i] = filter->y_delay[section][i - 1];
        }
        filter->y_delay[section][0] = current_output;

        // 当前节的输出作为下一节的输入
        current_input = current_output;
    }

    // 应用最后的增益
    current_output *= gain_coeffs[LOW_PASS_SECTIONS];

    // 返回最终输出
    return current_output;
}

/**
 * @brief 重置16阶IIR低通滤波器状态（100Hz采样率）
 * @param filter 16阶滤波器结构体指针
 */
void iir_lowpass_filter_16th_100hz_reset(iir_lowpass_filter_16th_100hz_t *filter)
{
    if (filter == NULL) {
        return;
    }

    // 清零所有延迟线
    memset(filter->x_delay, 0, sizeof(filter->x_delay));
    memset(filter->y_delay, 0, sizeof(filter->y_delay));
}

/**
 * @brief 设置16阶IIR低通滤波器类型（100Hz采样率）
 * @param filter 16阶滤波器结构体指针
 * @param type 新的滤波器类型
 */
void iir_lowpass_filter_16th_100hz_set_type(iir_lowpass_filter_16th_100hz_t *filter, iir_lowpass_100_filter_type_t type)
{
    if (filter == NULL || type >= IIR_LOWPASS_100_TYPE_COUNT) {
        return;
    }

    // 重置滤波器状态
    iir_lowpass_filter_16th_100hz_reset(filter);

    // 设置新的滤波器类型
    filter->filter_type = type;
}

/**
 * @brief 批量初始化所有100Hz采样率的IIR低通滤波器
 * @param filters 滤波器数组
 */
void iir_lowpass_filter_16th_100hz_init_all(iir_lowpass_filter_16th_100hz_t filters[IIR_LOWPASS_100_TYPE_COUNT])
{
    if (filters == NULL) {
        return;
    }

    for (int i = 0; i < IIR_LOWPASS_100_TYPE_COUNT; i++) {
        iir_lowpass_filter_16th_100hz_init(&filters[i], (iir_lowpass_100_filter_type_t)i);
    }
}

/**
 * @brief 批量重置所有100Hz采样率的IIR低通滤波器
 * @param filters 滤波器数组
 */
void iir_lowpass_filter_16th_100hz_reset_all(iir_lowpass_filter_16th_100hz_t filters[IIR_LOWPASS_100_TYPE_COUNT])
{
    if (filters == NULL) {
        return;
    }

    for (int i = 0; i < IIR_LOWPASS_100_TYPE_COUNT; i++) {
        iir_lowpass_filter_16th_100hz_reset(&filters[i]);
    }
}

/**
 * @brief 获取滤波器类型对应的截止频率（100Hz采样率）
 * @param type 滤波器类型
 * @return 截止频率（Hz）
 */
float32_t iir_lowpass_get_cutoff_freq_100hz(iir_lowpass_100_filter_type_t type)
{
    if (type >= IIR_LOWPASS_100_TYPE_COUNT) {
        return 0.15f;  // 默认返回0.15Hz
    }

    return iir_lowpass_100hz_cutoff_freqs[type];
}
