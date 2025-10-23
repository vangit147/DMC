/**
  *****************************************************************************
  * FILENAME   : iir_highpass_filter.c
  * COPYRIGHT  : ModingTech(ShangHai) Co.,Ltd2025.
  * CREATEDDATE: 2025.09.10 20:36
  * DESCRIPTION: IIR高通滤波器实现，实现16阶IIR高通巴特沃斯滤波器
  *              支持100Hz采样率，多种截止频率，用于对低速旋转的加速度计信号进行离心力分离
  *              适用于需要高通滤波的场景，如分离低频漂移和高频振动信号
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  *   Gordon Li                  2025.09.10                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "iir_highpass_filter.h"
#include <string.h>

// 系统参数定义
#define SAMPLING_FREQ 100.0f       // 采样频率 100Hz (10ms周期)
#define HIGH_PASS_SECTIONS 8       // 高通滤波器节数（16阶，8个二阶节）

// 16阶高通滤波 - 8个二阶节格式（MATLAB生成）
// 16阶Butterworth高通滤波器，多种截止频率，采样频率100Hz
// 采用双二阶节级联结构，双线性变换，确保数值稳定性

// 0.05Hz截止频率滤波器系数
static const float32_t iir_005hz_gain[9] = {
    0.9996896982f,     // 第1个增益节
    0.9990864396f,     // 第2个增益节
    0.998518765f,      // 第3个增益节
    0.9980084896f,     // 第4个增益节
    0.9975749254f,     // 第5个增益节
    0.9972345829f,     // 第6个增益节
    0.9970002174f,     // 第7个增益节
    0.9968808293f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_005hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
};

static const float32_t iir_005hz_a_coeffs[8][3] = {
    {1.0f, -1.999374509f, 0.9993843436f},  // 第1个2阶节
    {1.0f, -1.998167872f, 0.9981777668f},  // 第2个2阶节
    {1.0f, -1.997032642f, 0.9970425367f},  // 第3个2阶节
    {1.0f, -1.996012092f, 0.9960219264f},  // 第4个2阶节
    {1.0f, -1.995144963f, 0.995154798f},   // 第5个2阶节
    {1.0f, -1.994464159f, 0.9944740534f},  // 第6个2阶节
    {1.0f, -1.993995547f, 0.9940053821f},  // 第7个2阶节
    {1.0f, -1.993756771f, 0.9937665462f}   // 第8个2阶节
};

// 0.2Hz截止频率滤波器系数（从IIR_HIGH_100_16_020_BUTT.h提取）
static const float32_t iir_020hz_gain[9] = {
    0.9987304211f,     // 第1个增益节
    0.9963262081f,     // 第2个增益节
    0.9940720201f,     // 第3个增益节
    0.9920520782f,     // 第4个增益节
    0.9903406501f,     // 第5个增益节
    0.9890001416f,     // 第6个增益节
    0.9880789518f,     // 第7个增益节
    0.9876099229f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_020hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
};

static const float32_t iir_020hz_a_coeffs[8][3] = {
    {1.0f, -1.997381926f, 0.9975396395f},  // 第1个2阶节
    {1.0f, -1.992573738f, 0.9927310348f},  // 第2个2阶节
    {1.0f, -1.9880656f, 0.988222599f},     // 第3个2阶节
    {1.0f, -1.984025836f, 0.984182477f},   // 第4个2阶节
    {1.0f, -1.980603099f, 0.9807595611f},  // 第5个2阶节
    {1.0f, -1.977922201f, 0.9780784249f},  // 第6个2阶节
    {1.0f, -1.976079822f, 0.9762358665f},  // 第7个2阶节
    {1.0f, -1.975141883f, 0.9752978683f}   // 第8个2阶节
};

// 0.3Hz截止频率滤波器系数（从IIR_HIGH_100_16_030_BUTT.h提取）
static const float32_t iir_030hz_gain[9] = {
    0.9980672598f,     // 第1个增益节
    0.9944700003f,     // 第2个增益节
    0.9911051393f,     // 第3个增益节
    0.9880961776f,     // 第4个增益节
    0.9855516553f,     // 第5个增益节
    0.9835615754f,     // 第6个增益节
    0.9821954966f,     // 第7个增益节
    0.9815005064f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_030hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
};

static const float32_t iir_030hz_a_coeffs[8][3] = {
    {1.0f, -1.995957255f, 0.996311903f},   // 第1个2阶节
    {1.0f, -1.988763332f, 0.9891167283f},  // 第2个2阶节
    {1.0f, -1.982034206f, 0.982386291f},   // 第3个2阶节
    {1.0f, -1.976016879f, 0.9763679504f},  // 第4个2阶节
    {1.0f, -1.970928192f, 0.9712783694f},  // 第5个2阶节
    {1.0f, -1.96694839f, 0.9672979116f},   // 第6个2阶节
    {1.0f, -1.964216471f, 0.9645654559f},  // 第7个2阶节
    {1.0f, -1.96282661f, 0.9631753564f}    // 第8个2阶节
};

// 0.5Hz截止频率滤波器系数（从IIR_HIGH_100_16_050_BUTT.h提取）
static const float32_t iir_050hz_gain[9] = {
    0.9966846704f,     // 第1个增益节
    0.9907198548f,     // 第2个增益节
    0.9851660132f,     // 第3个增益节
    0.9802206159f,     // 第4个增益节
    0.9760538936f,     // 第5个增益节
    0.9728048444f,     // 第6个增益节
    0.9705793858f,     // 第7个增益节
    0.9694488049f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_050hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
};

static const float32_t iir_050hz_a_coeffs[8][3] = {
    {1.0f, -1.992877483f, 0.9938613176f},  // 第1个2阶节
    {1.0f, -1.980950713f, 0.9819286466f},  // 第2个2阶节
    {1.0f, -1.969845772f, 0.9708182216f},  // 第3个2阶节
    {1.0f, -1.95995748f, 0.9609251022f},   // 第4个2阶节
    {1.0f, -1.951625943f, 0.9525894523f},  // 第5个2阶节
    {1.0f, -1.945129514f, 0.9460898042f},  // 第6个2阶节
    {1.0f, -1.940679789f, 0.941637814f},   // 第7个2阶节
    {1.0f, -1.938419104f, 0.9393760562f}   // 第8个2阶节
};

// 0.7Hz截止频率滤波器系数（从IIR_HIGH_100_16_070_BUTT.h提取）
static const float32_t iir_070hz_gain[9] = {
    0.9952273965f,     // 第1个增益节
    0.9869201183f,     // 第2个增益节
    0.9792207479f,     // 第3个增益节
    0.9723933935f,     // 第4个增益节
    0.9666617513f,     // 第5个增益节
    0.9622055888f,     // 第6个增益节
    0.9591599703f,     // 第7个增益节
    0.9576147199f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_070hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
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

// 1.2Hz截止频率滤波器系数（从IIR_HIGH_100_16_120_BUTT.h提取）
static const float32_t iir_120hz_gain[9] = {
    0.9912606478f,     // 第1个增益节
    0.977211535f,      // 第2个增益节
    0.9643369913f,     // 第3个增益节
    0.9530368447f,     // 第4个增益节
    0.9436331987f,     // 第5个增益节
    0.9363739491f,     // 第6个增益节
    0.9314383268f,     // 第7个增益节
    0.9289421439f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_120hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
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

// 1.7Hz截止频率滤波器系数（从IIR_HIGH_100_16_170_BUTT.h提取）
static const float32_t iir_170hz_gain[9] = {
    0.9868382215f,     // 第1个增益节
    0.9672173262f,     // 第2个增益节
    0.9494354129f,     // 第3个增益节
    0.9339820147f,     // 第4个增益节
    0.9212303758f,     // 第5个增益节
    0.9114531875f,     // 第6个增益节
    0.9048383236f,     // 第7个增益节
    0.9015029073f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_170hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
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

// 2.2Hz截止频率滤波器系数（从IIR_HIGH_100_16_220_BUTT.h提取）
static const float32_t iir_220hz_gain[9] = {
    0.9819684625f,     // 第1个增益节
    0.956954062f,      // 第2个增益节
    0.9345293641f,     // 第3个增益节
    0.9152277112f,     // 第4个增益节
    0.8994295001f,     // 第5个增益节
    0.8873943686f,     // 第6个增益节
    0.8792901039f,     // 第7个增益节
    0.8752152324f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_220hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
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

// 2.7Hz截止频率滤波器系数（从IIR_HIGH_100_16_270_BUTT.h提取）
static const float32_t iir_270hz_gain[9] = {
    0.9766599536f,     // 第1个增益节
    0.9464375973f,     // 第2个增益节
    0.9196311235f,     // 第3个增益节
    0.8967719078f,     // 第4个增益节
    0.8782074451f,     // 第5个增益节
    0.8641519547f,     // 第6个增益节
    0.8547292352f,     // 第7个增益节
    0.8500041962f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_270hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
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

// 3.2Hz截止频率滤波器系数（从IIR_HIGH_100_16_320_BUTT.h提取）
static const float32_t iir_320hz_gain[9] = {
    0.9709217548f,     // 第1个增益节
    0.9356833696f,     // 第2个增益节
    0.9047518373f,     // 第3个增益节
    0.8786120415f,     // 第4个增益节
    0.8575419784f,     // 第5个增益节
    0.8416831493f,     // 第6个增益节
    0.8310962915f,     // 第7个增益节
    0.8258009553f,     // 第8个增益节
    1.0f               // 第9个增益节（直通）
};

static const float32_t iir_320hz_b_coeffs[8][3] = {
    {1.0f, -2.0f, 1.0f},  // 第1个2阶节
    {1.0f, -2.0f, 1.0f},  // 第2个2阶节
    {1.0f, -2.0f, 1.0f},  // 第3个2阶节
    {1.0f, -2.0f, 1.0f},  // 第4个2阶节
    {1.0f, -2.0f, 1.0f},  // 第5个2阶节
    {1.0f, -2.0f, 1.0f},  // 第6个2阶节
    {1.0f, -2.0f, 1.0f},  // 第7个2阶节
    {1.0f, -2.0f, 1.0f}   // 第8个2阶节
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

// 滤波器系数指针表
static const float32_t* const iir_gain_coeffs[IIR_HIGHPASS_100_TYPE_COUNT] = {
    iir_005hz_gain,   // 0.1Hz - 已添加
    iir_020hz_gain,   // 0.2Hz - 已添加
    iir_030hz_gain,   // 0.3Hz - 已添加
    iir_050hz_gain,   // 0.5Hz - 已添加
    iir_070hz_gain,   // 0.7Hz - 已添加
    iir_120hz_gain,   // 1.2Hz - 已添加
    iir_170hz_gain,   // 1.7Hz - 已添加
    iir_220hz_gain,   // 2.2Hz - 已添加
    iir_270hz_gain,   // 2.7Hz - 已添加
    iir_320hz_gain    // 3.2Hz - 已添加
};

static const float32_t (* const iir_b_coeffs[IIR_HIGHPASS_100_TYPE_COUNT])[3] = {
    iir_005hz_b_coeffs,  // 0.1Hz - 已添加
    iir_020hz_b_coeffs,  // 0.2Hz - 已添加
    iir_030hz_b_coeffs,  // 0.3Hz - 已添加
    iir_050hz_b_coeffs,  // 0.5Hz - 已添加
    iir_070hz_b_coeffs,  // 0.7Hz - 已添加
    iir_120hz_b_coeffs,  // 1.2Hz - 已添加
    iir_170hz_b_coeffs,  // 1.7Hz - 已添加
    iir_220hz_b_coeffs,  // 2.2Hz - 已添加
    iir_270hz_b_coeffs,  // 2.7Hz - 已添加
    iir_320hz_b_coeffs   // 3.2Hz - 已添加
};

static const float32_t (* const iir_a_coeffs[IIR_HIGHPASS_100_TYPE_COUNT])[3] = {
    iir_005hz_a_coeffs,  // 0.1Hz - 已添加
    iir_020hz_a_coeffs,  // 0.2Hz - 已添加
    iir_030hz_a_coeffs,  // 0.3Hz - 已添加
    iir_050hz_a_coeffs,  // 0.5Hz - 已添加
    iir_070hz_a_coeffs,  // 0.7Hz - 已添加
    iir_120hz_a_coeffs,  // 1.2Hz - 已添加
    iir_170hz_a_coeffs,  // 1.7Hz - 已添加
    iir_220hz_a_coeffs,  // 2.2Hz - 已添加
    iir_270hz_a_coeffs,  // 2.7Hz - 已添加
    iir_320hz_a_coeffs   // 3.2Hz - 已添加
};

// 截止频率表
static const float32_t cutoff_frequencies[IIR_HIGHPASS_100_TYPE_COUNT] = {
    0.1f,   // IIR_HIGHPASS_100_010_HZ
    0.2f,   // IIR_HIGHPASS_100_020_HZ
    0.3f,   // IIR_HIGHPASS_100_030_HZ
    0.5f,   // IIR_HIGHPASS_100_050_HZ
    0.7f,   // IIR_HIGHPASS_100_070_HZ
    1.2f,   // IIR_HIGHPASS_100_120_HZ
    1.7f,   // IIR_HIGHPASS_100_170_HZ
    2.2f,   // IIR_HIGHPASS_100_220_HZ
    2.7f,   // IIR_HIGHPASS_100_270_HZ
    3.2f    // IIR_HIGHPASS_100_320_HZ
};

/******************************* 函数实现 ********************************/

/**
 * @brief 初始化16阶IIR高通滤波器（8个二阶节格式）
 * @param filter 16阶滤波器结构体指针
 * @param type 滤波器类型
 */
void iir_highpass_filter_16th_init(iir_highpass_filter_16th_t *filter, iir_highpass_100_filter_type_t type)
{
    if (filter == NULL || type >= IIR_HIGHPASS_100_TYPE_COUNT) {
        return;
    }
    
    // 清零所有延迟线
    memset(filter->x_delay, 0, sizeof(filter->x_delay));
    memset(filter->y_delay, 0, sizeof(filter->y_delay));
    
    filter->filter_type = type;
    filter->is_initialized = true;
}

/**
 * @brief 处理一个输入样本（16阶IIR高通滤波器 - 8个二阶节格式）
 * @param filter 16阶滤波器结构体指针
 * @param input 输入样本
 * @return 滤波后的输出样本
 */
float32_t iir_highpass_filter_16th_process(iir_highpass_filter_16th_t *filter, float32_t input)
{
    if (filter == NULL || !filter->is_initialized || filter->filter_type >= IIR_HIGHPASS_100_TYPE_COUNT) {
        return input;
    }
    
    // 获取当前滤波器类型的系数
    const float32_t* gain_coeffs = iir_gain_coeffs[filter->filter_type];
    const float32_t (*b_coeffs)[3] = iir_b_coeffs[filter->filter_type];
    const float32_t (*a_coeffs)[3] = iir_a_coeffs[filter->filter_type];
    
    // 检查系数是否已加载
    if (gain_coeffs == NULL || b_coeffs == NULL || a_coeffs == NULL) {
        return input;  // 系数未加载，直接返回输入
    }
    
    float32_t current_input = input;
    float32_t current_output = 0.0f;
    
    // 处理8个二阶节
    for (int section = 0; section < HIGH_PASS_SECTIONS; section++) {
        // 计算当前节的输出：y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        current_output = b_coeffs[section][0] * current_input;
        
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
        filter->x_delay[section][0] = current_input;
        
        for (int i = 1; i > 0; i--) {
            filter->y_delay[section][i] = filter->y_delay[section][i - 1];
        }
        filter->y_delay[section][0] = current_output;
        
        // 应用当前节的增益
        current_output *= gain_coeffs[section];
        
        // 当前节的输出作为下一节的输入
        current_input = current_output;
    }
    
    // 应用最后的增益
    current_output *= gain_coeffs[HIGH_PASS_SECTIONS];
    
    // 返回最终输出
    return current_output;
}

/**
 * @brief 重置16阶IIR高通滤波器状态
 * @param filter 16阶滤波器结构体指针
 */
void iir_highpass_filter_16th_reset(iir_highpass_filter_16th_t *filter)
{
    if (filter == NULL) {
        return;
    }
    
    // 清零所有延迟线
    memset(filter->x_delay, 0, sizeof(filter->x_delay));
    memset(filter->y_delay, 0, sizeof(filter->y_delay));
}

/**
 * @brief 设置16阶IIR高通滤波器类型
 * @param filter 16阶滤波器结构体指针
 * @param type 新的滤波器类型
 */
void iir_highpass_filter_16th_set_type(iir_highpass_filter_16th_t *filter, iir_highpass_100_filter_type_t type)
{
    if (filter == NULL || type >= IIR_HIGHPASS_100_TYPE_COUNT) {
        return;
    }
    
    // 重置滤波器状态
    iir_highpass_filter_16th_reset(filter);
    
    // 设置新的滤波器类型
    filter->filter_type = type;
}

/**
 * @brief 批量初始化所有100Hz采样率的IIR高通滤波器
 * @param filters 滤波器数组
 */
void iir_highpass_filter_16th_init_all_100hz(iir_highpass_filter_16th_t filters[IIR_HIGHPASS_100_TYPE_COUNT])
{
    if (filters == NULL) {
        return;
    }
    
    for (int i = 0; i < IIR_HIGHPASS_100_TYPE_COUNT; i++) {
        iir_highpass_filter_16th_init(&filters[i], (iir_highpass_100_filter_type_t)i);
    }
}

/**
 * @brief 批量重置所有100Hz采样率的IIR高通滤波器
 * @param filters 滤波器数组
 */
void iir_highpass_filter_16th_reset_all_100hz(iir_highpass_filter_16th_t filters[IIR_HIGHPASS_100_TYPE_COUNT])
{
    if (filters == NULL) {
        return;
    }
    
    for (int i = 0; i < IIR_HIGHPASS_100_TYPE_COUNT; i++) {
        iir_highpass_filter_16th_reset(&filters[i]);
    }
}

/**
 * @brief 获取滤波器类型对应的截止频率
 * @param type 滤波器类型
 * @return 截止频率（Hz）
 */
float32_t iir_highpass_get_cutoff_freq(iir_highpass_100_filter_type_t type)
{
    if (type >= IIR_HIGHPASS_100_TYPE_COUNT) {
        return 0.0f;
    }
    
    return cutoff_frequencies[type];
}

/*
 * 使用示例：
 * 
 * // 单个滤波器使用（0.1Hz截止频率）
 * iir_highpass_filter_16th_t filter;
 * iir_highpass_filter_16th_init(&filter, IIR_HIGHPASS_100_010_HZ);
 * float32_t output = iir_highpass_filter_16th_process(&filter, input);
 * 
 * // 批量初始化所有滤波器
 * iir_highpass_filter_16th_t all_filters[IIR_HIGHPASS_100_TYPE_COUNT];
 * iir_highpass_filter_16th_init_all_100hz(all_filters);
 * 
 * // 使用不同截止频率的滤波器
 * float32_t output_010hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_010_HZ], input);
 * float32_t output_020hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_020_HZ], input);
 * float32_t output_030hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_030_HZ], input);
 * float32_t output_050hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_050_HZ], input);
 * float32_t output_070hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_070_HZ], input);
 * float32_t output_120hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_120_HZ], input);
 * float32_t output_170hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_170_HZ], input);
 * float32_t output_220hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_220_HZ], input);
 * float32_t output_270hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_270_HZ], input);
 * float32_t output_320hz = iir_highpass_filter_16th_process(&all_filters[IIR_HIGHPASS_100_320_HZ], input);
 * 
 * // 批量重置所有滤波器
 * iir_highpass_filter_16th_reset_all_100hz(all_filters);
 * 
 * // 动态切换滤波器类型
 * iir_highpass_filter_16th_set_type(&filter, IIR_HIGHPASS_100_050_HZ);
 * 
 * // 获取滤波器截止频率
 * float32_t cutoff_freq = iir_highpass_get_cutoff_freq(IIR_HIGHPASS_100_050_HZ); // 返回 0.5f
 */
