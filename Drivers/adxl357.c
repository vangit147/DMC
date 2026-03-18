/**
  *****************************************************************************
  * FILENAME   : adxl357.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2025.04.07 10:00:00 Monday
  * DESCRIPTION: ADXL357加速度计驱动，支持FIFO数据读取
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.25                    CREATE
  * Grodon Li                  2025.04.06                    MODIFY - ???4??
  *****************************************************************************
  */

/**
  *****************************************************************************
  * 数据获取方式说明
  *
  * 本文件支持两种ADXL357数据获取方式：
  *
  * 方式一：直接读取寄存器方式（on_adxl357_data_ready_event）
  * - 从TEMP2寄存器开始连续读取12字节数据
  * - 适合单次数据获取，响应速度快
  * - 数据格式：温度(2字节) + X轴(3字节) + Y轴(3字节) + Z轴(3字节)
  *
  * 方式二：FIFO方式（adxl357_process_data）
  * - 从FIFO中批量读取多个样本数据
  * - 适合批量数据获取，效率高
  * - 数据格式：每个样本9字节（X轴3字节 + Y轴3字节 + Z轴3字节）
  *
  * 中断机制：
  * - 使用中断服务函数adxl357_data_ready_isr
  * - 当ADXL357数据就绪时触发中断
  * - 通过任务通知机制通知adxl357_task处理数据
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#include "adxl357.h"
#include "filtering_functions.h"

#include "fir_config.h"

// ADXL357寄存器定义（根据数据手册）
#define REG_DEVID_AD        0x00    // 设备ID寄存器
#define REG_DEVID_MST       0x01    // 主设备ID寄存器
#define REG_PARTID          0x02    // 部件ID寄存器
#define REG_REVID           0x03    // 修订ID寄存器
#define REG_STATUS          0x04    // 状态寄存器（包含FIFO_FULL和FIFO_OVR位）
#define REG_FIFO_ENTRIES    0x05    // FIFO条目数寄存器
#define REG_TEMP2           0x06    // 温度数据寄存器2
#define REG_TEMP1           0x07    // 温度数据寄存器1
#define REG_XDATA3          0x08    // X轴数据寄存器3
#define REG_XDATA2          0x09    // X轴数据寄存器2
#define REG_XDATA1          0x0A    // X轴数据寄存器1
#define REG_YDATA3          0x0B    // Y轴数据寄存器3
#define REG_YDATA2          0x0C    // Y轴数据寄存器2
#define REG_YDATA1          0x0D    // Y轴数据寄存器1
#define REG_ZDATA3          0x0E    // Z轴数据寄存器3
#define REG_ZDATA2          0x0F    // Z轴数据寄存器2
#define REG_ZDATA1          0x10    // Z轴数据寄存器1
#define REG_FIFO_DATA       0x11    // FIFO数据寄存器
#define REG_OFFSET_X_H      0x1E    // X轴偏移高字节寄存器
#define REG_OFFSET_X_L      0x1F    // X轴偏移低字节寄存器
#define REG_OFFSET_Y_H      0x20    // Y轴偏移高字节寄存器
#define REG_OFFSET_Y_L      0x21    // Y轴偏移低字节寄存器
#define REG_OFFSET_Z_H      0x22    // Z轴偏移高字节寄存器
#define REG_OFFSET_Z_L      0x23    // Z轴偏移低字节寄存器
#define REG_ACT_EN          0x24    // 活动检测使能寄存器
#define REG_ACT_THRESH_H    0x25    // 活动阈值高字节寄存器
#define REG_ACT_THRESH_L    0x26    // 活动阈值低字节寄存器
#define REG_ACT_COUNT       0x27    // 活动计数寄存器
#define REG_FILTER          0x28    // 滤波器设置寄存器（ODR配置）
#define REG_FIFO_SAMPLES    0x29    // FIFO样本数寄存器
#define REG_INT_MAP         0x2A    // 中断映射寄存器
#define REG_SYNC            0x2B    // 同步寄存器
#define REG_RANGE           0x2C    // 量程寄存器
#define REG_POWER_CTL       0x2D    // 电源控制寄存器
#define REG_SELF_TEST       0x2E    // 自检寄存器
#define REG_RESET           0x2F    // 复位寄存器

// 量程设置（Register 0x2C）
#define SET_RANGE_10G       0x01        // 10g量程
#define SET_RANGE_20G       0x02        // 20g量程
#define SET_RANGE_40G       0x03        // 40g量程

// 高通滤波器设置（Register 0x28, Bits[6:4]）
#define SET_HPF_OFF         0x00        // 高通滤波器关闭
#define SET_HPF_247         0x01        // 247Hz高通滤波器
#define SET_HPF_62_084      0x02        // 62.084Hz高通滤波器
#define SET_HPF_15_545      0x03        // 15.545Hz高通滤波器
#define SET_HPF_3_862       0x04        // 3.862Hz高通滤波器
#define SET_HPF_0_954       0x05        // 0.954Hz高通滤波器
#define SET_HPF_0_238       0x06        // 0.238Hz高通滤波器

// ODR配置值和时间周期宏定义已移至adxl357.h中定义

// FIFO配置参数
// 注意：ADXL357_SAMPLES_PER_READ和ADXL357_DATA_GROUPS_PER_READ已移至adxl357.h中定义

// 数据转换相关
#define ADXL357_DATA_MASK           0x000FFFFF  // 20位数据掩码
#define ADXL357_SIGN_BIT            (1 << 19)   // 符号位（第19位）

#define EVENT_ADXL357_DATA_READY    0x80000000

// 全局变量定义

TaskHandle_t     adx357_task_handle;  // ADXL357任务句柄
adxl357_vibration_data_t vibration_data = {0};  // 显式初始化为0

// ADXL357重新配置后的FIFO清空控制
static uint32_t adxl357_fifo_clear_count = 0;        // FIFO清空次数计数
static uint32_t adxl357_fifo_clear_limit = ADXL357_FIFO_CLEAR_LIMIT;        // FIFO清空次数限制（使用宏定义配置）
static bool adxl357_recently_reconfigured = true;   // 最近是否重新配置过

// RMS计算相关的静态变量（用于日志统计）
static float sum_squared = 0.0f;  // 用于RMS计算的平方和
static uint32_t rms_count = 0;    // 用于RMS计算的数据点计数

// 新增：数据缓冲区，用于存储从FIFO读取的原始数据
uint8_t adxl357_raw_buffer[2][ADXL357_SAMPLES_PER_READ + 1];  // 双缓冲，+1 for command byte
volatile uint8_t adxl357_data_ready = 0;  // 数据就绪标志
volatile uint8_t adxl357_buffer_entries = 0;  // 缓冲区中的数据条目数
volatile uint8_t adxl357_current_buffer = 0;  // 当前写入缓冲区索引
volatile uint8_t adxl357_process_buffer = 0;  // 当前处理缓冲区索引

/******************************** Functions **********************************/
void Reset_Vibration_Stats(void)
{
    vibration_data.account = 0;
    vibration_data.min_vibration = 99999;
    vibration_data.max_vibration = -99999;
    vibration_data.avg_vibration = 0;

    // 重置日志记录相关的统计字段
    vibration_data.delta_count_total = 0;
    vibration_data.rms_over_count_total = 0;
    vibration_data.current_rms_value = 0.0f;
    vibration_data.max_delta_value_in_period = 0.0f;

    // 重置RMS计算相关的静态变量
    sum_squared = 0.0f;
    rms_count = 0;
}

void Get_Vibration_Data(void)
{
    // 直接使用全局vibration_detector结构体中的配置参数
    // 避免重复从Flash读取，提高效率
    // acc_norm已经在adxl357_process_data中计算完成，无需重复计算

    // 更新基本统计信息
    if (vibration_data.vibration < vibration_data.min_vibration)
        vibration_data.min_vibration = vibration_data.vibration;
    if (vibration_data.vibration > vibration_data.max_vibration)
        vibration_data.max_vibration = vibration_data.vibration;

    // 防止account溢出（虽然120秒内不太可能，但作为安全措施）
    if (vibration_data.account < 0xFFFFFFFF) {
        vibration_data.avg_vibration = (vibration_data.avg_vibration * vibration_data.account + vibration_data.vibration) / (vibration_data.account + 1);
        vibration_data.account++;
    }

    // 更新RMS计算（参考vibration_detector.c的滑动窗口RMS计算）
    // 防止rms_count溢出
    if (rms_count < 0xFFFFFFFF) {
        sum_squared += vibration_data.vibration * vibration_data.vibration;
        rms_count++;

        // 计算当前RMS值（使用所有历史数据）
        if (rms_count > 0) {
            vibration_data.current_rms_value = sqrtf(sum_squared / rms_count);
        }
    }

    // 差值检测（参考vibration_detector.c的差值计算）
    float current_delta = fabs(vibration_detector.config.threshold - vibration_data.vibration);
    if (current_delta > vibration_detector.config.delta_threshold) {
        vibration_data.delta_count_total++;
    }

    // 更新差值最大值统计
    if (current_delta > vibration_data.max_delta_value_in_period) {
        vibration_data.max_delta_value_in_period = current_delta;
    }

    // RMS超过阈值检测（参考vibration_detector.c的RMS检测逻辑）
    if (fabs(vibration_data.current_rms_value - vibration_detector.config.threshold) > vibration_detector.config.threshold * vibration_detector.config.rms_threshold) {
        vibration_data.rms_over_count_total++;
    }
}

/**
  *******************************************************************************
  * @Description: 读取ADXL357寄存器
  * @Parameters : reg - 寄存器地址
  * @RetValue   : 寄存器值
  * @Note       : 使用SPI通信读取指定寄存器
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
static uint8_t adxl357_read_reg(uint32_t reg)
{
    uint32_t     cmd_address_data;

    cmd_address_data = (reg << 1 | 1) & 0xff;  // 命令字节（读操作）
    cmd_address_data |= 0x00 << 8;  // 数据字节（发送0x00来读取数据）
    spi0_cs2_transfer((uint8_t*)&cmd_address_data, (uint8_t*)&cmd_address_data, 2);
    return (cmd_address_data >> 8) & 0xff;  // 返回接收到的数据
}

/**
  *******************************************************************************
  * @Description: 写入ADXL357寄存器
  * @Parameters : reg - 寄存器地址, data - 要写入的数据
  * @RetValue   : 0表示成功
  * @Note       : 使用SPI通信写入指定寄存器
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
static int32_t adxl357_write_reg(uint32_t reg, uint32_t data)
{
    uint32_t     cmd_address_data;

    cmd_address_data = (reg << 1) & 0xff;  // 命令字节（写操作）
    cmd_address_data |= (data & 0xff) << 8;  // 数据字节
    spi0_cs2_transfer((uint8_t*)&cmd_address_data, (uint8_t*)&cmd_address_data, 2);

    return 0;
}

/**
  *******************************************************************************
  * @Description: ADXL357中断服务函数
  * @Parameters : int_flag - 中断标志
  * @RetValue   : 无
  * @Note       : 当ADXL357数据就绪时触发，通知任务处理数据
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */


static void adxl357_data_ready_isr(uint32_t int_flag)
{
    // static uint32_t interrupt_count = 0;
    // static uint32_t last_print_count = 0;
    // static uint32_t last_interrupt_time = 0;
    // uint32_t current_time;

    if(get_downhole() == 0)
    {
        xTaskGenericNotifyFromISR(adx357_task_handle, EVENT_ADXL357_DATA_READY, eSetBits, NULL, NULL);
    }

    // 只检查PTE1中断（ADXL357的INT1引脚）
    if(int_flag & (1 << 1) && get_downhole() != 0) {  // PTE1对应位1
        // interrupt_count++;
        // current_time = xTaskGetTickCountFromISR();

        // // 每100次中断打印一次日志（约1秒一次），减少输出频率
        // if(interrupt_count % 100 == 1) {
        //     uint32_t interrupts_since_last = interrupt_count - last_print_count;
        //     uint32_t time_since_last = current_time - last_interrupt_time;
        //     printf("ADXL357 ISR: PTE1 interrupt triggered! INT_FLAG: 0x%08X (Count: %lu, Since last: %lu, Time: %lu ms)\r\n",
        //            int_flag, interrupt_count, interrupts_since_last, time_since_last);
        //     last_print_count = interrupt_count;
        //     last_interrupt_time = current_time;
        // }

        xTaskGenericNotifyFromISR(adx357_task_handle, EVENT_ADXL357_DATA_READY, eSetBits, NULL, NULL);
    }
}

/**
  *******************************************************************************
  * @Description: 验证ADXL357配置并打印详细信息
  * @Parameters : 无
  * @RetValue   : 无
  * @Note       : 读取所有关键寄存器并显示配置状态
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
void adxl357_verify_configuration(void)
{
    /* printf("=== ADXL357 Configuration Verification ===\r\n"); */
    uint8_t power_ctl = adxl357_read_reg(REG_POWER_CTL);
    uint8_t filter = adxl357_read_reg(REG_FILTER);
    uint8_t range = adxl357_read_reg(REG_RANGE);
    uint8_t fifo_samples = adxl357_read_reg(REG_FIFO_SAMPLES);
    uint8_t int_map = adxl357_read_reg(REG_INT_MAP);

    // 根据FILTER寄存器值自动计算FIFO满时间
    // 使用自动化宏获取ODR周期和名称
    uint8_t odr_value = filter & 0x0F;
    const char* odr_name = GET_ODR_NAME(odr_value);

    // 解析工作模式
    const char* power_mode = "Unknown";
    if(power_ctl == POWER_CTL_START) {
        power_mode = "Measurement Mode";
    } else if(power_ctl == POWER_CTL_STOP) {
        power_mode = "Standby Mode";
    }

    // 解析量程设置
    const char* range_name = "Unknown";
    switch(range) {
        case SET_RANGE_10G:
            range_name = "+/-10g";
            break;
        case SET_RANGE_20G:
            range_name = "+/-20g";
            break;
        case SET_RANGE_40G:
            range_name = "+/-40g";
            break;
    }

    /* printf("POWER_CTL: 0x%02X - %s\r\n", power_ctl, power_mode); */
    /* printf("RANGE: 0x%02X - %s\r\n", range, range_name); */
    /* printf("FIFO_SAMPLES: 0x%02X (%d bytes, max 96 bytes)\r\n", fifo_samples, fifo_samples); */

    // 解析中断映射配置
    /* printf("INT_MAP: 0x%02X - ", int_map); */
    if(int_map & INT_MAP_DATA_RDY) {
        /* printf("DATA_READY "); */
    }
    if(int_map & INT_MAP_FIFO_FULL) {
        /* printf("FIFO_FULL "); */
    }
    if(int_map & INT_MAP_FIFO_OVR) {
        /* printf("FIFO_OVR "); */
    }
    if(int_map & INT_MAP_ACT) {
        /* printf("ACT "); */
    }
    if(int_map == 0x00) {
        /* printf("No interrupts mapped"); */
    }
    /* printf("\r\n"); */

    // 计算FIFO满时间：每组数据9字节，FIFO_SAMPLES设置的是字节数
    uint8_t data_groups = fifo_samples / 9;  // 每组数据9字节

    // 转换为毫秒并判断显示格式
    float odr_period_ms_float = (float)GET_ODR_PERIOD(odr_value) / 1000.0f;  // 转换为浮点型毫秒

    if(odr_period_ms_float < 1.0f) {
        // 小于1ms，使用浮点型显示
        float fifo_full_time_ms_float = (float)data_groups * odr_period_ms_float;
        /* printf("ODR Configuration: %s (0x%02X), Period: %.3f ms per data group\r\n", odr_name, filter & 0x0F, odr_period_ms_float); */
        /* printf("FIFO Configuration: %d bytes = %d data groups\r\n", fifo_samples, data_groups); */
        /* printf("Expected FIFO full time: %.2f ms (%d data groups * %.3f ms per group)\r\n",
               fifo_full_time_ms_float, data_groups, odr_period_ms_float); */
    } else {
        // 大于等于1ms，转换为整型显示
        uint32_t odr_period_ms = (uint32_t)odr_period_ms_float;  // 直接转换为整型毫秒
        uint32_t fifo_full_time_ms = data_groups * odr_period_ms;  // 毫秒为单位（整型）
        /* printf("ODR Configuration: %s (0x%02X), Period: %lu ms per data group\r\n", odr_name, filter & 0x0F, odr_period_ms); */
        /* printf("FIFO Configuration: %d bytes = %d data groups\r\n", fifo_samples, data_groups); */
        /* printf("Expected FIFO full time: %lu ms (%d data groups * %lu ms per group)\r\n",
               fifo_full_time_ms, data_groups, odr_period_ms); */
    }
    /* printf("=== Configuration Verification End ===\r\n"); */
}

/**
  *******************************************************************************
  * @Description: 重新配置ADXL357中断设置
  * @Parameters : interrupt_type - 中断类型
  *               INT_MAP_FIFO_FULL: FIFO满中断（井下模式默认）
  *               INT_MAP_DATA_RDY: 数据就绪中断（上位机模式）
  * @RetValue   : 无
  * @Note       : 在中断被禁用后重新启用时调用，确保ADXL357中断配置正确
  *               核心流程：进入待机模式 → 配置中断类型 → 重新进入测量模式
  *
  * 中断类型说明：
  * - INT_MAP_FIFO_FULL: FIFO满中断，适合井下模式，批量读取数据
  * - INT_MAP_DATA_RDY: 数据就绪中断，适合上位机模式，实时读取数据
  *
  * ADXL357待机模式（Standby Mode）说明：
  * - 在待机模式下，设备处于低功耗状态
  * - 温度和加速度数据路径不工作
  * - 数字功能（包括FIFO指针）会重置
  * - 设备的配置设置更改必须在standby=1时进行
  * - 例外：高通滤波器可以在设备运行时更改
  *
  * 重新配置流程的必要性：
  * 1. Flash操作期间，ADXL357可能进入待机模式或中断被禁用
  * 2. 待机模式下FIFO指针会重置，即FIFO数据会被清空
  * 3. 只有重新进入测量模式才能恢复数据采集和中断功能
  *
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
void adxl357_reconfigure_interrupts(uint8_t interrupt_type)
{
    // 1. 进入待机模式，清空FIFO并停止数据采集
    // 注意：ADXL357的配置更改必须在待机模式下进行
    // 待机模式下，FIFO指针会重置，数据路径停止工作
    adxl357_write_reg(REG_POWER_CTL, POWER_CTL_STOP);
    vTaskDelay(10);  // 等待进入待机模式

    // ========================================
    // 若需要修改ADXL357的其他配置，可以在这里添加
    // 例如：
    // - 修改FIFO样本数：adxl357_write_reg(REG_FIFO_SAMPLES, new_value);
    // - 修改中断映射：adxl357_write_reg(REG_INT_MAP, new_value);
    // - 修改滤波器设置：adxl357_write_reg(REG_FILTER, new_value);
    // - 修改其他寄存器配置
    // 注意：所有配置修改必须在待机模式下进行
    // ========================================

    if(interrupt_type == INT_MAP_DATA_RDY) {
        // 上位机模式：配置为数据就绪中断
        // 数据就绪中断：每次有新数据时立即触发中断
        adxl357_write_reg(REG_INT_MAP, INT_MAP_DATA_RDY);
        // printf("ADXL357: Configured for DATA_RDY interrupt (host mode)\r\n");
    } else {
        // 井下模式：使用默认的FIFO满中断（无需修改）
        // FIFO满中断：FIFO满时触发中断，批量读取数据
        // printf("ADXL357: Using default FIFO_FULL interrupt (downhole mode)\r\n");
    }

    // 2. 重新进入测量模式 - 这是关键步骤！
    // 只有重新进入测量模式才能恢复数据采集和中断功能
    // 测量模式下，ADXL357开始采集数据并填充FIFO
    adxl357_write_reg(REG_POWER_CTL, POWER_CTL_START);
    vTaskDelay(10);  // 等待进入测量模式

    // 3. 按照数据手册建议：主动等待10ms以上
    // 数据手册建议在重新进入测量模式后等待至少10ms
    // 根据实际测试，可能需要更长的等待时间
    // printf("ADXL357: Waiting 20ms for sensor stabilization (extended wait)...\r\n");
    vTaskDelay(20);  // 等待20ms，给ADXL357更多时间稳定

    // 4. 设置FIFO清空标志，在中断任务中执行清空操作
    // 避免在重新配置过程中与中断任务产生冲突
    adxl357_recently_reconfigured = true;
    adxl357_fifo_clear_count = 0;  // 重置FIFO清空计数
    // printf("ADXL357: FIFO clearing will be performed in interrupt task...\r\n");

    // 5. 验证重新配置后的设置
    // 使用现有的验证函数，避免代码重复
    // adxl357_verify_configuration();
    // printf("ADXL357: Reconfiguration completed, sensor ready for data collection.\r\n");
}

/**
  *******************************************************************************
  * @Description: 上位机模式下的ADXL357中断控制函数
  * @Parameters : enable - true启用中断，false禁用中断
  * @RetValue   : 无
  * @Note       : 仅用于上位机模式，只控制ADXL357的中断
  *               上位机模式使用数据就绪中断，避免与Flash操作冲突
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
void adxl357_host_mode_interrupt_control(bool enable)
{
    if(enable) {
        // 上位机模式：先配置GPIO引脚，再配置ADXL357为数据就绪中断
        PINS_DRV_SetPinIntSel(PORTE, 1, PORT_INT_FALLING_EDGE);  // PTE1 - ADXL357
        adxl357_reconfigure_interrupts(INT_MAP_DATA_RDY);
        // printf("ADXL357 Host Mode: Configured GPIO and DATA_RDY interrupt...\r\n");
    } else {
        // 上位机模式：禁用ADXL357中断
        PINS_DRV_SetPinIntSel(PORTE, 1, PORT_DMA_INT_DISABLED);  // PTE1 - ADXL357
        // printf("ADXL357 Host Mode: Disabling interrupt...\r\n");
    }
}

/**
  *******************************************************************************
  * @Description: ADXL357初始化函数
  * @Parameters : 无
  * @RetValue   : 0表示成功，-1表示失败
  * @Note       : 配置ADXL357为±10g量程，1000Hz采样率，FIFO中断模式
  *
  * ADXL357初始化流程说明：
  * 1. 设备检测：验证ADXL357是否正确连接
  * 2. 设备复位：清除所有寄存器配置
  * 3. 进入待机模式：所有配置更改必须在待机模式下进行
  * 4. 配置参数：量程、采样率、FIFO、中断等
  * 5. 进入测量模式：开始数据采集和中断功能
  *
  * 待机模式的重要性：
  * - ADXL357的配置更改必须在待机模式下进行（standby=1）
  * - 待机模式下，FIFO指针会重置，数据路径停止工作
  * - 只有重新进入测量模式才能恢复数据采集功能
  * - 例外：高通滤波器可以在设备运行时更改
  *
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
void adxl357_init(void)
{
    // 1. 设备检测
    if(adxl357_read_reg(REG_DEVID_AD) == 0XAD)
    {
        uint8_t devid, partid, revid;

        devid = adxl357_read_reg(REG_DEVID_MST);
        partid = adxl357_read_reg(REG_PARTID);
        revid = adxl357_read_reg(REG_REVID);
        printf("Found ADXL357! MEMS_ID:0x%02x DEV_ID:0x%02x  REV_ID:0x%02x\r\n", devid, partid, revid);
    }
    else
    {
        printf("Can NOT find ADXL357! \r\n");
        return;
    }

    // 2. 设备复位
    // 复位后所有寄存器恢复默认值，FIFO指针重置
    adxl357_write_reg(REG_RESET, 0x52);
    vTaskDelay(10);

    // 3. 进入待机模式进行配置
    // 重要：ADXL357的配置更改必须在待机模式下进行
    // 待机模式下，数据路径停止工作，FIFO指针重置
    adxl357_write_reg(REG_POWER_CTL, POWER_CTL_STOP);
    vTaskDelay(10);

    // 4. 配置量程为±10g
    // 在待机模式下配置量程，确保配置生效
    adxl357_write_reg(REG_RANGE, SET_RANGE_10G);
    vTaskDelay(10);

    // 5. 配置滤波器设置寄存器（使用统一的采样频率配置）
    // 设置ODR（输出数据速率）和高通滤波器
    adxl357_write_reg(REG_FILTER, ADXL357_SAMPLE_RATE_ODR | (SET_HPF_OFF << 4));
    vTaskDelay(10);

    // 6. 配置FIFO样本数（Flash操作友好配置）
    // 90字节 = 10组完整的三轴数据，配合1000Hz ODR，提供10ms的响应时间
    // 这个配置提供了较高的数据更新频率，同时保持Flash操作友好
    adxl357_write_reg(REG_FIFO_SAMPLES, ADXL357_SAMPLES_PER_READ);
    vTaskDelay(10);

    // 7. 注册中断回调函数
    gpio_porte_register_cb(adxl357_data_ready_isr);
    vTaskDelay(10);

    // 8. 配置INT1中断映射
    adxl357_write_reg(REG_INT_MAP, 0x00);  // 先禁用所有中断
    vTaskDelay(10);
    // 选择中断类型：
    // INT_MAP_FIFO_FULL - 只在FIFO满时触发（推荐用于批量读取）
    // INT_MAP_DATA_RDY  - 每次有新数据时触发（适合实时读取）
    adxl357_write_reg(REG_INT_MAP, INT_MAP_FIFO_FULL);  // 使用FIFO_FULL中断
    vTaskDelay(10);

    // 9. 进入测量模式
    // 重要：只有进入测量模式才能开始数据采集和中断功能
    // 测量模式下，ADXL357开始采集数据并填充FIFO
    adxl357_write_reg(REG_POWER_CTL, POWER_CTL_START);
    vTaskDelay(10);

    // 10. 初始化g_norm（根据量程设置）
    // 对于±10g量程，g_norm = 2^19 / 10 = 52428.8
    // g_norm = 52428.8f; // 已删除，使用vibration_detector中的norm配置
    printf("ADXL357: Using vibration_detector.config.norm for acceleration conversion\r\n");

    // 11. 验证最终配置
    adxl357_verify_configuration();

    printf("ADXL357 configured and started successfully!\r\n");
}

/**
  *******************************************************************************
  * @Description: 通过FIFO方式读取ADXL357三轴加速度数据
  * @Parameters : 无
  * @RetValue   : 无
  * @Note       : 只负责从FIFO中读取数据到缓冲区
  *               数据处理将在VibrationMonitor_Task中进行
  *               这样可以大大减少中断处理时间
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
void adxl357_process_data(void)
{
    // 0. 检查是否需要清空FIFO（在重新配置后的前几次中断中）
    if(adxl357_recently_reconfigured && adxl357_fifo_clear_count < adxl357_fifo_clear_limit) {
        adxl357_fifo_clear_count++;
        // printf("ADXL357: Clearing FIFO buffer in interrupt task (attempt %lu/%lu)...\r\n",
            //    adxl357_fifo_clear_count, adxl357_fifo_clear_limit);

        // 清空FIFO中的所有数据
        uint8_t entries = adxl357_read_reg(REG_FIFO_ENTRIES) & 0x7F;
        if(entries > 0) {
            // printf("ADXL357: Clearing %d bytes from FIFO\r\n", entries);

            // 读取并丢弃FIFO中的所有数据
            uint8_t tx_buffer[entries + 1];
            uint8_t rx_buffer[entries + 1];

            tx_buffer[0] = REG_FIFO_DATA << 1 | 1;  // 读取FIFO数据寄存器命令
            for(uint8_t i = 1; i <= entries; i++) {
                tx_buffer[i] = 0x00;  // 发送0x00来读取数据
            }

            spi0_cs2_transfer(tx_buffer, rx_buffer, entries + 1);
            vTaskDelay(1);  // 短暂等待，确保FIFO指针更新
            // printf("ADXL357: FIFO cleared successfully\r\n");
        } else {
            // printf("ADXL357: FIFO was already empty\r\n");
        }

        // 检查是否完成所有清空操作
        if(adxl357_fifo_clear_count >= adxl357_fifo_clear_limit) {
            adxl357_recently_reconfigured = false;  // 清除重新配置标志
            // printf("ADXL357: FIFO clearing completed, returning to normal operation\r\n");
        }

        return;  // 清空完成后直接返回，不处理数据
    }

    // 1. 读取FIFO条目数
    uint8_t entries = adxl357_read_reg(REG_FIFO_ENTRIES) & 0x7F;

    if(entries == 0) {
        // printf("FIFO is empty, no data to process (Entries: %d)\r\n", entries);
        return;  // 如果FIFO为空，直接返回
    }

    // printf("FIFO entries (samples): %d\r\n", entries);

    if(entries >= 9) {  // 确保至少有9字节（一个完整样本）
        // 2. 计算完整样本的字节数
        uint8_t complete_bytes = (entries / 9) * 9;

        // 3. 准备SPI传输命令和接收缓冲区
        uint8_t tx_buffer[complete_bytes + 1];  // 发送缓冲区
        uint8_t rx_buffer[complete_bytes + 1];  // 接收缓冲区

        tx_buffer[0] = REG_FIFO_DATA << 1 | 1;  // 读取FIFO数据寄存器命令
        for(uint8_t i = 1; i <= complete_bytes; i++) {
            tx_buffer[i] = 0x00;  // 发送0x00来读取数据
        }

        // 测量SPI传输时间
        // uint32_t spi_start_time = xTaskGetTickCount();

        // 4. 执行SPI传输 - 使用独立的发送和接收缓冲区
        spi0_cs2_transfer(tx_buffer, rx_buffer, complete_bytes + 1);

        // uint32_t spi_end_time = xTaskGetTickCount();
        // uint32_t spi_transfer_time_ms = (spi_end_time - spi_start_time) * portTICK_PERIOD_MS;

        // 打印SPI传输时间
        // printf("ADXL357 SPI: Transferred %d bytes in %lu ms\r\n", complete_bytes + 1, spi_transfer_time_ms);

        // 5. 将接收到的数据复制到当前写入缓冲区
        memcpy(adxl357_raw_buffer[adxl357_current_buffer], rx_buffer, complete_bytes + 1);

        // 6. 切换缓冲区并设置数据就绪标志
        adxl357_process_buffer = adxl357_current_buffer;
        adxl357_current_buffer = 1 - adxl357_current_buffer;  // 在0和1之间切换

        adxl357_data_ready = 1; // 设置数据就绪标志
        adxl357_buffer_entries = complete_bytes;

        // 打印井下模式下的缓冲区信息
        // printf("ADXL357 Downhole: Buffer[%d] data (bytes: %d)\r\n", adxl357_process_buffer, complete_bytes);

        // 7. 数据已读取到缓冲区，等待VibrationMonitor_Task处理
        // 数据处理将在VibrationMonitor_Task中进行
    }
}

/**
  *******************************************************************************
  * @Description: 通过直接读取寄存器方式获取ADXL357三轴加速度数据
  * @Parameters : 无
  * @RetValue   : 无
  * @Note       : 从TEMP2寄存器开始连续读取12字节数据，包含三轴加速度数据
  *               这是方式一：直接读取寄存器方式，适合单次数据获取
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
static void on_adxl357_data_ready_event(void)
{
    static uint8_t sample_count = 0;  // 累积的样本计数
    uint8_t         transfer_buffer[12];

    // 从TEMP2寄存器开始读取12字节数据
    transfer_buffer[0] = REG_TEMP2 << 1 | 1;  // 读取命令
    spi0_cs2_transfer((uint8_t*)transfer_buffer, (uint8_t*)transfer_buffer, 12);

    // 计算当前样本在缓冲区中的起始位置
    // 格式：命令字节 + 多个样本（每个样本9字节：X轴3字节 + Y轴3字节 + Z轴3字节）
    uint16_t sample_start = 1 + (sample_count * 9);  // 跳过命令字节，每个样本9字节

    // 存储当前样本的三轴数据到当前写入缓冲区
    // X轴数据（字节3-5）
    adxl357_raw_buffer[adxl357_current_buffer][sample_start] = transfer_buffer[3];
    adxl357_raw_buffer[adxl357_current_buffer][sample_start + 1] = transfer_buffer[4];
    adxl357_raw_buffer[adxl357_current_buffer][sample_start + 2] = transfer_buffer[5];

    // Y轴数据（字节6-8）
    adxl357_raw_buffer[adxl357_current_buffer][sample_start + 3] = transfer_buffer[6];
    adxl357_raw_buffer[adxl357_current_buffer][sample_start + 4] = transfer_buffer[7];
    adxl357_raw_buffer[adxl357_current_buffer][sample_start + 5] = transfer_buffer[8];

    // Z轴数据（字节9-11）
    adxl357_raw_buffer[adxl357_current_buffer][sample_start + 6] = transfer_buffer[9];
    adxl357_raw_buffer[adxl357_current_buffer][sample_start + 7] = transfer_buffer[10];
    adxl357_raw_buffer[adxl357_current_buffer][sample_start + 8] = transfer_buffer[11];

    sample_count++;

    // 当累积了足够的样本时，切换缓冲区并设置数据就绪标志
    if(sample_count >= ADXL357_DATA_GROUPS_PER_READ) {
        // 设置命令字节
        adxl357_raw_buffer[adxl357_current_buffer][0] = REG_FIFO_DATA << 1 | 1;

        // 切换到另一个缓冲区
        adxl357_process_buffer = adxl357_current_buffer;
        adxl357_current_buffer = 1 - adxl357_current_buffer;  // 在0和1之间切换

        adxl357_data_ready = 1;
        adxl357_buffer_entries = sample_count * 9;  // 累积的样本数 * 9字节

        // 打印存储在数组中的数据（只打印第一组）
        // printf("ADXL357: Buffer[%d] data (samples: %d, bytes: %d):\r\n", adxl357_process_buffer, sample_count, adxl357_buffer_entries);
        // printf("Command: 0x%02X\r\n", adxl357_raw_buffer[adxl357_process_buffer][0]);
        // if(sample_count > 0) {
        //     uint16_t sample_start = 1;  // 第一组数据的起始位置
        //     printf("Sample[0]: X=0x%02X%02X%02X Y=0x%02X%02X%02X Z=0x%02X%02X%02X\r\n",
        //            adxl357_raw_buffer[adxl357_process_buffer][sample_start], adxl357_raw_buffer[adxl357_process_buffer][sample_start+1], adxl357_raw_buffer[adxl357_process_buffer][sample_start+2],
        //            adxl357_raw_buffer[adxl357_process_buffer][sample_start+3], adxl357_raw_buffer[adxl357_process_buffer][sample_start+4], adxl357_raw_buffer[adxl357_process_buffer][sample_start+5],
        //            adxl357_raw_buffer[adxl357_process_buffer][sample_start+6], adxl357_raw_buffer[adxl357_process_buffer][sample_start+7], adxl357_raw_buffer[adxl357_process_buffer][sample_start+8]);
        // }

        sample_count = 0;  // 重置样本计数，准备下一轮累积
    }

}

/**
  *******************************************************************************
  * @Description: ADXL357任务函数
  * @Parameters : p - 任务参数（未使用）
  * @RetValue   : 无
  * @Note       : 轮询模式，等待中断通知后读取数据
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
void adxl357_task(void* p)
{
    adxl357_init();
    Reset_Vibration_Stats();

    // 模式检测和配置
    static bool adxl357_host_mode_configured = false;

    for(;;)
    {
        uint32_t notify;

        // 检测是否进入井上模式并配置ADXL357
        if(get_downhole() == 0 && !adxl357_host_mode_configured) {
            // 井上模式，配置为数据就绪中断
            printf("ADXL357 Task: Host mode detected, configuring DATA_RDY interrupt...\r\n");
            adxl357_host_mode_interrupt_control(false);
            adxl357_host_mode_interrupt_control(true);
            adxl357_host_mode_configured = true;
        }
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if(notify & EVENT_ADXL357_DATA_READY)
        {
            // 记录任务开始处理时间
            // uint32_t task_start_time = xTaskGetTickCount();

            // 根据当前模式选择不同的处理方式
            if(adxl357_host_mode_configured) {
                // 井上模式：数据就绪中断，直接处理单次数据
                on_adxl357_data_ready_event();
            } else {
                // 井下模式：FIFO满中断，使用FIFO方式批量读取数据
                adxl357_process_data();
            }

            // 计算任务处理时间
            // uint32_t task_end_time = xTaskGetTickCount();
            // uint32_t task_processing_time_ms = (task_end_time - task_start_time) * portTICK_PERIOD_MS;

            // 每次处理都打印任务处理时间
            // static uint32_t total_process_count = 0;
            // total_process_count++;
            // printf("ADXL357 Task[%lu]: Task processing time: %lu ms\r\n",
            //        total_process_count, task_processing_time_ms);
        }
    }
}
//START_TASK(adxl357_task, "adxl357_task", 256, NULL, TASK_PRIORITY_ADXL357, &adx357_task_handle);
