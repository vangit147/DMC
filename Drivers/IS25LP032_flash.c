/**
 *****************************************************************************
 * FILENAME   : IS25LP032_flash.c
 * COPYRIGHT  : Moding Tech(ShangHai) Co.,Ltd2023.
 * CREATEDDATE: 2023.10.04 09:57:51 Wednesday
 * DESCRIPTION:
 *
 *
 * EDIT HISTORY:
 *   NAME                        DATE                      CONTENT
 * YangHaifeng               2023.10.04                    CREATE
 *****************************************************************************
 */
/************* Included files, Macros, Various and Declarations ***************/
#include "IS25LP032_flash.h"

#define FLASH_PP 0X02   //页编程
#define FLASH_RDSR 0X05 //读状态寄存器
#define FLASH_WREN 0X06 //写使能
#define FLASH_NORD 0X03 //读数据
#define FLASH_RDMDID 0X90 //读制造商和设备ID
#define FLASH_SER 0XD7 //串行
/**
 * Flash临时缓冲区大小定义
 *
 * 重要说明：
 * 1. 此值必须足够大，能够容纳最大的结构体 DEV_CFG_T (1024字节)
 * 2. 由于 flash_temp_buffer 是 uint64_t 数组，实际字节大小 = 256 * 8 = 2048字节
 * 3. 最小安全值计算：1024字节 ÷ 8字节 = 128，因此当前设置256是安全的
 *
 * 结构体大小参考：
 * - DEV_CFG_T: 1024字节 (最大，决定最小安全值)
 * - total_runtime_record_t: 32字节
 * - log_context_t: 16字节
 *
 * 注意：如果修改此值，必须确保 >= 128，否则设备配置读取将失效
 */
#define FLASH_TEMP_BUFFER_SIZE 256

static uint64_t  flash_temp_buffer[FLASH_TEMP_BUFFER_SIZE];    //2K读写缓冲区 (从512减少到256)

/*
FLASH 数据区定义 (总容量: 4MB = 4,194,304字节)
+--------------------------+
|       FLASH_INIT   (4K)  | 0x00000000 - 0x00000FFF
+--------------------------+
|       DEV_CFG      (4K)  | 0x00001000 - 0x00001FFF
+--------------------------+
|     TOTAL_RUN_TIME (8K)  | 0x00002000 - 0x00003FFF
+--------------------------+
|     LOG CONTEXT    (8K)  | 0x00004000 - 0x00005FFF
+--------------------------+
|          LOG             | 0x00006000 - 0x3FFFFFFF (约33,828条日志，每条124字节)
+--------------------------+

*/
#define FLASH_INITED_FLAG 0XABEEBEE9
#define FLASH_TOTAL_SIZE (4 * 1024 * 1024)
// CFG
#define DEVICE_CFG_FLAG 0XBEE1BEEE
#define DEVICE_CFG_FLASH_ADDRESS 0X1000
#define DEVICE_CFG_FLASH_SIZE 0X1000
#define DEVICE_CFG_SIZE 0X400
typedef __packed struct // 占用1022字节，最后2字节为CRC16校验和
{
    // 基础字段 (16字节)
    uint32_t device_cfg_tag;       // 标志
    uint32_t device_cfg_sn;        // 每更新一次，该值加1
    uint32_t device_cfg_reserved0; // 保留后续使用
    uint32_t device_cfg_reserved1; // 保留后续使用

    // 配置字段 (4字节)
    uint16_t log_saved_period;      // LOG保存周期，秒
    uint8_t acc_sensor_type;       // 1字节
    uint8_t gyro_sensor_type;      // 1字节

    // 加速度计装配误差和虚拟半径限制 (24字节)
    float offset[2];               // 8字节 (2 * 4)
    double xr_limit;               // 8字节
    double yr_limit;               // 8字节

    // 陀螺仪零偏 (13字节)与补偿函数阶数
    float gx_bias;                 // 4字节
    float gy_bias;                 // 4字节
    float gz_bias;                 // 4字节
    int8_t degree;                 // 1字节

    // 加速度计多项式拟合系数 (144字节)
    double axB0, axB1, axB2, axB3, axB4, axB5; // 48字节 (6 * 8)
    double ayB0, ayB1, ayB2, ayB3, ayB4, ayB5; // 48字节 (6 * 8)
    double azB0, azB1, azB2, azB3, azB4, azB5; // 48字节 (6 * 8)

    // 陀螺仪多项式拟合系数 (144字节)
    double gxB0, gxB1, gxB2, gxB3, gxB4, gxB5; // 48字节 (6 * 8)
    double gyB0, gyB1, gyB2, gyB3, gyB4, gyB5; // 48字节 (6 * 8)
    double gzB0, gzB1, gzB2, gzB3, gzB4, gzB5; // 48字节 (6 * 8)

    // 加速度计零偏 (12字节)
    float bx, by, bz;             // 12字节 (3 * 4)

    // MS矩阵系数 (48字节)
    double mxx, mxy, mxz, myy, myz, mzz; // 48字节 (6 * 8)

    // 数组字段 (107字节)
    float px[5];                   // 20字节 (5 * 4)
    float py[5];                   // 20字节 (5 * 4)
    float pz[5];                   // 20字节 (5 * 4)
    char pro_id[15];               // 15字节
    char version[VERSION_MAX_LENGTH]; // 32字节 (VERSION_MAX_LENGTH = 32)

    // 温度补偿设置 (16字节)
    float temp_comp_lower_limit;   // 4字节
    float temp_comp_upper_limit;   // 4字节
    float t_scale;                 // 4字节
    float t_intercept;             // 4字节

    // 脉冲传输配置(12字节)
    uint16_t pump_delay1;  										//停泵状态下的静态井斜前置延迟时间,单位秒,默认为10秒
    uint16_t pump_delay2;	 									//停泵状态下的静态井斜前置延迟时间,单位秒,默认为10秒
    uint32_t pulse_retry_for_pump_off_data; 			        //开泵后对停泵状态下静态井斜的重传次数
    uint32_t pulse_interval_for_pump_off_data; 		            //开泵后对停泵状态下静态井斜的重传时间间隔，秒
    uint32_t pulse_interval;									//正常开泵情况下的泥浆脉冲时间传输时间间隔，秒

    // 其他系统参数 (8字节)
    uint32_t idle_hook_enable;     // 4字节
    float calibration_data;        // 4字节
} CFG_T;

// 默认参数：硬编码的默认配置值
static const CFG_T default_cfg = {
    .device_cfg_tag = DEVICE_CFG_FLAG,
    .device_cfg_sn = 1,
    .device_cfg_reserved0 = 0,
    .device_cfg_reserved1 = 0,
    // 配置字段
    .log_saved_period = 60,
    .acc_sensor_type = 1,  // 默认使用VS1005加速度计
    .gyro_sensor_type = 0, // 默认使用IAM-20680HT陀螺仪

    .offset = {0.0f, 0.0f},
    .xr_limit = 0.01f,
    .yr_limit = 0.001f,
    .gx_bias = 0.0f,
    .gy_bias = 0.0f,
    .gz_bias = 0.0f,
    .degree = 0,
    // 加速度计多项式拟合系数
    .axB0 = 0.0, .axB1 = 0.0, .axB2 = 0.0, .axB3 = 0.0, .axB4 = 0.0, .axB5 = 0.0,
    .ayB0 = 0.0, .ayB1 = 0.0, .ayB2 = 0.0, .ayB3 = 0.0, .ayB4 = 0.0, .ayB5 = 0.0,
    .azB0 = 0.0, .azB1 = 0.0, .azB2 = 0.0, .azB3 = 0.0, .azB4 = 0.0, .azB5 = 0.0,
    // 陀螺仪多项式拟合系数
    .gxB0 = 0.0, .gxB1 = 0.0, .gxB2 = 0.0, .gxB3 = 0.0, .gxB4 = 0.0, .gxB5 = 0.0,
    .gyB0 = 0.0, .gyB1 = 0.0, .gyB2 = 0.0, .gyB3 = 0.0, .gyB4 = 0.0, .gyB5 = 0.0,
    .gzB0 = 0.0, .gzB1 = 0.0, .gzB2 = 0.0, .gzB3 = 0.0, .gzB4 = 0.0, .gzB5 = 0.0,
    // x、y、z加速度计零偏
    .bx = 0.0f,
    .by = 0.0f,
    .bz = 0.0f,
    // MS矩阵系数
    .mxx = 1.0f,
    .mxy = 0.0f,
    .mxz = 0.0f,
    .myy = 1.0f,
    .myz = 0.0f,
    .mzz = 1.0f,
    // px、py、pz数组暂时不用
    .px = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .py = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pz = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pro_id = {0},
    .version = {0},  // 版本字段设为空，版本信息总是从TGDMC宏获取
    /* 温度补偿范围设置 */
    .temp_comp_lower_limit = -20.0f, // 温度补偿下限，默认-20°C
    .temp_comp_upper_limit = 150.0f, // 温度补偿上限，默认150°C
    .t_scale = -0.000164327854f,
    .t_intercept = 321.9705002,
    /* 泥浆脉冲设置 */
    .pump_delay1 = 8,
    .pump_delay2 = 12,
    .pulse_retry_for_pump_off_data = 3,
    .pulse_interval_for_pump_off_data = 1200,
    .pulse_interval = 3600,
    /* 其他系统参数 */
    .idle_hook_enable = 0,      // 低功耗状态
    .calibration_data = 0.0f    // 校准数据变量
};

typedef __packed union
{
    CFG_T cfg;
    uint8_t reserved[1024 - 2];
} U_CFG_T;

typedef __packed struct
{
    U_CFG_T u_cfg;
    uint16_t crc16;
} DEV_CFG_T;

static DEV_CFG_T dev_cfg;
static int8_t dev_cfg_index;

/*总共运行时间*/
#define TOTAL_RUNTIME_FLAG 0xA000B000
#define TOTAL_RUNTIME_FLASH_ADDRESS (DEVICE_CFG_FLASH_ADDRESS + DEVICE_CFG_FLASH_SIZE)
#define TOTAL_RUNTIME_FLASH_SIZE 0X2000
#define TOTAL_RUNTIME_ITEMS 10
typedef __packed struct
{
    uint32_t magic;
    uint32_t sn;
    uint16_t total_times[TOTAL_RUNTIME_ITEMS];
    uint16_t reserved;
    uint16_t crc16;
} total_runtime_record_t;
static total_runtime_record_t total_run_time;
static total_runtime_record_t *last_run_time_flash_address;

// LOG上下文
#define LOG_CONTEXT_TAG 0XBEE1
#define LOG_CONTEXT_FLASH_ADDRESS (TOTAL_RUNTIME_FLASH_ADDRESS + TOTAL_RUNTIME_FLASH_SIZE)
#define LOG_CONTEXT_FLASH_SIZE 0X2000 // 占用两个扇区
typedef __packed struct _log_context
{
    /*0*/
    uint32_t log_context_sn; // 32位数，SN不会重复

    /*4*/
    uint16_t log_total_count;
    uint16_t log_to_be_read_count;

    /*8*/
    int32_t log_write_index;

    /*8*/
    int32_t log_write_total;

    /*8*/
    int32_t log_write_last_total;
    /*12*/
    uint16_t log_tag;
    uint16_t crc16;
} log_context_t;
static log_context_t log_context;
static int32_t log_context_index;

#define LOG_FLASH_ADDRESS (LOG_CONTEXT_FLASH_ADDRESS + LOG_CONTEXT_FLASH_SIZE)
#define LOG_FLASH_SIZE (FLASH_TOTAL_SIZE - LOG_FLASH_ADDRESS)

static int32_t is25pl032_flash_save_runtime_record(total_runtime_record_t *record);
/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:日志上下文更新
  * @Parameters :
  * @RetValue   :0表示成功，-1表示失败
  * @Note       :将内存中的日志上下文信息保存到Flash存储器中

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.10.14 23:59:05 Saturday
  *******************************************************************************
  */
static int32_t is25pl032_flash_update_context(void)
{
    int32_t i;

    log_context.log_to_be_read_count = log_context.log_total_count;
    for (i = 0; i < 3; i++)
    {
        uint32_t flash_wr_address = LOG_CONTEXT_FLASH_ADDRESS;
        log_context_t rd_back;

        log_context.log_context_sn++;
        log_context.crc16 = CRC16(&log_context, sizeof(log_context_t) - 2);

        log_context_index++;
        if (log_context_index >= LOG_CONTEXT_FLASH_SIZE / sizeof(log_context_t))
            log_context_index = 0;
        flash_wr_address += log_context_index * sizeof(log_context_t);

        is25pl032_flash_normal_write(flash_wr_address, (uint8_t *)&log_context, sizeof(log_context_t));

        // READ BACK
        is25pl032_flash_normal_read(flash_wr_address, (uint8_t *)&rd_back, sizeof(log_context_t));
        if (memcmp(&rd_back, &log_context, sizeof(log_context_t)) == 0)
            break;
    }
    if (i == 3)
        return -1;

    return 0;
}
/**
  *******************************************************************************
  * @Description:重置日志读取索引
  * @Parameters :
  * @RetValue   :
  * @Note       :将日志读取位置重置到最新记录的位置

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.10.15 00:02:01 Sunday
  *******************************************************************************
  */
int32_t is25pl032_flash_reset_rd_index(void)
{
    log_context.log_to_be_read_count = log_context.log_total_count;
    log_context.log_write_total = log_context.log_write_last_total;
    return is25pl032_flash_update_context();
}
/**
  *******************************************************************************
  * @Description:删除所有日志记录
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.10.15 00:02:42 Sunday
  *******************************************************************************
  */
int32_t is25pl032_flash_delete_all_log(void)
{
    log_context.log_total_count = 0;
    log_context.log_to_be_read_count = 0;
    log_context.log_write_total = 0;
    log_context.log_write_last_total = 0;
    return is25pl032_flash_update_context();
}

/**
  *******************************************************************************
  * @Description:读取单条日志记录
  * @Parameters :
  * @RetValue   :
  * @Note       :从Flash存储器中按顺序读取日志数据

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 22:35:19 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_read_one_log(log_t *log)
{
    while (log_context.log_to_be_read_count)
    {
        uint32_t flash_address = LOG_FLASH_ADDRESS;
        int32_t rd_index;

        rd_index = log_context.log_write_index - log_context.log_write_total;
        log_context.log_write_total--;
        // 循环缓冲区：如果读取索引为负，需要加上最大容量 (约33,628条日志)
        if (rd_index < 0)
            rd_index += LOG_FLASH_SIZE / sizeof(log_t);
        flash_address += sizeof(log_t) * rd_index;
        is25pl032_flash_normal_read(flash_address, (uint8_t *)log, sizeof(log_t));

        for (int32_t i = 0; i < 3; i++)
        {
            uint32_t flash_wr_address = LOG_CONTEXT_FLASH_ADDRESS;
            log_context_t rd_back;

            log_context.log_context_sn++;
            log_context.crc16 = CRC16(&log_context, sizeof(log_context_t) - 2);

            log_context_index++;
            if (log_context_index >= LOG_CONTEXT_FLASH_SIZE / sizeof(log_context_t))
                log_context_index = 0;
            flash_wr_address += log_context_index * sizeof(log_context_t);

            if ((flash_wr_address & 0xfff) == 0) // 4K扇区边界
                is25pl032_flash_erase_sector(flash_wr_address, 0);
            is25pl032_flash_normal_write(flash_wr_address, (uint8_t *)&log_context, sizeof(log_context_t));

            // READ BACK
            is25pl032_flash_normal_read(flash_wr_address, (uint8_t *)&rd_back, sizeof(log_context_t));
            if (memcmp(&rd_back, &log_context, sizeof(log_context_t)) == 0)
                break;
        }

        if (log->crc16 == CRC16(log, sizeof(log_t) - 2))
        {
            log_context.log_to_be_read_count--;
            return 1;
        }
        else
            printf("Reading ONE log failed! log->crc16=%d,CRC16(log, sizeof(log_t) - 2)=%d,rd_index=%d flash_address=0x%08x\r\n", log->crc16, CRC16(log, sizeof(log_t) - 2), rd_index, flash_address);
    }
    return 0;
}
/**
  *******************************************************************************
  * @Description: 写入一条日志到Flash存储器
  * @Parameters : log - 指向要写入的日志数据结构的指针
  * @RetValue   : 0-成功，-1-日志写入失败，-2-日志上下文更新失败，-3-FLASH_INITED_FLAG恢复失败
  * @Note       : 该函数会将日志数据写入Flash，更新日志上下文信息，并确保Flash初始化标志正确
  *               如果任何步骤失败，会进行最多3次重试。日志数据采用循环缓冲区存储方式。
  *
  *               Flash存储布局说明：
  *               +--------------------------+
  *               |       FLASH_INIT   (4K)  | 0x00000000 - 0x00000FFF
  *               +--------------------------+
  *               |       DEV_CFG      (4K)  | 0x00001000 - 0x00001FFF
  *               +--------------------------+
  *               |     TOTAL_RUN_TIME (8K)  | 0x00002000 - 0x00003FFF
  *               +--------------------------+
  *               |     LOG CONTEXT    (8K)  | 0x00004000 - 0x00005FFF
  *               +--------------------------+
  *               |          LOG             | 0x00006000 - 0x3FFFFFFF
  *               +--------------------------+
  *
   *               关键范围值含义：
 *               - LOG_FLASH_ADDRESS: 0x00006000 (日志数据起始地址)
 *               - LOG_FLASH_SIZE: 0x3FA00000 (4,169,728字节，实际日志存储空间)
 *               - LOG_FLASH_SIZE / sizeof(log_t): 约33,628条日志记录 (每条日志124字节)
 *               - 日志结构体详细组成：
 *                 * 基础信息: 8字节 (时间戳、保留字段)
 *                 * 传感器数据: 24字节 (三轴加速度、三轴角速度)
 *                 * 倾角数据: 36字节 (倾角1/2/6的最大/最小/平均值)
 *                 * 虚拟半径数据: 16字节 (X/Y半径最小/最大值)
 *                 * 系统状态数据: 20字节 (高边、Z轴角速度统计、温度)
 *                 * 统计计算和电源数据: 20字节 (标准差、标志位、电池电压、CRC16)
 *               - 日志采用循环缓冲区，当写满后覆盖最早的记录

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 19:56:44 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_write_one_log(log_t *log)
{
    int32_t i;
    uint32_t init_flag;
    int32_t ret = 0;

    // Flash操作前禁用传感器中断
    flash_control_sensor_interrupts(false);

    // 增加延时确保中断完全禁用
    vTaskDelay(1);

    log->crc16 = CRC16(log, sizeof(log_t) - 2); // 计算日志数据CRC16校验码，排除最后两字节的CRC字段本身
    for (i = 0; i < 3; i++)
    {
        uint32_t flash_wr_address = LOG_FLASH_ADDRESS;
        log_t log_rd;

        log_context.log_write_index++;
        if (log_context.log_write_index >= LOG_FLASH_SIZE / sizeof(log_t)) //循环写入
            log_context.log_write_index = 0;
        log_context.log_write_total++;
        if (log_context.log_write_total >= LOG_FLASH_SIZE / sizeof(log_t))
            log_context.log_write_total = LOG_FLASH_SIZE / sizeof(log_t);
        log_context.log_write_last_total = log_context.log_write_total;
        flash_wr_address += log_context.log_write_index * sizeof(log_t);
        is25pl032_flash_normal_write(flash_wr_address, (uint8_t *)log, sizeof(log_t));

        // READ BACK
        is25pl032_flash_normal_read(flash_wr_address, (uint8_t *)&log_rd, sizeof(log_t));
        if (memcmp(&log_rd, log, sizeof(log_t)) == 0)
            break;
        else
            printf("Writing ONE log failed! log_context.log_write_index=%d flash_wr_address=0x%08x\r\n", log_context.log_write_index, flash_wr_address);
    }

    if (i == 3)
    {
        // Flash操作后重新启用传感器中断
        flash_control_sensor_interrupts(true);
        return -1;
    }

    // 成功写入一条log
    log_context.log_total_count++;
    // 限制总日志数量不超过最大容量 (约33,628条)
    if (log_context.log_total_count > LOG_FLASH_SIZE / sizeof(log_t))
        log_context.log_total_count = LOG_FLASH_SIZE / sizeof(log_t);
    log_context.log_to_be_read_count++;
    if (log_context.log_to_be_read_count > log_context.log_total_count)
        log_context.log_to_be_read_count = log_context.log_total_count;
    for (i = 0; i < 3; i++)
    {
        uint32_t flash_wr_address = LOG_CONTEXT_FLASH_ADDRESS;
        log_context_t rd_back;

        log_context.log_context_sn++;
        log_context.crc16 = CRC16(&log_context, sizeof(log_context_t) - 2);

        log_context_index++;
        if (log_context_index >= LOG_CONTEXT_FLASH_SIZE / sizeof(log_context_t))
            log_context_index = 0;
        flash_wr_address += log_context_index * sizeof(log_context_t);
        //写入日志上下文context
        is25pl032_flash_normal_write(flash_wr_address, (uint8_t *)&log_context, sizeof(log_context_t));

        // READ BACK
        is25pl032_flash_normal_read(flash_wr_address, (uint8_t *)&rd_back, sizeof(log_context_t));
        if (memcmp(&rd_back, &log_context, sizeof(log_context_t)) == 0)
            break;
    }
    if (i == 3)
    {
        // Flash操作后重新启用传感器中断
        flash_control_sensor_interrupts(true);
        return -2;
    }

    // 检查并恢复FLASH_INITED_FLAG的值
    for (i = 0; i < 3; i++)
    {
        // 读取当前FLASH_INITED_FLAG的值
        is25pl032_flash_normal_read(0, (uint8_t *)&init_flag, sizeof(init_flag));

        // 如果值不正确，则恢复
        if (init_flag != FLASH_INITED_FLAG)
        {
            printf("FLASH_INITED_FLAG changed to 0x%08X, restoring...\r\n", init_flag);

            // 准备写入数据
            memset(flash_temp_buffer, 0xff, sizeof(flash_temp_buffer));
            *(uint32_t *)flash_temp_buffer = FLASH_INITED_FLAG;

            // 写入FLASH_INITED_FLAG
            ret = is25pl032_flash_normal_write(0, (uint8_t *)flash_temp_buffer, sizeof(flash_temp_buffer));
            if (ret != 0)
            {
                printf("Failed to restore FLASH_INITED_FLAG on attempt %d\r\n", i + 1);
                continue;
            }

            // 验证写入是否成功
            is25pl032_flash_normal_read(0, (uint8_t *)&init_flag, sizeof(init_flag));
            if (init_flag == FLASH_INITED_FLAG)
            {
                printf("Successfully restored FLASH_INITED_FLAG on attempt %d\r\n", i + 1);
                break;
            }
        }
        else
        {
//            printf("FLASH_INITED_FLAG is correct (0x%08X)\r\n", init_flag);
            break;
        }
    }

    if (i == 3)
    {
        printf("Failed to restore FLASH_INITED_FLAG after 3 attempts\r\n");
        // Flash操作后重新启用传感器中断
        flash_control_sensor_interrupts(true);
        return -3;
    }

    // Flash操作后重新启用传感器中断
    flash_control_sensor_interrupts(true);
    return 0;
}
/**
  *******************************************************************************
  * @Description:读取Flash状态寄存器
  * @Parameters :
  * @RetValue   :
  * @Note       :获取IS25LP032 Flash芯片的当前状态信息

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 10:56:01 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_read_status(void)
{
    uint32_t reg;

    reg = FLASH_RDSR;
    spi0_cs0_transfer((uint8_t *)&reg, (uint8_t *)&reg, 2);

    return reg >> 8 & 0xff;
}
/**
  *******************************************************************************
  * @Description:擦除Flash扇区
  * @Parameters :
  * @RetValue   :
  * @Note       :擦除IS25LP032 Flash芯片中指定地址的4KB扇区
  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 11:22:33 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_erase_sector(uint32_t address, uint32_t check_empty)
{
    uint32_t wren_reg = 0;
    uint32_t cmd_address;
    uint8_t *p_cmd_address = (uint8_t *)&cmd_address;

    address &= 0xfffff000;
    wren_reg = FLASH_WREN;
    p_cmd_address[0] = FLASH_SER;
    p_cmd_address[1] = (uint8_t)(address >> 16);
    p_cmd_address[2] = (uint8_t)(address >> 8);
    p_cmd_address[3] = (uint8_t)(address >> 0);

    spi0_cs0_transfer((uint8_t *)&wren_reg, (uint8_t *)&wren_reg, 1);
    spi0_cs0_transfer(p_cmd_address, p_cmd_address, 4);

    wren_reg = 1000;
    while ((is25pl032_flash_read_status() & 0x1) && wren_reg)
    {
        wren_reg--;
        vTaskDelay(1);
    }
    if (wren_reg == 0)
        printf("Erasing flash 0x%08x timeout!\r\n", address);

    if (check_empty)
    {
        uint32_t rx_buffer[65];
        uint8_t *cmd_address_data = (uint8_t *)rx_buffer;
        uint32_t len = 4096;
        while (len >= 256)
        {
            cmd_address_data[0] = FLASH_NORD;
            cmd_address_data[1] = (uint8_t)(address >> 16);
            cmd_address_data[2] = (uint8_t)(address >> 8);
            cmd_address_data[3] = (uint8_t)(address >> 0);

            address += 256;
            len -= 256;
            spi0_cs0_transfer(cmd_address_data, cmd_address_data, 260);

            for (int i = 1; i < 65; i++)
            {
                if (rx_buffer[i] != 0xffffffff)
                {
                    printf("*****Erasing Flash Failed! Some units is not empty!*****\r\n");
                    return -1;
                }
            }
        }
    }

    return 0;
}
/**
  *******************************************************************************
  * @Description:读取Flash芯片ID
  * @Parameters :
  * @RetValue   :
  * @Note       :FLASH_RDMDID：定义为 0x90，这是Flash的"Read Manufacturer/Device ID"命令

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 11:00:09 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_read_id(void)
{
    uint8_t cmd_data[8];

    cmd_data[0] = FLASH_RDMDID;
    spi0_cs0_transfer(cmd_data, cmd_data, 6);//SPI0接口的CS0片选传输函数
    return cmd_data[3] << 16 | cmd_data[4] << 8 | cmd_data[5];//接收到的3字节ID数据组合成24位整数
}
/**
  *******************************************************************************
  * @Description:页内写入Flash
  * @Parameters :
  * @RetValue   :
  * @Note       :向Flash的指定页面写入数据

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.15 23:10:55 Friday
  *******************************************************************************
  */
static int32_t is25pl032_flash_write_in_page(uint32_t flash_address, uint8_t *data, uint32_t len)
{
    uint32_t cmd_address_data[260 / 4];// 65个32位整数
    uint8_t *p_cmd_address = (uint8_t *)cmd_address_data;//将32位数组转换为8位指针，便于操作
    uint8_t wren_reg;
    uint8_t timeout;

    p_cmd_address[0] = FLASH_PP;
    p_cmd_address[1] = (uint8_t)(flash_address >> 16);
    p_cmd_address[2] = (uint8_t)(flash_address >> 8);
    p_cmd_address[3] = (uint8_t)(flash_address >> 0);
    if (len > 256)
        len = 256;
    memcpy(p_cmd_address + 4, data, len);

    wren_reg = FLASH_WREN;
    spi0_cs0_transfer(&wren_reg, &wren_reg, 1);
    spi0_cs0_transfer((uint8_t *)cmd_address_data, (uint8_t *)cmd_address_data, len + 4);

    timeout = 10;
    while ((is25pl032_flash_read_status() & 0x1) && timeout)
    {
        timeout--;
        vTaskDelay(1);
    }
    if (timeout == 0)
    {
        printf("flash_write_in_page 0x%08x timeout!\r\n", flash_address);
        return -2;
    }

    // check
    is25pl032_flash_normal_read(flash_address, (uint8_t *)cmd_address_data, len);
    if (memcmp((uint8_t *)cmd_address_data, data, len) != 0)
    {
        printf("flash_write_in_page 0x%08x failed! Data mismatch\r\n", flash_address);
        return -3;
    }

    return 0;
}

/**
  *******************************************************************************
  * @Description:页写入Flash的函数，用于处理跨页边界的数据写入，自动管理Flash页面的写入操作
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.10.15 09:27:35 Sunday
  *******************************************************************************
  */
static int32_t is25pl032_flash_write_page(uint32_t flash_address, uint8_t *data, uint32_t len)
{
#if 0
    uint32_t    cmd_address_data[260 / 4];
    uint8_t*    p_cmd_address = (uint8_t*)cmd_address_data;
    uint8_t     wren_reg;
    uint8_t     timeout;
    uint32_t    offset_of_page;
    uint32_t    writed_bytes;

    offset_of_page = flash_address & 0xff;
    writed_bytes = 256 - offset_of_page;
    if(writed_bytes > len)
        writed_bytes = len;

    p_cmd_address[0] = FLASH_PP;
    p_cmd_address[1] = (uint8_t)(flash_address >> 16);
    p_cmd_address[2] = (uint8_t)(flash_address >> 8);
    p_cmd_address[3] = (uint8_t)(flash_address >> 0);
    memcpy(p_cmd_address + 4, data, writed_bytes);
    len -= writed_bytes;
    flash_address += writed_bytes;
    data += writed_bytes;

    wren_reg = FLASH_WREN;
    PINS_DRV_WritePin(CS_PORT, CS_PIN, 0);
    DISABLE_ADS1278_IRQ;
    LPSPI_DRV_MasterTransferBlocking(LPSPICOM0, &wren_reg, &wren_reg, 1, 1);
    ENABLE_ADS1278_IRQ;
    PINS_DRV_WritePin(CS_PORT, CS_PIN, 1);

    PINS_DRV_WritePin(CS_PORT, CS_PIN, 0);
    DISABLE_ADS1278_IRQ;
    LPSPI_DRV_MasterTransferBlocking(LPSPICOM0, p_cmd_address, p_cmd_address, writed_bytes + 4, 10);
    ENABLE_ADS1278_IRQ;
    PINS_DRV_WritePin(CS_PORT, CS_PIN, 1);

    timeout = 10;
    vTaskDelay(1);
    while((is25pl032_flash_read_status() & 0x1) && timeout)
    {
        timeout--;
        vTaskDelay(1);
    }
    if(timeout == 0)
    {
        printf("Writing flash 0x%08x timeout!\r\n", flash_address - 256);
        return -2;
    }

    while(len)
    {
        if(len > 256)
            writed_bytes = 256;
        else
            writed_bytes = len;

        p_cmd_address[0] = FLASH_PP;
        p_cmd_address[1] = (uint8_t)(flash_address >> 16);
        p_cmd_address[2] = (uint8_t)(flash_address >> 8);
        p_cmd_address[3] = (uint8_t)(flash_address >> 0);
        memcpy(p_cmd_address + 4, data, writed_bytes);
        len -= writed_bytes;
        flash_address += writed_bytes;
        data += writed_bytes;

        wren_reg = FLASH_WREN;
        PINS_DRV_WritePin(CS_PORT, CS_PIN, 0);
        DISABLE_ADS1278_IRQ;
        LPSPI_DRV_MasterTransferBlocking(LPSPICOM0, &wren_reg, &wren_reg, 1, 1);
        ENABLE_ADS1278_IRQ;
        PINS_DRV_WritePin(CS_PORT, CS_PIN, 1);

        PINS_DRV_WritePin(CS_PORT, CS_PIN, 0);
        DISABLE_ADS1278_IRQ;
        LPSPI_DRV_MasterTransferBlocking(LPSPICOM0, p_cmd_address, p_cmd_address, writed_bytes + 4, 10);
        ENABLE_ADS1278_IRQ;
        PINS_DRV_WritePin(CS_PORT, CS_PIN, 1);

        timeout = 10;
        vTaskDelay(1);
        while((is25pl032_flash_read_status() & 0x1) && timeout)
        {
            timeout--;
            vTaskDelay(1);
        }
        if(timeout == 0)
        {
            printf("Writing flash 0x%08x timeout!\r\n", flash_address - 256);
            return -2;
        }
    }
    return 0;
#endif
    int32_t ret;
    uint32_t offset_of_page;
    uint32_t writed_bytes;

    while (len)
    {
        offset_of_page = flash_address & 0xff;
        writed_bytes = 256 - offset_of_page;
        if (writed_bytes > len)
            writed_bytes = len;
        ret = is25pl032_flash_write_in_page(flash_address, data, writed_bytes);
        if (ret)
            return ret;
        flash_address += writed_bytes;
        data += writed_bytes;
        len -= writed_bytes;
    }
    return 0;
}
/**
  *******************************************************************************
  * @Description:通用Flash写入函数
  * @Parameters :
  * @RetValue   :
  * @Note       :向Flash写入任意长度的数据，自动处理扇区擦除、页写入等操作

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 10:48:29 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_normal_write(uint32_t flash_address, uint8_t *data_buffer, uint32_t len)
{
    static uint8_t write_buffer[4096];
    uint32_t offset_of_4k = flash_address & 0xfff;
    uint32_t writed_bytes = 4096 - offset_of_4k;

    // 确定在当前扇区需写入的字节数，检查该FLASH是否为空
    if (writed_bytes >= len)
        writed_bytes = len;
    is25pl032_flash_normal_read(flash_address, write_buffer, writed_bytes);
    for (int i = 0; i < writed_bytes; i++)
    {
        if (write_buffer[i] != 0xff)
        {
            is25pl032_flash_normal_read(flash_address & ~0xfff, write_buffer, offset_of_4k);
            is25pl032_flash_erase_sector(flash_address, 1);
            is25pl032_flash_write_page(flash_address & ~0xfff, write_buffer, offset_of_4k);
            break;
        }
    }
    is25pl032_flash_write_page(flash_address, data_buffer, writed_bytes);
    // 剩余字节数
    len -= writed_bytes;
    flash_address += writed_bytes;
    data_buffer += writed_bytes;

    while (len)
    {
        is25pl032_flash_erase_sector(flash_address, 0);
        if (len > 4096)
            writed_bytes = 4096;
        else
            writed_bytes = len;
        is25pl032_flash_write_page(flash_address, data_buffer, writed_bytes);
        len -= writed_bytes;
        flash_address += writed_bytes;
        data_buffer += writed_bytes;
    }

    return 0;
}
/**
  *******************************************************************************
  * @Description:通用Flash读取函数
  * @Parameters :
  * @RetValue   :
  * @Note       :从Flash读取任意长度的数据，自动处理页读取等操作

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 10:33:07 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_normal_read(uint32_t flash_address, uint8_t *data_buffer, uint32_t len)
{
    uint8_t cmd_address_data[260];

    memset(cmd_address_data, 0x0, sizeof(cmd_address_data));
    while (len >= 256)
    {
        cmd_address_data[0] = FLASH_NORD;
        cmd_address_data[1] = (uint8_t)(flash_address >> 16);
        cmd_address_data[2] = (uint8_t)(flash_address >> 8);
        cmd_address_data[3] = (uint8_t)(flash_address >> 0);

        spi0_cs0_transfer(cmd_address_data, cmd_address_data, 260);

        memcpy(data_buffer, cmd_address_data + 4, 256);
        len -= 256;
        data_buffer += 256;
        flash_address += 256;
    }

    if (len)
    {
        cmd_address_data[0] = FLASH_NORD;
        cmd_address_data[1] = (uint8_t)(flash_address >> 16);
        cmd_address_data[2] = (uint8_t)(flash_address >> 8);
        cmd_address_data[3] = (uint8_t)(flash_address >> 0);

        spi0_cs0_transfer(cmd_address_data, cmd_address_data, len + 4);

        memcpy(data_buffer, cmd_address_data + 4, len);
    }

    return 0;
}
/**
  *******************************************************************************
  * @Description:保存设备配置的函数
  * @Parameters :
  * @RetValue   :
  * @Note       :将内存中的设备配置数据保存到Flash存储器，支持配置版本管理和循环存储

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 15:49:36 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_save_dev_cfg(void)
{
    DEV_CFG_T *p_dev_config = (DEV_CFG_T *)flash_temp_buffer;

    is25pl032_flash_normal_read(DEVICE_CFG_FLASH_ADDRESS + sizeof(dev_cfg) * dev_cfg_index,
                                (uint8_t *)flash_temp_buffer, sizeof(dev_cfg));
    if (memcmp(p_dev_config, &dev_cfg, sizeof(dev_cfg)) == 0)
    {
        return 0;
    }

    dev_cfg.u_cfg.cfg.device_cfg_tag = DEVICE_CFG_FLAG;
    dev_cfg.u_cfg.cfg.device_cfg_sn++;
    dev_cfg.crc16 = CRC16(&dev_cfg, sizeof(DEV_CFG_T) - 2);
    dev_cfg_index++;
    if (dev_cfg_index >= 4)
    {
        dev_cfg_index = 0;
        is25pl032_flash_erase_sector(DEVICE_CFG_FLASH_ADDRESS, 0);
    }
    is25pl032_flash_normal_write(DEVICE_CFG_FLASH_ADDRESS + sizeof(dev_cfg) * dev_cfg_index,
                                 (uint8_t *)&dev_cfg, sizeof(dev_cfg));

    load_algorithm_setting_from_flash();

    return 0;
}
/**
  *******************************************************************************
  * @Description:Flash存储系统的初始化函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 09:58:22 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_init(void)
{
    DEV_CFG_T *p_dev_config;
    uint32_t dev_cfg_max_sn;
    uint32_t log_conext_max_sn;
    total_runtime_record_t *p_runtime_record;

    log_context_index = -1;
    dev_cfg_index = -1;

    printf("FLASH ID: 0x%x\r\n", is25pl032_flash_read_id());

    // 检查FLASH是否已被初始化
    is25pl032_flash_normal_read(0, (uint8_t *)flash_temp_buffer, sizeof(flash_temp_buffer));
    if (*(uint32_t *)flash_temp_buffer != FLASH_INITED_FLAG)
    {
        // 擦除BLOCK
        for (int i = 0; i < 4 * 1024 * 24; i += 4 * 1024)
        {
            printf("Erasing 0x%04x\r\n", i);
            is25pl032_flash_erase_sector(i, 0);
        }

        // 设置已初始化标志
        memset(flash_temp_buffer, 0xff, sizeof(flash_temp_buffer));
        *(uint32_t *)flash_temp_buffer = FLASH_INITED_FLAG;
        is25pl032_flash_normal_write(0, (uint8_t *)flash_temp_buffer, sizeof(flash_temp_buffer));
    }

    // 检查参数 - 使用动态计算替代硬编码
    dev_cfg_max_sn = 0;
    uint32_t dev_cfg_buffer_count = DEVICE_CFG_FLASH_SIZE / sizeof(flash_temp_buffer);
    for (uint32_t i = 0; i < dev_cfg_buffer_count; i++)
    {
        is25pl032_flash_normal_read(DEVICE_CFG_FLASH_ADDRESS + sizeof(flash_temp_buffer) * i, (uint8_t *)flash_temp_buffer, sizeof(flash_temp_buffer));

        // 计算在当前buffer中能容纳多少个DEV_CFG_T结构体
        uint32_t max_dev_cfg_count = sizeof(flash_temp_buffer) / sizeof(DEV_CFG_T);
        for (uint32_t j = 0; j < max_dev_cfg_count; j++)
        {
            p_dev_config = (DEV_CFG_T *)flash_temp_buffer + j;
            if (p_dev_config->u_cfg.cfg.device_cfg_tag == DEVICE_CFG_FLAG)
            {
                int32_t crc = CRC16(p_dev_config, sizeof(DEV_CFG_T) - 2);
                if (crc == p_dev_config->crc16)
                {
                    if (p_dev_config->u_cfg.cfg.device_cfg_sn > dev_cfg_max_sn)
                    {
                        dev_cfg_max_sn = p_dev_config->u_cfg.cfg.device_cfg_sn;
                        dev_cfg_index = i;
                        dev_cfg = *p_dev_config;
                    }
                }
            }
        }
    }
    // 初始化
    if (dev_cfg_max_sn == 0)
    {
        dev_cfg.u_cfg.cfg = default_cfg;

        dev_cfg.crc16 = CRC16(&dev_cfg, sizeof(DEV_CFG_T) - 2);
        memset(flash_temp_buffer, 0x0, sizeof(flash_temp_buffer));
        dev_cfg_index = 0;
        memcpy((void *)flash_temp_buffer, &dev_cfg, sizeof(dev_cfg));
        is25pl032_flash_normal_write(DEVICE_CFG_FLASH_ADDRESS, (uint8_t *)flash_temp_buffer, sizeof(dev_cfg));
    }

    // 确定运行时间最后记录 - 使用动态计算替代硬编码
    uint32_t runtime_buffer_count = TOTAL_RUNTIME_FLASH_SIZE / sizeof(flash_temp_buffer);
    for (uint32_t i = 0, j = 0; i < runtime_buffer_count; i++)
    {
        uint16_t crc;

        is25pl032_flash_normal_read(TOTAL_RUNTIME_FLASH_ADDRESS + sizeof(flash_temp_buffer) * i, (uint8_t *)flash_temp_buffer, sizeof(flash_temp_buffer));
        p_runtime_record = (total_runtime_record_t *)flash_temp_buffer;

        // 计算在当前buffer中能容纳多少个total_runtime_record_t结构体
        uint32_t max_runtime_count = sizeof(flash_temp_buffer) / sizeof(total_runtime_record_t);
        for (uint32_t k = 0; k < max_runtime_count; k++)
        {
            p_runtime_record = (total_runtime_record_t *)flash_temp_buffer + k;
            if (p_runtime_record->magic != TOTAL_RUNTIME_FLAG)
                continue;
            crc = CRC16(p_runtime_record, sizeof(*p_runtime_record) - 2);
            if (crc != p_runtime_record->crc16)
                continue;
            if (total_run_time.sn < p_runtime_record->sn)
            {
                total_run_time = *p_runtime_record;
                last_run_time_flash_address = (total_runtime_record_t *)(TOTAL_RUNTIME_FLASH_ADDRESS + j * sizeof(total_runtime_record_t));
                continue;
            }
        }
    }
    if (last_run_time_flash_address == NULL)
    {
        total_run_time.magic = TOTAL_RUNTIME_FLAG;
        total_run_time.sn = 1;
        total_run_time.crc16 = CRC16(&total_run_time, sizeof(total_runtime_record_t) - 2);
        is25pl032_flash_erase_sector(TOTAL_RUNTIME_FLASH_ADDRESS, 0);
        is25pl032_flash_normal_write(TOTAL_RUNTIME_FLASH_ADDRESS, (uint8_t *)&total_run_time, sizeof(total_runtime_record_t));
        last_run_time_flash_address = (total_runtime_record_t *)TOTAL_RUNTIME_FLASH_ADDRESS;
    }

    // 检查LOG上下文 - 使用动态计算替代硬编码
    log_context.log_tag = LOG_CONTEXT_TAG;
    log_context.log_write_index = -1;
    log_context.log_write_total = 0;
    log_conext_max_sn = 0;
    uint32_t log_context_buffer_count = LOG_CONTEXT_FLASH_SIZE / sizeof(flash_temp_buffer);
    for (uint32_t i = 0; i < log_context_buffer_count; i++)
    {
        log_context_t *p_log_context;

        p_log_context = (log_context_t *)flash_temp_buffer;
        is25pl032_flash_normal_read(LOG_CONTEXT_FLASH_ADDRESS + sizeof(flash_temp_buffer) * i, (uint8_t *)flash_temp_buffer, sizeof(flash_temp_buffer));
        // 计算在当前buffer中能容纳多少个log_context_t结构体
        uint32_t max_log_context_count = sizeof(flash_temp_buffer) / sizeof(log_context_t);
        for (uint32_t j = 0; j < max_log_context_count; j++)
        {
            p_log_context = (log_context_t *)flash_temp_buffer + j;
            int32_t crc;

            if (p_log_context->log_tag != LOG_CONTEXT_TAG)
                continue;
            crc = CRC16(p_log_context, sizeof(*p_log_context) - 2);
            if (crc != p_log_context->crc16)
                continue;
            if (log_conext_max_sn >= p_log_context->log_context_sn)
                continue;

            log_conext_max_sn = p_log_context->log_context_sn;
            log_context = *p_log_context;
            log_context_index = j + i * sizeof(flash_temp_buffer) / sizeof(log_context_t);
        }
    }

    printf("There are %d logs in flash.\r\n", log_context.log_to_be_read_count);
    return 0;
}

/**
  *******************************************************************************
  * @Description:设置日志保存周期
  * @Parameters :
  * @RetValue   :
  * @Note       :设置设备配置中的日志保存周期

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.10.15 15:25:04 Sunday
  *******************************************************************************
  */
uint32_t is25pl032_flash_set_log_period(uint8_t period)
{
    dev_cfg.u_cfg.cfg.log_saved_period = period;
    is25pl032_flash_save_dev_cfg();
    return 0;
}

/**
  *******************************************************************************
  * @Description:获取日志保存周期
  * @Parameters :
  * @RetValue   :
  * @Note       :获取设备配置中的日志保存周期

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.10.15 15:23:34 Sunday
  *******************************************************************************
  */
uint32_t is25pl032_flash_get_log_period(void)
{
    return dev_cfg.u_cfg.cfg.log_saved_period;
}
/**
  *******************************************************************************
  * @Description:重置设备配置
  * @Parameters :
  * @RetValue   :
  * @Note       :将设备配置重置为默认配置，并更新设备配置的序列号

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.14 22:02:22 Thursday
  *******************************************************************************
  */
uint32_t is25pl032_flash_reset_cfg(void)
{
    uint32_t last_sn = dev_cfg.u_cfg.cfg.device_cfg_sn;
    dev_cfg.u_cfg.cfg = default_cfg;
    dev_cfg.u_cfg.cfg.device_cfg_sn = last_sn;
    is25pl032_flash_save_dev_cfg();
    return 0;
}
/**
  *******************************************************************************
  * @Description:设置偏移量
  * @Parameters :
  * @RetValue   :
  * @Note       :设置设备配置中的偏移量

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.14 22:19:27 Thursday
  *******************************************************************************
  */
void is25pl032_flash_set_offset(uint32_t index, float d)
{
    if (index < 2)
    {
        dev_cfg.u_cfg.cfg.offset[index] = d;
        is25pl032_flash_save_dev_cfg();
    }
}
// void is25pl032_flash_set_roll_pitch_357(uint32_t index, double d)
//{
//     if(index < 2)
//     {
//         dev_cfg.u_cfg.cfg.roll_pitch_357[index] = d;
//         is25pl032_flash_save_dev_cfg();
//     }
// }
/**
  *******************************************************************************
  * @Description:获取偏移量
  * @Parameters :
  * @RetValue   :
  * @Note       :获取设备配置中的偏移量

  * @CreatedBy  : YuQiang
  * @CreatedDate: 2023.12.27
  *******************************************************************************
  */
float is25pl032_flash_get_offset(uint32_t index)
{
    if (index < 2)
        return dev_cfg.u_cfg.cfg.offset[index];
    return 0.0;
}
// double is25pl032_flash_get_roll_pitch_357(uint32_t index)
//{
//     if(index < 2)
//         return dev_cfg.u_cfg.cfg.roll_pitch_357[index];
//     return 0.0;
// }
/**
  *******************************************************************************
  * @Description:获取不同温度下的运行时间记录
  * @Parameters :
  * @RetValue   :
  * @Note       :获取设备配置中的运行时间记录

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.14 22:30:53 Thursday
  *******************************************************************************
  */
int32_t get_total_time_per_temp(uint16_t **addr)
{
    *addr = (uint16_t *)total_run_time.total_times;
    return TOTAL_RUNTIME_ITEMS;
}
void is25pl032_flash_set_imu(double b[])
{
    dev_cfg.u_cfg.cfg.bx = b[0];
    dev_cfg.u_cfg.cfg.by = b[1];
    dev_cfg.u_cfg.cfg.bz = b[2];
    dev_cfg.u_cfg.cfg.mxx = b[3];
    dev_cfg.u_cfg.cfg.mxy = b[4];
    dev_cfg.u_cfg.cfg.mxz = b[5];
    dev_cfg.u_cfg.cfg.myy = b[6];
    dev_cfg.u_cfg.cfg.myz = b[7];
    dev_cfg.u_cfg.cfg.mzz = b[8];
    is25pl032_flash_save_dev_cfg();
}

void is25pl032_flash_get_imu(double *b)
{
    *b = dev_cfg.u_cfg.cfg.bx;
    b += 1;
    *b = dev_cfg.u_cfg.cfg.by;
    b += 1;
    *b = dev_cfg.u_cfg.cfg.bz;
    b += 1;
    *b = dev_cfg.u_cfg.cfg.mxx;
    b += 1;
    *b = dev_cfg.u_cfg.cfg.mxy;
    b += 1;
    *b = dev_cfg.u_cfg.cfg.mxz;
    b += 1;
    *b = dev_cfg.u_cfg.cfg.myy;
    b += 1;
    *b = dev_cfg.u_cfg.cfg.myz;
    b += 1;
    *b = dev_cfg.u_cfg.cfg.mzz;
}
void is25pl032_flash_set_px(float p[])
{
    for (uint8_t i = 0; i < 5; i++)
    {
        dev_cfg.u_cfg.cfg.px[i] = p[i];
    }
    is25pl032_flash_save_dev_cfg();
}
void is25pl032_flash_get_px(float *b)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        *b++ = dev_cfg.u_cfg.cfg.px[i];
    }
}
void is25pl032_flash_set_py(float p[])
{
    for (uint8_t i = 0; i < 5; i++)
    {
        dev_cfg.u_cfg.cfg.py[i] = p[i];
    }
    is25pl032_flash_save_dev_cfg();
}
void is25pl032_flash_get_py(float *b)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        *b++ = dev_cfg.u_cfg.cfg.py[i];
    }
}
void is25pl032_flash_set_pz(float p[])
{
    for (uint8_t i = 0; i < 5; i++)
    {
        dev_cfg.u_cfg.cfg.pz[i] = p[i];
    }
    is25pl032_flash_save_dev_cfg();
}
void is25pl032_flash_get_pz(float *b)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        *b++ = dev_cfg.u_cfg.cfg.pz[i];
    }
}
/**
  *******************************************************************************
  * @Description:保存运行时间记录
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.16 11:08:37 Saturday
  *******************************************************************************
  */
static int32_t is25pl032_flash_save_runtime_record(total_runtime_record_t *record)
{
    last_run_time_flash_address++;
    if ((uint32_t)last_run_time_flash_address > LOG_CONTEXT_FLASH_ADDRESS + LOG_CONTEXT_FLASH_SIZE)
        last_run_time_flash_address = (total_runtime_record_t *)LOG_CONTEXT_FLASH_ADDRESS;
    if ((uint32_t)last_run_time_flash_address == LOG_CONTEXT_FLASH_ADDRESS || (uint32_t)last_run_time_flash_address == LOG_CONTEXT_FLASH_ADDRESS + 0x1000)
        is25pl032_flash_erase_sector((uint32_t)last_run_time_flash_address, 0);
    is25pl032_flash_normal_write((uint32_t)last_run_time_flash_address, (uint8_t *)record, sizeof(total_runtime_record_t));
    return 0;
}
/**
  *******************************************************************************
  * @Description:更新不同温度下的运行时间记录
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.14 22:41:21 Thursday
  *******************************************************************************
  */
void update_total_time_per_temp(float temp)
{
    static int32_t seconds;

    uint32_t index;
    int32_t t = temp;

    seconds++;
    if (seconds >= 60) /*每60秒保存一次*/
    {
        seconds = 0;
        if (t < 20)
            index = 0;
        else if (t >= 180)
            index = TOTAL_RUNTIME_ITEMS - 1;
        else
            index = t * TOTAL_RUNTIME_ITEMS / 200;
        total_run_time.total_times[index]++;
        is25pl032_flash_save_runtime_record(&total_run_time);
    }
}

void is25pl032_flash_set_param(double xr_limit, double yr_limit)
{
    dev_cfg.u_cfg.cfg.xr_limit = xr_limit;
    dev_cfg.u_cfg.cfg.yr_limit = yr_limit;
    is25pl032_flash_save_dev_cfg();
}

void is25pl032_flash_get_param(double *xr_limit, double *yr_limit)
{
    *xr_limit = dev_cfg.u_cfg.cfg.xr_limit;
    *yr_limit = dev_cfg.u_cfg.cfg.yr_limit;
}

void is25pl032_flash_set_degree(uint8_t t, double b[], int8_t degree)
{
    if (t == 0)
    {
        dev_cfg.u_cfg.cfg.degree = degree;
        dev_cfg.u_cfg.cfg.axB0 = b[0];
        dev_cfg.u_cfg.cfg.axB1 = b[1];
        dev_cfg.u_cfg.cfg.axB2 = b[2];
        dev_cfg.u_cfg.cfg.axB3 = b[3];
        dev_cfg.u_cfg.cfg.axB4 = b[4];
        dev_cfg.u_cfg.cfg.axB5 = b[5];
    }
    if (t == 1)
    {
        dev_cfg.u_cfg.cfg.degree = degree;
        dev_cfg.u_cfg.cfg.ayB0 = b[0];
        dev_cfg.u_cfg.cfg.ayB1 = b[1];
        dev_cfg.u_cfg.cfg.ayB2 = b[2];
        dev_cfg.u_cfg.cfg.ayB3 = b[3];
        dev_cfg.u_cfg.cfg.ayB4 = b[4];
        dev_cfg.u_cfg.cfg.ayB5 = b[5];
    }
    if (t == 2)
    {
        dev_cfg.u_cfg.cfg.degree = degree;
        dev_cfg.u_cfg.cfg.azB0 = b[0];
        dev_cfg.u_cfg.cfg.azB1 = b[1];
        dev_cfg.u_cfg.cfg.azB2 = b[2];
        dev_cfg.u_cfg.cfg.azB3 = b[3];
        dev_cfg.u_cfg.cfg.azB4 = b[4];
        dev_cfg.u_cfg.cfg.azB5 = b[5];
    }
    if (t == 3)
    {
        dev_cfg.u_cfg.cfg.degree = degree;
        dev_cfg.u_cfg.cfg.gxB0 = b[0];
        dev_cfg.u_cfg.cfg.gxB1 = b[1];
        dev_cfg.u_cfg.cfg.gxB2 = b[2];
        dev_cfg.u_cfg.cfg.gxB3 = b[3];
        dev_cfg.u_cfg.cfg.gxB4 = b[4];
        dev_cfg.u_cfg.cfg.gxB5 = b[5];
    }
    if (t == 4)
    {
        dev_cfg.u_cfg.cfg.degree = degree;
        dev_cfg.u_cfg.cfg.gyB0 = b[0];
        dev_cfg.u_cfg.cfg.gyB1 = b[1];
        dev_cfg.u_cfg.cfg.gyB2 = b[2];
        dev_cfg.u_cfg.cfg.gyB3 = b[3];
        dev_cfg.u_cfg.cfg.gyB4 = b[4];
        dev_cfg.u_cfg.cfg.gyB5 = b[5];
    }
    if (t == 5)
    {
        dev_cfg.u_cfg.cfg.degree = degree;
        dev_cfg.u_cfg.cfg.gzB0 = b[0];
        dev_cfg.u_cfg.cfg.gzB1 = b[1];
        dev_cfg.u_cfg.cfg.gzB2 = b[2];
        dev_cfg.u_cfg.cfg.gzB3 = b[3];
        dev_cfg.u_cfg.cfg.gzB4 = b[4];
        dev_cfg.u_cfg.cfg.gzB5 = b[5];
    }
    is25pl032_flash_save_dev_cfg();
}

void is25pl032_flash_get_degree(double *b, int8_t *degree)
{
    *degree = dev_cfg.u_cfg.cfg.degree;
    *b++ = dev_cfg.u_cfg.cfg.axB0;
    *b++ = dev_cfg.u_cfg.cfg.axB1;
    *b++ = dev_cfg.u_cfg.cfg.axB2;
    *b++ = dev_cfg.u_cfg.cfg.axB3;
    *b++ = dev_cfg.u_cfg.cfg.axB4;
    *b++ = dev_cfg.u_cfg.cfg.axB5;

    *b++ = dev_cfg.u_cfg.cfg.ayB0;
    *b++ = dev_cfg.u_cfg.cfg.ayB1;
    *b++ = dev_cfg.u_cfg.cfg.ayB2;
    *b++ = dev_cfg.u_cfg.cfg.ayB3;
    *b++ = dev_cfg.u_cfg.cfg.ayB4;
    *b++ = dev_cfg.u_cfg.cfg.ayB5;

    *b++ = dev_cfg.u_cfg.cfg.azB0;
    *b++ = dev_cfg.u_cfg.cfg.azB1;
    *b++ = dev_cfg.u_cfg.cfg.azB2;
    *b++ = dev_cfg.u_cfg.cfg.azB3;
    *b++ = dev_cfg.u_cfg.cfg.azB4;
    *b++ = dev_cfg.u_cfg.cfg.azB5;

    *b++ = dev_cfg.u_cfg.cfg.gxB0;
    *b++ = dev_cfg.u_cfg.cfg.gxB1;
    *b++ = dev_cfg.u_cfg.cfg.gxB2;
    *b++ = dev_cfg.u_cfg.cfg.gxB3;
    *b++ = dev_cfg.u_cfg.cfg.gxB4;
    *b++ = dev_cfg.u_cfg.cfg.gxB5;

    *b++ = dev_cfg.u_cfg.cfg.gyB0;
    *b++ = dev_cfg.u_cfg.cfg.gyB1;
    *b++ = dev_cfg.u_cfg.cfg.gyB2;
    *b++ = dev_cfg.u_cfg.cfg.gyB3;
    *b++ = dev_cfg.u_cfg.cfg.gyB4;
    *b++ = dev_cfg.u_cfg.cfg.gyB5;

    *b++ = dev_cfg.u_cfg.cfg.gzB0;
    *b++ = dev_cfg.u_cfg.cfg.gzB1;
    *b++ = dev_cfg.u_cfg.cfg.gzB2;
    *b++ = dev_cfg.u_cfg.cfg.gzB3;
    *b++ = dev_cfg.u_cfg.cfg.gzB4;
    *b++ = dev_cfg.u_cfg.cfg.gzB5;
}

void is25pl032_flash_set_proid(char pro_id[], uint8_t len)
{
    memcpy(dev_cfg.u_cfg.cfg.pro_id, pro_id, len);
    is25pl032_flash_save_dev_cfg();
}

void is25pl032_flash_get_proid(char *pro_id)
{
    memcpy(pro_id, dev_cfg.u_cfg.cfg.pro_id, sizeof(dev_cfg.u_cfg.cfg.pro_id));
}

void is25pl032_flash_get_version(char *version)
{
    // 始终使用编译时的版本信息，确保版本号与当前代码一致
    // 先清零整个缓冲区
    memset(version, 0, sizeof(dev_cfg.u_cfg.cfg.version));
    // 复制TGDMC字符串，使用strcpy确保完整复制
    strcpy(version, TGDMC);
}

/**
 *******************************************************************************
 * @Description: 获取陀螺仪X轴零偏
 * @Parameters : 无
 * @RetValue   : X轴零偏值
 * @Note       : 获取陀螺仪X轴的零偏补偿值
 *******************************************************************************
 */
float get_gx_offset(void)
{
    return dev_cfg.u_cfg.cfg.gx_bias;
}
/**
 *******************************************************************************
 * @Description: 获取陀螺仪Y轴零偏
 * @Parameters : 无
 * @RetValue   : Y轴零偏值
 * @Note       : 获取陀螺仪Y轴的零偏补偿值
 *******************************************************************************
 */
float get_gy_offset(void)
{
    return dev_cfg.u_cfg.cfg.gy_bias;
}
/**
 *******************************************************************************
 * @Description: 获取陀螺仪Z轴零偏
 * @Parameters : 无
 * @RetValue   : Z轴零偏值
 * @Note       : 获取陀螺仪Z轴的零偏补偿值
 *******************************************************************************
 */
float get_gz_offset(void)
{
    return dev_cfg.u_cfg.cfg.gz_bias;
}

/* 获取加速度计型号 */
uint8_t get_acc_sensor_type(void)
{
    // 如果配置无效或未初始化，返回默认值
    if (dev_cfg.u_cfg.cfg.device_cfg_tag != DEVICE_CFG_FLAG)
    {
        return default_cfg.acc_sensor_type; // 返回默认值 0 (IAM-20680HT)
    }
    return dev_cfg.u_cfg.cfg.acc_sensor_type;
}

/* 获取陀螺仪型号 */
uint8_t get_gyro_sensor_type(void)
{
    // 如果配置无效或未初始化，返回默认值
    if (dev_cfg.u_cfg.cfg.device_cfg_tag != DEVICE_CFG_FLAG)
    {
        return default_cfg.gyro_sensor_type; // 返回默认值 0 (IAM-20680HT)
    }
    return dev_cfg.u_cfg.cfg.gyro_sensor_type;
}

/* 设置加速度计型号 */
int32_t set_acc_sensor_type(uint8_t type)
{
    dev_cfg.u_cfg.cfg.acc_sensor_type = type;
    return is25pl032_flash_save_dev_cfg();
}

/* 设置陀螺仪型号 */
int32_t set_gyro_sensor_type(uint8_t type)
{
    dev_cfg.u_cfg.cfg.gyro_sensor_type = type;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取温度补偿下限值
 * @Parameters : 无
 * @RetValue   : 温度补偿下限值（°C）
 * @Note       : 获取温度补偿有效范围的下限值
 *******************************************************************************
 */
float get_temp_comp_lower_limit(void)
{
    return dev_cfg.u_cfg.cfg.temp_comp_lower_limit;
}

/**
 *******************************************************************************
 * @Description: 获取温度补偿上限值
 * @Parameters : 无
 * @RetValue   : 温度补偿上限值（°C）
 * @Note       : 获取温度补偿有效范围的上限值
 *******************************************************************************
 */
float get_temp_comp_upper_limit(void)
{
    return dev_cfg.u_cfg.cfg.temp_comp_upper_limit;
}

/**
 *******************************************************************************
 * @Description: 设置温度补偿下限值
 * @Parameters : limit - 温度补偿下限值（°C）
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置温度补偿有效范围的下限值
 *******************************************************************************
 */
int32_t set_temp_comp_lower_limit(float limit)
{
    dev_cfg.u_cfg.cfg.temp_comp_lower_limit = limit;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 设置温度补偿上限值
 * @Parameters : limit - 温度补偿上限值（°C）
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置温度补偿有效范围的上限值
 *******************************************************************************
 */
int32_t set_temp_comp_upper_limit(float limit)
{
    dev_cfg.u_cfg.cfg.temp_comp_upper_limit = limit;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取停泵状态下的静态井斜前置延迟时间1
 * @Parameters : 无
 * @RetValue   : 延迟时间（秒）
 * @Note       : 获取停泵状态下静态井斜前置延迟时间1的配置值
 *               默认为8秒，用于控制停泵后多久开始记录静态井斜数据
 * @CreatedBy  : Assistant
 * @CreatedDate: 2025.01.27
 *******************************************************************************
 */
uint16_t is25pl032_flash_get_pump_delay1(void)
{
    return dev_cfg.u_cfg.cfg.pump_delay1;
}

/**
 *******************************************************************************
 * @Description: 设置停泵状态下的静态井斜前置延迟时间1
 * @Parameters : delay - 延迟时间（秒）
 * @RetValue   : 0-成功
 * @Note       : 设置停泵状态下静态井斜前置延迟时间1
 *               该参数控制停泵后多久开始记录静态井斜数据
 *               默认为8秒
 * @CreatedBy  : Assistant
 * @CreatedDate: 2025.01.27
 *******************************************************************************
 */
uint16_t is25pl032_flash_set_pump_delay1(uint16_t delay)
{
    dev_cfg.u_cfg.cfg.pump_delay1 = delay;
    return 0;
}

/**
 *******************************************************************************
 * @Description: 获取停泵状态下的静态井斜前置延迟时间2
 * @Parameters : 无
 * @RetValue   : 延迟时间（秒）
 * @Note       : 获取停泵状态下静态井斜前置延迟时间2的配置值
 *               默认为12秒，用于控制停泵后多久开始记录静态井斜数据
 *               与pump_delay1配合使用，提供更灵活的延迟控制
 * @CreatedBy  : Assistant
 * @CreatedDate: 2025.01.27
 *******************************************************************************
 */
uint16_t is25pl032_flash_get_pump_delay2(void)
{
    return dev_cfg.u_cfg.cfg.pump_delay2;
}

/**
 *******************************************************************************
 * @Description: 设置停泵状态下的静态井斜前置延迟时间2
 * @Parameters : delay - 延迟时间（秒）
 * @RetValue   : 0-成功
 * @Note       : 设置停泵状态下静态井斜前置延迟时间2
 *               该参数控制停泵后多久开始记录静态井斜数据
 *               默认为12秒，与pump_delay1配合使用，提供更灵活的延迟控制
 * @CreatedBy  : Assistant
 * @CreatedDate: 2025.01.27
 *******************************************************************************
 */
uint16_t is25pl032_flash_set_pump_delay2(uint16_t delay)
{
    dev_cfg.u_cfg.cfg.pump_delay2 = delay;
    return 0;
}

/**
 *******************************************************************************
 * @Description: 设置泥浆脉冲数据重传次数
 * @Parameters : count - 重传次数
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置泥浆脉冲数据有效范围的重传次数
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_pulse_retry_for_pump_off_data(uint32_t count)
{
    dev_cfg.u_cfg.cfg.pulse_retry_for_pump_off_data = count;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取静态脉冲数据重传次数
 * @Parameters : 无
 * @RetValue   : 静态脉冲数据重传次数
 * @Note       : 获取静态脉冲数据有效范围的重传次数
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_pulse_retry_for_pump_off_data(void)
{
    return dev_cfg.u_cfg.cfg.pulse_retry_for_pump_off_data;
}

/**
 *******************************************************************************
 * @Description: 设置静态脉冲数据重传时间间隔
 * @Parameters : retry_interval - 静态脉冲数据重传时间间隔（秒）
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置静态脉冲数据重传时间间隔
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_pulse_interval_for_pump_off_data(uint32_t retry_interval)
{
    dev_cfg.u_cfg.cfg.pulse_interval_for_pump_off_data = retry_interval;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取静态脉冲数据重传时间间隔
 * @Parameters : 无
 * @RetValue   : 静态脉冲数据重传时间间隔（秒）
 * @Note       : 获取静态脉冲数据重传时间间隔
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_pulse_interval_for_pump_off_data(void)
{
    return dev_cfg.u_cfg.cfg.pulse_interval_for_pump_off_data;
}

/**
 *******************************************************************************
 * @Description: 设置动态脉冲数据周期性上传时间
 * @Parameters : pulse_interval - 动态脉冲数据周期性上传时间
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置动态脉冲数据周期性上传时间
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_pulse_interval(uint32_t pulse_interval)
{
    dev_cfg.u_cfg.cfg.pulse_interval = pulse_interval;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取动态脉冲数据周期性上传时间
 * @Parameters : 无
 * @RetValue   : 动态脉冲数据周期性上传时间
 * @Note       : 获取动态脉冲数据周期性上传时间
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_pulse_interval(void)
{
    return dev_cfg.u_cfg.cfg.pulse_interval;
}

/**
 *******************************************************************************
 * @Description: 设置低功耗状态
 * @Parameters : enable - 使能状态（0-禁用，1-启用）
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置低功耗状态
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_idle_hook_enable(uint32_t enable)
{
    dev_cfg.u_cfg.cfg.idle_hook_enable = enable;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取低功耗状态
 * @Parameters : 无
 * @RetValue   : 低功耗状态（0-禁用，1-启用）
 * @Note       : 获取低功耗状态
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_idle_hook_enable(void)
{
    return dev_cfg.u_cfg.cfg.idle_hook_enable;
}

/**
 *******************************************************************************
 * @Description: 获取校准数据
 * @Parameters : 无
 * @RetValue   : 校准数据值
 * @Note       : 获取校准数据
 *******************************************************************************
 */
float is25pl032_flash_get_calibration_data(void)
{
    return dev_cfg.u_cfg.cfg.calibration_data;
}

/**
 *******************************************************************************
 * @Description: 设置校准数据
 * @Parameters : calibration_data - 要设置的校准数据值
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置校准数据
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_calibration_data(float calibration_data)
{
    dev_cfg.u_cfg.cfg.calibration_data = calibration_data;
    is25pl032_flash_save_dev_cfg();
    return 0;
}

/**
  *******************************************************************************
  * @Description: Flash操作时的中断控制函数
  * @Parameters : enable - true启用中断，false禁用中断
  * @RetValue   : 无
  * @Note       : 在Flash操作前禁用传感器中断，操作后重新启用
  * @CreatedBy  : Assistant
  * @CreatedDate: 2025.01.27
  *******************************************************************************
  */
void flash_control_sensor_interrupts(bool enable)
{
    if(enable) {
        // 重新启用传感器中断
        // printf("Flash: Re-enabling sensor interrupts...\r\n");
        PINS_DRV_SetPinIntSel(PORTE, 5, PORT_INT_FALLING_EDGE);  // PTE5 - IAM20680HT
        // printf("Flash: Sensor interrupts re-enabled (PTE5: IAM20680HT)\r\n");
    } else {
        // 禁用传感器中断
        // printf("Flash: Disabling sensor interrupts for Flash operation...\r\n");
        PINS_DRV_SetPinIntSel(PORTE, 5, PORT_DMA_INT_DISABLED);  // PTE5 - IAM20680HT
        // printf("Flash: Sensor interrupts disabled (PTE5: IAM20680HT)\r\n");
    }
}
