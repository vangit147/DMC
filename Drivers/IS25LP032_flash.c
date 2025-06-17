/**
 *****************************************************************************
 * FILENAME   : IS25LP032_flash.c
 * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
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
#include "mud_pulse.h"

#define FLASH_PP 0X02
#define FLASH_RDSR 0X05
#define FLASH_WREN 0X06
#define FLASH_NORD 0X03
#define FLASH_RDMDID 0X90
#define FLASH_SER 0XD7

static uint64_t flash_temp_buffer[512]; // 4K读写缓冲区

/*
FLASH 数据区定义
+--------------------------+
|       FLASH_INIT   (4K)  |
+--------------------------+
|       DEV_CFG      (4K)  |
+--------------------------+
|     TOTAL_RUN_TIME (8K)  |
+--------------------------+
|     LOG CONTEXT    (8K)  |
+--------------------------+
|          LOG             |
+--------------------------+

*/
#define FLASH_INITED_FLAG 0XABEEBEE7
#define FLASH_TOTAL_SIZE (4 * 1024 * 1024)
// CFG
#define DEVICE_CFG_FLAG 0XBEE1BEEE
#define DEVICE_CFG_FLASH_ADDRESS 0X1000
#define DEVICE_CFG_FLASH_SIZE 0X1000
#define DEVICE_CFG_SIZE 0X400
typedef __packed struct // 占用1k字节，最后2字节为CRC16校验和
{
    uint32_t device_cfg_tag;       // 标志
    uint32_t device_cfg_sn;        // 每更新一次，该值加1
    uint32_t device_cfg_reserved0; // 保留后续使用
    uint32_t device_cfg_reserved1; // 保留后续使用
    uint8_t log_saved_period;      // LOG保存周期，秒
    uint8_t acc_sensor_type;       // 加速度传感器型号
    uint8_t gyro_sensor_type;      // 陀螺仪传感器型号
    uint8_t unused_0[1];           // 保留1字节
    // X、Y轴加速度计装配误差
    float offset[2];
    // X、Y轴虚拟半径最大限制
    double xr_limit; // 半径
    double yr_limit; // 半径余量
    // 陀螺仪X、Y、Z三轴的零偏
    float gx_bias;
    float gy_bias;
    float gz_bias;
    int8_t degree; // 阶数
    // 加速度计多项式拟合系数
    double axB0;
    double axB1;
    double axB2;
    double axB3;
    double axB4;
    double axB5;

    double ayB0;
    double ayB1;
    double ayB2;
    double ayB3;
    double ayB4;
    double ayB5;

    double azB0;
    double azB1;
    double azB2;
    double azB3;
    double azB4;
    double azB5;

    // 陀螺仪多项式拟合系数
    double gxB0;
    double gxB1;
    double gxB2;
    double gxB3;
    double gxB4;
    double gxB5;

    double gyB0;
    double gyB1;
    double gyB2;
    double gyB3;
    double gyB4;
    double gyB5;

    double gzB0;
    double gzB1;
    double gzB2;
    double gzB3;
    double gzB4;
    double gzB5;
    // x、y、z加速度计零偏
    double bx;
    double by;
    double bz;
    // MS矩阵系数
    double mxx;
    double mxy;
    double mxz;
    double myy;
    double myz;
    double mzz;
    // px、py、pz数组暂时不用
    float px[5];
    float py[5];
    float pz[5];
    char pro_id[15];
    char version[15];
    /* 温度补偿范围设置 */
    float temp_comp_lower_limit; // 温度补偿下限，默认-20°C
    float temp_comp_upper_limit; // 温度补偿上限，默认150°C
    float t_scale;               // 温度比例系数
    float t_intercept;           // 温度截距
    /* 泥浆脉冲设置 */
    mud_pulse_config_t mud_pulse_cfg; // 泥浆脉冲配置
    /* 振动参数设置 */
    float vibration_threshold;      // 振动阈值
    uint32_t vibration_sensitivity; // 振动灵敏度
    uint32_t idle_hook_enable;      // 低功耗状态
    float calibration_data;         // 添加校准数据变量
} CFG_T;

// 默认参数
static const CFG_T default_cfg = {
    .device_cfg_tag = DEVICE_CFG_FLAG,
    .device_cfg_sn = 1,
    .log_saved_period = 60,
    .acc_sensor_type = 0,  // 默认使用IAM-20680HT加速度计
    .gyro_sensor_type = 0, // 默认使用IAM-20680HT陀螺仪
    .offset = {0.0f, 0.0f},
    .xr_limit = 0.0015f,
    .yr_limit = 0.015f,
    .gx_bias = 0.0f,
    .gy_bias = 0.0f,
    .gz_bias = 0.0f,
    .bx = 0.0f,
    .by = 0.0f,
    .bz = 0.0f,
    .mxx = 1.0f,
    .mxy = 0.0f,
    .mxz = 0.0f,
    .myy = 1.0f,
    .myz = 0.0f,
    .mzz = 1.0f,
    .px = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .py = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .pz = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .temp_comp_lower_limit = -20.0f, // 温度补偿下限默认值
    .temp_comp_upper_limit = 150.0f, // 温度补偿上限默认值
    .t_scale = -0.000164327854f,
    .t_intercept = 321.9705002,
    .vibration_threshold = THRESHOLD,
    .vibration_sensitivity = SENSITIVITY,
    .idle_hook_enable = 0,
    .mud_pulse_cfg = {
        .timer_hz = 100,              // 100Hz定时器频率
        .no_vibration_time = 60,      // 60秒无振动时间
        .group_interval = 60,         // 60秒组间隔
        .send_delay = 60,             // 60秒发送延时
        .max_retry_count = 1,         // 1次重试
        .number_of_groups = 3,        // 3组发送
        .static_collection_time = 30, // 30秒静态数据采集时间
        .auto_send_period = 10800       // 10800秒定时发送时间
    },
    .calibration_data = 0.0f
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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.10.15 00:02:01 Sunday
  *******************************************************************************
  */
int32_t is25pl032_flash_reset_rd_index(void)
{
    log_context.log_to_be_read_count = log_context.log_total_count;
    return is25pl032_flash_update_context();
}
/**
  *******************************************************************************
  * @Description:
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
    log_context.log_to_be_read_count = log_context.log_total_count;
    return is25pl032_flash_update_context();
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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

        rd_index = log_context.log_write_index + 1 - log_context.log_to_be_read_count;
        log_context.log_to_be_read_count--;
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
            return 1;
    }
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 19:56:44 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_write_one_log(log_t *log)
{
    int32_t i;
    uint32_t init_flag;
    int32_t ret = 0;

    log->crc16 = CRC16(log, sizeof(log_t) - 2);
    for (i = 0; i < 3; i++)
    {
        uint32_t flash_wr_address = LOG_FLASH_ADDRESS;
        log_t log_rd;

        log_context.log_write_index++;
        // if(log_context.log_write_index >= log_context.log_total_count > LOG_FLASH_SIZE / sizeof(log_t))
        if (log_context.log_write_index >= LOG_FLASH_SIZE / sizeof(log_t))
            log_context.log_write_index = 0;
        flash_wr_address += log_context.log_write_index * sizeof(log_t);
        is25pl032_flash_normal_write(flash_wr_address, (uint8_t *)log, sizeof(log_t));

        // READ BACK
        is25pl032_flash_normal_read(flash_wr_address, (uint8_t *)&log_rd, sizeof(log_t));
        if (memcmp(&log_rd, log, sizeof(log_t)) == 0)
            break;
    }

    if (i == 3)
        return -1;

    // 成功写入一条log
    log_context.log_total_count++;
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

        is25pl032_flash_normal_write(flash_wr_address, (uint8_t *)&log_context, sizeof(log_context_t));

        // READ BACK
        is25pl032_flash_normal_read(flash_wr_address, (uint8_t *)&rd_back, sizeof(log_context_t));
        if (memcmp(&rd_back, &log_context, sizeof(log_context_t)) == 0)
            break;
    }
    if (i == 3)
        return -2;

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
            printf("FLASH_INITED_FLAG is correct (0x%08X)\r\n", init_flag);
            break;
        }
    }

    if (i == 3)
    {
        printf("Failed to restore FLASH_INITED_FLAG after 3 attempts\r\n");
        return -3;
    }

    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 11:00:09 Wednesday
  *******************************************************************************
  */
int32_t is25pl032_flash_read_id(void)
{
    uint8_t cmd_data[8];

    cmd_data[0] = FLASH_RDMDID;
    spi0_cs0_transfer(cmd_data, cmd_data, 6);
    return cmd_data[3] << 16 | cmd_data[4] << 8 | cmd_data[5];
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.15 23:10:55 Friday
  *******************************************************************************
  */
static int32_t is25pl032_flash_write_in_page(uint32_t flash_address, uint8_t *data, uint32_t len)
{
    uint32_t cmd_address_data[260 / 4];
    uint8_t *p_cmd_address = (uint8_t *)cmd_address_data;
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
        printf("flash_write_in_page 0x%08x failed!\r\n", flash_address);
        return -3;
    }

    return 0;
}

/**
  *******************************************************************************
  * @Description:
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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
        len -= writed_bytes;
        flash_address += writed_bytes;
        data_buffer += writed_bytes;
        is25pl032_flash_write_page(flash_address, data_buffer, writed_bytes);
    }

    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
        return 0;

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
  * @Description:
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

    // 检查参数
    is25pl032_flash_normal_read(DEVICE_CFG_FLASH_ADDRESS, (uint8_t *)flash_temp_buffer, sizeof(flash_temp_buffer));
    p_dev_config = (DEV_CFG_T *)flash_temp_buffer;
    dev_cfg_max_sn = 0;
    for (int32_t i = 0; i < 4; i++, p_dev_config++)
    {
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

    // 确定运行时间最后记录
    for (int i = 0, j = 0; i < 2; i++)
    {
        uint16_t crc;

        is25pl032_flash_normal_read(TOTAL_RUNTIME_FLASH_ADDRESS + 0x1000 * i, (uint8_t *)flash_temp_buffer, sizeof(flash_temp_buffer));
        p_runtime_record = (total_runtime_record_t *)flash_temp_buffer;

        for (p_runtime_record = (total_runtime_record_t *)flash_temp_buffer; (uint32_t)p_runtime_record < (uint32_t)flash_temp_buffer + 0x1000;
             p_runtime_record++, j++)
        {
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

    // 检查LOG上下文
    log_context.log_tag = LOG_CONTEXT_TAG;
    log_context.log_write_index = -1;
    log_conext_max_sn = 0;
    for (int i = 0; i < 2; i++)
    {
        log_context_t *p_log_context;

        p_log_context = (log_context_t *)flash_temp_buffer;
        is25pl032_flash_normal_read(LOG_CONTEXT_FLASH_ADDRESS + 0x1000 * i, (uint8_t *)flash_temp_buffer, sizeof(flash_temp_buffer));
        for (int32_t j = 0; (uint32_t)p_log_context < (uint32_t)flash_temp_buffer + sizeof(flash_temp_buffer); p_log_context++, j++)
        {
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
            log_context_index = j + i * 0x1000 / 16;
        }
    }

    printf("There are %d logs in flash.\r\n", log_context.log_to_be_read_count);
    return 0;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

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
  * @Description:
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
  * @Description:
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

void is25pl032_flash_set_param(uint8_t accfir, uint8_t gyrofir, double xr_limit, double yr_limit, double gain)
{
    dev_cfg.u_cfg.cfg.xr_limit = xr_limit;
    dev_cfg.u_cfg.cfg.yr_limit = yr_limit;
    is25pl032_flash_save_dev_cfg();
}

void is25pl032_flash_get_param(uint8_t *accfir, uint8_t *gyrofir, double *xr_limit, double *yr_limit, double *gain)
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
    memcpy(version, "DMC250429_04.2", sizeof(dev_cfg.u_cfg.cfg.version));
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.23 23:15:18 Friday
  *******************************************************************************
  */
// float get_iam_20680ht_b2_kp(void)
//{
//     return dev_cfg.u_cfg.cfg.iam_20680ht_b2_kp;
// }
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.23 23:16:21 Friday
  *******************************************************************************
  */
// float get_iam_20680ht_b3_ki(void)
//{
//     return dev_cfg.u_cfg.cfg.iam_20680ht_b3_ki;
// }
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.23 23:16:37 Friday
  *******************************************************************************
  */
// float get_iam_20680ht_b4_limiting(void)
//{
//     return dev_cfg.u_cfg.cfg.iam_20680ht_b4_limiting;
// }
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.23 23:17:05 Friday
  *******************************************************************************
  */
// float get_iam_20680ht_b5_max_rating(void)
//{
//     return dev_cfg.u_cfg.cfg.iam_20680ht_b5_max_rating;
// }

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.23 23:32:13 Friday
  *******************************************************************************
  */
// float get_iam_20680ht_acc_zero_offset_x(void)
//{
//     return dev_cfg.u_cfg.cfg.iam_20680ht_acc_offset_x;
// }
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.23 23:32:33 Friday
  *******************************************************************************
  */
// float get_iam_20680ht_acc_zero_offset_y(void)
//{
//     return dev_cfg.u_cfg.cfg.iam_20680ht_acc_offset_y;
// }
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.23 23:32:51 Friday
  *******************************************************************************
  */
// float get_iam_20680ht_acc_zero_offset_z(void)
//{
//     return dev_cfg.u_cfg.cfg.iam_20680ht_acc_offset_z;
// }
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
 * @Description: 设置泥浆脉冲数据重传次数
 * @Parameters : count - 重传次数
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置泥浆脉冲数据有效范围的重传次数
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_retry_count(uint32_t count)
{
    dev_cfg.u_cfg.cfg.mud_pulse_cfg.max_retry_count = count;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取泥浆脉冲数据重传次数
 * @Parameters : 无
 * @RetValue   : 泥浆脉冲数据重传次数
 * @Note       : 获取泥浆脉冲数据有效范围的重传次数
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_retry_count(void)
{
    return dev_cfg.u_cfg.cfg.mud_pulse_cfg.max_retry_count;
}

/**
 *******************************************************************************
 * @Description: 设置每组泥浆脉冲数据的间隔
 * @Parameters : Pulse_group_interval - 时间间隔
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置每组泥浆脉冲数据的有效间隔
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_Pulse_group_interval(uint32_t Pulse_group_interval)
{
    dev_cfg.u_cfg.cfg.mud_pulse_cfg.group_interval = Pulse_group_interval;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取每组泥浆脉冲数据的间隔
 * @Parameters : 无
 * @RetValue   : 每组泥浆脉冲数据的间隔
 * @Note       : 获取每组泥浆脉冲数据的间隔
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_Pulse_group_interval(void)
{
    return dev_cfg.u_cfg.cfg.mud_pulse_cfg.group_interval;
}

/**
 *******************************************************************************
 * @Description: 设置泥浆脉冲数据发送延时时间
 * @Parameters : Pulse_send_delay - 延时时间
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置泥浆脉冲数据发送延时时间
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_Pulse_send_delay(uint32_t Pulse_send_delay)
{
    dev_cfg.u_cfg.cfg.mud_pulse_cfg.send_delay = Pulse_send_delay;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取泥浆脉冲数据发送延时时间
 * @Parameters : 无
 * @RetValue   : 泥浆脉冲数据发送延时时间
 * @Note       : 获取泥浆脉冲数据发送延时时间
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_Pulse_send_delay(void)
{
    return dev_cfg.u_cfg.cfg.mud_pulse_cfg.send_delay;
}

/**
 *******************************************************************************
 * @Description: 设置泥浆脉冲数据定时发送时间
 * @Parameters : Pulse_auto_send - 定时发送时间
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置泥浆脉冲数据定时发送时间
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_Pulse_auto_send(uint32_t Pulse_auto_send)
{
    dev_cfg.u_cfg.cfg.mud_pulse_cfg.auto_send_period = Pulse_auto_send;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取泥浆脉冲数据定时发送时间
 * @Parameters : 无
 * @RetValue   : 泥浆脉冲数据定时发送时间
 * @Note       : 获取泥浆脉冲数据定时发送时间
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_Pulse_auto_send(void)
{
    return dev_cfg.u_cfg.cfg.mud_pulse_cfg.auto_send_period;
}

/**
 *******************************************************************************
 * @Description: 设置静态数据收集的时间
 * @Parameters : Static_data_collection - 静态数据收集的时间
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置静态数据收集的时间
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_Static_data_collection(uint32_t Static_data_collection)
{
    dev_cfg.u_cfg.cfg.mud_pulse_cfg.static_collection_time = Static_data_collection;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取静态数据收集的时间
 * @Parameters : 无
 * @RetValue   : 静态数据收集的时间
 * @Note       : 获取静态数据收集的时间
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_Static_data_collection(void)
{
    return dev_cfg.u_cfg.cfg.mud_pulse_cfg.static_collection_time;
}

/**
 *******************************************************************************
 * @Description: 设置泥浆脉冲发送的组数
 * @Parameters : Number_of_pluse_group - 泥浆脉冲发送的组数
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置泥浆脉冲发送的组数
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_Number_of_pluse_group(uint32_t Number_of_pluse_group)
{
    dev_cfg.u_cfg.cfg.mud_pulse_cfg.number_of_groups = Number_of_pluse_group;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取泥浆脉冲发送的组数
 * @Parameters : 无
 * @RetValue   : 泥浆脉冲发送的组数
 * @Note       : 获取泥浆脉冲发送的组数
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_Number_of_pluse_group(void)
{
    return dev_cfg.u_cfg.cfg.mud_pulse_cfg.number_of_groups;
}

/**
 *******************************************************************************
 * @Description: 设置ADXL357振动阈值数据
 * @Parameters : vibration_threshold - ADXL357振动阈值
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置ADXL357振动阈值数据
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_vibration_threshold(float vibration_threshold)
{
    dev_cfg.u_cfg.cfg.vibration_threshold = vibration_threshold;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取ADXL357振动阈值数据
 * @Parameters : 无
 * @RetValue   : ADXL357振动阈值数据
 * @Note       : 获取ADXL357振动阈值数据
 *******************************************************************************
 */
float is25pl032_flash_get_vibration_threshold(void)
{
    return dev_cfg.u_cfg.cfg.vibration_threshold;
}

/**
 *******************************************************************************
 * @Description: 设置ADXL357振动灵敏度
 * @Parameters : vibration_sensitivity - 灵敏值
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 设置ADXL357振动灵敏度
 *******************************************************************************
 */
uint32_t is25pl032_flash_set_vibration_sensitivity(uint32_t vibration_sensitivity)
{
    dev_cfg.u_cfg.cfg.vibration_sensitivity = vibration_sensitivity;
    return is25pl032_flash_save_dev_cfg();
}

/**
 *******************************************************************************
 * @Description: 获取ADXL357振动灵敏度
 * @Parameters : 无
 * @RetValue   : 0-成功，-1-失败
 * @Note       : 获取ADXL357振动灵敏度
 *******************************************************************************
 */
uint32_t is25pl032_flash_get_vibration_sensitivity(void)
{
    return dev_cfg.u_cfg.cfg.vibration_sensitivity;
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
