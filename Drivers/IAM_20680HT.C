/**
 *****************************************************************************
 * FILENAME   : IAM_20680HT.c
 * COPYRIGHT  : MD.Tec(ShangHai) Co.,Ltd2024.
 * CREATEDDATE: 2025.04.07 10:30:00 Monday
 * DESCRIPTION: IAM-20680HT传感器驱动模块，负责初始化和数据采集
 *
 *
 * EDIT HISTORY:
 *   NAME                        DATE                      CONTENT
 * NickYang                  2024.02.19                    CREATE
 * AI Assistant              2024.06.01                    重构代码，移除信号处理逻辑
 * AI Assistant              2025.04.07                    更新注释和日期
 * AI Assistant              2025.04.07                    使用s_iam_global_raw_data作为全局变量
 *****************************************************************************
 */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"  // 添加main.h，它包含了spi.h
#include "IAM_20680HT.h"
#include "semphr.h"  // 添加信号量头文件

#define SMPLRT_DIV 0x19
#define CONFIG 0x1a
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1c
#define ACCEL_CONFIG_2 0x1d
#define FIFO_EN 0x23
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3a
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define USER_CTRL 0x6a
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0X75
#define PWR_MGMT_1 0X6B

#define DEVICE_ID 0XFA
#define ROOMTEMP_OFFSET 0x0
#define TEMP_SENSITIVITY 326.8f

#define EVENT_IAM_20680HT_10MS_TIMER 0X08000000
const uint32_t EVENT_IAM_20680HT_DATA_READY[4] = {0X80000000, 0X40000000, 0X20000000, 0X10000000};

TaskHandle_t iam_20680ht_task_handle;

// raw data direclty read from sensor
static int16_t iam_raw_temp;
static int16_t iam_raw_gyro_x;
static int16_t iam_raw_gyro_y;
static int16_t iam_raw_gyro_z;
static int16_t iam_raw_acc_x;
static int16_t iam_raw_acc_y;
static int16_t iam_raw_acc_z;
static iam_global_raw_data_t s_iam_local_raw_data[4];

static GPIO_Type           *iam_power_port[4];
static pins_channel_type_t  iam_power_pin[4];

static GPIO_Type           *iam_comm_cs_port[2];
static pins_channel_type_t  iam_comm_cs_pin[2];

// 全局变量，供其他文件直接访问
iam_global_raw_data_t s_iam_global_raw_data = {0};

/******************************** Functions **********************************/


/**
  *******************************************************************************
  * @Description: 向IAM-20680HT传感器写入寄存器数据
  * @Parameters : reg - 寄存器地址
  *               data - 要写入的数据
  *               check - 是否检查写入结果
  * @RetValue   : 0表示成功，-1表示失败
  * @Note       : 通过SPI接口与传感器通信
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
static int32_t iam_20680ht_write_reg(uint8_t cs_num, uint32_t reg, uint32_t data, uint32_t check)
{
    uint32_t cmd_address_data;

    /*写数据*/
    cmd_address_data = reg & 0X7f;
    cmd_address_data |= (data & 0xff) << 8;
    spi1_transfer(cs_num, (uint8_t *)&cmd_address_data, (uint8_t *)&cmd_address_data, 2);

    if (check)
    {
        uint8_t rd_val;
        rd_val = iam_20680ht_read_reg(cs_num, reg);
        if (rd_val != data)
        {
            /* printf("IAM 20680HT: Write 0x%02x to reg(0x%02x) failed! reg_val:0x%02x\r\n", data, reg, rd_val); */
            return -1;
        }
        // printf("IAM 20680HT: Write 0x%02x to reg(0x%02x) OK!\r\n", data, reg);
    }
    return 0;
}

/**
  *******************************************************************************
  * @Description: IAM-20680HT数据就绪中断处理函数
  * @Parameters : int_flag - 中断标志
  * @RetValue   : 无
  * @Note       : 在中断上下文中通知任务处理新数据
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
static void iam_20680ht_data_ready_isr_0(void)
{
    xTaskGenericNotifyFromISR(iam_20680ht_task_handle, EVENT_IAM_20680HT_DATA_READY[0], eSetBits, NULL, NULL);
}

static void iam_20680ht_data_ready_isr_1(void)
{
    xTaskGenericNotifyFromISR(iam_20680ht_task_handle, EVENT_IAM_20680HT_DATA_READY[1], eSetBits, NULL, NULL);
}

static void iam_20680ht_data_ready_isr_2(void)
{
    xTaskGenericNotifyFromISR(iam_20680ht_task_handle, EVENT_IAM_20680HT_DATA_READY[2], eSetBits, NULL, NULL);
}

static void iam_20680ht_data_ready_isr_3(void)
{
    xTaskGenericNotifyFromISR(iam_20680ht_task_handle, EVENT_IAM_20680HT_DATA_READY[3], eSetBits, NULL, NULL);
}

/**
  *******************************************************************************
  * @Description: IAM-20680HT传感器初始化
  * @Parameters : 无
  * @RetValue   : 0表示成功，-1表示失败
  * @Note       : 配置传感器工作模式、采样率等参数
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
static int32_t iam_20680ht_init(uint8_t cs_num)
{
//    const char *info[2] = {"NOT ", ""};
    uint32_t found = 0;
    uint8_t reg_val;

    /*
    复位芯片
    PWR_MGMT_1
    Bit7: DEVICE_RESET
    Bit6: SLEEP
    Bit5: ACCEL_CYCLE
    Bit4: GYRO_STANDBY
    Bit3: TEMP_DIS
    Bit2..0: CLKSEL[2:0]
    */
    iam_20680ht_write_reg(cs_num, PWR_MGMT_1, 0X80, 0);
    vTaskDelay(1);
    reg_val = iam_20680ht_read_reg(cs_num, PWR_MGMT_1);
    //printf("IAM 20680HT: PWR_MGMT_1 = 0x%02x\r\n", reg_val);
    vTaskDelay(1);

    /*
    读ID寄存器
    */
    for (int i = 0; i < 10; i++)
    {
        reg_val = iam_20680ht_read_reg(cs_num, WHO_AM_I);
        if (reg_val == DEVICE_ID)
        {
            found = 1;
            break;
        }
        vTaskDelay(1);
    }
    /* printf("%sFound IAM_20680HT! ID=0x%02x\r\n", info[found], reg_val); */

    /*
    Bit7:    -
    Bit6:    FIFO_MODE, 0--Overwrite 1--Discard
    Bit5..3: EXT_SYNC_SET[2:0]
    Bit2..0: DLPF_CFG[2:0]
    */
    reg_val = 6; // set the corner frequency of the DLPF of the gyroscope to 5 Hz
    iam_20680ht_write_reg(cs_num, CONFIG, reg_val, 1);

    /*
    [7:6] FIFO_SIZE[1:0]
    Specifies FIFO size according to the following:
    0 = 512 Byte
    1 = 1 kByte
    2 = 2 kByte
    3 = 4 kByte
    [5:4] DEC2_CFG[1:0]
    Averaging filter settings for WoM Accelerometer Mode:
    0 = Average 4 samples
    1 = Average 8 samples
    2 = Average 16 samples
    3 = Average 32 samples
    [3]     ACCEL_FCHOICE_B Used to bypass DLPF as shown in the table below.
    [2:0] A_DLPF_CFG Accelerometer low pass filter setting as shown in the table below

    ACCEL_FCHOICE_B     A_DLPF_CFG      3-dB BW(Hz)    NoiseBW(Hz)    Rate(kHz)
    1                       X           1046.0          1100.0          4
    0                       0           218.1           235.0           1
    0                       1           218.1           235.0           1
    0                       2           99.0            121.3           1
    0                       3           44.8            61.5            1
    0                       4           21.2            31.0            1
    0                       5           10.2            15.5            1
    0                       6           5.1             7.8             1
    0                       7           420.0           441.6           1
    */
    reg_val = 6; // set the frequency of the DLPF of the  to 5 Hz
    iam_20680ht_write_reg(cs_num, ACCEL_CONFIG_2, reg_val, 1);

    /*
    Bit7: XG_ST
    Bit6: YG_ST
    Bit5: ZG_ST
    Bit4..3: FS_SEL [1:0], Gyro Full Scale Select:
            00 = ±250 dps
            01 = ±500 dps
            10 = ±1000 dps
            11 = ±2000 dps
    Bit2: -
    Bit1..0: FCHOICE_B[1:0]
    */

    reg_val = iam_20680ht_read_reg(cs_num, GYRO_CONFIG);
    reg_val &= ~0x3 << 3;
    reg_val |= 0x3 << 3; // GYO量程：±2000 dps
    iam_20680ht_write_reg(cs_num, GYRO_CONFIG, reg_val, 1);
    reg_val = iam_20680ht_read_reg(cs_num, GYRO_CONFIG);

    /*Output Data Rate Selection
    SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    Where INTERNAL_SAMPLE_RATE = 1 kHz
    数据输出率：250HZ
    */
    iam_20680ht_write_reg(cs_num, SMPLRT_DIV, 3, 1);

    /*
    ACC_CONFIG: 0X1C
    Bit7: XA_ST
    Bit6: YA_ST
    Bit5: ZA_ST
    Bit4..3: ACCEL_FS_SEL[1:0]
    */
    reg_val = 2 << 3; // ±8G
    iam_20680ht_write_reg(cs_num, ACCEL_CONFIG, reg_val, 1);
    /*
    Bit7: TEMP_FIFO_EN
    Bit6: XG_FIFO_EN
    Bit5: YG_FIFO_EN
    Bit4: ZG_FIFO_EN
    Bit3: ACCEL_FIFO_EN
    Bit2: -
    Bit1: -
    Bit0: -
    */
    reg_val = 0xf8;
    iam_20680ht_write_reg(cs_num, FIFO_EN, reg_val, 1);

    /*
    USER_CTRL:0x6a
    Bit7: -
    Bit6: FIFO_EN
    Bit5: -
    Bit4: I2C_IF_DIS
    Bit3: -
    Bit2: FIFO_RST
    Bit1: -
    Bit0: SIG_COND_RST
    */
    reg_val = 0x1 << 6;
    iam_20680ht_write_reg(cs_num, USER_CTRL, reg_val, 1);

    /*
    中断引脚配置
    Bit7: INT_LEVEL 1 – The logic level for INT/INT2 pin is active low.
    Bit6: INT_OPEN  1 – INT/INT2 pin is configured as open drain.
    Bit5: LATCH _INT_EN 1 – INT/INT2 pin level held until interrupt status is cleared.
    Bit4: INT_RD _CLEAR 0 – Interrupt status is cleared only by reading INT_STATUS register
    Bit3: FSYNC_INT_LEVEL
    Bit2: FSYNC _INT_MODE_EN
    Bit1: -
    Bit0: INT2_EN
    */
    reg_val = 0xc0;
    iam_20680ht_write_reg(cs_num, INT_PIN_CFG, reg_val, 1);

    /*
    中断使能
    Bit7..5: WOM_INT_EN[2:0]
    Bit4:    FIFO_OFLOW_EN
    Bit3:    -
    Bit2:    GDRIVE_INT_EN
    Bit1:    -
    Bit0:    DATA_RDY_INT_EN
    */
    reg_val = 0x11;
    iam_20680ht_write_reg(cs_num, INT_ENABLE, reg_val, 1);

    return found;
}

/**
  *******************************************************************************
  * @Description: 从IAM-20680HT传感器读取寄存器数据
  * @Parameters : reg - 寄存器地址
  * @RetValue   : 读取到的寄存器值
  * @Note       : 通过SPI接口与传感器通信
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
uint8_t iam_20680ht_read_reg(uint8_t cs_num, uint32_t reg)
{
    uint32_t cmd_address_data;
    uint8_t rd_val;

    /*读数据*/
    cmd_address_data = reg | 0x80;
    spi1_transfer(cs_num, (uint8_t *)&cmd_address_data, (uint8_t *)&cmd_address_data, 2);
    rd_val = cmd_address_data >> 8;

    return rd_val;
}

/**
  *******************************************************************************
  * @Description: IAM-20680HT传感器数据读取事件处理
  * @Parameters : 无
  * @RetValue   : 无
  * @Note       : 从传感器读取原始数据并更新全局变量
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
static void iam_20680ht_on_rd_event(uint8_t index)
{
    static uint8_t cmd_data[512 + 2]; // FIFO默认长度是512字节
    uint16_t fifo_counth, fifo_countl, int_status;
    uint16_t fifo_count;
    static int16_t last_adc_temp;
    int16_t current_temp, temp_diff;
    uint8_t local_index = index;

    /*
    中断状态寄存器
    Bit7..5: WOM_INT[2:0]
    Bit4: FIFO _OFLOW _INT
    Bit3: -
    Bit2: GDRIVE_INT
    Bit1: -
    Bit0: DATA _RDY_INT
    */
    int_status = (INT_STATUS | 0x80);
    spi1_transfer(local_index, (uint8_t *)&int_status, (uint8_t *)&int_status, 2);
    if ((int_status & 0x1100) == 0)
        return;

    // 读取FIFO数据长度
    fifo_counth = (FIFO_COUNTH | 0x80);
    spi1_transfer(local_index, (uint8_t *)&fifo_counth, (uint8_t *)&fifo_counth, 2);
    fifo_countl = (FIFO_COUNTL | 0x80);
    spi1_transfer(local_index, (uint8_t *)&fifo_countl, (uint8_t *)&fifo_countl, 2);

    fifo_count = (fifo_counth & 0xff00) | (fifo_countl >> 8 & 0xff);
    fifo_count++; // 第一个字节为寄存器地址
    if (fifo_count > sizeof(cmd_data))
        fifo_count = sizeof(cmd_data);

    // 读取数据
    cmd_data[0] = (FIFO_R_W | 0x80);
    spi1_transfer(local_index, (uint8_t *)cmd_data, (uint8_t *)&cmd_data, fifo_count);

    // Convention: NED
    // 先计算温度，如果与上次的值相差较大，则丢弃本次采样值，以上次的值为准。
    current_temp = cmd_data[7] << 8 | cmd_data[8];
    temp_diff = current_temp - last_adc_temp;
    if (-(int32_t)TEMP_SENSITIVITY <= temp_diff && temp_diff <= (int32_t)TEMP_SENSITIVITY)
    {
        iam_raw_acc_z = cmd_data[1] << 8 | cmd_data[2];
        iam_raw_acc_x = cmd_data[3] << 8 | cmd_data[4];
        iam_raw_acc_y = cmd_data[5] << 8 | cmd_data[6];
        iam_raw_temp = cmd_data[7] << 8 | cmd_data[8];
        iam_raw_gyro_z = cmd_data[9] << 8 | cmd_data[10];
        iam_raw_gyro_z = -iam_raw_gyro_z;
        iam_raw_gyro_x = cmd_data[11] << 8 | cmd_data[12];
        iam_raw_gyro_y = cmd_data[13] << 8 | cmd_data[14];
        iam_raw_gyro_y = -iam_raw_gyro_y;
    }
    last_adc_temp = current_temp;

    // 更新全局变量结构体，供其他模块使用
    // 暂停所有任务，保护数据访问
 //   vTaskSuspendAll();
    
    s_iam_local_raw_data[local_index].t_raw = iam_raw_temp;
    s_iam_local_raw_data[local_index].gx_raw = iam_raw_gyro_x;
    s_iam_local_raw_data[local_index].gy_raw = iam_raw_gyro_y;
    s_iam_local_raw_data[local_index].gz_raw = iam_raw_gyro_z;
    s_iam_local_raw_data[local_index].ax_raw = iam_raw_acc_x;
    s_iam_local_raw_data[local_index].ay_raw = iam_raw_acc_y;
    s_iam_local_raw_data[local_index].az_raw = iam_raw_acc_z;

    s_iam_global_raw_data.t_raw = s_iam_local_raw_data[local_index].t_raw;
    s_iam_global_raw_data.gx_raw = s_iam_local_raw_data[local_index].gx_raw;
    s_iam_global_raw_data.gy_raw = s_iam_local_raw_data[local_index].gy_raw;
    s_iam_global_raw_data.gz_raw = s_iam_local_raw_data[local_index].gz_raw;
    s_iam_global_raw_data.ax_raw = s_iam_local_raw_data[local_index].ax_raw;
    s_iam_global_raw_data.ay_raw = s_iam_local_raw_data[local_index].ay_raw;
    s_iam_global_raw_data.az_raw = s_iam_local_raw_data[local_index].az_raw;
    
    // 恢复所有任务
 //   xTaskResumeAll();
}

/**
  *******************************************************************************
  * @Description: 10ms定时器回调函数
  * @Parameters : xTimer - 定时器句柄
  * @RetValue   : 无
  * @Note       : 通知任务处理定时事件
  * @CreatedBy  : NY
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
static void iam_10ms_timer_cb(TimerHandle_t xTimer)
{
    xTaskGenericNotify(iam_20680ht_task_handle, EVENT_IAM_20680HT_10MS_TIMER, eSetBits, NULL);
}

static void iam_power_register(void)
{
    iam_power_port[0] = PTA;
    iam_power_pin[0] = 0;

    iam_power_port[1] = PTA;
    iam_power_pin[1] = 1;

    iam_power_port[2] = PTB;
    iam_power_pin[2] = 12;

    iam_power_port[3] = PTB;
    iam_power_pin[3] = 13;

    iam_comm_cs_port[0] = PTD;
    iam_comm_cs_pin[0] = 15;

    iam_comm_cs_port[1] = PTD;
    iam_comm_cs_pin[1] = 16;
}

static void iam_power_on(uint8_t index)
{
    switch(index)
    {
        case 0:
            PINS_DRV_WritePin(iam_power_port[0], iam_power_pin[0], 0);
            PINS_DRV_WritePin(iam_power_port[1], iam_power_pin[1], 1);
            PINS_DRV_WritePin(iam_power_port[2], iam_power_pin[2], 1);
            PINS_DRV_WritePin(iam_power_port[3], iam_power_pin[3], 1);
            PINS_DRV_WritePin(iam_comm_cs_port[0], iam_comm_cs_pin[0], 0);
            PINS_DRV_WritePin(iam_comm_cs_port[1], iam_comm_cs_pin[1], 0);
        break;
        case 1:
            PINS_DRV_WritePin(iam_power_port[0], iam_power_pin[0], 1);
            PINS_DRV_WritePin(iam_power_port[1], iam_power_pin[1], 0);
            PINS_DRV_WritePin(iam_power_port[2], iam_power_pin[2], 1);
            PINS_DRV_WritePin(iam_power_port[3], iam_power_pin[3], 1);
            PINS_DRV_WritePin(iam_comm_cs_port[0], iam_comm_cs_pin[0], 1);
            PINS_DRV_WritePin(iam_comm_cs_port[1], iam_comm_cs_pin[1], 0);
        break;
        case 2:
            PINS_DRV_WritePin(iam_power_port[0], iam_power_pin[0], 1);
            PINS_DRV_WritePin(iam_power_port[1], iam_power_pin[1], 1);
            PINS_DRV_WritePin(iam_power_port[2], iam_power_pin[2], 0);
            PINS_DRV_WritePin(iam_power_port[3], iam_power_pin[3], 1);
            PINS_DRV_WritePin(iam_comm_cs_port[0], iam_comm_cs_pin[0], 0);
            PINS_DRV_WritePin(iam_comm_cs_port[1], iam_comm_cs_pin[1], 1);
        break;
        case 3:
            PINS_DRV_WritePin(iam_power_port[0], iam_power_pin[0], 1);
            PINS_DRV_WritePin(iam_power_port[1], iam_power_pin[1], 1);
            PINS_DRV_WritePin(iam_power_port[2], iam_power_pin[2], 1);
            PINS_DRV_WritePin(iam_power_port[3], iam_power_pin[3], 0);
            PINS_DRV_WritePin(iam_comm_cs_port[0], iam_comm_cs_pin[0], 1);
            PINS_DRV_WritePin(iam_comm_cs_port[1], iam_comm_cs_pin[1], 1);
        break;
        default:
            PINS_DRV_WritePin(iam_power_port[0], iam_power_pin[0], 1);
            PINS_DRV_WritePin(iam_power_port[1], iam_power_pin[1], 1);
            PINS_DRV_WritePin(iam_power_port[2], iam_power_pin[2], 1);
            PINS_DRV_WritePin(iam_power_port[3], iam_power_pin[3], 1);
            PINS_DRV_WritePin(iam_comm_cs_port[0], iam_comm_cs_pin[0], 0);
            PINS_DRV_WritePin(iam_comm_cs_port[1], iam_comm_cs_pin[1], 0);
        break;
    }
}

/**
  *******************************************************************************
  * @Description: IAM-20680HT传感器任务
  * @Parameters : p - 任务参数
  * @RetValue   : 无
  * @Note       : 负责传感器初始化、数据采集和处理
  * @CreatedBy  : NickYang
  * @CreatedDate: 2025.04.07
  *******************************************************************************
  */
void iam_20680ht_task(void *p)
{
//    uint8_t i;
    static uint8_t current_sensor = 3;  // 当前使用的传感器索引 0-3
//    static uint32_t switch_counter = 0;  // 切换计数器
//    const uint32_t SWITCH_INTERVAL_MS = 10000;  // 切换间隔10秒
//    const uint32_t TIMER_PERIOD_MS = 10;  // 定时器周期10ms
//    const uint32_t SWITCH_THRESHOLD = SWITCH_INTERVAL_MS / TIMER_PERIOD_MS;  // 切换阈值

    /*    注册中断回调函数    */
    gpio_porte_register_cb(3, iam_20680ht_data_ready_isr_0); //PE3
    gpio_porte_register_cb(0, iam_20680ht_data_ready_isr_1); //PE0
    gpio_portc_register_cb(8, iam_20680ht_data_ready_isr_2); //PC8
    gpio_porte_register_cb(2, iam_20680ht_data_ready_isr_3); //PE2

    iam_power_register();

    iam_power_on(current_sensor);
    vTaskDelay(10);  // 等待电源稳定
    iam_20680ht_init(current_sensor);

    xTimerStart(xTimerCreate("iam_10ms_timer", 10, 1, 0, iam_10ms_timer_cb), 1000);

    uint32_t start_timestamp = xTaskGetTickCount();

    for (;;)
    {
        uint32_t notify;
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        // 处理10ms定时器事件 - 用于传感器切换
        if (notify & EVENT_IAM_20680HT_10MS_TIMER)
        {
            if (notify & EVENT_IAM_20680HT_DATA_READY[current_sensor])
                iam_20680ht_on_rd_event(current_sensor);
//            for(i = 0; i < 4; i++)
//            {
//                if (notify & EVENT_IAM_20680HT_DATA_READY[i])
//                    iam_20680ht_on_rd_event(i);
//            }
//
//            switch_counter++;
//
//            // 到达切换时间
//            if (switch_counter >= SWITCH_THRESHOLD)
//            {
//                switch_counter = 0;
//                uint8_t next_sensor = (current_sensor + 1) % 4;
//
//                // 关闭当前传感器电源
//                iam_power_off(current_sensor);
//                // 打开新传感器电源
//                iam_power_on(next_sensor);
//                vTaskDelay(10);  // 等待电源稳定
//
//                // 初始化新传感器
//                iam_20680ht_init(next_sensor);
//                current_sensor = next_sensor;
//            }
        }
    }
}
START_TASK(iam_20680ht_task, "iam_20680ht_task", 256, NULL, TASK_PRIORITY_IAM, &iam_20680ht_task_handle);

