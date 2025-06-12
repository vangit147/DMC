/**
  *****************************************************************************
  * FILENAME   : ads1278.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2025.04.07 10:00:00 Monday
  * DESCRIPTION: ADS1278 ADC????,?????????????????
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.25                    CREATE
  * Grodon Li                  2025.04.06                    MODIFY - ???4??
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#include "adxl357.h"
#include "filtering_functions.h"

#include "fir_config.h"

#define REG_DEVID_AD        0X00
#define REG_DEVID_MST       0X01
#define REG_PARTID          0X02
#define REG_REVID           0X03
#define REG_STATUS          0X04
#define REG_TEMP2           0X06
#define REG_TEMP1           0X07
#define REG_XDATA3          0X08
#define REG_XDATA2          0X09
#define REG_XDATA1          0X0A
#define REG_YDATA3          0X0B
#define REG_YDATA2          0X0C
#define REG_YDATA1          0X0D
#define REG_ZDATA3          0X0E
#define REG_ZDATA2          0X0F
#define REG_ZDATA1          0X10
#define REG_FIFO            0X11
#define REG_INT_MAP         0X2A
#define REG_POWER_CTL       0x2d
#define REG_RESET           0X2F
#define EVENT_ADXL357_DATA_READY    0X80000000
#define ADXL357_FREQUENCY_256MS 0x0A   //256ms
#define ADXL357_FREQUENCY_128MS 0x09   //128ms
#define ADXL357_FREQUENCY_64MS 0x08   //64ms
#define ADXL357_FREQUENCY_32MS 0x07   //32ms
#define ADXL357_FREQUENCY_16MS 0x06   //16ms
#define ADXL357_FREQUENCY_8MS 0x05    //8ms
#define ADXL357_FREQUENCY_4MS 0x04    //4ms
#define ADXL357_FREQUENCY_2MS 0x03    //2ms
#define ADXL357_FREQUENCY_1MS 0x02    //1ms
#define ADXL357_FREQUENCY_05MS 0x01   //0.5ms
#define ADXL357_FREQUENCY_025MS 0x00   //0.25ms

static int32_t  raw_acc_x, raw_acc_y, raw_acc_z;
static float    adxl357_acc_x, adxl357_acc_y, adxl357_acc_z;

TaskHandle_t     adx357_task_handle;
adxl357_vibration_data_t vibration_data;
/******************************** Functions **********************************/
void adxl357_get_adc_data(float* x, float* y, float* z)
{
    *x = adxl357_acc_x;
    *y = adxl357_acc_y;
    *z = adxl357_acc_z;
}

void Reset_Vibration_Stats(void)
{
    vibration_data.account = 0;
    vibration_data.min_vibration = 99999;
    vibration_data.max_vibration = -99999;
    vibration_data.avg_vibration = 0;
}

void Get_Vibration_Data(void)
{
		SensorData accel;
		float v_data;
    adxl357_get_adc_data(&accel.x, &accel.y, &accel.z);

    v_data = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

    if (v_data < vibration_data.min_vibration)
		    vibration_data.min_vibration = v_data;
    if (v_data > vibration_data.max_vibration)
		    vibration_data.max_vibration = v_data;
		vibration_data.avg_vibration = (vibration_data.avg_vibration * vibration_data.account + v_data) / (vibration_data.account + 1);
		vibration_data.account++;
}
static uint8_t adxl357_read_reg(uint32_t reg)
{
    uint32_t     cmd_address_data;

    cmd_address_data = (reg << 1 | 1) & 0xff;
    spi0_cs2_transfer((uint8_t*)&cmd_address_data, (uint8_t*)&cmd_address_data, 2);
    return (cmd_address_data >> 8) & 0xff;
}
static int32_t adxl357_write_reg(uint32_t reg, uint32_t data)
{
    uint32_t     cmd_address_data;

    cmd_address_data = (reg << 1) & 0xff;
    cmd_address_data |= (data & 0xff) << 8;
    spi0_cs2_transfer((uint8_t*)&cmd_address_data, (uint8_t*)&cmd_address_data, 2);

    return 0;
}
static void adxl357_data_ready_isr(uint32_t int_flag)
{
    xTaskGenericNotifyFromISR(adx357_task_handle, EVENT_ADXL357_DATA_READY, eSetBits, NULL, NULL);
}
static int32_t adxl357_start(uint32_t start)
{
    uint8_t     temp;
    temp = adxl357_read_reg(REG_POWER_CTL) & 0xfe;
    if(!start)
        temp |= 0x1;    //Standy By mode
    adxl357_write_reg(REG_POWER_CTL, temp);
    //printf("ADX1357 POWER_CTL register: 0x%02x\r\n", adxl357_read_reg(REG_POWER_CTL));

    return 0;
}

static int32_t adxl357_init(void)
{
    uint8_t     temp;
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
        return -1;
    }

    gpio_porta_register_cb(adxl357_data_ready_isr);

    //RESET DEVICE
    adxl357_write_reg(REG_RESET, 0x52);
    vTaskDelay(10);
    adxl357_write_reg(REG_INT_MAP, 0X1);    //DATA_RDY interrupt enable on INT1
    printf("ADX1357 INT_MAP register: 0x%02x\r\n", adxl357_read_reg(REG_INT_MAP));
    vTaskDelay(10);
    adxl357_write_reg(0x28, ADXL357_FREQUENCY_1MS);
    printf("ADX1357 FREQUENCY: 1ms\r\n");
    vTaskDelay(10);

    adxl357_start(1);
    return 0;
}

static void on_adxl357_data_ready_event(void)
{
    uint8_t         transfer_buffer[12];
    int32_t         acc_x, acc_y, acc_z;

    transfer_buffer[0] = REG_TEMP2 << 1 | 1;
    spi0_cs2_transfer((uint8_t*)transfer_buffer, (uint8_t*)transfer_buffer, 12);

    acc_x = transfer_buffer[3] << 12 | transfer_buffer[4] << 4 | transfer_buffer[5] >> 4;
    if(acc_x & 0x80000)
        acc_x |= 0XFFF00000;


    acc_y = transfer_buffer[6] << 12 | transfer_buffer[7] << 4 | transfer_buffer[8] >> 4;
    if(acc_y & 0x80000)
        acc_y |= 0XFFF00000;

    acc_z = transfer_buffer[9] << 12 | transfer_buffer[10] << 4 | transfer_buffer[11] >> 4;
    if(acc_z & 0x80000)
        acc_z |= 0XFFF00000;

    raw_acc_x = acc_x;
    raw_acc_y = acc_y;
    raw_acc_z = acc_z;

    adxl357_acc_x = raw_acc_x * 10.0f / (1 << 19);
    adxl357_acc_y = raw_acc_y * 10.0f / (1 << 19);
    adxl357_acc_z = raw_acc_z * 10.0f / (1 << 19);
}

void adxl357_task(void* p)
{
    adxl357_init();
    Reset_Vibration_Stats();

    for(;;)
    {
        uint32_t notify;
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if(notify & EVENT_ADXL357_DATA_READY)
        {
            on_adxl357_data_ready_event();
            Get_Vibration_Data();
        }
    }
}
//START_TASK(adxl357_task, "adxl357_task", 256, NULL, TASK_PRIORITY_ADXL357, &adx357_task_handle);
