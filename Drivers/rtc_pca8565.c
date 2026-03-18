/**
  *****************************************************************************
  * FILENAME   : rtc_pca8565.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.10.03 17:04:48 Tuesday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.10.03                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "rtc_pca8565.h"

/*RTC*/
#define PCA8565_I2C_CHIP_ADDR     (0XA2 >> 1)
static uint8_t              rtc_rx_data_buffer[16];
static uint8_t              rtc_tx_data_buffer[8];
static volatile uint8_t     rtc_tx_data_valid;
static uint8_t              rtc_state_machine;              //状态机

#define SM_RTC_IDLE             0
#define SM_RTC_SET_DATE_TIME    1
#define SM_RTC_GET_DATE_TIME    2
/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.11 21:04:26 Monday
  *******************************************************************************
  */
static void pca8565_transfer_endded_ISR(void)
{
    if(rtc_state_machine == SM_RTC_GET_DATE_TIME)
    {
        LPI2C_DRV_MasterReceiveData(INST_LPI2C0, (uint8_t*)rtc_rx_data_buffer, 7, 1);
    }
    rtc_state_machine = SM_RTC_IDLE;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.03 17:09:11 Tuesday
  *******************************************************************************
  */
int32_t pca8565_init(void)
{
//    uint8_t         reg[16];
//    uint32_t        sec, min, hours, day, mon, year;

    i2c_register_end_transfer_cb(pca8565_transfer_endded_ISR);
    LPI2C_DRV_MasterSetSlaveAddr(INST_LPI2C0, PCA8565_I2C_CHIP_ADDR, 0);

// NickYang 2023.12.11 21:36:59 Mon
//  while(1)
//  {
//      reg[0] = 2;
//      LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C0, (uint8_t*)&reg, 1, 0, 1);
//      LPI2C_DRV_MasterReceiveDataBlocking(INST_LPI2C0, (uint8_t*)&reg, 8, 1, 10);
//      sec = (reg[0] >> 4 & 0x7) * 10 + (reg[0] & 0xf);
//      min = (reg[1] >> 4 & 0x7) * 10 + (reg[1] & 0xf);
//      hours = (reg[2] >> 4  & 0x3) * 10 + (reg[2] & 0xf);
//      day = (reg[3] >> 4  & 0x3) * 10 + (reg[3] & 0xf);
//      mon = (reg[5] >> 4  & 0x1) * 10 + (reg[5] & 0xf);
//      year = (reg[6] >> 4  & 0x1) * 10 + (reg[6] & 0xf);
//      printf("%02d-%02d-%02d %02d:%02d:%02d ", year, mon, day, hours, min, sec);
//
//      vTaskDelay(1000);
//  }
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.03 22:47:21 Tuesday
  *******************************************************************************
  */
int32_t PCA8565_set_data(uint32_t year, uint32_t mon, uint32_t day,
                         uint32_t hour, uint32_t min, uint32_t sec)
{
    uint8_t     tens_number;
    uint8_t     unit_number;

    rtc_tx_data_buffer[0] = 2;  //起始寄存器地址

    tens_number = sec / 10;
    unit_number = sec % 10;
    rtc_tx_data_buffer[1] = tens_number << 4 | unit_number;

    tens_number = min / 10;
    unit_number = min % 10;
    rtc_tx_data_buffer[2] = tens_number << 4 | unit_number;

    tens_number = hour / 10;
    unit_number = hour % 10;
    rtc_tx_data_buffer[3] = tens_number << 4 | unit_number;

    tens_number = day / 10;
    unit_number = day % 10;
    rtc_tx_data_buffer[4] = tens_number << 4 | unit_number;

    tens_number = mon / 10;
    unit_number = mon % 10;
    rtc_tx_data_buffer[6] = tens_number << 4 | unit_number;

    tens_number = year / 10;
    unit_number = year % 10;
    rtc_tx_data_buffer[7] = tens_number << 4 | unit_number;

    __disable_irq();
    memcpy(rtc_rx_data_buffer, rtc_tx_data_buffer + 1, 7);
    __enable_irq();

    rtc_tx_data_valid = 1;

    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.03 19:42:21 Tuesday
  *******************************************************************************
  */
int32_t PCA8565_get_data(uint8_t* data, uint32_t len)
{
    __disable_irq();
    memcpy(data, rtc_rx_data_buffer, len);
    __enable_irq();
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.12.11 21:10:37 Monday
  *******************************************************************************
  */
void PCA8565_on_timer_event(void)
{
    status_t    status;
    uint8_t     reg_address;

    if(rtc_tx_data_valid == 1)
    {
        //设置RTC时间
        rtc_state_machine = SM_RTC_SET_DATE_TIME;
        rtc_tx_data_valid = 0;
        status = LPI2C_DRV_MasterSendData(INST_LPI2C0, (uint8_t*)&rtc_tx_data_buffer, 8, 0);
    }
    else
    {
        //读取RTC时间
        rtc_state_machine = SM_RTC_GET_DATE_TIME;
        reg_address = 2;
        status = LPI2C_DRV_MasterSendData(INST_LPI2C0, (uint8_t*)&reg_address, 1, 0);
    }

    if(status != STATUS_SUCCESS)
    {
        /* printf("I2C busy!\r\n"); */
    }
}



