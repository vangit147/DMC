/**
  *****************************************************************************
  * FILENAME   : spi.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.24 23:46:06 Sunday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.24                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#include "spi.h"

void LPSPI0_IRQHandler(void);
void LPSPI1_IRQHandler(void);
void LPSPI2_IRQHandler(void);

static GPIO_Type           *spi1_cs0_port;
static pins_channel_type_t  spi1_cs0_pin;
static uint8_t             *spi1_cs0_send_buff;
static uint8_t             *spi1_cs0_receive_buff;
static uint32_t             spi1_cs0_transfer_len;
static SemaphoreHandle_t    spi1_cs0_available_sem;
static SemaphoreHandle_t    spi1_cs0_transfer_end_sem;

static GPIO_Type           *spi1_cs1_port;
static pins_channel_type_t  spi1_cs1_pin;
static uint8_t             *spi1_cs1_send_buff;
static uint8_t             *spi1_cs1_receive_buff;
static uint32_t             spi1_cs1_transfer_len;
static SemaphoreHandle_t    spi1_cs1_available_sem;
static SemaphoreHandle_t    spi1_cs1_transfer_end_sem;

static GPIO_Type           *spi1_cs2_port;
static pins_channel_type_t  spi1_cs2_pin;
static uint8_t             *spi1_cs2_send_buff;
static uint8_t             *spi1_cs2_receive_buff;
static uint32_t             spi1_cs2_transfer_len;
static SemaphoreHandle_t    spi1_cs2_available_sem;
static SemaphoreHandle_t    spi1_cs2_transfer_end_sem;

static GPIO_Type           *spi1_cs3_port;
static pins_channel_type_t  spi1_cs3_pin;
static uint8_t             *spi1_cs3_send_buff;
static uint8_t             *spi1_cs3_receive_buff;
static uint32_t             spi1_cs3_transfer_len;
static SemaphoreHandle_t    spi1_cs3_available_sem;
static SemaphoreHandle_t    spi1_cs3_transfer_end_sem;

static void (*spi0_end_transfer)(void);
static void (*spi1_end_transfer)(void);
static void (*spi2_end_transfer)(void);

#define SPI1_CS0_TRANSFER_EVENT    0X80000000
#define SPI1_CS1_TRANSFER_EVENT    (SPI1_CS0_TRANSFER_EVENT >> 1)
#define SPI1_CS2_TRANSFER_EVENT    (SPI1_CS0_TRANSFER_EVENT >> 2)
#define SPI1_CS3_TRANSFER_EVENT    (SPI1_CS0_TRANSFER_EVENT >> 3)

static TaskHandle_t             spi1_drv_task_handle;

/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.02 11:46:07 Monday
  *******************************************************************************
  */
static void spi0_callback(void *driverState, spi_event_t event, void *userData)
{
    if (spi0_end_transfer)
        spi0_end_transfer();
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.02 11:46:23 Monday
  *******************************************************************************
  */
static void spi1_callback(void *driverState, spi_event_t event, void *userData)
{
    if (spi1_end_transfer)
        spi1_end_transfer();
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.27 00:08:39 Wednesday
  *******************************************************************************
  */
static void spi2_callback(void *driverState, spi_event_t event, void *userData)
{
    if (spi2_end_transfer)
        spi2_end_transfer();
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.21 22:17:06 Wednesday
  *******************************************************************************
  */
int32_t spi0_cs0_transfer(uint8_t *send_buff, uint8_t *receive_buff, uint32_t len)
{
//    xSemaphoreTake(spi0_cs0_available_sem, portMAX_DELAY);

//    spi0_cs0_send_buff = send_buff;
//    spi0_cs0_receive_buff = receive_buff;
//    spi0_cs0_transfer_len = len;
//    xSemaphoreTake(spi0_cs0_transfer_end_sem, 0);
//    xTaskGenericNotify(spi0_drv_task_handle, SPI0_CS0_TRANSFER_EVENT, eSetBits, NULL);
//    xSemaphoreTake(spi0_cs0_transfer_end_sem, portMAX_DELAY);

//    xSemaphoreGive(spi0_cs0_available_sem);
    PINS_DRV_WritePin(PTB, 0, 0);
    LPSPI_DRV_MasterTransferBlocking(LPSPICOM0, send_buff, receive_buff, len, 200);
    PINS_DRV_WritePin(PTB, 0, 1);
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.21 22:17:06 Wednesday
  *******************************************************************************
  */
int32_t spi1_cs0_transfer(uint8_t* send_buff, uint8_t* receive_buff, uint32_t len)
{
    xSemaphoreTake(spi1_cs0_available_sem, portMAX_DELAY);

    spi1_cs0_send_buff = send_buff;
    spi1_cs0_receive_buff = receive_buff;
    spi1_cs0_transfer_len = len;
    xSemaphoreTake(spi1_cs0_transfer_end_sem, 0);
    xTaskGenericNotify(spi1_drv_task_handle, SPI1_CS0_TRANSFER_EVENT, eSetBits, NULL);
    xSemaphoreTake(spi1_cs0_transfer_end_sem, portMAX_DELAY);

    xSemaphoreGive(spi1_cs0_available_sem);
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.21 22:31:21 Wednesday
  *******************************************************************************
  */
int32_t spi1_cs1_transfer(uint8_t* send_buff, uint8_t* receive_buff, uint32_t len)
{
    xSemaphoreTake(spi1_cs1_available_sem, portMAX_DELAY);

    spi1_cs1_send_buff = send_buff;
    spi1_cs1_receive_buff = receive_buff;
    spi1_cs1_transfer_len = len;
    xSemaphoreTake(spi1_cs1_transfer_end_sem, 0);
    xTaskGenericNotify(spi1_drv_task_handle, SPI1_CS1_TRANSFER_EVENT, eSetBits, NULL);
    xSemaphoreTake(spi1_cs1_transfer_end_sem, portMAX_DELAY);

    xSemaphoreGive(spi1_cs1_available_sem);
    return 0;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.21 22:32:02 Wednesday
  *******************************************************************************
  */
int32_t spi1_cs2_transfer(uint8_t* send_buff, uint8_t* receive_buff, uint32_t len)
{
    xSemaphoreTake(spi1_cs2_available_sem, portMAX_DELAY);

    spi1_cs2_send_buff = send_buff;
    spi1_cs2_receive_buff = receive_buff;
    spi1_cs2_transfer_len = len;
    xSemaphoreTake(spi1_cs2_transfer_end_sem, 0);
    xTaskGenericNotify(spi1_drv_task_handle, SPI1_CS2_TRANSFER_EVENT, eSetBits, NULL);
    xSemaphoreTake(spi1_cs2_transfer_end_sem, portMAX_DELAY);

    xSemaphoreGive(spi1_cs2_available_sem);
    return 0;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.21 22:32:02 Wednesday
  *******************************************************************************
  */
int32_t spi1_cs3_transfer(uint8_t* send_buff, uint8_t* receive_buff, uint32_t len)
{
    xSemaphoreTake(spi1_cs3_available_sem, portMAX_DELAY);

    spi1_cs3_send_buff = send_buff;
    spi1_cs3_receive_buff = receive_buff;
    spi1_cs3_transfer_len = len;
    xSemaphoreTake(spi1_cs3_transfer_end_sem, 0);
    xTaskGenericNotify(spi1_drv_task_handle, SPI1_CS3_TRANSFER_EVENT, eSetBits, NULL);
    xSemaphoreTake(spi1_cs3_transfer_end_sem, portMAX_DELAY);

    xSemaphoreGive(spi1_cs3_available_sem);
    return 0;
}

void spi1_transfer(uint8_t cs_num, uint8_t* send_buff, uint8_t* receive_buff, uint32_t len)
{
    if(cs_num == 0)
        spi1_cs0_transfer(send_buff, receive_buff, len);
    else if(cs_num == 1)
        spi1_cs1_transfer(send_buff, receive_buff, len);
    else if(cs_num == 2)
        spi1_cs2_transfer(send_buff, receive_buff, len);
    else if(cs_num == 3)
        spi1_cs3_transfer(send_buff, receive_buff, len);
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.21 22:13:46 Wednesday
  *******************************************************************************
  */
static void on_spi1_transfer_event(uint8_t* send_buff, uint8_t* receive_buff, uint32_t len,
                                   GPIO_Type * port,  pins_channel_type_t pin, SemaphoreHandle_t transfer_end_sem)
{
//    PINS_DRV_SetPinDirection(port, pin, 1);
    PINS_DRV_WritePin(port, pin, 0);
    LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, send_buff, receive_buff, len, 10);
//    PINS_DRV_SetPinDirection(port, pin, 0);
    PINS_DRV_WritePin(port, pin, 1);

    xSemaphoreGive(transfer_end_sem);
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.21 22:54:27 Wednesday
  *******************************************************************************
  */
static void spi1_drv_task(void* p)
{
    for(;;)
    {
        uint32_t notify;
        xTaskNotifyWait(0x0, 0xffffffff, &notify, portMAX_DELAY);

        if(notify & SPI1_CS0_TRANSFER_EVENT)
        {
            on_spi1_transfer_event(spi1_cs0_send_buff, spi1_cs0_receive_buff, spi1_cs0_transfer_len,
                                   spi1_cs0_port, spi1_cs0_pin, spi1_cs0_transfer_end_sem);
        }

        if(notify & SPI1_CS1_TRANSFER_EVENT)
        {
            on_spi1_transfer_event(spi1_cs1_send_buff, spi1_cs1_receive_buff, spi1_cs1_transfer_len,
                                   spi1_cs1_port, spi1_cs1_pin, spi1_cs1_transfer_end_sem);
        }

        if(notify & SPI1_CS2_TRANSFER_EVENT)
        {
            on_spi1_transfer_event(spi1_cs2_send_buff, spi1_cs2_receive_buff, spi1_cs2_transfer_len,
                                   spi1_cs2_port, spi1_cs2_pin, spi1_cs2_transfer_end_sem);
        }

        if(notify & SPI1_CS3_TRANSFER_EVENT)
        {
            on_spi1_transfer_event(spi1_cs3_send_buff, spi1_cs3_receive_buff, spi1_cs3_transfer_len,
                                   spi1_cs3_port, spi1_cs3_pin, spi1_cs3_transfer_end_sem);
        }
    }
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.25 21:09:29 Monday
  *******************************************************************************
  */
int32_t spi0_init(void)
{
//    BaseType_t      ret;

//    spi0_cs0_available_sem = xSemaphoreCreateBinary();
//    xSemaphoreGive(spi0_cs0_available_sem);
//    spi0_cs1_available_sem = xSemaphoreCreateBinary();
//    xSemaphoreGive(spi0_cs1_available_sem);
//    spi0_cs2_available_sem = xSemaphoreCreateBinary();
//    xSemaphoreGive(spi0_cs2_available_sem);

//    spi0_cs0_transfer_end_sem = xSemaphoreCreateBinary();
//    spi0_cs1_transfer_end_sem = xSemaphoreCreateBinary();
//    spi0_cs2_transfer_end_sem = xSemaphoreCreateBinary();

//    spi0_cs0_port = PTB;
//    spi0_cs0_pin  = 0;
//    spi0_cs1_port = PTB;
//    spi0_cs1_pin  = 5;
//    spi0_cs2_port = PTE;
//    spi0_cs2_pin  = 6;

//    ret = xTaskCreate(spi0_drv_task, "spi0_drv_task", 256, NULL, TASK_PRIORITY_SPI0_DRV, &spi0_drv_task_handle);
//    if(ret == pdPASS)
//        //printf("Creating %s task OK...\r\n", "spi0_drv_task");
//    else
//        //printf("Creating %s task failed! Cause code: %ld\r\n", "spi0_drv_task", ret);

    lpspiCom0_MasterConfig0.callback = spi0_callback;
    INT_SYS_InstallHandler(LPSPI0_IRQn, LPSPI0_IRQHandler, 0);
    INT_SYS_SetPriority(LPSPI0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    LPSPI_DRV_MasterInit(LPSPICOM0, &lpspiCom0State, &lpspiCom0_MasterConfig0);

    if (lpspiCom0_MasterConfig0.transferType == LPSPI_USING_DMA)
    {
        INT_SYS_InstallHandler((IRQn_Type)lpspiCom0_MasterConfig0.rxDMAChannel, get_DMA_IRQHandler(lpspiCom0_MasterConfig0.rxDMAChannel), 0);
        INT_SYS_SetPriority((IRQn_Type)lpspiCom0_MasterConfig0.rxDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);

        INT_SYS_InstallHandler((IRQn_Type)lpspiCom0_MasterConfig0.txDMAChannel, get_DMA_IRQHandler(lpspiCom0_MasterConfig0.txDMAChannel), 0);
        INT_SYS_SetPriority((IRQn_Type)lpspiCom0_MasterConfig0.txDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
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
  * @CreatedDate: 2023.09.24 23:47:13 Sunday
  *******************************************************************************
  */
int32_t spi1_init(void)
{
    BaseType_t      ret;

    spi1_cs0_available_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(spi1_cs0_available_sem);
    spi1_cs1_available_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(spi1_cs1_available_sem);
    spi1_cs2_available_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(spi1_cs2_available_sem);
    spi1_cs3_available_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(spi1_cs3_available_sem);

    spi1_cs0_transfer_end_sem = xSemaphoreCreateBinary();
    spi1_cs1_transfer_end_sem = xSemaphoreCreateBinary();
    spi1_cs2_transfer_end_sem = xSemaphoreCreateBinary();
    spi1_cs3_transfer_end_sem = xSemaphoreCreateBinary();

    spi1_cs0_port = PTE;
    spi1_cs0_pin  = 6;
    spi1_cs1_port = PTE;
    spi1_cs1_pin  = 7;
    spi1_cs2_port = PTE;
    spi1_cs2_pin  = 8;
    spi1_cs3_port = PTE;
    spi1_cs3_pin  = 9;

    ret = xTaskCreate(spi1_drv_task, "spi1_drv_task", 256, NULL, TASK_PRIORITY_SPI1_DRV, &spi1_drv_task_handle);
    if(ret == pdPASS)
        printf("Creating %s task OK...\r\n", "spi1_drv_task");
    else
        printf("Creating %s task failed! Cause code: %ld\r\n", "spi1_drv_task", ret);

    lpspiCom1_MasterConfig0.callback = spi1_callback;
    INT_SYS_InstallHandler(LPSPI1_IRQn, LPSPI1_IRQHandler, 0);
    // 将SPI1中断优先级降低到与SPI2相同，避免SPI2中断被延迟处理，修复ADS1278数据跳变问题
    INT_SYS_SetPriority(LPSPI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    LPSPI_DRV_MasterInit(LPSPICOM1, &lpspiCom1State, &lpspiCom1_MasterConfig0);

    if (lpspiCom1_MasterConfig0.transferType == LPSPI_USING_DMA)
    {
        INT_SYS_InstallHandler((IRQn_Type)lpspiCom1_MasterConfig0.rxDMAChannel, get_DMA_IRQHandler(lpspiCom1_MasterConfig0.rxDMAChannel), 0);
        // DMA中断优先级也相应降低
        INT_SYS_SetPriority((IRQn_Type)lpspiCom1_MasterConfig0.rxDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

        INT_SYS_InstallHandler((IRQn_Type)lpspiCom1_MasterConfig0.txDMAChannel, get_DMA_IRQHandler(lpspiCom1_MasterConfig0.txDMAChannel), 0);
        // DMA中断优先级也相应降低
        INT_SYS_SetPriority((IRQn_Type)lpspiCom1_MasterConfig0.txDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
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
  * @CreatedDate: 2023.09.25 21:08:51 Monday
  *******************************************************************************
  */
int32_t spi2_init(void)
{
    lpspiCom2_MasterConfig0.callback = spi2_callback;
    INT_SYS_InstallHandler(LPSPI2_IRQn, LPSPI2_IRQHandler, 0);
    INT_SYS_SetPriority(LPSPI2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    LPSPI_DRV_MasterInit(LPSPICOM2, &lpspiCom2State, &lpspiCom2_MasterConfig0);

    if (lpspiCom2_MasterConfig0.transferType == LPSPI_USING_DMA)
    {
        INT_SYS_InstallHandler((IRQn_Type)lpspiCom2_MasterConfig0.rxDMAChannel, get_DMA_IRQHandler(lpspiCom2_MasterConfig0.rxDMAChannel), 0);
        INT_SYS_SetPriority((IRQn_Type)lpspiCom2_MasterConfig0.rxDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

        INT_SYS_InstallHandler((IRQn_Type)lpspiCom2_MasterConfig0.txDMAChannel, get_DMA_IRQHandler(lpspiCom2_MasterConfig0.txDMAChannel), 0);
        INT_SYS_SetPriority((IRQn_Type)lpspiCom2_MasterConfig0.txDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
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
  * @CreatedDate: 2023.09.27 00:12:24 Wednesday
  *******************************************************************************
  */
void spi2_register_end_transfer_cb(void (*cb)(void))
{
    spi2_end_transfer = cb;
}
