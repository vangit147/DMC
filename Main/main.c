/**
  *****************************************************************************
  * FILENAME   : main.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.09 14:17:21 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.09                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"

#include "clock_S32K1xx.h"
#include "clockMan1.h"

#include "pin_mux.h"

#include "lpuart_driver.h"
#include "lpuart1.h"
/* [YangHaifeng 2023.09.20 23:37:27 Wed] */
extern uint32_t __initial_sp;
uint32_t    __DATA_ROM;
uint32_t    __RAM_START = 0x20000000;
uint32_t    __VECTOR_RAM = 0x20000000;
/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 22:59:08 Sunday
  *******************************************************************************
  */
static void lpuart0_tx_cb(uint32_t uartHandle)
{
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 23:59:08 Sunday
  *******************************************************************************
  */
static void lpuart0_rx_cb(uint32_t uartHandle, uint32_t data)
{
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.09 15:38:12 Saturday
  *******************************************************************************
  */
void vApplicationIdleHook(void)
{
    if (is25pl032_flash_get_idle_hook_enable())
    {
        uint32_t SCR = SCB->SCR;
        SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);
        SCR |= SCB_SCR_SEVONPEND_Msk;
        SCB->SCR = SCR;
        __WFI();
    }
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.18 22:18:14 Monday
  *******************************************************************************
  */
__weak void main_task(void* p)
{
    uint32_t    counter = 0;
    for(;;)
    {
        printf("CN: %d\r\n", counter++);
        vTaskDelay(1000);
    }
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 23:29:25 Sunday
  *******************************************************************************
  */
static void working_led_timer_cb(TimerHandle_t xTimer)
{
    PINS_DRV_TogglePins(PTE, 1 << 10);
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 23:29:00 Sunday
  *******************************************************************************
  */
static int main_init(void)
{
    xTimerStart(xTimerCreate("working_led_timer", 500, 1, "", working_led_timer_cb), 1000);
    return 0;
}
INIT_CALL_0(main_init);

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.09 14:26:54 Saturday
  *******************************************************************************
  */
int32_t main(void)
{
    /*设置优先级分组寄存。高4位为抢占优先级，低4位为平先优先级。 [NickYang 2023.12.02 12:13:37 Sat] */
    NVIC_SetPriorityGrouping(3);
    CLOCK_DRV_Init(g_clockManConfigsArr[0]);

    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0,
                  edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);

    LPUART0_init(lpuart0_tx_cb, lpuart0_rx_cb);

    printf("\r\n\r\nStarting system...\r\nVersion: v%d.%d\nBuilding: %s %s\r\nCPU Clock: %dHz\r\n", MAJOR_VERSION,
           MINOR_VERSION, __DATE__, __TIME__, SystemCoreClock);

    start_init_task("main_task", main_task, TASK_PRIORITY_MAIN_TASK, 1024 * 2);
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.18 21:57:55 Monday
  *******************************************************************************
  */
int fputc(int ch, FILE* f)
{
    while(LPUART0_send((uint8_t*)&ch, 1) == 0);
    return 1;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.04 14:36:29 Wednesday
  *******************************************************************************
  */
uint32_t CRC16(void* data, uint32_t len)
{
    unsigned char* pch = (unsigned char*)data;
    static const unsigned char aucCRCHi[] =
    {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40
    };
    static const unsigned aucCRCLo[] =
    {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
        0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
        0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
        0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
        0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
        0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
        0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
        0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
        0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
        0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
        0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
        0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
        0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
        0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
        0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
        0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
        0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
        0x41, 0x81, 0x80, 0x40
    };
    unsigned char ucCRCHi = 0xFF;
    unsigned char ucCRCLo = 0xFF;
    int iIndex;
    while(len--)
    {
        iIndex = ucCRCLo ^ *(pch++);
        ucCRCLo = (unsigned char)(ucCRCHi ^ aucCRCHi[iIndex]);
        ucCRCHi = aucCRCLo[iIndex];
    }
    return (unsigned int)(ucCRCHi << 8 | ucCRCLo);
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2023.10.15 08:54:36 Sunday
  *******************************************************************************
  */
int print_buff(const void * v_addr, unsigned int size, const char* func, int line)
{
    static const char* const convert_tab = "0123456789ABCDEF";
    char out_buff[128];
    unsigned char temp_char;
    unsigned int index;
    unsigned int i;
    unsigned char* addr = (unsigned char*)v_addr;

    printf("-----%s  line:%d-----\r\n", func, line);
    while(size)
    {
        unsigned int current_len;
        if(size >= 16)
            current_len = 16;
        else
            current_len = size;
        size -= current_len;

        memset(out_buff, 0x20, sizeof(out_buff));
        index = snprintf(out_buff, sizeof(out_buff), "%p: ", addr);
        for(i = 0; i < current_len; i++)
        {
            temp_char = *addr;
            addr++;
            if(i == 8)
            {
                out_buff[index++] = '-';
                out_buff[index++] = ' ';
            }
            out_buff[index++] = convert_tab[temp_char >> 4];
            out_buff[index++] = convert_tab[temp_char & 0XF];
            out_buff[index++] = ' ';
            if((temp_char < 0x20) || (temp_char > 0x7e))
                temp_char = '.';
            out_buff[64 + i] = temp_char;
        }
        out_buff[64 + i + 1] = 0x0d;
        out_buff[64 + i + 2] = 0x0a;
        out_buff[64 + i + 3] = 0x0;
        printf("%s", out_buff);
    }
    return 0;
}
/**
  *******************************************************************************
  * @Description: 由DMA通道号返回对应的IRQ处理函数
  * @Parameters : DMA通道号
  * @RetValue   : IRQ处理函数
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.22 23:49:49 Thursday
  *******************************************************************************
  */
void (*get_DMA_IRQHandler(uint32_t channel))(void)
{
    static void (* const dma_irq_handler[])(void) =
    {
        DMA0_IRQHandler,
        DMA1_IRQHandler,
        DMA2_IRQHandler,
        DMA3_IRQHandler,
        DMA4_IRQHandler,
        DMA5_IRQHandler,
        DMA6_IRQHandler,
        DMA7_IRQHandler,
        DMA8_IRQHandler,
        DMA9_IRQHandler,
        DMA10_IRQHandler,
        DMA11_IRQHandler,
        DMA12_IRQHandler,
        DMA13_IRQHandler,
        DMA14_IRQHandler,
        DMA15_IRQHandler,
    };

    return dma_irq_handler[channel];
}

