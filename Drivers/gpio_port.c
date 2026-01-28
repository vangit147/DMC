/**
  *****************************************************************************
  * FILENAME   : gpio_port.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.26 22:40:24 Tuesday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.26                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#include "gpio_port.h"

void (*portc_callback[32])(void);

void (*porte_callback[32])(void);
/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.26 22:43:45 Tuesday
  *******************************************************************************
  */
static void PORTC_IRQHandler(void)
{
    uint32_t int_flag = PINS_DRV_GetPortIntFlag(PORTC);
    uint8_t pin_num;
    PINS_DRV_ClearPortIntFlagCmd(PORTC);
    for(pin_num = 0; pin_num < 32; pin_num++)
    {
        if(int_flag & (0x00000001 << pin_num))
        {
            if(portc_callback[pin_num])
                portc_callback[pin_num]();
        }
    }
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.20 20:54:08 Tuesday
  *******************************************************************************
  */
static void PORTE_IRQHandler(void)
{
    uint32_t int_flag = PINS_DRV_GetPortIntFlag(PORTE);
    uint8_t pin_num;
    PINS_DRV_ClearPortIntFlagCmd(PORTE);
    for(pin_num = 0; pin_num < 32; pin_num++)
    {
        if(int_flag & (0x00000001 << pin_num))
        {
            if(porte_callback[pin_num])
                porte_callback[pin_num]();
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
  * @CreatedDate: 2023.09.26 22:41:29 Tuesday
  *******************************************************************************
  */
int32_t gpio_port_init(void)
{
    INT_SYS_InstallHandler(PORTC_IRQn, PORTC_IRQHandler, 0);
    INT_SYS_SetPriority(PORTC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    INT_SYS_EnableIRQ(PORTC_IRQn);

    INT_SYS_InstallHandler(PORTE_IRQn, PORTE_IRQHandler, 0);
    INT_SYS_SetPriority(PORTE_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 4);
    INT_SYS_EnableIRQ(PORTE_IRQn);
    return 0;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.28 23:16:26 Thursday
  *******************************************************************************
  */
int32_t gpio_portc_register_cb(uint8_t pin_num, void (*cb)(void))
{
    portc_callback[pin_num] = cb;
    return 0;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.20 20:55:36 Tuesday
  *******************************************************************************
  */
int32_t gpio_porte_register_cb(uint8_t pin_num, void (*cb)(void))
{
    porte_callback[pin_num] = cb;
    return 0;
}

