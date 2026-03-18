/**
  *****************************************************************************
  * FILENAME   : gpio_port.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.26 22:40:38 Tuesday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.26                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _GPIO_PORT_H_2023_09_26_22_40_38_412_
#define _GPIO_PORT_H_2023_09_26_22_40_38_412_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/



/******************************** Functions **********************************/
int32_t gpio_port_init(void);
int32_t gpio_portc_register_cb(uint8_t pin_num, void (*cb)(void));
int32_t gpio_porte_register_cb(uint8_t pin_num, void (*cb)(void));

#ifdef __cplusplus
}
#endif
#endif

