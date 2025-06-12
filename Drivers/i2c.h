/**
  *****************************************************************************
  * FILENAME   : i2c.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.24 14:56:03 Sunday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.24                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _I2C_H_2023_09_24_22_56_3_455_
#define _I2C_H_2023_09_24_22_56_3_455_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/



/******************************** Functions **********************************/
int32_t i2c_init(void);
status_t i2c_send(uint32_t instance, const uint8_t * txBuff, uint32_t txSize,  bool sendStop);
int32_t i2c_register_end_transfer_cb(void(*cb)(void));


#ifdef __cplusplus
}
#endif
#endif

