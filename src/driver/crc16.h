#ifndef _CRC16_H_
#define _CRC16_H_

#include "hal_types.h"

/*******************************************************************************
 * @brief       Calculates the crc16.
 * input parameters
 * @param       *buf - pointer to data array that is calculated
 * @param       len  - Length of data array to be calculated
 * output parameters
 * @return      crc16 value
********************************************************************************/
uint16 crc16(uint8 * buf,uint16 len);

/*******************************************************************************
 * @brief       Check if the rx crc16 is equal with the calculated crc16.
 * input parameters
 * @param       *buf - pointer to data array that is calculated
 * @param       len  - Length of data array to be calculated
 * output parameters
 * @return      OK - 1;ERROR - 0.
********************************************************************************/
uint8 crc16_check(uint8 * buf,uint16 len);

#endif
