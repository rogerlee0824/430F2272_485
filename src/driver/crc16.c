#include "crc16.h"

/*******************************************************************************
 * @brief       Calculates the crc16.
 * input parameters
 * @param       *buf - pointer to data array that is calculated
 * @param       len  - Length of data array to be calculated
 * output parameters
 * @return      crc16 value
********************************************************************************/
uint16 crc16(uint8 * buf,uint16 len)
{
    uint16 crc          = 0xFFFF;
    uint16 crc_poly     = 0xA001;
    uint16 index,index1;
    
    for(index = 0;index < len;index ++)
    {
        crc ^= *(buf + index);
        for(index1 = 0;index1 < 8;index1 ++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= crc_poly;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

/*******************************************************************************
 * @brief       Check if the rx crc16 is equal with the calculated crc16.
 * input parameters
 * @param       *buf - pointer to data array that is calculated
 * @param       len  - Length of data array to be calculated
 * output parameters
 * @return      OK - 1;ERROR - 0.
********************************************************************************/
uint8 crc16_check(uint8 * buf,uint16 len)
{
	uint16 crc_tmp = ((uint16)buf[len - 1] << 8) | (uint16)buf[len - 2];
	
	if(crc16(buf,len - 2) == crc_tmp)				// OK
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
