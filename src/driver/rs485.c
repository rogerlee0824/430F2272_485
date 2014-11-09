#include "rs485.h"
#include "uart.h"
#include "init.h"
#include "crc16.h"
#include "delay.h"

rs485_rx_t tRS485_rx;

/*******************************************************************************
 * @brief       Sends data to RS485 bus.
 * input parameters
 * @param       *buf - pointer to data array that is sent
 * @param       len  - Length of data array that is sent
 * output parameters
 * @return      void
********************************************************************************/
void RS485_SendData(uint8* pBuf,uint16 len)
{
    uint16 tmp_crc;
    
	/*tmp_crc = crc16(pBuf,len);
    EnableRS485TX();

    UartSendByte((uint8)0xaa);
    EnableRS485RX();*/
	
	tmp_crc = crc16(pBuf,len);
    EnableRS485TX();
    UartSendMultiBytes(pBuf,len);
    UartSendByte((uint8)tmp_crc);
	Delay1mS(3);
    UartSendByte((uint8)(tmp_crc >> 8));
	Delay1mS(3);
    EnableRS485RX();
}

/*******************************************************************************
 * @brief       Recieve data from RS485 bus.
 * input parameters
 * @param       *buf - pointer to data array that is sent
 * @param       len  - Length of data array that is sent
 * output parameters
 * @return      The status of RS485 RX
********************************************************************************/
uint8 RS485_ReceiveData(uint8* pBuf,uint16 len)
{
	uint16 count = 65000;
	
    tRS485_rx.rx_enable = true;
    tRS485_rx.rx_ok     = false;
    tRS485_rx.rx_len    = len;
    tRS485_rx.rx_buf    = pBuf;
    
    while((!tRS485_rx.rx_ok) && (count > 0))
	{
		Delay1us(1);
		count--;
	}
	tRS485_rx.rx_enable = false;
	tRS485_rx.rx_ok	    = false;
	
	// Chech if RS485_RX_TIMEOUT?
	if(count == 0)
	{
		return RS485_RX_TIMEOUT;
	}
	
	// Chech if the crc16 is OK.
	if(!crc16_check(pBuf, len))
	{
		return RS485_RX_CRCERROR;
	}
	
	return RS485_RX_SUCCESS;
}
uint8 rx_buffer[50];

uint8 GetVoltage(void)
{
	uint8 tx_buffer[6];
	uint8 rx_status;
	
	tx_buffer[0] = 0x01;							// Slave addr
	tx_buffer[1] = 0x03;							// Cmd
	tx_buffer[2] = 0x00;							// Start addr(High byte)
	tx_buffer[3] = 0x12;							// Start addr(Low byte)
	tx_buffer[4] = 0x00;							// Num of Register(High byte)
	tx_buffer[5] = 0x06;							// Num of Register(Low byte)
	RS485_SendData(tx_buffer,sizeof(tx_buffer));
	
	rx_status = RS485_ReceiveData(rx_buffer,17);
	
	return rx_status;
}