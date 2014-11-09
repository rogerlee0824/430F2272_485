#include "main.h"
#include <stdio.h>
#include "msp430x22x2.h"
#include "init.h"
#include "uart.h"
#include "delay.h"
#include "cc112x_spi.h"
#include "hal_spi_rf_trxeb.h"
#include "cc1120_rx_sniff_mode_reg_config.h"
#include "rs485.h"

#define RADIO_PACKET_SIZE           18
#define PACKET_HEADER               0xAA
#define PACKET_ACK_HEADER           0xA6
#define PACKET_END           	    0xBB
#define ReadMeterCMD		    	0x11   //抄表
#define ReadMeterACKCMD		    	0x21
#define CheckMeterCMD		    	0xee   //写表头及数据
#define CheckMeterACKCMD	    	0x2e
#define OpenValveCMD		    	0x13
#define OpenValveACKCMD		    	0x23
#define CloseValveCMD		    	0x14
#define CloseValveACKCMD	    	0x24
#define CalibrateDataCMD            0x15   //校准数据
#define CalibrateDataACKCMD         0x25

#define V_STATUS                    0x01   //阀状态
#define F_STATUS                    0x02   //FLASH状态
#define M_STATUS                    0x04   //防磁状态
#define B_STATUS                    0x08   //电池电量状态

//信息存储区010FFh -- 01000h
/*程序修订待完成工作说明*/
/*1，增加防磁关阀报警，两个计数中断同时为高电平，即可认为受外界磁石干挠，执行关阀，并产生防磁报警位*/
/*2，增加一个字节(共8位)，以位来记录表各类故障状态，从低至高依次为：阀状态，FLASH，防磁，电池电量等*/
/*3，430IO口输出为高阻，所有未用IO口应设置为输出，并关闭内部比较器*/

// Defined for Uart 
INT8U ucUartRXBuffer[UART_RX_LEN];
//INT8U *pucUartRXBuffer;
INT8U ucUartTXMaxSize       			= 0;
INT8U ucUartRXMaxSize       			= 0;
INT8U ucUartRXSize          			= 0;
INT16U wUartRXCompletedFlag 			= 0;
uint32 u32AmmeterCount				    = 0;

volatile INT8U x = 0,y = 0;
INT8U ucAllowBit2INT = 1,ucAllowBit3INT = 1;

INT8U ucRFTxBuffer[19];
INT8U ucRFRxBuffer[128];
INT8U Buffer_count[5];							

// Defined for Radios
uint8  RXpacketSemaphore;
uint8  TXpacketSemaphore;
uint8  ucRadioWorkingStatus;

#pragma location=0x1000
no_init INT8U ucAmmeterMACAddr[5];	

#pragma location=0x1080
no_init INT8U ucAmmeterCount[5];							

INT8U ucAmmeterMACAddr_temp[5] = {0x00,0x00,0x00,0x00,0x02};
uint8 u8Buffer_count[5] = {0x00,0x00,0x00,0x00,0x00};		// Save the data,左起第一个字节用作状态字节			
uint8 u8Buffer_count_temp[5] = {0x00,0x00,0x00,0x23,0x21};	// Save the data

uint8 u8Buffer_temp[6] = {0x01,0x03,0x00,0x0E,0x00,0x02};	// Save the data

unsigned char Counter_flag=0;         
unsigned char usart_rx_index = 0;   
unsigned char Receive_Flag=0;   
unsigned char Receive_Complete_Flag=0;
unsigned char flag_save_count = 0;
unsigned char timerflag = 0;

void HEX_to_BCD(unsigned long ulCounterTmp,uint8 *pucArrar);
uint32 BCD_to_HEX(uint8 *pu8Arrar);
//uint8 createTxPacket(uint8 *pucSrcBuffer,uint8 *pucDestBuffer);

uint8 gpioConfigSlave[] = 
{
    //gpio2,re rx=0
    //gpio3,te tx=0
    50,//gpio3
    0x01,//gpio2
    0xB0,
    0x06,
};
uint8 preambleConfig[] = 
{
   0x10,
   0x2A,
};
uint8 worConfig[] = 
{
   0x20,
   0x63,
   0x7B,
   //0x42,
   //0x2f,
};

void main(void)
{
    uint8 i;
	WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
	initMCU();
	
	_EINT(); 
    while(1)
    {
		i = GetVoltage();
		Delay10mS(10) ;
		if(i == RS485_RX_SUCCESS)
		{
			LED2ON();
		}
        /*P2DIR  |= BIT5 | BIT6 | BIT7;   					// P2.2,3,4 pins are input
		P2REN  |= (BIT3 | BIT4);                            // Enables P2.1,3,4 Pullup/Pulldown resistor
		P2OUT  &= ~(BIT3 | BIT4);					        // Enables P2.1,3,4 pins pull-down resistor
		P2IES  &= ~(BIT3|BIT4);						        // P2.1,3,4 pins are set with a low-to-high transition
    	P2IES  &= BIT2;						                // P2.2 pins are set with a high-to-low transition
		P2IFG  &= 0x00;										// The P2IFGx flag is cleared
		P2IE   |= BIT2 | BIT3 |BIT4;						// Enables the interrupt of the P2.2,3,4 pins
		
    	WDTCTL = WDTPW + WDTHOLD;							// 进入低功耗之前关闭看门狗    
        _EINT();
    	__bis_SR_register(LPM3_bits + GIE);
		
			if(RXpacketSemaphore == ISR_ACTION_REQUIRED) 
	    {
			WDTCTL = WDT_ARST_1000;
        	RXpacketSemaphore = ISR_IDLE;
			LED1ON();
			i = ReadDataFromRadio(ucRFRxBuffer);
			LED1OFF();

			//if(i>0 && createTxPacket(ucRFRxBuffer,ucRFTxBuffer))
			{
				trxSpiCmdStrobe(CC112X_SPWD);                   
				for(i =(2 - ucRFTxBuffer[2]); i>0; i--)
				{
					Delay1mS(400);
					WDTCTL = WDT_ARST_1000;
				}
				LED2ON();
				WDTCTL = WDT_ARST_250;
				TXpacketSemaphore = ISR_IDLE;
				trxSpiCmdStrobe(CC112X_SIDLE);
				TE();
				SendPacketByRF(ucRFTxBuffer, RADIO_PACKET_SIZE + 1);
				P1IFG = 0;                                
				LED2OFF();
				 //RE();
			}
			trxSpiCmdStrobe(CC112X_SWORRST);
			trxSpiCmdStrobe(CC112X_SWOR);
			//IDLE();
			RE();
				   
			WDTCTL = WDT_ARST_1000;
	   }	*/
    }
}


/*******************************************************************************
*   @fn         USCIA0RX_ISR
*   @brief      ISR for packet handling in RX. Sets packet semaphore
*               and clears interrupt flag
*   @param      none
*   @return     none
*******************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIA0RX_ISR(void)
{
    INT8U ucIFG2;
    
    ucIFG2  = IFG2;  
	if(ucIFG2 & UCA0RXIFG)
   	{
		//if(tRS485_rx.rx_enable)
		{
        	tRS485_rx.rx_buf[usart_rx_index++] = UCA0RXBUF; 
			if(usart_rx_index >= tRS485_rx.rx_len)
			{
				tRS485_rx.rx_ok = true;
				usart_rx_index = 0;
			}
    	}
	}
}
/*******************************************************************************
*   @fn         radioRxISR
*   @brief      ISR for packet handling in RX. Sets packet semaphore
*               and clears interrupt flag
*   @param      none
*   @return     none
*******************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void ISR_PORT1(void) 
{
    uint8 ucP1IFG;	
    
	ucP1IFG = P1IFG;
	if(ucP1IFG & BIT0)							// Interrupt from P1.0
	{
		P1IFG  &= ~BIT0;      // Clear isr flag
  		//TXpacketSemaphore = ISR_ACTION_REQUIRED;
        //RE();
        //LED1ON();
	}
	else if(ucP1IFG & BIT1)						// Interrupt from P1.1
	{
		P1IFG  &= ~BIT1; // Clear isr flag
        RE();
  		RXpacketSemaphore = ISR_ACTION_REQUIRED;		
		LPM3_EXIT;
	}
	else if(ucP1IFG & BIT2)
    {
        P1IFG  &= ~BIT2; // Clear isr flag
        if(P1IES & BIT2)//下降沿，无线芯片工作
        {
            LED1ON();
            RE();
        }
        else
        {
            LED1OFF();
            IDLE();
        }
        P1IES ^= BIT2;
        P1IFG &= ~BIT2;
        P1IE  |= BIT2;
        }
    else
	{}
}
/*******************************************************************************
*   @fn         ISR_PORT2
*   @brief      Calculate.
*   @param      none
*   @return     none
*******************************************************************************/
#pragma vector=PORT2_VECTOR
__interrupt void ISR_PORT2(void)//计数中断
{
    INT8U ucP2IFG;
	
    ucP2IFG = P2IFG;
    if(ucP2IFG & BIT2)			            // Interrupt from P2.2
    {  
        P2IFG  &= ~(BIT2);	                // Clear the interrupt flag
        LPM3_EXIT;
    }
    if(ucP2IFG & BIT4)			            // Interrupt from P2.4
    {   	
		P2IE   &= ~BIT4;                    // Disables interrupt from P2.4
        P2DIR  |= BIT4;                     // P2.4 is output
        P2OUT  &= ~BIT3;                    // P2.3 is low
        P2DIR  &= ~BIT3;                    // P2.3 is input
        P2OUT  |= BIT4;                     // P2.4 is high
		P2IE   |= BIT3;                     // Enables interrupt from P2.3
        P2IFG  &= ~(BIT4 | BIT3);	        // Clear the interrupt flag
		x = 1;
    }
    if(ucP2IFG & BIT3)			            // Interrupt from P2.3
    {   		
		P2IE    &= ~BIT3;                   // Disables interrupt from P2.3
        P2DIR  |= BIT3;                     // P2.3 is output
        P2OUT  &= ~BIT4;                    // P2.4 is low
        P2DIR  &= ~BIT4;                    // P2.4 is input
        P2OUT  |= BIT3;                     // P2.3 is high
		P2IE    |= BIT4;                    // Enables interrupt from P2.4
        P2IFG   &=  ~(BIT3 | BIT4);			// Clear the interrupt flags
		y = 1;
    }
    if(x == 1 && y == 1)
    {
		LPM3_EXIT;
    }
}

#pragma vector=TIMERA0_VECTOR
__interrupt void TimerA(void)//定时器中断
{
   CCTL0 &= ~CCIE;
   IDLE();
   LED1OFF();
}
