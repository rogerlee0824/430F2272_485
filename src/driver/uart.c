#include "uart.h"
#include "msp430x41x2.h"
#include <stdio.h>
#include "delay.h"

void UartSendByte(INT8U chr)
{
  	UCA0TXBUF = chr;                    // TX 
	while(!(IFG2 & UCA0TXIFG));			// Wait for TX complete
}

int putchar(int ch)
{
      UartSendByte(ch);

      return (ch);
}

void UartSendMultiBytes(void *pvBuffer,INT16U wSize)
{
    while(wSize--)
    {
        UartSendByte(*((INT8U *)pvBuffer));
        Delay1mS(3);
		pvBuffer = (INT8U *)pvBuffer + 1;
    }
}

void UartSendATCmd(void *pvBuffer,INT16U wSize)
{
	wSize--;
	while(wSize--)
    {
        UartSendByte(*((INT8U *)pvBuffer));
        pvBuffer = (INT8U *)pvBuffer + 1;
    }
	UartSendByte('\r');
	UartSendByte('\n');
}