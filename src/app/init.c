#include "init.h"
#include "msp430x22x2.h"
#include "hw_types.h"
#include "hal_spi_rf_trxeb.h"
#include "delay.h"

#define DELTA 2930                					// target DCO = DELTA*(4096) = 12MHz

void initSysClock(void);
void initGPIO(void);
void initUart(void);

void Set_DCO(void)                          		// Set DCO to selected frequency
{
  	unsigned int Compare, Oldcapture = 0;

  	BCSCTL1 |= DIVA_3;                        		// ACLK= LFXT1CLK/8
  	TACCTL2 = CM_1 + CCIS_1 + CAP;            		// CAP, ACLK
  	TACTL = TASSEL_2 + MC_2 + TACLR;          		// SMCLK, cont-mode, clear

  	while (1)
  	{
    	while (!(CCIFG & TACCTL2));             	// Wait until capture occured
    	TACCTL2 &= ~CCIFG;                      	// Capture occured, clear flag
    	Compare = TACCR2;                       	// Get current captured SMCLK
    	Compare = Compare - Oldcapture;         	// SMCLK difference
    	Oldcapture = TACCR2;                    	// Save current captured SMCLK

    	if (DELTA == Compare)
      		break;									// If equal, leave "while(1)"
    	else if (DELTA < Compare)
    	{
      		DCOCTL--;								// DCO is too fast, slow it down
      		if (DCOCTL == 0xFF)						// Did DCO roll under?
        		if (BCSCTL1 & 0x0f)
          		BCSCTL1--;							// Select lower RSEL
    	}
    	else
    	{
      		DCOCTL++;								// DCO is too slow, speed it up
      		if (DCOCTL == 0x00)                 	// Did DCO roll over?
        		if ((BCSCTL1 & 0x0f) != 0x0f)
          		BCSCTL1++;							// Sel higher RSEL
    	}
  	}
  	TACCTL2 = 0;                              		// Stop TACCR2
  	TACTL = 0;                                		// Stop Timer_A
    BCSCTL1 &= ~DIVA_3;                             // ACLK = LFXT1CLK
}

void initMCU(void)
{	
    initSysClock();
    initGPIO();	
    initUart();
    LED2ON(); 
    trxRfSpiInterfaceInit(0x20); 
	FCTL2 = FWKEY + FSSEL_1 + FN5; 
}

void initSysClock(void)
{
	WDTCTL = WDTPW + WDTHOLD; 							// Stop watchdog timer
    Set_DCO();
}

void initGPIO(void)
{
	// P1
  	P1DIR  |= BIT0 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;  // P1.0,1,2 pins are interrupt input from CC1120,P1.3 pin is output
	P1OUT  = 0;
	P1REN  |= BIT0+BIT1+BIT2;
	P1OUT |= BIT2;         
	P1IFG  &= 0x00;					        			// The P1IFGx flag is cleared
	
	Delay1mS(100);
	P1OUT |= BIT3;
	
	// P2
	P2DIR  |= BIT3 | BIT5 | BIT6 | BIT7;   				// P2.0,1,2,4 pins are input,P2.3 pin is output,others are unused
	P2REN  |= (BIT1 | BIT3 | BIT4);
	P2OUT  &= ~(BIT1 | BIT3 | BIT4);					// Enables P2.1,3,4 pins pull-down resistor
	P2IES  &= ~(BIT1|BIT3|BIT4);						// P2.1,3,4 pins are set with a low-to-high transition
	P2IFG  &= 0x00;										// The P2IFGx flag is cleared
	P2IE   |= (BIT1 | BIT4);							// Enables the interrupt of the P2.1,4 pins
	
	// P3
	P3SEL  |= BIT4 | BIT5;								// P3.4,5 = USCI_A0 UART TXD/RXD 	
	P3DIR  |= BIT4 | BIT6 | BIT7;						// P3.0,1,2,3 are SPI interfaces,P3.4,5 = USCI_A0 UART TXD/RXD
                                        				// P3.4,6,7 are output
	P3OUT	= 0;
	
    // P4
	P4DIR |= BIT7;
	P4OUT	= 0;
}

void initUart(void)
{
	UCA0CTL1 |= UCSWRST;                       			// **Initialize USCI state machine**
	UCA0CTL1 |= UCSSEL_1;                       		// CLK = ACLK
	UCA0BR0   = 0x06;                             		// 32kHz/4800 = 6
    UCA0BR1   = 0x00;                             		//
    UCA0MCTL  = UCBRS2 + UCBRS1 + UCBRS0;            	// Modulation UCBRSx = 7
	
	P3SEL  |= BIT4 | BIT5;								// P3.4,5 = USCI_A0 UART TXD/RXD 
	P3DIR  |= BIT4;										// P3.4 is output

    UCA0CTL1 &= ~UCSWRST;                       		// **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                 					// Enable USCI_A0 RX interrupt  
}

void LED1ON(void)
{
	P3OUT  |= BIT7; 
}

void LED1OFF(void)
{
	P3OUT  &= ~BIT7; 
}

void LED2ON(void)
{
	P4OUT  |= (BIT2);
}

void LED2OFF(void)
{
	P4OUT  &= ~BIT2; 
}

void RE(void)
{
   P4OUT |= BIT0;//4.0te 4.1RE
   P4OUT &= ~BIT1;
}

void EnableRS485TX(void)
{
   P4OUT |= BIT7;           
}

void EnableRS485RX(void)
{
   P4OUT &= ~BIT7;            
}

void TE(void)
{
  P4OUT |= BIT1;//4.0te 4.1RE
  P4OUT &= ~BIT0;
}
void IDLE(void)
{
  P4OUT &= ~BIT0;
  P4OUT &= ~BIT1;
}
