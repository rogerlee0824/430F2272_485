#ifndef _RS485_H_
#define _RS485_H_

#include <stdbool.h>
#include "hal_types.h"

#define RS485_RX_SUCCESS			0x00
#define RS485_RX_CRCERROR			0x01
#define RS485_RX_TIMEOUT			0x02

#define RS485_RX_LEN                 128

typedef struct {
    bool      rx_enable;              // Enable the RX from RS485 bus
    uint8     rx_len;                 // The RX length from RS485 bus
    bool      rx_ok;                  // The complete rx from RS485 bus
    uint8     *rx_buf;                // Pointer to the buffer to save from RS485
} rs485_rx_t;

extern rs485_rx_t tRS485_rx;

uint8 GetVoltage(void);

#endif 
