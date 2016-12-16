#pragma once
#ifndef TELEMETRY_UART
#define TELEMETRY_UART
#define BAUD 115200
#include <util/setbaud.h>
#include "IBUSTelemetry.h"


#define RXCIE_on() UCSR1B |= _BV(RXCIE1)
#define RXCIE_off() UCSR1B &= ~_BV(RXCIE1);

#define UDRIE_on() UCSR1B |= _BV(UDRIE1)
#define UDRIE_off() UCSR1B &= ~_BV(UDRIE1);

uint8_t volatile NextIbusMessageReady = 0;
uint8_t volatile uartCurrentPos = 0;
uint8_t volatile uartLength = 0;
uint8_t* txPtr = 0;
void initSerial() { //serial 1 more control
	UBRR1H = UBRRH_VALUE;
	UBRR1L = UBRRL_VALUE;
#if USE_2X
	UCSR1A |= _BV(U2X1);
#else
	UCSR1A &= ~(_BV(U2X1));
#endif
	UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); /* 8-bit data */
	UCSR1B = _BV(RXEN1) | _BV(TXEN1);   /* Enable RX and TX */
	uartCurrentPos = 0;
	uartLength = 0;
	NextIbusMessageReady = 0;
	txPtr = 0;
	RXCIE_on();
}

void usart_send(uint8_t* dataPtr, uint8_t len)
{
	if (len <= 0 || dataPtr == 0) return;
	txPtr = dataPtr;
	uartCurrentPos = 0;
	uartLength = len;
	NextIbusMessageReady = 0;
	cli();//disable interrupts
	UCSR1B = _BV(TXEN1) | _BV(UDRIE1);   /* TX only with interrupt*/
	sei();//enable interrupts
}

//USART RX interrupt
ISR(USART1_RX_vect)
{
	NextIbusMessageReady = ibusRXByte(UDR1);
}

//USART - empty UDR1 - data can be send
ISR(USART1_UDRE_vect)
{
	if (uartCurrentPos < uartLength && txPtr != 0) {
		uint8_t val = txPtr[uartCurrentPos++];
		UDR1 = val;
	}
	else {
		uartLength = uartCurrentPos = 0;
		txPtr = 0;
		UCSR1B = _BV(RXEN1) | _BV(RXCIE1);   /* Enable RX so we can receive next frame*/
	}
}
#endif // !TELEMETRY_UART
