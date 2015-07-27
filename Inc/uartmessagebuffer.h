/*
 * uartmessagebuffer.h
 *
 *  Created on: 27.07.2015
 *      Author: raphael
 */

#ifndef INC_UARTMESSAGEBUFFER_H_
#define INC_UARTMESSAGEBUFFER_H_

#include "inttypes.h"
#include "triggerchannel.h"
#include "hihatchannel.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

#define VOLTDRUMINTRFACE_MESSAGEBUFFERSIZE 10


typedef struct {
	uint8_t *data;
	uint8_t size;
}uartmessage;


typedef struct{
	UART_HandleTypeDef* uarthandle;

	uint8_t *txBuffer;
	uint8_t txBufferSize;

	uartmessage **txMessageBuffer;
	uint8_t txMessageBufferSize;
	uint8_t txMessageBufferReadHead;
	uint8_t txMessageBufferWriteHead;

}UartMessageBuffer;


UartMessageBuffer* uartmessagebuffer_create(uint8_t messagesBufferSize, uint8_t bytesPerMessageBufferSize, UART_HandleTypeDef *uartHandle);
void uartmessageBuffer_destroy(UartMessageBuffer *messageBuffer);

void uartmessagebuffer_createmessage (UartMessageBuffer *messageBuffer, uint8_t size, uint8_t* data);
void uartmessagebuffer_sendNextMessage(UartMessageBuffer *messageBuffer);
void uartmessagebuffer_onTxComplete(UartMessageBuffer *messageBuffer);


#endif /* INC_UARTMESSAGEBUFFER_H_ */
