

#include "uartmessagebuffer.h"
#include "string.h"
#include "stdlib.h"

  UartMessageBuffer* uartmessagebuffer_create(uint8_t txMessagesBufferSize, uint8_t txBufferSize, UART_HandleTypeDef *uartHandle){
	UartMessageBuffer* messageBuffer = malloc(sizeof(UartMessageBuffer));
	messageBuffer->txBufferSize = txBufferSize;
	messageBuffer->txMessageBufferSize = txMessagesBufferSize;

	messageBuffer->txBuffer = malloc(sizeof(uint8_t) * txBufferSize);
	messageBuffer->txMessageBuffer = malloc(sizeof(uartmessage*) * messageBuffer->txMessageBufferSize);
	messageBuffer->txMessageBufferReadHead = 0;
	messageBuffer->txMessageBufferWriteHead = 0;
	messageBuffer->uarthandle = uartHandle;

	return messageBuffer;
}

void uartmessageBuffer_destroy(UartMessageBuffer *messageBuffer){
	free(messageBuffer->txBuffer);
	free(messageBuffer->txMessageBuffer);
	/*
	 * TODO: verbleibende nachrichten löschen
	 */
	free(messageBuffer);
}

void uartmessagebuffer_createmessage (UartMessageBuffer *messageBuffer, uint8_t size, uint8_t* data){
	uartmessage* msg = malloc(sizeof(uartmessage));
	msg->data = data;
	msg->size = size;

	messageBuffer->txMessageBuffer[messageBuffer->txMessageBufferWriteHead] = msg;
	messageBuffer->txMessageBufferWriteHead = (messageBuffer->txMessageBufferWriteHead+1) % messageBuffer->txMessageBufferSize;
	uartmessagebuffer_sendNextMessage(messageBuffer);
}

void uartmessagebuffer_sendNextMessage(UartMessageBuffer *messageBuffer){
	//check if buffer is empty
	if(messageBuffer->txMessageBufferReadHead == messageBuffer->txMessageBufferWriteHead){
		return;
	}

	// check if uart is ready
	if(HAL_UART_GetState(messageBuffer->uarthandle) != HAL_UART_STATE_READY){
		return;
	}

	//send message
	uartmessage *msg = messageBuffer->txMessageBuffer[messageBuffer->txMessageBufferReadHead];
	if(msg->size > messageBuffer->txBufferSize) msg->size = messageBuffer->txBufferSize;
	memcpy(messageBuffer->txBuffer, msg->data, msg->size);
  HAL_UART_Transmit_IT(messageBuffer->uarthandle, messageBuffer->txBuffer, msg->size);

  messageBuffer->txMessageBufferReadHead = (messageBuffer->txMessageBufferReadHead+1) % messageBuffer->txMessageBufferSize;


	//destroy message
	free(msg->data);
	free(msg);

}




void uartmessagebuffer_onTxComplete(UartMessageBuffer *messageBuffer){
	uartmessagebuffer_sendNextMessage(messageBuffer);
}
