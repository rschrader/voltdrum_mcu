/*
 * midi.c
 *
 *  Created on: 19.05.2015
 *      Author: raphael
 */

#include "midi.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "string.h"
#include "stdlib.h"

extern UART_HandleTypeDef huart3;

void midi_sendNote(uint8_t channel, uint8_t note, uint8_t velocity){


	//note on
	uint8_t bfrsize = 6;
	uint8_t* bfr = malloc(sizeof(uint8_t)*bfrsize);

	bfr[0] = 0xFF & ( 0x90 | channel  & 0xF);
	bfr[1] = note & 0x7F;
	bfr[2] = velocity & 0x7F;

	//note off
	bfr[3] = 0xFF & ( 0x80 | channel  & 0xF);
	bfr[4] = note & 0x7F;
	bfr[5] = velocity & 0x7F;

	midi_createmessage(bfrsize, bfr);


}

void midi_sendControlChange(uint8_t channel, uint8_t control, uint8_t value){

	uint8_t bfrsize = 3;
	uint8_t* bfr = malloc(sizeof(uint8_t)*bfrsize);

	bfr[0] = 0xFF & ( 0b10110000 | channel & 0xF);
	bfr[1] = control & 0x7F;
	bfr[2] = value & 0x7F;

	midi_createmessage(bfrsize, bfr);

}


/**
 * Send mecahnics
 */


void midi_createmessage (uint8_t size, uint8_t* data){
	midimessage* msg = malloc(sizeof(midimessage));
	msg->data = data;
	msg->size = size;

	midi_MessageBuffer[midi_messageBufferWriteHead] = msg;
	midi_messageBufferWriteHead = (midi_messageBufferWriteHead+1) % MIDI_MESSAGEBUFFERSIZE;
	midi_sendNextMessage();
}

void midi_sendNextMessage(){
	//check if buffer is empty
	if(midi_messageBufferReadHead == midi_messageBufferWriteHead){
		return;
	}

	// check if uart is ready
	if(HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY){
		return;
	}

	//send message
	midimessage *msg = midi_MessageBuffer[midi_messageBufferReadHead];
	if(msg->size > 10) msg->size = 10;
	memcpy(midi_TxBuffer, msg->data, msg->size);
  HAL_UART_Transmit_IT(&huart3, midi_TxBuffer, msg->size);

  midi_messageBufferReadHead = (midi_messageBufferReadHead+1) % MIDI_MESSAGEBUFFERSIZE;


	//destroy message
	free(msg->data);
	free(msg);

}




void midi_onTxComplete(){
	midi_sendNextMessage();
}










