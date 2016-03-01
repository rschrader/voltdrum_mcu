/*
 * midi.c
 *
 *  Created on: 19.05.2015
 *      Author: raphael
 */

#include "voltdrum_interface.h"
#include "stdlib.h"
#include "voltdrum_protocoll.h"

extern UART_HandleTypeDef huart1;


void voltdrum_init(){
	voltdrumMessagebuffer = uartmessagebuffer_create(VOLTDRUM_TXMESSAGEBUFFERSIZE,VOLTDRUM_TXBUFFERSIZE, &huart1);
}


void voltdrum_sendDrumtriggerEvent(TriggerChannel *triggerChannel, uint8_t velocity){
	//note on
	uint8_t bfrsize = 3;
	uint8_t* bfr = malloc(sizeof(uint8_t)*bfrsize);

	bfr[2] = MASK_TYPE | (MASK_CONTENT & CMD_TYPE_DRUMTRIGGER);
	bfr[1] = MASK_CONTENT & triggerChannel->voltdrumChannel;
	bfr[0] = MASK_CONTENT & velocity;

	uartmessagebuffer_createmessage(voltdrumMessagebuffer, bfrsize, bfr);

}

void voltdrum_sendHiHatChange(HiHatChannel * hihatChannel, uint8_t value){

	//note on
	uint8_t bfrsize = 3;
	uint8_t* bfr = malloc(sizeof(uint8_t)*bfrsize);

	bfr[2] = MASK_TYPE | (MASK_CONTENT & CMD_TYPE_HIHAT);
	bfr[1] = MASK_CONTENT & hihatChannel->voltdrumChannel;
	bfr[0] = MASK_CONTENT & value;

	uartmessagebuffer_createmessage(voltdrumMessagebuffer, bfrsize, bfr);

}












