/*
 * midi.c
 *
 *  Created on: 19.05.2015
 *      Author: raphael
 */

#include "voltdrum_interface.h"
#include "stdlib.h"

extern UART_HandleTypeDef huart1;


void voltdrum_init(){
	voltdrumMessagebuffer = uartmessagebuffer_create(VOLTDRUM_TXMESSAGEBUFFERSIZE,VOLTDRUM_TXBUFFERSIZE, &huart1);
}


void voltdrum_sendDrumtriggerEvent(TriggerChannel *triggerChannel, uint8_t velocity){
	//note on
	uint8_t bfrsize = 2;
	uint8_t* bfr = malloc(sizeof(uint8_t)*bfrsize);

	bfr[1] = 0b10000000;
	bfr[0] = velocity & 0x7F;

	uartmessagebuffer_createmessage(voltdrumMessagebuffer, bfrsize, bfr);

}

void voltdrum_sendHiHatChange(HiHatChannel * hihatChannel, uint8_t value){

	uint8_t bfrsize = 3;
	uint8_t* bfr = malloc(sizeof(uint8_t)*bfrsize);

//	bfr[0] = 0xFF & ( 0b10110000 | channel & 0xF);
//	bfr[1] = control & 0x7F;
//	bfr[2] = value & 0x7F;

	//voltdrum_createmessage(bfrsize, bfr);

}












