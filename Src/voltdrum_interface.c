/*
 * midi.c
 *
 *  Created on: 19.05.2015
 *      Author: raphael
 */

#include "voltdrum_interface.h"

extern UART_HandleTypeDef huart1;

void voltdrum_sendDrumtriggerEvent(TriggerChannel *triggerChannel, uint8_t velocity){


	//note on
	uint8_t bfrsize = 6;
	uint8_t* bfr = malloc(sizeof(uint8_t)*bfrsize);

//	bfr[0] = 0xFF & ( 0x90 | channel  & 0xF);
//	bfr[1] = note & 0x7F;
//	bfr[2] = velocity & 0x7F;
//
//	//note off
//	bfr[3] = 0xFF & ( 0x80 | channel  & 0xF);
//	bfr[4] = note & 0x7F;
//	bfr[5] = velocity & 0x7F;

	voltdrum_createmessage(bfrsize, bfr);


}

void voltdrum_sendHiHatChange(HiHatChannel * hihatChannel, uint8_t value){

	uint8_t bfrsize = 3;
	uint8_t* bfr = malloc(sizeof(uint8_t)*bfrsize);

//	bfr[0] = 0xFF & ( 0b10110000 | channel & 0xF);
//	bfr[1] = control & 0x7F;
//	bfr[2] = value & 0x7F;

	voltdrum_createmessage(bfrsize, bfr);

}












