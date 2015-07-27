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

void midi_init(){
	midiMessagebuffer = uartmessagebuffer_create(MIDI_TXMESSAGEBUFFERSIZE,MIDI_TXBUFFERSIZE, &huart3);
}

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

	uartmessagebuffer_createmessage(midiMessagebuffer, bfrsize, bfr);


}

void midi_sendControlChange(uint8_t channel, uint8_t control, uint8_t value){

	uint8_t bfrsize = 3;
	uint8_t* bfr = malloc(sizeof(uint8_t)*bfrsize);

	bfr[0] = 0xFF & ( 0b10110000 | channel & 0xF);
	bfr[1] = control & 0x7F;
	bfr[2] = value & 0x7F;

	//uartmessagebuffer_createmessage(midiMessagebuffer, bfrsize, bfr);

}











