/*
 * midi.h
 *
 *  Created on: 19.05.2015
 *      Author: raphael
 */

#ifndef INC_MIDI_H_
#define INC_MIDI_H_

#include "inttypes.h"

#define MIDI_MESSAGEBUFFERSIZE 10


typedef struct{
	uint8_t *data;
	uint8_t size;
}midimessage;

uint8_t midi_TxBuffer[10];
midimessage *midi_MessageBuffer[MIDI_MESSAGEBUFFERSIZE];
uint8_t midi_messageBufferReadHead;
uint8_t midi_messageBufferWriteHead;


void midi_createmessage (uint8_t size, uint8_t* data);
void midi_sendNextMessage();
void midi_onTxComplete();


void midi_sendNote(uint8_t channel, uint8_t note, uint8_t velocity);
void midi_sendControlChange(uint8_t channel, uint8_t control, uint8_t value);



#endif /* INC_MIDI_H_ */
