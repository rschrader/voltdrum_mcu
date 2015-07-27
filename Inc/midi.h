/*
 * midi.h
 *
 *  Created on: 19.05.2015
 *      Author: raphael
 */

#ifndef INC_MIDI_H_
#define INC_MIDI_H_

#include "uartmessagebuffer.h"
#include "inttypes.h"

#define MIDI_TXMESSAGEBUFFERSIZE 10
#define MIDI_TXBUFFERSIZE 10

UartMessageBuffer *midiMessagebuffer;

void midi_init();
void midi_sendNote(uint8_t channel, uint8_t note, uint8_t velocity);
void midi_sendControlChange(uint8_t channel, uint8_t control, uint8_t value);



#endif /* INC_MIDI_H_ */
