/*
 * hihatchannel.c
 *
 *  Created on: 23.05.2015
 *      Author: raphael
 */


#include "../Inc/hihatchannel.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "midi.h"

extern UART_HandleTypeDef huart1;

void hihatchannel_init(HiHatChannel *chan, uint32_t *dma_address, TriggerChannel *relatedTrigger){
	chan->closed = 0;
	chan->max_value = 2000;
	chan->last_send_value = 0;
	chan->intervalls_since_last_change = 0;
	chan->intervalls_since_last_controlChange = 0;
	chan->dma_value_ptr = dma_address;

	//calibration settings default
	chan->offset_to_send = 100;
	chan->send_controlChange_delay = 5;
	chan->closedValue = 700;
	chan->openedValue = 1200;
	chan->velocityFactor = 1;


	//midi settings
	chan->midi_control = 0x04;
	chan->midi_pedal_note = 44;
	chan->midi_note_open = 46;
	chan->midi_note_closed =42;
	chan->relatedTriggerpad = relatedTrigger;
	chan->sendControlChange = 1;
}


int buffer[20];
void hihatchannel_process(HiHatChannel *chan){
	uint32_t value = *chan->dma_value_ptr;

	//check maximum value;
	if(value > chan->max_value) chan->max_value = value;




	//check sending
	if(chan->last_send_value > value + chan->offset_to_send ||
			chan->last_send_value + chan->offset_to_send < value ){
			hihatchannel_onChange(chan, value);

			chan->intervalls_since_last_change = 0;
	}
	chan->intervalls_since_last_change++;
	chan->intervalls_since_last_controlChange++;


}

void hihatchannel_onChange(HiHatChannel *chan, uint32_t value){



	//process hihat pedal midinote
	if( value < chan->closedValue && !chan->closed){
		chan->closed = 1;
		uint8_t velocity =  (128 * chan->velocityFactor) /chan->intervalls_since_last_change;
		if(velocity > 127) velocity = 127;
		midi_sendNote(10, chan->midi_pedal_note, velocity);

		//set midi note of the related trigger
		chan->relatedTriggerpad->midinote = chan->midi_note_closed;

	}

	if(value > chan->openedValue){
		chan->closed = 0;
		chan->relatedTriggerpad->midinote = chan->midi_note_open;
	}

	//process control change
	if(chan->sendControlChange && chan->intervalls_since_last_controlChange >= chan->send_controlChange_delay){
		uint8_t velocity = 127 - ((value * 127) /chan->max_value) ;
		midi_sendControlChange(10, chan->midi_control, velocity);
		chan->intervalls_since_last_controlChange = 0;
	}

	//update last send value
	chan->last_send_value = value;

}


