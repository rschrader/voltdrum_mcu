/*
 * hihatchannel.h
 *
 *  Created on: 23.05.2015
 *      Author: raphael
 */

#ifndef INC_HIHATCHANNEL_H_
#define INC_HIHATCHANNEL_H_


#include "inttypes.h"
#include "triggerchannel.h"

typedef struct{
	uint32_t max_value;
	uint32_t last_send_value;
	uint32_t intervalls_since_last_change;
	uint32_t intervalls_since_last_controlChange;
	uint32_t closed;
	uint32_t *dma_value_ptr;

	//calibration settings
	uint32_t offset_to_send;
	uint32_t send_controlChange_delay;
	uint32_t closedValue;
	uint32_t openedValue;
	float velocityFactor;

	// midi settings
	uint32_t midi_control;
	uint32_t midi_pedal_note;
	TriggerChannel* relatedTriggerpad;
	uint32_t midi_note_open;
	uint32_t midi_note_closed;
	uint8_t sendControlChange;
}HiHatChannel;


void hihatchannel_init(HiHatChannel *chan, uint32_t *dma_adress,  TriggerChannel *relatedTrigger);
void hihatchannel_process(HiHatChannel *chan);
void hihatchannel_onChange(HiHatChannel *chan, uint32_t value);



#endif /* INC_HIHATCHANNEL_H_ */
