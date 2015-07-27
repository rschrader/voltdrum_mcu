/*
 * trigger_sm.c
 *
 *  Created on: 19.05.2015
 *      Author: raphael
 */

#include "../Inc/triggerchannel.h"

#include "mcp4251.h"

void triggerchannel_init (TriggerChannel *sm, uint32_t* dma_adress0, uint32_t* dma_adress1,uint32_t* dma_adress2,uint32_t* dma_adress3){

	//editable values
	sm->midinote = 38;
	sm->threshhold_static = 10;
	sm->threshhold_dynamic = 0.8;
	sm->samples_to_take = 3;							//15
	sm->time_offset = 100;									//400us
	sm->lastProcessedSamples_size = 15;							//400us


	//sm->wiperPosition = 60;

	//not editable values
	sm->state = TRIGGERSM_MONITOR;
	sm->max_velocity = 0;
	sm->dma_value_ptr[0] = dma_adress0;
	sm->dma_value_ptr[1] = dma_adress1;
	sm->dma_value_ptr[2] = dma_adress2;
	sm->dma_value_ptr[3] = dma_adress3;
	sm->intervallcount_since_trans = 0;
	sm->storeSampleFlag = 0;


	//set buffer
	sm->lastProcessedSamples = malloc(sizeof(uint32_t) * sm->lastProcessedSamples_size);
	sm->lastProcessedSamples_head = 0;
	int i;
	for(i = 0; i < sm->lastProcessedSamples_size; i++){
		sm->lastProcessedSamples[i] = 0;
	}
}

void triggerchannel_setWiper(TriggerChannel *chan, uint8_t wiper){
	chan->wiperPosition = wiper;
	mcp4261_setWiperPosition(chan->potiCsPort,chan->potiCsPin,chan->potiWiperId,wiper);
}

//dispatcher for process methods
void triggerchannel_process (TriggerChannel *sm){

	//get the highest value of the last 4 samples to process
	sm->current_value =  *(sm->dma_value_ptr[0]);
	uint32_t currentdmaval, i;
	for(i=1; i<4; i++){
		currentdmaval = *sm->dma_value_ptr[i];
		if(currentdmaval > sm->current_value) sm->current_value = currentdmaval;
	}

	//process common tasks before dispatch
	sm->intervallcount_since_trans++;

	//dispatch by state
	if(sm->state == TRIGGERSM_MONITOR) triggerchannel_process_monitor(sm);
	else if(sm->state == TRIGGERSM_EVENT) triggerchannel_process_triggerevent(sm);

	//process common tasks after dispatch
	triggerchannel_addSample(sm, sm->current_value);
}

//process methods for different states
void triggerchannel_process_triggerevent (TriggerChannel *sm){

	if(sm->current_value > sm->max_velocity) 								//check max value
		sm->max_velocity = sm->current_value;

	//check transition to monitor
	if(sm->intervallcount_since_trans >= sm->samples_to_take){
		triggerchannel_transition_triggerevent_monitor(sm);
	}

}


void triggerchannel_process_monitor (TriggerChannel *sm){

	//check transition
	uint32_t lastSamplesMax = triggerchannel_getLastSamplesMax(sm);
	uint32_t threshhold =  sm->threshhold_static + lastSamplesMax * sm->threshhold_dynamic;

	if( lastSamplesMax + threshhold <= sm->current_value
			&& sm->time_offset < sm->intervallcount_since_trans){

		triggerchannel_transition_monitor_triggerevent(sm);
	}

}

//State transition methods
void triggerchannel_transition_triggerevent_monitor (TriggerChannel *sm){
	sm->state = TRIGGERSM_MONITOR;
	sm->intervallcount_since_trans = 0;


	//note detected
	midi_sendNote(10,sm->midinote, sm->max_velocity >> 5);



}
void triggerchannel_transition_monitor_triggerevent (TriggerChannel *sm){
	sm->state = TRIGGERSM_EVENT;
	sm->intervallcount_since_trans = 0;
	sm->max_velocity = sm->current_value;
}

void triggerchannel_addSample(TriggerChannel *sm,uint32_t sample){

	if(sm->storeSampleFlag <= 0){
		sm->storeSampleFlag = 2;
		return;
	}
	sm->storeSampleFlag--;


	sm->lastProcessedSamples_head = (sm->lastProcessedSamples_head+1) % sm->lastProcessedSamples_size;
	sm->lastProcessedSamples[sm->lastProcessedSamples_head] = sample;

}

uint32_t triggerchannel_getLastSample(TriggerChannel *sm){
	return sm->lastProcessedSamples[sm->lastProcessedSamples_head];
}

uint32_t triggerchannel_getLastSamplesMax(TriggerChannel *sm){
	uint32_t i,max = 0;
	for(i = 0; i < sm->lastProcessedSamples_size; i++){
		if(sm->lastProcessedSamples[i] > max)
			max = sm->lastProcessedSamples[i];
	}
	return max;
}


