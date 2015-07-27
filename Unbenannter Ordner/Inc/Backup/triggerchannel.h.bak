/*
 * trigger_sm.h
 *
 *  Created on: 19.05.2015
 *      Author: raphael
 */

#ifndef INC_TRIGGERCHANNEL_H_
#define INC_TRIGGERCHANNEL_H_

#include "inttypes.h"
#include "stm32f3xx_hal.h"

typedef enum{
	TRIGGERSM_EVENT,
	TRIGGERSM_MONITOR
}TriggerSM_State;


typedef struct{
	//addresses of the last sample values. will be filled over DMA
	volatile uint32_t *dma_value_ptr[4];

	// Statusdaten, die bei der Verarbeitung der Statemachine benötigt werden
	uint32_t intervallcount_since_trans;		// Intervallzähler seit letzter transition
	uint32_t max_velocity;									// Merker zur ermittlung der Velocity
	TriggerSM_State state;									// Status des Zustandsautomaten
	uint32_t *lastProcessedSamples;					// Buffer für die zuletzt beatrbeiteten Samples
	uint32_t lastProcessedSamples_size;			// Buffergröße
	uint32_t lastProcessedSamples_head;			// Bufferhead
	uint32_t storeSampleFlag;								// Statusvariable für den Buffer
	uint32_t current_value;									// aktuelles Sample

	// Konfigurierbare Daten für den Zusatndsautomaten
	uint32_t threshhold_static;							// fester threshhold zur Schlagerkennung
	float threshhold_dynamic;								// dynamisches threshhold zur Schlagerkennung
	uint32_t samples_to_take;
	uint32_t time_offset;
	uint32_t midinote;

	// variables for amplifier control
	GPIO_TypeDef* potiCsPort;
	uint16_t potiCsPin;
	uint8_t potiWiperId;
	uint8_t wiperPosition;

}TriggerChannel;



void triggerchannel_init (TriggerChannel *sm, uint32_t* dma_adress0, uint32_t* dma_adress1,uint32_t* dma_adress2,uint32_t* dma_adress3);
void triggerchannel_setWiper(TriggerChannel *chan, uint8_t wiper);

//dispatcher for process methods
void triggerchannel_process (TriggerChannel *sm);


//process methods for different states
void triggerchannel_process_triggerevent (TriggerChannel *sm);
void triggerchannel_process_monitor (TriggerChannel *sm);

//State transition methods
void triggerchannel_transition_triggerevent_monitor (TriggerChannel *sm);
void triggerchannel_transition_monitor_triggerevent (TriggerChannel *sm);

//helper
void triggerchannel_addSample(TriggerChannel *sm,uint32_t sample);
uint32_t triggerchannel_getLastSample(TriggerChannel *sm);
uint32_t triggerchannel_getLastSamplesMax(TriggerChannel *sm);

#endif /* INC_TRIGGERCHANNEL_H_ */
