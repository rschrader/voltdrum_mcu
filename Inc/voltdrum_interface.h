/*
 * midi.h
 *
 *  Created on: 19.05.2015
 *      Author: raphael
 */

#ifndef INC_VOLTDRUM_INTERFACE_H_
#define INC_VOLTDRUM_INTERFACE_H_

#include "inttypes.h"
#include "triggerchannel.h"
#include "hihatchannel.h"
#include "uartmessagebuffer.h"


#define VOLTDRUM_TXMESSAGEBUFFERSIZE 10
#define VOLTDRUM_TXBUFFERSIZE 10

UartMessageBuffer *voltdrumMessagebuffer;

void voltdrum_init();
void voltdrum_sendDrumtriggerEvent(TriggerChannel *triggerChannel, uint8_t velocity);
void voltdrum_sendHiHatChange(HiHatChannel * hihatChannel, uint8_t value);



#endif /* INC_VOLTDRUM_INTERFACE_H_ */
