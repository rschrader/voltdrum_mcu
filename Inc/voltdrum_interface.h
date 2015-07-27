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



void voltdrum_sendDrumtriggerEvent(TriggerChannel *triggerChannel, uint8_t velocity);
void voltdrum_sendHiHatChange(HiHatChannel * hihatChannel, uint8_t value);



#endif /* INC_VOLTDRUM_INTERFACE_H_ */
