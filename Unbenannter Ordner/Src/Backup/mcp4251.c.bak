/*
 * mcp4251.c
 *
 *  Created on: 13.04.2015
 *      Author: raphael
 */
#include "mcp4251.h"

void mcp4261_setWiperPosition(const GPIO_TypeDef *csGpioGroup, const uint16_t csGpioPin, const uint8_t wiperId, const uint8_t wiperPosition) {

	uint8_t tx1 = wiperId;
	uint8_t tx2 = wiperPosition;

	//send LSByte First
	uint16_t tx = (tx1 << 8) | tx2;
	uint16_t rx = 0x0000;

	// set chipselect pin
	HAL_GPIO_WritePin(csGpioGroup,csGpioPin,GPIO_PIN_RESET);

	//transmit data
	HAL_SPI_TransmitReceive(&hspi2,&tx,&rx,2,100);

	// release chipselect pin
	HAL_GPIO_WritePin(csGpioGroup,csGpioPin,GPIO_PIN_SET);

}
