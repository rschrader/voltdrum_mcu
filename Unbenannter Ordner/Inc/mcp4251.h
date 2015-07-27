/*
 * MCP4251.h
 *
 *  Created on: 13.04.2015
 *      Author: raphael
 */

#ifndef INC_MCP4251_H_
#define INC_MCP4251_H_

#include "stm32f3xx_hal.h"
#include "gpio.h"
#include "spi.h"

#define MCP4261_COMAND_READ 0b11000000
#define MCP4261_COMAND_WRITE 0b00000000
#define MCP4261_COMAND_INC 0b01000000
#define MCP4261_COMAND_DEC 0b10000000

#define MCP4261_WIPER0 0x00
#define MCP4261_WIPER1 0x10

void mcp4261_setWiperPosition(const GPIO_TypeDef *csGpioGroup, const uint16_t csGpioPin, const uint8_t wiperId, const uint8_t wiperPosition);



#endif /* INC_MCP4251_H_ */
