/*
 * performance_measurement.c
 *
 *  Created on: 23.05.2015
 *      Author: raphael
 */

#include "performance.h"

void performance_setProcessing1(){
	HAL_GPIO_WritePin(PERFORMANCE_PORT1, PERFORMANCE_PIN1, GPIO_PIN_SET);
}

void performance_setIdle1(){
	HAL_GPIO_WritePin(PERFORMANCE_PORT1, PERFORMANCE_PIN1, GPIO_PIN_RESET);

}


void performance_setProcessing2(){
	HAL_GPIO_WritePin(PERFORMANCE_PORT2, PERFORMANCE_PIN2, GPIO_PIN_SET);
}

void performance_setIdle2(){
	HAL_GPIO_WritePin(PERFORMANCE_PORT2, PERFORMANCE_PIN2, GPIO_PIN_RESET);

}

