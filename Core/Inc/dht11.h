/*
 * dht11.h
 *
 *  Created on: Jul 14, 2025
 *      Author: Zhang
 */

#ifndef DHT11_H_
#define DHT11_H_

#include "main.h"

typedef struct{
	uint8_t humidity_int;
	uint8_t humidity_dec;
	uint8_t temp_int;
	uint8_t temp_dec;
} DHT11_Data;


DHT11_Data DHT11_READ();

#endif /* DHT11_H_ */
