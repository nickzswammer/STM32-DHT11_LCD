/*
 * dht11.c
 *
 *  Created on: Jul 14, 2025
 *      Author: Zhang
 */

#include "dht11.h"

  /* DHT11 Initialization Process
   * GPIO Initially Pulled High by DHT11
   * GPIO Set as output
   * 1. GPIO Pulls Low for at least 18ms (Start Signal)
   * 2. GPIO Pulls up
   * 3. GPIO Set as Input
   * 4. Wait for DHT response (20-40 us)
   * 5. DHT sends response signal LOW, keeps for 80 us
   * 6. DHT sets bus voltage level to high, keeps for 80 us, prep send data
   * 7. DHT sends 1 bit, starting with 50us low, then length of high voltage for 0 or 1
   * 	- 0 (26-28 us)
   * 	- 1 (70 us)
   * 8. next bit starts to transmit (50us)
   * 9. repeats until finished, then voltage pulls up and ready to read
   */

#define B9_PinState ((GPIOB->IDR >> 9)&0x1)

uint8_t celcius_to_fahrenheit(uint8_t temp_C){
	float temp_f = (temp_C * 9.0f / 5.0f) + 32.0f;

	return (uint8_t)temp_f;
}

void DHT11_GET_BITS(DHT11_Data *return_data){
	uint8_t bits_recieved = 0;
	uint64_t bit_stream = 0;
	uint8_t checksum;
	uint8_t computed;

	while(bits_recieved < 40){
		uint32_t timeout = 100;

		// wait until pin is high
		while(!B9_PinState && timeout--) delay_us(1);
		if(timeout == 0) return; // time out error

		// delay 40, if its still high, its a 1, if its LOW, its a 0
		delay_us(40);

		// shift bits to the left, then add the bit
		bit_stream = bit_stream << 1;
		if(B9_PinState)
			bit_stream |= 1;

		bits_recieved++;

		// wait until line goes LOW again to start next bit
		timeout = 100;
		while(B9_PinState && timeout--) delay_us(1);
		if (timeout == 0) return;

	}

	// extract 5 bytes
    uint8_t data[5];
    for (int i = 0; i < 5; i++) {
        data[i] = (bit_stream >> ((4 - i) * 8)) & 0xFF;
    }

    checksum = data[4];
    computed = (data[0] + data[1] + data[2] + data[3]) & 0xFF;

    if(checksum == computed){
    	return_data->humidity_int = data[0];
    	return_data->humidity_dec = data[1];
    	return_data->temp_int = celcius_to_fahrenheit(data[2]);
    	return_data->temp_dec= celcius_to_fahrenheit(data[3]);
    }
    else{
    	return_data->humidity_int = 0xFF;
    	return_data->humidity_dec= 0xFF;
    	return_data->temp_int = 0xFF;
    	return_data->temp_dec= 0xFF;
    }
}

DHT11_Data DHT11_READ(){
	DHT11_Data return_data = {0};

	// GPIO set as output, already pulled high
	GPIOB->MODER &= ~GPIO_MODER_MODER9_Msk; // reset

	GPIOB->MODER |= (1U << GPIO_MODER_MODE9_Pos);  // shift 1 to the 18th bit

	// GPIO Set as Open Drain
	GPIOB->OTYPER |= (1U << GPIO_OTYPER_OT9_Pos);

	// GPIO pulls low, now wait 18ms
	GPIOB->ODR &= ~GPIO_ODR_OD9_Msk;

	// wait until CNT is 20ms (each tick is 1uS, so 18,000 us)
	delay_us(20000);

	// set as INPUT (also pulls up due to open drain mode) and wait for response
	GPIOB->MODER &= ~GPIO_MODER_MODER9_Msk; // reset

	// delay 40, DHT should be sending a response now
	delay_us(40);

	// middle of DHT pulling low, wait until DHT sends high
	delay_us(40);

	// while B9 is low, wait
	while(!B9_PinState);

	// now is high, next low will signal a bit
	while(B9_PinState);

	DHT11_GET_BITS(&return_data);

	return (return_data);
}
