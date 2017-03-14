/*
 * Arduino.h
 *
 *  Created on: Mar 14, 2017
 *      Author: Moonlight.Admin
 */

#include "WPILib.h"


#ifndef SRC_ARDUINO_H_
#define SRC_ARDUINO_H_



class Arduino{

	I2C thearduino;
	unsigned int ms_poll_rate;
	float ultrasonicreturns[8];
	uint8_t lightcanonbrightness[9];
	unsigned long ms_last_poll;

public:

	static constexpr unsigned int UltrasonicFrontLeft = 1;
	static constexpr unsigned int UltrasonicFrontRight = 2;
	static constexpr unsigned int UltrasonicCenterRight = 3;
	static constexpr unsigned int UltrasonicBackRight = 4;
	static constexpr unsigned int UltrasonicBackLeft = 5;
	static constexpr unsigned int UltrasonicCenterLeft = 6;

	static constexpr unsigned int LightCanonFront = 1;
	static constexpr unsigned int LightCanonBack = 2;

	Arduino(int i2cbus, int i2cid);

	float GetUltrasonicReading(uint32_t UltrasonicID);
	float SetLightCanon(uint32_t LightCanonID, uint8_t brightness);

};


#endif /* SRC_ARDUINO_H_ */
