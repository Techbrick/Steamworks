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

private:

	I2C thearduino;
	Timer thetimer;
	unsigned int ultrasonic_sample;
	double ms_poll_period;
	static constexpr uint32_t num_sr04 = 6;
	float ultrasonicreturns[num_sr04];
	static constexpr uint32_t num_lc = 9;
	uint8_t lightcanonbrightness[num_lc];
	double ms_last_poll;

	void init();
	void UpdateUltrasonics();
	void UpdateLightCanons();


public:

	static constexpr unsigned int UltrasonicFrontLeft = 1;
	static constexpr unsigned int UltrasonicFrontRight = 2;
	static constexpr unsigned int UltrasonicCenterRight = 3;
	static constexpr unsigned int UltrasonicBackRight = 4;
	static constexpr unsigned int UltrasonicBackLeft = 5;
	static constexpr unsigned int UltrasonicCenterLeft = 6;

	static constexpr unsigned int LightCanonFront = 1;
	static constexpr unsigned int LightCanonBack = 2;

	Arduino();
	Arduino(int i2caddr);
	Arduino(int i2caddr, int msrefreshrate);
	virtual ~Arduino();

	float GetUltrasonicReading(uint32_t UltrasonicID);
	void SetLightCanon(uint32_t LightCanonID, uint8_t brightness);



};

float bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
unsigned int bytesToUnsignedInt(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);


#endif /* SRC_ARDUINO_H_ */
