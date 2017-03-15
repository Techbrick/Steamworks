/*
 * Arduino.c
 *
 *  Created on: Mar 14, 2017
 *      Author: Moonlight.Admin
 */

#include "Arduino.h"

Arduino::Arduino() :
		thearduino(frc::I2C::kOnboard, 6),
		thetimer()
{
	init();
}

Arduino::Arduino(int i2caddr) :
		thearduino(frc::I2C::kOnboard, i2caddr),
		thetimer()
{
	init();
}

void Arduino::init(){
	ms_last_poll = 0;
	ms_poll_period = .025; // We expect the arduino to update once every ~100ms, so we should poll it a little faster than that.
	for(unsigned int i = 0; i<num_sr04; i++){
		ultrasonicreturns[i] = 0;
	}
	for(unsigned int i = 0; i<num_lc; i++){
		lightcanonbrightness[i] = 0;
	}
	ultrasonic_sample = 0;
	thetimer.Start();
}

float Arduino::GetUltrasonicReading(uint32_t UltrasonicID){
	// wish I could use GetPPCTimestamp(), but that doesn't seem to be available.
	double now = thetimer.Get();
	if (now - ms_last_poll > ms_poll_period){
		ms_last_poll = now;
		// TODO: Query I2C
		UpdateUltrasonics();
	}
	return ultrasonicreturns[UltrasonicID-1];
}

void Arduino::SetLightCanon(uint32_t LightCanonID, uint8_t brightness){
	lightcanonbrightness[LightCanonID-1] = brightness;
	UpdateLightCanons();
}

void Arduino::UpdateUltrasonics(){
	uint8_t toSend[10];//array of bytes to send over I2C
	uint8_t toReceive[50];//array of bytes to receive over I2C
	uint8_t numToSend = 2;//number of bytes to send
	uint8_t numToReceive = 28;//number of bytes to receive
	toSend[0] = lightcanonbrightness[LightCanonFront-1];
	toSend[1] = lightcanonbrightness[LightCanonBack-1];
	thearduino.Transaction(toSend, numToSend, toReceive, numToReceive);
	ultrasonic_sample = bytesToUnsignedInt(toReceive[0], toReceive[1], toReceive[2], toReceive[3]);
	for(unsigned int i = 0; i < num_sr04; i++){
		ultrasonicreturns[i] = bytesToFloat(toReceive[(i+1)*4+0],toReceive[(i+1)*4+1],toReceive[(i+1)*4+2],toReceive[(i+1)*4+3]);
	}
}

void Arduino::UpdateLightCanons(){
	uint8_t toSend[10];//array of bytes to send over I2C
	uint8_t toReceive[50];//array of bytes to receive over I2C
	uint8_t numToSend = 2;//number of bytes to send
	uint8_t numToReceive = 0;//number of bytes to receive
	toSend[0] = lightcanonbrightness[LightCanonFront-1];
	toSend[1] = lightcanonbrightness[LightCanonBack-1];
	thearduino.Transaction(toSend, numToSend, toReceive, numToReceive);
}

float bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
 float result;
 unsigned char* sResult = (unsigned char*)&result; // (unsigned char*) might be needed.
 sResult[3] = b3;
 sResult[2] = b2;
 sResult[1] = b1;
 sResult[0] = b0;
 return result;
}

unsigned int bytesToUnsignedInt(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
 unsigned int result;
 unsigned char* sResult = (unsigned char*)&result; // (unsigned char*) might be needed.
 sResult[3] = b3;
 sResult[2] = b2;
 sResult[1] = b1;
 sResult[0] = b0;
 return result;
}

Arduino::~Arduino(){
	for(int i = 0; i<9; i++){
		lightcanonbrightness[i] = 0;
	}
	UpdateLightCanons();
}
