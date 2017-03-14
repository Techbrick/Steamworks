/*
 * Arduino.c
 *
 *  Created on: Mar 14, 2017
 *      Author: Moonlight.Admin
 */

//#include "Arduino.h"

Arduino::Arduino(int i2cbus, int i2cid)
{
	ms_poll_rate = 55;
	ms_last_poll = 0;

}
