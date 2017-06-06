//Contains code from https://github.com/kevinkessler/TelemetryBridge
#pragma once
#include <stdio.h>
#include <stdlib.h>
#ifndef TELEMETRY_UTILS
#define ToDeg(x) (x*57.2957795131)  // *180/pi
#define ToRad(x) (x*0.01745329252)  // *pi/180 0,01745329252
#define ToRadFromWGS84(x) (x*0.000000001745329252)


boolean getBit(byte targetByte, byte bit) {
	return (boolean) targetByte & (1 << bit);
}

byte setBit(byte &targetByte, byte bit, boolean val) {
	if (val) targetByte = targetByte | (1 << bit);
	else targetByte = targetByte & ~(1 << bit);
	return targetByte;
}

// Divide by 10 fixed-point
uint16_t divideBy10(uint16_t dividend)
{
	return (uint16_t)(((uint32_t)dividend * (uint32_t)0xCCCD) >> 16) >> 3;
}

// Divide by 100 fixed-point
uint32_t divideBy100(int32_t dividend)
{
	return (uint32_t)((((((int64_t)dividend*(int64_t)0x47AF) >> 16) + dividend) >> 1) >> 6);
}
int32_t divideInt32By10(int32_t dividend)
{
	return (int32_t)(((int64_t)dividend * (int64_t)0xCCCD) >> 16) >> 3;
}


uint16_t divideBy1E7(uint32_t dividend)
{
	return (uint16_t)((((int64_t)dividend * 0xD6E0LU) >> 16) >> 23);
}
//source
//https://github.com/kevinkessler/TelemetryBridge/blob/master/main.c
//return 1 when minus sign
uint8_t parseCoord(uint8_t *deg, uint8_t *min, uint8_t *sec, uint16_t *subSec, int32_t coord)
{
	uint8_t retval = 0;
	if (coord < 0)
	{
		coord = -coord;
		retval = 1;
	}
	*deg = divideBy1E7(coord);
	coord -= *deg * 10000000;
	coord *= 60;
	*min = divideBy1E7(coord);
	coord -= *min * 10000000;
	coord *= 60;
	*sec = divideBy1E7(coord);
	coord -= *sec * 10000000;
	*subSec = coord/10000;
	return retval;

}

#endif
