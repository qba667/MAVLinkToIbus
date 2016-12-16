// iBus telemetry dongle converter for PIC 12F1572
//
// Copyright Dave Borthwick 2015
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//    
// 
// Multisensor implementation for Arduino Micro
// Copyright Jakub Klimasz 2016
#pragma once
#include <stdio.h>
#include <stdlib.h>
#ifndef IBUS_TELEMETRY
#define IBUS_TELEMETRY
#define IBUS_CMD_TIMER_SYNC		0x81	// Command to synchronize a telemetry
#define IBUS_CMD_TEL_DISC		0x80	// Command to discover a telemetry module
#define IBUS_CMD_TEL_TYPE		0x90    // Command to discover type of a telemetry module
#define IBUS_CMD_TEL_MEAS		0xA0    // Command to get measurement from a telemetry module

#define IBUS_MEAS_TYPE_INTV		0x00    // Internal Voltage
#define IBUS_MEAS_TYPE_TEM		0x01    // Temperature
#define IBUS_MEAS_TYPE_MOT		0x02    // RPM
#define IBUS_MEAS_TYPE_EXTV		0x03    // External Voltage
#define IBUS_MEAS_TYPE_PRES		0x41    // Pressure
#define IBUS_MEAS_TYPE_ODO1		0x7c    // Odometer1
#define IBUS_MEAS_TYPE_ODO2		0x7d    // Odometer2
#define IBUS_MEAS_TYPE_SPE		0x7e    // Speed			//2byte km/h
#define IBUS_MEAS_TYPE_ALT		0xf9    // Altitude			//2 bytes signed in m
#define IBUS_MEAS_TYPE_SNR		0xfa    // SNR
#define IBUS_MEAS_TYPE_NOISE	0xfb    // Noise
#define IBUS_MEAS_TYPE_RSSI		0xfc    // RSSI
#define IBUS_MEAS_TYPE_ERR		0xfe    // Error rate

/////////////////////////////////////////////////////
//CUSTOM
///////////////////////////// up to 7 bytes
//LONG NAMES
//will be handled by build in km/h formatting - format method must be checked
#define IBUS_MEAS_TYPE_GPS_GROUND_SPEED	IBUS_MEAS_TYPE_SPE //2 bytes 

#define IBUS_MEAS_TYPE_RESERVED			0x0f //2 bytes Pressure in
//map to name - for now display just value
#define IBUS_MEAS_TYPE_FLIGHT_MODE		0x0e //2 bytes simple index listed below
//modes
//0 "Unknown"
//1 "Stab\0\0\0"
//2 "AHold\0\0"
//3 "Loiter\0"
//4 "RTL\0\0\0\0"
//5 "Auto\0\0\0"
//6 "Acro\0\0\0"
//7 "Land\0\0\0"
//8 "PosHold"
//map to name - for now display just value
#define IBUS_MEAS_TYPE_ARMED			0x0d //2 bytes
//just format to "%um"
#define IBUS_MEAS_TYPE_GPS_DIST			0x0c //calc?
#define IBUS_MEAS_TYPE_GROUND_SPEED		0x0b //2byte m/s *100 different unit than build-in sensor
//check against 0x8000
//when set add - to buffer and move pointer
//div by 100
//format into "%u.%02u m/s"
#define IBUS_MEAS_TYPE_VERTICAL_SPEED	0x0a //2byte m/s *100	
//check against 0x8000
//when set add - to buffer and move pointer
//div by 100
//format into "%u.%02u�"
#define IBUS_MEAS_TYPE_ACC_X			0x09 //pitch 2bytes	deg * 100.0 signed!!!
#define IBUS_MEAS_TYPE_ACC_Y			0x08 //roll 2bytes deg * 100.0 	signed!!!
#define IBUS_MEAS_TYPE_ACC_Z			0x07 //yaw 2bytes deg * 100.0	signed!!!
//just other format %u.%02u m/s
#define IBUS_MEAS_TYPE_CLIMB_RATE		0x06 //2bytes m/s *100
//check against 0x8000
//when set add return UNKNOWN
//div by 100
//format into "%u.%02u"
#define IBUS_MEAS_TYPE_BAT_CURR			0x05 //battery current  10*milliamperes (1 = 10 milliampere) 2byts
//use fake decimal 00
//format into "%u.%02u"
#define IBUS_MEAS_TYPE_CMP_HEAD			0x04 //Heading  0..360 deg, 0=north 2bytes	


////////////////////////////////////////////////////////////////
//when requested directly return UNKNOWN sensor and no formating  
#define IBUS_MEAS_TYPE_GPS				0xfd //1byte fix 1 byte satellites 4 bytes LAT 4 bytes LON 4bytes alt
/////////////////////////////
//SHAORT NAMES
#define IBUS_MEAS_TYPE_COG				0x80 //2bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
//special parse byte by byte
#define IBUS_MEAS_TYPE_GPS_STATUS		0x81 //2bytes 
//check against 0x80000000
//when set add - to buffer and move pointer
//later use divide method
//mask 0x80000000 to remove sign
//format into "%u�%02u'%02u"
#define IBUS_MEAS_TYPE_GPS_LON			0x82 //4bytes signed WGS84 in degrees * 1E7 
#define IBUS_MEAS_TYPE_GPS_LAT			0x83 //4bytes signed WGS84 in degrees * 1E7 
#define IBUS_MEAS_TYPE_ALT_CUSTOM		0x84 //2bytes signed!!! barometer alt
//check against 0x80000000
//when set add - to buffer and move pointer
//div by 100
//format into "%u.%02u m"
//already defined
//#define IBUS_MEAS_TYPE_GPS_ALT			0xf9 //2bytes in m signed!!!



/////////////////////////////////////////////////////

// macros to code the different measurement types
#define TEM_1(x) ((400+(x))&0xFF)
#define TEM_2(x) ((400+(x))>>8)
#define LBYTE(x) ((x)&0xFF)
#define HBYTE(x) ((x)>>8&0xFF)


#define IBUS_BUFFSIZE 33				// biggest iBus message seen so far + 1
#define IBUS_HEADER_FOOTER_SIZE 4

typedef struct IBUS_SENSOR {
	uint8_t type;
	uint8_t payloadSize;
	uint8_t address;
} IBUS_SENSOR;

//serial specific method must be defined
void usart_send(uint8_t* bytes, uint8_t len);

uint8_t ibuxRxBuffer[IBUS_BUFFSIZE];	// IBUS frame receive buffer
uint8_t ibusFrameTX[IBUS_BUFFSIZE];		// IBUS frame transmit buffer
uint8_t volatile ibusFrameCount;					// Count of bytes received for current IBUS frame
uint8_t volatile ibusGapDetect;					// used to detect time between frames
										// set to 1 every-time a byte is received
										// set to 0 every timer 0
										// if it is still 0 after a timer 0 period then

//never try define more than F sensors
IBUS_SENSOR sensorsList[] = {
	{ .type = IBUS_MEAS_TYPE_FLIGHT_MODE,.payloadSize = 2,.address = 0 },
	//{.type = IBUS_MEAS_TYPE_EXTV,			.payloadSize = 2,.address = 0 },
	//{.type = IBUS_MEAS_TYPE_TEM,			.payloadSize = 2,.address = 0 },
	{.type = IBUS_MEAS_TYPE_ALT,			.payloadSize = 2,.address = 0 },
	{.type = IBUS_MEAS_TYPE_SPE,			.payloadSize = 2,.address = 0 },
	{.type = IBUS_MEAS_TYPE_VERTICAL_SPEED,	.payloadSize = 2,.address = 0 },
	{.type = IBUS_MEAS_TYPE_ACC_X,			.payloadSize = 2,.address = 0 },
	{.type = IBUS_MEAS_TYPE_ACC_Y,			.payloadSize = 2,.address = 0 },
	{.type = IBUS_MEAS_TYPE_ACC_Z,			.payloadSize = 2,.address = 0 },
	//{.type = IBUS_MEAS_TYPE_BAT_CURR,		.payloadSize = 2,.address = 0 },
	{.type = IBUS_MEAS_TYPE_CLIMB_RATE,		.payloadSize = 2,.address = 0 },
	{.type = IBUS_MEAS_TYPE_CMP_HEAD,		.payloadSize = 2,.address = 0 },
	//{.type = IBUS_MEAS_TYPE_ARMED,			.payloadSize = 2,.address = 0 },
	{.type = IBUS_MEAS_TYPE_GPS_STATUS,		.payloadSize = 2,.address = 0 },
	//{.type = IBUS_MEAS_TYPE_PRES,			.payloadSize = 2,.address = 0 },
	
};

uint8_t sensors = sizeof(sensorsList) / sizeof(IBUS_SENSOR);

// Checksum is 0xFFFF - sum of all bytes except last
// two. These last two are the little endian checksum
uint16_t getChecksum(uint8_t* buffer) {
	uint16_t checksum = 0xFFFF;
	uint8_t dataSize = buffer[0] - 2;
	for (uint8_t i = 0; i < dataSize; i++) {
		checksum -= buffer[i];
	}
	return checksum;
}

void setChecksum(uint8_t* buffer) {
	uint16_t checksum = getChecksum(buffer);
	uint8_t dataSize = buffer[0] - 2;
	buffer[dataSize] = checksum & 0xFF;
	buffer[dataSize + 1] = checksum >> 8;
}

//Method fill provided buffer with sensor definition
//used as response for command IBUS_CMD_TEL_TYPE
//buffer - pointer to packet buffer
//index - sensor index used to resolve type and data length

void getSensorType(uint8_t* buffer, uint8_t index) {
	buffer[0] = IBUS_HEADER_FOOTER_SIZE + 2;
	buffer[1] = IBUS_CMD_TEL_TYPE + sensorsList[index].address;
	buffer[2] = sensorsList[index].type;
	buffer[3] = sensorsList[index].payloadSize;
	setChecksum(buffer);
}
//int16_t GetInt16TelemetryValue(uint8_t sensorType);

void setTelemetryValueToBuffer(uint8_t* buffer, uint8_t sensorType, uint8_t length);

//Method fill provided buffer with sensor data
//used as response for command IBUS_CMD_TEL_MEAS
//buffer - pointer to packet buffer
//index - sensor index used to resolve type and data length
  uint16_t tmp = 0;
void getSensorData(uint8_t* buffer, uint8_t index) {
	buffer[0] = IBUS_HEADER_FOOTER_SIZE + sensorsList[index].payloadSize;
	buffer[1] = IBUS_CMD_TEL_MEAS + sensorsList[index].address;
	setTelemetryValueToBuffer(buffer + 2, sensorsList[index].type, sensorsList[index].payloadSize);
	setChecksum(buffer);
}

//RX byte from IBUS port return true when packet is completed
bool ibusRXByte(uint8_t value) {
	ibuxRxBuffer[ibusFrameCount] = value;
	ibusGapDetect = 1;
	if (ibusFrameCount < IBUS_BUFFSIZE - 1) ibusFrameCount++;
	// Idle processing - take the data from the ibus buffer and process
	// Error conditions requiring restart of frame
	if ((ibusFrameCount > 0 && ibuxRxBuffer[0] > IBUS_BUFFSIZE - 1) ||
		(ibusFrameCount > 0 && ibuxRxBuffer[0] < 4) ||
		(ibusFrameCount > 0 && ibuxRxBuffer[0] % 2 == 1) ||
		((ibusFrameCount > 1 && (ibuxRxBuffer[1] & 0xF0) != IBUS_CMD_TEL_DISC) &&
		(ibusFrameCount > 1 && (ibuxRxBuffer[1] & 0xF0) != IBUS_CMD_TEL_TYPE) &&
			(ibusFrameCount > 1 && (ibuxRxBuffer[1] & 0xF0) != IBUS_CMD_TEL_MEAS)))
	{
		ibusFrameCount = 0;
	}
	// Full frame check it is good
	else if (ibusFrameCount > 3 && ibusFrameCount >= ibuxRxBuffer[0])
	{
		// reset the frame regardless if it is a good one
		// or it failed the checksum
		ibusFrameCount = 0;
		// Complete frame - calculate checksum
		uint8_t length = ibuxRxBuffer[0];
		return getChecksum(ibuxRxBuffer) == ((ibuxRxBuffer[length - 1] << 8) + ibuxRxBuffer[length - 2]);
	}
	return false;
}


void ibusTX() {
	switch (ibuxRxBuffer[1] & 0xF0)
	{
	case IBUS_CMD_TEL_DISC:
		for (uint8_t i = 0; i < sensors; i++) {
			// We haven't responded yet or we are being asked again
			if (sensorsList[i].address == 0 || sensorsList[i].address == (ibuxRxBuffer[1] & 0x0F))
			{
				// We haven't responded yet or we are being asked again
				sensorsList[i].address = ibuxRxBuffer[1] & 0x0F;
				memcpy(ibusFrameTX, ibuxRxBuffer, ibuxRxBuffer[0]);
				usart_send(ibusFrameTX, ibusFrameTX[0]);
				break;
			}
		}
		break;
	case IBUS_CMD_TEL_TYPE:
		for (uint8_t i = 0; i < sensors; i++) {
			if (sensorsList[i].address == (ibuxRxBuffer[1] & 0x0F))
			{
				getSensorType(ibusFrameTX, i);
				usart_send(ibusFrameTX, ibusFrameTX[0]);
				break;
			}
		}
		break;
	case IBUS_CMD_TEL_MEAS:
		for (uint8_t i = 0; i < sensors; i++) {
			if (sensorsList[i].address == (ibuxRxBuffer[1] & 0x0F))
			{
				getSensorData(ibusFrameTX, i);
				usart_send(ibusFrameTX, ibusFrameTX[0]);
				break;
			}
		}
		break;
	}
}


#endif // IBUS_TELEMETRY
