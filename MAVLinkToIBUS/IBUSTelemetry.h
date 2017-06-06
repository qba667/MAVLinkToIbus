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
#include "ppm.h"
#ifndef IBUS_TELEMETRY
#define IBUS_TELEMETRY
#define IBUS_CMD_TIMER_SYNC		0x81	// Command to synchronize a telemetry
#define IBUS_CMD_TEL_DISC		0x80	// Command to discover a telemetry module
#define IBUS_CMD_TEL_TYPE		0x90    // Command to discover type of a telemetry module
#define IBUS_CMD_TEL_MEAS		0xA0    // Command to get measurement from a telemetry module
#define IBUS_CMD_CHANNEL		0x40    // Data of 14 ibus channels	

//IF PATCHED IA6B is used we can send aggregated data.
#define IA6B_PATCHED

/////////////////////////////////////////////////////
#define IBUS_MEAS_TYPE_INTV				0x00    // Internal Voltage
#define IBUS_MEAS_TYPE_TEM				0x01    // Temperature
#define IBUS_MEAS_TYPE_MOT				0x02    // RPM
#define IBUS_MEAS_TYPE_EXTV				0x03    // External Voltage
#define IBUS_MEAS_TYPE_CELL				0x04    // Avg Cell voltage
#define IBUS_MEAS_TYPE_BAT_CURR			0x05    // battery current A * 100
#define IBUS_MEAS_TYPE_FUEL				0x06	// remaining battery percentage / mah drawn otherwise or fuel level no unit!
#define IBUS_MEAS_TYPE_RPM				0x07	// throttle value / battery capacity
#define IBUS_MEAS_TYPE_CMP_HEAD			0x08 	//Heading  0..360 deg, 0=north 2bytes
#define IBUS_MEAS_TYPE_CLIMB_RATE 		0x09  	//2 bytes m/s *100
#define IBUS_MEAS_TYPE_COG				0x0a 	//2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
#define IBUS_MEAS_TYPE_GPS_STATUS		0x0b 	//2 bytes
#define IBUS_MEAS_TYPE_ACC_X			0x0c 	//2 bytes m/s *100 signed
#define IBUS_MEAS_TYPE_ACC_Y			0x0d 	//2 bytes m/s *100 signed
#define IBUS_MEAS_TYPE_ACC_Z			0x0e 	//2 bytes m/s *100 signed
#define IBUS_MEAS_TYPE_ROLL				0x0f 	//2 bytes deg *100 signed
#define IBUS_MEAS_TYPE_PITCH			0x10 	//2 bytes deg *100 signed
#define IBUS_MEAS_TYPE_YAW				0x11 	//2 bytes deg *100 signed
#define IBUS_MEAS_TYPE_VERTICAL_SPEED	0x12 	//2 bytes m/s *100
#define IBUS_MEAS_TYPE_GROUND_SPEED		0x13 	//2 bytes m/s *100 different unit than build-in sensor
#define IBUS_MEAS_TYPE_GPS_DIST			0x14 	//2 bytes dist from home m unsigned
#define IBUS_MEAS_TYPE_ARMED			0x15 	//2 bytes
#define IBUS_MEAS_TYPE_FLIGHT_MODE		0x16 	//2 bytes simple index listed below

#define IBUS_MEAS_TYPE_PRES				0x41    // Pressure
#define IBUS_MEAS_TYPE_ODO1				0x7c    // Odometer1
#define IBUS_MEAS_TYPE_ODO2				0x7d    // Odometer2
#define IBUS_MEAS_TYPE_SPE				0x7e    // Speed			//2byte km/h
#define IBUS_MEAS_TYPE_TX_V				0x7f    // TX Voltage

//4 byte sensors
#define IBUS_MEAS_TYPE_GPS_LAT			0x80 //4bytes signed WGS84 in degrees * 1E7
#define IBUS_MEAS_TYPE_GPS_LON			0x81 //4bytes signed WGS84 in degrees * 1E7
#define IBUS_MEAS_TYPE_GPS_ALT			0x82 //4bytes signed!!! GPS alt m*100
#define IBUS_MEAS_TYPE_ALT				0x83 //4bytes signed!!! Alt m*100
#define IBUS_MEAS_TYPE_S84				0x84
#define IBUS_MEAS_TYPE_S85				0x85
#define IBUS_MEAS_TYPE_S86				0x86
#define IBUS_MEAS_TYPE_S87				0x87
#define IBUS_MEAS_TYPE_S88				0x88
#define IBUS_MEAS_TYPE_S89				0x89
#define IBUS_MEAS_TYPE_S8a				0x8a

//#define IBUS_MEAS_TYPE_ALT_FLYSKY		0xf9    // Altitude			//2 bytes signed in m
#define IBUS_MEAS_TYPE_SNR				0xfa    // SNR
#define IBUS_MEAS_TYPE_NOISE			0xfb    // Noise
#define IBUS_MEAS_TYPE_RSSI				0xfc    // RSSI
#define IBUS_MEAS_TYPE_ERR				0xfe    // Error rate
#define IBUS_MEAS_TYPE_UNKNOWN			0xff

#ifdef IA6B_PATCHED
	#define IBUS_MEAS_TYPE_GPS_FULL			0xfd
	#define IBUS_MEAS_TYPE_VOLT_FULL		0xf0
	#define IBUS_MEAS_TYPE_ACC_FULL			0xef
#endif


#define IBUS_2BYTE_SESNSOR 2
#define IBUS_4BYTE_SESNSOR 4

// macros to code the different measurement types
#define TEM_1(x) ((400+(x))&0xFF)
#define TEM_2(x) ((400+(x))>>8)
#define LBYTE(x) ((x)&0xFF)
#define HBYTE(x) ((x)>>8&0xFF)

#define IBUS_BUFFSIZE 33				// biggest iBus message seen so far + 1
#define IBUS_HEADER_FOOTER_SIZE 4

typedef struct IBUS_FULL_GPS {
	uint8_t	status;
	uint8_t	sats;
	unsigned long lat;
	unsigned long lon;
	unsigned long alt;
} IBUS_FULL_GPS;

typedef struct IBUS_FULL_VOLT {
	uint16_t extV;
	uint16_t cell;
	uint16_t current;
	uint16_t fuel;
	uint16_t rpm;
} IBUS_FULL_VOLT;

typedef struct IBUS_FULL_ACC {
	uint16_t accX;
	uint16_t accY;
	uint16_t accZ;
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
} IBUS_FULL_ACC;


typedef struct IBUS_SENSOR_DEF {
	uint8_t type;
	uint8_t payloadSize;
} IBUS_SENSOR_DEF;

const IBUS_SENSOR_DEF FULL_GPS_IDS[] = {
	{ .type = IBUS_MEAS_TYPE_GPS_STATUS,	.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_GPS_LAT,		.payloadSize = IBUS_4BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_GPS_LON,		.payloadSize = IBUS_4BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_GPS_ALT,		.payloadSize = IBUS_4BYTE_SESNSOR },
};

const IBUS_SENSOR_DEF FULL_VOLT_IDS[] = {
	{ .type = IBUS_MEAS_TYPE_EXTV,			.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_CELL,			.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_BAT_CURR,		.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_FUEL,			.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_RPM,			.payloadSize = IBUS_2BYTE_SESNSOR },
};

const IBUS_SENSOR_DEF FULL_ACC_IDS[] = {
	{ .type = IBUS_MEAS_TYPE_ACC_X,	.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_ACC_Y,	.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_ACC_Z,	.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_ROLL,	.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_PITCH,	.payloadSize = IBUS_2BYTE_SESNSOR },
	{ .type = IBUS_MEAS_TYPE_YAW,	.payloadSize = IBUS_2BYTE_SESNSOR },
};

typedef struct IBUS_SENSOR {
	uint8_t type;
	uint8_t payloadSize;
	uint8_t address;
} IBUS_SENSOR;

//serial specific method must be defined
void usart_send(uint8_t* bytes, uint8_t len);

uint8_t ibusRXBuff[IBUS_BUFFSIZE];	// IBUS frame receive buffer
uint8_t ibusTXBuff[IBUS_BUFFSIZE];		// IBUS frame transmit buffer


uint8_t volatile ibusFrameCount;		// Count of bytes received for current IBUS frame
uint8_t volatile ibusGapDetect;			// used to detect time between frames
										// set to 1 every-time a byte is received
										// set to 0 every timer 0
										// if it is still 0 after a timer 0 period then

//never try define more than F sensors
IBUS_SENSOR sensorsList[] = {
#ifdef IA6B_PATCHED
	{ .type =	IBUS_MEAS_TYPE_GPS_FULL,		.payloadSize = sizeof(IBUS_FULL_GPS),	.address = 0 },
	{ .type =	IBUS_MEAS_TYPE_VOLT_FULL,		.payloadSize = sizeof(IBUS_FULL_VOLT),	.address = 0 },
	{ .type =	IBUS_MEAS_TYPE_ACC_FULL,		.payloadSize = sizeof(IBUS_FULL_ACC),	.address = 0 },
	{ .type =	IBUS_MEAS_TYPE_ARMED,			.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type =	IBUS_MEAS_TYPE_PRES,			.payloadSize = IBUS_4BYTE_SESNSOR,		.address = 0 },
	{ .type =	IBUS_MEAS_TYPE_ALT,				.payloadSize = IBUS_4BYTE_SESNSOR,		.address = 0 },
#else
	//GPS
	{ .type = IBUS_MEAS_TYPE_GPS_STATUS,		.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_GPS_LAT,			.payloadSize = IBUS_4BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_GPS_LON,			.payloadSize = IBUS_4BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_GPS_ALT,			.payloadSize = IBUS_4BYTE_SESNSOR,		.address = 0 },
	//BATTERY
	{ .type = IBUS_MEAS_TYPE_EXTV,				.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_BAT_CURR,			.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_FUEL,				.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
#endif

	{ .type = IBUS_MEAS_TYPE_TEM,				.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_FLIGHT_MODE,		.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_GPS_DIST,			.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_CMP_HEAD,			.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_CLIMB_RATE,		.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_COG,				.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_VERTICAL_SPEED,	.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },
	{ .type = IBUS_MEAS_TYPE_GROUND_SPEED,		.payloadSize = IBUS_2BYTE_SESNSOR,		.address = 0 },

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
//telemetry specific value implemented in MAVLINK - because mavlink knows data
void setTelemetryValueToBuffer(uint8_t* buffer, uint8_t sensorType, uint8_t length);

//Method fill provided buffer with sensor data
//used as response for command IBUS_CMD_TEL_MEAS
//buffer - pointer to packet buffer
//index - sensor index used to resolve type and data length
void getSensorData(uint8_t* buffer, uint8_t index) {
	buffer[0] = IBUS_HEADER_FOOTER_SIZE + sensorsList[index].payloadSize;
	buffer[1] = IBUS_CMD_TEL_MEAS + sensorsList[index].address;
	setTelemetryValueToBuffer(buffer + 2, sensorsList[index].type, sensorsList[index].payloadSize);
	setChecksum(buffer);
}

//RX byte from IBUS port return true when packet is completed
bool knownCommand(uint8_t command) {
	switch (command) {
	case IBUS_CMD_TEL_DISC:
	case IBUS_CMD_TEL_TYPE:
	case IBUS_CMD_TEL_MEAS:
  case IBUS_CMD_CHANNEL:
		return true;
	}
	return false;
}

bool ibusRXByte(uint8_t value) {
	ibusRXBuff[ibusFrameCount] = value;
	ibusGapDetect = 1;

  if (ibusFrameCount < IBUS_BUFFSIZE - 1) ibusFrameCount++;
  bool validFrame = true;
  if(ibusFrameCount > 0){
    if(ibusRXBuff[0] > IBUS_BUFFSIZE - 1) validFrame = false;
    if(ibusRXBuff[0] < 4) validFrame = false;
    if(ibusRXBuff[0] % 2 == 1)validFrame = false;
  }
  if(ibusFrameCount > 1 && !knownCommand(ibusRXBuff[1] & 0xF0)) validFrame = false;
  
	if (!validFrame)
	{
		ibusFrameCount = 0;
	}
	// Full frame check it is good
	else if (ibusFrameCount > 3 && ibusFrameCount >= ibusRXBuff[0])
	{
		// reset the frame regardless if it is a good one or it failed the checksum
		ibusFrameCount = 0;
		uint8_t length = ibusRXBuff[0];
    bool checkSum = getChecksum(ibusRXBuff) == (uint16_t)(((ibusRXBuff[length - 1] << 8) + ibusRXBuff[length - 2]));
    
    if(ibusRXBuff[1]==IBUS_CMD_CHANNEL){ 
      pinMode(LED_BUILTIN_RX, OUTPUT);
      digitalWrite(LED_BUILTIN_RX, LOW);
      updateChannelData(ibusRXBuff, checkSum);
      return false; 
    }
		return checkSum;
	}
	return false;
}


void ibusTX() {
  ibusTXBuff[0] = 0;
	switch (ibusRXBuff[1] & 0xF0)
	{
		case IBUS_CMD_TEL_DISC:
			for (uint8_t i = 0; i < sensors; i++) {
			// We haven't responded yet or we are being asked again
			if (sensorsList[i].address == 0 || sensorsList[i].address == (ibusRXBuff[1] & 0x0F))
			{
				//We haven't responded yet or we are being asked again
				sensorsList[i].address = ibusRXBuff[1] & 0x0F;
				memcpy(ibusTXBuff, ibusRXBuff, ibusRXBuff[0]);;
				break;
			}
			}
		break;
		case IBUS_CMD_TEL_TYPE:
			for (uint8_t i = 0; i < sensors; i++) {
				if (sensorsList[i].address == (ibusRXBuff[1] & 0x0F))
				{
				getSensorType(ibusTXBuff, i);
				break;
				}
			}
		break;
		case IBUS_CMD_TEL_MEAS:
			for (uint8_t i = 0; i < sensors; i++) {
				if (sensorsList[i].address == (ibusRXBuff[1] & 0x0F))
				{
					getSensorData(ibusTXBuff, i);
					break;
				}
			}
		break;
	}
  if(ibusTXBuff[0] != 0){ 
    usart_send(ibusTXBuff, ibusTXBuff[0]);
  }
}


#endif // IBUS_TELEMETRY
