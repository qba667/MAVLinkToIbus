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

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "TelemetryUtils.h"
#include "mavlink_types.h"
#include "IBUSTelemetry.h"
#include "MAVLinkTelemetry.h"
#include "uart.h"
#include "ppm.h"
//TIMERS
//timer0 software Sketch timer functions, like __delay()__, __millis()__ and __micros()__
//timer1 AltSerial 16 bit timer T1
//timer3 time mesurment for telemetry
//timer4 PPM

void timerInit() {
	noInterrupts(); // disable all interrupts
	//1472uS
	TCCR4A = 0; //normal port mode
	TCCR4B = 0;
	TCNT4 = 0;
	// //1472uS
	//OCR3A = 0x016F;
	//712uS
	OCR4A = 0xB1;
	//TCCR4B |= (1 << WGM32); // CTC mode
	TCCR4B |=  (1 << CS42) |(1 << CS41) | (1 << CS40); // 64 prescale
	TIMSK4 |= (1 << OCIE4A); // enable timer compare interrupt
	interrupts(); // enable all interrupts
}
void setup() {
	Serial.begin(115200);
	initSerial();
	timerInit();
	mavlinkTelemetryInit();
  pinMode(LED_BUILTIN_RX, INPUT);
  pinMode(LED_BUILTIN_TX, INPUT);
  setupPpm();
  Serial.println("inited");
}

#define WAITING_FOR_IBUS_DATA			  1
#define TELEMETRY_RESPONSE_DELAY		2
#define IBUS_TX							        3

int8_t volatile state = WAITING_FOR_IBUS_DATA;

void waitFor(uint16_t value) {
	noInterrupts();
	TCNT4 = 0;
  TC4H = (uint8_t)(value >> 8);
	OCR4A = (uint8_t)(value & 0xff);
	interrupts(); // enable all interrupts
}
//RX - channel
//TX - telemetry

void loop() {
	state = 1;
	while (true) {
		switch (state) {
		case WAITING_FOR_IBUS_DATA:
			if (telemetryRequest == 1) {
				telemetryRequest = 0;
				state = TELEMETRY_RESPONSE_DELAY;
				//it seems to take up to 850 //should be 712 
				//waitFor(0x00B1);
				//so set it to 0x92 588
				waitFor(0x90);
			}
			handeMavlink();
			break;
		case TELEMETRY_RESPONSE_DELAY:
			handeMavlink(); //hope that sine handling is not longer than 712 - longest parsing time so far 350 us
			break;
		}
	}
}


ISR(TIMER4_COMPA_vect) {
	if (state == WAITING_FOR_IBUS_DATA) {
    pinMode(LED_BUILTIN_RX, INPUT);
		// restart a frame
		if (ibusGapDetect == 0) ibusFrameCount = 0;
		ibusGapDetect = 0;
    if(++ibusChannelDataTicks > FAIL_SAFE_AFTER_MISSING_FRAMES){
        updateChannelData(0, false);
    }
	}
	if (state == TELEMETRY_RESPONSE_DELAY) {
		ibusTX();
		//state can be changed but data will be recieved
		state = WAITING_FOR_IBUS_DATA;
		waitFor(0x016F);
	}


}
