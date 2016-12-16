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


void timerInit() {
  noInterrupts(); // disable all interrupts
  //1472uS
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  // //1472uS
  //OCR3A = 0x016F;
  //712uS
  OCR3A = 0x00B1;
  TCCR3B |= (1 << WGM32); // CTC mode
  TCCR3B |= (1 << CS31)  | (1 << CS30); // 64 prescale
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  interrupts(); // enable all interrupts
}
void setup() {
  initSerial();
  timerInit();
  mavlinkTelemetryInit();
  Serial.begin(9600);
  Serial.println("Setup done");
}
#define WAITING_FOR_IBUS_FRAME			1
#define PROCESSING_BETWEEN_IBUS_RX_TX	2
#define IBUS_TX							3
unsigned long timerStart = micros();
unsigned long timeDiff = 0;
int8_t volatile state = WAITING_FOR_IBUS_FRAME;


void waitFor(uint16_t value) {
	noInterrupts();
	TCNT3 = 0;
	OCR3A = value; 
	interrupts(); // enable all interrupts
}

void loop() {
  state = 1;
  while (true) {
    switch (state) {
      case WAITING_FOR_IBUS_FRAME:
		if (NextIbusMessageReady==1) {
			NextIbusMessageReady = 0;
			state = PROCESSING_BETWEEN_IBUS_RX_TX;
			//it seems to take up to 850 //should be 712 
			//waitFor(0x00B1);
			//so set it to 0x92 588
			waitFor(0x90);
		}
		handeMavlink();
        break;
      case PROCESSING_BETWEEN_IBUS_RX_TX:
        handeMavlink(); //hope that sine handling is not longer than 712 - longest parsing time so far 350 us
        break;
    }
  }
}


ISR(TIMER3_COMPA_vect) {
	if (state == WAITING_FOR_IBUS_FRAME) {
		if (ibusGapDetect == 0)// restart a frame
		{
			ibusFrameCount = 0;
		}
		ibusGapDetect = 0;
	}
	if (state == PROCESSING_BETWEEN_IBUS_RX_TX) {
		ibusTX();
		state = WAITING_FOR_IBUS_FRAME;
		waitFor(0x016F);
	}


}
