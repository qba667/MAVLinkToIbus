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
