# MAVLinkToIbus
Emulation of IBUS sensors with use of MAVLink telemetry data.

Atmega32u4 – Arduino Pro Micro MAVLink to IBUS 
Pinout of Pro Micro:

PIN 4 – AltSoftSerial RX – Flight Controller TX

PIN 9 – AltSoftSerial TX – Flight Controller RX

TX0 via 4k7 to RX1

RX1 – IA6B Sensor port - pin S

RAW – IA6B Sensor port – pin V+ 

GND – IA6B Sensor port – pin G and and Flight Controller GND

For MAVLink communication AltSoftSerial has been used. It will work also with use of hardware serial but not on ProMicro becuase 2nd serial is attached to USB. Do not use SoftSerial or NewSoftSerial - they are too slow.
After downloading of AltSoftSerial lib configuration for ARDUINO_AVR_YUN, ARDUINO_AVR_LEONARDO and __AVR_ATmega32U4__ must be replaced with:

//#elif defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega32U4__)

//	#define ALTSS_USE_TIMER1

//	#define INPUT_CAPTURE_PIN	  4 // receive

//	#define OUTPUT_COMPARE_A_PIN  9 // transmit

Input capture pin is 4 and output compare pin is 9. 

In file IBUSTelemetry.h sensors can be configured. You can define up to 0xF sensors.



