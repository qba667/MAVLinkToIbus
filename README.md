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
After downloading of AltSoftSerial lib configuration for ARDUINO_AVR_YUN, ARDUINO_AVR_LEONARDO and __AVR_ATmega32U4__ (in file AltSoftSerial_Boards.h) must be replaced.

Oryginal definition
```c
#elif defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega32U4__)

  //#define ALTSS_USE_TIMER1
  //#define INPUT_CAPTURE_PIN		4  // receive
  //#define OUTPUT_COMPARE_A_PIN	9 // transmit
  //#define OUTPUT_COMPARE_B_PIN	10 // unusable PWM
  //#define OUTPUT_COMPARE_C_PIN	11 // unusable PWM

  #define ALTSS_USE_TIMER3
  #define INPUT_CAPTURE_PIN		13 // receive
  #define OUTPUT_COMPARE_A_PIN		5 // transmit
```
Working definition:
```c
#elif defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega32U4__)

#define ALTSS_USE_TIMER1
#define INPUT_CAPTURE_PIN	  4 // receive
#define OUTPUT_COMPARE_A_PIN  9 // transmit
```
As input capture pin 4 must be used and as output compare pin 9 must be used.

In file IBUSTelemetry.h sensors can be configured. It is possible to define up to 0xF sensors. 
Always use latest firmware for I6 from:
https://www.rcgroups.com/forums/showthread.php?2486545-FlySky-FS-i6-8-channels-firmware-patch%21/page129

