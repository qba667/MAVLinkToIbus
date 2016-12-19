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
After downloading of AltSoftSerial lib configuration for ARDUINO_AVR_YUN, ARDUINO_AVR_LEONARDO and __AVR_ATmega32U4__ must be replaced with definitions provided in MAVLinkTelemetry.h line 18. As input capture pin 4 must be used and as output compare pin 9 must be used.

In file IBUSTelemetry.h sensors can be configured. It is possible to define up to 0xF sensors. All already defined 2 bytes sensors (everything but not GPS) should works with RX patched using bin file from i6 directory.


