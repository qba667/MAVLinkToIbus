.align  2
.global main
.thumb
main:
BL				getSensorName
BL				ms
BL				call1
BL				call2
BL				addedSensor
BX 				lr
.org	0x19F8
sprintf:
PUSH    {LR}
POP    {PC}
.org	0x1E7A
div:
PUSH    {LR}
POP    {PC}
.org	0x2fa0
BL		getSensorName
.org	0x67ac
BL		getSensorName
.org	0xF020
getSensorName:
.balign 4
PUSH {LR}
CMP		R0, #0
BLT		sensorUnknown
CMP		R0, #255
BGT		sensorUnknown
CMP		R0, #65
BNE		not65
MOVS	R0, #0xf
not65:
CMP		R0, #0xF
BLE		shift3AndSet
CMP		R0, #0x7C
BLT		sensorUnknown
CMP		R0, #0x7F
BGT		greaterThan7F
B		sensor7C
greaterThan7F:
CMP		R0, #0x89
BLE		sensor80
CMP		R0, #0xf9
BGE		sensorF9
B		sensorUnknown
sensor7C:
SUB		R0, #0x7C
ADD		R1, #0x80
B		shift3AndSet
sensor80:
SUB		R0, #0x80
ADD		R1, #0xA0
B		shift2AndSet
sensorF9:
SUB		R0, #0xF9
ADD		R1, #0xC8
B		shift3AndSet
shift2AndSet:
LSL		R0, R0, #2
ADD		R0, R1, R0
B		returnSensorName
shift3AndSet:
LSL		R0, R0, #3
ADD		R0, R1, R0
B		returnSensorName
sensorUnknown:
MOV		R0, #0xE0
ADD		R0, R1, R0
returnSensorName:
POP {PC}
.org	0xFB5C
PUSH    {R1,LR}
LDR     R1, =notImplemented
BL      getSensorName
POP     {R1,PC}
.org	0xFB6C
formatSensor:
.align  2
PUSH    {r4-r7, LR}
MOV		R7, R0
MOV		R6, R1
MOV		R5, R2
MOV		R3, #0
CMP		R6, #0x4
BEQ		angle0to360
CMP		R6, #0x5
BEQ		signedBivBy100
CMP		R6, #0x6
BEQ		speedMS
CMP		R6, #0x7
BEQ		signedBivBy100
CMP		R6, #0x8
BEQ		signedBivBy100
CMP		R6, #0x9
BEQ		signedBivBy100
CMP		R6, #0xA
BEQ		speedMS
CMP		R6, #0xB
BEQ		speedMS
CMP		R6, #0xC
BEQ		distance
CMP		R6, #0xD
BEQ		armedStatus
CMP		R6, #0xE
BEQ		mode
CMP		R6, #0xF
BEQ		defSensor
CMP		R6, #0x80 
BEQ		angle0to360
CMP		R6, #0x81 
BEQ		gpsStatus
CMP		R6, #0x84
BEQ		distanceSigned
CMP		R6, #0xF9
BEQ		distanceSigned
CMP		R6, #0xFD
BEQ		gpsLongFrame
CMP		R6, #0xFA
BEQ		SNRSensor
BGT		signalSensor
B		defSensor
POP		{r4-r7,PC}
gpsLongFrame:
LDR		R1, =notImplemented
B		callSprintf
gpsStatus:
LDR		R1, =gpsStatusStr
MOV		R4, #0xFF
MOV		R3, R2
LSR 	R3, R3, #8
AND		R3, R3, R4
AND		R2, R2, R4
B		callSprintf
angle0to360:
LDR		R1, =genericDecimal
B		callSprintf
speedMS:
LDR     R4, =speed
B		checkSign
signedBivBy100:
LDR     R4, =genericDecimal
B		checkSign
distanceSigned:
LDR     R4, =distanceFormat
B		checkSign
armedStatus:
LDR		R1, =unarmed
CMP		R2, #0
BEQ		callSprintf
LDR		R1, =armed
B		callSprintf
mode:
LDR		R1, =unknownMode
CMP		R2, #9
BGT		callSprintf
LDR		R1, =stabilize
LSL		R2, R2, #3
ADD		R1, R2, R1
BGT		callSprintf
checkSign:
SXTH    R0, R2
CMP     R0, #0
BGE     formatDecimal
NEG	    R0, R0
SXTH    R0, R0
MOV		R1, #0x2D
STRB	R1, [R7]
ADD		R7, #0x1
B		formatDecimal
formatDecimal:
MOVS    R1, #100
BL      div
MOV     R3, R1
MOV     R2, R0
MOV     R1, R4
MOV     R0, R7
B      	callSprintf
distance:
LDR		R1, =distanceFormat
B		callSprintf
SNRSensor:
LDR		R1, =dBsensor
B		callSprintf
signalSensor:
LDR		R1, =dBmSensor
B		callSprintf
defSensor:
LDR		R1, =defaultSensor
B		callSprintf
callSprintf:
BL		sprintf
POP		{r4-r7,PC}
notImplemented: .asciz "NOT IMPL"
gpsStatusStr: .asciz "%u %02u"
genericDecimal: .asciz "%u.%02u"
defaultSensor: .asciz "%u"
distanceFormat:  .asciz "%um"
dBsensor:  .asciz "%udB"
dBmSensor: .asciz "-%udBm"
speed: .asciz "%u.%02u m/s"
gps: .asciz "%u %02u'%02u"
armed: .asciz "Armed\0\0"
unarmed: .asciz "Unarmed"
unknownMode: .asciz "Unknown"
stabilize: .asciz "Stab\0\0\0"
acro: .asciz "Acro\0\0\0"
AHold: .asciz "AHold\0\0"
Auto: .asciz "Auto\0\0\0"
Guided: .asciz "Guided\0"
Loiter: .asciz "Loiter\0"
RTL: .asciz "RTL\0\0\0\0"
Circle: .asciz "Circle\0"
PosHold: .asciz "PosHold"
Land: .asciz "Land\0\0\0"

