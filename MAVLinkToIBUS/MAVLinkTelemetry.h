
//Ghettostation / GhettoProxy / Mavlink.cpp
//gpsParsing 
//https://github.com/kevinkessler/TelemetryBridge/blob/master/main.c


#pragma once
#ifndef DEBUG
//#define DEBUG
#endif

#ifndef MAVLink_TELEMETRY
#include "IBUSTelemetry.h"
#include "TelemetryUtils.h"
#include "mavlink_types.h"
#include "math.h"
//WARNING!!!
//To use AltSoftSerial on Micro with USB definition must be changed for 
//#elif defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega32U4__)
//	#define ALTSS_USE_TIMER1
//	#define INPUT_CAPTURE_PIN	  4 // receive
//	#define OUTPUT_COMPARE_A_PIN  9 // transmit

#include <AltSoftSerial.h>
AltSoftSerial altSerial;
mavlink_system_t mavlink_system;
#ifndef YOUR_MAVLINK_BRIDGE_HEADER_H
#define YOUR_MAVLINK_BRIDGE_HEADER_H
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
	altSerial.write(ch);
}
#endif
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "common\mavlink.h"

void mavlinkTelemetryInit() {
	altSerial.begin(57600);
	mavlink_system.sysid = 100; // System ID, 1-255
	mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
}

uint16_t streamMessages[] = {
	MAVLINK_MSG_ID_SYS_STATUS,
	MAVLINK_MSG_ID_GPS_RAW_INT,
	MAVLINK_MSG_ID_ATTITUDE,
	MAVLINK_MSG_ID_SCALED_PRESSURE,
	MAVLINK_MSG_ID_VFR_HUD,
	MAVLINK_MSG_ID_SCALED_IMU,
	MAVLINK_MSG_ID_BATTERY_STATUS,
  MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
#ifdef DEBUG
	
	MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
#endif
};

uint32_t heartBeat = 0;
uint8_t messageRecieved = 0;

//MAVLINK_MSG_ID_HEARTBEAT
uint32_t flightMode = 0;
uint32_t baseMode = 0;
uint8_t mavType = 0;
uint8_t motorArmed = 0;

//MAVLINK_MSG_ID_SYS_STATUS
uint16_t voltage;
int16_t currA;
int8_t batteryRemaining;

//MAVLINK_MSG_ID_GPS_RAW_INT
int32_t lat;
int32_t lon;
int32_t altGPS;
uint8_t fixType;
uint8_t satellitesVisible;
uint16_t cog;

//MAVLINK_MSG_ID_ATTITUDE
int16_t pitch;
int16_t roll;
int16_t yaw;

//MAVLINK_MSG_SCALED_IMU
int16_t accx;
int16_t accy;
int16_t accz;
//voltage
uint16_t voltages[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN];


//MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT
//float nav_roll;
//float nav_pitch;
//float nav_bearing;

//MAVLINK_MSG_ID_SCALED_PRESSURE
int32_t pressure;
int16_t temperature;

//MAVLINK_MSG_ID_VFR_HUD
int16_t airspeed;
int16_t groundspeed;
int16_t heading; // 0..360 deg, 0=north
uint16_t throttle;
int32_t alt;
int16_t climb;
int16_t dist;

mavlink_message_t msg;
mavlink_status_t status;
uint8_t GPS_set;
int32_t firstLat;
int32_t firstLon;
int32_t firstAltGps;

double meanRadius = 6371008.0;
uint32_t equatorialLen = 111196672;
uint32_t currentLen = 0;
uint32_t singleUnitInMmForLon = divideBy1E7(equatorialLen);
uint32_t singleUnitInMmForLat = 0;


//using trigonometric functions good for bigger differences
/*
uint16_t getDistnace(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
	if (lat == lat2 && lon == lon2) return 0.0;	
	double latD = (double)ToRadFromWGS84(lat1);
	double lonD = (double)ToRadFromWGS84(lon1);

	double latD2 = (double)ToRadFromWGS84(lat2);
	double lonD2 = (double)ToRadFromWGS84(lon2);


	double acosArg = sin(latD)*sin(latD2) + cos(latD) *cos(latD2) * cos(lonD - lonD2);
	double dist = 0.0;
	if (tmp < 1) dist = meanRadius * acos(acosArg);
	return (uint16_t)dist;
}*/


uint16_t getDistnace(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {

	if (currentLen == 0 || singleUnitInMmForLat==0) {
		currentLen = (uint32_t)ceil(cos((double)((lat1 + lat2) / 2) * 0.000000001745329252) * equatorialLen);
		singleUnitInMmForLat = divideBy1E7(currentLen);
	}

	uint32_t diffLat = 0;
	if (lat2 > lat1) diffLat = lat2 - lat1;
	else diffLat = lat1 - lat2;

	uint32_t diffLon = 0;
	if (lon2 > lon1) diffLon = lon2 - lon1;
	else diffLon = lon1 - lon2;

	diffLon *= singleUnitInMmForLon;
	diffLat *= singleUnitInMmForLat;
	//convert to m
	diffLon = diffLon / 1000;
	diffLat = diffLat / 1000;
	//sqrt(a^2 +b^2)
	return (uint16_t)ceil(sqrt(diffLon * diffLon + diffLat * diffLat));
}


#ifdef DEBUG
unsigned long timerStartMavlink = 0;
unsigned long timeDiffMavlink = 0;
unsigned long maxDiff = 0;
#endif

#define MAP_FLIGHT_MODE


#ifdef MAP_FLIGHT_MODE
#define MAP_FLIGHT_MODE_STABILIZE 0                     // hold level position
#define MAP_FLIGHT_MODE_ACRO 1                          // rate control
#define MAP_FLIGHT_MODE_ALT_HOLD 2                      // Altitude Hold
#define MAP_FLIGHT_MODE_AUTO 3                          // Waypoint
#define MAP_FLIGHT_MODE_GUIDED 4                        // Hold a single location from command
#define MAP_FLIGHT_MODE_LOITER 5                        // Hold a single location
#define MAP_FLIGHT_MODE_RTL 6                           // return to launch
#define MAP_FLIGHT_MODE_CIRCLE 7                        // orbit around a single location
#define MAP_FLIGHT_MODE_POSHOLD_IBUS	8               // position hold with manual override
#define MAP_FLIGHT_MODE_LAND 9                          // Landing
#define MAP_FLIGHT_MODE_OF_LOITER 10                    // Hold a single location using optical flow sensor
#define MAP_FLIGHT_MODE_DRIFT 11                        // DRIFT mode (Note: 12 is no longer used)
#define MAP_FLIGHT_MODE_SPORT 13                        // earth frame rate control
#define MAP_FLIGHT_MODE_FLIP        14                  // flip the vehicle on the roll axis
#define MAP_FLIGHT_MODE_AUTOTUNE    15                  // autotune the vehicle's roll and pitch gains
#define MAP_FLIGHT_MODE_POSHOLD     16                  // position hold with manual override
#endif

void PrintFlightModeArm() {
#ifdef DEBUG
	Serial.print("flightMode: ");
	Serial.println(flightMode);
	Serial.print("motorArmed: ");
	Serial.println(motorArmed);
#endif
}
void PrintBattery() {
#ifdef DEBUG
	Serial.print("V: ");
	Serial.println(voltage);
	Serial.print("A: ");
	Serial.println(currA);
	Serial.print("bat: ");
	Serial.println(batteryRemaining);
#endif // DEBUG
}
void PrintGPS() {
#ifdef DEBUG
	uint8_t minus;
	uint8_t deg;
	uint8_t min;
	uint8_t sec;
	uint16_t subSec; //up to 999

	minus = parseCoord(&deg, &min, &sec, &subSec, lat);

	Serial.print(deg);
	Serial.print(' ');
	Serial.print(min);
	Serial.print('\'');
	Serial.print(sec);
	Serial.print('.');
	Serial.print(subSec);
	Serial.println('"');


	minus = parseCoord(&deg, &min, &sec, &subSec, lon);

	Serial.print(deg);
	Serial.print(' ');
	Serial.print(min);
	Serial.print('\'');
	Serial.print(sec);
	Serial.print('.');
	Serial.print(subSec);
	Serial.println('"');


	Serial.print("lat: ");
	Serial.println(lat);
	Serial.print("lon: ");
	Serial.println(lon);
	Serial.print("dist: ");
	Serial.println(dist);
	Serial.print("fixType: ");
	Serial.println(fixType);
	Serial.print("satellitesVisible: ");
	Serial.println(satellitesVisible);
	Serial.print("cog: ");
	Serial.println(cog);
	Serial.print("altGPS: ");
	Serial.println(altGPS);

#endif
}

void PrintPos() {
#ifdef DEBUG
	Serial.print("pitch:");
	Serial.println(pitch);
	Serial.print("roll:");
	Serial.println(roll);
	Serial.print("yaw:");
	Serial.println(yaw);
#endif
}
void PrintACC() {
#ifdef DEBUG
	Serial.print("accx:");
	Serial.println(accx);
	Serial.print("accy:");
	Serial.println(accy);
	Serial.print("accz:");
	Serial.println(accz);
#endif
}
void PrintPressure() {
#ifdef DEBUG
	Serial.print("pressure:");
	Serial.println(pressure);
	Serial.print("temperature:");
	Serial.println(temperature);
#endif
}
void PrintHUD(){
#ifdef DEBUG
	Serial.print("airspeed:");
	Serial.println(airspeed);
	Serial.print("groundspeed:");
	Serial.println(groundspeed);
	Serial.print("heading:");
	Serial.println(heading);
	Serial.print("alt:");
	Serial.println(alt);
	Serial.print("climb:");
	Serial.println(climb);
#endif
}

void handeMavlink() {
	if (!altSerial.available()) return;
#ifdef DEBUG
	timerStartMavlink = micros();
#endif
	if (mavlink_parse_char(MAVLINK_COMM_0, altSerial.read(), &msg, &status))
	{
		
		if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
			messageRecieved = 1;
		}
		
		switch (msg.msgid)
		{
		case MAVLINK_MSG_ID_HEARTBEAT:
#ifdef DEBUG
			Serial.print("MAVLINK_MSG_ID_HEARTBEAT: ");
#endif
			if (++heartBeat > 3 && !messageRecieved) {
				//0 as system id id broadcast
				for (int i = 0; i < 0; i++) {
					mavlink_msg_message_interval_send(MAVLINK_COMM_0, (uint16_t)streamMessages[i], 200000);
				}
				heartBeat = 0;
			};
			flightMode = mavlink_msg_heartbeat_get_custom_mode(&msg);
			if (flightMode == MAP_FLIGHT_MODE_POSHOLD) flightMode = MAP_FLIGHT_MODE_POSHOLD_IBUS;

			mavType = mavlink_msg_heartbeat_get_type(&msg);
			baseMode = mavlink_msg_heartbeat_get_base_mode(&msg);
			motorArmed = baseMode & MAV_MODE_FLAG_SAFETY_ARMED;
			if (!motorArmed && GPS_set) { //reset start position
				GPS_set = 0;
				firstAltGps = 0;
				firstLat = 0;
				firstLon = 0;
			}
			PrintFlightModeArm();
			break;
		case MAVLINK_MSG_ID_BATTERY_STATUS:
			//Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
			mavlink_msg_battery_status_get_voltages(&msg, (uint16_t *)voltages);
			//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
			mavlink_msg_battery_status_get_current_battery(&msg);
			//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
			mavlink_msg_battery_status_get_current_consumed(&msg);
			//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
			mavlink_msg_battery_status_get_battery_remaining(&msg);
			PrintBattery();
			break;
		case MAVLINK_MSG_ID_SYS_STATUS:
			voltage = divideBy10(mavlink_msg_sys_status_get_voltage_battery(&msg)); //in millivolts radio needs in 10*milliamperes
			currA = mavlink_msg_sys_status_get_current_battery(&msg); //current, in 10*milliamperes (1 = 10 milliampere)
			batteryRemaining = mavlink_msg_sys_status_get_battery_remaining(&msg); //(0%: 0, 100%: 100)
			break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      //heading = mavlink_msg_global_position_int_get_hdg(&msg);
      alt = divideInt32By10(mavlink_msg_global_position_int_get_relative_alt(&msg));
      //latitude = mavlink_msg_global_position_int_get_lat(&msg);
      //longitude = mavlink_msg_global_position_int_get_lon(&msg);
      //Serial.print("alt _GP:");
      //Serial.println(alt);
      break;
		case MAVLINK_MSG_ID_GPS_RAW_INT:
			lat = mavlink_msg_gps_raw_int_get_lat(&msg);	 // Latitude(WGS84), in degrees * 1E7
			lon = mavlink_msg_gps_raw_int_get_lon(&msg); //Longitude  (WGS84), in degrees * 1E7
     
			altGPS = mavlink_msg_gps_raw_int_get_alt(&msg)/10;
			//divideInt32By10(mavlink_msg_gps_raw_int_get_alt(&msg)); //in m*1000 div by 100 so in m 

			fixType = mavlink_msg_gps_raw_int_get_fix_type(&msg);
			satellitesVisible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
			cog = mavlink_msg_gps_raw_int_get_cog(&msg); //cog Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees.
			
			if (motorArmed && !GPS_set && fixType > GPS_FIX_TYPE_NO_FIX && satellitesVisible >=6) {
				GPS_set = 1;
				firstAltGps = altGPS;
				firstLat = lat;
				firstLon = lon;
			}
			if(GPS_set && motorArmed) dist = getDistnace(lat, lon, firstLat, firstLon);
			else dist = 0;
			PrintGPS();
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			pitch = (int16_t)(ToDeg(mavlink_msg_attitude_get_pitch(&msg))*100.0);
			roll =	(int16_t)(ToDeg(mavlink_msg_attitude_get_roll(&msg))*100.0);
			yaw = (int16_t)(ToDeg(mavlink_msg_attitude_get_yaw(&msg))*100.0);
			PrintPos();
			break;
#ifdef DEBUG
		case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
			//nav_roll = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
			//nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
			//nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
			//wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
			//wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
			//alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
			//aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
			//xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);
			break;
#endif
		case MAVLINK_MSG_ID_SCALED_IMU:
			accx = (uint16_t)((double)mavlink_msg_scaled_imu_get_xacc(&msg) * 98.0665);
			accy = (uint16_t)((double)mavlink_msg_scaled_imu_get_yacc(&msg) * 98.0665);
			accz = (uint16_t)((double)mavlink_msg_scaled_imu_get_zacc(&msg) * 98.0665);
			PrintACC();
			break;
		case MAVLINK_MSG_ID_SCALED_PRESSURE:
			pressure = (int32_t)(round(mavlink_msg_scaled_pressure_get_press_abs(&msg) *100)); //hectopascal 
			temperature = divideBy10(mavlink_msg_scaled_pressure_get_temperature(&msg)); // Temperature measurement (0.01 degrees celsius)
			PrintPressure();
			break;
		case MAVLINK_MSG_ID_VFR_HUD:
			airspeed = (uint16_t)(round(mavlink_msg_vfr_hud_get_airspeed(&msg)*100.0)); //m/s  *100
			groundspeed = (uint16_t)(round(mavlink_msg_vfr_hud_get_groundspeed(&msg)*100.0)); //m/s *100
			heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
			throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
			//consider using
			//mavlink_msg_global_position_int_get_relative_alt
			//alt = (int32_t)(round(mavlink_msg_vfr_hud_get_alt(&msg)*100.0)); //m *100
			climb = (int16_t)(round(mavlink_msg_vfr_hud_get_climb(&msg)*100.0)); // m/s *100
			PrintHUD();
		}
#ifdef DEBUG
		timeDiffMavlink = micros() - timerStartMavlink;
		if (timeDiffMavlink > maxDiff) {
			maxDiff = timeDiffMavlink;
			Serial.println(maxDiff);
		}
#endif	
	}

}
#ifdef DEBUG
uint8_t baadFood[] = { 0xBA, 0xAD, 0xF0, 0xF0 };
#endif 

void setInt32Value(uint8_t* buffer, int32_t val) {
 buffer[0] = (uint8_t)val;
 buffer[1] = (uint8_t)(((uint32_t)val >> 8) & 0xFF);
 buffer[2] = (uint8_t)(((uint32_t)val >> 16) & 0xFF);
 buffer[3] = (uint8_t)(((uint32_t)val >> 24) & 0xFF);
}
void setUint32Value(uint8_t* buffer, uint32_t val) {
  /*
	buffer[0] = (val >> 24) & 0xFF;
	buffer[1] = (val >> 16) & 0xFF;
	buffer[2] = HBYTE(val);
	buffer[3] = LBYTE(val);*/
 
  buffer[0] = (uint8_t)val;
  buffer[1] = (uint8_t)(((uint32_t)val >> 8) & 0xFF);
  buffer[2] = (uint8_t)(((uint32_t)val >> 16) & 0xFF);
  buffer[3] = (uint8_t)(((uint32_t)val >> 24) & 0xFF);
}




void setTelemetryValueToBuffer(uint8_t* buffer, uint8_t sensorType, uint8_t length) {
	uint8_t i = 0;
	uint8_t offset = 0;
	for (i = 0; i < length; i++) { 
#ifdef DEBUG
		buffer[i] = baadFood[i % sizeof(baadFood)];
#else
		buffer[i] = 0;
#endif // DEBUG
	}
	switch (sensorType) {
#ifdef IA6B_PATCHED
		case IBUS_MEAS_TYPE_GPS_FULL:
			for (i = 0; i < sizeof(FULL_GPS_IDS)/ sizeof(IBUS_SENSOR_DEF); i++) {
				setTelemetryValueToBuffer(buffer + offset, FULL_GPS_IDS[i].type, FULL_GPS_IDS[i].payloadSize);
				offset += FULL_GPS_IDS[i].payloadSize;
			}
		case IBUS_MEAS_TYPE_VOLT_FULL:
			for (i = 0; i < sizeof(FULL_VOLT_IDS) / sizeof(IBUS_SENSOR_DEF); i++) {
				setTelemetryValueToBuffer(buffer + offset, FULL_VOLT_IDS[i].type, FULL_VOLT_IDS[i].payloadSize);
				offset += FULL_VOLT_IDS[i].payloadSize;
			}
		case IBUS_MEAS_TYPE_ACC_FULL:
			for (i = 0; i < sizeof(FULL_ACC_IDS) / sizeof(IBUS_SENSOR_DEF); i++) {
				setTelemetryValueToBuffer(buffer + offset, FULL_ACC_IDS[i].type, FULL_ACC_IDS[i].payloadSize);
				offset += FULL_ACC_IDS[i].payloadSize;
			}
			break;
#endif
		case IBUS_MEAS_TYPE_TEM:
			buffer[0] = TEM_1(temperature);
			buffer[1] = TEM_2(temperature);
			break;
		case IBUS_MEAS_TYPE_MOT:			
			//RPM?!
			break;
		case IBUS_MEAS_TYPE_EXTV:	
			buffer[0] = LBYTE(voltage);
			buffer[1] = HBYTE(voltage);
			break;
		case IBUS_MEAS_TYPE_CELL:		
			buffer[0] = LBYTE(voltages[0]);
			buffer[1] = HBYTE(voltages[0]);
			break;
		case IBUS_MEAS_TYPE_BAT_CURR:	
			buffer[0] = LBYTE(currA);
			buffer[1] = HBYTE(currA);
			break;
		case IBUS_MEAS_TYPE_FUEL:	
			buffer[0] = LBYTE(batteryRemaining);
			buffer[1] = HBYTE(batteryRemaining);
		case IBUS_MEAS_TYPE_RPM:	
			//if armed : throttle value, battery capacity otherwise. (Blade number needs to be set to 12 in Taranis).
			if (motorArmed) {
				buffer[0] = LBYTE(throttle);
				buffer[1] = HBYTE(throttle);
			}
			break;
		case IBUS_MEAS_TYPE_CMP_HEAD:	
			buffer[0] = LBYTE(heading);
			buffer[1] = HBYTE(heading);
			break;
		case IBUS_MEAS_TYPE_CLIMB_RATE: 
			buffer[0] = LBYTE(climb);
			buffer[1] = HBYTE(climb);
			break;
		case IBUS_MEAS_TYPE_COG:	
			buffer[0] = LBYTE(cog);
			buffer[1] = HBYTE(climb);
			break;
		case IBUS_MEAS_TYPE_GPS_STATUS:	
			buffer[0] = fixType;
			buffer[1] = satellitesVisible;
			break;
		case IBUS_MEAS_TYPE_ACC_X:		
			buffer[0] = LBYTE(accx);
			buffer[1] = HBYTE(accx);
			break;
		case IBUS_MEAS_TYPE_ACC_Y:
			buffer[0] = LBYTE(accy);
			buffer[1] = HBYTE(accy);
			break;
		case IBUS_MEAS_TYPE_ACC_Z:		
			buffer[0] = LBYTE(accz);
			buffer[1] = HBYTE(accz);
			break;
		case IBUS_MEAS_TYPE_ROLL:	
			buffer[0] = LBYTE(roll);
			buffer[1] = HBYTE(roll);
			break;
		case IBUS_MEAS_TYPE_PITCH:	
			buffer[0] = LBYTE(pitch);
			buffer[1] = HBYTE(pitch);
			break;
		case IBUS_MEAS_TYPE_YAW:		
			buffer[0] = LBYTE(yaw);
			buffer[1] = HBYTE(yaw);
			break;
		case IBUS_MEAS_TYPE_VERTICAL_SPEED:	
			buffer[0] = LBYTE(airspeed);
			buffer[1] = HBYTE(airspeed);
			break;
		case IBUS_MEAS_TYPE_GROUND_SPEED:
			buffer[0] = LBYTE(groundspeed);
			buffer[1] = HBYTE(groundspeed);
			break;
		case IBUS_MEAS_TYPE_GPS_DIST:	
			buffer[0] = LBYTE(dist);
			buffer[1] = HBYTE(dist);
			break;
		case IBUS_MEAS_TYPE_ARMED:	
			buffer[0] = motorArmed;
			buffer[1] = 0;
			break;
		case IBUS_MEAS_TYPE_FLIGHT_MODE:	
			buffer[0] = LBYTE(flightMode);
			buffer[1] = HBYTE(flightMode);
			break;
		case IBUS_MEAS_TYPE_PRES:
			setInt32Value(buffer, pressure);
			break;
		case IBUS_MEAS_TYPE_GPS_LAT:
			setInt32Value(buffer, lat);
			break;
		case IBUS_MEAS_TYPE_GPS_LON:
			setInt32Value(buffer, lon);
			break;
		case IBUS_MEAS_TYPE_GPS_ALT:	
			setInt32Value(buffer, altGPS);
			break;
		case IBUS_MEAS_TYPE_ALT:			
			setInt32Value(buffer, alt);
			break;
		default:
			break;
	}
}
#endif
