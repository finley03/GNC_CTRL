#ifndef MAIN_H
#define MAIN_H

#include "util.h"

//typedef enum {
	//FLIGHT_MODE_MANUAL,
	//FLIGHT_MODE_MANUAL_HEADING_HOLD,
	//FLIGHT_MODE_AUTO_WAYPOINT
//} Flight_Mode;

typedef enum {
	FLIGHT_MODE_MANUAL, // manual override on all control surfaces
	FLIGHT_MODE_FBW, // fly by wire with manual pitch and bank
	FLIGHT_MODE_FBWH, // fly by wire with automatic pitch and bank, manual heading and altitude
	FLIGHT_MODE_AUTO, // follow mission
	FLIGHT_MODE_LOITER, // loiters point where mode was enabled
	FLIGHT_MODE_RTL, // returns to the launch position
	FLIGHT_MODE_LAUNCH
} Flight_Mode;

#endif