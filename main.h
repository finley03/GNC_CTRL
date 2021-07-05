#include "util.h"
#include "spi_flash.h"
#include "spi_eeprom.h"
#include "uart.h"
#include "spi.h"

#define NAV_DEVICE_ID 0xd5d5
#define DEVICE_ID 0xd6d6


// nav_data_packet
// packet received from navigation computer
typedef struct __attribute__((aligned(4))) { // aligned is for CRC calculation
	uint16_t device_id;
	
	// kalman filter output
	
	float position_x;
	float position_y;
	float position_z;
	
	float velocity_x;
	float velocity_y;
	float velocity_z;
	
	float accel_x;
	float accel_y;
	float accel_z;
	
	float accelraw_x;
	float accelraw_y;
	float accelraw_z;
	
	float orientation_x;
	float orientation_y;
	float orientation_z;
	
	float accelmagorientation_x;
	float accelmagorientation_y;
	float accelmagorientation_z;
	
	float angularvelocity_x;
	float angularvelocity_y;
	float angularvelocity_z;
	
	float mag_x;
	float mag_y;
	float mag_z;
	
	// gps output
	
	float latitude;
	float longitude;
	float gps_height;
	float h_acc; // horizontal accuracy
	float v_acc; // vertical accuracy
	uint16_t gps_satellites; // number of satellites
	uint32_t gps_watchdog; // time since last data in ms
	
	// raw data output
	
	float pressure; // pa
	float imu_temperature;
	float baro_temperature;
	
	float debug1;
	float debug2;
	
	
	uint32_t crc;
} Nav_Data_Packet_Type;


typedef union __attribute__((aligned(4))) { // union to ease accessing of data
	Nav_Data_Packet_Type bit;
	
	uint8_t reg[sizeof(Nav_Data_Packet_Type)];
} NAV_Data_Packet;


// nav_selftest_packet
// packet nagivation computer returns after selftest command
typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	uint8_t device_code;
	uint8_t imu_code;
	uint8_t mag_code;
	uint8_t baro_code;
	
	uint32_t crc;
} NAV_Selftest_Packet_Type;


typedef union __attribute__((aligned(4))) {
	NAV_Selftest_Packet_Type bit;
	
	uint8_t reg[sizeof(NAV_Selftest_Packet_Type)];
} NAV_Selftest_Packet;


#define CTRL_ACK_OK 0x0000
#define CTRL_ACK_ERROR 0xffff

// packet returned by CTRL computer after packet
// send request from host computer
typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	uint16_t status_code;
	
	uint32_t crc;
} CTRL_ACK_Packet_Type;


typedef union __attribute__((aligned(4))) {
	CTRL_ACK_Packet_Type bit;
	
	uint8_t reg[sizeof(CTRL_ACK_Packet_Type)];
} CTRL_ACK_Packet;