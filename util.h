#ifndef UTIL_H
#define UTIL_H

#include "samd21g18a.h"
#include "time.h"
#include "mat.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#define _STD_G 9.80665

typedef enum {
	_PID_X,
	_PID_Y,
	_PID_Z
} CTRL_Param;

//#define F_CPU 48000000
#define LED PORT_PB11

#define ABS(a) ((a < 0) ? -(a) : a)
#define MAX_2(a, b) ((a > b) ? a : b)
#define MIN_2(a, b) ((b > a) ? a : b);
#define MAX_3(a, b, c) (MAX_2(MAX_2(a, b), c))
#define MIN_3(a, b, c) (MIN_2(MIN_2(a, b), c))
#define UMAX_2(a, b) MAX_2(ABS(a), ABS(b))
#define UMIN_2(a, b) MIN_2(ABS(a), ABS(b))
#define UMAX_3(a, b, c) MAX_3(ABS(a), ABS(b), ABS(c))
#define UMIN_3(a, b, c) MIN_3(ABS(a), ABS(b), ABS(c))

#define pow(a, b) exp2(b * log2(a))
#define powf(a, b) exp2f(b * log2f(a))

#define radians(x) (x * 0.01745329251994329576923690768489)
#define degrees(x) (x * 57.295779513082320876798154814105)

void LED_print_8(uint8_t data);

void crc_init();

#define CRC32_CHECK 0x2144DF1C
uint32_t crc32(uint8_t* data, uint32_t data_size);

void sbinary8(char* buffer, uint8_t value);
void sbinary16(char* buffer, uint16_t value);
void sbinary32(char* buffer, uint32_t value);


void control_load_values();
void control_load_value(CTRL_Param parameter);
void control_save_value(CTRL_Param parameter);
void control_set_value(CTRL_Param parameter, float* value);
void control_read_value(CTRL_Param parameter, float* value);


#define NAV_DEVICE_ID 0xd5d5
#define DEVICE_ID 0xd6d6


typedef enum {
	REQUEST_HEADER_START,
	TRANSFER_REQUEST_HEADER,
	EEPROM_READ_REQUEST_HEADER,
	EEPROM_READ_N_REQUEST_HEADER,
	EEPROM_WRITE_REQUEST_HEADER,
	EEPROM_WRITE_N_REQUEST_HEADER,
	SET_VEC3_REQUEST_HEADER,
	READ_VEC3_REQUEST_HEADER,
	SAVE_VEC3_REQUEST_HEADER,
	SET_SCALAR_REQUEST_HEADER,
	READ_SCALAR_REQUEST_HEADER,
	SAVE_SCALAR_REQUEST_HEADER
} RequestHeader;


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


//#define TRANSFER_REQUEST_HEADER 0x0003

// packet sent by computer to request transfer
typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint16_t command;
	
	uint32_t crc;
} Transfer_Request_Type;


typedef union __attribute__((aligned(4))) {
	Transfer_Request_Type bit;
	
	uint8_t reg[sizeof(Transfer_Request_Type)];
} Transfer_Request;


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


typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	uint8_t status;
	uint8_t data;
	
	uint32_t crc;
} CTRL_EEPROM_Read_Packet_Type;


typedef union __attribute__((aligned(4))) {
	CTRL_EEPROM_Read_Packet_Type bit;
	
	uint8_t reg[sizeof(CTRL_EEPROM_Read_Packet_Type)];
} CTRL_EEPROM_Read_packet;


typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	uint8_t status;
	uint8_t nr_bytes;
	uint8_t data[64];
	
	uint32_t crc;
} CTRL_EEPROM_Read_N_Packet_Type;


typedef union __attribute__((aligned(4))) {
	CTRL_EEPROM_Read_N_Packet_Type bit;
	
	uint8_t reg[sizeof(CTRL_EEPROM_Read_N_Packet_Type)];
} CTRL_EEPROM_Read_N_packet;


//#define EEPROM_READ_REQUEST_HEADER 0x0001

typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint32_t address;
	
	uint32_t crc;
} EEPROM_Read_Request_type;


typedef union __attribute__((aligned(4))) {
	EEPROM_Read_Request_type bit;
	
	uint8_t reg[sizeof(EEPROM_Read_Request_type)];
} EEPROM_Read_Request;


//#define EEPROM_READ_N_REQUEST_HEADER 0x0004

typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint8_t size;
	uint32_t address;
	
	uint32_t crc;
} EEPROM_Read_N_Request_type;


typedef union __attribute__((aligned(4))) {
	EEPROM_Read_N_Request_type bit;
	
	uint8_t reg[sizeof(EEPROM_Read_N_Request_type)];
} EEPROM_Read_N_Request;


//#define EEPROM_WRITE_REQUEST_HEADER 0x0002

typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint32_t address;
	uint8_t data;
	
	uint32_t crc;
} EEPROM_Write_Request_Type;


typedef union __attribute__((aligned(4))) {
	EEPROM_Write_Request_Type bit;
	
	uint8_t reg[sizeof(EEPROM_Write_Request_Type)];
} EEPROM_Write_Request;


//#define EEPROM_WRITE_N_REQUEST_HEADER 0x0005

typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint8_t size;
	uint32_t address;
	uint8_t data[64];
	
	uint32_t crc;
} EEPROM_Write_N_Request_Type;


typedef union __attribute__((aligned(4))) {
	EEPROM_Write_N_Request_Type bit;
	
	uint8_t reg[sizeof(EEPROM_Write_N_Request_Type)];
} EEPROM_Write_N_Request;


//#define SET_VEC3_REQUEST_HEADER 0x0006

typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint16_t parameter; // cast from CTRL_Param
	float data[3];
	
	uint32_t crc;
} Set_Vec3_Request_Type;


typedef union __attribute__((aligned(4))) {
	Set_Vec3_Request_Type bit;
	
	uint8_t reg[sizeof(Set_Vec3_Request_Type)];
} Set_Vec3_Request;


typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint16_t parameter;
	
	uint32_t crc;
} Read_Vec3_Request_Type;

typedef union __attribute__((aligned(4))) {
	Read_Vec3_Request_Type bit;
	
	uint8_t reg[sizeof(Read_Vec3_Request_Type)];
} Read_Vec3_Request;


typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	float data[3];
	
	uint32_t crc;
} CTRL_Read_Vec3_Type;

typedef union __attribute__((aligned(4))) {
	CTRL_Read_Vec3_Type bit;
	
	uint8_t reg[sizeof(CTRL_Read_Vec3_Type)];
} CTRL_Read_Vec3;


typedef struct {
	uint16_t header;
	
	uint16_t parameter;
	
	uint32_t crc;
} Save_Vec3_Request_Type;

typedef union {
	Save_Vec3_Request_Type bit;
	
	uint8_t reg[sizeof(Save_Vec3_Request_Type)];
} Save_Vec3_Request;


typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint16_t parameter; // cast from CTRL_Param
	float data;
	
	uint32_t crc;
} Set_Scalar_Request_Type;


typedef union __attribute__((aligned(4))) {
	Save_Vec3_Request_Type bit;
	
	uint8_t reg[sizeof(Save_Vec3_Request_Type)];
} Set_Scalar_Request;


typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	float data[3];
	
	uint32_t crc;
} CTRL_Read_Scalar_Type;

typedef union __attribute__((aligned(4))) {
	CTRL_Read_Scalar_Type bit;
	
	uint8_t reg[sizeof(CTRL_Read_Scalar_Type)];
} CTRL_Read_Scalar;


typedef struct {
	uint16_t header;
	
	uint16_t parameter;
	
	uint32_t crc;
} Save_Scalar_Request_Type;

typedef union {
	Save_Scalar_Request_Type bit;
	
	uint8_t reg[sizeof(Save_Scalar_Request_Type)];
} Save_Scalar_Request;


#endif