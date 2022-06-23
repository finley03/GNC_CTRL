#ifndef SPI_EEPROM_H
#define SPI_EEPROM_H

#include "util.h"
#include "spi.h"

// memory map
#define EEPROM_PAGE_SIZE 64
// programmed waypoints occupy first 32k of eeprom
#define EEPROM_PATH_START 0x0000
// configurations occupy second 32k of eeprom
#define EEPROM_CONFIG_START 0x1000
// pid values for control loop
#define EEPROM_PID_X 0x1000 // vec3
#define EEPROM_PID_Y 0x100C // vec3
#define EEPROM_PID_Z 0x1018 // vec3
#define EEPROM_X_MIX 0x1024 // vec3
#define EEPROM_Y_MIX 0x1030 // vec3
#define EEPROM_Z_MIX 0x103C // vec3
#define EEPROM_POSITION_PID 0x1048 // vec3
#define EEPROM_WAYPOINT_THRESHOLD 0x1054 // scalar
#define EEPROM_THRO_CONFIG 0x1058 // vec3 (zero, proportional pitch, reserved)
#define EEPROM_CHANNEL_TRIM 0x1064 // vec3 (aileron, elevator, rudder)
#define EEPROM_CHANNEL_REVERSE 0x1070 // int32 (packed booleans)
#define EEPROM_HEADING_PID 0x1074 // vec3
#define EEPROM_ALTITUDE_PID 0x1080 // vec3
#define EEPROM_ELEVATOR_TURN_P 0x108C // scalar
#define EEPROM_FLIGHT_MODE 0x1090 // scalar
#define EEPROM_DISABLE_KALMAN_UPDATE_DELAY 0x1094 // scalar
#define EEPROM_CTRL_FLAGS_1 0x1098 // int32 (packed booleans)
#define EEPROM_AOA 0x109C // scalar
#define EEPROM_ROLL_LIMIT 0x10A0 // scalar
#define EEPROM_PITCH_LIMIT 0x10A4 // scalar
#define EEPROM_FLIGHT_MODE_0 0x10A8 // scalar
#define EEPROM_FLIGHT_MODE_1 0x10AC // scalar
#define EEPROM_FLIGHT_MODE_2 0x10B0 // scalar
#define EEPROM_LOITER_RADIUS 0x10B4 // scalar
#define EEPROM_HOME_LOITER_ALT 0x10B8 // scalar
#define EEPROM_LAUNCH_THRO 0x10BC // scalar
#define EEPROM_LAUNCH_PITCH 0x10C0 // scalar
#define EEPROM_LAUNCH_MINACC 0x10C4 // scalar
#define EEPROM_LAUNCH_MINSPD 0x10C8 // scalar
#define EEPROM_LAUNCH_THRODELAY 0x10CC // scalar
#define EEPROM_LAUNCH_TIME 0x10D0 // scalar

#define EEPROM_NAV_CONFIG_START 0x1800

#define EEPROM_KALMAN_POSITION_UNCERTAINTY 0x1800 // vec3
#define EEPROM_KALMAN_VELOCITY_UNCERTAINTY 0x180C // vec3
#define EEPROM_KALMAN_ORIENTATION_UNCERTAINTY 0x1818 // vec3
#define EEPROM_KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY 0x1824 // vec3
#define EEPROM_KALMAN_BARO_VARIANCE 0x1830 // scalar
#define EEPROM_KALMAN_ACCEL_VARIANCE 0x1834 // scalar
#define EEPROM_KALMAN_ANGULARVELOCITY_VARIANCE 0x1838 // scalar
#define EEPROM_KALMAN_GNSS_ZEROLAT 0x183C // scalar
#define EEPROM_KALMAN_GNSS_ZEROLONG 0x1840 // scalar
#define EEPROM_MAG_A_1 0x1844 // vec3
#define EEPROM_MAG_A_2 0x1850 // vec3
#define EEPROM_MAG_A_3 0x185C // vec3
#define EEPROM_MAG_B 0x1868 // vec3
#define EEPROM_ACCEL_B 0x1874 // vec3
#define EEPROM_GYRO_B 0x1880 // vec3
#define EEPROM_KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL 0x188C // scalar
#define EEPROM_KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL 0x1890 // scalar
#define EEPROM_BARO_HEIGHT_CAL 0x1894 // scalar


#define SCALAR_SIZE (sizeof(float))
#define VEC3_SIZE (3 * sizeof(float))
#define INT32_SIZE (sizeof(int32_t))
// 

void spi_eeprom_init();

void spi_eeprom_write_enable();
void spi_eeprom_write_disable();

uint8_t spi_eeprom_read_status();
void spi_eeprom_write_status(uint8_t data);

uint8_t spi_eeprom_read_byte(uint32_t address);
void spi_eeprom_write_byte(uint32_t address, uint8_t data);

void spi_eeprom_read_n(uint32_t address, void* writeback, uint32_t n);
// function is not page safe, but is faster
void spi_eeprom_write_n(uint32_t address, void* data, uint32_t n);
// function is page safe but slower. Handling of write enable is automatic.
void spi_eeprom_write_n_s(uint32_t address, void* data, uint32_t n);


#endif