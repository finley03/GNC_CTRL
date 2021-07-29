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

#define EEPROM_NAV_CONFIG_START 0x1800

#define EEPROM_KALMAN_POSITION_UNCERTAINTY 0x1800 // vec3
#define EEPROM_KALMAN_VELOCITY_UNCERTAINTY 0x180C // vec3
#define EEPROM_KALMAN_ORIENTATION_UNCERTAINTY 0x1818 // vec3
#define EEPROM_KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY 0x1824 // vec3
#define EEPROM_KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL 0x1828 // scalar
#define EEPROM_KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL 0x182C // scalar
#define EEPROM_KALMAN_BARO_VARIANCE 0x1830 // scalar
#define EEPROM_KALMAN_ACCEL_VARIANCE 0x1834 // scalar
#define EEPROM_KALMAN_ANGULARVELOCITY_VARIANCE 0x1838 // scalar
#define EEPROM_KALMAN_GNSS_ZEROLAT 0x183C // scalar
#define EEPROM_KALMAN_GNSS_ZEROLONG 0x1840 // scalar


#define SCALAR_SIZE (sizeof(float))
#define VEC3_SIZE (3 * sizeof(float))
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