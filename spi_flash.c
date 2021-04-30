#include "spi_flash.h"


void spi_flash_init() {
	// setup SS pins
	REG_PORT_OUTSET0 = SPI_FLASH_SS;
	REG_PORT_DIRSET0 = SPI_FLASH_SS;
}


uint8_t spi_flash_checkid() {
	REG_PORT_OUTCLR0 = SPI_FLASH_SS;
	spi_command(READ_JEDEC_ID);
	uint8_t manufID = spi_command(0);
	uint8_t deviceID_0 = spi_command(0);
	uint8_t deviceID_1 = spi_command(0);
	REG_PORT_OUTSET0 = SPI_FLASH_SS;
	
	if (manufID == 0x1f && deviceID_0 == 0x84 && deviceID_1 == 0x01) return 0;
	else return 1;
}


void spi_flash_write_enable() {
	REG_PORT_OUTCLR0 = SPI_FLASH_SS;
	spi_command(WRITE_ENABLE);
	REG_PORT_OUTSET0 = SPI_FLASH_SS;
}


void spi_flash_write_disable() {
	REG_PORT_OUTCLR0 = SPI_FLASH_SS;
	spi_command(WRITE_DISABLE);
	REG_PORT_OUTSET0 = SPI_FLASH_SS;
}
