/*
 * Shiftreg.c
 *
 * Created: 20.03.2014 15:21:07
 *  Author: Øystein
 */ 
#include <asf.h>
#include "FT800.h"
#include "shiftreg.h"


struct spi_device SPI_DEVICE_1 = {
	.id = 1
};
struct spi_device SPI_DEVICE_2 = {
	.id = 2
};
struct spi_device SPI_DEVICE_3 = {
	.id = 3
};

void ShiftRegister_Init(void){
	
	spi_master_init((&AVR32_SPI0));
	spi_master_setup_device((&AVR32_SPI0), &SPI_DEVICE_1, SPI_MODE_1, SPI_BAUDRATE, 1);
	spi_master_setup_device((&AVR32_SPI0), &SPI_DEVICE_2, SPI_MODE_1, SPI_BAUDRATE, 1);
	spi_master_setup_device((&AVR32_SPI0), &SPI_DEVICE_3, SPI_MODE_1, SPI_BAUDRATE, 1);
}

//MÅ SKRIVES OM DA ET AV REGISTRENE VIL KUNNE HA FLERE NULLER, FORDI DEN TAR INN START OG TRACTIVE
uint8_t readShiftRegister(uint8_t CS){
	uint8_t MSB = 0;
	uint8_t rxdata = 0x00;
	uint8_t txdata = 0x00;
	uint8_t val = 0;
	delay_us(1);
	//Select slave
	gpio_set_pin_high(CS);	
	//Wait
	delay_us(1);
	//Get the MSB. Of some reason get the first bit shifted out from the shift 
	//register when CS go high
	MSB = gpio_get_pin_value(MISO); 
	spi_select_device((&AVR32_SPI0), &SPI_DEVICE_1);
	while(!spi_is_tx_ready(&AVR32_SPI0));
	spi_write_single(&AVR32_SPI0, txdata);
	
	//Read the spi data register
 	rxdata = (uint8_t)spi_get(&AVR32_SPI0);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	
	spi_deselect_device((&AVR32_SPI0), &SPI_DEVICE_1);
	//Parallel load shift register
	gpio_set_pin_low(CS);
	//Bitshift to get the bit in MSB position
	MSB = MSB<<7;
	//Bitshift rxdata
	rxdata = rxdata>>1;
	//add them together to compensate for MSB lost in SPI transfer
	rxdata += MSB;

	return rxdata;
}
	
