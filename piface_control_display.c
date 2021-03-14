/***************************************************************************//**
 * @file piface_control_display.c
 * @brief this file contains example code for piface control and display module
 *******************************************************************************
 * This file is part of the GPLv3 distribution
 * (https://github.com/engineershrenik/pico_piface_control_and_display) Copyright (c) 2020
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "mcp23s17.h"

/* Example code to talk to piface control and display module over spi.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor SPI) cannot be used at 5v.

   You will need to use a level shifter on the SPI lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board and a piface control board.

TODO: Below is to be changed acc to piface<->pico connections

   GPIO 4 MISO/spi0_rx-> SDO/SDO on piface board
   GPIO 5 Chip select -> CSB/!CS on piface board
   GPIO 2 SCK/spi0_sclk -> SCL/SCK on piface board
   GPIO 3 MOSI/spi0_tx -> SDA/SDI on piface board
   3.3v (pin 3;6) -> VCC on piface board
   GND (pin 38)  -> GND on piface board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.

   Reference datasheet can be found below
  https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
*/


//#define PIN_MISO 4
//#define PIN_CS   5
//#define PIN_SCK  6
//#define PIN_MOSI 7

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3


#define SPI_PORT spi0
#define READ_BIT 0x80

const int hw_addr = 0;

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

const uint LED_PIN = 25;

void spi_gpio_init(void)
{
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
     //gpio_set_function(PIN_CS, GPIO_FUNC_SIO);
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

static uint8_t get_spi_control_byte(uint8_t rw_cmd, uint8_t hw_addr)
{
    hw_addr = (hw_addr << 1) & 0xE;
    rw_cmd &= 1; // just 1 bit long
    return 0x40 | hw_addr | rw_cmd;
}

static void mcp23s17_write_register (uint8_t data, uint8_t reg) {

    uint8_t control_byte = get_spi_control_byte(WRITE_CMD, hw_addr);
    uint8_t tx_buf[3] = {control_byte, reg, data};
    printf("SPI control byte 0x%x\n",control_byte);
    //uint8_t rx_buf[sizeof tx_buf];
    cs_select();
    spi_write_blocking(SPI_PORT, tx_buf, 2);
    cs_deselect();
    sleep_ms(10);
}



static void mcp23s17_read_register(uint8_t reg, uint8_t *rx_buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t control_byte = get_spi_control_byte(READ_CMD, hw_addr);
    uint8_t tx_buf[3] = {control_byte, reg, 0};
    //uint8_t rx_buf[3];
    printf("SPI control byte 0x%x\n",control_byte);
    cs_select();
    spi_write_blocking(SPI_PORT, tx_buf, 2);
    sleep_ms(1);
    spi_read_blocking(SPI_PORT, 0, rx_buf, 1);
    cs_deselect();
    sleep_ms(1);
}

int main()
{
    // config register
    const uint8_t ioconfig = BANK_OFF | \
                             INT_MIRROR_OFF | \
                             SEQOP_OFF | \
                             DISSLW_OFF | \
                             HAEN_ON | \
                             ODR_OFF | \
                             INTPOL_LOW;

    //bi_decl(bi_program_description("This is a test binary."));
    //bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

    stdio_init_all();

     // This example will use SPI0 at 0.5MHz.
    spi_init(SPI_PORT, 10000 * 1000);
    spi_gpio_init();

     // See if SPI is working - interrograte the device for its I2C ID number, should be 0x60
    uint8_t id;

    mcp23s17_write_register(ioconfig, IOCON);
     // I/O direction
    mcp23s17_write_register(0x00, IODIRB);
    mcp23s17_write_register(0xff, IODIRA);

    // GPIOB pull ups
    mcp23s17_write_register(0xff, GPPUA);

    // Write 0xaa to GPIO Port B
    mcp23s17_write_register(0x00, GPIOB);
    sleep_ms(10);
    mcp23s17_write_register(0xaa, GPIOB);
    sleep_ms(10);
    mcp23s17_write_register(0x55, GPIOB);
    sleep_ms(10);
    mcp23s17_write_register(0x00, GPIOB);

    mcp23s17_read_register(GPIOA, &id, 1);
    printf("Read data register 0x%X 0x%x\n",GPIOA, id);
}