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
   GPIO 6 SCK/spi0_sclk -> SCL/SCK on piface board
   GPIO 7 MOSI/spi0_tx -> SDA/SDI on piface board
   3.3v (pin 3;6) -> VCC on piface board
   GND (pin 38)  -> GND on piface board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.

   Reference datasheet can be found below
  https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
*/


#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7

#define SPI_PORT spi0
#define READ_BIT 0x80

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

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

static void mcp23s17_write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg & 0x7f;  // remove read bit as this is a write
    buf[1] = data;
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
    sleep_ms(10);
}

static void mcp23s17_read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    uint8_t local_buf[2];
    //reg |= READ_BIT;
    local_buf[0] = READ_BIT; //Considering slave address as 000
    local_buf[1] = reg; //
    cs_select();
    spi_write_blocking(SPI_PORT, local_buf, 2);
    sleep_ms(10);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
    sleep_ms(10);
}

int main()
{

    bi_decl(bi_program_description("This is a test binary."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

    stdio_init_all();

     // This example will use SPI0 at 0.5MHz.
    spi_init(SPI_PORT, 500 * 1000);
    spi_gpio_init();

     // See if SPI is working - interrograte the device for its I2C ID number, should be 0x60
    uint8_t id;
    mcp23s17_read_registers(0x00, &id, 1);
    printf("Chip ID is 0x%x\n", id);
}