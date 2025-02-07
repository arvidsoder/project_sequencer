#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"
#include <i2cmaster.h>
#include <stdio.h>
#include <avr/interrupt.h>
#define DAC_MCP4725ADDR (0x60 << 1) // address of DAC i2c, 0xC0
#define EEPROM_24AA512ADDR (0b1010000 << 1) // address of EEPROM

// Define SPI pins
#define MOSI    PB3
#define SCK     PB5
#define SS      PB2  // Chip Select for MAX7219



void DAC_Write(uint16_t value){
    i2c_start_wait(DAC_MCP4725ADDR + I2C_WRITE);    //Address the DAC (write)
    i2c_write(64);                              //Write value to DAC
    i2c_write(value >> 4);                      //High byte, 8 out of 12 bits
    i2c_write((value % 15) << 4);               //Low byte , last 4 bits
    i2c_stop();
}

void EEPROMwriteAddress(uint16_t address, uint8_t val){ // Writes a value val to specified memory address on EEPROM
    i2c_start_wait(EEPROM_24AA512ADDR + I2C_WRITE);
    i2c_write(address >> 8); //address
    i2c_write(address & 0xFF); //address
    i2c_write(val);
    i2c_stop();
    _delay_ms(5);
}

uint8_t EEPROMreadAddress(uint16_t address){ // Reads value from address of EEPROM
    i2c_start_wait(EEPROM_24AA512ADDR + I2C_WRITE);

    i2c_write(address >> 8); //address
    i2c_write(address & 0xFF); //address
    i2c_rep_start(EEPROM_24AA512ADDR + I2C_READ);

    uint8_t reading = i2c_readNak();
    i2c_stop();
    return reading;
}

void SPI_init() {
    DDRB |= (1 << MOSI) | (1 << SCK) | (1 << SS); // Set SPI pins as output
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0); // Enable SPI, Master mode, f/16 speed
}

void SPI_send(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF))); // Wait for transmission complete
}

void SPI_disable() {
    SPCR &= ~(1 << SPE);  // Disable SPI
}

void SPI_enable() {
    SPCR |= (1 << SPE);   // Enable SPI
}

void MAX7219_send(uint8_t address, uint8_t data) {
    PORTB &= ~(1 << SS);  // Select MAX7219 (CS LOW)
    SPI_send(address);     // Send register address
    SPI_send(data);        // Send data
    PORTB |= (1 << SS);   // Deselect MAX7219 (CS HIGH)
}

void MAX7219_init() {
    MAX7219_send(0x09, 0x00); // No decode mode (LED matrix mode)
    MAX7219_send(0x0A, 0x05); // Medium brightness
    MAX7219_send(0x0B, 0x07); // Scan limit = 8 rows
    MAX7219_send(0x0C, 0x01); // Normal operation mode
    MAX7219_send(0x0F, 0x00); // Disable test mode
}

void display_pattern(uint8_t *pattern) {
    for (uint8_t i = 0; i < 8; i++) {
        MAX7219_send(i + 1, pattern[i]);
    }
}

uint8_t smiley_face[8] = {
    0b00111100,
    0b01000010,
    0b10100101,
    0b10000001,
    0b10100101,
    0b10011001,
    0b01000010,
    0b00111100
};

void init(void)
{
    _delay_ms(100);
    sei();
    i2c_init();
    _delay_ms(1);
    lcd_init( LCD_DISP_ON );
    _delay_ms(1);
    lcd_clrscr();
    _delay_ms(1);
    SPI_init();
    _delay_ms(1);
    MAX7219_init();

}

int main(void)
{
    uint8_t ret;
    uint16_t value;
    char lcd_buffer1[16];
    uint8_t led_buffer[8];
    memcpy(led_buffer, smiley_face, 8*sizeof(smiley_face));

    init();
    value = 0;
    display_pattern(smiley_face);
    SPI_disable();
    EEPROMwriteAddress(0x0005, 69);
    ret = EEPROMreadAddress(0x0005);

    sprintf(lcd_buffer1, "mem: %d", ret);
    lcd_clrscr();
    lcd_puts("hadj");
    _delay_ms(50);
    
    _delay_ms(50);
    
    
    while (1)
    {
        value = value % 4000;
        DAC_Write(value);
        value ++;
        _delay_ms(50);
        lcd_clrscr();
        sprintf(lcd_buffer1, "%d",value);
        lcd_puts(lcd_buffer1);

    }
    
    return 0;
}