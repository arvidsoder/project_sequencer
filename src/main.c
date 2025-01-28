#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"
#include <i2cmaster.h>
#include <stdio.h>
#include <avr/interrupt.h>
#define DAC_MCP4725ADDR (0x60 << 1) // address of DAC i2c, 0xC0
#define EEPROM_24AA512ADDR (0b1010000 << 1) // address of EEPROM



void init(void)
{
    sei();
    i2c_init();
    lcd_init( LCD_DISP_ON );
    lcd_clrscr();
    lcd_puts("Start");
}

void DAC_Write(uint16_t value){
    i2c_start_wait(DAC_MCP4725ADDR + I2C_WRITE);    //Address the DAC (write)
    i2c_write(64);                              //Write value to DAC
    i2c_write(value >> 4);                      //High byte, 8 out of 12 bits
    i2c_write((value % 15) << 4);               //Low byte , last 4 bits
    i2c_stop();
}

void writeAddress(uint16_t address, uint8_t val){ // Writes a value val to specified memory address on EEPROM
    i2c_start_wait(EEPROM_24AA512ADDR + I2C_WRITE);
    i2c_write(address >> 8); //address
    i2c_write(address & 0xFF); //address
    i2c_write(val);
    i2c_stop();
}

uint8_t readAddress(uint16_t address){ // Reads value from address of EEPROM
    i2c_start_wait(EEPROM_24AA512ADDR + I2C_WRITE);

    i2c_write(address >> 8); //address
    i2c_write(address & 0xFF); //address
    i2c_rep_start(EEPROM_24AA512ADDR + I2C_READ);

    uint8_t reading = i2c_readNak();
    i2c_stop();
    return reading;
}

int main(void)
{
    uint8_t ret;
    uint16_t value;
    char buffer1[16];

    init();
    value = 0;
    
    writeAddress(0x0005, 69);
    ret = readAddress(0x0005);
    

    sprintf(buffer1, "mem: %d", ret);
    lcd_clrscr();
    lcd_puts(buffer1);

    while (1)
    {
        value = value % 4000;
        DAC_Write(value);
        value ++;


    }
    
    return 0;
}