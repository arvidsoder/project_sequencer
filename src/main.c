

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"
#include <i2cmaster.h>
#include <stdio.h>
#include <avr/interrupt.h>
#define MCP4725ADDR (0x60 << 1) // address of DAC i2c, 0xC0

int value;

void init(void)
{
    sei();
    i2c_init();
    lcd_init( LCD_DISP_ON );
    lcd_clrscr();
    lcd_puts("Start");
}

int main(void)
{
    init();
    value = 0;
    lcd_clrscr();
    lcd_puts("Hello World");
    while (1)
    {
        value = value % 4000;
        i2c_start_wait(MCP4725ADDR + I2C_WRITE);
        i2c_write(64);
        i2c_write(value >> 4);
        i2c_write((value % 15) << 4); 
        i2c_stop();
        value ++;
    }
    
    return 0;
}