#include <avr/io.h>
#include <util/delay.h>
#include <i2cmaster.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "nokia5110.h"
#define DAC_MCP4725ADDR (0x60 << 1) // address of DAC i2c, 0xC0
#define EEPROM_24AA512ADDR (0b1010000 << 1) // address of EEPROM

// Define SPI pins
#define MOSI    PB3
#define SCK     PB5
#define SS      PB2  // Chip Select for MAX7219
// Rotary Encoder
#define ENC_P1  PB6 //PB6
#define ENC_P2  PB7 //PB7
#define ENC_P3  PD5 //PD5
#define ENC_P4  PD6 //PD6


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

uint8_t Graytobin[16] = {0,15,7,8,3,12,4,11,1,14,6,9,2,13,5,10};

uint8_t read_encoder(void){
    uint8_t inv = 0;
    uint8_t gray = 1*((PIND & (1<<ENC_P4))==0) + 2*((PIND & (1<<ENC_P3))==0) + 4*((PINB & (1<<ENC_P2))==0) + 8*((PINB & (1<<ENC_P1))==0);
    inv = Graytobin[gray];
    return inv;
}

char midi_array[12][3] = {"C ","C#","D ","D#","E ","F ","F#","G ","G#","A ","A#","B "};


volatile uint8_t rotary_enc;
volatile uint8_t note_array[64];
volatile uint8_t note_array_length = 8;

void init(void)
{
 
    i2c_init();
    SPI_init();
    _delay_ms(1);
    MAX7219_init();
    NOKIA_init(0);
    NOKIA_setVop(50);
    rotary_enc = read_encoder(); //initial value
    DDRB &= ~((1 << ENC_P1) | (1 << ENC_P2)); //rotary encoder
    DDRD &= ~((1 << ENC_P3) | (1 << ENC_P4));

    TCCR0A = (0 << COM0A1 ) | (0 << COM0A0 ) // no PWM
           | (0 << COM0B1 ) | (0 << COM0B0 ) // 
           | (0 << WGM01 ) | (0 << WGM00 ); // WGMxx sets the mode of the timer
    TCCR0B = (0 << WGM02 ) // WGMxx sets the mode of the timer
           | (0 << CS02) | (1 << CS01) | (0 << CS00 ); // clock prescaler
            // enable the TIMER0 OVERFLOW INTERRUPT
    TIMSK0 = (0 << OCIE0B ) | (0 << OCIE0A ) | (1 << TOIE0 );
    
    sei();
}

volatile char lcd_buffer1[8];
uint8_t led_buffer[8];
volatile int8_t foobar = 60;

ISR(TIMER0_OVF_vect){
    int8_t diff = read_encoder() - rotary_enc;
    int8_t increment = 0;

    if (((diff > 0) && (diff < 6)) || (diff < -7)) 
    {
        increment = 1;
    }  else if (((diff < 0) && (diff > -6)) || (diff > 7))
    {
        increment = -1;
    }
    foobar += increment;
    rotary_enc = read_encoder();
        
}

int main(void)
{
    uint8_t ret;
    uint16_t value;



    init();
    value = 0;
    
    display_pattern(smiley_face);
    
    while (1)
    {
        value = value % 4000;
        DAC_Write(value);
        value ++;
        sprintf(lcd_buffer1,"en %2s%d",midi_array[abs(foobar)%12],(foobar)/12 -1);
        NOKIA_clear();
        NOKIA_print(0,0,lcd_buffer1,2);
        NOKIA_update();

    }
    
    return 0;
}