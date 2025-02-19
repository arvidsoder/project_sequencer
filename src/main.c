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

#define _MIN_ -100
#define _MAX_ 999
#define ENC_PIN_A 6  // PD6 (PCINT22)
#define ENC_PIN_B 5  // PD5 (PCINT21)

volatile static int16_t encoder_count = 0;
// Variables to track encoder state
volatile static uint8_t A_state = 0;
volatile static uint8_t B_state = 0;
volatile static int16_t R_count = 0;
volatile static uint8_t last_state = 0;
volatile static uint8_t last_last_state = 0;
void encoder_init() {
    // Set Pin A (PD2) and Pin B (PD3) as inputs with pull-ups
    DDRD &= ~((1 << PD2) | (1 << PD3));  


    // Set PB0 as input for push button with pull-up
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);

    // Enable external interrupts for Pin A (PD2) and Pin B (PD3)
    EICRA |= (0 << ISC01) | (1 << ISC00) | (0 << ISC11) | (1 << ISC10); // Any change on INT0 and INT1
    EIMSK |= (1 << INT1) | (1 << INT0);   // Enable INT0 and INT1

}

// Interrupt Service Routine for Pin A (PD2)
void decode_rotor(void) {
    
    uint8_t pinState = PIND;
    uint8_t new_state = ((pinState & (1 << ENC_PIN_A)) ? 1 : 0) | (((pinState & (1 << ENC_PIN_B)) ? 1 : 0) << 1);

    if ((last_last_state == 0b11 && last_state == 0b10 && new_state == 0b00)) {
        if (R_count < _MAX_) R_count++;
    }

    if ((last_last_state == 0b11 && last_state == 0b01 && new_state == 0b00))  {
        if (R_count > _MIN_) R_count--;
    }

    last_state = new_state;
    last_last_state = last_state;
     //debug
}

// Interrupt Service Routine for Pin B (PD3)
//ISR(INT1_vect) {
////decode_rotor();
//}
//ISR(INT0_vect){
////decode_rotor();
//}


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

uint8_t Read_encoder(void){
    uint8_t inv = 0;
    uint8_t gray = 1*((PIND & (1<<ENC_P4))==0) + 2*((PIND & (1<<ENC_P3))==0) + 4*((PINB & (1<<ENC_P2))==0) + 8*((PINB & (1<<ENC_P1))==0);
    inv = Graytobin[gray];
    return inv;
}



char midi_array[12][3] = {"C ","C#","D ","D#","E ","F ","F#","G ","G#","A ","A#","B "};


volatile uint8_t rotary_enc;
volatile uint8_t note_array[64];
volatile uint8_t note_array_length = 8;
volatile char lcd_buffer1[8];
uint8_t led_buffer[8];
volatile int8_t foobar = 60;


void init(void)
{
    
    //i2c_init();
    SPI_init();
    //_delay_ms(1);
    //MAX7219_init();
    NOKIA_init(0);
    _delay_ms(40);
    NOKIA_setVop(255);
    NOKIA_clear();
    NOKIA_print(0,0,"dfs",0);
    NOKIA_update();
    
    rotary_enc = 0; //initial value
    //DDRB &= ~((1 << ENC_P1) | (1 << ENC_P2)); //rotary encoder
    //DDRD &= ~((1 << ENC_P3) | (1 << ENC_P4));
/*
    TCCR0A = (0 << COM0A1 ) | (0 << COM0A0 ) // no PWM
           | (0 << COM0B1 ) | (0 << COM0B0 ) // 
           | (0 << WGM01 ) | (0 << WGM00 ); // WGMxx sets the mode of the timer
    TCCR0B = (0 << WGM02 ) // WGMxx sets the mode of the timer
           | (0 << CS02) | (1 << CS01) | (0 << CS00 ); // clock prescaler
            // enable the TIMER0 OVERFLOW INTERRUPT
    TIMSK0 = (0 << OCIE0B ) | (0 << OCIE0A ) | (1 << TOIE0 );
*/
    //cli(); //disable interrups
    sei();
}








int main(void)
{
    uint8_t ret;
    uint16_t value;



    init();
    value = 0;


    while (1)
    {
    

    sprintf(lcd_buffer1,"je %d", 2); //1*((PIND & (1 << PD2))==0), 1*((PIND & (1 << PD3))==0));
    NOKIA_clear();
    NOKIA_print(0,0,"srs",0);
    
    NOKIA_update();

    _delay_ms(20);

        
    }
    
    return 0;
}