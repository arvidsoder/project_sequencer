#include <avr/io.h>
#include <util/delay.h>
#include <i2cmaster.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <lcd.h>
#include <math.h>

#define DAC_MCP4725ADDR (0x60 << 1) // address of DAC i2c, 0xC0
#define EEPROM_24AA512ADDR (0b1010000 << 1) // address of EEPROM
#define IO_EXPANDER_ADDR 0b01000000

// Define SPI pins
#define MOSI    PB3
#define SCK     PB5
#define SS      PB2  // Chip Select for MAX7219
// Rotary Encoder
#define ENC_PIN_A 2  // PD2
#define ENC_PIN_B 0  // PB0 
#define ENC_BUTTON_PIN 3//

// Variables to track encoder state
volatile static uint8_t A_state = 0;
volatile static uint8_t B_state = 0;
volatile static int16_t R_count = 0;
volatile static uint8_t rotor_flag = 0;

void encoder_init() {
    // Set Pin A (PD2) and button (PD3) as inputs with pull-ups
    DDRD &= ~((1 << ENC_PIN_A) | (1 << ENC_BUTTON_PIN));  


    // Set PB0 as input for B with pull-up
    DDRB &= ~(1 << ENC_PIN_B);
    PORTB |= (1 << PB0);

    // Enable external interrupts for Pin A (PD2) and Pin B (PD3)
    EICRA |= (1 << ISC01) | (0 << ISC00) | (1 << ISC11) | (0 << ISC10); //| (0 << ISC11) | (1 << ISC10); // Any change on INT0 and falling edge INT1 (button)
    EIMSK |= (1 << INT1) | (1 << INT0);   // Enable INT0 and INT1

}

// Interrupt Service Routine for Pin A (PD2)
void decode_rotor(void) {
    if (!A_state) // A falling edge.
    {
    if (!B_state) R_count ++; // Positive direction (arbitrary)
    else R_count --;    // Negative direction (arbitrary)
    }
else // A falling edge.
    {
    if (B_state) R_count ++; // Positive direction (arbitrary)
    else R_count --; // Negative direction (arbitrary)
    }
}



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




char midi_array[12][3] = {"C ","C#","D ","D#","E ","F ","F#","G ","G#","A ","A#","B "};


volatile uint8_t note_array[64];
volatile uint8_t note_array_length = 8;
uint8_t tempo = 120;
uint8_t swing = 50;
int8_t mem_bank = 0;
int8_t note_row = 0;
uint8_t lcd_posx = 0;
uint8_t lcd_posy = 0;
volatile uint8_t menu_select=0; //latching 
int8_t menu_item = 0;
char lcd_buffer1[32];
uint8_t led_buffer[8];
volatile int8_t lcd_print_flag = 0;

// Interrupt Service Routine for 
ISR(INT1_vect) { 
    menu_select = !menu_select;
}

// Interrupt Service Routine for Pin A (PD2)
//}
ISR(INT0_vect){ 
rotor_flag=1;
A_state = ((PIND & (1 << ENC_PIN_A)) >= 1);
B_state = ((PINB & (1 << ENC_PIN_B)) >= 1);

}

ISR(TIMER0_OVF_vect){
    lcd_print_flag=1;
}

void init(void)
{
    
    i2c_init();
    SPI_init();
    _delay_ms(1);
    MAX7219_init();
    encoder_init();
    lcd_init(LCD_DISP_ON);
    lcd_clrscr();
  
    lcd_puts("hello world");
    //DDRB &= ~((1 << ENC_P1) | (1 << ENC_P2)); //rotary encoder
    //DDRD &= ~((1 << ENC_P3) | (1 << ENC_P4));

    TCCR0A = (0 << COM0A1 ) | (0 << COM0A0 ) // no PWM
           | (0 << COM0B1 ) | (0 << COM0B0 ) // 
           | (0 << WGM01 ) | (0 << WGM00 ); // WGMxx sets the mode of the timer
    TCCR0B = (0 << WGM02 ) // WGMxx sets the mode of the timer
           | (1 << CS02) | (0 << CS01) | (1 << CS00 ); // clock prescaler
            // enable the TIMER0 OVERFLOW INTERRUPT
    TIMSK0 = (0 << OCIE0B ) | (0 << OCIE0A ) | (1 << TOIE0 );

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
        if (!menu_select){
            menu_item = (menu_item+R_count)%4;
            if (menu_item < 0) menu_item=3;
            R_count=0;
            
        }
        else {
            switch (menu_item) {
                case 0:
                    tempo = tempo+R_count;
                    
                    break;
                case 1:
                    mem_bank = (mem_bank+R_count)%100;
                    
                    break;
                case 2:
                    swing = (swing+R_count)%100;
                    
                    break;
                case 3:
                    note_row = (note_row+R_count)%8;
                    if (note_row < 0) note_row=7;
                    break;

                default:
                    break;
            }
            R_count=0;
        }

        
        
        if (lcd_print_flag==1){
            sprintf(lcd_buffer1," Tempo%3d Bank%2d Swing %2d Row  %1d", tempo, mem_bank, swing, note_row+1);
            
            lcd_clrscr();
            lcd_gotoxy(0,0);
            lcd_puts(lcd_buffer1);

            switch (menu_item) {
                case 0:
                    lcd_posx=0;
                    lcd_posy=0;
                    break;
                case 1:
                    lcd_posx=9;
                    lcd_posy=0;
                    break;
                case 2:
                    lcd_posx=0;
                    lcd_posy=1;
                    break;
                case 3:
                    lcd_posx=9;
                    lcd_posy=1;
                    break;
                default:
                    break;
            }

            lcd_gotoxy(lcd_posx, lcd_posy);
            if (menu_select) lcd_command(LCD_DISP_ON_CURSOR_BLINK);
            else lcd_command(LCD_DISP_ON_CURSOR);
            display_pattern(smiley_face);

            lcd_print_flag=0;
        }
        
        if (rotor_flag){
            decode_rotor(); //R_count++ or --
            _delay_ms(5); //debounce
            rotor_flag=0;
        }
    }
    
    return 0;
}