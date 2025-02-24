#define F_CPU 1000000UL
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
#define IO_EXPANDER_CS PC3

// Define SPI pins
#define MOSI    PB3
#define SCK     PB5
#define SS      PB2  // Chip Select for MAX7219
// Rotary Encoder
#define ENC_PIN_A 2  // PD2
#define ENC_PIN_B 0  // PB0 
#define ENC_BUTTON_PIN 3 //PD3
//buttons
#define MENU_BUTTON_PIN 0 //IO exp PORTB


char lcd_buffer1[32];
volatile static uint16_t millis_tempo = 0;
volatile static uint16_t millis = 0;
// Variables to track encoder state
volatile static uint8_t A_state = 0;
volatile static uint8_t B_state = 0;
volatile static int16_t R_count = 0;
volatile static uint8_t rotor_flag = 0;
//button
static uint8_t menu_button_latch = 0;
volatile static uint8_t menu_select=0; //latching 
static uint8_t last_page_button_state = 1;
static uint8_t last_save_button_state = 1;

volatile static uint8_t note_array[64];
volatile static uint8_t velocity_array[64];

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



void DAC_Write(uint8_t value){
    uint16_t dac_value = ((uint16_t)value - 21)*36;
    if ((dac_value < 0) || (dac_value > 4095)) return;

    i2c_start_wait(DAC_MCP4725ADDR + I2C_WRITE);    //Address the DAC (write)
    i2c_write(64);                              //Write value to DAC
    i2c_write(dac_value >> 4);                      //High byte, 8 out of 12 bits
    i2c_write((dac_value % 15) << 4);               //Low byte , last 4 bits
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

void EEPROMwritePage128(uint16_t address, uint8_t *data64_1, uint8_t *data64_2){
    i2c_start_wait(EEPROM_24AA512ADDR + I2C_WRITE);
    i2c_write(address >> 8); //address
    i2c_write(address & 0xFF); //address

    for (uint8_t i=0; i < 64; i++) {
        i2c_write(data64_1[i]);
    }

    for (uint8_t i=0; i < 64; i++) {
        i2c_write(data64_2[i]);
    }
    
    i2c_stop();
    _delay_ms(5);
}

void EEPROMreadPage128(uint16_t address){
    i2c_start_wait(EEPROM_24AA512ADDR + I2C_WRITE);

    i2c_write(address >> 8); //address
    i2c_write(address & 0xFF); //address
    i2c_rep_start(EEPROM_24AA512ADDR + I2C_READ);

    for (uint8_t i=0; i < 64; i++) {
        note_array[i] = i2c_readAck();
    }

    for (uint8_t i=0; i < 63; i++) {
        velocity_array[i] = i2c_readAck();
    }
    velocity_array[63] = i2c_readNak();

    i2c_stop();
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
    DDRC |= (1 << IO_EXPANDER_CS);
    SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR1) | (1 << SPR0) | (0 << CPOL) | (0 << CPHA); // Enable SPI, Master mode, f/16 speed
}

uint8_t SPI_send(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF))); // Wait for transmission complete
    return SPDR;
}

void SPI_disable() {
    SPCR &= ~(1 << SPE);  // Disable SPI
}

void SPI_enable() {
    SPCR |= (1 << SPE);   // Enable SPI
}

void io_expander_write(uint8_t regist, uint8_t data){
    PORTC &= ~(1 << IO_EXPANDER_CS);
    SPI_send(IO_EXPANDER_ADDR);
    SPI_send(regist);
    SPI_send(data);
    PORTC |= (1 << IO_EXPANDER_CS);
}

uint8_t io_expander_read(uint8_t regist){
    uint8_t data;

    PORTC &= ~(1 << IO_EXPANDER_CS); // CS LOW
    uint8_t addr = IO_EXPANDER_ADDR | 0b00000001; // Read Mode
    SPI_send(addr); // Send Address + Read
    SPI_send(regist); // Send Register Address
    data = SPI_send(0x00); // Read Data
    PORTC |= (1 << IO_EXPANDER_CS); // CS HIGH
    return data;
    }

void io_expander_init(void){
    PORTC |= (1 << IO_EXPANDER_CS);
    io_expander_write(0x00, 0xFF); //PORT A as input
    io_expander_write(0x0C, 0x00); //PORT A disable pullup
    io_expander_write(0x01, 0b11110011); //PORT B as input
    io_expander_write(0x0D, 0b11111100); //PORT B disable pullup
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

void menu_button_press_read(uint8_t button_port_B){
    uint8_t page_button_state = button_port_B & (0x01);
    uint8_t save_button_state = button_port_B & (0x02);
    if (page_button_state == 0 && last_page_button_state == 1) {
        menu_button_latch = !menu_button_latch;  // Toggle state
        menu_select = 0;
    }
    if (save_button_state == 1 && last_save_button_state == 0) {
        EEPROMwritePage128(mem_bank*128, note_array, velocity_array);
    }

    last_page_button_state = page_button_state;
}

int in_the_range(int value, int start, int stop){
    int ret_value = value;
    if (value < start) ret_value = start;
    else if (value > stop) ret_value = stop;
    return ret_value;
}


char midi_array[12][3] = {"C ","C#","D ","D#","E ","F ","F#","G ","G#","A ","A#","B "};




uint8_t step = 0;
uint8_t current_note;
uint16_t inv_tempo;
//page 0
uint16_t tempo = 120;
int8_t note_row = 0;
int8_t mem_bank = 0;
volatile uint8_t note_array_length = 64;
//page 1
uint8_t swing = 50;
uint8_t legato = 100;
uint8_t legato_flag = 0;
uint8_t lcd_posx = 0;
uint8_t lcd_posy = 0;
uint8_t menu_page = 0;
int8_t menu_item = 0;

uint8_t led_buffer[8] = {0,0,0,0,0,0,0,0};
volatile int8_t lcd_print_flag = 0;

// IO
uint8_t trigger_flag = 0;
uint8_t Button_portA = 0;
uint8_t last_Button_portA = 0;
uint8_t Button_portB = 0xFF;

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

ISR(TIMER0_OVF_vect){ // f = 187500Hz 
    lcd_print_flag=1;
}

ISR(TIMER2_COMPA_vect){
    millis_tempo ++; //
    millis ++;
}

void init(void)
{
    
    i2c_init();
    SPI_init();
    _delay_ms(1);
    MAX7219_init();
    io_expander_init();
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
           | (1 << CS02) | (0 << CS01) | (0 << CS00 ); // clock prescaler
            // enable the TIMER0 OVERFLOW INTERRUPT
    TIMSK0 = (0 << OCIE0B ) | (0 << OCIE0A ) | (1 << TOIE0 );

    //timer 2
    TCCR2A = (0 << COM2A1 ) | (0 << COM2A0 ) // no PWM
           | (0 << COM2B1 ) | (0 << COM2B0 ) // 
           | (1 << WGM21 ) | (0 << WGM20 ); // WGMxx sets the mode of the timer
    TCCR2B = (0 << WGM22 ) // WGMxx sets the mode of the timer
           | (0 << CS22) | (1 << CS21) | (0 << CS20 ); // clock prescaler
            // enable the TIMER0 OVERFLOW INTERRUPT
    TIMSK2 = (0 << OCIE2B ) | (1 << OCIE2A ) | (0 << TOIE2 ); // f/8
    OCR2A = 125;
    sei();

    inv_tempo = 1000*30/tempo;
    note_array = {71,71,71,71,71,0,71,71,71,71,71,71,71,0,76,76,76,76,76,76,76,0,74,74,74,74,74,74,74,0,69,69,71,71,71,71,71,0,71,71,71,71,71,71,71,0,76,76,71,71,71,71,71,0,71,71,71,71,71,71,71,0,76,76};
    
}








int main(void)
{
    init();
    
    while (1)
    {
        
        if(Button_portA==0) {
            menu_page = menu_button_latch;
            if (last_Button_portA!=0) menu_select = 0; //falling edge
        } 
        else if (last_Button_portA==0){ // rising edge
            menu_page = 2;
            menu_select = 1;
            menu_item = 8;
        } 
        else { // holding button
            menu_page = 2;
        }
        last_Button_portA = Button_portA;

        if (Button_portA != 0) {
            step = in_the_range(Button_portA-1,0,7) + 8*note_row;
            DAC_Write(note_array[step]);
            legato_flag = 0;
            }
        else if ((millis_tempo >= inv_tempo) || (millis_tempo < 0)){
            millis_tempo = 0 ;
            step = (step + 1)%note_array_length;
            DAC_Write(note_array[step]);
            io_expander_write(0x13, 0b00001100);
            legato_flag = 1;
            trigger_flag = 1;
        }
        else if (((100*millis_tempo)/inv_tempo >= legato) && (legato_flag)){ // length of gate signal
            
            io_expander_write(0x13, io_expander_read(0x13) & (0b11111011));
            legato_flag = 0;
        }
        else if (trigger_flag) { //Trigger signal off
            io_expander_write(0x13, io_expander_read(0x13) & (0b11110111)); // PB3 = trigger PB2 = gate
            trigger_flag = 0;
        }
        current_note = note_array[step];

        if (!menu_select){
            menu_item = (menu_item+R_count)%4 + 4*menu_page;
            if (menu_item < 0) menu_item=3 + 4*menu_page;
            R_count=0;
            
        }
        else {
            switch (menu_item) {
                //page 0
                case 0:
                    tempo = (uint16_t)in_the_range(tempo+R_count, 10, 500);
                    inv_tempo = (uint16_t)(30.0*1000.0/(float)tempo); // period between quarter notes in ms
                    break;
                case 1:
                    note_row = in_the_range(note_row+R_count, 0, (note_array_length-1)/8);
                    break;
                case 2: 
                    note_array_length = in_the_range(note_array_length+R_count, 1, 64);
                    break;
                case 3:
                    mem_bank = in_the_range(mem_bank+R_count,0,99);
                    break;
                //page 1
                case 4:
                    swing = (swing+R_count)%100;
                    break;
                case 5:
                    
                    break;
                case 6:
                    legato = in_the_range(legato + R_count, 1, 100);
                    break;
                case 7:
                    break;
                //page 2
                case 8:
                    note_array[step] = note_array[step] + R_count;
                    break;
                case 9:
                    note_array[step] = 255;
                    break;
                //case 10: velocity
                //case 11: 

                default:
                    break;
            }
            R_count=0;
        }

        

        if (lcd_print_flag==1){
            switch (menu_page){
                case 0:
                    sprintf(lcd_buffer1," Tempo%3d Row %2d Notes %2d Ba%d", tempo,note_row, note_array_length, inv_tempo);
                    break;
                case 1:
                    sprintf(lcd_buffer1," Swing %2d        Legato %d", swing, legato);
                    break;
                case 2:
                    sprintf(lcd_buffer1," Note %2s%d  Clear %d  Velocity ", midi_array[current_note%12], (current_note-12)/12, current_note);
                    break;
            }

                
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
                case 4:
                    lcd_posx=0;
                    lcd_posy=0;
                    break;
                case 5:
                    lcd_posx=9;
                    lcd_posy=0;
                    break;
                case 6:
                    lcd_posx=0;
                    lcd_posy=1;
                    break;
                case 7:
                    lcd_posx=9;
                    lcd_posy=1;
                    break;
                case 8:
                    lcd_posx=0;
                    lcd_posy=0;
                    break;
                case 9:
                    lcd_posx=9;
                    lcd_posy=0;
                    break;
                case 10:
                    lcd_posx=0;
                    lcd_posy=1;
                    break;
                case 11:
                    lcd_posx=9;
                    lcd_posy=1;
                    break;
                default:
                    break;
            }

            lcd_gotoxy(lcd_posx, lcd_posy);
            if (menu_select) lcd_command(LCD_DISP_ON_CURSOR_BLINK);
            else lcd_command(LCD_DISP_ON_CURSOR);
            
            //LED Matrix
            for (int i=0;i<note_array_length/8;i++){ //upper rows
                led_buffer[i]=0xFF;
            }
            led_buffer[note_array_length/8] = (0xFF & ~(0xFF >> (note_array_length%8))); //last row
            
            led_buffer[step/8] &= ~(0b10000000 >> step%8);  //step highlight

            for (int i=note_array_length/8 +1;i<=7;i++){ //bottom rows
                led_buffer[i]=0x00;
            }

            if ((millis%600 > 300) && (menu_item==1) && (menu_select==1)){ //rows selected
                led_buffer[note_row] = 0x00;
            }
            
            display_pattern(led_buffer);

            Button_portA = (uint8_t)(log((255-io_expander_read(0x12))*2)/log(2));
            Button_portB = io_expander_read(0x13);
            menu_button_press_read(Button_portB);
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