/*
 * vim:ts=4:sw=4:expandtab
 *
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#define BAUD 9600
#include <util/setbaud.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/wdt.h>

#define TILT_LIMIT 40000
#define HEIGHT_LIMIT 90000

// True if measuring is going on to detect timer overflows
volatile bool measuring = false;

// true if the USART-Receive-Buffer was completely read by the main loop. Set to false when buffer full by usart-receive-interrupt.
volatile bool buffer_read = true;

// Positions of the 3 achsis relative to the endstop.
int32_t positions[4];

/**
 * Get the current direction of the stepper driver. True means up.
 */
bool get_direction(uint8_t driver) {
    switch(driver) {
        case 1:
            return !(PORTC & (1<<PC0));
        case 2:
            return !(PORTA & (1<<PA5));
        case 3:
            return (PORTA & (1<<PA2));
    }
}

void enable() {
    // Driver 1
    DDRC  |=  (1<<DDC2);
    PORTC &= ~(1<<PC2);
    // Driver 2
    DDRA  |=  (1<<DDA3);
    PORTA &= ~(1<<DDA3);
    // Driver 3
    DDRA  |=  (1<<DDA0);
    PORTA &= ~(1<<DDA0);
}

// Test if one axis reached the height limit.
bool heightLimitReached(uint8_t driver) {
    if (driver < 1 || driver > 3) {
        return true;
    }
    if (positions[driver] > HEIGHT_LIMIT) {
        return true;
    }
    return false;
}

// Test of any of the three axis reached the height limit
bool anyHeightLimitReached() {
    return heightLimitReached(1) || heightLimitReached(2) || heightLimitReached(3);
}

/**
 * Set the direction of a stepper driver
 *
 * @param driver index of the driver (1-3)
 * @param direction the new direction. True means up.
 */
void set_direction(uint8_t driver, bool dir) {
    switch(driver) {
        case 1:
            DDRC |= (1<<DDC0);
            if (dir != get_direction(driver)) {
                PORTC ^= (1<<PC0);
                _delay_us(1);
            }
            break;
        case 2:
            DDRA |= (1<<DDA5);
            if (dir != get_direction(driver)) {
                PORTA ^= (1<<PA5);
                _delay_us(1);
            }
            break;
        case 3:
            DDRA |= (1<<DDA2);
            if (dir != get_direction(driver)) {
                PORTA ^= (1<<PA2);
                _delay_us(1);
            }
            break;
    }
}

/** 
 * Check if a endstop has been reached. True means yes.
 *
 * @param motor 
 */
bool stop(uint8_t motor) {
    switch(motor) {
        case 1: return PIND & (1<<6);
        case 2: return PIND & (1<<3);
        case 3: return PIND & (1<<4);
    }
    return true;
}

/**
 * Execute one step in the given direction.
 *
 * @param driver index of the stepper driver (1-3)
 * @param direction Direction. True means up.
 */
void step(uint8_t driver, bool direction) {
    if (driver < 1 || driver > 3) {
        return;
    }
    if (stop(driver) && !direction) {
        return;
    }
    if (direction) {
        positions[driver]++;
    }
    else {
        positions[driver]--;
    }
    set_direction(driver, direction);
    switch(driver) {
        case 1:
            DDRC   |= (1<<DDC1);
            PORTC  |= (1<<PC1);
            _delay_us(1);
            PORTC &= ~(1<<PC1);
            _delay_us(1);
            break;
        case 2:
            DDRA   |= (1<<DDA4);
            PORTA  |= (1<<PA4);
            _delay_us(1);
            PORTA &= ~(1<<PA4);
            _delay_us(1);
            break;
        case 3:
            DDRA   |= (1<<DDA1);
            PORTA  |= (1<<PA1);
            _delay_us(1);
            PORTA &= ~(1<<PA1);
            _delay_us(1);
            break;
    }
}

// Execute one step up
void stepUp(uint8_t driver) {
    step(driver, true);
}

// Execute one step up for each motor
void stepAllUp() {
    stepUp(1);
    stepUp(2);
    stepUp(3);
}

// Execute one step down
void stepDown(uint8_t driver) {
    step(driver, false);
}

// Execute one step up for each motor
void stepAllDown() {
    stepDown(1);
    stepDown(2);
    stepDown(3);
}


static void uart_puts(const char *str);

// Timer 0 overflow
ISR(TIMER1_OVF_vect) {
}

// Timer 1 overflow
ISR(TIMER0_OVF_vect) {
}

// React to external enable signal
ISR(INT0_vect) {
}


static void uart_puts(const char *str) {
    const char *walk;
    for (walk = str; *walk != '\0'; walk++) {
        while ( !( UCSR0A & (1<<UDRE0)) );
        UDR0 = *walk;
    }
}

static void uart_put(char i) {
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = i;
}

static void uart_puti(int16_t val) {
    char buffer[12];
    sprintf(buffer, "%i", val);
    uart_puts(buffer);
}


static void uart_hex8(uint8_t i) {
    char chars[] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
    uart_put(chars[0xf & (i >> 4)]);
    uart_put(chars[0xf & i]);
}

static void uart_hex16(uint16_t i) {
    char chars[] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
    uart_put(chars[0xf & (i >> 12)]);
    uart_put(chars[0xf & (i >>  8)]);
    uart_put(chars[0xf & (i >>  4)]);
    uart_put(chars[0xf & i]);
}

static void uart_hex32(uint32_t i) {
    char chars[] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
    uart_put(chars[0xf & (i >> 28)]);
    uart_put(chars[0xf & (i >> 24)]);
    uart_put(chars[0xf & (i >> 20)]);
    uart_put(chars[0xf & (i >> 16)]);
    uart_put(chars[0xf & (i >> 12)]);
    uart_put(chars[0xf & (i >>  8)]);
    uart_put(chars[0xf & (i >>  4)]);
    uart_put(chars[0xf & i]);
}

static void uart_hex64(uint64_t i) {
    char chars[] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
    uart_put(chars[0xf & (i >> 60)]);
    uart_put(chars[0xf & (i >> 56)]);
    uart_put(chars[0xf & (i >> 52)]);
    uart_put(chars[0xf & (i >> 48)]);
    uart_put(chars[0xf & (i >> 44)]);
    uart_put(chars[0xf & (i >> 40)]);
    uart_put(chars[0xf & (i >> 36)]);
    uart_put(chars[0xf & (i >> 32)]);
    uart_put(chars[0xf & (i >> 28)]);
    uart_put(chars[0xf & (i >> 24)]);
    uart_put(chars[0xf & (i >> 20)]);
    uart_put(chars[0xf & (i >> 16)]);
    uart_put(chars[0xf & (i >> 12)]);
    uart_put(chars[0xf & (i >>  8)]);
    uart_put(chars[0xf & (i >>  4)]);
    uart_put(chars[0xf & i]);
}

static void uart_bin(uint8_t i) {
    uart_puts((i & (1<<7)) ? "1":"0");
    uart_puts((i & (1<<6)) ? "1":"0");
    uart_puts((i & (1<<5)) ? "1":"0");
    uart_puts((i & (1<<4)) ? "1|":"0|");
    uart_puts((i & (1<<3)) ? "1":"0");
    uart_puts((i & (1<<2)) ? "1":"0");
    uart_puts((i & (1<<1)) ? "1":"0");
    uart_puts((i & 1) ? "1":"0");
}

void enableLED() {
    DDRA |= (1<<4);
    PORTA |= (1<<4);
}

void disableLED() {
    DDRA |= (1<<4);
    PORTA &= ~(1<<4);
}

void adcInit() {

    // Set AVCC as reference
    ADMUX = (1<<REFS0);

    // Enable ADC, set free running, enable interrupt, set prescaler to 1/128
    //ADCSRA = (1<<ADEN) | (1<<ADSC);
    //ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    ADCSRA = (1<<ADEN) | (1<<ADSC) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
}

void adc_select_channel(uint8_t channel) {
    ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
}

/* ADC Einzelmessung */
uint16_t ADC_Read( uint8_t channel ) {
    // Kanal waehlen, ohne andere Bits zu beeinflußen
    ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
    while (ADCSRA & (1<<ADSC) ) {   // auf Abschluss der Konvertierung warten
    }
    return ADCW;                    // ADC auslesen und zurückgeben
}

#define buffer_size 16
uint8_t buffer[buffer_size];
volatile uint8_t buffer_index;

// Receive data via USART
ISR(USART0_RX_vect) {
    uint8_t byte = UDR0;
    if (!buffer_read) {
        return;
    }
    if (byte == '\r' || byte == '\n') {
        buffer[buffer_index] = 0;
        buffer_index = 0;
        buffer_read = false;
    }
    else {
        buffer[buffer_index] = byte;
        buffer_index++;
    }
}

void gotoEndstops() {
    int i = 0;
    // Count the number of steps until endstop
    uint32_t steps[3];
    steps[0] = steps[1] = steps[2] = 0;
    while (!stop(1) || !stop(2) || !stop(3)) {
        double moved = false;
        for (i = 1; i <= 3; i++) {
            if (!stop(i)) {
                moved = true;
                step(i, false);
                steps[i-1]++;
            }
        }
        if (!moved) {
            break;
        }
        _delay_us(400);
        wdt_reset();
    }
    uart_puts("Steps until endstop: ");
    for (i = 0; i < 3; i++) {
        uart_hex32(steps[i]);
        uart_puts(" ");
    }
    uart_puts("\r\n");

}

void leaveEndstops() {
    int i = 0;
    while (stop(1) || stop(2) || stop(3)) {
        double moved = false;
        for (i = 1; i <= 3; i++) {
            if (!stop(i)) {
                moved = true;
                step(i, true);
            }
        }
        if (!moved) {
            break;
        }
        _delay_us(400);
        wdt_reset();
    }
}

// go <limit> steps up
void goUp(const int limit) {
    int i = 0;
    int j = 0;
    for (j=0; j < limit; j++) {
        double moved = false;
        for (i = 1; i <= 3; i++) {
            step(i, true);
        }
        _delay_us(400);
        wdt_reset();
    }
}

// True if user pressed button for moving up.
bool buttonUp() {
    return !(bool)(PINC & (1<<7));
}

// True if user pressed button for moving down.
bool buttonDown() {
    return !(bool)(PINC & (1<<6));
}

// True if user pressed button for tilting front down.
bool buttonTiltFrontDown() {
    return !(bool)(PINC & (1<<5));
}

// True if user pressed button for tilting front up.
bool buttonTiltFrontUp() {
    return !(bool)(PINC & (1<<4));
}

// True if user pressed button for leveling.
bool buttonLevel() {
    return !(bool)(PINC & (1<<3));
}


void printPositions() {
    uart_hex32(positions[1]);
    uart_puts(" ");
    uart_hex32(positions[2]);
    uart_puts(" ");
    uart_hex32(positions[3]);
    uart_puts("\r\n");
}

int main() {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);

    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);

    // Use default timer mode (no compare match, no pwm)
    TCCR1A = 0;
    // Clock select 001 for CPU_FREQ
    TCCR1B = (0<<CS12) | (0<<CS11) | (1<<CS10);

    TIMSK1 = (1<<TOIE1);


    // Configure PD2 as input
    DDRD = 0;
    // Enable pullup for PD2, 6, 4, 3
    PORTD = (1<<2) | (1<<3) | (1<<4) | (1<<6);

    // Configure PC0 and PC1 as output
    DDRC |= (1 << 0 | 1 << 1);
    PORTC = 0;

    // Configure PC3-7 as input with pullups enabled
    PORTC |= (1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);

    // Enable INT0 interrupts
    EIMSK = (1<<INT0);

    // Configure INT0 interrupt to trigger for falling edge
    EICRA = (1<<ISC01);
    // enable interrupts for the timer
    sei();

    /* Timer aufsetzen: nach 1 ms soll der Interrupt ausgelöst werden. */
    /* 8 bit counter (TIMER0) */
    /* normal mode */
    TCCR0A = 0;
    /* CLK/64 */
    TCCR0B = (1 << CS01) | (1 << CS00);
    /* timer ticks: 250 */
    TCNT0 = 5;
    TIMSK0 = (1 << TOIE0);
    TIFR0 = (1 << TOV0);

    adcInit();

    DDRA |= (1<<7);
    PORTA |= (1<<7);
    int16_t i, j;

    for (i = 0; i < 4; i++) {
        positions[i] = 0;
    }


    // enable drivers
    enable();    

    wdt_enable(WDTO_2S);


    uart_puts("Starting up\r\n"); 

    gotoEndstops();
    goUp(300);
    gotoEndstops();


    bool buttonLastTime = false;

    for(i = 0;;i++){
        wdt_reset();
        _delay_us(200);
        if (!buffer_read) {
            uart_puts("Received buffer:\r\n");
            uart_puts(buffer);
            uart_puts("\r\n");
            buffer_read = true;
        }
        int8_t buttons = 0;
        if (buttonUp()) {
            buttons++;
        }
        if (buttonDown()) {
            buttons++;
        }
        if (buttonTiltFrontUp()) {
            buttons++;
        }
        if (buttonTiltFrontDown()) {
            buttons++;
        }
        if (buttonLevel()) {
            buttons++;
        }
        if (buttonLastTime && (buttons != 1)) {
            printPositions();
        }
        if (buttons == 1) {
            buttonLastTime = true;
            if (buttonDown()) {
                if (!stop(1) && !stop(2) && !stop(3)) {
                    stepAllDown();
                }
            }
            if (buttonUp()) {
                if (!anyHeightLimitReached()) {
                    stepAllUp();
                }
            }
            if (buttonTiltFrontUp()) {
                if (!stop(1) && !stop(2) && !stop(3) && !anyHeightLimitReached() && (positions[3]-positions[1] < TILT_LIMIT)) {
                    stepUp(3);
                    stepDown(1);
                }
            }
            if (buttonTiltFrontDown()) {
                if (!stop(1) && !stop(2) && !stop(3) && !anyHeightLimitReached() && (positions[1]-positions[3] < TILT_LIMIT)) {
                    stepDown(3);
                    stepUp(1);
                }
            }
            if (buttonLevel()) {
                if (!stop(1) && !stop(2) && !stop(3)) {
                    if (positions[1] > positions[3]+1) {
                        stepDown(1);
                        stepUp(3); 
                    }
                    else if (positions[1]+1 < positions[3]) {
                        stepDown(3);
                        stepUp(1);
                    }
                } 
            }
        }
        else {
            buttonLastTime = false;
        }
        if (buttons > 1) {
            uart_puts("Multiple buttons pressed\r\n");
        }
    }

}

