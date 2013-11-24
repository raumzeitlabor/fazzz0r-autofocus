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

// Limit for tilting
#define TILT_LIMIT 40000

// Limit for height
#define HEIGHT_LIMIT 90000

// ADC value when the autofocus switch is pressed.
// Significantly higher values mean broken cable.
#define CLOSED_VALUE 671

// ADC value when the autofocus switch is not pressed.
// Significyntly lower values mean short circuit.
#define OPEN_VALUE 318

// Maximum difference between ADC value and OPEN_VALUE for which the autofocus switch is considered open.
// Values outside are considered proof of pressed switch, so moving up will not work.
#define INTERVAL_SIZE 23

// Lower ADC values will be consideres proof for short circuit.
#define SHORT_CIRCUIT_VALUE 286

// Higher ADC values will be consideres proof for broken cable.
#define CABLE_BROKEN_VALUE 704

// Delay between two steps in micro seconds when starting to move.
#define INITIAL_DELAY 500

// Shortest delay between two steps allowed (i.e. fastest movement).
#define MIN_DELAY 40

// Distance between a hit of the focus switch and the point of focus.
#define FOCUS_DISTANCE 3200*4

// true if the USART-Receive-Buffer was completely read by the main loop. Set to false when buffer full by usart-receive-interrupt.
volatile bool buffer_read = true;

// Positions of the 3 achsis relative to the endstop. [0] has the "average position"
int32_t positions[4];

// "average position" (see above) of the autofocus switch
int32_t autofocusPosition;

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

// Set the output pins for the three motor drivers.
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

/*!
 * Check if any of the endstops has been reached. True means yes.
 */
bool anyStopReached() {
    return stop(1) || stop(2) || stop(3);
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
	positions[0]++;
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
	positions[0]--;
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

/*!
 * Initialize the ADC
 */
void adcInit() {

    // Set AVCC as reference
    ADMUX = (1<<REFS0);

    // Enable ADC, set free running, enable interrupt, set prescaler to 1/128
    //ADCSRA = (1<<ADEN) | (1<<ADSC);
    //ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    ADCSRA = (1<<ADEN) | (1<<ADSC) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
}

/*!
 * Select the channel for the next conversion.
 */
void adc_select_channel(const uint8_t channel) {
    ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
}

/*!
 * Start a conversion.
 */
void adc_start_conversion(const uint8_t channel) {
    adc_select_channel(channel);
    ADCSRA |= (1<<ADSC);          // single conversion
}

/*!
 * Wait for the conversion to finish and return the result.
 */
uint16_t adc_wait() {
    while (ADCSRA & (1<<ADSC) ) {   // auf Abschluss der Konvertierung warten
    }
    return ADCW;                    // ADC auslesen und zurückgeben
}


// Buffer for receiving data via USART
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

/*!
 * Move down until all endstops are hit.
 */
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

/*!
 * Move up until no endstops are hit.
 */
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

// True if the user wants to got to the autofocus position
bool buttonFocus() {
	return !(bool)(PINB & (1<<0));
}

// Print the positions of the axis
void printPositions() {
    uart_hex32(positions[0]);
    uart_puts(" ");
    uart_hex32(positions[1]);
    uart_puts(" ");
    uart_hex32(positions[2]);
    uart_puts(" ");
    uart_hex32(positions[3]);
}

/*!
 * Simple delay function for which the time does not need to be known at compile time.
 */
void myDelayUs(uint16_t length) {
    while (--length) {
        _delay_us(1);
    }
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

	// Configure PB0 as input (gotoFocus button)
	DDRB |= (1<<0);
	PORTB |= (1<<0);

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
    
	for (i = 0; i < 4; i++) {
        positions[i] = 0;
    }


    bool buttonLastTime = false;
    
    // Set ADC used for switch as input and disable pullup
    DDRA &= ~(1<<7 | 1<<6);
    PORTA &= (1<<7 | 1<<6);

    // Count the number of cycles the autofocus is stuck in mid-state
    uint8_t mid_state_counter = 0;

    // Count the number of successive movements in the same direction before increasing speed
    uint16_t movement_counter = 0;

    uint16_t current_delay = INITIAL_DELAY;

    enum movement{NONE, UP, DOWN, TILT_DOWN, TILT_UP, LEVEL} last_movement, movement;
    last_movement = NONE;
    movement = NONE;

	// Reset autofocus position
	autofocusPosition = 0;

    for(i = 0;;i++){
        wdt_reset();
        adc_start_conversion(7);
        myDelayUs(current_delay);
        const uint16_t autofocus_result = adc_wait();
        // True if autofocus endstop not yet hit.
        const bool autofocus_clear = (autofocus_result <= OPEN_VALUE + INTERVAL_SIZE && autofocus_result >= OPEN_VALUE - INTERVAL_SIZE);
        
        if (!autofocus_clear) {
            uart_puts("Autofocus hit at: ");
            printPositions();
			autofocusPosition = positions[0];
            uart_puts("\r\n");
            if (autofocus_result < SHORT_CIRCUIT_VALUE) {
                uart_puts("Autofocus has short circuit, value is: ");
                uart_hex16(autofocus_result);
                uart_puts("\r\n");
            }
            else if (autofocus_result > CABLE_BROKEN_VALUE) {
                uart_puts("Autofocus cable is broken, value is: ");
                uart_hex16(autofocus_result);
                uart_puts("\r\n");
            }
            else if(autofocus_result > OPEN_VALUE + INTERVAL_SIZE && autofocus_result < CLOSED_VALUE - INTERVAL_SIZE) {
                if (mid_state_counter > 10) {
                    uart_puts("Autofocus stuck in mid-state, value is: 0x");
                    uart_hex16(autofocus_result);
                    uart_puts("\r\n");
                }
                else {
                    mid_state_counter++;
                }
            }
        }
        else {
            mid_state_counter = 0;
        }
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
		if (buttonFocus()) {
			buttons++;
		}
        if (buttonLastTime && (buttons != 1)) {
            printPositions();
            uart_puts("\r\n");
        }
        movement = NONE;
        if (buttons == 1) {
            buttonLastTime = true;
            if (buttonDown()) {
                if (!anyStopReached()) {
                    stepAllDown();
                    movement = DOWN;
                }
            }
            if (buttonUp()) {
                if (!anyHeightLimitReached() && autofocus_clear) {
                    stepAllUp();
                    movement = UP;
                }
            }
            if (buttonTiltFrontUp()) {
                if (!anyStopReached() && !anyHeightLimitReached() && (positions[3]-positions[1] < TILT_LIMIT) && autofocus_clear) {
                    stepUp(3);
                    stepDown(1);
                    movement = TILT_UP;
                }
            }
            if (buttonTiltFrontDown()) {
                if (!anyStopReached() && !anyHeightLimitReached() && (positions[1]-positions[3] < TILT_LIMIT) && autofocus_clear) {
                    stepDown(3);
                    stepUp(1);
                    movement = TILT_DOWN;
                }
            }
            if (buttonLevel()) {
                if (!anyStopReached() && autofocus_clear) {
                    if (positions[1] > positions[3]+1) {
                        stepDown(1);
                        stepUp(3);
                        movement = LEVEL;
                    }
                    else if (positions[1]+1 < positions[3]) {
                        stepDown(3);
                        stepUp(1);
                        movement = LEVEL;
                    }
                } 
            }
			if (buttonFocus()) {
				if (positions[0] > autofocusPosition - FOCUS_DISTANCE) {
					stepAllDown();
					movement = DOWN;
				}
			}
        }
        else {
            buttonLastTime = false;
        }
        if (buttons > 1) {
            uart_puts("Multiple buttons pressed\r\n");
        }
        if (movement == last_movement && movement != NONE) {
            if (movement_counter > 400) {
                current_delay--;
                if (current_delay < MIN_DELAY) {
                    current_delay = MIN_DELAY;
                }
                movement_counter = 0;
            }
            movement_counter += current_delay;
        }
        else {
            current_delay = INITIAL_DELAY;
            movement_counter = 0;
        }
        last_movement = movement;
    }

}

