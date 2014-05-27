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

//#define DEBUG 1

#define BUTTONCOUNT 1

// The thread pitch is 1.75mm per revolution (M12)
// 3200 steps are 1 revolution

// Limit for tilting
#define TILT_LIMIT 80000

// Limit for height
#define HEIGHT_LIMIT 190000

// ADC value when the autofocus switch is pressed.
// Significantly higher values mean broken cable.
#define CLOSED_VALUE 671

// ADC value when the autofocus switch is not pressed.
// Significyntly lower values mean short circuit.
#define OPEN_VALUE 316

// Maximum difference between ADC value and OPEN_VALUE for which the autofocus switch is considered open.
// Values outside are considered proof of pressed switch, so moving up will not work.
#define INTERVAL_SIZE 23

// Lower ADC values will be consideres proof for short circuit.
#define SHORT_CIRCUIT_VALUE 286

// Higher ADC values will be consideres proof for broken cable.
#define CABLE_BROKEN_VALUE 770

// Delay between two steps in micro seconds when starting to move.
#define INITIAL_DELAY 50

// Shortest delay between two steps allowed (i.e. fastest movement).
#define MIN_DELAY 1

// Distance between a hit of the focus switch and the point of focus.
#define FOCUS_DISTANCE 21628

// true if the USART-Receive-Buffer was completely read by the main loop. Set to false when buffer full by usart-receive-interrupt.
volatile bool buffer_read = true;

// Positions of the 3 achsis relative to the endstop. [0] has the "average position"
int32_t positions[4];

// "average position" (see above) of the autofocus switch
int32_t autofocusPosition;

#define setStep1 {PORTC |= (1<<PC1);}
#define setStep2 {PORTC |= (1<<PC3);}
#define setStep3 {PORTC |= (1<<PC5);}

#define unsetStep1 {PORTC &= ~(1<<PC1);}
#define unsetStep2 {PORTC &= ~(1<<PC3);}
#define unsetStep3 {PORTC &= ~(1<<PC5);}

#define MOTOR_1_OFFSET 0
#define MOTOR_2_OFFSET 2754
#define MOTOR_3_OFFSET 2165

#define MOTOR_MAX_OFFSET 4000


/**
 * Set the direction of a stepper driver
 *
 * @param direction the new direction. True means up.
 */
#define setDir1(dir) {\
	if (dir) PORTC |= (1<<PC0); \
	else PORTC &= ~(1<<PC0);\
	_delay_us(1);\
}
#define setDir2(dir) {\
	if (dir) PORTC |= (1<<PC2); \
	else PORTC &= ~(1<<PC2);\
	_delay_us(1);\
}
#define setDir3(dir) {\
	if (dir) PORTC |= (1<<PC4); \
	else PORTC &= ~(1<<PC4);\
	_delay_us(1);\
}

// Test if one axis reached the height limit.
#define heightLimitReached(driver) (positions[driver] > HEIGHT_LIMIT) 

// Test of any of the three axis reached the height limit
#define anyHeightLimitReached() (heightLimitReached(1) || heightLimitReached(2) || heightLimitReached(3))

int32_t max(int32_t a, int32_t b) {
	if (a>b) {
		return a;
	}
	return b;
}


/** 
 * Check if a endstop has been reached. True means yes.
 */
#define stop1 ((bool)(PINA & (1<<PA1)))
#define stop2 ((bool)(PINA & (1<<PA2)))
#define stop3 ((bool)(PINA & (1<<PA3)))

#define fault1 ((bool)(PINA & (1<<PA5)))
#define fault2 ((bool)(PINA & (1<<PA6)))
#define fault3 ((bool)(PINA & (1<<PA7)))
#define sleeping (!(bool)(PIND & (1<<PD7)))
#define sleep()  {PORTD &= ~(1<<PD7); DDRD  |= (1<<PD7);}
#define wakeup() {DDRD  &= ~(1<<PD7); PORTD |= (1<<PD7);}

/*!
 * Check if any of the endstops has been reached. True means yes.
 */
#define anyStopReached() (stop1 || stop2 || stop3)

#define stepUp1() { \
	if (sleeping) {\
		wakeup();\
		_delay_ms(100);\
	}\
	if (!sleeping) {\
		positions[1]++;\
		setDir1(true);\
		setStep1;\
		_delay_us(2);\
		unsetStep1;\
	}\
}

#define stepUp2() {\
	if (sleeping) {\
		wakeup();\
		_delay_ms(100);\
	}\
	if (!sleeping) {\
		positions[2]++;\
		setDir2(true);\
		setStep2;\
		_delay_us(2);\
		unsetStep2;\
	}\
}

#define stepUp3() {\
	if (sleeping) {\
		wakeup();\
		_delay_ms(100);\
	}\
	if (!sleeping) {\
		positions[3]++;\
		setDir3(true);\
		setStep3;\
		_delay_us(2);\
		unsetStep3;\
	}\
}

#define stepDown1() { \
	if (sleeping) {\
		wakeup();\
		_delay_ms(100);\
	}\
	if (!sleeping) {\
		positions[1]--;\
		setDir1(false);\
		setStep1;\
		_delay_us(2);\
		unsetStep1;\
	}\
}

#define stepDown2() {\
	if (sleeping) {\
		wakeup();\
		_delay_ms(100);\
	}\
	if (!sleeping) {\
		positions[2]--;\
		setDir2(false);\
		setStep2;\
		_delay_us(2);\
		unsetStep2;\
	}\
}

#define stepDown3() {\
	if (sleeping) {\
		wakeup();\
		_delay_ms(100);\
	}\
	if (!sleeping) {\
		positions[3]--;\
		setDir3(false);\
		setStep3;\
		_delay_us(2);\
		unsetStep3;\
	}\
}

#define stepAllUp()   {positions[0]++; stepUp1();   stepUp2();   stepUp3();}
#define stepAllDown() {positions[0]--; stepDown1(); stepDown2(); stepDown3();}


// Execute one step up for each motor


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

	// Set AREF as reference
	ADMUX = 0;

	// Enable ADC, set free running, enable interrupt, set prescaler to 1/128
	//ADCSRA = (1<<ADEN) | (1<<ADSC);
	//ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

/*!
 * Start a conversion.
 */
#define adc_start_conversion() {ADCSRA |= (1<<ADSC);}          // single conversion

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
#define gotoEndstops() {\
	if (!stop1) {\
		stepDown1();\
	}\
	else {\
		positions[1] = -MOTOR_1_OFFSET;\
	}\
	if (!stop2) {\
		stepDown2();\
	}\
	else {\
		positions[2] = -MOTOR_2_OFFSET;\
	}\
	if (!stop3) {\
		stepDown3();\
	}\
	else {\
		positions[3] = -MOTOR_3_OFFSET;\
	}\
}

// True if user pressed button for moving up.
#define buttonUp() (!(bool)(PINB & (1<<1)))

// True if user pressed button for moving down.
#define buttonDown() (!(bool)(PINB & (1<<3)))

// True if user pressed button for tilting front down.
#define buttonTiltFrontDown() (!(bool)(PINB & (1<<4)))

// True if user pressed button for tilting front up.
#define buttonTiltFrontUp() (!(bool)(PINB & (1<<0)))

// True if user pressed button for leveling.
#define buttonLevel() (!(bool)(PINB & (1<<2)))

// True if the user wants to got to the autofocus position
#define buttonFocus() (!(bool)(PIND & (1<<2)))

// True if user wants to do an emergency movement
#define buttonEmergency() (!(bool)(PIND & 1<<3))

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
#define myDelayUs(length) {uint16_t _myDelayUs_tmp = length; while (--_myDelayUs_tmp) {_delay_us(1);}}

void initialize() {
	DDRC |= (1<<DDC0); // DIR1
	DDRC |= (1<<DDC1); // STEP1

	DDRC |= (1<<DDC2); // DIR2
	DDRC |= (1<<DDC3); // STEP2
	
	DDRC |= (1<<DDC4); // DIR3
	DDRC |= (1<<DDC5); // STEP3
	
	// Joystick
	DDRB &= 0;
	PORTB |= 0xff;

	// Buttons, LEDs and stuff
	DDRD &= 0;
	PORTD |= 0xff;

	// Lower endstops and autofocus-endstop. No pullup for autofocus endstop (PA0)
	DDRA &= 0;
	PORTA |= 0b11111110;

	// Set ADC used for switch as input and disable pullup
	DDRA &= ~(1<<PA0);
	PORTA &= ~(1<<PA0);

}

int main() {
	initialize();
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);

	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);


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

	wdt_enable(WDTO_2S);

/*
	for(;;i++) {
		uart_bin(PINB);
		uart_puts(" ");
		uart_bin(PIND);
		uart_puts(" ");
		uart_hex16(i);
		uart_puts("\r\n");
		_delay_ms(1000);
		wdt_reset();
	}
*/

	uart_puts("Starting up\r\n"); 
	
	for (i = 0; i < 4; i++) {
		positions[i] = 0;
	}

	bool buttonLastTime = false;

	// Count the number of cycles the autofocus is stuck in mid-state
	uint8_t mid_state_counter = 0;

	// Count the number of successive movements in the same direction before increasing speed
	uint8_t movement_counter = 0;

	uint8_t current_delay = INITIAL_DELAY;

	enum movement{NONE, UP, DOWN, TILT_DOWN, TILT_UP, LEVEL} last_movement, movement;
	last_movement = NONE;
	movement = NONE;

	// Reset autofocus position
	autofocusPosition = 0;

	/*
	   while(1) {  
	   adc_start_conversion(7);
	   const uint16_t autofocus_result = adc_wait();
	   uart_hex16(autofocus_result);
	   wdt_reset();
	   uart_puts("\r\n");
	   }
	// */

	uint8_t autofocus_hit_counter = 0;

	movement_counter = 0;
	current_delay = INITIAL_DELAY;
	
	initialize();
	adcInit();
	
	adc_start_conversion();
	uint16_t autofocus_result = adc_wait();

	uint16_t no_movement_counter = 0;

	for(i = 0;;i++){
		wdt_reset();
		/*
		for (i=0; i < current_delay; i++) {
			_delay_us(1);
		}
		// */
		#ifdef DEBUG
		uart_hex16(current_delay);
		uart_puts("\r\n");
		#endif
		// True if autofocus endstop not yet hit.
		const bool autofocus_switch_clear = (autofocus_result <= OPEN_VALUE + INTERVAL_SIZE && autofocus_result >= OPEN_VALUE - INTERVAL_SIZE);
		bool autofocus_clear = true;

		if (!autofocus_switch_clear) {
			if (autofocus_hit_counter > 10) {
				autofocus_clear = false;
				if (i % 1024 == 0) {
					uart_puts("Autofocus hit at: ");
					printPositions();
					autofocusPosition = positions[0];
					uart_puts("\r\n");
				}
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
			// Increment autofocus_hit_counter in order to detect series of hit autofocus.
			if (autofocus_hit_counter < 20) {
				autofocus_hit_counter++;
			}
		}
		else {
			mid_state_counter = 0;
			autofocus_hit_counter = 0;
		}
		#ifdef BUTTONCOUNT
		int8_t buttons = 0;
		if (buttonUp()) {
			buttons++;
			#ifdef DEBUG
			uart_puts("buttonUp()\r\n");
			#endif
		}
		if (buttonDown()) {
			buttons++;
			#ifdef DEBUG
			uart_puts("buttonDown()\r\n");
			#endif
		}
		if (buttonTiltFrontUp()) {
			buttons++;
			#ifdef DEBUG
			uart_puts("buttonTiltFrontUp()\r\n");
			#endif
		}
		if (buttonTiltFrontDown()) {
			buttons++;
			#ifdef DEBUG
			uart_puts("buttonTiltFrontDown()\r\n");
			#endif
		}
		if (buttonLevel()) {
			buttons++;
			#ifdef DEBUG
			uart_puts("buttonLevel()\r\n");
			#endif
		}
		if (buttonFocus()) {
			buttons++;
			#ifdef DEBUG
			uart_puts("buttonFocus()\r\n");
			#endif
		}
		if (buttonLastTime && (buttons != 1)) {
			printPositions();
			uart_puts("\r\n");
		}
		#ifdef DEBUG
		if (buttonEmergency()) {
			uart_puts("buttonEmergency()\r\n");
		}
		if (anyStopReached()) {
			uart_puts("anyStopReached()\r\n");
		}
		if (stop1) {
			uart_puts("stop1\r\n");
		}
		if (stop2) {
			uart_puts("stop2\r\n");
		}
		if (stop3) {
			uart_puts("stop3\r\n");
		}
		#endif
		#else
		int8_t buttons = 1;
		#endif
		movement = NONE;
		if (buttons == 1 && !buttonEmergency()) {
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
					stepUp3();
					stepDown1();
					movement = TILT_UP;
				}
			}
			if (buttonTiltFrontDown()) {
				if (!anyStopReached() && !anyHeightLimitReached() && (positions[1]-positions[3] < TILT_LIMIT) && autofocus_clear) {
					stepDown3();
					stepUp1();
					movement = TILT_DOWN;
				}
			}
			if (buttonLevel()) {
				if (!anyStopReached() && autofocus_clear) {
					if (positions[1] > positions[3]+1) {
						stepDown1();
						stepUp3();
						movement = LEVEL;
					}
					else if (positions[1]+1 < positions[3]) {
						stepDown3();
						stepUp1();
						movement = LEVEL;
					}
				} 
			}
			if (buttonFocus()) {
				#ifdef DEBUG
					printPositions();
				#endif
				if ((positions[0] > autofocusPosition - FOCUS_DISTANCE) && !anyStopReached()) {
					stepAllDown();
					movement = DOWN;
				}
			}
		}
		else {
			buttonLastTime = false;
		}
		/*
		if (buttons > 1) {
			uart_puts("Multiple buttons pressed\r\n");
		}
		*/
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

		// Do all the emergency stuff
		if (buttonEmergency()) {
			_delay_us(17);
			if (buttonEmergency() && buttonFocus()) {
				gotoEndstops();
			}
			if (buttonUp()) {
				if (buttonTiltFrontDown()) {
					stepUp1();
				}
				if (buttonLevel()) {
					stepUp2();
				}
				if (buttonTiltFrontUp()) {
					stepUp3();
				}
			}
			if (buttonDown()) {
				if (buttonTiltFrontDown()) {
					stepDown1();
				}
				if (buttonLevel()) {
					stepDown2();
				}
				if (buttonTiltFrontUp()) {
					stepDown3();
				}
				
			}
			if (buttonLevel()) {
				int32_t maxPosition = max(positions[1], max(positions[2], positions[3]));
				if (positions[1] < maxPosition) {
					stepUp1();
				}
				if (positions[2] < maxPosition) {
					stepUp2();
				}
				if (positions[3] < maxPosition) {
					stepUp3();
				}
			}
			
		}

		last_movement = movement;
		if (movement == NONE) {
			no_movement_counter++;
			if (no_movement_counter > 50000) {
				sleep();
			}
		}
		else {
			no_movement_counter = 0;
		}
		if(!(ADCSRA & (1<<ADSC))) {   // auf Abschluss der Konvertierung warten
			autofocus_result = ADCW;
			adc_start_conversion();
		}
	}

}
