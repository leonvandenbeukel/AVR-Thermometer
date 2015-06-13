/*
*
*  7segment.c
*
*  Created: 11-5-2015 21:01:59
*  Author: Leon van den Beukel
* 
*/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include "ds18b20/ds18b20.h"

#define NEGATIVE 11
#define CLEAR 12
#define TEMPNOTSET 88.8

#define DEBOUNCE_TIME 350
#define INTERVAL 30
#define INTERVAL_DELAY_MS 100
volatile uint16_t currentInterval = 0;

volatile uint8_t thousand, hundred, ten, one;
volatile uint8_t position = 0;
volatile uint8_t negative = 0;
volatile uint8_t activesensor = DS18B20_DQ_SENSOR1;

volatile double minimumTemperatureSensor1 = TEMPNOTSET;
volatile double maximumTemperatureSensor1 = TEMPNOTSET;
volatile double minimumTemperatureSensor2 = TEMPNOTSET;
volatile double maximumTemperatureSensor2 = TEMPNOTSET;

void setTemperature(double);
void clearSegments();
void setActiveSensorLed();
void getTemperature();
uint8_t debounce(uint8_t);
void flashLed(uint8_t);

volatile typedef enum {
	currentTemperature,
	minimumTemperature,
	maximumTemperature	
} displayModeType;

volatile displayModeType displayMode = currentTemperature;

const int8_t numbers [13] =
{
	0b11000000, // 0
	0b11111001, // 1
	0b10100100, // 2
	0b10110000, // 3
	0b10011001, // 4
	0b10010010, // 5
	0b10000010, // 6
	0b11111000, // 7
	0b10000000, // 8
	0b10010000, // 9
	0b01111111, // DP
	0b10111111, // Negative sign
	0b11111111  // Clear
};

void digit(uint8_t number, uint8_t pin)
{	
	PORTD = numbers[number];
	
	// If PC2/ten then enable the dot also
	if(pin == PC2){
		PORTD &= ~(1<<PIND7);
	}
	
	// Transistor
	PORTC &= ~(1 << pin);
}

// Interrupt service routine for TIMER0
ISR(TIMER0_COMPA_vect)
{
	PORTC |= (1<<PC0) | (1<<PC1) | (1<<PC2) | (1<<PC3); // Disable all pnp's
	
	switch (position){
		case 0: 
			//digit (thousand, PC0);
			if (negative) {
				digit(NEGATIVE, PC0);	// Negative sign
			} else {
				digit(CLEAR, PC0);		// Clear				
			}
		break;
		case 1: 
			if (hundred == 0){
				digit(CLEAR, PC1);		// Clear	
			} else {
				digit (hundred, PC1);				
			}
		break;
		case 2: digit (ten, PC2);
		break;
		case 3: digit (one, PC3);
		break;
	}
	
	position++;
	
	if (position==4){
		position=0;
	}
}

void display_number (volatile uint16_t number){
	thousand = number / 1000;
	number   = number % 1000;
	
	hundred  = number / 100;
	number   = number % 100;
	
	ten      = number / 10;
	number   = number %10;
	
	one      = number;
}

uint8_t debounce(uint8_t pin) {
	
	if ((PINB & (1<<pin))==0){		// Check if button is pressed
		
		_delay_us(DEBOUNCE_TIME);
		
		if ((PINB & (1<<pin))==0){	// Check if button is still pressed
			return 1;	
		}		
	}	
	
	return 0;	
}

// Clears all segments
void clearSegments(){
	thousand = CLEAR;
	hundred = CLEAR;
	ten = CLEAR;
	one = CLEAR;
	digit(CLEAR, PC0);
	digit(CLEAR, PC1);
	digit(CLEAR, PC2);
	digit(CLEAR, PC3);
}

// There are two leds that indicate which sensor is active
void setActiveSensorLed() {
	if (activesensor == DS18B20_DQ_SENSOR1) {
		PORTC |= (1<<PC4);
		PORTC &= ~(1<<PC5);
	} else {
		PORTC |= (1<<PC5);
		PORTC &= ~(1<<PC4);					
	}
}
	
// Get's the temperature from the currently active sensor
void getTemperature() {	
	
	setTemperature( ds18b20_gettemp(activesensor) );
		
}

void setTemperature(double temperature) {
	// Do some corrections
	temperature *= 10;
	if (temperature < 0){
		temperature *= -1;
		negative = 1;
	} else {
		negative = 0;
	}
	
	display_number((uint16_t)temperature);
}

void flashLed(uint8_t led){
	// Flash the led on/off
	for (uint8_t i=0; i<12; i++){
		PORTC ^= (1<<led);
		_delay_ms(500);
	}
}

// Main loop
int main(void)
{
	uint8_t buttonWasPressed = 0;
	
	// Disable clock division (prescaler)
	clock_prescale_set(clock_div_1);
	
	// Settings for driving the 7 segment display
	DDRD  |= 0b11111111;	// Make PD0-7 an output (segments)
	DDRC  |= 0b00111111; 	// Make PC0-3 an output	for driver transistors and PC4-PC5 output for current sensor 
	PORTC |= 0xf;			// Make PC0-PC3 high to disable the PNP transistors
	
	// Settings for input buttons
 	DDRB &= ~(1<<PINB2);	// Make Port D 2,3,4 an input for the buttons (set to 0)
 	DDRB &= ~(1<<PINB3);
 	DDRB &= ~(1<<PINB4);
	PORTB |= (1<<PINB2) | (1<<PINB3) | (1<<PINB4);	// Enable pull-up resistors
	
	// Settings for timer
	TCCR0A = (1<<WGM01); 							// Set CTC Bit
	OCR0A  = (uint16_t)(F_CPU/1024.0 * 0.005);		// Reference for 0,005s: clock = 8MHz/1024 = 7812,5 pulses per second
	TIMSK0 = (1<<OCIE0A);							// Interrupt when timer reference is reached
 	TCCR0B = (1<<CS02) | (1<<CS00);					// Start at 1024 prescalar
 	
	sei();											// Enable global interrupt 	 	
	
	setActiveSensorLed();	

	while(1)
	{		
		if (debounce(PINB2)) {								// Debounced button press
			if (buttonWasPressed == 0) {					
				// Switch the active sensor
				if (activesensor == DS18B20_DQ_SENSOR1) {
					activesensor = DS18B20_DQ_SENSOR2;
					} else {
					activesensor = DS18B20_DQ_SENSOR1;
				}
				
				clearSegments();
				setActiveSensorLed();
				getTemperature(); 
				
				buttonWasPressed = 1;
			}
       
	    } else if (debounce(PINB3)) {
			
			if (buttonWasPressed == 0) {			
				// Change min/max view
				switch (displayMode)
				{
					case currentTemperature:
						displayMode = minimumTemperature;
						clearSegments();
						if (activesensor == DS18B20_DQ_SENSOR1) {
							setTemperature(minimumTemperatureSensor1);	
						} else {
							setTemperature(minimumTemperatureSensor2);
						}
						break;
					case minimumTemperature:
						displayMode = maximumTemperature;
						clearSegments();
						if (activesensor == DS18B20_DQ_SENSOR1){
							setTemperature(maximumTemperatureSensor1);
						} else {
							setTemperature(maximumTemperatureSensor2);
						}						
						break;
					case maximumTemperature:
						displayMode = currentTemperature;
						clearSegments();
						getTemperature();
						break;
					default:
						break;
				}
			
				buttonWasPressed = 1;
			}
			
		} else if (debounce(PINB4)) {
			
			// Reset button was pressed
			if (buttonWasPressed == 0) {
				
				if (activesensor == DS18B20_DQ_SENSOR1) {
					// Reset values
					minimumTemperatureSensor1 = TEMPNOTSET;
					maximumTemperatureSensor1 = TEMPNOTSET;
					
					// Flash led
					flashLed(PC4);
				} else {
					// Reset values
					minimumTemperatureSensor2 = TEMPNOTSET;
					maximumTemperatureSensor2 = TEMPNOTSET;
					
					// Flash led
					flashLed(PC5);
				}
				  
				setActiveSensorLed();							
				buttonWasPressed = 1;	   
			}
									
		} else {
			// Button is not pressed and therefore update the status 											
			buttonWasPressed = 0;							
			
			if (displayMode == currentTemperature && currentInterval >= INTERVAL){	
				// Get next temperature values
				getTemperature();							
				
				// Reset the interval
				currentInterval = 0;
				
				// Store the min and max for both sensors
				double t1 = ds18b20_gettemp(DS18B20_DQ_SENSOR1);
				if (t1 < minimumTemperatureSensor1 || minimumTemperatureSensor1 == TEMPNOTSET){
					minimumTemperatureSensor1 = t1;
				} else if (t1 > maximumTemperatureSensor1 || maximumTemperatureSensor1 == TEMPNOTSET){
					maximumTemperatureSensor1 = t1;
				}
				
				// Store the min and max for both sensors
				t1 = ds18b20_gettemp(DS18B20_DQ_SENSOR2);
				if (t1 < minimumTemperatureSensor2 || minimumTemperatureSensor2 == TEMPNOTSET){
					minimumTemperatureSensor2 = t1;
				} else if (t1 > maximumTemperatureSensor2 || maximumTemperatureSensor2 == TEMPNOTSET){
					maximumTemperatureSensor2 = t1;
				}
				
			} else {
				currentInterval++;
				_delay_ms(INTERVAL_DELAY_MS);
			}			
		}
	}
	
	return 0;
}