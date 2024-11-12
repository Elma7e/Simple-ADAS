#ifndef US_H_
#define US_H_

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

// Ultrasonic Sensor (HC-SR04) Definitions
#define US_TRIG_PORT PORTC
#define US_TRIG_DDR DDRC
#define US_TRIG_PIN PC0

#define US_ECHO_PORT PINC
#define US_ECHO_DDR DDRC
#define US_ECHO_PIN PC1

// Function Prototypes
void us_init(void);
uint8_t us_measure(void);
void us_display_value(uint8_t distance);

// Function Implementations
void us_init(void) {
	// Set Trigger Pin as output
	US_TRIG_DDR |= (1 << US_TRIG_PIN);
	// Set Echo Pin as input
	US_ECHO_DDR &= ~(1 << US_ECHO_PIN);
}

uint8_t us_measure(void) {
	uint32_t duration = 0;
	uint8_t distance = 0;

	// Send a 10us pulse to the Trigger Pin
	US_TRIG_PORT |= (1 << US_TRIG_PIN);
	_delay_us(10);
	US_TRIG_PORT &= ~(1 << US_TRIG_PIN);

	// Wait for the Echo Pin to go high
	while (!(US_ECHO_PORT & (1 << US_ECHO_PIN)));

	// Measure the time the Echo Pin stays high
	while (US_ECHO_PORT & (1 << US_ECHO_PIN)) {
		_delay_us(1);
		duration++;
	}

	// Calculate distance in centimeters (Speed of sound = 343 m/s)
	distance = (duration / 58); // Convert to centimeters

	return distance;
}

void us_display_value(uint8_t distance) {
	char buffer[16];
	LCD_Clear();
	sprintf(buffer, "US Value: %d cm", distance);
	LCD_String(buffer);
}

#endif /* US_H_ */
