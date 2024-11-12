/*
 * buzzer.h
 *
 * Created: 8/29/2024 3:07:01 AM
 *  Author: omarf
 */ 


#ifndef BUZZER_H_
#define BUZZER_H_

#include <avr/io.h>
#include <util/delay.h>

// Define buzzer port and pin
#define BUZZER_PORT PORTB
#define BUZZER_DDR DDRB
#define BUZZER_PIN PB0

// Buzzer states
#define BUZZER_OFF 0
#define BUZZER_ON  1

// Function prototypes
void buzzer_control(uint8_t state) {
	switch (state) {
		case BUZZER_OFF:
		BUZZER_PORT &= ~(1 << BUZZER_PIN); // Turn off the buzzer
		break;

		case BUZZER_ON:
		BUZZER_PORT |= (1 << BUZZER_PIN);  // Turn on the buzzer
		break;

		default:
		BUZZER_PORT &= ~(1 << BUZZER_PIN); // Default to off
		break;
	}
}

#endif /* BUZZER_H_ */
