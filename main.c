/*
 * ADAS.c
 *
 * Created: 8/28/2024 4:13:28 AM
 * Author : omarf
 */ 
//#define F_CPU 8000000UL  // 8 MHz clock speed
#include "config.h"
#include "lcd.h"
#include "us.h"
#include "motor.h"
#include "ir.h"
#include <avr/io.h>

#define BUTTON_PIN PD7

void init_button(void) {
	// Set PD7 as input
	DDRD &= ~(1 << BUTTON_PIN);
	// Enable internal pull-up resistor
	PORTD |= (1 << BUTTON_PIN);
}

int main(void)
{
	init_ports();
	init_button();
	us_init();
	us_init();
	motor_init();
	ir_init();
	
	uint8_t button_pressed = 0;
	uint8_t system_running = 0;
	uint8_t last_button_state = PIND & (1 << BUTTON_PIN);
	
	while (1)
	{
		uint8_t current_button_state = PIND & (1 << BUTTON_PIN);
		
		// Check for rising edge
		if (current_button_state && !last_button_state) {
			button_pressed = 1;
		}
		
		// Toggle system state on button press
		if (button_pressed) {
			if (system_running) {
				stop_system();
				system_running = 0;
			} 
			else {
				start_system();
				system_running = 1;
			}
			button_pressed = 0; // Reset button press flag
		}
		
		// If the system is running, measure distance and display
		if (system_running) {
			uint8_t distance = us_measure();
			us_display_value(distance);
			motor_control(distance);
			ir_check();
			_delay_ms(200);  // Adjust delay as needed
		}

		last_button_state = current_button_state; // Update the last state
	}
}
