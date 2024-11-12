#ifndef MOTOR_H_
#define MOTOR_H_

#include <avr/io.h>
#include <util/delay.h>

// Motor Driver (L298N) Definitions
#define MOTOR_PORT PORTD
#define MOTOR_DDR DDRD
#define MOTOR_ENABLE_A PD0  // ENA
#define MOTOR_ENABLE_B PD5  // ENB
#define MOTOR_PIN1 PD1  // IN1
#define MOTOR_PIN2 PD2  // IN2
#define MOTOR_PIN3 PD3  // IN3
#define MOTOR_PIN4 PD4  // IN4

// Function Prototypes
void motor_init(void);
void motor_control(uint8_t speed);
void motor_stop(void);

// Function Implementations
void motor_init(void) {
	// Set motor pins as output
	MOTOR_DDR |= (1 << MOTOR_PIN1) | (1 << MOTOR_PIN2) | (1 << MOTOR_PIN3) | (1 << MOTOR_PIN4);
	MOTOR_DDR |= (1 << MOTOR_ENABLE_A) | (1 << MOTOR_ENABLE_B);

	// Initialize motor in stop condition
	motor_stop();
}

void motor_control(uint8_t distance) {
	uint8_t speed;
	if (distance > 10) {
		// Increase speed proportionally to distance
		speed = (distance > 100) ? 255 : (distance * 2.55); // Cap speed at 255
		} else {
		// Decrease speed quickly to stop
		speed = 0;
	}

	// Control motor speed (PWM)
	if (speed > 0) {
		MOTOR_PORT |= (1 << MOTOR_ENABLE_A) | (1 << MOTOR_ENABLE_B);
		MOTOR_PORT |= (1 << MOTOR_PIN1) | (1 << MOTOR_PIN3); // Forward direction
		MOTOR_PORT &= ~((1 << MOTOR_PIN2) | (1 << MOTOR_PIN4)); // Ensure reverse is off
		} else {
		motor_stop();
	}
}

void motor_stop(void) {
	// Stop the motor
	MOTOR_PORT &= ~((1 << MOTOR_ENABLE_A) | (1 << MOTOR_ENABLE_B));
	MOTOR_PORT &= ~((1 << MOTOR_PIN1) | (1 << MOTOR_PIN2) | (1 << MOTOR_PIN3) | (1 << MOTOR_PIN4));
}

#endif /* MOTOR_H_ */
