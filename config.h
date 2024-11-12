#ifndef CONFIG_H
#define CONFIG_H

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"
#include "buzzer.h"
#include "us.h"

// Motor Driver (L298N)
#define MOTOR_PORT PORTD
#define MOTOR_DDR DDRD
#define MOTOR_ENABLE_A PD0  // ENA
#define MOTOR_PIN1 PD1  // IN1
#define MOTOR_PIN2 PD2  // IN2
#define MOTOR_PIN3 PD3  // IN3
#define MOTOR_PIN4 PD4  // IN4
#define MOTOR_ENABLE_B PD5  // ENB

// Ultrasonic Sensor (HC-SR04)
#define ULTRASONIC_TRIG_PORT PORTC
#define ULTRASONIC_TRIG_DDR DDRC
#define ULTRASONIC_TRIG_PIN PC0  // Trigger Pin
#define ULTRASONIC_ECHO_PORT PINC
#define ULTRASONIC_ECHO_PIN PC1  // Echo Pin

// Buzzer
#define BUZZER_PORT PORTB
#define BUZZER_DDR DDRB
#define BUZZER_PIN PB0

// IR Sensor
#define IR_SENSOR_PORT PORTC
#define IR_SENSOR_DDR DDRC
#define IR_SENSOR_PIN PC2  // IR Sensor Pin

// LCD Display (2x16)
#define LCD_DATA_PORT PORTA
#define LCD_DATA_DDR DDRA
#define LCD_CTRL_PORT PORTB
#define LCD_CTRL_DDR DDRB
#define LCD_RS PB1  // Register Select
#define LCD_RW PB2  // Read/Write
#define LCD_EN PB3  // Enable

// Function Prototypes
void init_ports(void) {
	// Motor Driver (L298N) setup
	MOTOR_DDR |= (1 << MOTOR_PIN1) | (1 << MOTOR_PIN2) | (1 << MOTOR_PIN3) | (1 << MOTOR_PIN4) |
	(1 << MOTOR_ENABLE_A) | (1 << MOTOR_ENABLE_B);

	// Ultrasonic Sensor (HC-SR04) setup
	ULTRASONIC_TRIG_DDR |= (1 << ULTRASONIC_TRIG_PIN);
	ULTRASONIC_TRIG_PORT &= ~(1 << ULTRASONIC_TRIG_PIN); // Initially low
	ULTRASONIC_TRIG_DDR &= ~(1 << ULTRASONIC_ECHO_PIN);  // Echo pin as input

	// Buzzer setupus_init
	BUZZER_DDR |= (1 << BUZZER_PIN);

	// IR Sensor setup
	IR_SENSOR_DDR &= ~(1 << IR_SENSOR_PIN);

	// LCD setup
	LCD_Init();
}

void start_system(void) {
	// Display startup message
	LCD_Clear();
	LCD_String("ADAS Starting...");

	// Ensure motors are off
	//motor_control(0);

	// Initialize sensors
	//ultrasonic_read();
	//ir_sensor_check();

	// Signal system ready
	buzzer_control(1);
	_delay_ms(1000);
	buzzer_control(0);
}

void stop_system(void) {
	// Stop motor
	//motor_control(0);

	// Display stop message
	LCD_Clear();
	LCD_String("System Stopped");

	// Signal system stop
	buzzer_control(1);
	_delay_ms(500);
	buzzer_control(0);
	
}


void idle_mode(void);
void low_speed_mode(void);
void high_speed_mode(void);

void motor_control(uint8_t state);
void ultrasonic_read(void);
void ir_sensor_check(void);

#endif // CONFIG_H
