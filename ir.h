#ifndef IR_H_
#define IR_H_

#include <avr/io.h>
#include "lcd.h"

// IR Sensor Definitions
#define IR_SENSOR_RIGHT_PORT PINC
#define IR_SENSOR_RIGHT_DDR DDRC
#define IR_SENSOR_RIGHT_PIN PC2

#define IR_SENSOR_LEFT_PORT PINC
#define IR_SENSOR_LEFT_DDR DDRC
#define IR_SENSOR_LEFT_PIN PC3

// Function Prototypes
void ir_init(void);
void ir_check(void);

// Function Implementations
void ir_init(void) {
	// Set IR sensor pins as input
	IR_SENSOR_RIGHT_DDR &= ~(1 << IR_SENSOR_RIGHT_PIN);
	IR_SENSOR_LEFT_DDR &= ~(1 << IR_SENSOR_LEFT_PIN);
}

void ir_check(void) {
	uint8_t right_detected = !(IR_SENSOR_RIGHT_PORT & (1 << IR_SENSOR_RIGHT_PIN));
	uint8_t left_detected = !(IR_SENSOR_LEFT_PORT & (1 << IR_SENSOR_LEFT_PIN));

	if (right_detected && left_detected) {
		// Both sensors detect something, clear the second row
		LCD_Set_Cursor(2, 1);
		LCD_String("                "); // Clear the second row
		} else if (right_detected) {
		// Right sensor detects something, go left
		LCD_Set_Cursor(2, 1);
		LCD_String("going right --->");
		} else if (left_detected) {
		// Left sensor detects something, go right
		LCD_Set_Cursor(2, 1);
		LCD_String("<--- going left ");
		} else {
		// No sensor detects anything, clear the second row
		LCD_Set_Cursor(2, 1);
		LCD_String("                "); // Clear the second row
	}
}

#endif /* IR_H_ */
