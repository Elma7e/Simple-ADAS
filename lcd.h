#ifndef LCD_H_
#define LCD_H_

#include <avr/io.h>
#include <util/delay.h>


// Define LCD port and pins
#define LCD_DATA_PORT PORTA
#define LCD_CTRL_PORT PORTB
#define LCD_DATA_DDR  DDRA
#define LCD_CTRL_DDR  DDRB
#define RS PB1
#define RW PB2
#define EN PB3

// Function Prototypes and Implementations
void LCD_Command(unsigned char cmd) {
	// Send higher nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (cmd & 0xF0);
	LCD_CTRL_PORT &= ~ (1 << RS);  // RS = 0 for command
	LCD_CTRL_PORT &= ~ (1 << RW);  // RW = 0 for write
	LCD_CTRL_PORT |= (1 << EN);    // Enable pulse
	_delay_us(1);
	LCD_CTRL_PORT &= ~ (1 << EN);

	// Send lower nibble
	_delay_us(200);
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (cmd << 4);
	LCD_CTRL_PORT |= (1 << EN);
	_delay_us(1);
	LCD_CTRL_PORT &= ~ (1 << EN);
	_delay_ms(2);
}

void LCD_Char(unsigned char data) {
	// Send higher nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (data & 0xF0);
	LCD_CTRL_PORT |= (1 << RS);    // RS = 1 for data
	LCD_CTRL_PORT &= ~ (1 << RW);  // RW = 0 for write
	LCD_CTRL_PORT |= (1 << EN);    // Enable pulse
	_delay_us(1);
	LCD_CTRL_PORT &= ~ (1 << EN);

	// Send lower nibble
	_delay_us(2);
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (data << 4);
	LCD_CTRL_PORT |= (1 << EN);
	_delay_us(1);
	LCD_CTRL_PORT &= ~ (1 << EN);
	_delay_ms(2);
}

void LCD_Init(void) {
	LCD_DATA_DDR = 0xFF;           // Make data port output
	LCD_CTRL_DDR |= (1 << RS) | (1 << RW) | (1 << EN); // Make control pins output
	_delay_ms(20);                 // LCD power up delay

	LCD_Command(0x02);             // Initialize LCD in 4-bit mode
	LCD_Command(0x28);             // 2 line, 5x7 matrix
	LCD_Command(0x0C);             // Display on, cursor off
	LCD_Command(0x06);             // Increment cursor
	LCD_Command(0x01);             // Clear display
	_delay_ms(2);
}

void LCD_String(char *str) {
	while(*str) {
		LCD_Char(*str++);
		_delay_ms(1);
	}
}

void LCD_Clear(void) {
	LCD_Command(0x01);             // Clear display
	_delay_ms(2);
	LCD_Command(0x80);             // Move cursor to the beginning
}

void LCD_Set_Cursor(unsigned char row, unsigned char column) {
	unsigned char position;

	if(row == 1)
	position = 0x80 + column - 1;  // Row 1
	else if(row == 2)
	position = 0xC0 + column - 1;  // Row 2

	LCD_Command(position);
}

#endif /* LCD_H_ */
