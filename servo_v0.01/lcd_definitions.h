/*
 * All definitions added to lcd_definitions.h will override the default definitions from lcd.h
 * This file is included by adding -D_LCD_DEFINITIONS_FILE to the CDEFS section in the Makefile
 */
// AVR board by Sebesta
/*
#define LCD_PORT         PORTA      // Port for LCD lines
#define LCD_DATA0_PIN    4          // Pin for 4bit data bit 0
#define LCD_DATA1_PIN    5          // Pin for 4bit data bit 1
#define LCD_DATA2_PIN    6          // Pin for 4bit data bit 2
#define LCD_DATA3_PIN    7          // Pin for 4bit data bit 3
#define LCD_RS_PIN       1          // Pin for RS line
#define LCD_RW_PIN       2          // Pin for RW line
#define LCD_E_PIN        3          // Pin for Enable line
*/
#define LCD_LINES        2         // Number of visible lines of the display
#define LCD_DISP_LENGTH  16         // Visible characters per line of the display
#define LCD_LINE_LENGTH  0x20      // Internal line length of the display
#define LCD_START_LINE1  0x00     // DDRAM address of first char of line 1
#define LCD_START_LINE2  0x40      // DDRAM address of first char of line 2


// Arduino Uno LCD Keypad Shield
#include <avr/io.h>

#define LCD_PORT         PORTD      // Port for LCD lines
#define LCD_DATA0_PIN    4          // Pin for 4bit data bit 0
#define LCD_DATA1_PIN    5          // Pin for 4bit data bit 1
#define LCD_DATA2_PIN    6          // Pin for 4bit data bit 2
#define LCD_DATA3_PIN    7          // Pin for 4bit data bit 3
#define LCD_RS_PORT      PORTB      // Port for RS line
#define LCD_RS_PIN       0          // Pin for RS line
// R/W pin is connected to GND
#define LCD_RW_PORT      PORTB      // Port for RW line
#define LCD_RW_PIN       5          // Pin for RW line
#define LCD_E_PORT       PORTC      // Port for Enable line
#define LCD_E_PIN        0          // Pin for Enable line

