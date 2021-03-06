/*
 * servo_v0.01.c
 *
 * Created: 28.11.2017 16:41:25
 * Author : fkla
 */

/* headliners initialization */
#define F_CPU 16000000UL            // fCPU 16 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
#include <stdlib.h>
#include "uart.h"

extern int16_t transfer(uint16_t);

/* global variables */
char buffer[9];                       // buffer for ITOA function
volatile uint8_t step = 5;            // preset step size

/*********************************************************************************
*
* Function Name : set_duty
* Description    : set correct OCR1B registers bit, which define pulse width time (tON) PWM waveform
*
*********************************************************************************/
#define set_duty(x) (OCR1B = (uint16_t)(x))

/*********************************************************************************
*
* Function Name : show
* Description    : this function print the pulse width d = tON  on the display
*
*********************************************************************************/
void show()
{
	uint16_t d = OCR1B;
    itoa(d,buffer,10);          // conversion numerical system from d to buffer. Third parameter define desired numerical system.
    lcd_gotoxy(6,0);            // go to line 1 and 0 character
    lcd_puts(buffer);           // and print content of buffer (pulse width)
    lcd_puts("us ");            // print units

    lcd_gotoxy(6,1);
    itoa(transfer(d)/10,buffer,10);
    lcd_puts(buffer);
    lcd_putc(',');
    itoa(abs(transfer(d)) % 10,buffer,10);
    lcd_puts(buffer);
    lcd_putc(0b11011111);
    lcd_puts(" ");

    /*print step size */
    if (step == 100)                //first goto xy based on size of value step size
        lcd_gotoxy(13,1);
    else if ((step == 50) | (step == 20) | (step == 10)){
        lcd_puts("   ");
        lcd_gotoxy(14,1);}
    else{
        lcd_puts("    ");
        lcd_gotoxy(15,1);}
    itoa(step,buffer,10);            // convert again, now step, save to buffer, decimal num. system
    lcd_puts(buffer);                // print step and two spaces
    lcd_puts("  ");
}

/*********************************************************************************
*
* Function Name : setup
* Description    : main setup initialization function, which set control
*                  registers for PORTs, interrupts, display initialization
*
*********************************************************************************/

void setup()
{
    // Setup for used PORTS
    DDRB |= 1<<2;                   // set bit PB2 as output port for servo
    DDRB &= ~(1<<1);                // set bit PB1 as input for rotary encoder
    DDRD &= ~(1<<2);                // set bit PD2 as input for rotary encoder
    DDRD &= ~(1<<3);                // set bit PD3 as input step size button
    DDRB &= ~(1<<3);                // set bit PB3 as input reset pulse
    PORTD |= 1<<2;                  // activate internal pull-up resistors on bit PD2
    PORTD |= 1<<3;                  // activate internal pull-up resistors on bit PD3
    PORTB |= 1<<1;                  // activate internal pull-up resistors on bit PB1
    PORTB |= 1<<3;                  // activate internal pull-up resistors on bit PB3
	
	OCR1B = 1500;
	// set period T = 20 ms, f = 50 Hz. This is define TOP of counter.
	OCR1A = 20000;

    uart_init();

    /* MOD 9 Phase correct */
    TCCR1A = (1<<WGM10) | (1<<COM1B1);
    TCCR1B = (1<<WGM13) | (1<<CS11);

    //setup interrupt OCIEA
    TIMSK1 |= 1<<OCIE1A;

    // setup interrupt for rotary encoder
    EICRA |= 1<<ISC01;               // the falling edge on INT0 (PD2) generate interrupt request
    EIMSK |= 1<<INT0;                // enable external interrupt request from INT0 (PD2)

    // setup interupt for step size button
    EICRA |= 1<<ISC11;               // the falling edge on INT1 (PD3) generate interrupt request
    EIMSK |= 1<<INT1;                // enable external interrupt request from INT1 (PD3)

    sei();                           // enable all interrupts

    // display initialization
    lcd_init(LCD_DISP_ON);           // display on

    /* Add new symbol  up and down */
    char new_symbol[] = {
        0b00100,
        0b01110,
        0b10101,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b10101,
        0b01110,
        0b00100
    };

    lcd_command(1<<LCD_CGRAM);
    for (uint8_t i = 0; i < 16; i++)
        lcd_data(new_symbol[i]);

    /* print introduction */
    lcd_clrscr();                   // clear display
    lcd_gotoxy(0,0);                // go to zero line and zero character
    lcd_puts("Width");              // print on display text
    lcd_gotoxy(0,1);
    lcd_puts("Angle");
    lcd_gotoxy(13,0);
    lcd_puts("St");
    lcd_putc(1);
}


/*********************************************************************************
*
* Function Name : main
* Description    : main function
*
*********************************************************************************/
int main(void)
{
    setup();                        // system initialization
    //uart_puts("LOOP\n");
    while (1)
    {
        show();                    // print the pulse width d = tON  on the display;
    }
    
    return 0;
}

/*********************************************************************************
*
* Function Name : INT0_vect
* Description    : interrupt from INT1 service rotary encoder on PB1 (up/dw) and PD2 (INT0)
*
*********************************************************************************/
ISR(INT0_vect)
{
    cli();
	volatile  uint16_t d = OCR1B;
	//uart_puts("INT0 ");
    /* ternary operation, which fill variable s value = step*either -1, if PINB1 is in HI, or +1, if PINB1 is in LOW */
    int16_t s = step * ((PINB & (1<<1)) ? -1 : 1);

    /* if new d after do this routine is in range from 615 to 2100 -> do routine. s > or < 0 shows if new value will increment or decrement  */
    if (( (d+step) <= 2100 && s > 0 ) || ( (d-step) >= 615 && s < 0 ))
        d += s;                    // routine: fill with new value d+s
    else
        d = (s > 0) ? 2100 : 615;  // else in range from 615 to 2100 set fixed value
	
	
	OCR1B = d;
	
	sei();
}

/*********************************************************************************
*
* Function Name : INT1_vect
* Description    : interrupt from INT1 service change step size on PD3 (INT1)
*
*********************************************************************************/
ISR(INT1_vect)
{
    cli();
	static uint8_t status = 0;                // preset status of step size
    //uart_puts("set_step button INT1 ");
    switch(status)
    {
        case 0:
            step = 5;
            status=1;
            break;
        case 1:
            step = 10;
            status=2;
            break;
        case 2:
            step = 20;
            status=3;
            break;
        case 3:
            step = 50;
            status=4;
            break;
        default:
            step = 100;
            status = 0;
    }
	sei();
}

ISR(TIMER1_COMPA_vect)
{
    cli();
	//uart_puts("TIMER1 COMPA ");
    if (bit_is_clear(PINB,3))
	{
		//uart_puts("TIMER1 COMPA ");
		OCR1B = 1500;
	}
	sei();
}
