#include "uart.h"

#ifndef F_CPU
    #define F_CPU 16000000UL
#endif

void uart_init(void)
{
    UBRR0 = F_CPU/16/9600-1;
    UCSR0A = 1<<U2X0;                  // double speed mode
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);  // enable receiver and transmitter
    UCSR0C = 3<<UCSZ00;                // 8n1
    UCSR0B |= 1<<RXCIE0;               // receiver interrupt
}

void uart_putc(char data)
{
    while ( !( UCSR0A & (1<<UDRE0)) ); // Wait for empty transmit buffer
    UDR0 = data;                       // Put data into buffer, sends the data
}

void uart_puts(char str[])
{
    for (int i=0; str[i]; i++)
        uart_putc(str[i]);
}

char uart_getc(void)
{
    return UDR0;
}
