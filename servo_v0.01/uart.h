/*
 * library for work with UART
 *
 * autor: wykys
 * verze: 1.0
 * datum: 2.11.2017
 */

#ifndef WYK_UART_H_INCLUDED
#define WYK_UART_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

// ========================================================
void uart_init(void);
void uart_putc(char data);
void uart_puts(char str[]);
char uart_getc(void);
// ========================================================

#ifdef __cplusplus
}
#endif // END C++
#endif // WYK_UART_H_INCLUDED
