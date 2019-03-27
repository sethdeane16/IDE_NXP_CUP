#ifndef  UART_H
#define  UART_H
#include  <stdint.h>
void uart0_init(void);
void uart3_init(void);
void  put(char *ptr_str );
uint8_t  getChar(void);
void  putChar(char ch);
void  putnumU(int i);
#endif  /*  ifndef  UART_H  */
