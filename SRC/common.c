/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 *
 * Author: Seth Deane & Brian Powers
 * Created:
 * Modified:
 */
 
#include "MK64F12.h"
#include "uart.h"
#include "camera.h"
#include "pwm.h"
#include "common.h"
 
 // Char array that allows for string to be held
char print_string[100];


/* delay
* Description:
*   Waits for a delay (in milliseconds)
* 
* Parameters:
*   del - The delay in milliseconds
* 
* Returns:
*   void
*/
void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}


/* getChar
* Description:
*   get a character from the terminal
* 
* Parameters:
*   void
* 
* Returns:
*   uint8_t - character from terminal
*/
uint8_t getChar(void)
{
    /* Wait until there is space for more data in the receiver buffer*/
    while ((UART0_S1 & (1 << 5)) == 0);

    /* Return the 8-bit data from the receiver */
    return UART0_D;
}

/* putChar
* Description:
*   put a single character to the terminal
* 
* Parameters:
*   ch - character to put to the terminal
* 
* Returns:
*   void
*/
void putChar(char ch)
{
    /* Wait until transmission of previous bit is complete */
    while ((UART0_S1 & (1 << 7)) == 0);
        
    /* Send the character */
    UART0_D = ch;
}

/* put
* Description:
*   put a string to the terminal
* 
* Parameters:
*   ch - character to put to the terminal
* 
* Returns:
*   void
*/
void put(char *ptr_str)
{
    while(*ptr_str)
    putChar(*ptr_str++);
}

void putnumU(int i)
{
    char num[10];
    sprintf( num, "%d", i );
    put( num );
}



/* print_array
* Description:
*   Prints array
* 
* Parameters:
*   array - array that should be printed
*   del - The delay in milliseconds
*   length - length of the array
* 
* Returns:
*   void
*/
void print_array(uint16_t* array, int del, int length) {
    sprintf(print_string,"\n\r["); // start value
    put(print_string);
    for (int i = 0; i < length; i++) {
        sprintf(print_string,"%i ", array[i]);
        put(print_string);
    }
    sprintf(print_string,"]\n\n\r"); // end value
    put(print_string);
    delay(del);
}