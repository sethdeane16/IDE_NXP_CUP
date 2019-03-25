/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 *
 * Author:
 * Created:
 * Modified:
 */

#include "MK64F12.h"
#include "uart.h"
#include "pwm.h"

void initialize();
void en_interrupts();
void delay();

int main(void)
{
	// Initialize UART and PWM
	initialize();


	//Generate 20% duty cycle at 10kHz
	SetMotorDutyCycle(40, 10000, 1);		// C4 Active
    SetServoDutyCycle(8);
    while(1){
        for(int i=0; i<3; i++){
            SetServoDutyCycle(i+6);
            delay(10);
        }
    }

	return 0;
}


/**
 * Waits for a delay (in milliseconds)
 *
 * del - The delay in milliseconds
 */
void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}

void initialize()
{
	// Initialize UART
	uart0_init();
	uart3_init();

	// Initialize the FlexTimer
	InitPWM();
}
