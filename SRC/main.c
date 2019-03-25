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

    int cam_vals[128];

    while(1){
        // Read Trace Camera
        cam_vals = camera_main();
        //print to verify?

        // Normalize Trace
        // Median

        // Weighted averaging

        // Find Left and Right edge
        // (Assuming left to right read)
        // Left is max of derivative Right is min

        // Distance from left and right to middle
        // Figure out margin and if we need to turn
        SetMotorDutyCycle(40, 10000, 1);		// C4 Active
        SetServoDutyCycle(8);
        // Else go straight
        SetMotorDutyCycle(40, 10000, 1);		// C4 Active
        SetServoDutyCycle(8);

        // EXTRA: Make more gradual turn by using floats

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
