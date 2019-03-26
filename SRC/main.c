/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 *
 * Author:
 * Created:
 * Modified:
 */

#include "../INCLUDE/MK64F12.h"
#include "../INCLUDE/camera.h"
#include "../INCLUDE/uart.h"
#include "../INCLUDE/pwm.h"

void initialize();
void en_interrupts();
void delay();

int main(void)
{
	// Initialize UART and PWM
	initialize();

    int CAM_VALS = 128;
    int* ptr;



    while(1){
        // Read Trace Camera
        ptr = camera_main();
        //print to verify?

        // Normalize Trace
        // Median

        // Weighted averaging

        // Find Left and Right edge
        // (Assuming left to right read)
        // Left is max of derivative Right is min

        // Distance from left and right to middle
        // Figure out margin use margin to determine how much to turn
        //SetMotorDutyCycle(40, 10000, 1);		// C4 Active
        //SetServoDutyCycle(8);
        // Else go straight
        //SetMotorDutyCycle(40, 10000, 1);		// C4 Active
        //SetServoDutyCycle(8);

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
