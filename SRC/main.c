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
#include "camera.h"
#include "pwm.h"

void initialize();
void en_interrupts();
void delay();

int main(void)
{
	// Initialize UART and PWM
	initialize();

    int CAM_VALS = 128;
    int HALF = 64;
    int MARGIN;
    uint16_t* ptr;



    while(1){
        // Read Trace Camera
        ptr = Camera_Main();
        //print to verify?
        // ptr_size = sizeof(ptr)/sizeof(ptr[0]);

        // Make double

        // Normalize Trace
        // Median
        // double no_peaks[128];
        // median_filter(ptr, );

        // Weighted averaging
        // weight_kern = [1 2 4 2 1];
        // double clean_sig[128];
        // convolve_avg(no_peaks, weight_kern, clean_sig, ptr_size, sizeof(weight_kern)/sizeof(weight_kern[0])););

        // Find Left and Right edge
        // (Assuming left to right read)
        // Left is max of derivative Right is min
        // deriv_kern = [-1 0 1];
        // double deriv[128];
        // convolve(clean_sig, deriv_kern, deriv, ptr_size, sizeof(deriv_kern)/sizeof(deriv_kern[0]));
        // left_idx = max(deriv);
        // right_idx = min(deriv);

        // FOR LATER: Base off previous state? And use difference to determine how hard to turn
        // Could be function
        // Distance from left and right to middle
        // Figure out margin use margin to determine how much to turn
        // delta_r = right_idx - HALF;
        // delta_l = HALF - left_idx;
        // if (delta_r > delta_l + MARGIN)
        // {
        //     //steer right based off servo
        //     SetMotorDutyCycle(40, 10000, 1);		// C4 Active
        //     SetServoDutyCycle(8);
        // }
        // else if(delta_l > delta_r + MARGIN)
        // {
        //     //steer left based off servo
        //     SetMotorDutyCycle(40, 10000, 1);		// C4 Active
        //     SetServoDutyCycle(8);
        // }
        // else
        // {
        //     SetMotorDutyCycle(40, 10000, 1);		// C4 Active
        //     SetServoDutyCycle(8);
        // }

        // EXTRA: Make more gradual turn by using floats

    }

	return 0;
}


// median_filter
// median_filter()
// A three point median filter


// convolve
// convolve(constdouble &x,constdouble &h, double &y, constint xSize,  constint hSize)
// {
    // // size of y (output vector) needs to be the same size as f (input vector)
    // // start hSizein so we don’t index less than 0  (boundary condition)
    // for (int i=(hSize-1);i <  xSize; i++)  //need to add end boundary condition
    // {
        // double sum = 0.0;
        // for (int j=hSize; j >=0; j--)
        // {
            // sum += h[j] * x[i-j];   //inner dot product
        // }
        // y[i] = sum;
    // }
// }

// // convolve_avg
// convolve_avg(constdouble &x,constdouble &h, double &y, constint xSize,  constint hSize)
// {
    // // size of y (output vector) needs to be the same size as f (input vector)
    // // start hSizein so we don’t index less than 0  (boundary condition)
    // for (int i=(hSize-1);i <  xSize; i++)  //need to add end boundary condition
    // {
        // double sum = 0.0;
        // for (int j=hSize; j >=0; j--)
        // {
            // sum += h[j] * x[i-j];   //inner dot product
        // }
        // y[i] = sum * (1/10);
    // }
// }

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
    
    // Initialize Camera
    init_GPIO(); // For CLK and SI output on GPIO
    init_FTM2(); // To generate CLK, SI, and trigger ADC
    init_ADC0();
    init_PIT(); // To trigger camera read based on integration time

	// Initialize the FlexTimer
	InitPWM();
}
