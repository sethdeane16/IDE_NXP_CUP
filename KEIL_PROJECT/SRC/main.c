/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 *
 * file:    main.c
 * Authors: Seth Deane & Brian Powers
 * Created: March 21 2019
 */

#include "MK64F12.h"
#include "uart.h"
#include "camera.h"
#include "pwm.h"
#include "common.h"
#include "main.h"

#define     OTE     128 // one twenty eight
#define     DEBUG   0

int main(void)
{
    // Initialize UART and PWM
    initialize();

    uint16_t* camera_sig;   // previously ptr

    while(1){
        // Read Trace Camera
        camera_sig = Camera_Main();
        
        // print camera signal
        if (DEBUG) {
            put("camera_sig");
            print_array_u(camera_sig, OTE);
        }
        
        /*****************************
         Normalize Trace
        *****************************/
        // step 1) Median filter
        uint16_t median_sig[OTE];
        median_filter(camera_sig, median_sig, OTE);
        
        // print median signal
        if (DEBUG) {
            put("median_sig");
            print_array_u(median_sig, OTE);
        }
        
        // step 2) Weighted average filter
        int16_t weight_fil[5] = {1,2,4,2,1};
        uint16_t weight_sig[OTE];
        convolve(median_sig, weight_fil, weight_sig, OTE, sizeof(weight_fil)/sizeof(weight_fil[0]),10);
        
        // print clean signal
        if (DEBUG) {
            put("weight_sig");
            print_array_u(weight_sig, OTE);
        }

//        // step 3) Derivative filter
//        int16_t deriv_fil[2] = {-5,5};
//        int16_t deriv_sig[OTE];
//        der_convolve(weight_sig, deriv_fil, deriv_sig, OTE, sizeof(deriv_fil)/sizeof(deriv_fil[0]),1);

//        // print derivative signal
//        if (DEBUG) {
//            put("derivative_sig");
//            print_array_s(deriv_sig, OTE);
//        }
//        
//        // step 4) Weighted average filter
//        convolve(deriv_sig, weight_fil, weight_sig, OTE, sizeof(weight_fil)/sizeof(weight_fil[0]),10);
//        
//        // print clean signal
//        if (DEBUG) {
//            put("weight_sig2");
//            print_array_u(weight_sig, OTE);
//        }
        
//        while(1){}

        
        /* Find Left and Right edge */
        // (Assuming left to right read)
        // Left is max of derivative Right is min
        int left_index;
        int right_index;
        for (int i=0; i < OTE; i++) {
            if (weight_sig[i] > 40000) {
                left_index = i;
            }
        }
        
        for (int i=OTE-1; i >= 0; i--) {
            if (weight_sig[i] > 40000) {
                right_index = i;
            }
        }

        double turn = ((double)right_index + (double)left_index) / (double)2;


        turn_car(turn);

        delay(50);
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
    }

	return 0;
}

/* median_filter
 * Description:
 *  Apply a median filter to get rid of any peaks in the signal
 * 
 * Parameters:
 *  double &x - input array
 *  double &y - output array
 *  int n - size of arrays
 *
 * Returns:
 *  void
 */
void median_filter(uint16_t *x, uint16_t *y, int x_size) {
    // A three point median filter
    for (int i=0; i < x_size; i++) {
        
        // first element
        if (i == 0) {
            if(x[i] < x[i+1]) {
                y[i] = x[i];
            }
            else {
                y[i] = x[i+1];
            }
        }
        
        // last element
        else if(i == x_size - 1) {
            if(x[i] > x[i-1]) {
                y[i] = x[i];
            }
            else {
                y[i] = x[i-1];
            }
        }
        
        // middle elements
        else {
            if ((x[i-1] <= x[i]) && (x[i-1] <= x[i+1])) {
                y[i] = (x[i] <= x[i+1]) ? x[i] : x[i+1];
            }
            else if ((x[i] <= x[i-1]) && (x[i] <= x[i+1])) {
                y[i] = (x[i-1] <= x[i+1]) ? x[i-1] : x[i+1];
            }
            else {
                y[i] = (x[i-1] <= x[i]) ? x[i-1] : x[i];
            }
        }
    }
}

/* convolve
 * Description:
 *  Filters a 1D signal with the given inputs.
 * 
 * Parameters:
 *  double x - input array
 *  double h - filter array
 *  double y - output array
 *  int xSize - length of x array
 *  int hSize - length of h array
 *	int correction - correction factor for given filter
 *									 (e.g. if filter is 1,2,1 correction is sum = 4)
 *
 * Returns:
 * 	void
 */
void convolve(uint16_t *x, int16_t *h, uint16_t *y, int xSize, int hSize, int correction) {
    // size of y (output vector) needs to be the same size as f (input vector)
    // start hSizein so we don’t index less than 0  (boundary condition)
    for (int i=(hSize-1);i <  xSize; i++)  //need to add end boundary condition
    {
        double sum = 0.0;
        for (int j=hSize; j >=0; j--) {
            sum += h[j] * x[i-j];   //inner dot product
        }
        y[i] = sum / correction;
    }
}

void der_convolve(uint16_t *x, int16_t *h, int16_t *y, int xSize, int hSize, int correction) {
    // size of y (output vector) needs to be the same size as f (input vector)
    // start hSizein so we don’t index less than 0  (boundary condition)
    for (int i=(hSize-1);i <  xSize; i++)  //need to add end boundary condition
    {
        double sum = 0.0;
        for (int j=hSize; j >=0; j--) {
            sum += h[j] * x[i-j];   //inner dot product
        }
        y[i] = sum / correction;
    }
}

/* initialize
* Description:
*   Function that contains all the initialization function
* 
* Parameters:
*   void
* 
* Returns:
*   void
*/
void initialize(void) {
	// Initialize UART
	uart0_init();
	uart3_init();
    
    // Initialize Camera
    init_GPIO(); // For CLK and SI output on GPIO
    init_FTM2(); // To generate CLK, SI, and trigger ADC
    init_ADC0();
    init_PIT(); // To trigger camera read based on integration time

	// Initialize the FlexTimer
	init_PWM();
}

/* turn_car
 * Description:
 *  Turn the car with a standard value range
 * 
 * Parameters:
 *  double angle - value from 0 to 127 where... 
 * 	               0 is left,
 *                 64 is straight,
 *                 127 is right
 * 
 * Returns:
 *  void
 */
void turn_car(double angle){
    double dutycycle = 4.8 + (angle / (double) 29);
    SetServoDutyCycle(dutycycle);
}
