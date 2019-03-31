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

int main(void)
{
	// Initialize UART and PWM
	initialize();

    int CAM_VALS = 128;
    int HALF = 64;
    int MARGIN;
    uint16_t* camera_sig;   // previously ptr

    while(1) {

        turnServo(0);
        delay(200);
        
        turnServo(50);
        delay(200);
        
        turnServo(100);
        delay(200);

    }
    
    while(1){
        // Read Trace Camera
        camera_sig = Camera_Main();

        // print camera signal
        put("camera_sig");
        print_array(camera_sig, 20, 128);
        
        /* Normalize Trace */
        // step 1) Median filter
//        uint16_t median_sig[128];
//        median_filter(ptr, );

        // step 2) Weighted average filter
        uint16_t weight_filter[5] = {1,2,4,2,1};
        uint16_t clean_sig[128];
        weighted_filter(camera_sig, weight_filter, clean_sig, 128, sizeof(weight_filter)/sizeof(weight_filter[0]));
        

        // print clean signal
        put("clean_sig");
        print_array(clean_sig, 20, 128);

        // step 3) Derivative filter
        int derivative_filter[2] = {-1,1};
        int16_t deriv_sig[128];
        deriva_filter(clean_sig, derivative_filter, deriv_sig, 128, sizeof(derivative_filter)/sizeof(derivative_filter[0]));

        // print clean signal
        put("derivative_sig");
        print_array(deriv_sig, 20, 128);
        
        delay(2000000000);
        
        /* Find Left and Right edge */
        // (Assuming left to right read)
        // Left is max of derivative Right is min
//        int left_index = max(deriv_sig);
//        int right_index = min(deriv_sig);

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

/* median_filter
* Description:
*   Apply a median filter to get rid of any peaks in the signal
* 
* Parameters:
*   double &x - TODO
*   double &y - TODO
*   int n - TODO
*
* Returns:
*   void
*/
//void median_filter(double &x, double &y, int n)
//{
//}

/* convolve
* Description:
*   TODO
* 
* Parameters:
*   double x - input array
*   double h - filter array
*   double y - output array
*   int xSize - length of x array
*   int hSize - length of h array
*
* Returns:
*   void
*/
void weighted_filter(uint16_t *x,uint16_t *h, uint16_t *y, int xSize, int hSize)
{
    // size of y (output vector) needs to be the same size as f (input vector)
    // start hSizein so we don’t index less than 0  (boundary condition)
    for (int i=(hSize-1);i <  xSize; i++)  //need to add end boundary condition
    {
        double sum = 0.0;
        for (int j=hSize; j >=0; j--)
        {
            sum += h[j] * x[i-j];   //inner dot product
        }
        y[i] = sum / 10;
    }
}

/* convolve
* Description:
*   TODO
* 
* Parameters:
*   double x - input array
*   double h - filter array
*   double y - output array
*   int xSize - length of x array
*   int hSize - length of h array
*
* Returns:
*   void
*/
void deriva_filter(uint16_t *x,int *h, int16_t *y, int xSize, int hSize)
{
    // size of y (output vector) needs to be the same size as f (input vector)
    // start hSizein so we don’t index less than 0  (boundary condition)
    for (int i=(hSize-1);i <  xSize; i++)  //need to add end boundary condition
    {
        double sum = 0.0;
        for (int j=hSize; j >=0; j--)
        {
            sum += h[j] * x[i-j];   //inner dot product
        }
        y[i] = sum;
    }
}

/* initialize
* Description:
*   Function that contains all the initialization data
* 
* Parameters:
*   None
* 
* Returns:
*   void
*/
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
	init_PWM();
}




void turnServo(double angle){
    double dutycycle = 4.6 + (angle / (double) 22);
    SetServoDutyCycle(dutycycle);
}
