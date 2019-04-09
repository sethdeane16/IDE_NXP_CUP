/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 *
 * file:    main.c
 * Authors: Seth Deane & Brian Powers
 * Created: March 21 2019
 */

#include "stdlib.h"
#include "MK64F12.h"
#include "uart.h"
#include "camera.h"
#include "pwm.h"
#include "common.h"
#include "main.h"

#define     ONE_TWENTY_EIGHT    128
#define     SIXTY_FOUR          64
#define     MARGIN              3
#define     SERV_MIN            4.8
#define     SERV_MAX            9.14545
#define     SERV_MID            (SERV_MIN + ((SERV_MIN + SERV_MAX) / 2))
#define     CENTER              72

#define     DUTY                40

#define     CAM_DEBUG           0
#define     SER_DEBUG           1


struct greaterSmaller {
 int left, right;
};

typedef struct greaterSmaller Struct;

Struct LeftRightIndex(int16_t* array, int size);
void DriveAllNight(Struct left_right);

int main(void)
{
    // Initialize UART and PWM
    initialize();

    uint16_t* camera_sig;   // previously ptr

    while(1){
        // Read Trace Camera
        camera_sig = Camera_Main();

        // print camera signal
        if (CAM_DEBUG) {
            put("camera_sig");
            print_array_u(camera_sig, ONE_TWENTY_EIGHT);
        }

        /*****************************
         Normalize Trace
        *****************************/
        // step 1) Median filter
        uint16_t median_sig[ONE_TWENTY_EIGHT];
        median_filter(camera_sig, median_sig, ONE_TWENTY_EIGHT);

        // print median signal
        if (CAM_DEBUG) {
            put("median_sig");
            print_array_u(median_sig, ONE_TWENTY_EIGHT);
        }

        // step 2) Weighted average filter
        int16_t weight_fil[5] = {1,2,4,2,1};
        uint16_t weight_sig[ONE_TWENTY_EIGHT];
        convolve(median_sig, weight_fil, weight_sig, ONE_TWENTY_EIGHT, sizeof(weight_fil)/sizeof(weight_fil[0]),10);

        // Correct the zeros at the beginning
        for (int k = 2; k < ONE_TWENTY_EIGHT-2; k++){
            weight_sig[k] = weight_sig[k+2];
        }
        weight_sig[0] = weight_sig[2];
        weight_sig[1] = weight_sig[2];
        weight_sig[126] = weight_sig[125];
        weight_sig[127] = weight_sig[125];

        // print clean signal
        if (CAM_DEBUG) {
            put("weight_sig");
            print_array_u(weight_sig, ONE_TWENTY_EIGHT);
        }

        // step 3) Derivative filter
        int16_t deriv_fil[3] = {1,0,-1};
        int16_t deriv_sig[ONE_TWENTY_EIGHT];
        der_convolve(weight_sig, deriv_fil, deriv_sig, ONE_TWENTY_EIGHT, sizeof(deriv_fil)/sizeof(deriv_fil[0]),1);

        // print derivative signal
        if (CAM_DEBUG) {
            put("derivative_sig");
            print_array_s(deriv_sig, ONE_TWENTY_EIGHT);
        }

        if (SER_DEBUG) {
            char taco[100];
            Struct jr = LeftRightIndex(deriv_sig, ONE_TWENTY_EIGHT);
            int da_mid = jr.left + ((jr.right-jr.left)/2);
            int delta = abs(CENTER-da_mid);
            sprintf(taco,"%d, %d, %d, %d\n\r",jr.left,da_mid,jr.right, delta);
            put(taco);
        }

        /*****************************
         DRIVE BABY DRIVE
        *****************************/


        // SetMotorDutyCycleL(DUTY, 10000, 1);
        // SetMotorDutyCycleR(DUTY, 10000, 1);
        DriveAllNight(LeftRightIndex(deriv_sig, ONE_TWENTY_EIGHT));

    }

	return 0;
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
 *  uint16_t x - input array
 *  int16_t h - filter array
 *  uint16_t y - output array
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

/* der_convolve
 * Description:
 *  Filters to the derivative of a given signal.
 *
 * Parameters:
 *  uint16_t x - input array
 *  int16_t h - filter array
 *  int16_t y - output array
 *  int xSize - length of x array
 *  int hSize - length of h array
 *	int correction - correction factor for given filter
 *									 (e.g. if filter is 1,2,1 correction is sum = 4)
 *
 * Returns:
 * 	void
 */
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


/* LeftRightIndex
 * Description:
 *  Find the left and right index of an array
 *
 * Parameters:
 *  int16_t array - input array
 *  int size - size of array
 *
 * Returns:
 *  int - min and max index
 */
Struct LeftRightIndex(int16_t* array, int size) {
    Struct s;

    int16_t minimum = array[0];
    int min_idx = 0;
    int16_t maximum = array[0];
    int max_idx = 0;

    for (int c = 1; c < size; c++)
    {
        if (array[c] < minimum)
        {
            minimum = array[c];
            min_idx = c;
        }
        if (array[c] > maximum)
        {
            maximum = array[c];
            max_idx = c;
        }
    }

    s.left = max_idx;
    s.right = min_idx;

    return s;
}


/* DriveAllNight
 * Description:
 *  Determine whether to turn or drive straight
 *
 * Parameters:
 *  Struct edge_index - input structure with the index of
 *                      left and right side of the track
 *
 * Returns:
 *  void
 */
void DriveAllNight(Struct edge_index) {

    uint16_t middle = ((edge_index.right - edge_index.left)/2) + edge_index.left ;

    // char tacos[100];
    // sprintf(tacos,"%d\n\r", middle);
    // put(tacos);

    // steer right if on the left side of the track
    if (middle > CENTER + MARGIN) {
        TurnCar(edge_index.right);
//        if (middle > CENTER + 2*MARGIN)
//        {
//            SetMotorDutyCycleL(DUTY, 10000, 1);
//            SetMotorDutyCycleR(DUTY/2, 10000, 1);
//            TurnCar(edge_index.right);
//        }
//        else
//        {
//            SetMotorDutyCycleL(DUTY, 10000, 1);
//            SetMotorDutyCycleR(DUTY, 10000, 1);
//            TurnCar(edge_index.right);
//        }
    }
    // steer left if on the right side of the track
    else if(middle < CENTER - MARGIN) {
        TurnCar(edge_index.left);
//        if (middle > CENTER - 2*MARGIN)
//        {
//            SetMotorDutyCycleL(DUTY/2, 10000, 1);
//            SetMotorDutyCycleR(DUTY, 10000, 1);
//            TurnCar(edge_index.left);
//        }
//        else
//        {
//            SetMotorDutyCycleL(DUTY, 10000, 1);
//            SetMotorDutyCycleR(DUTY, 10000, 1);
//            TurnCar(edge_index.left);
//        }
    }
    // otherwise, drive straight
    else {
//        SetMotorDutyCycleL(DUTY, 10000, 1);
//        SetMotorDutyCycleR(DUTY, 10000, 1);
        TurnCar(SIXTY_FOUR);
    }
}

/* TurnCar
 * Description:
 *  Turn the car with a standard value range.
 *  Takes an index and ouptuts the duty cycle to
 *  turn the servo the appropriate amount.
 *
 * Parameters:
 *  double angle - value from 0 to 127 where...
 * 	               0 is completely left,
 *                 64 is straight,
 *                 127 is completely right
 *
 * Returns:
 *  void
 */
void TurnCar(uint16_t index){
    // Using middle index gives better for multipliers for both gradual and extreme turns once tested
    // GENERAL IDEA: Save previous middles??? use to adjust middle. Probably not
    if ((index < ONE_TWENTY_EIGHT) && (index >= 0)) {
        double dutycycle = (double) SERV_MID;
        double servo_range = (double) SERV_MAX - (double) SERV_MIN;
        double range_mult = (double) ONE_TWENTY_EIGHT / servo_range;
        double norm_duty = (double) SERV_MIN + ((double) index / range_mult); // THIS IS WHAT YOU HAD
        if (index <= 30)
        {
            dutycycle = SERV_MIN;
        }
        // Left side won't turn as hard as right
        else if ((index > 30) && (index <= 40))
        {
            dutycycle = .7 * norm_duty;
        }
        else if ((index > 40) && (index <= 50))
        {
            dutycycle = .85 * norm_duty;
        }
        else if ((index > 50) && (index <= 60))
        {
            dutycycle = 1 * norm_duty;
        }
        else if ((index >= 70) && (index < 80))
        {
            dutycycle = 1 * norm_duty;
        }
        else if ((index >= 80) && (index < 90))
        {
            dutycycle = 1.15 * norm_duty;
        }
        else if ((index >= 90) && (index < 100))
        {
            dutycycle = 1.3 * norm_duty;
        }
        else if (index >= 100)
        {
            dutycycle = SERV_MAX;
        }
        else
        {
            dutycycle = dutycycle;
        }
        SetServoDutyCycle(dutycycle);
    }
}
