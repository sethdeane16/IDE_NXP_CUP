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
#define     SERV_MIN            4.5
#define     SERV_MAX            9
#define     SERV_MID            (SERV_MIN + ((SERV_MIN + SERV_MAX) / 2))

#define     DUTY                45

#define     CAM_DEBUG           0
#define     SER_DEBUG           0

// Structure to hold the greatest and smallest value from the camera array.
// Left is the smaller index, Right is the larger index.
struct greaterSmaller {
 int left, right;
};

// Declarations
typedef struct greaterSmaller Struct;
Struct LeftRightIndex(int16_t* array, int size);
void DriveAllNight(Struct left_right);
void Filter(uint16_t* camera_sig, int16_t* deriv_sig);

int main(void)
{
    // Initialize UART and PWM
    initialize();

    // Array holding the 128 length array containing camera signal
    uint16_t* camera_sig;
    
    // Make sure wheels are straightened initally
    SetServoDutyCycle(SERV_MIN);
    
    // Initialize variables for PID control
    double ServoTurnOld = 64.0;
    double ErrOld1 = 0.0;
    double ErrOld2 = 0.0;
    
    while(1){
        
        // Read Trace Camera
        camera_sig = Camera_Main();

        // Filter linescan camera signal
        int16_t deriv_sig[ONE_TWENTY_EIGHT];
        Filter(camera_sig, deriv_sig);

        // Turn on motors
        SetMotorDutyCycleL(DUTY, 10000, 1);
        SetMotorDutyCycleR(DUTY, 10000, 1);
        
        // Calculate center of track
        Struct edge_index = LeftRightIndex(deriv_sig, ONE_TWENTY_EIGHT);
        int calculated_middle = ((edge_index.right - edge_index.left)/2) + edge_index.left ;
        

        // combos that work, kp=6, ki=0, kd=2
        double Kp = 5.0;
        double Ki = 0.0;   // default .15 try .02
        double Kd = 2.0;   // default .20

        double Err = (double) SIXTY_FOUR - (double) calculated_middle;
        double ServoTurn = ServoTurnOld - \
                           Kp * (Err-ErrOld1) - \
                           Ki * (Err+ErrOld1)/2 - \
                           Kd * (Err - 2*ErrOld1 + ErrOld2);
        char taco[10000];
        sprintf(taco,"%3.2lf\n\r",ServoTurn);
        put(taco);
        
        double servo_range = (double) SERV_MAX - (double) SERV_MIN;
        double range_mult = (double) ONE_TWENTY_EIGHT / servo_range;
        double dutycycle = (double) SERV_MIN + (ServoTurn / (double) range_mult);
        
        if (dutycycle > SERV_MAX)
        {
            SetServoDutyCycle(SERV_MAX);
        }
        else if (dutycycle < SERV_MIN)
        {
            SetServoDutyCycle(SERV_MIN);
        }
        else
        {
            SetServoDutyCycle(dutycycle);
        }
        ServoTurnOld = ServoTurn;
        ErrOld2 = ErrOld1;
        ErrOld1 = Err;
     
//        DriveAllNight(LeftRightIndex(deriv_sig, ONE_TWENTY_EIGHT));

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
    if (middle > SIXTY_FOUR + MARGIN) {
        TurnCar(edge_index.right);
//        if (middle > SIXTY_FOUR + 2*MARGIN)
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
    else if(middle < SIXTY_FOUR - MARGIN) {
        TurnCar(edge_index.left);
//        if (middle > SIXTY_FOUR - 2*MARGIN)
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
    if ((index < ONE_TWENTY_EIGHT) && (index >= 0))
    {
        double servo_range = (double) SERV_MAX - (double) SERV_MIN;
        double range_mult = (double) ONE_TWENTY_EIGHT / servo_range;
        double dutycycle = (double) SERV_MIN + ((double) index / (double) 29);
        SetServoDutyCycle(dutycycle);
    }
}


/* Filters
 * Description:
 *  Take an input signal from the linescan camera
 *  and output a struct
 *
 * Parameters:
 *  camera_sig - Camera Input signal.
 *  deriv_sig - Derivative of smoothed signal.
 *
 * Returns:
 *  void
 */
 void Filter(uint16_t* camera_sig, int16_t* deriv_sig)
 {
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
        
        // Get get rid of 4 zeros that are created by convolve function
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
            int delta = abs(SIXTY_FOUR-da_mid);
            sprintf(taco,"%d, %d, %d, %d\n\r",jr.left,da_mid,jr.right, delta);
            put(taco);
        }
 }