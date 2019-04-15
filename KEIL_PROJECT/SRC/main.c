/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 *
 * file:    main.c
 * Authors: Seth Deane & Brian Powers
 * Created: March 21 2019
 */

#include "MK64F12.h"
#include "filters.h"
#include "camera.h"
#include "common.h"
#include "stdlib.h"
#include "main.h"
#include "uart.h"
#include "pwm.h"

// Common Static Values
#define     ONE_TWENTY_EIGHT    128
#define     SIXTY_FOUR          64
#define     SERV_MIN            4.5
#define     SERV_MAX            9
#define     SERV_MID            (SERV_MIN + ((SERV_MIN + SERV_MAX) / 2))
#define     MOTOR_DUTY          50
#define     MARGIN              4
#define     MAX_DUTY            60
#define     MIN_DUTY            40

// PID Values
// combo that work
//  - kp=6, ki=0, kd=2 at 40% duty cycle
//  - kp=5, ki=0, kd=2 at 50% duty cycle
#define     KP                  5.0
#define     KI                  0.0
#define     KD                  2.0

// Debugging variables (1 = Debug True)
#define     CAM_DEBUG           0
#define     SER_DEBUG           0

// Structure to hold the greatest and smallest value from the camera array.
// Left is the smaller index, Right is the larger index.
struct greaterSmaller {
 int left, right;
};

typedef struct greaterSmaller Struct;

int main(void)
{
    // Initialize UART and PWM
    initialize();

    // Array holding the 128 length array containing camera signal
    uint16_t* camera_sig;

    // Initialize the starting duty cycle
    double dutycycle = MOTOR_DUTY;
    double old_dutycycle = MOTOR_DUTY;

    // Initialize variables for PID control
    double servo_turn_old = 64.0;
    double err_old1 = 0.0;
    double err_old2 = 0.0;

    while(1){

        // Read Trace Camera
        camera_sig = Camera_Main();

        // Filter linescan camera signal
        int16_t deriv_sig[ONE_TWENTY_EIGHT];
        Filter(camera_sig, deriv_sig);

        // Calculate center of track
        Struct edge_index = left_right_index(deriv_sig, ONE_TWENTY_EIGHT);
        int calculated_middle = ((edge_index.right - edge_index.left)/2) + edge_index.left;
        int middle_delta = abs(SIXTY_FOUR - calculated_middle);

        // Print the middle delta as to determine what is usual and what to make the MARGIN
        char mid_delta[10000];
        sprintf("%d \n\r", middle_delta);
        put(mid_delta);

        // Might rubber band if on turn it rectifies itself enough
        // could be made into a function
        // Turning slow down
        if (middle_delta > MARGIN)
        {
            if (old_dutycycle > MOTOR_DUTY)
            {
                dutycycle = (double) MOTOR_DUTY;
            }
            else if (old_dutycycle == MIN_DUTY)
            {
                dutycycle = old_dutycycle;
            }
            else
            {
                dutycycle = old_dutycycle - 5.0;
            }
        }
        // Straight Speed up
        else
        {
            if (old_dutycycle < MOTOR_DUTY)
            {
                dutycycle = (double) MOTOR_DUTY;
            }
            else if (old_dutycycle == MAX_DUTY)
            {
                dutycycle = old_dutycycle;
            }
            else
            {
                dutycycle = old_dutycycle + 2.0;
            }
        }
        old_dutycycle = dutycycle;

        // Turn on motors
        SetMotorDutyCycleL(dutycycle, 10000, 1);
        SetMotorDutyCycleR(dutycycle, 10000, 1);

        // Perform PID calculations
        double err = (double) SIXTY_FOUR - (double) calculated_middle;
        double servo_turn = ServoTurnOld - \
                           (double) KP * (err-err_old1) - \
                           (double) KI * (err+err_old1)/2 - \
                           (double) KD * (err - 2*err_old1 + err_old2);

        // convert to a
        double servo_range = (double) SERV_MAX - (double) SERV_MIN;
        double range_mult = (double) ONE_TWENTY_EIGHT / servo_range;
        double dutycycle = (double) SERV_MIN + (servo_turn / (double) range_mult);

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
        servo_turn_old = servo_turn;
        err_old2 = err_old1;
        err_old1 = err;
    }

	return 0;
}


/*
 * Function: initialize
 * --------------------
 *  Function that contains all the initialization function.
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

/* Function: left_right_index
 * --------------------------
 *  Find the left and right index of an array.
 *
 *  array: input array
 *  size: size of array
 *
 *  Returns: Struct containing min and max index of track
 */
Struct left_right_index(int16_t* array, int size) {
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

/*
 * Function: filter_main
 * ---------------------
 *  Take an input signal from the linescan camera and output a 3x filtered
 *  signal. First filter is a median filter to remove unwanted spikes. Second
 *  filter is a weighted average filter which smooths out signal. Third filter
 *  is a derivative filter which assists in finding left and right sides of the
 *  track.
 *
 *  camera_sig: Diret camera input signal.
 *  deriv_sig: Derivative of smoothed signal.
 *
 *  Returns: Void
 */
 void filter_main(uint16_t* camera_sig, int16_t* deriv_sig)
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
    convolve(median_sig, \
             weight_fil, \
             weight_sig, \
             ONE_TWENTY_EIGHT, \
             sizeof(weight_fil)/sizeof(weight_fil[0]), \
             10);

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
    der_convolve(weight_sig, \
                 deriv_fil, \
                 deriv_sig, \
                 ONE_TWENTY_EIGHT, \
                 sizeof(deriv_fil)/sizeof(deriv_fil[0]), \
                 1);

    // print derivative signal
    if (CAM_DEBUG) {
        put("derivative_sig");
        print_array_s(deriv_sig, ONE_TWENTY_EIGHT);
    }
 }
