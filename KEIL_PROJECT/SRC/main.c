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
#include "math.h"

// Common Static Values
#define     ONE_TWENTY_EIGHT    128
#define     SIXTY_FOUR          64
#define     MIN_MARGIN          4
#define     MAX_MARGIN          8

// Servo ranges
#define     SERVO_MIN           4.5
#define     SERVO_MID           7.25
#define     SERVO_MAX           9

// Motor Ranges
#define     MOTOR_MAX           70
#define     MOTOR_MIN           35

// PID Values
#define     KP                  4.25
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
Struct left_right_index(int16_t* array, int old_calculated_middle);

int main(void)
{
    // Initialize UART and PWM
    initialize();

    // Array holding the 128 length array containing camera signal
    uint16_t* camera_sig;

    // Initialize variables for PID control
    double servo_turn_old = 64.0;
    double servo_err_old1 = 0.0;
    double servo_err_old2 = 0.0;

    // Initialize the starting duty cycle
    int motor_duty_left = MOTOR_MAX;
    int motor_duty_right = MOTOR_MIN;

    // motor speeds that can be changed based off the button
    int motor_max = MOTOR_MAX;
    int motor_min = MOTOR_MIN;

    int old_calculated_middle = SIXTY_FOUR;

    // all LED colors off
    GPIOE_PSOR = (1UL << 26);
    GPIOB_PSOR = (1UL << 21);
    GPIOB_PSOR = (1UL << 22);

    // White
    GPIOE_PCOR = (1UL << 26);
    GPIOB_PCOR = (1UL << 21);
    GPIOB_PCOR = (1UL << 22);

    // Wait until the button is pressed to start
    while (1) {
        // Once we select the mode, we break out of this loop ((GPIOC_PDIR & (1 << 6)) == 0)
        if((GPIOA_PDIR & (1 << 4)) == 0)
        {
            // Turn off the LEDs
            GPIOE_PSOR = (1UL << 26);
            GPIOB_PSOR = (1UL << 21);
            GPIOB_PSOR = (1UL << 22);

            // Wait to make sure the SW3 is unpressed
			delay(20);
            break;
        }
    }

    for (int button_count = 0; button_count < 6; button_count++)
    {
        if (button_count % 2 == 0){
            if (button_count == 0)
            {
                // GREEN
                GPIOE_PCOR = (1UL << 26);

                // Motor values
                motor_max = MOTOR_MAX;
                motor_min = MOTOR_MIN;
            }
            else if(button_count == 2)
            {
                // Blue
                GPIOB_PCOR = (1UL << 21);

                // Motor values
                motor_max = MOTOR_MAX - 8;
                motor_min = MOTOR_MIN - 8;
            }
            else
            {
                // RED
                GPIOB_PCOR = (1UL << 22);

                // Motor values
                motor_max = MOTOR_MAX - 16;
                motor_min = MOTOR_MIN - 16;
            }
            while(1){

                // Read Trace Camera
                camera_sig = Camera_Main();

                // Filter linescan camera signal
                int16_t deriv_sig[ONE_TWENTY_EIGHT];
                filter_main(camera_sig, deriv_sig);

                // Calculate center of track
                Struct edge_index = left_right_index(deriv_sig, old_calculated_middle);
                int calculated_middle = ((edge_index.right - edge_index.left)/2) + edge_index.left;
                int middle_delta = abs(SIXTY_FOUR - calculated_middle);

                // Perform PID calculations
                double servo_err = (double) SIXTY_FOUR - (double) calculated_middle;
                double servo_turn = servo_turn_old - \
                                   (double) KP * (servo_err-servo_err_old1) - \
                                   (double) KI * (servo_err+servo_err_old1)/2 - \
                                   (double) KD * (servo_err - 2*servo_err_old1 + servo_err_old2);

                // convert to a number usable by the servos
                double servo_range = (double) SERVO_MAX - (double) SERVO_MIN;
                double range_mult = (double) ONE_TWENTY_EIGHT / servo_range;
                double servo_duty = (double) SERVO_MIN + (servo_turn / (double) range_mult);

                // convert to a number useable for rear differential turning
                double middle_servo_offset = servo_duty - (double) SERVO_MIN;
                int middle_servo_percent = 100 * (middle_servo_offset / servo_range);
                int abs_motor_percent = abs(25 - (middle_servo_percent/2));

                // TURN ALL THE WAY RIGHT
                if (servo_duty > SERVO_MAX)
                {
                    SetServoDutyCycle(SERVO_MAX);
                    motor_duty_left = (motor_max + motor_min) / 2;
                    motor_duty_right = motor_min - 8;
                }
                // TURN ALL THE WAY LEFT
                else if (servo_duty < SERVO_MIN)
                {
                    SetServoDutyCycle(SERVO_MIN);
                    motor_duty_left = motor_min - 8;
                    motor_duty_right = (motor_max + motor_min) / 2;
                }
                else
                {
                    SetServoDutyCycle(servo_duty);
                    motor_duty_left = motor_max - abs_motor_percent;
                    motor_duty_right = motor_max - abs_motor_percent;
                }

                // Turn on motors
               SetMotorDutyCycleL(motor_duty_left, 10000, 1);
               SetMotorDutyCycleR(motor_duty_right, 10000, 1);

                // update old servo values
                servo_turn_old = servo_turn;
                servo_err_old2 = servo_err_old1;
                servo_err_old1 = servo_err;

                // update old middle
                old_calculated_middle = calculated_middle;

                // break out if the SW3 is pressed
                if((GPIOA_PDIR & (1 << 4)) == 0){
                    break;
                }
            }
        }
        else
        {
            // Stop before next run
            SetMotorDutyCycleL(0, 10000, 1);
            SetMotorDutyCycleR(0, 10000, 1);
            SetServoDutyCycle(SERVO_MID);

            // Wait to make sure the SW3 is unpressed
			delay(20);

            // Turn off the LEDs
			GPIOE_PSOR = (1UL << 26);
            GPIOB_PSOR = (1UL << 21);
            GPIOB_PSOR = (1UL << 22);

            // Wait to be turned on and ready to go slower
			for(;;)
            {
                if((GPIOA_PDIR & (1 << 4)) == 0)
                {
                    // Wait to make sure the SW3 is unpressed
					delay(20);
                    break;
                }
            }
        }
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
 *
 *  Returns: Struct containing min and max index of track
 */
Struct left_right_index(int16_t* array, int old_calculated_middle) {
    Struct s;

    int min_idx = SIXTY_FOUR;
    int max_idx = SIXTY_FOUR;


    // calculate the mean
    int total = 0;
    for (int i = 0; i < ONE_TWENTY_EIGHT; i++)
    {
        total += array[i];
    }

//    print_array_s(array, ONE_TWENTY_EIGHT);

    int mean = total / ONE_TWENTY_EIGHT;

    // calculate difference squared from mean
    int difference = 0;
    for (int i = 0; i < ONE_TWENTY_EIGHT; i++)
    {
        difference += pow((array[i] - mean), 2);
    }

    // calculate standard deviation
    int stdev = sqrt(difference/(ONE_TWENTY_EIGHT - 1));

    // Print the middle delta as to determine what is usual and what to make the MARGIN
//    char mid_delta[10000];
//    sprintf(mid_delta, "%d %d \n\r", mean, stdev);
//    put(mid_delta);

    int breakmin = 0;
    for (int c = old_calculated_middle; c < ONE_TWENTY_EIGHT; c++)
    {
        if ((array[c] < (mean - stdev)) && (breakmin == 0))
        {
            min_idx = c;
            breakmin = 1;
        }
    }

    int breakmax = 0;
    for (int c = old_calculated_middle; c > 0; c--)
    {
        if ((array[c] > (mean + stdev)) && (breakmax == 0))
        {
            max_idx = c;
            breakmax = 1;
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
