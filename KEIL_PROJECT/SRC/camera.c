/*
 *  Freescale Cup linescan camera code
 *
 *  This method of capturing data from the line
 *  scan cameras uses a flex timer module, periodic
 *  interrupt timer, an ADC, and some GPIOs.
 *  CLK and SI are driven with GPIO because the FTM2
 *  module used doesn't have any output pins on the
 *  development board. The PIT timer is used to
 *  control the integration period. When it overflows
 *  it enables interrupts from the FTM2 module and then
 *  the FTM2 and ADC are active for 128 clock cycles to
 *  generate the camera signals and read the camera
 *  output.
 *
 *  PTB9      - camera CLK
 *  PTB23     - camera SI
 *  ADC0_DP0  - camera AOut
 *
 * File:    camera.c
 * Authors: Seth Deane & Brian Powers
 * Created: 3/21/2019
 */

#include "MK64F12.h"
#include "camera.h"
#include "uart.h"
#include <stdio.h>

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk
//  (camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
uint16_t line[128];

// These variables are for streaming the camera
//   data over UART
int debugcamdata = 0;
int capcnt = 0;
char str[100];

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

/* Camera_Main
* Description:
* 	Retrives a scan from the camera
* 
* Parameters:
*   void
* 
* Returns:
*   uint16_t* - 128 int array of camera values
*/
uint16_t* Camera_Main(void) {

    int i;

    if (debugcamdata) {
        // Every 2 seconds
        if (capcnt >= (500)) {
            GPIOB_PCOR |= (1 << 22);
            // send the array over uart
            sprintf(str,"%i\n\r",-1); // start value
            put(str);
            for (i = 0; i < 127; i++) {
                sprintf(str,"%i ", line[i]);
                put(str);
            }
            sprintf(str,"\n%i\n\r",-2); // end value
            put(str);
            capcnt = 0;
            GPIOB_PSOR |= (1 << 22);
        }
    }

    return line;

} // Camera_Main

/* ADC0_IRQHandler
* Description:
* 	ADC0 Conversion Complete ISR
*
* Parameters:
* 	void
*
* Returns:
*	void
*/
void ADC0_IRQHandler(void) {

	// Reading ADC0_RA clears the conversion complete flag
	ADC0VAL = ADC0_RA;

} // ADC0_IRQHandler

/* FTM2_IRQHandler
* Description:
* 	FTM2 handles the camera driving logic
*   This ISR gets called once every integration period
*   by the periodic interrupt timer 0 (PIT0)
*   When it is triggered it gives the SI pulse,
*   toggles clk for 128 cycles, and stores the line
*   data from the ADC into the line variable
*
* Parameters:
* 	void
*
* Returns:
*	void
*/
void FTM2_IRQHandler(void) {

    // Clear interrupt
    FTM2_SC &= ~FTM_SC_TOF_MASK;

    // Toggle clk
    clkval ^= 1;
    GPIOB_PTOR = (1 << 9);

    // Line capture logic
    if ((pixcnt >= 2) && (pixcnt < 256)) {
        if (!clkval) {  // check for falling edge
            // ADC read (note that integer division is
            //  occurring here for indexing the array)
            line[pixcnt/2] = ADC0VAL;
        }
        pixcnt += 1;
    } else if (pixcnt < 2) {
        if (pixcnt == -1) {
            GPIOB_PSOR |= (1 << 23); // SI = 1
        } else if (pixcnt == 1) {
            GPIOB_PCOR |= (1 << 23); // SI = 0
            // ADC read
            line[0] = ADC0VAL;
        }
        pixcnt += 1;
    } else {
        GPIOB_PCOR |= (1 << 9); // CLK = 0
        clkval = 0; // make sure clock variable = 0
        pixcnt = -2; // reset counter
        // Disable FTM2 interrupts (until PIT0 overflows
        //   again and triggers another line capture)
        FTM2_SC &= ~FTM_SC_TOIE_MASK;

    }

    return;

} // FTM2_IRQHandler

/* PIT0_IRQHandler
* Description:
* 	PIT0 determines the integration period
*   When it overflows, it triggers the clock logic from
*   FTM2. Note the requirement to set the MOD register
*   to reset the FTM counter because the FTM counter is
*   always counting, I am just enabling/disabling FTM2
*   interrupts to control when the line capture occurs
*
* Parameters:
*	void
*
* Returns:
*	void
*/
void PIT0_IRQHandler(void) {

    if (debugcamdata){
        // Increment capture counter so that we can only
        //  send line data once every ~2 seconds
        capcnt += 1;
    }

    // Clear interrupt
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

    // Setting mod resets the FTM counter
    FTM2->MOD = DEFAULT_SYSTEM_CLOCK/100000;    // about 200

    // Enable FTM2 interrupts (camera)
    FTM2_SC |= FTM_SC_TOIE_MASK;

    return;

} // PIT0_IRQHandler

/* init_FTM2
* Description:
* 	Initialization of FTM2 for camera
*
* Parameters:
* 	void
*
* Returns:
* 	void
*/
void init_FTM2(void) {

    // Enable clock
    SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

    // Disable Write Protection
    FTM2_MODE |= FTM_MODE_WPDIS_MASK;

    // Set output to '1' on init
    FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;

    // Initialize the CNT to 0 before writing to MOD
    FTM2_CNT = 0;

    // Set the Counter Initial Value to 0
    FTM2_CNTIN = 0;

    // Set the period (~10us)
	int period = DEFAULT_SYSTEM_CLOCK/100000;
	FTM2->MOD = period;   // about 200

    // 50% duty
	//FTM2_C0V &= ~FTM_CnV_VAL_MASK;  // clear first
	FTM2_C0V = (period >> 1);  // about 200 / 2 = 100

    // Set edge-aligned mode
    FTM2_C0SC |= FTM_CnSC_MSB_MASK;
    FTM2_C0SC |= FTM_CnSC_MSA_MASK;

    // Enable High-true pulses
    // ELSB = 1, ELSA = 0
    FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
    FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK;

    // Enable hardware trigger from FTM2
    FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;

    // Don't enable interrupts yet (disable)
    FTM2_SC &= ~FTM_SC_TOIE_MASK;

    // No prescalar, system clock
	FTM2_SC &= ~FTM_SC_PS_MASK;
    FTM2_SC |= (0x01 << FTM_SC_CLKS_SHIFT);

	// Enable interrupt
	FTM2_SC |= FTM_SC_TOIE_MASK;

    // Set up interrupt
    NVIC_EnableIRQ(FTM2_IRQn);

    return;

} // init_FTM2

/* init_PIT
* Description:
* 	Initialization of PIT timer to control integration period
*
* Parameters:
*	void
*
* Returns:
* 	void
*/
void init_PIT(void) {

    // Setup periodic interrupt timer (PIT)
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

    // Enable clock for timers
	PIT_MCR &= ~PIT_MCR_MDIS_MASK;

    // Enable timers to continue in debug mode
    PIT_MCR = PIT_MCR_FRZ_MASK;

    // PIT clock frequency is the system clock
    // Load the value that the timer will count down from
    PIT_LDVAL0 = (uint32_t)(DEFAULT_SYSTEM_CLOCK * INTEGRATION_TIME);

    // Enable timer interrupts
    PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;

    // Enable the timer
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

    // Clear interrupt flag
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

    // Enable PIT interrupt in the interrupt controller
    NVIC_EnableIRQ(PIT0_IRQn);
    
    return;

} // init_PIT


/* init_GPIO
* Description:
* 	Set up pins for GPIO
*   PTB9    - camera clk
*  	PTB23   - camera SI
*   PTB22  	- red LED
*
* Parameters:
*	void
*
* Returns:
*	void
*/
void init_GPIO(void) {

    //initialize clocks for each different port used.
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enables Clock on PORTB (LED)

    //Configure Port Control Register for Inputs with pull enable and pull up resistor

    // Configure mux for Outputs
    PORTB_PCR9 = PORT_PCR_MUX(1);  	// camera  	(clk)
    PORTB_PCR22 = PORT_PCR_MUX(1);  // red   	(LED)
    PORTB_PCR23 = PORT_PCR_MUX(1);  // camera 	(SI)

    // Switch the GPIO pins to output mode
    GPIOB_PDDR |= (1 << 9);  	// camera  	(clk)
    GPIOB_PDDR |= (1 << 22);  	// red   	(LED)
    GPIOB_PDDR |= (1 << 23);  	// camera 	(SI)

    GPIOB_PDDR = (1 << 23) | (1 << 22) | (1 << 9);  // camera (SI)

    // Turn off the outputs
    GPIOB_PSOR = (1UL << 22);  // red off   (LED)

    return;

} // init_GPIO

/* init_ADC0
* Description:
* 	Set up ADC for digitizing camera data 
* 
* Parameters:
* 	void
*
* Returns:
* 	void
*/
void init_ADC0(void) {

	unsigned int calib;
	// Turn on ADC0
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; // Enables Clock on ADC0

    // Single ended 16 bit conversion, no clock divider
    ADC0_CFG1 &= ~ADC_CFG1_ADIV_MASK; // No division
    ADC0_CFG1 |= ADC_CFG1_MODE(0x03); // single ended 16 bit

	// Do ADC Calibration for Singled Ended ADC. Do not touch.
	ADC0_SC3 = ADC_SC3_CAL_MASK;
	while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
	calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
	calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
	calib = calib >> 1; calib |= 0x8000;
	ADC0_PG = calib;

	// Select hardware trigger.
	ADC0_SC2 |= ADC_SC2_ADTRG_MASK;

	// Set to single ended mode
	ADC0_SC1A = 0;
	ADC0_SC1A |= ADC_SC1_AIEN_MASK;
    ADC0_SC1A &= ~ADC_SC1_DIFF_MASK;
	ADC0_SC1A &= ~ADC_SC1_ADCH(0x1F);

    // Set up FTM2 trigger on ADC0
    SIM_SOPT7 &= ~(0xF << SIM_SOPT7_ADC0TRGSEL_SHIFT);
    SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0x0A); // FTM2 select
    SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
    SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK; // Pretrigger A

    // Enable NVIC interrupt
	NVIC_EnableIRQ(ADC0_IRQn);
    
} // init_ADC0
