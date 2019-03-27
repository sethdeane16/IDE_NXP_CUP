/*
 * isr.c
 */

#include <stdio.h>
#include "MK64F12.h"
#include "uart.h"
#include "isr.h"

//variables global to the IRQ handlers which dictates if timer is enabled &  timer counter
int sw2Pressed = 0; // 1 for pressed
int ftmCounter = 0; // ftm counter

void PDB0_IRQHandler(void)
{
    // clear the interrupt in register PDB0_SC
    PDB0_SC &= ~PDB_SC_PDBIF_MASK;

    // toggle the output state for LED1
    GPIOB_PTOR = (1UL << 22); // red led

    return;
}

void FTM0_IRQHandler(void)
{
    // clear the interrupt in register FTM0_SC
    FTM0_SC &= ~FTM_SC_TOF_MASK;

    // If switch2 has set the local variable to signal a button press...
    if (sw2Pressed == 1)
    {
        // then increment the local counter variable, else do nothing
        ftmCounter ++;
    }

	return;
}

//For switch 3
void PORTA_IRQHandler(void)
{
    // clear the interrupt
    PORTA_PCR4 |= PORT_PCR_ISF_MASK;

    // if the pdb timer is enabled...
    if (PDB0_SC & (1 << PDB_SC_PDBEN_SHIFT))
    {
        // disable the pdb timer
        PDB0_SC &= ~PDB_SC_PDBEN_MASK;
    }

    // else
    else
    {
        // enable the timer
        PDB0_SC |= PDB_SC_PDBEN_MASK;

        // start it with a trigger
        PDB0_SC |= PDB_SC_SWTRIG_MASK;
    }

	return;
}

void PORTC_IRQHandler(void){ //For switch 2

    // clear the interrupt
    PORTC_PCR6 |= PORT_PCR_ISF_MASK;

    // Switch 2, pressed
    if((GPIOC_PDIR & (1 << 6)) == 0)
    {
        // Set a local variable to affect the timer2 function
        sw2Pressed = 1;

        // reset the ftm timer
        FTM0_CNT = 0;

        // Reset the timer counter
        ftmCounter = 0;

        // turn on the blue LED while the button is pressed
        GPIOB_PCOR = (1UL << 21); // blue led on
    }

    else
    {
        // reset the local variable to affect the timer2 fucntion
        sw2Pressed = 0;

        // turn off the blue LED while button is up
        GPIOB_PSOR = (1UL << 21); // blue led off

        // print the result
        put("Button held for ");
        putnumU(ftmCounter);
        put(" milliseconds!\r\n");
    }

	return;
}
