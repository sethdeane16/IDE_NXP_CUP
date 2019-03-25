/*
 * File:        Lab4_uart.c
 * Purpose:     Provide UART3 for Bluetooth
 *
 * Notes:		
 *
 */

#include "MK64F12.h"
#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)

void uart0_init()
{
    //define variables for baud rate and baud rate fine adjust
    uint16_t ubd, brfa;

    //Enable clock for UART
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    //Configure the port control register to alternative 3 (which is UART mode for K64)
    PORTB_PCR16 = PORT_PCR_MUX(3);
    PORTB_PCR17 = PORT_PCR_MUX(3);

    /*Configure the UART for establishing serial communication*/

    //Disable transmitter and receiver until proper settings are chosen for the UART module
    UART0_C2 &= ~UART_C2_TE_MASK;
    UART0_C2 &= ~UART_C2_RE_MASK;

    //Select default transmission/reception settings for serial communication of UART by clearing the control register 1
    UART0_C1 = 0x00;

    //UART Baud rate is calculated by: baud rate = UART module clock / (16 ? (SBR[12:0] + BRFD))
    //13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
    //BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
    //BRFA is defined by the lower 4 bits of control register, UART0_C4 

    //calculate baud rate settings: ubd = UART module clock/16* baud rate
    ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));

    //clear SBR bits of BDH
    UART0_BDH &= ~UART_BDH_SBR_MASK;

    //distribute this ubd in BDH and BDL
    UART0_BDH |= (uint32_t) (ubd >> 8) & 0x1F & UART_BDH_SBR_MASK;
    UART0_BDL |= (uint32_t) ubd & UART_BDL_SBR_MASK;

    //BRFD = (1/32)*BRFA 
    //make the baud rate closer to the desired value by using BRFA
    brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

    //write the value of brfa in UART0_C4
    UART0_C4 |= brfa & UART_C4_BRFA_MASK;
        
    //Enable transmitter and receiver of UART
    UART0_C2 |= UART_C2_TE_MASK;
    UART0_C2 |= UART_C2_RE_MASK;
}


void uart3_init()
{
    //define variables for baud rate and baud rate fine adjust
    uint16_t ubd, brfa;

    //Enable clock for UART
    SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    //Configure the port control register to alternative 3 (which is UART mode for K64)
    PORTB_PCR10 = PORT_PCR_MUX(3);
    PORTB_PCR11 = PORT_PCR_MUX(3);

    /*Configure the UART for establishing serial communication*/

    //Disable transmitter and receiver until proper settings are chosen for the UART module
    UART3_C2 &= ~UART_C2_TE_MASK;
    UART3_C2 &= ~UART_C2_RE_MASK;

    //Select default transmission/reception settings for serial communication of UART by clearing the control register 1
    UART3_C1 = 0x00;

    //UART Baud rate is calculated by: baud rate = UART module clock / (16 � (SBR[12:0] + BRFD))
    //13 bits of SBR are shared by the 8 bits of UART3_BDL and the lower 5 bits of UART3_BDH 
    //BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
    //BRFA is defined by the lower 4 bits of control register, UART3_C4 

    //calculate baud rate settings: ubd = UART module clock/16* baud rate
    ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));

    //clear SBR bits of BDH
    UART3_BDH &= ~UART_BDH_SBR_MASK;

    //distribute this ubd in BDH and BDL
    UART3_BDH |= (uint32_t) (ubd >> 8) & 0x1F & UART_BDH_SBR_MASK;
    UART3_BDL |= (uint32_t) ubd & UART_BDL_SBR_MASK;

    //BRFD = (1/32)*BRFA 
    //make the baud rate closer to the desired value by using BRFA
    brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

    //write the value of brfa in UART3_C4
    UART3_C4 |= brfa & UART_C4_BRFA_MASK;
        
    //Enable transmitter and receiver of UART
    UART3_C2 |= UART_C2_TE_MASK;
    UART3_C2 |= UART_C2_RE_MASK;
}

uint8_t getChar()
{
    /* Wait until there is space for more data in the receiver buffer*/
    while ((UART0_S1 & (1 << 5)) == 0);

    /* Return the 8-bit data from the receiver */
    return UART0_D;
}

void putChar(char ch)
{
    /* Wait until transmission of previous bit is complete */
    while ((UART0_S1 & (1 << 7)) == 0);
        
    /* Send the character */
    UART0_D = ch;
}

void put(char *ptr_str)
{
    while(*ptr_str)
    putChar(*ptr_str++);
}

void putnumU(int i)
{
    char num[10];
	sprintf( num, "%d", i );
	put( num );
}
