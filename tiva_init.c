/********************************************************************
 ************ COPYRIGHT (c) 2015 by ke0ff, Taylor, TX   *************
 *
 *  File name: tiva_init.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  Tiva processor init functions
 *
 *******************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions
#include "tiva_init.h"
#include "serial.h"
#include "PLL.h"

#include "version.h"

//=============================================================================
// local registers


//=============================================================================
// local Fn declarations


//*****************************************************************************
// proc_init()
//  initializes the processor I/O peripherals
//	returns bitmapped initialize result status as U16
//
//*****************************************************************************
U16 proc_init(void){
	volatile uint32_t ui32Loop;
	U16	ipl;					// initialize response value
//	uint32_t i;

	// init PLL
	PLL_Init(SYSCLK);
    // Enable the GPIO port clocks.
    SYSCTL_RCGCGPIO_R = PORTF|PORTE|PORTD|PORTC|PORTB|PORTA;

    // Do a dummy read to insert a few cycles after enabling the peripheral.
    ui32Loop = SYSCTL_RCGCGPIO_R;

    // Enable the GPIO pins.
    GPIO_PORTF_LOCK_R = 0x4C4F434B;										// unlock PORTF
    GPIO_PORTF_CR_R = 0xff;
    GPIO_PORTF_DIR_R = PORTF_DIRV;
    GPIO_PORTF_DEN_R = PORTF_DENV;
    GPIO_PORTF_PUR_R = SW1 | SW2;
	GPIO_PORTF_AFSEL_R = 0;
    GPIO_PORTE_DIR_R = PORTE_DIRV;
    GPIO_PORTE_DEN_R = PORTE_DENV;
    GPIO_PORTE_PUR_R = PORTE_PURV;
    GPIO_PORTD_LOCK_R = 0x4C4F434B;										// unlock PORTD
    GPIO_PORTD_CR_R = 0xff;
    GPIO_PORTD_DIR_R = PORTD_DIRV;
    GPIO_PORTD_DEN_R = PORTD_DENV;
    GPIO_PORTD_PUR_R = PORTD_PURV;
    GPIO_PORTC_DIR_R &= 0x0f;											// preserve JTAG pin assignments
    GPIO_PORTC_DEN_R &= 0x0f;
    GPIO_PORTB_DIR_R = PORTB_DIRV;										//0x32
    GPIO_PORTB_DEN_R = PORTB_DENV;										//0xb0;
    GPIO_PORTB_PUR_R = PORTB_PURV;
    GPIO_PORTA_DIR_R = PORTA_DIRV;
    GPIO_PORTA_DEN_R = PORTA_DENV;
    GPIO_PORTF_DATA_R = 0x00;
    GPIO_PORTE_DATA_R = 0x00;
    GPIO_PORTD_DATA_R = 0x00;
    GPIO_PORTC_DATA_R = 0x00;
    GPIO_PORTB_DATA_R = 0x00;
    GPIO_PORTA_DATA_R = 0x00;

	// init timer2A (appl timer, count down, no GPIO)
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    ui32Loop = SYSCTL_RCGCTIMER_R;
	TIMER2_CTL_R &= ~(TIMER_CTL_TAEN);									// disable timer
	TIMER2_CFG_R = 0x4;
	TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
	TIMER2_TAPR_R = (TIMER2_PS - 1);									// prescale reg = divide ratio - 1
	TIMER2_TAILR_R = (uint16_t)(SYSCLK/(1000 * TIMER2_PS));
	TIMER2_IMR_R = TIMER_IMR_TATOIM;									// enable timer intr
	TIMER2_CTL_R |= (TIMER_CTL_TAEN);									// enable timer
	TIMER2_ICR_R = TIMER2_MIS_R;
	NVIC_EN0_R |= 0x00800000L;											// enable timer intr in the NVIC
    ipl = IPL_TIMER2INIT;

	// init PWMs on PF1-3
	SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    ui32Loop = SYSCTL_RCGCPWM_R;										// delay a few cycles
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    ui32Loop = SYSCTL_RCGCGPIO_R;										// delay a few cycles
    GPIO_PORTF_AFSEL_R |= LEDR|LEDG|LEDB;								// enable alt fn, PF1-3
	GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF3_M|GPIO_PCTL_PF2_M|GPIO_PCTL_PF1_M);
	GPIO_PORTF_PCTL_R |= (GPIO_PCTL_PF3_M1PWM7|GPIO_PCTL_PF2_M1PWM6|GPIO_PCTL_PF1_M1PWM5);
	SYSCTL_RCC_R = (SYSCTL_RCC_R & ~SYSCTL_RCC_PWMDIV_M) | (PWM_DIV << 17) | SYSCTL_RCC_USEPWMDIV;
	PWM1_2_CTL_R = 0;
	PWM1_2_GENB_R = PWM_2_GENB_ACTCMPBD_ZERO|PWM_2_GENB_ACTLOAD_ONE;	//0x80c;
	PWM1_2_LOAD_R = PWM_ZERO;											// 4KHz at SYSCLK = 16MHz (2000-1);
	PWM1_2_CMPB_R = PWM_ZERO - 1;										//1900;
	PWM1_3_CTL_R = 0;
	PWM1_3_GENA_R = PWM_3_GENA_ACTCMPAD_ZERO|PWM_3_GENA_ACTLOAD_ONE;	//0x8c;
	PWM1_3_GENB_R = PWM_3_GENB_ACTCMPBD_ZERO|PWM_3_GENB_ACTLOAD_ONE;	//0x80c;
	PWM1_3_LOAD_R = PWM_ZERO; 											// 4KHz at SYSCLK = 16MHz (2000-1);
	PWM1_3_CMPA_R = PWM_ZERO - 1;										//1499;
	PWM1_3_CMPB_R = PWM_ZERO - 1;										//1499;
	PWM1_2_CTL_R = PWM_2_CTL_ENABLE;									//1;
	PWM1_3_CTL_R = PWM_3_CTL_ENABLE;									//1;
	PWM1_ENABLE_R = PWM_ENABLE_PWM7EN|PWM_ENABLE_PWM6EN|PWM_ENABLE_PWM5EN;	//0xe0;
    ipl |= IPL_PWM1INIT;

	// init UARTs
	initserial();						// init UART0-2
	process_xrx(RX_STATE_CLEAR);		// init xmodem receive
	NVIC_EN0_R = 0x0020L;				// enable UART0 intr
    ipl |= IPL_UART0INIT;
	return ipl;
}
