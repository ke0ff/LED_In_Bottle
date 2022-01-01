/********************************************************************
 ************ COPYRIGHT (c) 2015 by ke0ff, Taylor, TX   *************
 *
 *  File name: init.h
 *
 *  Module:    Control
 *
 *  Summary:   defines and global declarations for main.c
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>

#ifndef INIT_H
#define INIT_H
#endif

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

//#define EXTXTAL           	// un-comment if external xtal is used

#ifdef EXTXTAL
#define XTAL 1
#else
#define XTAL 0
#endif

#define	ADC_MAX	32767			// ADC max positive value

#define OSC_LF 4            	// osc clock selects
#define OSC_HF 0
#define OSC_EXT 1

#define SEC10MS    10           // timer constants (ms)
#define SEC50MS    50           // timer constants (ms)
#define SEC75MS    74
#define SEC100MS  100
#define SEC250MS  250
#define SEC500MS  500
#define SEC750MS  750
#define SEC1     1000
#define ONESEC   SEC1
#define SEC2     2000
#define SEC3     3000
#define SEC5     5000
#define SEC10   10000
#define SEC15   15000
#define SEC30   30000
#define SEC60   60000
#define SEC300 300000L
#define	ONEMIN	(SEC60)
#define	REG_WAIT_DLY 200	// 200ms wait limit for register action

// timer definitions.  Uses EXTXTAL #def to select between ext crystal and int osc
//  for normal mode.
// SYSCLK value in Hz
#define SYSCLKL 10000L
#define SYSCLK	(50000000L)			// sysclk freq (bus clk)
#define PIOCLK	(16000000L)			// internal osc freq
#define TIMER2_PS 32
#define	TPULSE	(100L)				// in usec
#define TMIN	(((SYSCLK / 100) * TPULSE)/10000L)	// minimum pulse width

// Port A defines
#define TXD0			0x01		// out
#define RXD0			0x02		// in
#define DIR1			0x04		// out
#define FANON			0x08		// out
#define ANTON			0x10		// out
#define NANTDN			0x20		// out
#define N6MON			0x40		// out
#define NUHFON			0x80		// out
#define PORTA_DIRV		(TXD0|DIR1|FANON|ANTON|NANTDN|N6MON|NUHFON)
#define	PORTA_DENV		(TXD0|RXD0|DIR1|FANON|ANTON|NANTDN|N6MON|NUHFON)
#define	ANT_OFF			0
#define	ANT_UP			1
#define	ANT_DN			2
#define	ANT_QRY			0xfe

// Port B defines
#define RXD1			0x01		// in
#define TXD1			0x02		// out
#define SCL				0x04		// out
#define SDA				0x08		// i/o
#define NFANOPR			0x10		// in
#define PEPM			0x20		// out
#define T0P0			0x40		// in
#define WAKE			0x80		// in
#define PORTB_DIRV		(TXD1|SCL|PEPM)
#define	PORTB_DENV		(RXD1|TXD1|SCL|SDA|NFANOPR|T0P0|PEPM|WAKE)
#define	PORTB_PURV		(WAKE)

// Port C defines
#define C1M 			0x10
#define C1P 			0x20
#define C0P 			0x40
#define C0M 			0x80
#define PORTC_DIRV		(0)
#define	PORTC_DENV		(0)

// Port D defines
#define LOWRG			0x01		// out
#define NMANTA			0x02		// in
#define NMANTB			0x04		// in
#define NHFON			0x08		// in
#define PORTD_SPR4		0x10		// out
#define PORTD_SPR5		0x20		// out
#define RXD2			0x40
#define TXD2			0x80
#define PORTD_DIRV		(LOWRG|PORTD_SPR4|PORTD_SPR5|TXD2)
#define	PORTD_DENV		(LOWRG|NMANTA|NMANTB|NHFON|PORTD_SPR4|PORTD_SPR5|RXD2|TXD2)
#define	PORTD_PURV		(NMANTA|NMANTB)

#define MANUP			2
#define	MANDN			1
#define	MANOFF			0

// Port E defines
#define NSPARE			0x01		// in
#define PRF_ANA			0x02		// anin
#define PFF_ANA			0x04		// anin
#define IM_ANA			0x08		// anin
#define PORTE_SPR4		0x10		// out
#define PORTE_SPR5		0x20		// out
#define	IM_CHAN			0
#define	PRF_CHAN		1
#define	PFF_CHAN		2
#define PORTE_DIRV		(PORTE_SPR4|PORTE_SPR5)
#define	PORTE_DENV		(NSPARE|PORTE_SPR4|PORTE_SPR5)
#define	PORTE_PURV		(NSPARE)

// Port F defines
#define SW1				0x01
#define LEDR			0x02
#define LEDG			0x04
#define LEDB			0x08
#define SW2				0x10
#define PORTF_DIRV		(LEDR|LEDG|LEDB)
#define	PORTF_DENV		(SW1|LEDR|LEDG|LEDB|SW2)

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

#ifndef MAIN_C
extern U16	app_timer1ms;		// app timer
extern U16	xm_timer;			// xmodem timer
extern char	bchar;				// global bchar storage
extern char	swcmd;				// global swcmd storage
extern S8	handshake;			// xon/xoff enable
extern S8	xoffsent;			// xoff sent
#endif


//-----------------------------------------------------------------------------
// main.c Fn prototypes
//-----------------------------------------------------------------------------

void Init_Device(void);
void process_IO(U8 flag);
void waitpio(U16 waitms);
void wait(U16 waitms);
void wait2(U16 waitms);
U16 getipl(void);
float get_imin(U8 flag);
float get_imax(U8 flag);
U8 get_isamp(void);
U8 wait_reg0(volatile uint32_t *regptr, uint32_t clrmask, U16 delay);
U8 wait_reg1(volatile uint32_t *regptr, uint32_t setmask, U16 delay);
void led_runstat(U8 flag);
void led_rgb(S16 ledred, S16 ledblu, S16 ledgrn);
void Timer2_ISR(void);

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
