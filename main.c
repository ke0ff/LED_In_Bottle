/********************************************************************
 ************ COPYRIGHT (c) 2015 by ke0ff, Taylor, TX   *************
 *
 *  File name: main.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  This is the main code file for the LED-In-Bottle application.
 *  This device controls a tr-color (RGB) LED strip that is placed inside a
 *  frosted bottle.  The LEDs feature an open-drain FET switch that grounds
 *  the LED cathodes to turn them on.  The Port F 1, 2, and 3 I/O pins are used
 *  since these parallel the RGB LED that is present on the Tiva EK-TM4C123XL.
 *
 *  The host system is a Tiva EK-TM4C123XL LaunchPad Evaluation Module that
 *  is attached to a host board via the dual-row headers present on the module.
 *  The host board features a 5V power supply and connectors for power and the
 *  LED-In-Bottle.  The LED strip also features an I2C temperature sensor located
 *  at the top of the strip.  Another sensor is located on the host board.  There
 *  is also a 0.025ohm current shunt in series with the (-) power lead and an
 *  I2C ADC (ADS1014) that allows the system to measure the current draw at any
 *  given instant.
 *
 *  The PC serial connection runs at 115.2 Kbaud via the LP ICDI JTAG I.F.
 *  and the Stellaris USB serial port (using UART0).
 *
 *  The Tiva LP board features two user switches that may be used to change the behavior
 *  of the LEDs.  These switches can be captured in the process_IO() funtion and the
 *  resulting action translated to the LED controls.  An example would be to adjust
 *  the brightness and/or color of the LEDs.
 *
 *  Project scope revision history:
 *    <VERSION 0.1>
 *    10-16-15 jmh:  Adapted from HFB project.  Removed most of the irrelevant code and
 *    				  #defines.  S-Record and Xmodem code remains since these elements are
 *    				  difficult to sever.
 *    				 TA and TH code is drawn directly from the HFB, the ADS1014 code is
 *    				  new.
 *    10-16-15 jmh:  creation date
 *
 *******************************************************************/

/********************************************************************
 *  File scope declarations revision history:
 *    10-01-15 jmh:  creation date
 *
 *******************************************************************/

//-----------------------------------------------------------------------------
// main.c
//  Receives CLI entries and dispatches to the command line processor.
//  UART0 is used for user interface and data download at 115,200 baud
//	(with xon/xoff handshaking).
//
//  CLI is a simple maintenance/debug port with the following core commands:
//      VERS - interrogate SW version.
//		See "cmd_fn.c" for details
//
//  Interrupt Resource Map:
//      Timer2 int provides 1ms, count-down-from-N-and-halt application timers
//      UART0 is host CLI serial port (via the ICDI USB<->SERIAL circuit)
//
//  I/O Resource Map:
//      See "init.h"
//
//	Phase I SW: Basic functions:
//	1) PortF PWM: 3 PWM outputs individually control 3 LED strings via open-drain switches.
//	2) I2C devices are used to measure LED strip temperature, ambient temperature, and
//		the current shunt voltage (used to calculate the current draw of the system).
//	3) CLI via UART0 allows user to query system variables and execute test functions.
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
// compile defines

#define MAIN_C
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include <stdio.h>
#include <string.h>
#include "init.h"
#include "typedef.h"
#include "version.h"
#include "serial.h"
#include "cmd_fn.h"
#include "tiva_init.h"
#include "I2C0.h"
#include "io.h"

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

//  see init.h for main #defines
#define	MAX_REBUF	4		// max # of rebufs
#define	MAX_BARS	7		// max # of led bars
#define	GETS_INIT	0xff	// gets_tab() static initializer signal
#define DAC_LDAC 20			// dac spi defines
#define DAC_PWRUP 21
#define DAC_CLR 22

//-----------------------------------------------------------------------------
// Local Variables
//-----------------------------------------------------------------------------

// Processor I/O assignments
//bitbands
#define DUT_ON  (*(volatile uint32_t *)(0x40058000 + (0x04 * 4)))
#define DUT_OFF (*(volatile uint32_t *)(0x40058000 + (0x08 * 4)))
#define DUT_SCK (*(volatile uint32_t *)(0x40058000 + (0x10 * 4)))
#define DUT_D   (*(volatile uint32_t *)(0x40058000 + (0x20 * 4)))
#define DUT_Q   (*(volatile uint32_t *)(0x40058000 + (0x40 * 4)))
#define DUT_CS  (*(volatile uint32_t *)(0x40058000 + (0x80 * 4)))

//-----------------------------------------------------------------------------
// Global variables (extern conditionals are in init.h)
//-----------------------------------------------------------------------------
U16		app_timer1ms;					// app timer
U16		xm_timer;						// xmodem timer
char	bchar;							// break character trap register - traps ESC ascii chars entered at terminal
char	swcmd;							// software command flag
S8		handshake;						// xon/xoff enable
S8		xoffsent;						// xoff sent

//-----------------------------------------------------------------------------
// Local variables in this file
//-----------------------------------------------------------------------------
U16		procio_timer1ms;				// process IO timer
U32		abaud;							// 0 = 115.2kb (the default)
U8		iplt2;							// timer2 ipl flag
U16		waittimer;						// gp wait timer
S8		err_led_stat;					// err led status
uint8_t		idx;
U16		ipl;							// initial power on state
U16		kupper;							// ADC min/max tracking regs
U16		klower;
U8		ksamp;							// isample period expired
U8		lrun;							// auto-sweep enable

//-----------------------------------------------------------------------------
// Local Prototypes
//-----------------------------------------------------------------------------

void Timer_Init(void);
void Timer_SUBR(void);
char *gets_tab(char *buf, char *save_buf[3], int n);
void init_adc(void);

//*****************************************************************************
// main()
//  The main function runs a forever loop in which the main application operates.
//	Prior to the loop, Main performs system initialization and boot status
//	announcement.
//	The loop calls gets_tab() which polls for user input and runs the process loops (if any).
//	When gets_tab() returns, it means that the user has pressed "Enter" and thier input
//	is ready to be processed by the cmd_fn routines.
//	The loop also processes baud rate changes trapped by gets_tab().
//
//	The CLI maintains and processes re-do buffers (4 total) that are accessed by the
//	TAB key.  This allows the last 4 valid command lines to be recalled and executed
//	(after TAB'ing to desired recall command, press ENTER to execute, or ESC to
//	clear command line).
//	Autobaud rate reset allows user to select alternate baudarates after reset:
//		115,200 baud is default.  A CR entered after reset at 57600, 38400,
//		19200, or 9600 baud will reset the system baud rate and prompt for user
//		acceptance.  Once accepted, the baud rate is frozen.  If rejected, baud rate
//		returns to 115200.  The first valid command at the default rate will also
//		freeze the baud rate.  Once frozen, the baud rate can not be changed until
//		system is reset.
//*****************************************************************************
int
main(void)
{
	volatile uint32_t ui32Loop;
    char	buf[80];				// command line buffer
    char	rebuf0[80];				// re-do buffer#1
    char	rebuf1[80];				// re-do buffer#2
    char	rebuf2[80];				// re-do buffer#3
    char	rebuf3[80];				// re-do buffer#4
    char	got_cmd = FALSE;		// first valid cmd flag (freezes baud rate)
    U8		argn;					// number of args
    char*	cmd_string;				// CLI processing ptr
    char*	args[ARG_MAX];			// ptr array into CLI args
    char*	rebufN[4];				// pointer array to re-do buffers
    U16		offset = 0;				// srecord offset register
    U16		cur_baud = 0;			// current baud rate

    ipl = proc_init();								// initialize the processor I/O
    rebufN[0] = rebuf0;								// init CLI re-buf pointers
	rebufN[1] = rebuf1;
	rebufN[2] = rebuf2;
	rebufN[3] = rebuf3;
	I2C_Init();										// init I2C system
	dispSWvers(); 									// display reset banner
	wait(10);										// a bit of delay..
	rebuf0[0] = '\0';								// clear cmd re-do buffers
	rebuf1[0] = '\0';
	rebuf2[0] = '\0';
	rebuf3[0] = '\0';
	bcmd_resp_init();								// init bcmd response buffer
	wait(10);										// a bit more delay..
	while(gotchr()) getchr();						// clear serial input in case there was some POR garbage
	gets_tab(buf, rebufN, GETS_INIT);				// initialize gets_tab()
	process_IO(0xff);								// init process_io
	swcmd = 0;										// init SW command
	led_runstat(FALSE);								// halt sweep
	// main loop
    while(swcmd != SW_ESC){
		putchar_b(XON);
		buf[0] = '\0';
		putss("led>");										// prompt
		cmd_string = gets_tab(buf, rebufN, 80); 			// get cmd line & save to re-do buf
		if(!got_cmd){										// if no valid commands since reset, look for baud rate change
			if(cur_baud != abaud){							// abaud is signal from gets_tab() that indicates a baud rate change
				if(set_baud(abaud)){						// try to set new baud rate
					puts0("");								// move to new line
					dispSWvers();							// display reset banner & prompt to AKN new baud rate
					while(gotchr()) getchr();				// clear out don't care characters
					putss("press <Enter> to accept baud rate change: ");
					while(!gotchr());						// wait for user input
					puts0("");								// move to new line
					if(getchr() == '\r'){					// if input = CR
						cur_baud = abaud;					// update current baud = new baud
						got_cmd = TRUE;						// freeze baud rate
					}else{
						set_baud(0);						// input was not a CR, return to default baud rate
						cur_baud = abaud = 0;
					}
				}else{
					abaud = cur_baud;						// new baud rate not valid, ignore & keep old rate
				}
			}else{
				got_cmd = TRUE;								// freeze baud rate (@115.2kb)
			}
		}
		argn = parse_args(cmd_string,args);					// parse cmd line
		if(x_cmdfn(argn, args, &offset)) got_cmd = TRUE;	// process cmd line, set got_cmd if cmd valid
    }
    return 0;
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// gets_tab puts serial input into buffer, UART0.
//-----------------------------------------------------------------------------
// Main loop for command line input.
// waits for a chr and puts into buf.  If 1st chr = \t, copy re-do buf into
//  cmdbuf and cycle to next re-do buf.  if more than n chrs input, nul term buf,
//	disp "line too long", and return.  if \n or \r, copy buf to save_buf & return
//  returns buf (pointer).
//	if n == 0xff, initialize statics and exit.
//
//	11/08/13: Modified to support 4 (MAX_REBUF) rolling cmd save buffers
//	11/15/13: Modified to support auto-baud detect on CR input
//		For the following, each bit chr shown is one bit time at 115200 baud (8.68056us).
//			s = start bit (0), p = stop bit (1), x = incorrect stop, i = idle (1), bits are ordered  lsb -> msb:
//	 the ascii code for CR = 10110000
//			At 115.2kb, CR = s10110000p = 0x0D
//
//			At 57.6 kb, CR = 00110011110000000011 (1/2 115.2kb)
//			@115.2, this is: s01100111ps00000001i = 0xE6, 0x80
//
//			At 38.4 kb, CR = 000111000111111000000000000111 (1/3 115.2kb)
//			@115.2, this is: s00111000p11111s00000000xxxiii = 0x1c, 0x00
//
//			At 19.2 kb, CR = 000000111111000000111111111111000000000000000000000000111111 (1/6 115.2kb)
//			@115.2, this is: s00000111piis00000111piiiiiiiis00000000xxxxxxxxxxxxxxxiiiiii = 0xE0, 0xE0, 0x00
//
//			At 9600 b,  CR = 000000000000111111111111000000000000111111111111111111111111000000000000000000000000000000000000000000000000111111111111 (1/12 115.2kb)
//			@115.2, this is: s00000000xxxiiiiiiiiiiiis00000000xxxiiiiiiiiiiiiiiiiiiiiiiiis00000000xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxiiiiiiiiiiii = 0x00, 0x00, 0x00
//
//		Thus, @ 57.6 kb, a CR = 0xE6 followed by 0x80
//			  @ 38.4 kb, a CR = 0x1C followed by 0x00
//			  @ 19.2 kb, a CR = 0xE0 followed by 0xE0 (plus a 0x00)
//			  @ 9600 b, a  CR = 0x00 followed by 0x00 (plus a 0x00)
//
//		NOTE: gets_tab is only used for command line input and thus should not
//		see non-ascii data under normal circumstances.

char *gets_tab(char *buf, char *save_buf[], int n)
{
	char	*cp;
	char	*sp;
	char	c;
	int		i = 0;
	static	U8   rebuf_num;
	static	U8	 last_chr;

//	if((rebuf_num >= MAX_REBUF) || (n == GETS_INIT)){ // n == 0xff is static initializer signal
	if(n == GETS_INIT){ // n == 0xff is static initializer signal
		rebuf_num = 0;									// init recall buffer pointer
		last_chr = 0xff;								// init to 0xff (not a valid baud select identifier chr)
		return buf;										// skip rest of Fn
	}
    cp = buf;
    sp = save_buf[rebuf_num];
    do{
        c = getch00();									// look for chrs and run process_IO()
        switch(c){
			case 0xE0:									// look for 19.2kb autoselect
				if(last_chr == 0xE0){
					abaud = 19200L;
					c = '\r';
				}
				break;

			case 0x00:									// look for 38.4kb or 9600b autoselect
				if(last_chr == 0x1C){
					abaud = 38400L;
					c = '\r';
				}else{
					if(last_chr == 0x00){
						abaud = 9600L;
						c = '\r';
					}
				}
				break;

			case 0x80:									// look for 57.6kb autoselect
				if(last_chr == 0xE6){
					abaud = 57600L;
					c = '\r';
				}
				break;

            case '\t':
				if(i != 0){								// if tab, cycle through saved cmd buffers
					do{
						i--;							// update count/point
						cp--;
						if((*cp >= ' ') && (*cp <= '~')){
							putchar0('\b');				// erase last chr if it was printable
							putchar0(' ');
							putchar0('\b');
						}
					}while(i != 0);
					cp = buf;							// just in case we got out of synch
				}
				//copy saved string up to first nul, \n, or \r
				if(--rebuf_num == 0xff){
					rebuf_num = MAX_REBUF - 1;
				}
				sp = save_buf[rebuf_num];
				while((*sp != '\0') && (*sp != '\r') && (*sp != '\n')){
					putdch(*sp);
					*cp++ = *sp++;
					i++;
				}
                break;

            case '\b':
            case 0x7f:
                if(i != 0){								// if bs & not start of line,
                    i--;								// update count/point
                    cp--;
                    if((*cp >= ' ') && (*cp <= '~')){
                        putchar0('\b');					// erase last chr if it was printable
                        putchar0(' ');
                        putchar0('\b');
                    }
                }
                break;

            case '\r':									// if cr, nul term buf & exit
            case '\n':									// if nl, nul term buf & exit
                i++;
                *cp++ = c;
                break;

            case ESC:									// if esc, nul buf & exit
                cp = buf;
                c = '\r';								// set escape condition
				i = 0;
                break;

            default:
                i++;
                *cp++ = c;								// put chr in buf
                putdch(c);								// no cntl chrs here
                break;
        }
		last_chr = c;									// set last chr
    } while((c != '\r') && (c != '\n') && (i < n));		// loop until c/r or l/f or buffer full
	if(i >= n){
		puts0("!! buffer overflow !!");
		*buf = '\0';									// abort line
	}else{
		puts0("");										// echo end of line to screen
		*cp = '\0';										// terminate command line
		if((*buf >= ' ') && (*buf <= '~')){				// if new buf not empty (ie, 1st chr = printable),
			strncpy(save_buf[rebuf_num], buf, n);		// copy new buf to save
			if(++rebuf_num >= MAX_REBUF) rebuf_num = 0;
		}
	}
    return buf;
}

//-----------------------------------------------------------------------------
// process_IO() processes system I/O
//	this fn is called while the system is waiting for user input and does all of the
//	tasks that need to happen while not dealing with a user command.
//	This is where any automatic activities are placed.  A system timer should be used
//	to pace activities.  Also, state machine loops are generally the easiest way to
//	break tasks into pieces that allow this Fn to execute without tying up CPU time
//	for indefinite periods.
//-----------------------------------------------------------------------------
void process_IO(U8 flag){
#define	ISAMP_DLY	250								// (ms) min/max sample period
	U16	k;			// adc temp

	switch(flag){
	case 0xff:
	default:
		init_adc();									// init ADC
		kupper = 0;									// init upper to minimum
		klower = ADC_MAX;							// init lower to maximum
		break;

	case 0:
		if(procio_timer1ms == 0){
			procio_timer1ms = ISAMP_DLY;			// reset the sample timer
			k = I2C_Recv2(ADDR_ADC) * -1;			// we have to do this because the Isense resistor is wired backwards
			if(k>kupper) kupper = k;				// if value is new maximum, capture it
			if(k<klower) klower = k;				// if value is new minimum, capture it
			ksamp = TRUE;							// set sample period complete
		}
		if((GPIO_PORTF_DATA_R & SW1) == 0){
			led_runstat(TRUE);						// start sweep
		}
		if((GPIO_PORTF_DATA_R & SW2) == 0){
			led_runstat(FALSE);						// stop sweep
		}
		break;
	}
	return;
}

//-----------------------------------------------------------------------------
// get_imin() returns Imin value in float units of mA
//	Converts ADC reading to Rsense current in mA
//	Assumes that the ADC is set with the PGA at max gain (Vfull-scale = +/-0.256V)
//	and Rsense = 0.025 ohms: I = 1000 * V/R gives mA
//	Vadc = (ADC max V) * (ADC raw value (k)) / (ADC maximum value (for this ADC, 32768))
//	If flag == TRUE, reset the lower value to it's starting point
//-----------------------------------------------------------------------------
float get_imin(U8 flag){
	float f;		// float temp

	f = 256.0 * (float)klower/32768.0;		// calculate ADC voltage (mV)
	f = f/0.025;							// calculate current (mA)
	if(flag) klower = ADC_MAX;				// if flag, reset klower
	return f;
}

//-----------------------------------------------------------------------------
// get_imax() returns Imax value in float units of mA
//	Converts ADC reading to Rsense current in mA
//	Assumes that the ADC is set with the PGA at max gain (Vfull-scale = +/-0.256V)
//	and Rsense = 0.025 ohms: I = 1000 * V/R gives mA
//	Vadc = (ADC max V) * (ADC raw value (k)) / (ADC maximum value (for this ADC, 32768))
//	If flag == TRUE, reset the upper value to it's starting point
//-----------------------------------------------------------------------------
float get_imax(U8 flag){
	float f;		// float temp

	f = 256.0 * (float)kupper/32768.0;		// calculate ADC voltage (mV)
	f = f/0.025;							// calculate current (mA)
	if(flag) kupper = 0;					// if flag, reset kupper
	return f;
}

//-----------------------------------------------------------------------------
// get_isamp() returns TRUE if ksamp == true
//	set ksamp to FALSE before exit.
//-----------------------------------------------------------------------------
U8 get_isamp(void){
	U8 i = ksamp;	// rtn temp

	ksamp = FALSE;							// if flag, reset kupper
	return i;
}

//-----------------------------------------------------------------------------
// init_adc() initializes I2C ADC.
//-----------------------------------------------------------------------------
void init_adc(void){

	I2C_Send3(ADDR_ADC, 0x01, ADC_CONFIGH, ADC_CONFIGL);
	I2C_Send1(ADDR_ADC, 0x00);
	return;
}

//-----------------------------------------------------------------------------
// waitpio() uses a dedicated ms timer to establish a defined delay (+/- 1LSB latency)
//	loops through process_IO during wait period.
//-----------------------------------------------------------------------------
void waitpio(U16 waitms){
//	U32	i;

//	i = 545 * (U32)waitms;
    waittimer = waitms;
//    for(;i!=0;i--);		// patch
    while(waittimer != 0) process_IO(0);		// wait for the waittimer, run process_IO() in the mean time
}

//-----------------------------------------------------------------------------
// wait() uses a dedicated ms timer to establish a defined delay (+/- 1LSB latency)
//	waittimer is a 1ms count-down from "N" and halt timer that is processed by
//	the timer 2 interrupt.
//-----------------------------------------------------------------------------
void wait(U16 waitms)
{
//	U32	i;

//	i = 545 * (U32)waitms;
    waittimer = waitms;
//    for(;i!=0;i--);		// patch
    while(waittimer != 0);
}

//-----------------------------------------------------------------------------
// wait2() does quick delay pace.  This loop is affected by the SYSCLK setting
//	and will thus change as SYSCLK is changed.  !!! PLATFORM DEPENDENT !!!
//-----------------------------------------------------------------------------
void wait2(U16 waitms)
{
	U32	i;

	i = 10 * (U32)waitms;
    waittimer = waitms;
    for(;i!=0;i--);		// patch
//    while(waittimer != 0);
}

//-----------------------------------------------------------------------------
// wait_reg0() waits for (delay timer == 0) or (regptr* & clrmask == 0)
//	if delay expires, return TRUE, else return FALSE
//	allows calling function to poll a register bit(s) with a defined timeout
//	this prevents the system from locking up if the desired bit behavior does
//	not occur.  Returns TRUE if bit(s) fails to transition to 0
//-----------------------------------------------------------------------------
U8 wait_reg0(volatile uint32_t *regptr, uint32_t clrmask, U16 delay){
	U8 timout = FALSE;

    waittimer = delay;
    while((waittimer) && ((*regptr & clrmask) != 0));
    if(waittimer == 0) timout = TRUE;
    return timout;
}

//-----------------------------------------------------------------------------
// wait_reg1() waits for (delay timer == 0) or (regptr* & setmask == setmask)
//	if delay expires, return TRUE, else return FALSE
//	allows calling function to poll a register bit(s) with a defined timeout
//	this prevents the system from locking up if the desired bit behavior does
//	not occur.  Returns TRUE if bit(s) fails to transition to 1
//-----------------------------------------------------------------------------
U8 wait_reg1(volatile uint32_t *regptr, uint32_t setmask, U16 delay){
	U8 timout = FALSE;

    waittimer = delay;
    while((waittimer) && ((*regptr & setmask) != setmask));
    if(waittimer == 0) timout = TRUE;
    return timout;
}

//-----------------------------------------------------------------------------
// getipl() returns current ipl flags value
//-----------------------------------------------------------------------------
U16 getipl(void){

	return ipl;
}

//-----------------------------------------------------------------------------
// led_runstat() sets/clears lrun
//-----------------------------------------------------------------------------
void led_runstat(U8 flag){

	if(flag) lrun = TRUE;
	else lrun = FALSE;
	return;
}

//-----------------------------------------------------------------------------
// led_runstat() sets/clears lrun
//-----------------------------------------------------------------------------
void led_rgb(S16 ledred, S16 ledblu, S16 ledgrn){

	PWM1_2_CMPB_R = ledred;						// set PWM compare regsiters (sets duty cycle)
	PWM1_3_CMPA_R = ledblu;
	PWM1_3_CMPB_R = ledgrn;
	return;
}

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
//
// Called when timer2 A overflows (NORM mode):
//	intended to update app timers @ 1ms per lsb.  app timers are "count-down-and-
//		halt".  Fns write a time value to a particular timer variable, then poll
//		the variable until it reaches 0.  The counting then stops until the variable
//		is written with another value.  Note that there is a granularity error
//		of 1 ms.  This is due to the fact that all of the timers are updated on
//		the same 1ms cadence.  This means that the actual time value that expires
//		once the timer variable is written is +0/-1 ms from the value that was
//		written.  This is because there is no provision for re-setting the timer2
//		counters.  Fns may "synchronize" by writing a value of 1 to one of the
//		timer variables, then wait for that to reach 0.  At that point, the Fn
//		is synchronized to the timer2 divider chain, and subsequent timer variable
//		interactions are likely to be very close to the full value desired.
//		This is generally not an issued except for timer values that are very near
//		to 1ms (approx 10ms or less) or timing functions that require a great deal
//		of precision (such as time of day, or pulse width measurements.  These
//		applications should use a dedicated timer to provide the desired precision.
//	Also drives RGB "I'm alive" LED.  The LED transitions through the color
//	wheel in a way that can be modified by the #defines below. RATE, PERIOD,
//	and START values may be adjusted to taylor the color transitions,
//	transition rate, and cycle period.
//
//-----------------------------------------------------------------------------

void Timer2_ISR(void)
{
static	U16	prescale;				// prescales the ms rate to the LED update rate
static	U16	period_counter;			// counts the LED cycle period (ms)
static	S16	ired;					// PWM compare holding regs (RGB)
static	S16	iblu;
static	S16	igrn;
static	S16	deltar;					// delta DC holding regs (RGB)
static	S16	deltag;
static	S16	deltab;
static	U8	cycflag;				// cycle control flag
#define	PWM_RATE_RED	2			// delta duty cycle values (RGB) (this is added/subtracted to/fr the DCreg every 10ms)
#define	PWM_RATE_GRN	1
#define	PWM_RATE_BLU	3
#define	LED_PERIOD		6000		// sets length of LED cycle (ms)
#define	LED_GRN_STRT	500			// sets start of green sub-cycle (ms)\___ red starts at 0ms
#define	LED_GRN_FLG		0x01		// green cycle started bitmap
#define	LED_BLU_STRT	1000		// sets start of blue sub-cycle (ms) /
#define	LED_BLU_FLG		0x02		// blue cycle started bitmap

	GPIO_PORTE_DATA_R ^= PORTE_SPR5;		// toggle debug pin
	TIMER2_ICR_R = TIMER2_MIS_R;
	if(lrun){
		if((iplt2) || (++period_counter > LED_PERIOD)){	// if flag is set, perform ipl initialization
			prescale = 0;								// also, do this every LED cycle
			ired = PWM_ZERO - 1;						// set all colors to 0 level
			igrn = PWM_ZERO - 1;
			iblu = PWM_ZERO - 1;
	       	PWM1_2_CMPB_R = ired;						// set PWM compare regsiters (sets duty cycle)
	       	PWM1_3_CMPA_R = iblu;
	       	PWM1_3_CMPB_R = igrn;
			deltar = -PWM_RATE_RED;						// init delta registers
			deltag = 0;
			deltab = 0;
			period_counter = 0;							// reset period count
			iplt2 = 0;									// clear IPL flag
			cycflag = 0;
		}
	    if((++prescale > SEC10MS) && lrun){				// prescale sets PWM update period = 10ms
	    	if((period_counter > LED_GRN_STRT) && !(cycflag & LED_GRN_FLG)){
	    		deltag = -PWM_RATE_GRN;			// start green
	    		cycflag |= LED_GRN_FLG;			// set cycle started GRN
	    	}
	    	if((period_counter > LED_BLU_STRT) && !(cycflag & LED_BLU_FLG)){
	    		deltab = -PWM_RATE_BLU;			// start blue
	    		cycflag |= LED_BLU_FLG;			// set cycle started BLU
	    	}
	    	ired += deltar;						// adjust the LED PWM values
	    	igrn += deltag;
	    	iblu += deltab;
	    	if(ired < PWM_MAX){					// if red is max, start back to to zero
	    		ired = PWM_MAX;
	    		deltar = PWM_RATE_RED;
	    	}
	    	if(ired >= PWM_ZERO){				// if red is past zero, set to 0 and set delta to 0
	    		ired = PWM_ZERO-1;
	    		deltar = 0;
	    	}
	    	if(igrn < PWM_MAX){					// if grn is max, start back to to zero
	    		igrn = PWM_MAX;
	    		deltag = PWM_RATE_GRN;
	    	}
	    	if(igrn >= PWM_ZERO){				// if grn is past zero, set to 0 and set delta to 0
	    		igrn = PWM_ZERO-1;
	    		deltag = 0;
	    	}
	    	if(iblu < PWM_MAX){					// if blu is max, start back to to zero
	    		iblu = PWM_MAX;
	    		deltab = PWM_RATE_BLU;
	    	}
	    	if(iblu >= PWM_ZERO){				// if blu is past zero, set to 0 and set delta to 0
	    		iblu = PWM_ZERO-1;
	    		deltab = 0;
	    	}
	       	PWM1_2_CMPB_R = ired;				// set PWM compare regsiters (sets duty cycle)
	       	PWM1_3_CMPA_R = igrn;
	       	PWM1_3_CMPB_R = iblu;
	        prescale = 0;
	    }
	}
    if (app_timer1ms != 0){					// update app timer
        app_timer1ms--;
    }
    if (procio_timer1ms != 0){				// update procio timer
    	procio_timer1ms--;
    }
    if (waittimer != 0){					// update wait timer
        waittimer--;
    }
    if (xm_timer != 0){						// update xmodem timer
        xm_timer--;
    }
	GPIO_PORTE_DATA_R ^= PORTE_SPR5;		// toggle debug pin
}
//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------

