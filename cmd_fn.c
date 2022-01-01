/********************************************************************
 ************ COPYRIGHT (c) 2015 by ke0ff, Taylor, TX   *************
 *
 *  File name: cmd_fn.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  CLI Command Interpreter
 *  
 *******************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions

#include "cmd_fn.h"						// fn protos, bit defines, rtn codes, etc.
#include "serial.h"
#include "version.h"
#include "I2C0.h"
#include "io.h"

//=============================================================================
// local registers

U16	string_addr;						// string-parse next empty address
U16 sernum;								// serial number utility register
#define MAX_PRESP_BUF 80
char bcmd_resp_buf[MAX_PRESP_BUF + 10];
char* bcmd_resp_ptr;

// enum error message ID
enum err_enum{ no_response, no_device, target_timeout };

// enum list of command numerics
//	each enum corresponds to a command from the above list (lastcmd corresponds
//	with the last entry, 0xff)
enum cmd_enum{ adc_tst,log_data,tst_tempa,tst_templ,timer_tst,ztest,help1,help2,vers,lastcmd,helpcmd };

#define	cmd_type	char	// define as char for list < 255, else define as int

//=============================================================================
// local Fn declarations

U8 get_Dargs(U8 argsrt, U8 nargs, char* args[ARG_MAX], U16 params[ARG_MAX]);
cmd_type cmd_srch(char* string);
char do_cmd_help(U8 cmd_id);
char parm_srch(U8 nargs, char* args[ARG_MAX], char* parm_str);
void parse_ehex(char * sptr);
void disp_error(U8 errnum);
void disp_fail(char* buf, char* s, U16 c, U16 d);
void disp_wait_addr(char* buf);
U16 boot_se(U8* bptr, U16 start, U16 end, U16 ppaddr);
U8* log_error_byte(U8* lbuf, U8 d, U8 h, U16 a);
void disp_error_log(U8* lbuf, U16 len);
void do_help(void);
void disp_esc(char flag);
void puts_res(void);

//=============================================================================
// CLI cmd processor entry point
//	Uses number of args (nargs) and arg pointer array (args[]) to lookup
//	command and execute.
//	offset is srecord offset value which is maintianed in main() and passed
//	forward to srecord functions (upload)
//=============================================================================

int x_cmdfn(U8 nargs, char* args[ARG_MAX], U16* offset){

#define	OBUF_LEN 110				// buffer length
#define	MAX_LOG_ERRS (OBUF_LEN / 4)	// max number of errors logged by verify cmd
	char	obuf[OBUF_LEN];			// gp output buffer
	U16		params[ARG_MAX];		// holding array for numeric params
	char	c;						// char temp
	char	pw = FALSE;				// W flag (set if "W" found in args)
//	char	pc = FALSE;				// C flag (set if "C" found in args)
//	char	px = FALSE;				// X flag (set if "X" found in args)
//	char	pm = FALSE;				// minus flag (set if <wsp>-<wsp> found in args)
//	char	pv = FALSE;				// V flag (set if <wsp>V<wsp> found in args)
	int		cmd_found = TRUE;		// default to true, set to false if no valid cmd identified
	char*	s;						// char temp pointer
	char*	t;						// char temp pointer
	char	cmd_id;					// command enumerated id
	char	cmd_id1;				// command enumerated id for 1st param (see if it is "HELP")
	U8		i;						// temp
	U8		j;						// temp
	U16		k;						// U16 temp
	U16		ku;						// U16 temp
	U16		kl;						// U16 temp
	uint32_t ii;					// u32 temp
	float	fa;						// float temps
	float	fb;
	float	fc;
	float	fd;


	bchar = '\0';																// clear global escape
    if (nargs > 0){
		for(i = 0; i <= nargs; i++){											// upcase all args
			s = args[i];
			str_toupper(s);
		}
		t = args[0];															// point to first arg (cmd)
		cmd_id = cmd_srch(t);													// search for command
		t = args[1];															// point to 2nd arg
		cmd_id1 = cmd_srch(t);													// search for command
		s = args[1];															// point to 2nd arg (1st param)
		if((*s == '?') || (cmd_id1 == help2)){									// see if this is a "HELP HELP" entry
			if((cmd_id == help1) || (cmd_id == help2)){							// if base command is help, then:
				do_help();														// list standard help banner
				puts0("");														// blank line
				for(i = 0; i < lastcmd; i++){									// do command-specific helps for all commands
					if(do_cmd_help(i)) puts0("");
				}
			}else{																// otherwise:
				do_cmd_help(cmd_id);											// do help for cmd only
			}
		}else{
/*			c = parm_srch(nargs, args, "/");									// capture minus floater
			if(c){
				pm = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "V");									// capture v-flag floater
			if(c){
				pv = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "C");									// capture c-flag floater
			if(c){
				pc = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "X");									// capture x-flag select floater
			if(c){
				px = TRUE;
				nargs--;
			}*/
			c = parm_srch(nargs, args, "W");									// capture w-flag floater
			if(c){
				pw = TRUE;
				nargs--;
			}
			gas_gage(2);														// init gas gauge to disabled state
			switch(cmd_id){														// dispatch command
				case help1:
				case help2:														// MAIN HELP
					do_help();
					break;

				case vers:														// SW VERSION CMD
					dispSWvers();
					break;

				case timer_tst:
					puts0("Timer test (esc to exit)...");
					do{
						GPIO_PORTE_DATA_R ^= PORTE_SPR4;
						wait(100);
					}while(bchar != ESC);
					break;

				case tst_templ:
					i = 0;														// default resolution = 0.5C
					if(*args[1]){												// 0 = 0.5, 1 = 0.25, 2 = 0.125, 3 = 0.0625
						i = (*args[1]) & 0x03;
					}
					putss("Tled:");
					disp_esc(pw);												// display "press esc to exit" if pw is true
					fa = get_temp(1, i);
					do{
						switch(i){
						default:
						case 0:
							sprintf(obuf,"%.1f C",fa);
							break;

						case 1:
							sprintf(obuf,"%.2f C",fa);
							break;

						case 2:
							sprintf(obuf,"%.3f C",fa);
							break;

						case 3:
							sprintf(obuf,"%.4f C",fa);
							break;
						}
						puts0(obuf);
						wait(500);
					}while(pw && (bchar != ESC));
					break;

				case tst_tempa:
					i = 0;														// default resolution = 0.5C
					if(*args[1]){												// 0 = 0.5, 1 = 0.25, 2 = 0.125, 3 = 0.0625
						i = (*args[1]) & 0x03;
					}
					putss("Tambient:");
					disp_esc(pw);												// display "press esc to exit" if pw is true
					fa = get_temp(0, i);
					do{
						switch(i){
						default:
						case 0:
							sprintf(obuf,"%.1f C",fa);
							break;

						case 1:
							sprintf(obuf,"%.2f C",fa);
							break;

						case 2:
							sprintf(obuf,"%.3f C",fa);
							break;

						case 3:
							sprintf(obuf,"%.4f C",fa);
							break;
						}
						puts0(obuf);
						wait(500);
					}while(pw && (bchar != ESC));
					break;

				case adc_tst:													// ADC test cmd
/*					if(pw){
						// "W" displays peripheral properties reg
						ii = ADC0_PP_R;
						sprintf(obuf,"ADC0_PP: 0x%08x",ii);
						puts0(obuf);
					}*/
					I2C_Send3(ADDR_ADC, 0x01, ADC_CONFIGH, ADC_CONFIGL);
					I2C_Send1(ADDR_ADC, 0x00);
					ku = 0;
					kl = 32767;
					for(ii = 0; ii<10000; ii++){
						k = I2C_Recv2(ADDR_ADC) * -1;							// we have to do this because the Isense resistor is wired backwards
						if(k>ku) ku = k;
						if(k<kl) kl = k;
					}
					sprintf(obuf,"Raw ADC Hi = %04x, Low = %04x",ku,kl);
					puts0(obuf);
					fa = 256.0 * (float)ku/32768.0;
					fa = fa/0.025;
					sprintf(obuf,"Iin(hi) = %.1f (mA)",fa);
					puts0(obuf);
					fa = 256.0 * (float)kl/32768.0;
					fa = fa/0.025;
					sprintf(obuf,"Iin(low) = %.1f (mA)",fa);
					puts0(obuf);
					break;

				case log_data:													// log data to com port
					// display log data:  F ?/t ms per sample (1000 default)
					// log continuous if "W" param included
					pw = TRUE;													// force to loop
					disp_esc(pw);												// display "press esc to exit" if pw is true
					ii = 1;														// init loop count
					i = 0;														// default resolution = 0.5C
					if(*args[1]){												// 0 = 0.5, 1 = 0.25, 2 = 0.125, 3 = 0.0625
						i = (*args[1]) & 0x03;
					}
					j = 0;
					k = 0;
					do{
						if(j == 0){
							puts0("SAMP#, Tled(C), Tamb(C), Imin(mA), Imax(mA)");	// put up banner every 60 lines
							j = 60;
						}
						j -= 1;
						k += 1;
						fa = get_temp(1, i);									// get Tled
						fb = get_temp(0, i);									// get Tamb
						fc = get_imin(TRUE);									// get max/min current, reset min/max
						fd = get_imax(TRUE);
						sprintf(obuf,"%u,%.3f,%.3f,%.1f,%.1f",ii++,fa,fb,fc,fd); // display log data, advance loop count
						puts0(obuf);
						while(!get_isamp()){									// wait with process update
							process_IO(0);
						}
					}while(pw && (bchar != ESC));
					break;

				case ztest:
					// test command
					params[0] = TRUE;
					get_Dargs(1, nargs, args, &params[0]);
					if(params[0] == 0) led_runstat(FALSE);
					else led_runstat(TRUE);
					led_rgb(params[1], params[2], params[3]);
					break;

				default:
				case lastcmd:													// not valid cmd
					cmd_found = FALSE;
					break;
			}
		}
    }
	if(bchar == ESC) while(gotchr()) getchr();									// if ESC, clear CLI input chrs
	return cmd_found;
}

//=============================================================================
// do_help() displays main help screen
//=============================================================================
void do_help(void){

	puts0("LED In Bottle CMD List:");
	puts0("Syntax: <cmd> <arg1> <arg2> ... args are optional depending on cmd.");
	puts0("\t<arg> order is critical except for floaters.");
	puts0("\"?\" as first <arg> gives cmd help, \"? ?\" lists all cmd help lines. When");
	puts0("selectively entering <args>, use \"-\" for <args> that keep default value.");
	puts0("\"=\" must precede decimal values w/o spaces. Floating <args>: these non-number");
	puts0("<args> can appear anywhere in <arg> list: \"W\" = wait or loop");
	puts0("\tAdc test\t\t\tVERSion");
	puts0("\tTA: temp ambient (W to loop)\tTL: LED temp (W to loop)");
	puts0("\tTImer test\t\t\tLOG data to term");
	puts0("Supports baud rates of 115.2, 57.6, 38.4, 19.2, and 9.6 kb.  Press <Enter>");
	puts0("as first character after reset at the desired baud rate.");
}

//=============================================================================
// do_cmd_help() displays individual cmd help using cmd_id enum
//=============================================================================
char do_cmd_help(U8 cmd_id){

	char c = TRUE;
	
	switch(cmd_id){														// dispatch help line

		case adc_tst:													// ADC tst CMD
			puts0("ADC test: A ?");
			puts0("\tDisplays raw ADC values and min/max values for the");
			puts0("\tcurrent sense ADC.  min/max sampled over ~~3sec period.");
			break;

		case tst_tempa:													// HOST MEM WR CMD
			puts0("Test LED temperature: TL ?/W/<resolution>");
			puts0("\tDisplays temperature of LED sensor, 'W' to loop");
			puts_res();
			break;

		case tst_templ:													// HOST MEM WR CMD
			puts0("Test Ambient temperature: TA ?/W/<resolution>");
			puts0("\tDisplays temperature of ambient sensor, 'W' to loop");
			puts_res();
			break;

		case timer_tst:													// HOST MEM WR CMD
			puts0("Timer test: TI ?");
			puts0("\ttoggles PE4 @200ms period, ESC to exit");
			break;

		case log_data:													// HOST MEM WR CMD
			puts0("Log temperature and current: LOG ?/<resolution>");
			puts0("\tOutputs comma-delimited sample lines of temp and current data.");
			puts_res();
			break;

		case ztest:														// led test cmd
			puts0("LED rgb test: Z ?/<sweep>/<r>/<b>/<g>");
			puts0("\t<sweep> = 1 start LED sweep, else stop.");
			puts0("\t<r><b><g> = 0 max bright, 0x26f min bright");
			puts_res();
			break;

		default:
			c = FALSE;
			break;
	}
	return c;
}

//=============================================================================
// puts_res() displays resolution help
//=============================================================================
void puts_res(void){
	puts0("\t<resolution> sets the resolution of the temperature sensor(s).");
	puts0("\tand is a number from 0 to 3 (0 is the default):");
	puts0("\t\t0: 0.5C\t\t2: 0.125C");
	puts0("\t\t1: 0.25C\t3: 0.0625C");
	return;
}

//=============================================================================
// disp_wait_addr() dead code
//=============================================================================
void disp_wait_addr(char* buf){

	sprintf(buf,"disp_wait_addr = dead code\n");
	puts0(buf);
}

//=============================================================================
// exec_bcmd() dead code
//=============================================================================
void exec_bcmd(char* bcmdbuf_ptr, char* obuf, U16* offset){

}

//***********************//
// bcmd functions follow //
//***********************//

//******************************//
// hosekeeping functions follow //
//******************************//


//=============================================================================
// get numeric params from command line args
//	argsrt specifies the first item to be searched and aligns argsrt with params[0]
//	for multi-arg cmds, fields must be entered in order.
//=============================================================================
U8 get_Dargs(U8 argsrt, U8 nargs, char* args[ARG_MAX], U16 params[8]){

	char*	s;
	U16*	ptr1;
	U8		i;
	U8		count = 0;

	if(argsrt < nargs){											// test for start in limit (abort if not)
		for(i = argsrt; i < nargs; i++){						// convert strings to values
			s = args[i];										// set pointers to array items
			ptr1 = &params[i - argsrt];
			switch(*s){
				case '-':										// skip if user specified default
				case '\0':										// or if arg is empty
					break;

				default:
					count += sscanf(s,"%d",ptr1);				// get hex value
					break;

				case '$':
					s++;
					count += sscanf(s,"%x",ptr1);				// get decimal if leading "="
					break;
			}
		}
	}
	return count;
}

//=============================================================================
// search for command keywords, return cmd ID if found
//	uses the cmd_list[] array which is constructed as an array of null terminated
//	strings compressed into a single string definition.  Commands are added by
//	placing the minimum required text from the command name with a '\0' terminating
//	null.  cmd_list[] is terminated by an $ff after all of the command names.
//	cmd_enum{} holds an enumerated, named list of all valid commands.  The enum
//	definition must be at the top of this file, so the commented-out version shown
//	below must be copied and pasted to the top of the file whan any changes are
//	made (commands added or deleted).
//
//	Some thought must be put into the order of command names.  Shorter matching
//	names (e.g., single chr entries) must follow longer names to allow the algortihm
//	to properly trap the shorter cmd name.
//	
//=============================================================================
cmd_type cmd_srch(char* string){
// dummy end of list, used to break out of search loop
const char end_list[] = {0xff};
// list of minimum length command words. cmd words are separated by '\0' and list terminated with 0xff
const char cmd_list[] = {"A\0LOG\0TA\0TL\0TI\0Z\0?\0H\0VERS\0\xff"};
//enum cmd_enum{ adc_tst,log_data,tst_tempa,tst_templ,timer_tst,ztest,help1,help2,vers,lastcmd,helpcmd };
//!!! make changes to cmd_enum here, move them to top of file, then un-comment !!!

	char*	ptr;							// temp ptr
	char	cmdid = 0;						// start at beginning of cmd_enum
	char	i;								// temp
	char	found = FALSE;					// cmd found flag (default to not found)

	ptr = (char*)cmd_list;												// start at beginning of serach list
	while((*ptr & 0x80) != 0x80){								// process until 0xff found in search list
		i = strncmp(string, ptr, strlen(ptr));					// inbound string match search list?
		if(i){
			cmdid++;											// no, advance to next cmdid 
			while(*ptr++);										// skip to next item in search list
		}else{
			ptr = (char*)end_list;										// found match,
			found = TRUE;										// set break-out criteria
		}
	}
	if(!found) cmdid = lastcmd;									// not found, set error cmd id
	return cmdid;
}

//=============================================================================
// parm_srch() looks for a match of parm_str in any non-empty args[] strings
//	if found, remove the args entry from param list and return 1st chr of parm_str,
//	else return '\0'
//=============================================================================
char parm_srch(U8 nargs, char* args[ARG_MAX], char* parm_str){

	U8		i;								// counter temp
	char	c = '\0';						// first chr of matched parm_str (first time thru loop, there is no match)
	static char null_str[] = "";			// null string that persists

//	if(nargs > 1){
	    for(i = 1; i <= nargs; i++){							// search starting with first args[] item
			if(c){												// if(c!=null)...
				args[i] = args[i+1];							// if there was a match, move the subsequent pointers down one
			}else{
				if(strlen(parm_str) == strlen(args[i])){		// in order to match, the lengths have to be equal...
					if(strncmp(args[i], parm_str, strlen(parm_str)) == 0){ // look for match
						c = *parm_str;							// if match, capture 1st chr in matched string
						i--;									// back-up one to edit this item out of the list
					}
				}
			}
	    }
//	}
 	if(c != '\0'){
		args[ARG_MAX - 1] = null_str;							// if there was a match, the last pointer goes to null
		
	}
	return c;													// return first chr in matched string, or null if no match
}

//=============================================================================
// disp_esc() if param true, display "Press esc to exit" msg
//=============================================================================
void disp_esc(char flag){

	if(flag){
		putss("  Press <ESC> to exit.");
	}
	puts0("");
}

//=============================================================================
// convert all chrs in string to upper case
//=============================================================================
void str_toupper(char *string){

    while(*string != '\0'){
        *string++ = toupper(*string);
    }
}

//=============================================================================
// parse string for delimited arguments
//  on exit, the args[] array holds each delimited argument from the command string input:
//  args[0] holds first arg (command)
//  args[1] holds next arg
//  args[2] etc...
//  up to args[ARG_MAX]
//
//  nargs holds number of arguments collected.  i.e., nargs = 3 specifies that args[0] .. args[3]
//      all hold arguments (three total, including the command).
//=============================================================================
int parse_args(char* cmd_string, char* args[ARG_MAX]){
	int i;
	char quote_c = 0;
	static char null_string[2] = "";

    // clear args pointers
    for (i=0; i<ARG_MAX; i++){
        args[i] = null_string;
    }
    i = 0;
    do{
        if(quotespace(*cmd_string, 0)){         // process quotes until end quote or end of string
            quote_c = *cmd_string;              // close quote must = open quote
            args[i++] = ++cmd_string;               // start args with 1st char after quote
            while(!quotespace(*cmd_string,quote_c)){
                if(*cmd_string == '\0'){
                    return i;                   // end of cmd string, exit
                }
                cmd_string++;
            }
            *cmd_string++ = '\0';               // replace end quote with null
        }
        if(*cmd_string == '\0'){
            return i;                           // end of cmd string, exit
        }
        if(!whitespace(*cmd_string)){
            args[i++] = cmd_string++;			// when non-whitespace encountered, assign arg[] pointer
            if(i > ARG_MAX){
                return i;						// until all args used up
            }
            do{
                if(*cmd_string == '\0'){
                    return i;                   // end of cmd string, exit
                }
                if(whitespace(*cmd_string)){
                    *cmd_string = '\0';			// then look for next whitespace and delimit (terminate) the arg[] string
                    break;
                }
                cmd_string++;					// loop until end of cmd_string or next whitespace
            } while (1);
        }
        cmd_string++;							// loop...
    } while (1);
}

//=============================================================================
// parse_ehex() for embeded hex ($$) arguments
//  on exit, the string holds the original text with %xx replaced by a single
//	hex byte.
//=============================================================================
void parse_ehex(char * sptr){
	char* tptr;
	U8	i;

	while(*sptr){
		if((*sptr == '$') && (*(sptr+1) == '$')){
			i = asc_hex(*(sptr+2)) << 4;
			i |= asc_hex(*(sptr+3));
			*sptr++ = i;
			tptr = sptr;
			do{
				*tptr = *(tptr+3);
				tptr++;
			}while(*(tptr+2));
		}else{
			sptr++;
		}
	}
}
//=============================================================================
// test characer for whitespace
//=============================================================================
int whitespace(char c){

    switch (c){					// These are all valid whitespace:
        case '\n':          	// newline
        case '\r':          	// cr
        case '\t':          	// tab
        case 0x20:{         	// space
		case '/':				// slash is also wsp
            return TRUE;
        }
    }
    return FALSE;
}

//=============================================================================
// test characer for quote
//=============================================================================
int quotespace(char c, char qu_c){

    if(qu_c == '\0'){
        switch (c){				// if qu_c is null, these are valid quotes:
            case '\'':          // newline
            case '\"':          // cr
            case '\t':          // tab
                return TRUE;
            }
    } else {
        if(c == qu_c){			// else, only qu_c results in a TRUE match
            return TRUE;
        }
    }
    return FALSE;
}

//=============================================================================
// gas_gage() display up to 16 "*" chrs based on count rate.
//	Gauge appearance:
//	[****************]	all OK
//	[***.............]	errors detected
//
//	"len" cmds:
//	0: process gauge counter/display
//	1: set gauge error character = "."
//	2: disable gage counter/display (set clen = 0)
//	all others: set creset = count = len/16, display initial gauge characters
//	This calculation identifies how many bytes are in 1/16th of the total
//	byte count (len).  For count events (len == 0), this Fn decrements count, &
//	displays a gauge chr when count == 0.  count is then reloaded with creset.
//	process continues until 16 gauge chrs have been displayed.  After this,
//	any further count eventss result in no further change to the display.
//=============================================================================
U8 gas_gage(U16 len){

#define LENCMD_MAX 2		// max # of gas-gage() cmds

	static U16	creset;		// holding reg for data counter reset value
	static U16	count;		// data counter
	static U8	clen;		// gage chr counter
	static U8	gchr;		// gage chr storage
		   U8	c = 0;		// gage printed flag

	if(len <= LENCMD_MAX){
		if(!len && clen){
			if(--count == 0){ 
				putchar(gchr);					// disp gage chr
				count = creset;					// reset loop counters
				clen--;
				if(clen == 0) putchar(']');		// if end of gage, print end bracket
				c = 1;
			}
		}else{
			if(len == 1) gchr = '.';			// if error flag, change gauge chr to err mode
			if(len == 2) clen = 0;				// disable gauge
		}
	}else{
		creset = count = len >> 4;				// init count & count reset (creset) = len/16
		if(creset == 0) creset = 1;				// if overall length too short, set 1:1
		clen = 16;								// 16 gage chrs max
		gchr = '*';								// set * as gage chr
		putchar('[');							// print start bracket
		for(c = 0; c < 16; c++) putchar(' ');
		putchar(']');							// place end bracket for scale
		for(c = 0; c < 17; c++) putchar('\b');	// backspace to start of scale
		c = 1;
	}
	return c;
}

//=============================================================================
// log_error_byte() places error data into log buffer.  Log format is:
//	(device) (host) (addrH) (addrL).  Called by target verify fns to allow
//	a limited number of errors to be trapped (limit is the buffer used to
//	hold the error log).
//	returns updated pointer to next available log entry
//=============================================================================
U8* log_error_byte(U8* lbuf, U8 d, U8 h, U16 a){

	*lbuf++ = d;								// store device data
	*lbuf++ = h;								// store host data
	*lbuf++ = (U8)(a >> 8);						// store addr
	*lbuf++ = (U8)(a & 0xff);
	return lbuf;								// return updated pointer
}

//=============================================================================
// disp_error_log() displays errors logged into error string.  Log format is:
//	(device) (host) (addrH) (addrL)
//	Display format is:
//	nn: Dev ($xx) != $xx @$xxxx\n = 28 printed chrs
//	nn = err number (ordinal)
//	xx = data bytes
//	xxxx = error address
//=============================================================================
void disp_error_log(U8* lbuf, U16 len){

	char obuf[32];				// local buffer
	// use U16 type to simplify sprintf variable list
	U16  i;						// loop counter
	U16  d;						// device data
	U16  h;						// host data
	U16  a;						// addr

	len++;										// add 1 to end so that we can start loop at "1"
	for(i = 1; i < len; i++){					// loop from 1 to len+1 entries
		d = (U16)*lbuf++ & 0xff;				// format device data
		h = (U16)*lbuf++ & 0xff;				// format host data
		a = ((U16)*lbuf++ & 0xff) << 8;			// format addr
		a |= (U16)*lbuf++ & 0xff;
		sprintf(obuf,"%02u: Dev ($%02x) != $%02x @$%04x", i, d, h, a); // display err line
		puts0(obuf);
	}
}

//=============================================================================
// bcmd_resp_init() inits bcmd_resp_ptr
//=============================================================================
void bcmd_resp_init(void){

	bcmd_resp_ptr = bcmd_resp_buf;
	*bcmd_resp_ptr = '\0';
}

//=============================================================================
// asc_hex() converts ascii chr to 4-bit hex.  Returns 0xff if error
//=============================================================================
U8 asc_hex(S8 c){

	U8 i;

	if((c >= '0') && (c <= '9')){			// if decimal digit,
		i = (U8)(c - '0');					// subtract ASCII '0' to get hex nybble
	}else{
		if((c >= 'A') && (c <= 'F')){		// if hex digit,
			i = (U8)(c - 'A' + 0x0A);		// subtract ASCII 'A', then add 0x0A to get hex nybble
		}else{
			i = 0xff;						// if not valid hex digit, set error return
		}
	}
	return i;	
}

//=============================================================================
// temp_float() converts MCP9800 binary temp to a float (degrees C)
//=============================================================================
float temp_float(U16 k){
	U8		i;			// temp
	U8		j = 0;		// temp sign
	float	fa;			// temp float

	if(k & 0x8000){												// if negative,
		j = 1;													// preserve sign and
		k = ~k + 1;												// convert value to positive
	}
	i = k >> 8;													// get integer portion
	fa = (float)i;												// convert to float
	if(k & 0x0080) fa += 0.5;									// add fractional portion
	if(k & 0x0040) fa += 0.25;
	if(k & 0x0020) fa += 0.125;
	if(k & 0x0010) fa += 0.0625;
	if(j){														// if negative, convert
		fa *= -1;
	}
	return fa;
}
