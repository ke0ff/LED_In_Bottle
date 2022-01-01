/********************************************************************
 ************ COPYRIGHT (c) 2015 by ke0ff, Taylor, TX   *************
 *
 *  File name: version.h
 *
 *  Module:    Control
 *
 *  Summary:   This header contains the software version number
 *             as a character string.
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    07-30-14 jmh:  0.0 creation date
 *    10-20-15 jmh:  0.1 First field release
 *
 *******************************************************************/

#ifdef VERSOURCE
const S8    version_number[] = {"0.1"};
const S8    date_code[]      = {"20-Oct-2015"};
#endif

//-----------------------------------------------------------------------------
// Public Fn Declarations
//-----------------------------------------------------------------------------
void dispSWvers(void);
void ccmdSWvers(char* sbuf);

#define VERSION_INCLUDED
