/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */
/************************************************************************************
 **                                                                                **
 **                                        UHSDR                                   **
 **               a powerful firmware for STM32 based SDR transceivers             **
 **                                                                                **
 **--------------------------------------------------------------------------------**
 **                                                                                **
 **  File name:     osc_si569.h                                                    **
 **  Description:   Header File for SI569 Oscillator                               **
 **  Last Modified: 30th July 2021                                                 **
 **  Licence:		GNU GPLv3                                                      **
 ************************************************************************************/

// Test for any and all variants of SI569 oscillators with catch all definition
#if defined(__HW_SI569)
    #define USE_SI569
#elif defined(__HW_SI569A)
    #define USE_SI569_AGRADE
#elif defined(__HW_SI569B)
    #define USE_SI569_BGRADE
#elif defined(__HW_SI569C)
    #define USE_SI569_CGRADE
#elif defined(__HW_SI569D)
    #define USE_SI569_DGRADE
#elif defined(__HW_SI569CMOS)
    #define USE_SI569_CMOS
#else
    #define USE_SI569
#endif

#include "osc_interface.h"
#include "arm_math.h"

void 	Si569_Init(void);

// non-critical device information reading
float32_t   Si569_GetStartupFrequency(void);
uint8_t     Si569_GetI2CAddress(void);              // There is only one address defined (=0x55)
bool        Si569_IsPresent(void);

