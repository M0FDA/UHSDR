/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */
/************************************************************************************
 **                                                                                **
 **                                        UHSDR                                   **
 **               a powerful firmware for STM32 based SDR transceivers             **
 **                                                                                **
 **--------------------------------------------------------------------------------**
 **                                                                                **
 **  File name:     osc_si569.c                                                    **
 **  Description:   C++ Code Driver File for SI569                                 **
 **  Last Modified:                                                                **
 **  Licence:		GNU GPLv3                                                      **
 ************************************************************************************/

// Common
#include "uhsdr_board.h"
#include "codec.h"
//
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "uhsdr_hw_i2c.h"
#include "osc_si569.h"

// -------------------------------------------------------------------------------------
// Local Oscillator
// ------------------
/* There are five supported alternative choices for the Si569:
     USE_SI569          Default, uses lowest speed grade model for maximum compatibility
     USE_SI569_CMOS     frequencies up to  280 Mhz, lowest speed grade  <=  70 MHz
     USE_SI569_DGRADE   frequencies up to  325 Mhz, operating freq is   <=  80.125 MHz
     USE_SI569_CGRADE   frequencies up to  800 Mhz, operating freq is   <= 200 MHz
     USE_SI569_BGRADE   frequencies up to 1500 Mhz, operating freq is   <= 375 MHz
     USE_SI569_AGRADE   frequencies up to 3000 Mhz, operating freq is   <= 750 MHz
*/

// Define constants used by all speed grades
#define SI569_HARD_MIN_FREQ     200000      // 0.2 = 50 kHz min freq common to all speed grades
#define SI569_MIN_FREQ          1000000     // 1 = 250 kHz min operating freq (arbitrarily chosen)
#define SI569_MIN_FVCO          10800000000 // 10.8 GHz minimum VCO frequency
#define HSDIV_UpperLimit        2046
#define HSDIV_LoweLimit         4
#define HSDIV_UpperLimit_Odd    33
#define HSDIV_LowerLimit_Odd    5
#define LSDIV_UpperLimit        5
#define LSDIV_LowerLimit        0
#define POW_2_32                4294967296   //2^32
#define FBDIV_UpperLimit        2045 + ((POW_2_32 - 1) / POW_2_32) //0x7FDFFFFFFFF   //2045.99999999976
#define FBDIV_LowerLimit        60.0    // Note FBDIV comprises two concatenated parts, an integer and a fraction
#define SI569_XTAL_FREQ         152600000.0
#define SI569_PPM               950
#define ADPLL_PPM_STEPSIZE      0.0001164

// Define speed dependent constants
#ifdef USE_SI569
    #define SI569_MAX_FREQ      280000000   // 280 = 70 MHz
    #define SI569_MAX_FVCO      12206718160 // Hz
    #define OSC_DEVICE          OSC_SI569CMOS
#endif

#ifdef USE_SI569_CMOS
    #define SI569_MAX_FREQ      280000000   // 280 = 70 MHz
    #define SI569_MAX_FVCO      12206718160 // Hz
    #define OSC_DEVICE          OSC_SI569CMOS
#endif

#ifdef USE_SI569_AGRADE
    #define SI569_MAX_FREQ      3000000000  // 3000 = 750 MHz
    #define SI569_MAX_FVCO      13122222022 // Hz
    #define OSC_DEVICE          OSC_SI569A
#endif

#ifdef USE_SI569_BGRADE
    #define SI569_MAX_FREQ      1500000000  // 1500 = 375 MHz
    #define SI569_MAX_FVCO      12511886114 // Hz
    #define OSC_DEVICE          OSC_SI569B
#endif

#ifdef USE_SI569_CGRADE
    #define SI569_MAX_FREQ      800000000   // 800 = 200 MHz
    #define SI569_MAX_FVCO      12206718160 // Hz
    #define OSC_DEVICE          OSC_SI569C
#endif

#ifdef USE_SI569_DGRADE
    #define SI569_MAX_FREQ      325000000   // 325 = 80.125 MHz
    #define SI569_MAX_FVCO      12206718160 // Hz
    #define OSC_DEVICE          OSC_SI569D
#endif

//TODO imported from SI570.c, may need translation
#define SI569_RECALL            (1<<0)
#define SI569_FREEZE_DCO        (1<<4)
#define SI569_FREEZE_M          (1<<5)
#define SI569_NEW_FREQ          (1<<6)


// Register definitions
#define SI569_Reg07             7           // [7] >> Reset, [3] >> MS_ICAL2
#define SI569_Reg17             17          // ODC_OE = 1 or 0 (0 = Disable osc o/p)
#define SI569_Reg23             23          // HSDIV[7:0]
#define SI569_Reg24             24          // OD_LSDIV[2:0], HSDIV[10:8]   (*2^4,,/2^8)
#define SI569_Reg26             26          // FBDIV[7:0]
#define SI569_Reg27             27          // FBDIV[15:8]                  (/2^8)
#define SI569_Reg28             28          // FBDIV[23:16]                 (/2^16)
#define SI569_Reg29             29          // FBDIV[31:24]                 (/2^24)
#define SI569_Reg30             30          // FBDIV[39:32]                 (/2^32)
#define SI569_Reg31             31          // FBDIV[42:40]                 (/2^40)
#define SI569_Reg35             35          // CADC_FSGAIN[7,0]
#define SI569_Reg69             69          // Freeze DCO, [7] >> FCAL_OVR
#define SI569_Reg231            231         // ADPLL_DELTA_M[7,0]
#define SI569_Reg232            232         // ADPLL_DELTA_M[15,8]
#define SI569_Reg233            233         // ADPLL_DELTA_M[23,16]
#define SI569_Reg255            255         // Page control register

#define SI569_I2C_ADDRESS       0x55        // I2C address
#define ODC_OE                  (1>>7)

typedef struct
{
	uint8_t     reg07;
	uint16_t    hsdiv;
	uint8_t     lsdiv;
    float64_t   fdco;
    float64_t   freq;
    float64_t   divs_reg;

} Si569_Freq_Config_t;

typedef struct
{
	bool                    is_present;
    unsigned short          si569_address;

	Si569_Freq_Config_t     current;
	Si569_Freq_Config_t     next;

    bool                    next_is_small;
    float64_t               fxtal;      // base fxtal value
    float64_t               fxtal_ppm;  // Frequency Correction of fxtal_calc
    float64_t               fxtal_calc; // ppm corrected fxtal value

    float                   fout;       // contains startup frequency info of Si569
    uint8_t                 base_reg;
    uint8_t                 cur_regs[6];

} Si569_State_t;

Si569_State_t si569_state;

static uchar Si569_SetBits(uchar original, uchar reset_mask, uchar new_val)
{
    return ((original & reset_mask) | new_val);
}

static uint16_t Si569_WriteRegister(uint8_t reg, uint8_t val)
{
    return UhsdrHw_I2C_WriteRegister(SI569_I2C, si569_state.si569_address, reg, 1, val);
}

static uint16_t Si569_WriteBlock(uint8_t reg, uint8_t* val, uint32_t size)
{
    return UhsdrHw_I2C_WriteBlock(SI569_I2C, si569_state.si569_address, reg, 1, val, size);
}

static uint16_t Si569_Read_Reg(uint16_t reg, uint8_t* val)
{
    return UhsdrHw_I2C_ReadRegister(SI569_I2C, si569_state.si569_address, reg, 1, val);
}

static uint16_t Si569_ReadRegisters(uint16_t reg, uint8_t *data, uint32_t size)
{
    return UhsdrHw_I2C_ReadBlock(SI569_I2C, si569_state.si569_address, reg, 1, data, size);
}

static uint16_t Si569_Read_Fbdiv(uint16_t reg, uint8_t* val)
{
    return UhsdrHw_I2C_ReadBlock(SI569_I2C, si569_state.si569_address, reg, 1, val, 6);
}

/*
 * @brief reads Si569 registers and verifies if matches local copy of settings
 * @returns SI569_OK if match, SI569_I2C_ERROR if I2C not working, or SI569_ERROR otherwise
 * */
static Oscillator_ResultCodes_t Si569_VerifyRegisters()
{
    Oscillator_ResultCodes_t retval = OSC_OK;   // Default result
    uchar   fbdiv_data[8];
    uchar   ppm_data[3];
    uchar   divs_data[2];
    uint8_t ret1;
    uint8_t ret2;
    uint8_t ret3;

    ret1 = Si569_ReadRegisters(SI569_Reg23, &divs_data[0], 2);
    ret2 = Si569_ReadRegisters(SI569_Reg231, &ppm_data[0], 3);
    ret3 = Si569_Read_Fbdiv(SI569_Reg26, &fbdiv_data[0]);
    if (ret1 || ret2 || ret3)
    {
        retval = OSC_COMM_ERROR;
    }
    if (retval == OSC_OK)
    {
        if (memcmp(divs_data, &si569_state.current.divs_reg, 2) != 0)
        {
            retval = OSC_ERROR_VERIFY;
        }
        else if (memcmp(ppm_data, &si569_state.fxtal_ppm, 3) != 0)
        {
            retval = OSC_ERROR_VERIFY;
        }
        else if (memcmp(fbdiv_data, &si569_state.current.fdco, 6) !=0)
        {
            retval = OSC_ERROR_VERIFY;
        }
    }

    return retval;
}
static uint16_t Si569_WriteConfig(uint8_t* divs_reg, uint8_t* fbdiv, float64_t* ppm_data)
{
    bool ret;
    uint8_t page_data;
    uint8_t retval;
    ret = UhsdrHw_I2C_ReadRegister(SI569_I2C, si569_state.si569_address, SI569_Reg255, 1, &page_data); // Read current page info
    ret &= UhsdrHw_I2C_WriteRegister(SI569_I2C, si569_state.si569_address, SI569_Reg255, 1, 0);    // Set page 0
    ret &= UhsdrHw_I2C_WriteRegister(SI569_I2C, si569_state.si569_address, SI569_Reg69, 1, 0);     // Disable FCAL override
    ret &= UhsdrHw_I2C_WriteRegister(SI569_I2C, si569_state.si569_address, SI569_Reg17, 1, 0);     // Synchronously disable output
    ret &= UhsdrHw_I2C_WriteBlock(SI569_I2C, si569_state.si569_address, 2, SI569_Reg23, divs_reg, 2); // Update output dividers
    ret &= UhsdrHw_I2C_WriteBlock(SI569_I2C, si569_state.si569_address, 6, SI569_Reg26, fbdiv, 6); // Update feedback dividers
    ret &= UhsdrHw_I2C_WriteRegister(SI569_I2C, si569_state.si569_address, SI569_Reg07, 1, 0x08);  // Start device
    ret &= UhsdrHw_I2C_WriteRegister(SI569_I2C, si569_state.si569_address, SI569_Reg17, 1, 0x01);  // Synchronously enable output
    ret &= UhsdrHw_I2C_WriteRegister(SI569_I2C, si569_state.si569_address, SI569_Reg255, 1, page_data); // Reset original page
    if (ret == 0)
    {
        retval = Si569_VerifyRegisters();
    }
    else
    {
        retval = OSC_COMM_ERROR;
    }
    return retval;
}

//*----------------------------------------------------------------------------
//* Function Name       : si569_small_frequency_change
//* Object              : small frequency changes handling
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static Oscillator_ResultCodes_t Si569_SmallFrequencyChange(uint32_t Ppm_Delta)
{
    uint32_t ADPLL_Delta_M = (Ppm_Delta / ADPLL_PPM_STEPSIZE);
    uint8_t  reg_data[3] = 
    {
        (ADPLL_Delta_M & 0x000000FF),
        (ADPLL_Delta_M & 0x0000FF00) >> 8,
        (ADPLL_Delta_M & 0x00FF0000) >>16
    };
    uint16_t ret    = 1;
    Oscillator_ResultCodes_t retval = -1;        // 1=OK, -1=Requested PPM shift too large

    if ((Ppm_Delta <= SI569_PPM) && (Ppm_Delta >= -SI569_PPM))
    {
        retval = 0;
        // Write to the three registers
        ret = Si569_WriteBlock(SI569_Reg231, reg_data, 3);
        if (ret == 0)
        {
            retval = 1;
        }
    }
    
    return retval;
}

//*----------------------------------------------------------------------------
//* Function Name       : 
//* Object              : Calculate new data for large frequency changes
//* Input Parameters    :
//* Output Parameters   :
//* Functions called    :
//*----------------------------------------------------------------------------
static Oscillator_ResultCodes_t Si569_LargeFrequencyChange()
{
    uint16_t ret;
    uint16_t ret1;
    uint16_t ret2;
    Oscillator_ResultCodes_t retval = OSC_OK;
    uchar reg_7;
    uchar reg_23;
    uchar reg_24;
    uchar reg_FBDIV[6];


    // Read current state
    ret = Si569_SetBits(si569_state.current.reg07, SI569_Reg07, ODC_OE);
    if ( ret==0 )
    {
        // Read registers as 2 blocks (23,24  & 26-31)
        ret = Si569_Read_Reg(SI569_Reg23, &reg_23);
        ret1 = Si569_Read_Reg(SI569_Reg24, &reg_24);
        ret2 = Si569_Read_Fbdiv(SI569_Reg26, &reg_FBDIV[0]);
        if ((ret == 0) && (ret1 == 0))
        {
            retval = Si569_VerifyRegisters();
        }
        else
        {
            retval = OSC_COMM_ERROR;
        }
    }

    Si569_SetBits(si569_state.current.reg07, SI569_Reg07, ODC_OE);

/*
 * TODO Define SetLimits function or appropriate other before enabling following block
 *
    if (SetLimits(OSC_DEVICE,uint16_t Output_Freq) == 0)
    {
        // Calculate theoretical HSDIV * LSDIV value based on lowest VCO frequency
        float64_t Min_HSLS_Div = SI569_MIN_FVCO / Output_Freq;
    }
*/
    return   retval;
}

static Oscillator_ResultCodes_t Si569_ResetConfig()
{
    uint8_t retval = 0;
    short   res;
    ulong   rfreq_frac;
    ulong   rfreq_int;
    ulong   fbdiv_curr;
    uchar   hsdiv_curr;
    uchar   lsdiv_curr;

    // Reset publics
    si569_state.is_present = false;

    res = UhsdrHw_I2C_WriteRegister(SI569_I2C, si569_state.si569_address, SI569_Reg07, 1, SI569_RECALL);
    if(res != 0)
    {
        retval = 1;
    }
    else
    {
        uint8_t   ret;
        int i = 0;
        do
        {
            res = UhsdrHw_I2C_ReadRegister(SI569_I2C, si569_state.si569_address, SI569_Reg07, 1, &ret);
            if(res != 0)
            {
                retval = 2;
                break;
            }

            i++;

            if(i == 30)
            {
                retval = 3;
                break;
            }
        }  while(ret & SI569_RECALL);

        if (retval == 0 && Si569_ReadRegisters(SI569_Reg23,&(si569_state.cur_regs[0]),6) != 0)
        {
            retval = 4;
        }
        else
        {
            hsdiv_curr = ((si569_state.cur_regs[0] & 0xE0) >> 5) + 4;

            fbdiv_curr = 1 + ((si569_state.cur_regs[0] & 0x1F) << 2) + ((si569_state.cur_regs[1] & 0xC0) >> 6);


            rfreq_int = (si569_state.cur_regs[1] & 0x3F);
            rfreq_int = (rfreq_int << 4) + ((si569_state.cur_regs[2] & 0xF0) >> 4);

            rfreq_frac = (si569_state.cur_regs[2] & 0x0F);
            rfreq_frac = (rfreq_frac << 8) + si569_state.cur_regs[3];
            rfreq_frac = (rfreq_frac << 8) + si569_state.cur_regs[4];
            rfreq_frac = (rfreq_frac << 8) + si569_state.cur_regs[5];

            float64_t rfreq = rfreq_int + (float64_t)rfreq_frac / POW_2_32;
            si569_state.fxtal = ((float64_t)si569_state.fout * (float64_t)(fbdiv_curr *hsdiv_curr)) / rfreq;

//            Si569_SetPPM(si569_state.fxtal_ppm);

            si569_state.current.freq = rfreq;
            si569_state.current.divs_reg = fbdiv_curr;
            si569_state.current.hsdiv = hsdiv_curr;
//            si569_state.current.fdco = Si569_GetFDCOForFreq(si569_state.fout,fbdiv_curr,hsdiv_curr);

            si569_state.is_present = true;
        }
    }
    return retval;
}

void Si569_Init()
{
    si569_state.current.freq    = 10000000;
    si569_state.next.freq       = 10000000;
    si569_state.current.fdco    = 0;
    si569_state.si569_address   = SI569_I2C_ADDRESS;

    //si569_state.is_present          = 
    if (UhsdrHw_I2C_DeviceReady(SI569_I2C,si569_state.si569_address) == HAL_OK)
    {
        // Ensure everything is cleared and in initial state
        Si569_ResetConfig();
    }

    if (si569_state.is_present)
    {
     //   Si569_WriteConfig(SI569_Reg17,si569_state.current.fdco,0);  //si569_state.current.ppm);
    }
}

