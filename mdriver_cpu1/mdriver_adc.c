/*
 * tnb_mns_adc.c
 *
 *  Created on: Oct 17, 2021
 *      Author: dvarx
 */

#include <mdriver_cpu1.h>
#include "driverlib.h"
#include "device.h"
#include "stdbool.h"

//
// Defines
//
#define EX_ADC_RESOLUTION       16
// 12 for 12-bit conversion resolution, which supports single-ended signaling
// Or 16 for 16-bit conversion resolution, which supports single-ended or
// differential signaling
#define EX_ADC_SIGNALMODE EX_ADC_SIGNALMODE_DIFFERENTIAL
#define EX_ADC_SIGNALMODE_SINGLE_ENDED      0
#define EX_ADC_SIGNALMODE_DIFFERENTIAL      1
#define OFFSET16BIT 32768

// Sample on single pin (VREFLO is the low reference)
// Or "Differential" for ADC_MODE_DIFFERENTIAL:
// Sample on pair of pins (difference between pins is converted, subject to
// common mode voltage requirements; see the device data manual)

//current sensor calibration values
int32_t isensoroffsets[9]={   OFFSET16BIT,
                            OFFSET16BIT,
                            OFFSET16BIT,
                            OFFSET16BIT,
                            OFFSET16BIT,
                            OFFSET16BIT,
                            OFFSET16BIT,
                            OFFSET16BIT,
                            OFFSET16BIT
};
float isensorgains[9]={ 0.0004465987,
                        0.0004465987,
                        0.0004465987,
                        0.0004465987,
                        0.0004465987,
                        0.0004465987,
                        0.0004465987,
                        0.0004465987,
                        0.0004465987};

//
// Function to configure and power up ADCs A,B,C,D
//
void initADCs(void)
{
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCD_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCD_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
#elif(EX_ADC_RESOLUTION == 16)
    #if(EX_ADC_SIGNALMODE == EX_ADC_SIGNALMODE_SINGLE_ENDED)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_SINGLE_ENDED);
    #elif(EX_ADC_SIGNALMODE == EX_ADC_SIGNALMODE_DIFFERENTIAL)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setMode(ADCD_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    #endif
#endif

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCD_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    ADC_enableConverter(ADCC_BASE);
    ADC_enableConverter(ADCD_BASE);

    DEVICE_DELAY_US(1000);
}

//
// Function to configure SOCs 0 and 1 of ADCs A and C.
//
void initADCSOCs(void)
{
    //----------------------------------------------------------------
    // ADCA Configuration
    //  ADCA measures: [ia(A0/A1) ; ib(A2/A3) ; ic(A4/A5)]
    //----------------------------------------------------------------
    #if(EX_ADC_RESOLUTION == 12)
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0, 15);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN1, 15);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2, 15);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN3, 15);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN4, 15);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN5, 15);
    #elif(EX_ADC_RESOLUTION == 16)
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0_ADCIN1, 64);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2_ADCIN3, 64);
        ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN4_ADCIN5, 64);
    #endif

    //
    // Set SOC4 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //----------------------------------------------------------------
    // ADCB Configuration
    //  ADCB measures: [id(B0/B1) ; ie(B2/B3) ; ]
    //----------------------------------------------------------------

    #if(EX_ADC_RESOLUTION == 12)
        ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0, 15);
        ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2, 15);
        ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN4, 15);
        ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN5, 15);
        ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN1, 15);
    #elif(EX_ADC_RESOLUTION == 16)
        ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0_ADCIN1, 64);
        ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2_ADCIN3, 64);
    #endif

    //
    // Set SOC4 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    //----------------------------------------------------------------
    // ADCC Configuration
    //  ADCC measures: [ig(CH2,CH3) ; if(CH14,CH15)]
    //----------------------------------------------------------------
    #if(EX_ADC_RESOLUTION == 12)
        ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2, 15);
        ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN3, 15);
        ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN4, 15);
    #elif(EX_ADC_RESOLUTION == 16)
        ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN14_ADCIN15, 64);
        ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2_ADCIN3, 64);
    #endif

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
    //----------------------------------------------------------------
    // ADCD Configuration
    //  ADCD measures: [ih(CH0/CH1) ; ii(CH2/CH3)]
    //----------------------------------------------------------------

    #if(EX_ADC_RESOLUTION == 12)
        ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2, 15);
        ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN3, 15);
        ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0, 15);
        ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN1, 15);
    #elif(EX_ADC_RESOLUTION == 16)
        ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN0_ADCIN1, 64);
        ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN2_ADCIN3, 64);
    #endif

    //
    // Set SOC1 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCD_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCD_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
}

/*
 * calibration measurement is of the form m=alpha'*i+beta' => i=alpha*m+beta with alpha=1/alpha' and beta=-beta'/alpha
 */
const float calib_factor_current_alpha=-0.0076219958202716825;

inline float conv_adc_meas_to_current_a(const uint16_t adc_output,uint8_t sensorno){
    //current measured by sensor is inverted relative to the defined current
    return isensorgains[sensorno]*(float)((int32_t)adc_output-isensoroffsets[sensorno]);
}


#define BUFFER_NO 512
uint16_t buffer_i0s[BUFFER_NO];
uint16_t buffer_i1s[BUFFER_NO];
uint16_t buffer_i2s[BUFFER_NO];
float buffer_i0s_fl[BUFFER_NO];
float buffer_i1s_fl[BUFFER_NO];
float buffer_i2s_fl[BUFFER_NO];
uint16_t buffer_cnt=0;
unsigned int modcounter=0;
#define CURRENT_MODULO 10

// This function reads the analog inputs and stores them in the system_dyn_state structure
void readAnalogInputs(void){
    // ADC A Measurements -----------------------------------------------
    ADC_forceMultipleSOC(ADCA_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2));
    // Wait for ADCA to complete, then acknowledge flag
    while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false){}
    system_dyn_state.is[0] = conv_adc_meas_to_current_a(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0),0);
    system_dyn_state.is[1] = conv_adc_meas_to_current_a(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1),1);
    system_dyn_state.is[2] = conv_adc_meas_to_current_a(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2),2);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    // ADC B Measurements -----------------------------------------------
    ADC_forceMultipleSOC(ADCB_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1));
    // Wait for ADCB to complete, then acknowledge flag
    while(ADC_getInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1) == false){}
    system_dyn_state.is[3] = conv_adc_meas_to_current_a(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0),3);
    system_dyn_state.is[4] = conv_adc_meas_to_current_a(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1),4);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    // ADC C Measurements -----------------------------------------------
    ADC_forceMultipleSOC(ADCC_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1));
    // Wait for ADCC to complete, then acknowledge flag
    while(ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1) == false){}
    system_dyn_state.is[5] = conv_adc_meas_to_current_a(ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0),5);
    system_dyn_state.is[6] = conv_adc_meas_to_current_a(ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1),6);

    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

    // ADC D Measurements -----------------------------------------------
    ADC_forceMultipleSOC(ADCD_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1));
    // Wait for ADCD to complete, then acknowledge flag
    while(ADC_getInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1) == false){}
    system_dyn_state.is[7] = conv_adc_meas_to_current_a(ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0),7);
    system_dyn_state.is[8] = conv_adc_meas_to_current_a(ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1),8);
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);

    if((modcounter%CURRENT_MODULO)==0){
        buffer_i0s_fl[buffer_cnt]=system_dyn_state.is[0];
        buffer_i1s_fl[buffer_cnt]=system_dyn_state.is[1];
        buffer_i2s_fl[buffer_cnt]=system_dyn_state.is[2];
        buffer_cnt=(buffer_cnt+1)%BUFFER_NO;
    }
    modcounter++;
}
