/*
 * tnb_mns_cpu1.h
 *
 *  Created on: 14.10.2021
 *      Author: dvarx
 */

#ifndef MDRIVER_CPU1_H_
#define MDRIVER_CPU1_H_

#include <mdriver_cpu1.h>
#include <stdint.h>
#include "driverlib.h"
#include "device.h"
#include "stdint.h"
#include "fbctrl.h"
#include "comm_interface.h"

#define NO_CHANNELS 9
#define HEARTBEAT_GPIO 17
#define MAIN_RELAY_GPIO 92
#define SLAVE_RELAY_GPIO 62
#define DEFAULT_RES_FREQ_MILLIHZ    10000000
#define MINIMUM_RES_FREQ_MILLIHZ    80000
#define LED_GREEN 37
//#define LED_GREEN_2 37
#define LED_RED 36
//#define LED_BLUE 37
#define COMMUNICATION_TIMEOUT_MS    500
//define the pin polarity of the voltage needed to enable the ate driver
#define DRIVER_ENABLE_POLARITY 0
#define DRIVER_DISABLE_POLARITY 1

struct bridge_configuration{
    uint32_t enable_gpio;
    uint32_t state_u_gpio;
    uint32_t state_v_gpio;
    uint32_t bridge_h_pin;
    uint32_t bridge_h_pinconfig;
    uint32_t bridge_l_pin;
    uint32_t bridge_l_pinconfig;
    uint32_t epwmbase;
    bool is_inverted;
};

struct system_dynamic_state{
    float is[NO_CHANNELS];
    float vs[NO_CHANNELS];
    float is_res[NO_CHANNELS];
};

enum driver_channel_state {READY=0,BUCK_ENABLED=1,INIT_REGULAR=2,RUN_REGULAR=3,INIT_RESONANT=4,RUN_RESONANT=5,FAULT=6,TERMINATE_RESONANT=7,TERMINATE_REGULAR=8};

struct driver_channel{
    uint8_t channel_no;
    struct bridge_configuration* bridge_config;
    enum driver_channel_state channel_state;
};

extern struct driver_channel channela;
extern struct driver_channel channelb;
extern struct driver_channel channelc;
extern struct driver_channel channeld;
extern struct driver_channel channele;
extern struct driver_channel channelf;
extern struct driver_channel* driver_channels[NO_CHANNELS];

extern struct bridge_configuration cha_bridge;
extern struct bridge_configuration chb_bridge;
extern struct bridge_configuration chc_bridge;
extern struct bridge_configuration chd_bridge;
extern struct bridge_configuration che_bridge;
extern struct bridge_configuration chf_bridge;
extern struct bridge_configuration chg_bridge;
extern struct bridge_configuration chh_bridge;
extern struct bridge_configuration chi_bridge;

// ---------------------
// Main Program related globals
// ---------------------

extern bool run_main_task;                              //variable is set by CPU1 ISR
extern struct system_dynamic_state system_dyn_state;
extern float des_duty_bridge[NO_CHANNELS];             //desired duties for bridges, set by COMM interface
extern float des_duty_buck[NO_CHANNELS];               //desired duties for bucks, set by COMM interface
extern float des_currents[NO_CHANNELS];
extern uint32_t des_freq_resonant_mhz[NO_CHANNELS];     //desired frequencies for resonant bridges, set by COMM interface
extern struct first_order des_duty_buck_filt[NO_CHANNELS];
extern struct pi_controller current_pi[NO_CHANNELS];
extern struct mdriver_msg ipc_tnb_mns_msg;
extern bool communication_active;                       //variable indicates whether there is a TCP connection active (true if a package was received in the last 200ms)

// ---------------------
// Main CPU Timer Related Functions
// ---------------------

//
// Globals
//
extern uint16_t cpuTimer0IntCount;
//extern uint16_t cpuTimer1IntCount;
//extern uint16_t cpuTimer2IntCount;

__interrupt void cpuTimer0ISR(void);
__interrupt void IPC_ISR0(void);
__interrupt void cpuTimer1ISR(void);
void initCPUTimers(void);
void configCPUTimer(uint32_t, uint32_t);


#endif /* MDRIVER_CPU1_H_ */
