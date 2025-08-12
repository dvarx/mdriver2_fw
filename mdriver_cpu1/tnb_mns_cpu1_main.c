/*
 * ############################################################
 * CPU1 program code for MDriver2 control board based on TMS320F28388S v1.0
 * This is a minimally working example that blinks the output LED
 * ############################################################
 */

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "tnb_mns_cpu1.h"
#include "tnb_mns_epwm.h"
#include "tnb_mns_adc.h"
#include <stdbool.h>
#include "tnb_mns_fsm.h"
#include "fbctrl.h"
#include "tnb_mns_cpu1.h"
#include "tnb_mns_defs.h"
#include <math.h>
#include "ipc.h"
#include "tnb_mns_cpu1.h"

bool run_main_control_task=false;
bool enable_waveform_debugging=false;

void main(void)
{

    //----------------------------------------------------------------------------------------------
    // Initialize device clock & peripherals
    //----------------------------------------------------------------------------------------------
    Device_init();
    //
    // Boot CM core
    //
#ifdef _FLASH
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif
    // Disable pin locks and enable internal pull-ups.
    Device_initGPIO();

    //----------------------------------------------------------------------------------------------
    // Setup heartbeat GPIO & LED GPIOs
    //----------------------------------------------------------------------------------------------
    //heartbeat
    GPIO_setDirectionMode(HEARTBEAT_GPIO, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(HEARTBEAT_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //main input relay
    //GPIO_setDirectionMode(MAIN_RELAY_GPIO, GPIO_DIR_MODE_OUT);   //output
    //GPIO_setPadConfig(MAIN_RELAY_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //input relay to slave
    //GPIO_setDirectionMode(SLAVE_RELAY_GPIO, GPIO_DIR_MODE_OUT);   //output
    //GPIO_setPadConfig(SLAVE_RELAY_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //LED 1 for debugging
    GPIO_setDirectionMode(LED_GREEN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(LED_GREEN,GPIO_PIN_TYPE_STD);
    GPIO_writePin(LED_GREEN,1);
    //LED 2 for debugging
    //GPIO_setDirectionMode(LED_GREEN_2, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LED_GREEN_2,GPIO_PIN_TYPE_STD);
    //GPIO_writePin(LED_GREEN_2,1);

    //----------------------------------------------------------------------------------------------
    // Initialize ADCs
    //----------------------------------------------------------------------------------------------
    //setup ADC clock, single-ended mode, enable ADCs
    initADCs();
    //setup ADC SOC configurations
    initADCSOCs();

    //----------------------------------------------------------------------------------------------
    // Initialize Half Bridges
    //----------------------------------------------------------------------------------------------
    //channel A
    setup_pinmux_config_bridge(&cha_bridge);
    set_enabled(&cha_bridge,true);
    //channel B
    setup_pinmux_config_bridge(&chb_bridge);
    set_enabled(&chb_bridge,true);
    //channel C
    setup_pinmux_config_bridge(&chc_bridge);
    set_enabled(&chc_bridge,true);
    //channel D
    setup_pinmux_config_bridge(&chd_bridge);
    set_enabled(&chd_bridge,true);
    //channel E
    setup_pinmux_config_bridge(&che_bridge);
    set_enabled(&che_bridge,true);
    //channel F
    setup_pinmux_config_bridge(&chf_bridge);
    set_enabled(&chf_bridge,true);
    //channel G
    setup_pinmux_config_bridge(&chg_bridge);
    set_enabled(&chg_bridge,true);
    //channel H
    setup_pinmux_config_bridge(&chh_bridge);
    set_enabled(&chh_bridge,true);
    //channel I
    setup_pinmux_config_bridge(&chi_bridge);
    set_enabled(&chi_bridge,true);
    /*

    //
    // Initialize synchronization of bridges
    //
    //enable sync output of EPWM of channel 0, sync output will be generated when timer reaches zero
    EPWM_enableSyncOutPulseSource(driver_channels[0]->bridge_config->epwmbase,EPWM_SYNCOUTEN_ZEROEN);
    //channel 0 does not do a phase shift load
    EPWM_disablePhaseShiftLoad(driver_channels[0]->bridge_config->epwmbase);
    //set phase shift register to zero
    EPWM_setPhaseShift(driver_channels[0]->bridge_config->epwmbase, 0);
    //Hint : when channel 1,2,3,4 or 5 enter resonant mode, their pwm counter will be synchronized to channel 0
    // channel 0 always needs to be in resonant mode if one of the other coils is in resonant mode

    */


    //----------------------------------------------------------------------------------------------
    // Setup of main interrupts
    // - main control interrupt
    // - inter process communication with CM
    // - communication active interrupt
    //----------------------------------------------------------------------------------------------
    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    Interrupt_initModule();
    // Initializes the PIE vector table with pointers to the shell Interrupt Service Routines (ISRs)
    Interrupt_initVectorTable();
    //--------------- IPC interrupt ---------------
    //clear any IPC flags
    IPC_clearFlagLtoR(IPC_CPU1_L_CM_R, IPC_FLAG_ALL);
    //register IPC interrupt from CM to CPU1 using IPC_INT0
    IPC_registerInterrupt(IPC_CPU1_L_CM_R, IPC_INT0, IPC_ISR0);
    //synchronize CM and CPU1 using IPC_FLAG31
    IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG31);
    //--------------- CPU1 Timer0 interrupt (main task) ---------------
    // Register ISR for cupTimer0
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    // Initialize CPUTimer0
    configCPUTimer(CPUTIMER0_BASE, 1e6*deltaT);
    // Enable CPUTimer0 Interrupt within CPUTimer0 Module
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    // Enable TIMER0 Interrupt on CPU coming from TIMER0
    Interrupt_enable(INT_TIMER0);
    // Start CPUTimer0
    CPUTimer_startTimer(CPUTIMER0_BASE);
    //--------------- CPU1 Timer1 interrupt (communication active) ---------------
    // Register ISR for cupTimer1
    //Interrupt_register(INT_TIMER1, &cpuTimer1ISR);
    // Initialize CPUTimer1
    //configCPUTimer(CPUTIMER1_BASE, 500000);
    // Enable CPUTimer0 Interrupt within CPUTimer1 Module
    //CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    // Enable TIMER1 Interrupt on CPU coming from TIMER1
    //Interrupt_enable(INT_TIMER1);
    // Start CPUTimer0
    //CPUTimer_startTimer(CPUTIMER1_BASE);

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

#ifdef ETHERNET
    //
    // Set up EnetCLK to use SYSPLL as the clock source and set the
    // clock divider to 2.
    //
    // This way we ensure that the PTP clock is 100 MHz. Note that this value
    // is not automatically/dynamically known to the CM core and hence it needs
    // to be made available to the CM side code beforehand.
    SysCtl_setEnetClk(SYSCTL_ENETCLKOUT_DIV_2, SYSCTL_SOURCE_SYSPLL);

    //
    // Configure the GPIOs for ETHERNET.
    //

    //
    // MDIO Signals
    //
    GPIO_setPinConfig(GPIO_42_ENET_MDIO_CLK);
    GPIO_setPinConfig(GPIO_43_ENET_MDIO_DATA);

    //
    // Use this only for RMII Mode
    //GPIO_setPinConfig(GPIO_73_ENET_RMII_CLK);
    //

    //
    //MII Signals
    //
    GPIO_setPinConfig(GPIO_40_ENET_MII_CRS);
    GPIO_setPinConfig(GPIO_41_ENET_MII_COL);

    GPIO_setPinConfig(GPIO_59_ENET_MII_TX_DATA0);
    GPIO_setPinConfig(GPIO_60_ENET_MII_TX_DATA1);
    GPIO_setPinConfig(GPIO_61_ENET_MII_TX_DATA2);
    GPIO_setPinConfig(GPIO_62_ENET_MII_TX_DATA3);

    //
    //Use this only if the TX Error pin has to be connected
    //GPIO_setPinConfig(GPIO_46_ENET_MII_TX_ERR);
    //

    GPIO_setPinConfig(GPIO_56_ENET_MII_TX_EN);

    GPIO_setPinConfig(GPIO_52_ENET_MII_RX_DATA0);
    GPIO_setPinConfig(GPIO_53_ENET_MII_RX_DATA1);
    GPIO_setPinConfig(GPIO_54_ENET_MII_RX_DATA2);
    GPIO_setPinConfig(GPIO_55_ENET_MII_RX_DATA3);
    GPIO_setPinConfig(GPIO_51_ENET_MII_RX_ERR);
    GPIO_setPinConfig(GPIO_50_ENET_MII_RX_DV);

    GPIO_setPinConfig(GPIO_49_ENET_MII_RX_CLK);
    GPIO_setPinConfig(GPIO_44_ENET_MII_TX_CLK);

    //
    //Power down pin to bring the external PHY out of Power down
    //
    GPIO_setDirectionMode(68, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(68, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(68,1);

    //
    //PHY Reset Pin to be driven High to bring external PHY out of Reset
    //

    GPIO_setDirectionMode(67, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(67, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(67,1);
#endif

#ifdef UART
    //
    // Configure GPIO85 as the UART Rx pin.
    //
    GPIO_setPinConfig(GPIO_85_UARTA_RX);
    GPIO_setDirectionMode(85, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(85, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(85, GPIO_QUAL_ASYNC);

    //
    // Configure GPIO84 as the UART Tx pin.
    //
    GPIO_setPinConfig(GPIO_84_UARTA_TX);
    GPIO_setDirectionMode(84, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(84, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(84, GPIO_QUAL_ASYNC);
#endif

    //----------------------------------------------------------------------------------------------
    // Initialize Outputs
    //----------------------------------------------------------------------------------------------
    int n;
    for(n=0; n<NO_CHANNELS; n++){
        READY_enter(n);
        driver_channels[n]->channel_state=READY;
    }

    uint32_t loop_counter=0;
    // Main Loop
    while(1){
        if(run_main_task){
            //toggle heartbeat gpio
            GPIO_togglePin(LED_GREEN);

            //----------------------------------------------------------------------------------------------
            // State Machine
            //----------------------------------------------------------------------------------------------
            //main relay logic
            unsigned int channel_counter=0;
            bool main_relay_active=false;
            //GPIO_writePin(MAIN_RELAY_GPIO,main_relay_active);
            //GPIO_writePin(SLAVE_RELAY_GPIO,main_relay_active);
            //run state machine
            for(channel_counter=0; channel_counter<NO_CHANNELS; channel_counter++){
                run_channel_fsm(driver_channels[channel_counter]);
                //we enable the main relay when one channel is not in state READY anymore (e.g. when one channel requires power)
                if(driver_channels[channel_counter]->channel_state!=READY)
                    main_relay_active=true;
            }
            //communication active logic (if no communication via TCP, issue a stop flag)
            if(!communication_active){
                for(channel_counter=0; channel_counter<NO_CHANNELS; channel_counter++){
                    fsm_req_flags_stop[channel_counter]=1;
                }
            }



            //----------------------------------------------------------------------------------------------
            // Signal Acquisition & Filtering
            //----------------------------------------------------------------------------------------------

            // Read ADCs sequentially, this updates the system_dyn_state structure
            readAnalogInputs();
            // TODO: Filter the acquired analog signals in system_dyn_state_filtered
            // ---
            // TODO: Filter the input reference signals
//            unsigned int i=0;
//            for(i=0; i<NO_CHANNELS; i++){
//                update_first_order(des_duty_buck_filt+i,des_duty_buck[i]);
//            }



            //----------------------------------------------------------------------------------------------
            // Control Law Execution
            //----------------------------------------------------------------------------------------------
            /*
            //compute optional reference waveform
            //#define OMEGA 2*3.14159265358979323846*5
            //float ides=sin(OMEGA*loop_counter*deltaT);
            const unsigned int periodn=500e-3/deltaT;
            float ides=0.0;
            if(loop_counter%periodn<periodn/2)
                ides=1.0;
            else
                ides=-1.0;
            //regulate outputs of channels
            // ...
             * */


            //----------------------------------------------------------------------------------------------
            // Control Law Execution & Output Actuation
            //----------------------------------------------------------------------------------------------
            /*
            //set output duties for buck
//            for(i=0; i<NO_CHANNELS; i++){
//                set_duty_buck(driver_channels[i]->buck_config,(des_duty_buck_filt+i)->y);
//            }
            //set output duties for bridge [regular mode]
            for(i=0; i<NO_CHANNELS; i++){
                if(driver_channels[i]->channel_state==RUN_REGULAR){
                    #ifdef TUNE_CLOSED_LOOP
                        if(enable_waveform_debugging)
                            des_currents[i]=ides;
                        else
                            ides=0.0;
                    #endif
                    //execute the PI control low
                    float voltage_dclink=VIN*(des_duty_buck_filt+i)->y;
                    //compute feed forward actuation term (limits [-1,1] for this duty) - feed-forward term currently not used
                    #ifdef FEED_FORWARD_ONLY
                        float act_voltage_ff=des_currents[i]*RDC;
                        float act_voltage_fb=0.0;
                    #endif
                    #ifdef TUNE_CLOSED_LOOP
                        float act_voltage_ff=0.0;
                        //compute feedback actuation term (limits [-1,1] for this duty)
                        bool output_saturated=fabsf((current_pi+i)->u)>=0.9*voltage_dclink;
                        float act_voltage_fb=update_pid(current_pi+i,des_currents[i],system_dyn_state.is[i],output_saturated);
                    #endif
                    #ifdef CLOSED_LOOP
                        float act_voltage_ff=0.0;
                        //compute feedback actuation term (limits [-1,1] for this duty)
                        bool output_saturated=fabsf((current_pi+i)->u)>=0.9*voltage_dclink;
                        float act_voltage_fb=update_pid(current_pi+i,des_currents[i],system_dyn_state.is[i],output_saturated);
                    #endif
                    float duty_ff=act_voltage_ff/voltage_dclink;
                    float duty_fb=act_voltage_fb/(voltage_dclink);

                    //convert normalized duty cycle, limit it and apply
                    float duty_bridge=0.5*(1+(duty_ff+duty_fb));
                    if(duty_bridge>0.9)
                        duty_bridge=0.9;
                    if(duty_bridge<0.1)
                        duty_bridge=0.1;
                    set_duty_bridge(driver_channels[i]->bridge_config,duty_bridge);
                }
                //set_duty_bridge(driver_channels[i]->bridge_config,des_duty_bridge[i]);
            }
            //setup_phase_control(driver_channels,debug_phase0,debug_phase1);

            //---------------------
            // Read State Of Bridges
            //---------------------
            //uint32_t cha_buck_state=GPIO_readPin(cha_buck.state_gpio);
            uint32_t cha_bridge_state_u=GPIO_readPin(cha_bridge.state_v_gpio);
            uint32_t cha_bridge_state_v=GPIO_readPin(cha_bridge.state_u_gpio);

            //uint32_t chb_buck_state=GPIO_readPin(chb_buck.state_gpio);
            uint32_t chb_bridge_state_u=GPIO_readPin(chb_bridge.state_v_gpio);
            uint32_t chb_bridge_state_v=GPIO_readPin(chb_bridge.state_u_gpio);

            //uint32_t chc_buck_state=GPIO_readPin(chc_buck.state_gpio);
            uint32_t chc_bridge_state_u=GPIO_readPin(chc_bridge.state_v_gpio);
            uint32_t chc_bridge_state_v=GPIO_readPin(chc_bridge.state_u_gpio);
            */

            run_main_task=false;
            loop_counter=loop_counter+1;
        }
    }
}


