# TNB_MNS Controller CPU1

This project contains the code for CPU1 on the TMDSCNCD28388D  control card.

## PWM Settings

The PWM frequency is set by the parameters `EPWM_ClockDivider=1`, `EPWM_HSClockDivider=1` and `EPWM_TIMER_TBPRD_BRIDGE=1024`.
The resulting ePWM frequency is given by
$$ f_{PWM} = \frac{f_{MCU}}{\mathtt{EPWMTIMERTBPRDBRIDGE}*\mathtt{EPWMHSClockDivider}*\mathtt{EPWMClockDivider}*2}\approx 98kHz$$
The last factor of 2 comes from the fact that an up-down counter is used for the PWM resulting in a symmetric waveform.