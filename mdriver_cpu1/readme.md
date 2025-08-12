# TNB_MNS Controller CPU1

This project contains the code for CPU1 on the TMDSCNCD28388D  control card.

## PWM Settings

The PWM frequency is set by the parameters `EPWM_ClockDivider=1`, `EPWM_HSClockDivider=1` and `EPWM_TIMER_TBPRD_BRIDGE=1024`.
The resulting ePWM frequency is given by
$$ f_{PWM} = \frac{f_{MCU}}{\mathtt{EPWMTIMERTBPRDBRIDGE}*\mathtt{EPWMHSClockDivider}*\mathtt{EPWMClockDivider}*2}\approx 98kHz$$
The last factor of 2 comes from the fact that an up-down counter is used for the PWM resulting in a symmetric waveform.

## Current measurement

### Sensor gain
The differential output voltage of the current sensor (based on shunt resistance measurement) is given by:
$$
V_{diff}=I_c R_s A_v
$$
where $A_v=41V/V$ is the amplifier gain and $R_s=5m\Omega$ is the sense resistance. This value gets compared to a reference voltage $V_{ref}=3V$ and mapped onto a signed, 16bit integer:
$$
\texttt{ADCOUT} = I_c \frac{2^{15} R_s A_v}{V_{ref}} 
$$
the measured current is therefore given by
$$
I_c=\frac{V_{ref}}{2^{15} A_v R_s}\texttt{ADCOUT}\approx 0.0004465987*\texttt{ADCOUT}
$$
### Sensor offset
The nominal sensor offset is $0V$.