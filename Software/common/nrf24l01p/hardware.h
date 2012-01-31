#ifndef __HARDWARE_H
#define __HARDWARE_H

#define DEBUG

/*setup which timer is used for stimulation */
#define STIM_TIMER TIMER0
#define DAC_TIMER TIMER1 // Always opposite the one we're using. This turns on and off the op amps

/* Setup where TIMERPOS pin is connected */
#define TIMERPOS_N        0 
#define TIMERPOS        (1<<TIMERPOS)  
#define TIMERPOS_PORT   gpioPortA

/* Setup where TIMERNEG pin is connected */
#define TIMERNEG_N        1 
#define TIMERNEG        (1<<TIMERNEG)  
#define TIMERNEG_PORT   gpioPortA
/* Setup Debug Pin for timing information */
#define DEBUGPIN_N       1 
#define DEBUGPIN        (1<<DEBUGPIN_N)   
#define DEBUGPIN_PORT   gpioPortC

/* Setup where RFCS pin is connected */
#define RFCS_N        13 
#define RFCS        (1<<RFCS_N)   
#define RFCS_PORT   gpioPortE

/* Setup where RFCE pin is connected */
#define RFCE_N        0
#define RFCE        (1<<RFCE_N)   
#define RFCE_PORT   gpioPortC

/* Setup where RFIRQ pin is connected */
#define RFIRQ_N        2 
#define RFIRQ        (1<<RFIRQ_N)  
#define RFIRQ_PORT   gpioPortF

#define NRF_USART_SPI   USART0   /* which usart module are we using for the nrf24l01+ spi */
#define NRF_USART_LOCATION 0     /* which location are we using */

#define DEBUGPIN_SET   (GPIO->P[DEBUGPIN_PORT].DOUTSET = 1 << DEBUGPIN_N)
#define DEBUGPIN_CLEAR   (GPIO->P[DEBUGPIN_PORT].DOUTCLR = 1 << DEBUGPIN_N)

#define OPA_INIT_UNITY_GAIN_WDAC_RR                                                       \
  {                                                                               \
    opaNegSelUnityGain,             /* Unity gain.                             */ \
    opaPosSelDac,                /* Pos input from pad.                     */ \
    opaOutModeAll,                 /* Main output enabled.                    */ \
    opaResSelDefault,               /* Resistor ladder is not used.            */ \
    opaResInMuxDisable,             /* Resistor ladder disabled.               */ \
    0,                              /* No alternate outputs enabled.           */ \
    0x0F,   /* Default bias setting.             */       \
    true , /* Default half-bias setting.        */       \
    true,                          /* No low pass filter on pos pad.          */ \
    true,                          /* No low pass filter on neg pad.          */ \
    false,                          /* No nextout output enabled.              */ \
    false,                          /* Neg pad disabled.                       */ \
    false,                           /* Pos pad enabled, used as signal input.  */ \
    false,                          /* No shorting of inputs.                  */ \
    false,                          /* Rail-to-rail input enabled.             */ \
    true,                           /* Use factory calibrated opamp offset.    */ \
    0                               /* Opamp offset value (not used).          */ \
  }

#define OPA_INIT_STANDALONE_RR                                                      \
  {                                                                               \
    opaNegSelNegPad,             /* Unity gain.                             */ \
    opaPosSelPosPad,                /* Pos input from pad.                     */ \
    opaOutModeAll,                 /* Main output enabled.                    */ \
    opaResSelDefault,               /* Resistor ladder is not used.            */ \
    opaResInMuxDisable,             /* Resistor ladder disabled.               */ \
    DAC_OPA1MUX_OUTPEN_OUT3 | DAC_OPA1MUX_OUTPEN_OUT2 ,                              /* Op-amp location 3           */ \
    0x0F, /* Default bias setting.             */       \
    true , /* Default half-bias setting.        */       \
    true,                          /* No low pass filter on pos pad.          */ \
    true,                          /* No low pass filter on neg pad.          */ \
    false,                          /* No nextout output enabled.              */ \
    true,                          /* Neg pad enabled.                       */ \
    true,                           /* Pos pad enabled, used as signal input.  */ \
    false,                          /* No shorting of inputs.                  */ \
    false,                          /* Rail-to-rail input enabled.             */ \
    true,                           /* Use factory calibrated opamp offset.    */ \
    0                               /* Opamp offset value (not used).          */ \
  }
#endif /* __HARDWARE_H */