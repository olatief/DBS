#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "efm32.h"
#include "efm32_cmu.h"
#include "efm32_emu.h"
#include "efm32_chip.h"
#include "efm32_dac.h"
#include "efm32_timer.h"
#include "efm32_letimer.h"
#include "efm32_gpio.h"
#include "efm32_opamp.h"

#include "stim.h"
#include "radio.h"
#include "prog_types.h"
#include "hardware.h"

#define TXTIMER_PERIOD (2-1)

/* Max number of HFCycles for higher accuracy */
#define HFCYCLES            65094
/* HF_cycles x ref_freq / HFCLK_freq = (2^20 - 1) x 32768 / 14000000 = 2454 */
#define UP_COUNTER_TUNED    2133
/* Initial HFRCO TUNING value */
#define TUNINGMAX           0xFF


/* Function prototypes */
uint8_t calibrateHFRCO(uint32_t hfCycles, uint32_t upCounterTuned);
void DAC_setup(void);
void TIMER_setup(void);
void LETIMER_setup(void);
void DAC_WriteData(DAC_TypeDef *dac, unsigned int value, unsigned int ch);
void setupSWO(void);
void eADesigner_Init(void);

/* Global Variables */

PROG_TypeDef currentProg;
uint8_t state_TXPKT = 0;

PROG_TypeDef default_prog = { 0x03, 100,0,0,0,0};

void DEBUG_setup()
{
  /* Set DEBUG_PIN as output */
  GPIO_PinModeSet(DEBUGPIN_PORT, DEBUGPIN_N, gpioModePushPull, 0);
}

int main()
{
  uint32_t DAC_Value = 0;
  OPAMP_Init_TypeDef config0 =  OPA_INIT_UNITY_GAIN_WDAC_RR;
     OPAMP_Init_TypeDef config1 =  OPA_INIT_STANDALONE_RR;
   
 
  /* Chip errata */
  CHIP_Init();
 // EMU_EnterEM4();
 // setupSWO();
   
  
  eADesigner_Init();

  RADIO_setup();
 /* 
  GPIO->P[gpioPortA].DOUTSET = 1 << 1;
  GPIO->P[gpioPortA].DOUTSET = 1 << 0;
  GPIO->P[gpioPortA].DOUTCLR = 1 << 1;
  GPIO->P[gpioPortA].DOUTCLR = 1 << 0;
  */
  
  /* Initialise the DAC */
  DAC_setup();
  DAC_Enable(DAC0, 0, true);
 // DAC_Enable(DAC0, 1, true);
   
  /* Calculate output to be 0.5 V. */
   DAC_Value = (uint32_t)((0.5 * 4096) / 2.5);
   currentProg.amplitude = DAC_Value;
     /* Write new value to DAC register */
  DAC_WriteData(DAC0, 0, 0);
  /* Write new value to DAC register */
  DAC_WriteData(DAC0, DAC_Value, 0);
 // DAC_WriteData(DAC0, 0, 0);
  // 
    OPAMP_Enable(DAC0, OPA0, &config0);
    
    DAC0->OPACTRL &= ~DAC_OPACTRL_OPA0EN;
    OPAMP_Enable(DAC0, OPA1, &config1); 
   
        /*Turn on the DAC clock*/
   
    // DAC_Enable(DAC0, 1, false);
  
  
  /*Disable OPA0. This is done because we want to use OPA0 as a part of the DAC. 
    The configurations set above are still there.*/
    DAC0->OPACTRL &= ~DAC_OPACTRL_OPA0EN;
  //  DAC0->OPACTRL |= DAC_OPACTRL_OPA1EN;

    
 TIMER_setup();  
 
  calcTimerParams(100000,63);
 startStim();

  LETIMER_setup();
  DEBUG_setup(); 

   currentProg.enable = true;
 
  
  for(;;)
  { 
    if(state_TXPKT == true) // We have to transmit a packet to check if there's any messages for us
    {
      state_TXPKT = false;
     
      RADIO_TX((uint8_t*)(&currentProg), sizeof(currentProg));
      
    }
    
    if(procPayload)
    {
      procPayload = false;
      PROG_TypeDef *prog =  (PROG_TypeDef *)progPayload;
      
      /*
      prog->amplitude = __REV16(prog->amplitude);
      prog->period = __REV16(prog->period);
      prog->posPulseWidth = __REV16(prog->posPulseWidth);
      prog->negPulseWidth = __REV16(prog->negPulseWidth);
      */
      if( 0 == prog->period )
      {
        prog->enable = false;
      } else
      {
        prog->enable = true;
      }
      
      reprogStimTimers( *prog );
      
      if(prog->enable == false)
      {
        DAC_Enable(DAC0, 0, false);
      }
      else
      {
        DAC_WriteData(DAC0, (prog->amplitude<<12)/2500 , 0); // convert from mV to LSB for DAC input
        DAC_Enable(DAC0, 0, true);
      }
      
      currentProg = *prog;
    }
    
    if(currentProg.enable) // Checks if clock for DAC and stimulation still needs to be on
    {
      EMU_EnterEM1();
    } else
    {
      EMU_EnterEM2(false);
    }
  }
  
  // return 0;
}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{ 
   volatile int flags = TIMER_IntGet(TIMER0);
  
  if(flags & TIMER_IF_CC0) // Start Stim
  {
    TIMER_IntClear(TIMER0, TIMER_IF_CC0);
   DAC_WriteData(DAC0, currentProg.amplitude, 0);
  }
  else if(flags & TIMER_IF_CC1) // End Stim
  {
    TIMER_IntClear(TIMER0, TIMER_IF_CC1);
  //  DAC_WriteData(DAC0, 0, 0);
    DAC0->CH0CTRL &= ~DAC_CH0CTRL_EN;
    DAC0->CH1CTRL &= ~DAC_CH1CTRL_EN;
  } 
  else if(flags & TIMER_IF_CC2) // Warm up DAC and opamps
  {
    TIMER_IntClear(TIMER0, TIMER_IF_CC2); // before we stimulate we have to turn on the opamps
    DAC0->CH0CTRL |= DAC_CH0CTRL_EN;
    DAC0->CH1CTRL |= DAC_CH1CTRL_EN;
 //   DAC_WriteData(DAC0, 0, 0);
  } 
  
}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{ 
  /* clear flag for TIMER0 overflow interrupt */
  volatile int flags = TIMER_IntGet(TIMER1);
  
  if(flags & TIMER_IF_CC0) // Assuming we only have CC0 and CC1 enabled
  {
    TIMER_IntClear(TIMER1, TIMER_IF_CC0);
  } else
  {
    TIMER_IntClear(TIMER1, TIMER_IF_CC1);
  }
}

void TIMER_setup(void)
{
   /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);
    /* Enable clock for TIMER0 module */
//  CMU_ClockEnable(cmuClock_TIMER1, true); 
  /* Set CC0 location 0 pin (PC13) as output */
  GPIO_PinModeSet(TIMERPOS_PORT, TIMERPOS_N, gpioModePushPull, 0);
  /* Set CC1 location 0 pin (PC14) as output */
  GPIO_PinModeSet(TIMERNEG_PORT, TIMERNEG_N, gpioModePushPull, 0);
  /* Select CC0 channel parameters */
  TIMER_InitCC_TypeDef timerCC0Init = 
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionClear,
    .cmoa       = timerOutputActionToggle,
    .mode       = timerCCModeCompare,
    .filter     = false,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,
  };
  
  /* Configure CC channel 0 */
  TIMER_InitCC(TIMER0, 0, &timerCC0Init);

  /* Route CC0 to location 3 (PD1) and enable pin */  
  TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC0); 
 
   /* Select CC1 channel parameters */
  TIMER_InitCC_TypeDef timerCC1Init = 
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionSet,
    .cmoa       = timerOutputActionToggle,
    .mode       = timerCCModeCompare,
    .filter     = false,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,
  };
  
  /* Configure CC channel 1 */
  TIMER_InitCC(TIMER0, 1, &timerCC1Init);
  TIMER_InitCC(TIMER0, 2, &timerCC1Init);
//  TIMER_InitCC(TIMER1, 0, &timerCC1Init);
//  TIMER_InitCC(TIMER1, 1, &timerCC1Init);
  /* Route CC1 to location 3 (PD1) and enable pin */  
  TIMER0->ROUTE |= (TIMER_ROUTE_CC1PEN | TIMER_ROUTE_LOCATION_LOC0); 
  
  /* Set Top Value */
  TIMER_TopSet(TIMER0, 1000);
//  TIMER_TopSet(TIMER1, 1000);
  /* Set compare value starting at 0 - it will be incremented in the interrupt handler */
  TIMER_CompareBufSet(TIMER0, 0, 0);

  /* Set compare value starting at 0 - it will be incremented in the interrupt handler */
  TIMER_CompareBufSet(TIMER0, 1, 0);
  /* Select timer parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = false,
    .debugRun   = false,
    .prescale   = timerPrescale1,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = true,
  };
  
  /* Enable overflow interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IF_CC1);
  TIMER_IntEnable(TIMER0, TIMER_IF_CC2);
  TIMER_IntEnable(TIMER0, TIMER_IF_CC0);
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);
  
  /* Configure timer */
  TIMER_Init(STIM_TIMER, &timerInit);
  TIMER_Init(DAC_TIMER, &timerInit);
}
/**************************************************************************//**
 * @brief  Setup DAC
 * Configures and starts the DAC
 *****************************************************************************/
void DAC_setup(void)
{
  /* Use default settings */
  DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT;
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;
 DAC_InitChannel_TypeDef initChannel1 = DAC_INITCHANNEL_DEFAULT;
   OPAMP_Init_TypeDef config0 =  OPA_INIT_UNITY_GAIN_WDAC_RR;

  /* Enable the DAC clock */
   CMU_ClockEnable(cmuClock_DAC0, true);

  /* Set longest refresh cycle time and activate sample-hold mode */
 // init.refresh  = dacRefresh8;
  init.convMode = dacConvModeContinuous; // dacConvModeContinuous; // dacConvModeSampleHold;
  init.outMode = dacOutputPin;
  //init.ch0ResetPre = true;
  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 100kHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the current value actually is. */
 //  init.prescale = DAC_PrescaleCalc(100000 , 0);

  /* Set reference voltage to 1.25V */
  init.reference = dacRef2V5;
  init.lpEnable = false;
  /* Initialize the DAC and DAC channel. */
  DAC_Init(DAC0, &init);

  initChannel.refreshEnable = false;
 // initChannel.refreshEnable = true;
  initChannel1.enable = 0;
  DAC_InitChannel(DAC0, &initChannel, 0);
 // DAC_InitChannel(DAC0, &initChannel1, 1);
}

/**************************************************************************//**
 * @brief  LETIMER_setup
 * Configures and starts the LETIMER0
 *****************************************************************************/
void LETIMER_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LETIMER0, true);  
  CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_32768);
  
  /* Set initial compare values for COMP0 and COMP1 
     COMP0 keeps it's value and is used as TOP value
     for the LETIMER.
     COMP1 gets decremented through the program execution
     to generate a different PWM duty cycle */
  LETIMER_CompareSet(LETIMER0, 0, TXTIMER_PERIOD);
  
  /* Repetition values must be nonzero so that the outputs
     return switch between idle and active state */
  LETIMER_RepeatSet(LETIMER0, 0, 0x01);
  LETIMER_RepeatSet(LETIMER0, 1, 0x01);
  
  /* Route LETIMER to location 0 (PD6 and PD7) and enable outputs */
//  LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_OUT1PEN | LETIMER_ROUTE_LOCATION_LOC0;
  
  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit = 
  {
  .enable         = true,                   /* Start counting when init completed. */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
//  .out0Pol        = 0,                      /* Idle value for output 0. */
 // .out1Pol        = 0,                      /* Idle value for output 1. */
 // .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
//  .ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
  .repMode        = letimerRepeatFree       /* Count until stopped */
  };
  
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit); 
  
  /* Enable underflow interrupt */  
  LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);  
  
  /* Enable LETIMER0 interrupt vector in NVIC*/
  NVIC_EnableIRQ(LETIMER0_IRQn);
}


/**************************************************************************//**
 * @brief LETIMER0_IRQHandler
 * Interrupt Service Routine for LETIMER
 *****************************************************************************/
void LETIMER0_IRQHandler(void)
{ 
  /* Clear LETIMER0 underflow interrupt flag */
  LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
  
  state_TXPKT = 1;
}


/**************************************************************************//**
 * @brief  Write DAC conversion value
 *****************************************************************************/
void DAC_WriteData(DAC_TypeDef *dac, unsigned int value, unsigned int ch)
{
  /* Write data output value to the correct register. */
  if (!ch)
  {
    dac->CH0DATA = value;
  }
  else
  {
    dac->CH1DATA = value;
  }
}


void setupSWO(void)
{
  uint32_t *dwt_ctrl = (uint32_t *) 0xE0001000;
  uint32_t *tpiu_prescaler = (uint32_t *) 0xE0040010;
  uint32_t *tpiu_protocol = (uint32_t *) 0xE00400F0;

  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= 1;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  *dwt_ctrl = 0x400113FF;
  /* Set TPIU prescaler to 16. */
  *tpiu_prescaler = 0xf;
  /* Set protocol to NRZ */
  *tpiu_protocol = 2;
  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;
}


/**************************************************************************//**
 * @brief Calibrate HFRCO for 14Mhz against LFXO
 * @param hfCycles down counter initial value
 * @param upCounterTuned value expected for the upCounter when HFRCO is tuned
 *****************************************************************************/
uint8_t calibrateHFRCO(uint32_t hfCycles, uint32_t upCounterTuned)
{
  /* run the calibration routine to know the initial value
   * of the up counter */
  uint32_t upCounter = CMU_Calibrate(hfCycles, cmuOsc_LFXO);
  /* Variable to store the previous upcounter value for comparison */
  uint32_t upCounterPrevious;
  /* Read the initial tuning value */
  uint8_t  tuningVal = CMU_OscillatorTuningGet(cmuOsc_HFRCO);

  /* If the up counter result is smaller than the
   * tuned value, the HFRCO is running
   * at a higher frequency so the HFRCO tuning
   * register has to be decremented */
  if (upCounter < upCounterTuned)
  {
    while (upCounter < upCounterTuned)
    {
      /* store the previous value for comparison */
      upCounterPrevious = upCounter;
      /* Decrease tuning value */
      tuningVal--;
      /* Write the new HFRCO tuning value */
      CMU_OscillatorTuningSet(cmuOsc_HFRCO, tuningVal);
      /* Run the calibration routine again */
      upCounter = CMU_Calibrate(hfCycles, cmuOsc_LFXO);
    }
    /* Check which value goes closer to the tuned one
     * and increase the tuningval if the previous value
     * is closer */
    if ((upCounter - upCounterTuned) > (upCounterTuned - upCounterPrevious))
    {
      tuningVal++;
    }
  }

  /* If the up counter result is higher than the
   * desired value, the HFRCO is running
   * at a higher frequency so the tuning
   * value has to be incremented */
  if (upCounter > upCounterTuned)
  {
    while (upCounter > upCounterTuned)
    {
      /* store the previous value for comparison */
      upCounterPrevious = upCounter;
      /* Increase tuning value */
      tuningVal++;
      /* Write the new HFRCO tuning value */
      CMU_OscillatorTuningSet(cmuOsc_HFRCO, tuningVal);
      /* Run the calibration routine again */
      upCounter = CMU_Calibrate(hfCycles, cmuOsc_LFXO);
    }
    /* Check which value goes closer to the tuned one
     * and decrease the tuningval if the previous value
     * is closer */
    if ((upCounterTuned - upCounter) > (upCounterPrevious - upCounterTuned))
    {
      tuningVal--;
    }
  }

  /* Return final HFRCO tuning value */
  return tuningVal;
}

/*************************************************************************//**
 * @brief energyAware Designer MCU initialization
 *
 * This code is generated by the energyAware Designer appliction to configure
 * the EFM32TG210F32 for application specific operation. 
 *
 * The generated code is a starting point, which might require adjustment for
 * correct operation. Call this function at early initialization.
 *****************************************************************************/
void eADesigner_Init(void)
{
     /* Using HFRCO at 14MHz as high frequency clock, HFCLK */
  /* Calibrate HFRCO with HFXO as a reference */
//  CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
    CMU_HFRCOBandSet(cmuHFRCOBand_1MHz);

 // CMU_OscillatorEnable(cmuOsc_HFXO, false, true);  // disable HFXO to save power
 // CMU_ClockSelectSet(cmuClock_HF,cmuSelect_LFRCO);
 // CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);
  
/* Enable LFXO to clock the LFA clock branch */
  CMU->CTRL &= ~(_CMU_CTRL_LFXOBOOST_MASK);
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
  
    /* Enable LE clock and LFXO oscillator */
  // CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;
 //   CMU->OSCENCMD |= CMU_OSCENCMD_LFXOEN;
    /* Wait until LFXO ready */
    /* Note that this could be done more energy friendly with an interrupt in EM1 */
  //  while (!(CMU->STATUS & CMU_STATUS_LFXORDY)) ;
    
    // CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
    /* Enable GPIO clock */
    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
    
    /* Pin PA0 is configured to Push-pull */
    GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | GPIO_P_MODEL_MODE0_PUSHPULL;
    /* Pin PA1 is configured to Push-pull */
    GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE1_MASK) | GPIO_P_MODEL_MODE1_PUSHPULL;
    /* Pin PA2 is configured to Input enabled */
    GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_INPUT;
    /* Pin PC0 is configured to Push-pull */
    GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | GPIO_P_MODEL_MODE0_PUSHPULL;
    /* To avoid false start, configure output US0_TX as high on PE10 */
    GPIO->P[4].DOUT |= (1 << 10);
    /* Pin PE10 is configured to Push-pull */
    GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK) | GPIO_P_MODEH_MODE10_PUSHPULL;
    /* Pin PE11 is configured to Input enabled */
    GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK) | GPIO_P_MODEH_MODE11_INPUT;
    /* Pin PE12 is configured to Push-pull */
    GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK) | GPIO_P_MODEH_MODE12_PUSHPULL;
    /* To avoid false start, configure output US0_CS as high on PE13 */
    GPIO->P[4].DOUT |= (1 << 13);
    /* Pin PE13 is configured to Push-pull */
    GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK) | GPIO_P_MODEH_MODE13_PUSHPULL;
    /* Pin PF2 is configured to Input enabled */
    GPIO->P[5].MODEL = (GPIO->P[5].MODEL & ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_INPUT;
    
    /* Enable clock for TIMER0 */
    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;
    /* Enable signals CC0, CC1 */
    TIMER0->ROUTE |= TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN;
    
    /* Enable clock for USART0 */
    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_USART0;
    /* Enable signals TX, RX, CLK, CS */
    USART0->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;
   calibrateHFRCO(HFCYCLES, UP_COUNTER_TUNED);
}