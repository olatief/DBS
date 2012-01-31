#include "stim.h"
#include "efm32_cmu.h"
#include "efm32_timer.h"
#include "intrinsics.h"
#include "hardware.h"

void reprogStimTimers(PROG_TypeDef prog)
{
  if(prog.enable)
  {
    calcTimerParams(1000*prog.period, prog.posPulseWidth);
    startStim();
  } else {
    stopStim();
  }
}
uint16_t preCalc[16] = {0, 1, 2, 2, 3,3,3,3,4,4,4,4,4,4,4,4}; 

uint32_t calcTimerPS(uint32_t count)
{
  count >>= 16;
  return preCalc[count];
}

/* all parameter inputs are in us */
TIMER_CCParams_TypeDef calcTimerParams(uint32_t period, uint32_t pulseWidth)
{
  uint32_t timerFreq;
  uint32_t topCount;
  uint32_t negCount;
  uint32_t posCount;
  uint32_t durationCount;
  
  TIMER_CCParams_TypeDef ret; 
  
 // uint32_t oldTimerPS = ((STIM_TIMER->CTRL & _TIMER_CTRL_PRESC_MASK)>>_TIMER_CTRL_PRESC_SHIFT);
  uint32_t newTimerPS;
  
  if(period > 10000000)
    period = 10000000;
  if(pulseWidth > period/2)
    pulseWidth = period/2;
  
  timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0);
  
  topCount = (uint64_t)period * (uint64_t)timerFreq/(uint32_t)1000000;
  
  durationCount = (uint64_t)pulseWidth * (uint64_t)timerFreq/1000000;

  /* check if timer will overflow with given parameters */
    newTimerPS = calcTimerPS(topCount);
    
    topCount >>= newTimerPS;
    durationCount >>= newTimerPS;
    DAC_TIMER->CTRL &= ~(_TIMER_CTRL_PRESC_MASK);
    DAC_TIMER->CTRL |= (newTimerPS << _TIMER_CTRL_PRESC_SHIFT);
    
    STIM_TIMER->CTRL &= ~(_TIMER_CTRL_PRESC_MASK); // clear current prescaler
    STIM_TIMER->CTRL |= (newTimerPS << _TIMER_CTRL_PRESC_SHIFT); // set new prescaler

  
  posCount = topCount-durationCount;
  negCount = durationCount;
  
  ret.Top = topCount;
  ret.CC0 = posCount;
  ret.CC1 = negCount;
  
  /** Set Opamp Timer **/
  TIMER_CompareBufSet(DAC_TIMER, 0, posCount-5);
  TIMER_CompareBufSet(DAC_TIMER, 1, negCount-1);
  TIMER_TopBufSet(DAC_TIMER, topCount-1);
  
  /** Set Stim Timer **/
  TIMER_CompareBufSet(STIM_TIMER, 0, posCount-1);
  TIMER_CompareBufSet(STIM_TIMER, 1, negCount-1);
  TIMER_CompareBufSet(STIM_TIMER, 2, posCount-1000);
  TIMER_TopBufSet(STIM_TIMER, topCount-1);
  return ret;
}

void stopStim(void)
{
  // TODO: Disable DAC
  TIMER_Enable(STIM_TIMER, false);
}


void startStim(void)
{
  
  TIMER_Enable(STIM_TIMER, true);
}

/* TODO: not efficient if statement */
uint32_t divideRound(uint32_t u, uint32_t v)
{
  uint32_t res = u/v;
  
  if( (res+1)*v - u < u-res*v)
  {
    ++res;
  }
  
  return res;
}

uint32_t gcd(uint32_t u, uint32_t v)
{
    int shift;
 
     /* GCD(0,x) := x */
     if (u == 0 || v == 0)
       return u | v;
 
     /* Let shift := lg K, where K is the greatest power of 2
        dividing both u and v. */
     for (shift = 0; ((u | v) & 1) == 0; ++shift) {
         u >>= 1;
         v >>= 1;
     }
 
     while ((u & 1) == 0)
       u >>= 1;
 
     /* From here on, u is always odd. */
     do {
         while ((v & 1) == 0)  /* Loop X */
           v >>= 1;
 
         /* Now u and v are both odd, so diff(u, v) is even.
            Let u = min(u, v), v = diff(u, v)/2. */
         if (u < v) {
             v -= u;
         } else {
             uint32_t diff = u - v;
             u = v;
             v = diff;
         }
         v >>= 1;
     } while (v != 0);
 
     return u << shift;
}
