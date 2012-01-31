#ifndef __STIM_H
#define __STIM_H

#include <stdint.h>
#include "prog_types.h"

typedef struct
{
  uint32_t Top;  // controls period
  uint32_t CC0;  // controls + phase duration
  uint32_t CC1;  // controls - phase duration
} TIMER_CCParams_TypeDef;


/* all parameter inputs are in us */
TIMER_CCParams_TypeDef calcTimerParams(uint32_t period, uint32_t pulseWidth);
void reprogStimTimers(PROG_TypeDef prog);

void stopStim(void);
void startStim(void);

uint32_t gcd(uint32_t u, uint32_t v);
uint32_t divideRound(uint32_t u, uint32_t v);

#endif /* __STIM_H */