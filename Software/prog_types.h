#ifndef __PROGTYPES_H
#define __PROGTYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef __packed struct
{
  uint8_t pkt_info;
  uint16_t amplitude;
  uint16_t period;
  uint16_t posPulseWidth;
  uint16_t negPulseWidth;
  bool enable;
} PROG_TypeDef;

#endif /* __PROGTYPES_H */