//#include <stddef.h>

#include <Arduino.h>

#include "utils.h"

bool wait_to_exec(unsigned long *last_exec_ms, unsigned long interval_ms)
{
  unsigned long latched_timestamp_ms = millis();
  unsigned long expected_exec = *last_exec_ms + interval_ms;
  
  if (!(latched_timestamp_ms >= expected_exec))
    return false;
  
  *last_exec_ms = latched_timestamp_ms;
  return true;
}
