#include "Watchdog.h"
#include <Arduino.h>

Watchdog::Watchdog(unsigned int maxAge){
  lastFed = 0;
  this->maxAge = maxAge;
  WDOG_TOVALH = 0x006d;
  WDOG_TOVALL = 0xdd00;
  WDOG_PRESC  = 0x400;
}

void Watchdog::feed(){
  lastFed = millis();
}

bool Watchdog::hungry(){
  return millis() - lastFed >= maxAge;
}

void Watchdog::refresh(){
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
}
