#include "Watchdog.h"
#include <Arduino.h>

Watchdog::Watchdog() {
  WDOG_TOVALH = 0x006d;
  WDOG_TOVALL = 0xdd00;
  WDOG_PRESC  = 0x400;
}

void  Watchdog::feed(){
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}
