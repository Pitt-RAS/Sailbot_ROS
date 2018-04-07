#include <Watchdog.h>

const int led = 13;
const int maxAge = 1000UL;
bool ledState = false;
Watchdog dog(maxAge);

void setup() {

    pinMode(led, OUTPUT);
    ledState = false;
    digitalWrite(led, ledState);
    delay(1000UL);
    ledState = true;
    digitalWrite(led, ledState);
    delay(5000UL);

    // Setup WDT
    noInterrupts();                                         
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                         
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);                                   

    // Set options to enable WDT. You must always do this as a SINGLE write to WDOG_CTRLH
    WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
        WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
        WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
    interrupts();

}

void loop() {
    // dog.feed();
    ledState = !ledState;
    digitalWrite(led, ledState);
    delay(100UL);
    if (!dog.hungry()) {                                 
        noInterrupts();                                 
        dog.refresh();
        interrupts();
        }
}
