#include <Arduino.h>
#include "VoltageMonitor.h"
#include "config.h"

double getVoltage()
{
    double raw_in = analogRead(BATTERY_VOLTAGE);
    double voltage = map(raw_in, 0, 979, 0, 14);
    return(voltage);
}
