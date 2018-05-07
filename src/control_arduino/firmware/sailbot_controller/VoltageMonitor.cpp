#include <Arduino.h>
#include "VoltageMonitor.h"
#include "config.h"

VoltageMonitor::VoltageMonitor(ros::NodeHandle* _nh) : nh(_nh) {
    if ( shouldUseROS ) {
        batVoltPub = new ros::Publisher("voltage", &volt_msg);
        nh->advertise(*batVoltPub);
    }
}

double VoltageMonitor::getVoltage() {
    double raw_in = analogRead(BATTERY_VOLTAGE);
    double voltage = map(raw_in, 0, 979, 0, 14);
    return voltage;
}

void VoltageMonitor::update() {
    if( shouldUseROS ) {
        volt_msg.data = getVoltage();
        batVoltPub->publish(&volt_msg);
    }
}
