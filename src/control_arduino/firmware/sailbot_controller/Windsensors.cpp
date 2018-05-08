#include "Windsensors.h"

Windsensors* isrInstance = NULL;
void setupWindsensorISR(Windsensors* instance);

Windsensors::Windsensors(ros::NodeHandle *_nh) :
    nh(_nh), angleSensor(WINDSENSOR_CS, WINDSENSOR_CLK, WINDSENSOR_DO), windSensorDtUpdated(false) {
    setupWindsensorISR(this);

    angleSensor.begin();

    if ( nh != NULL ) {
        windSensorTickPublisher = new ros::Publisher("wind_sensor_tick", &windSensorTick);
        relativeWindDirectionPublisher = new ros::Publisher("/relative_wind_direction/raw", &relativeWindDirection);

        nh->advertise(*windSensorTickPublisher);
        nh->advertise(*relativeWindDirectionPublisher);
    }
}

void Windsensors::update() {
    if ( windSensorDtUpdated ) {
        windSensorTick.data = windSensorDt;
        windSensorTickPublisher->publish(&windSensorTick);
        windSensorDtUpdated = false;
    }

    relativeWindDirection.data = angleSensor.read();
    relativeWindDirectionPublisher->publish(&relativeWindDirection);
}

void windISR() {
    if ( isrInstance->windSensorDtUpdated )
        return;
    isrInstance->windSensorDt = millis() - isrInstance->windSensorLastTick;
    isrInstance->windSensorLastTick = millis();
    isrInstance->windSensorDtUpdated = true;
}

void setupWindsensorISR(Windsensors *instance) {
    isrInstance = instance;
    attachInterrupt(2, windISR, RISING);
}

