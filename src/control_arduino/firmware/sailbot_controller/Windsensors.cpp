#include "Windsensors.h"
#include "config.h"

Windsensors* isrInstance = NULL;
void setupWindsensorISR(Windsensors* instance);

Windsensors::Windsensors(ros::NodeHandle *_nh) :
    nh(_nh), windSensorDtUpdated(false) {
    setupWindsensorISR(this);
    pinMode(WINDSENSOR_PWM, INPUT);
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

    relativeWindDirection.data = pulseIn(WINDSENSOR_PWM, HIGH, 2000);
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

