#include "NonblockingRate.h"

NonblockingRate::NonblockingRate(int hz) {
        period = ros::Duration(1.0 / (double)hz);
        nextRun = ros::Time::now() + period;
}

bool NonblockingRate::needsRun() {
    ros::Time now = ros::Time::now();
    if ( nextRun < now ) {
        nextRun = now + period;
        return true;
    }
    return false;
}

