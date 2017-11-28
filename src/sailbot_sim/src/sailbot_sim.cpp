#include <ros/ros.h>
#include "BoatSim.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "sailbot_sim");
    ros::NodeHandle n;

    BoatSim sim(n);
    sim.run();
}

