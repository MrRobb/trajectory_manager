
#include "TrajectoryManager.hpp"
#include "ros/ros.h"

int main(int argc, char **argv) {
    TrajectoryManager tm(argc, argv);
    ros::spin();
    return 0;
}