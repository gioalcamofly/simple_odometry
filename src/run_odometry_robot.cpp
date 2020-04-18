#include "OdomRobot.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "odometry_robot");

    OdomRobot odomRobot;

    odomRobot.start();

    return 0;
}