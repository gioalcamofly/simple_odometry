#include "OdomRobot.h"

const double OdomRobot::FORWARD_SPEED = 0.2;
const double OdomRobot::SPIN_SPEED = 0.2;
const double OdomRobot::SQUARE_SIDE_LENGTH = 3;
const double OdomRobot::RIGHT_ANGLE = -M_PI/2;
const double OdomRobot::MIN_SCAN_ANGLE = 0.0/180*M_PI;
const double OdomRobot::MAX_SCAN_ANGLE = +60.0/180*M_PI;
const float OdomRobot::MIN_DIST_FROM_OBSTACLE = 0.5;


OdomRobot::OdomRobot() {

    initializeMap();

    velocityPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    odometrySubscriber = node.subscribe("odom", 1, &OdomRobot::odomCallback, this);
    laserScanSubscriber = node.subscribe("scan", 1, &OdomRobot::scanCallback, this);

}

void OdomRobot::initializeMap() {

    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            map[i][j] = notVisited;
        }
    }

    map[0][0] = actual;

}

void OdomRobot::odomCallback(const nav_msgs::Odometry::ConstPtr& odom){


    int nextX, nextY;
    createNextPosition(&nextX, &nextY);

    int actualX, actualY;
    getApproximateCoordinates(&actualX, abs(odom->pose.pose.position.x));
    getApproximateCoordinates(&actualY, abs(odom->pose.pose.position.y));

    ROS_INFO("actualX -> %d, actualY -> %d", actualX, actualY);

    if (nextX == floor(odom->pose.pose.position.x) &&
        nextY == floor(odom->pose.pose.position.y)) {
            actualMovement = *movements[STOP_MOVEMENT];
    }

}

void OdomRobot::createNextPosition(int* nextX, int* nextY) {

    *nextX = actualPosition[1] + actualMovement.getNextPosition()[1];
    *nextY = actualPosition[0] + actualMovement.getNextPosition()[0];

}

void OdomRobot::getApproximateCoordinates(int* actualPosition, double odomPosition) {

    if (ceil(odomPosition) - odomPosition <= 0.1) {
        *actualPosition = ceil(odomPosition);
    } else {
        *actualPosition = floor(odomPosition);
    }

}


void OdomRobot::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    bool isObstacleInFront = false;

    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex + 1; currIndex<= maxIndex; currIndex++) {
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
            isObstacleInFront = actualMovement.isObstaculizable();
            break;
        }
    }

    if (isObstacleInFront) {
        ROS_INFO("Stop!");
        setNextCoordinateAsOccupied();
    } 

}

void OdomRobot::setNextCoordinateAsOccupied() {

    int nextX, nextY;
    createNextPosition(&nextX, &nextY);

    map[nextY][nextX] = occupied;

}


double OdomRobot::getYaw(geometry_msgs::Quaternion orientation) {
    tf::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    );
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

void OdomRobot::move() {

    velocityPublisher.publish(actualMovement.getVelocity());

}

void OdomRobot::start() {

    ros::Rate rate(1000.0);

    actualMovement = *movements[FORWARD_MOVEMENT];

    loops = 0;

    while(ros::ok()) {

        move();

        ros::spinOnce();
        rate.sleep();

    }
}