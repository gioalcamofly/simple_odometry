#pragma once

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>


class Movement {

    public:
        
        geometry_msgs::Twist getVelocity() {
            return velocity;
        };

        bool isObstaculizable() {
            return obstaculizable;
        };
        
        int* getNextPosition() {
            return nextPosition;
        }

    protected:

        bool obstaculizable;

        static constexpr double SPEED = 0.5;

        geometry_msgs::Twist velocity;

        int nextPosition[2];
};
