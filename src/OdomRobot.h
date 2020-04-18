#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include "ForwardMovement.cpp"
#include "BackwardMovement.cpp"
#include "LeftMovement.cpp"
#include "RightMovement.cpp"
#include "TurningLeftMovement.cpp"
#include "TurningRightMovement.cpp"
#include "StopMovement.cpp"

class OdomRobot {

    public:

        const static double FORWARD_SPEED, 
                            SPIN_SPEED, 
                            SQUARE_SIDE_LENGTH, 
                            RIGHT_ANGLE,
                            MIN_SCAN_ANGLE,
                            MAX_SCAN_ANGLE;
        const static float MIN_DIST_FROM_OBSTACLE;
        const static int SQUARE_SIDES = 4;

        OdomRobot();
        void start();

    private:

        const static int FORWARD_MOVEMENT = 0;
        const static int BACKWARD_MOVEMENT = 1;
        const static int LEFT_MOVEMENT = 2;
        const static int RIGHT_MOVEMENT = 3;
        const static int TURNING_LEFT_MOVEMENT = 4;
        const static int TURNING_RIGHT_MOVEMENT = 5;
        const static int STOP_MOVEMENT = 6;

        const static int MAP_SIZE = 10;
        const static int STEP_SIZE = 0.5;



        ros::NodeHandle node;

        ros::Publisher velocityPublisher, odometry;
        ros::Subscriber odometrySubscriber, laserScanSubscriber;

        

        enum MapStates { actual, visited, notVisited, occupied };

        MapStates map[MAP_SIZE][MAP_SIZE];
        int actualPosition[2] = {0, 0};

        Movement actualMovement;
        Movement* movements[7] {
            new ForwardMovement(),
            new BackwardMovement(),
            new LeftMovement(),
            new RightMovement(),
            new TurningLeftMovement(),
            new TurningRightMovement(),
            new StopMovement()
        };

        int loops;
        
        void move();

        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        double getYaw(geometry_msgs::Quaternion orientation);

        void initializeMap();
        void setNextCoordinateAsOccupied();

        void createNextPosition(int* nextX, int* nextY);
        void getApproximateCoordinates(int* actualPosition, double odomPosition);

};