#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>

class OdomRobot {
    public:
        const static double FORWARD_SPEED;
        const static double SPIN_SPEED;
        const static bool LOOPS;


        OdomRobot();
        void startMoving();

    private:

        ros::NodeHandle node;

        ros::Publisher vel_pub;
        ros::Publisher odom_pub;
        ros::Subscriber odom_sub;
        tf::TransformBroadcaster odom_broadcaster;

        double x = 0.0;
        double y = 0.0;
        double th = 0.0;

        bool forwardOrSpin, stopMoving;
        int loopCount;

        ros::Time current_time, last_time;

        void moveForward();
        void turnRight();
        void stopMoving();
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void odomPublish();
        void publishForward();
        void publishSpin();
};