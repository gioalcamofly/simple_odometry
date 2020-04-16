#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>

class OdomRobot {
    public:
        const static double FORWARD_SPEED;
        const static double SPIN_SPEED;
        const static int LOOPS;
        const static double MAX_FORWARD;
        const static double MAX_SPIN;


        OdomRobot();
        void startMoving();

    private:

        ros::NodeHandle node;

        ros::Publisher vel_pub;
        ros::Publisher odom_pub;
        ros::Subscriber odom_sub;
        tf::TransformBroadcaster odom_broadcaster;

        double start_x = 0.0;
        double end_x = 0.0;
        double start_y = 0.0;
        double end_y = 0.0;
        double start_th = 0.0;
        double end_th = 0.0;

        double actualSpeed;
        int actualLoops;


        bool forwardOrSpin;
        int loopCount;

        ros::Time current_time, last_time;

        void moveForward();
        void turnRight();
        void stopMovingForward();
        void stopSpinning();
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void odomPublish();
        void publishForward();
        void publishSpin();
        void print();
};