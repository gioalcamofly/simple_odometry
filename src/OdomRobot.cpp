#include "OdomRobot.h"

const double OdomRobot::FORWARD_SPEED = 0.5;
const double OdomRobot::SPIN_SPEED = 0.5;
const double OdomRobot::MAX_FORWARD = 5.0;
const double OdomRobot::MAX_SPIN = -M_PI/2;
const int OdomRobot::LOOPS = 2;


OdomRobot::OdomRobot() {

    vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);
    odom_sub = node.subscribe("odom", 1, &OdomRobot::odomCallback, this);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    forwardOrSpin = true;

    actualLoops = 0;
}

double getYaw(geometry_msgs::Quaternion orientation) {
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

void OdomRobot::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    if ((odom->pose.pose.position.x - start_x) >= MAX_FORWARD) {
        stopMovingForward();
    }

    double yaw = getYaw(odom->pose.pose.orientation);

    if ((yaw - start_th) <= MAX_SPIN) {
        stopSpinning();
    }

    if (forwardOrSpin) {
        ROS_INFO("Distancia recorrida -> %lf", end_x - start_x);
    } else {
        ROS_INFO("Orientación = %lf", yaw);
    }

}

void OdomRobot::publishForward() {

    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    double delta_x = actualSpeed * dt;

    end_x += delta_x;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(end_th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = end_x;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = end_x;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = FORWARD_SPEED;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub.publish(odom);

    last_time = current_time;    

}

void OdomRobot::publishSpin() {

    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    double delta_th = SPIN_SPEED * dt;

    end_th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(end_th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = end_x;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = end_x;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = SPIN_SPEED;

    odom_pub.publish(odom);

    last_time = current_time;
}

void OdomRobot::odomPublish() {

    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    double delta_x = FORWARD_SPEED * dt;
    double delta_th = SPIN_SPEED * dt;

    end_x += delta_x;
    end_th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(end_th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = end_x;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = end_x;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = FORWARD_SPEED;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = SPIN_SPEED;

    odom_pub.publish(odom);

    last_time = current_time;

}

void OdomRobot::moveForward() {

    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED;
    vel_pub.publish(msg);   
    publishForward();

}

void OdomRobot::stopMovingForward() {

    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    vel_pub.publish(msg);
    actualSpeed = 0.0;
    end_x = 0.0;
    forwardOrSpin = false;
    publishForward();
}

void OdomRobot::turnRight() {

    geometry_msgs::Twist msg;
    msg.angular.z = -SPIN_SPEED;
    vel_pub.publish(msg);
    publishSpin();

}

void OdomRobot::stopSpinning() {

    geometry_msgs::Twist msg;
    msg.angular.z = 0.0;
    vel_pub.publish(msg);
    end_th = 0.0;
    actualLoops += 1;
    forwardOrSpin = true;
    publishSpin();

}

void OdomRobot::startMoving() {

    ros::Rate rate(10.0);

    actualSpeed = FORWARD_SPEED;
    while(node.ok() && actualLoops < LOOPS) {

        if (forwardOrSpin) {
            moveForward();
        } else if (!forwardOrSpin) {
            turnRight();
        }

        ros::spinOnce();
        rate.sleep();
    }

}