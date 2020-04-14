#include "OdomRobot.h"

const double OdomRobot::FORWARD_SPEED = 0.5;
const double OdomRobot::SPIN_SPEED = 0.5;


OdomRobot::OdomRobot() {

    vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);
    odom_sub = node.subscribe("odom", 1, &OdomRobot::odomCallback, this);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

}

void OdomRobot::publishForward() {

    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    double delta_x = FORWARD_SPEED * dt;

    x += delta_x;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
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

    th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
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

    x += delta_x;
    th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
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

}

void OdomRobot::turnRight() {

    geometry_msgs::Twist msg;
    msg.angular.z = SPIN_SPEED;
    vel_pub.publish(msg);

}

void OdomRobot::startMoving() {

    ros::Rate rate(1.0);
    while(node.ok()) {

        if (!stopMoving && forwardOrSpin) {
            moveForward();
        } else if (!stopMoving && !forwardOrSpin) {
            turnRight();
        }

        ros::spinOnce();
        rate.sleep();
    }

}