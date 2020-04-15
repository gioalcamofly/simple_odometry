#include "OdomRobot.h"

const double OdomRobot::FORWARD_SPEED = 0.5;
const double OdomRobot::SPIN_SPEED = 0.5;
const double OdomRobot::MAX_FORWARD = 10.0;


OdomRobot::OdomRobot() {

    vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);
    odom_sub = node.subscribe("odom", 1, &OdomRobot::odomCallback, this);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    forwardOrSpin = true;

}

void OdomRobot::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    if ((odom->pose.pose.position.x - start_x) >= MAX_FORWARD) {
        stopMoving();
    }

}

void OdomRobot::publishForward() {

    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    double delta_x = actualSpeed * dt;

    end_x += delta_x;
    ROS_INFO("start_x = %lf", start_x);
    ROS_INFO("delta_x = %lf", delta_x);
    ROS_INFO("end_x = %lf", end_x);

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

void OdomRobot::stopMoving() {

    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    vel_pub.publish(msg);
    actualSpeed = 0.0;

}

void OdomRobot::turnRight() {

    geometry_msgs::Twist msg;
    msg.angular.z = SPIN_SPEED;
    vel_pub.publish(msg);
    publishSpin();

}

void OdomRobot::print() {

    ROS_INFO("Distancia recorrida -> %lf", end_x - start_x);

}

void OdomRobot::startMoving() {

    ros::Rate rate(1.0);

    actualSpeed = FORWARD_SPEED;
    while(node.ok()) {

        if (forwardOrSpin) {
            moveForward();
            print();
        } else if (!forwardOrSpin) {
            turnRight();
        }

        ros::spinOnce();
        rate.sleep();
    }

}