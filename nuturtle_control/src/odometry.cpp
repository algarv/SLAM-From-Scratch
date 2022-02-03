#include <ros/ros.h>
#include <string>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>

int rate;
std::string body_id, odom_id, wheel_left, wheel_right;
turtlelib::Wheel_Angle saved_phi;
turtlelib::DiffDrive D;
turtlelib::Twist2D twist;
ros::Subscriber js_sub;
ros::Publisher odom_pub;
nav_msgs::Odometry odom_msg;

void update_odom(const sensor_msgs::JointState &wheels){

    turtlelib::Wheel_Angle wheel_angles;
    wheel_angles.L = wheels.position[0];
    wheel_angles.R = wheels.position[1];
    twist = D.get_twist(wheel_angles, saved_phi);

    odom_msg.header.frame_id = odom_id;
    odom_msg.twist.twist.linear.x = twist.vx;
    odom_msg.twist.twist.linear.y = twist.vy;
    odom_msg.twist.twist.angular.z = twist.w;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~"), pub_nh;

    if (ros::param::has("body_id")){
        ros::param::get("body_id", body_id);
    }
    else {
        ROS_DEBUG_ONCE("body_id not defined");
        ros::shutdown();
    }

    if (ros::param::has("wheel_left")){
        ros::param::get("wheel_left", wheel_left);
    }
    else {
        ROS_DEBUG_ONCE("wheel_left not defined");
        ros::shutdown();
    }

    if (ros::param::has("wheel_right")){
        ros::param::get("wheel_right", wheel_right);
    }
    else {
        ROS_DEBUG_ONCE("wheel_right not defined");
        ros::shutdown();
    }

    if (ros::param::has("rate")){
        ros::param::get("rate", rate);
    }
    else {
        ROS_DEBUG_ONCE("rate not defined");
        ros::shutdown();
    }

    nh.param<std::string>("odom_id",odom_id,"odom");

    ros::Rate r(rate);

    odom_pub = pub_nh.advertise<nav_msgs::Odometry>("odom", 100);

    js_sub = nh.subscribe("joint_states",100,update_odom);

    while(1){

        odom_pub.publish(odom_msg);        

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}