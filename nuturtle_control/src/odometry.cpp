/// \file odometry.cpp
/// \brief Publishes odometry messages for the turtlebot to follow.
///
/// PARAMETERS:
///     rate (rate.yaml): ros rate
///     body_id: The id of the body frame
///     odom_id: The id of the odom frame 
///     wheel_left: The name of the left wheel joint
///     wheel_right: The name of the right wheel joint
///     encoder_ticks_to_rad (diff_params.yaml): The number of radians per encoder tick
///
/// PUBLISHES:
///     odom (nav_msgs/Odometry): publishes the velocity of each wheel
/// 
/// SUBSCRIBERS:
///     joint_states (sensor_msgs/JointState): Receives the wheel joint angles

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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include "nuturtle_control/set_pose.h"

int rate;
std::string body_id, odom_id, wheel_left, wheel_right;
turtlelib::Wheel_Angle saved_phi;
turtlelib::DiffDrive D;
turtlelib::Twist2D twist;
static geometry_msgs::TransformStamped odom_tf;
ros::Subscriber js_sub;
ros::Publisher odom_pub;
static ros::ServiceServer pose_service;
nav_msgs::Odometry odom_msg;

void update_odom(const sensor_msgs::JointState &wheels){
/// \brief Receives a wheel joint states and translates into a twist for the odometry message
///
/// \param wheels - wheel joint states

    turtlelib::Wheel_Angle wheel_angles;
    wheel_angles.L = wheels.position[0];
    wheel_angles.R = wheels.position[1];
    twist = D.get_twist(wheel_angles, saved_phi);

    odom_msg.header.frame_id = odom_id;
    odom_msg.twist.twist.linear.x = twist.vx;
    odom_msg.twist.twist.linear.y = twist.vy;
    odom_msg.twist.twist.angular.z = twist.w;
}

bool set_pose(nuturtle_control::set_pose::Request &pos, nuturtle_control::set_pose::Response &response){

    twist.vx = pos.x;
    twist.vy = pos.y;
    twist.w = pos.w;

    return true;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~"), pub_nh;

    // if (ros::param::has("body_id")){
        ros::param::get("body_id", body_id);
    // }
    // else {
    //     ROS_DEBUG_ONCE("body_id not defined");
    //     ros::shutdown();
    // }

    // if (ros::param::has("wheel_left")){
        ros::param::get("wheel_left", wheel_left);
    // }
    // else {
    //     ROS_DEBUG_ONCE("wheel_left not defined");
    //     ros::shutdown();
    // }

    // if (ros::param::has("wheel_right")){
        ros::param::get("wheel_right", wheel_right);
    // }
    // else {
    //     ROS_DEBUG_ONCE("wheel_right not defined");
    //     ros::shutdown();
    // }

    // if (ros::param::has("rate")){
        ros::param::get("rate", rate);
    // }
    // else {
    //     ROS_DEBUG_ONCE("rate not defined");
    //     ros::shutdown();
    // }

    nh.param<std::string>("odom_id",odom_id,"odom");

    ros::Rate r(rate);

    odom_pub = pub_nh.advertise<nav_msgs::Odometry>("odom", 10);
    js_sub = pub_nh.subscribe("red/joint_states",100,update_odom);

    pose_service = nh.advertiseService("set_pose",set_pose);

    while(1){

        odom_pub.publish(odom_msg);        

        tf2_ros::TransformBroadcaster odom_broadcaster;

        odom_tf.header.stamp = ros::Time::now();
        odom_tf.header.frame_id = odom_id;
        odom_tf.child_frame_id = body_id;
        odom_tf.transform.translation.x = twist.vx;
        odom_tf.transform.translation.y = twist.vy;
        odom_tf.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, twist.w);
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        odom_broadcaster.sendTransform(odom_tf);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}