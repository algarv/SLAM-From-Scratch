/// \file odometry.cpp
/// \brief Publishes odometry messages for the turtlebot to follow.
///
/// PARAMETERS:
///     rate: ros rate
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
static bool teleporting = false;
static double x = 0, y = 0, w = 0;
std::string body_id, odom_id, wheel_left, wheel_right;
turtlelib::Wheel_Angle wheel_angles = {.L = 0, .R = 0}, old_wheel_angles = {.L = 0, .R = 0};
turtlelib::Wheel_Angular_Velocities wheel_vels;
turtlelib::DiffDrive D;
turtlelib::Twist2D twist;
turtlelib::q pos, old_pos;
static geometry_msgs::TransformStamped odom_tf;
ros::Subscriber js_sub;
ros::Publisher odom_pub;
static ros::ServiceServer pose_service;
nav_msgs::Odometry odom_msg;

void update_odom(const sensor_msgs::JointState &wheels){
/// \brief Receives a wheel joint states and translates into a twist for the odometry message
///
/// \param wheels - wheel joint states

    wheel_angles.L = wheels.position[0];
    wheel_angles.R = wheels.position[1];
    wheel_vels.L = wheels.velocity[0];
    wheel_vels.R = wheels.velocity[1];
    // twist = D.get_twist(wheel_angles, old_wheel_angles);
    twist = D.get_twist(wheel_vels);


    // odom_msg.header.frame_id = "odom";
    odom_msg.twist.twist.linear.x = twist.vx;
    odom_msg.twist.twist.linear.y = twist.vy;
    odom_msg.twist.twist.angular.z = twist.w;
}

bool set_pose(nuturtle_control::set_pose::Request &pose, nuturtle_control::set_pose::Response &response){

    pos.x = pose.x;
    pos.y = pose.y;
    pos.theta = pose.w;

    teleporting = true;

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

    nh.getParam("x0", x);
    nh.getParam("y0", y);
    nh.getParam("theta0", w);

    nh.param<std::string>("odom_id",odom_id,"odom");

    ros::Rate r(rate);

    odom_pub = pub_nh.advertise<nav_msgs::Odometry>("odom", 100);
    js_sub = pub_nh.subscribe("red/joint_states",10,update_odom);

    pose_service = nh.advertiseService("set_pose",set_pose);

    old_pos.theta = w;
    old_pos.x = x;
    old_pos.y = y;

    tf2_ros::TransformBroadcaster odom_broadcaster;

    while(ros::ok()){

        odom_pub.publish(odom_msg);        

        if (teleporting == false){
            pos = D.get_q(wheel_angles, old_wheel_angles, old_pos);
            // pos = D.get_q(twist,old_pos);
        }

        odom_tf.header.stamp = ros::Time::now();
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "blue_base_footprint";
        odom_tf.transform.translation.x = pos.x;
        odom_tf.transform.translation.y = pos.y;
        odom_tf.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, pos.theta);
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        odom_broadcaster.sendTransform(odom_tf);

        old_pos = pos;
        old_wheel_angles = {.L = wheel_angles.L, .R = wheel_angles.R};

        teleporting = false;

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}