/// \file turtle_interface.cpp
/// \brief The turtle_interface node translates geometry_msgs/Twist messages to directions for the turtlebot.
///
/// PARAMETERS:
///     rate (rate.yaml): ros rate
///     encoder_ticks_to_rad (diff_params.yaml): The number of radians per encoder tick

/// PUBLISHES:
///     red/wheel_cmd (nuturtlebot_msgs/WheelCommands): publishes the velocity of each wheel
///     joint_states (sensor_msgs/JointState): publishes the angle of each wheel
/// 
/// SUBSCRIBERS:
///     turtle_interface/cmd_vel (geometry_msgs/Twist): Recieves a twist command
///     turtle_interface/sensor_data (nuturtlebot_msgs/SensorData): Recieves encoder tick

#include <ros/ros.h>
#include <string>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>
#include <ros/console.h>

static int rate;
static ros::Subscriber cmd_vel_sub, sensor_sub;
static ros::Publisher wheel_pub, js_pub;
static double eticks_rad, mticks_radsec, L_ticks, R_ticks, d_L_ticks, d_R_ticks, saved_L_ticks = 0, saved_R_ticks = 0;
static std::string left_wheel = "red_wheel_left_joint", right_wheel = "red_wheel_right_joint";
static turtlelib::DiffDrive D; 
static turtlelib::Twist2D twist_cmd;
static turtlelib::Wheel_Angular_Velocities vel_cmd, wheel_vels;
static turtlelib::Wheel_Angle wheel_angles;
static sensor_msgs::JointState wheel_msg;
static nuturtlebot_msgs::WheelCommands wheel_vel_msg;

void follow_twist(const geometry_msgs::Twist &wheel_cmd){
/// \brief Recieves a twist message and returns a wheel cmd message.
///
/// \param wheel_cmd - Twist recieved from geometry messages 

    twist_cmd.vx = wheel_cmd.linear.x;
    twist_cmd.vy = wheel_cmd.linear.y;
    twist_cmd.w = wheel_cmd.angular.z;

    vel_cmd = D.wheel_vel(twist_cmd);
    // ROS_WARN("vx: %f,vy: %f,w: %f",twist_cmd.vx,twist_cmd.vy,twist_cmd.w);//edit

    wheel_vel_msg.left_velocity = vel_cmd.L / mticks_radsec;
    wheel_vel_msg.right_velocity = vel_cmd.R / mticks_radsec;

    if (wheel_vel_msg.left_velocity > 256){
        wheel_vel_msg.left_velocity = 256;
    }

    if (wheel_vel_msg.left_velocity < -256){
        wheel_vel_msg.left_velocity = -256;
    }

    if (wheel_vel_msg.right_velocity > 256){
        wheel_vel_msg.right_velocity = 256;
    }

    if (wheel_vel_msg.right_velocity < -256){
        wheel_vel_msg.right_velocity = -256;
    }

}

void calc_joint_states(const nuturtlebot_msgs::SensorData &sensor_data){
/// \brief Receives a sensor data message and uses the encoder ticks to return wheel positions.
///
/// \param sensor_data - Sensor data recieved from nuturtlebot_msgs
    
    L_ticks = sensor_data.left_encoder;
    R_ticks = sensor_data.right_encoder;

    d_L_ticks = L_ticks - saved_L_ticks;
    d_R_ticks = R_ticks - saved_R_ticks;

    wheel_angles.L = (L_ticks * eticks_rad);
    wheel_angles.R = (R_ticks * eticks_rad);

    // wheel_vels.L = d_L_ticks * mticks_radsec;
    // wheel_vels.R = d_R_ticks * mticks_radsec;

    saved_L_ticks = L_ticks;
    saved_R_ticks = R_ticks;
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "turtle_interface");
    
    ros::NodeHandle nh("~"), pub_nh;

    // if (ros::param::has("/turtle_interface/encoder_ticks_to_rad")){
        ros::param::get("red/encoder_ticks_to_rad", eticks_rad);
    // }
    // else {
    //     ROS_DEBUG_ONCE("encoder_ticks_to_rad not defined");
    //     // ros::shutdown();
    // }

    // if (ros::param::has("/turtle_interface/motor_cmd_to_radsec")){
        ros::param::get("red/motor_cmd_to_radsec", mticks_radsec);
    // }
    // else {
        // ROS_DEBUG_ONCE("motor_cmd_to_radsec not defined");
        // ros::shutdown();
    // }

    // if (ros::param::has("turtle_interface/rate")){
        ros::param::get("rate", rate);
    // }
    // else {
    //     ROS_DEBUG_ONCE("rate not defined");
    //     // ros::shutdown();
    // }

    ros::Rate r(rate);

    cmd_vel_sub = pub_nh.subscribe("cmd_vel",10,follow_twist); 
    sensor_sub = pub_nh.subscribe("sensor_data",10,calc_joint_states);

    wheel_pub = pub_nh.advertise<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 10);
    js_pub = pub_nh.advertise<sensor_msgs::JointState>("red/joint_states", 10);

    // wheel_msg.header.frame_id = "world";
    wheel_msg.header.stamp = ros::Time::now();
    wheel_msg.name = {left_wheel, right_wheel};
    wheel_msg.position = {0, 0};
    wheel_msg.velocity = {0, 0};

    while(ros::ok()){

        wheel_msg.header.stamp = ros::Time::now();
        wheel_msg.name = {left_wheel, right_wheel};
        wheel_msg.position = {wheel_angles.L, wheel_angles.R};
        wheel_msg.velocity = {vel_cmd.L, vel_cmd.R};

        wheel_pub.publish(wheel_vel_msg);  
        
        js_pub.publish(wheel_msg);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
