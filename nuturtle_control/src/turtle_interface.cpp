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
///     turtle_interface/sensor_data (nuturtlebot_msgs/SensorData): Recieves encoder tick count


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
static double eticks_rad;
static std::string left_wheel = "red_wheel_left_joint", right_wheel = "red_wheel_right_joint";
static turtlelib::DiffDrive D; 
static turtlelib::Twist2D twist_cmd;
static turtlelib::Wheel_Angular_Velocities vels;
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

    vels = D.wheel_vel(twist_cmd);

    wheel_vel_msg.left_velocity = vels.L;
    wheel_vel_msg.right_velocity = vels.R;
}

void calc_joint_states(const nuturtlebot_msgs::SensorData &sensor_data){
/// \brief Recieves a sensor data message and uses the encoder ticks to return wheel positions.
///
/// \param sensor_data - Sensor data recieved from nuturtlebot_msgs

    double L_ticks, R_ticks;
    
    L_ticks = sensor_data.left_encoder;
    R_ticks = sensor_data.right_encoder;

    wheel_angles.L = turtlelib::normalize_angle(L_ticks * eticks_rad);
    wheel_angles.R = turtlelib::normalize_angle(R_ticks * eticks_rad);

    wheel_msg.name = {left_wheel, right_wheel};
    wheel_msg.position = {wheel_angles.L, wheel_angles.R};
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "turtle_interface");
    
    ros::NodeHandle nh("~"), pub_nh;

    if (ros::param::has("encoder_ticks_to_rad")){
        ros::param::get("encoder_ticks_to_rad", eticks_rad);
    }
    else {
        ROS_DEBUG_ONCE("encoder_ticks_to_rad not defined");
        ros::shutdown();
    }

    if (ros::param::has("rate")){
        ros::param::get("rate", rate);
    }
    else {
        ROS_DEBUG_ONCE("rate not defined");
        ros::shutdown();
    }

    ros::Rate r(rate);

    output_cmd.left_velocity = 0;
    output_cmd.right_velocity = 0;

    cmd_vel_sub = nh.subscribe("cmd_vel",100,follow_twist); 
    sensor_sub = nh.subscribe("sensor_data",100,calc_joint_states);

    wheel_pub = pub_nh.advertise<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 100);
    js_pub = pub_nh.advertise<sensor_msgs::JointState>("joint_states", 100);

    while(true){

        wheel_pub.publish(wheel_vel_msg);  
        js_pub.publish(wheel_msg);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
