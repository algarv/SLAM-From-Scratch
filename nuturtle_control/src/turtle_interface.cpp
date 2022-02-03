#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>

static int rate;
static ros::Subscriber cmd_vel_sub, sensor_sub;
static ros::Publisher wheel_pub, js_pub;
static double eticks_rad;
static std::string left_wheel = "red_wheel_left_joint", right_wheel = "red_wheel_right_joint";
static turtlelib::DiffDrive D; 
static turtlelib::Twist2D twist_cmd;
static turtlelib::Wheel_Angular_Velocities vels;
static turtlelib::Wheel_Angle d_angles;
static sensor_msgs::JointState wheels;

nuturtlebot_msgs::WheelCommands output_cmd;

void follow_twist(const geometry_msgs::Twist &wheel_cmd)
{
    twist_cmd.vx = wheel_cmd.linear.x;
    twist_cmd.vy = wheel_cmd.linear.y;
    twist_cmd.w = wheel_cmd.angular.z;

    vels = D.wheel_vel(twist_cmd);

    output_cmd.left_velocity = vels.L;
    output_cmd.right_velocity = vels.R;
}

void calc_joint_states(const nuturtlebot_msgs::SensorData &sensor_data){
    double L_ticks, R_ticks;
    
    L_ticks = sensor_data.left_encoder;
    R_ticks = sensor_data.right_encoder;

    d_angles.L = turtlelib::normalize_angle(L_ticks * eticks_rad);
    d_angles.R = turtlelib::normalize_angle(R_ticks * eticks_rad);

    wheels.name = {left_wheel, right_wheel};
    wheels.position = {0, 0};
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "turtle_interface");
    
    ros::NodeHandle nh("~"), pub_nh;

    nh.getParam("encoder_ticks_to_rad", eticks_rad);
    nh.getParam("rate", rate);
    ros::Rate r(rate);

    output_cmd.left_velocity = 0;
    output_cmd.right_velocity = 0;

    cmd_vel_sub = nh.subscribe("cmd_vel",100,follow_twist); 
    sensor_sub = nh.subscribe("sensor_data",100,calc_joint_states);

    wheel_pub = pub_nh.advertise<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 100);
    js_pub = pub_nh.advertise<sensor_msgs::JointState>("joint_states", 100);

    while(true){

        wheel_pub.publish(output_cmd);  
        js_pub.publish(wheels);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
