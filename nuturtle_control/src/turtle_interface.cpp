#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"

static int rate;
static ros::Subscriber cmd_vel_sub;
static ros::Publisher wheel_pub;
static turtlelib::DiffDrive D; 
static turtlelib::Twist2D twist_cmd;
static turtlelib::Wheel_Angular_Velocities vels;
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

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "turtle_interface");
    
    ros::NodeHandle nh("~"), pub_nh;

    nh.getParam("rate", rate);
    ros::Rate r(rate);

    output_cmd.left_velocity = 0;
    output_cmd.right_velocity = 0;

    wheel_pub = pub_nh.advertise<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 100);
    cmd_vel_sub = nh.subscribe("cmd_vel",100,follow_twist);    

    while(true){
        wheel_pub.publish(output_cmd);  
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
