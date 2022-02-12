/// \file circle.cpp
/// \brief Advertises a service to make the turtlebot spin in a specified arc.
///
/// PUBLISHES:
///     cmd_vel (geometry_msgs/Twist): publishes the twist corresponding to the specified arc
/// 
/// SERVICES:
///     nuturtle_control/Control (control.srv): Sends the turtlebot back to the origin of the world frame
///     nuturtle_control/Reverse (Empty): Reverses the direction of the turtlebots trajectory
///     nuturtle_control/Stop (Empty): Stops the turtlebot

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
#include "nuturtle_control/control.h"
#include <std_srvs/Empty.h>

static double vel = 0.0, rad = 0.0;
bool stopped = false;
static int flag = 0;

bool control(nuturtle_control::control::Request &params, nuturtle_control::control::Response &response){

    stopped = false;
    vel = params.velocity;
    rad = params.radius;

    return true;

}

bool reverse(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){
    
    vel = -1 * vel;

    return true;
}

bool stop(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){
    stopped = true;
    flag = 0;
    return true;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "circle");
    ros::NodeHandle nh("~"), pub_nh;

    int rate = 500;
    pub_nh.param("rate", rate, 500);
    ros::Rate r(rate);

    ros::ServiceServer control_service = nh.advertiseService("Control",control);
    ros::ServiceServer reverse_service = nh.advertiseService("Reverse",reverse);
    ros::ServiceServer stop_service = nh.advertiseService("Stop",stop);

    ros::Publisher cmd_vel_pub = pub_nh.advertise<geometry_msgs::Twist>("cmd_vel", rate);

    geometry_msgs::Twist twist_cmd;
    
    while(ros::ok()) { 
        if(stopped == false){
            twist_cmd.linear.x = vel * rad;
            twist_cmd.angular.z = vel;

            cmd_vel_pub.publish(twist_cmd);
        }

        if(stopped == true && flag == 0){
            twist_cmd.linear.x = 0;
            twist_cmd.angular.z = 0;
            
            cmd_vel_pub.publish(twist_cmd);  
            flag = 1;
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}