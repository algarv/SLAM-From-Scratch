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

static double vel = 0.0, rad = 1.0;
bool stopped;

bool control(nuturtle_control::control::Request &params, nuturtle_control::control::Response &response){

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

    return true;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "circle");
    ros::NodeHandle nh("~"), pub_nh;

    int rate;
    nh.getParam("rate", rate);
    ros::Rate r(rate);

    ros::ServiceServer control_service = nh.advertiseService("Control",control);
    ros::ServiceServer reverse_service = nh.advertiseService("Reverse",reverse);
    ros::ServiceServer stop_service = nh.advertiseService("Stop",stop);

    ros::Publisher cmd_vel_pub = pub_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    stopped = false;

    geometry_msgs::Twist twist_cmd;
    
    while(stopped == false){
        if (rad == 0){
            twist_cmd.linear.x = 0;
            twist_cmd.angular.z = vel;
        }
        twist_cmd.linear.x = vel;
        twist_cmd.angular.z = vel/rad;
        cmd_vel_pub.publish(twist_cmd);
        ros::spinOnce();
        r.sleep();
    }

    twist_cmd.linear.x = 0;
    twist_cmd.angular.z = 0;
    cmd_vel_pub.publish(twist_cmd);
    ros::spinOnce();
    r.sleep();

    return 0;
}