#include <ros/ros.h>
#include <catch_ros/catch.hpp>
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

static turtlelib::Wheel_Angle wheel_angles;
double L_cmd = 0;
double R_cmd = 0;

int rate = 500;
ros::Rate r(rate);

ros::NodeHandle pub_nh;

void update_wheel_position(const nuturtlebot_msgs::WheelCommands::ConstPtr &wheel_cmd){
    L_cmd = wheel_cmd->left_velocity;
    R_cmd = wheel_cmd->right_velocity;
}

ros::Publisher cmd_vel_pub = pub_nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
// ros::Publisher sensor_pub = pub_nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data",100);

ros::Subscriber wheel_sub = pub_nh.subscribe("red/wheel_cmd", 10, update_wheel_position);
// ros::Subscriber cmd_vel_sub = pub_nh.subscribe("cmd_vel",10,follow_twist); 
// ros::Subscriber sensor_sub = pub_nh.subscribe("sensor_data",10,calc_joint_states);

TEST_CASE("constructor_all", "[transform]") { // Anna Garverick

    geometry_msgs::Twist twist_cmd;
    
    twist_cmd.linear.x = 1;
    
    cmd_vel_pub.publish(twist_cmd);

    r.sleep();
    ros::spinOnce();

    REQUIRE(L_cmd == 1/0.033);
    REQUIRE(R_cmd == 1/0.033);
}

