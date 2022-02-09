#include <ros/ros.h>
#include <catch_ros/catch.hpp>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// // ros::Subscriber cmd_vel_sub = pub_nh.subscribe("cmd_vel",10,follow_twist); 
// // ros::Subscriber sensor_sub = pub_nh.subscribe("sensor_data",10,calc_joint_states);

static turtlelib::Wheel_Angle wheel_angles;
static turtlelib::DiffDrive D; 
double L_cmd = 0;
double R_cmd = 0;
double wheel_rad = 0.033;
double chassis_rad = .08;
double eticks_rad = 0.00153398078789;



void update_wheel_position(const nuturtlebot_msgs::WheelCommands::ConstPtr &wheel_cmd){
    // L_cmd = wheel_cmd->left_velocity;
    // R_cmd = wheel_cmd->right_velocity;
}

void update_odom(const sensor_msgs::JointState &wheels){

    // wheel_angles.L = wheels.position[0];
    // wheel_angles.R = wheels.position[1];
    // wheel_vels.L = (wheel_angles.L - old_wheel_angles.L)*rate;//wheels.velocity[0];
    // wheel_vels.R = (wheel_angles.R - old_wheel_angles.R)*rate;//wheels.velocity[1];
    // // twist = D.get_twist(wheel_angles, old_wheel_angles);
    // twist = D.get_twist(wheel_vels);

    // // odom_msg.header.frame_id = "odom";
    // odom_msg.twist.twist.linear.x = twist.vx;
    // odom_msg.twist.twist.linear.y = twist.vy;
    // odom_msg.twist.twist.angular.z = twist.w;
}

void follow_twist(const geometry_msgs::Twist &wheel_cmd){

    turtlelib::Twist2D twist_cmd;

    twist_cmd.vx = wheel_cmd.linear.x;
    twist_cmd.vy = wheel_cmd.linear.y;
    twist_cmd.w = wheel_cmd.angular.z;

    turtlelib::Wheel_Angular_Velocities vel_cmd = D.wheel_vel(twist_cmd);

    L_cmd = vel_cmd.L;
    R_cmd = vel_cmd.R;
}

void calc_joint_states(const nuturtlebot_msgs::SensorData &sensor_data){
    
    double L_ticks = sensor_data.left_encoder;
    double R_ticks = sensor_data.right_encoder;

    wheel_angles.L = (L_ticks * eticks_rad);
    wheel_angles.R = (R_ticks * eticks_rad);
}


TEST_CASE("NU Turtle Control", "[Tests]") { // Anna Garverick
    
    ros::NodeHandle pub_nh;

    double rate = .25;
    ros::Rate r(rate);

    ros::Publisher wheel_pub = pub_nh.advertise<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 10);
    ros::Publisher js_pub = pub_nh.advertise<sensor_msgs::JointState>("red/joint_states", 10);
    ros::Publisher cmd_vel_pub = pub_nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Publisher sensor_pub = pub_nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data",100);
    ros::Publisher odom_pub = pub_nh.advertise<nav_msgs::Odometry>("odom", 100);
    
    ros::Subscriber js_sub = pub_nh.subscribe("red/joint_states",10,update_odom);
    ros::Subscriber wheel_sub = pub_nh.subscribe("red/wheel_cmd", 10, update_wheel_position);
    ros::Subscriber cmd_vel_sub = pub_nh.subscribe("cmd_vel",10,follow_twist); 
    ros::Subscriber sensor_sub = pub_nh.subscribe("sensor_data",10,calc_joint_states);

    geometry_msgs::Twist twist_cmd;
    
    SECTION("Linear Velocity") {
        twist_cmd.linear.x = 1.0;
        
        cmd_vel_pub.publish(twist_cmd);
        r.sleep();
        ros::spinOnce(); 

        REQUIRE(L_cmd == Approx(twist_cmd.linear.x/wheel_rad).margin(.001));
        REQUIRE(R_cmd == Approx(twist_cmd.linear.x/wheel_rad).margin(.001));
    }

    SECTION("Pure Rotation") {
        twist_cmd.angular.z = 1.0;
        
        cmd_vel_pub.publish(twist_cmd);
        r.sleep();
        ros::spinOnce(); 

        REQUIRE(L_cmd == Approx(-1 * twist_cmd.angular.z * chassis_rad/wheel_rad).margin(.001));
        REQUIRE(R_cmd == Approx(twist_cmd.angular.z * chassis_rad/wheel_rad).margin(.001));
    }

    SECTION("Encoder to Joint States") {
        nuturtlebot_msgs::SensorData sensor_data;
        sensor_data.left_encoder = 4096;
        sensor_data.right_encoder = 4096;

        sensor_pub.publish(sensor_data);
        r.sleep();
        ros::spinOnce(); 

        REQUIRE(wheel_angles.L == Approx(0.0).margin(.001));
        REQUIRE(wheel_angles.R == Approx(0.0).margin(.001));
    }

}

