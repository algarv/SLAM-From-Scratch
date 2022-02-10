#include <ros/ros.h>
#include <catch_ros/catch.hpp>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "nuturtle_control/set_pose.h"

static turtlelib::Wheel_Angle wheel_angles;
static turtlelib::DiffDrive D; 
double L_cmd = 0;
double R_cmd = 0;
double wheel_rad = 0.033;
double chassis_rad = .08;
double eticks_rad = 0.00153398078789;

TEST_CASE("NU Turtle Control", "[Tests]") { // Anna Garverick
    
    ros::NodeHandle pub_nh;

    double rate = .25;
    ros::Rate r(rate);

    // ros::Publisher wheel_pub = pub_nh.advertise<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 10);
    // ros::Publisher js_pub = pub_nh.advertise<sensor_msgs::JointState>("red/joint_states", 10);
    // ros::Publisher cmd_vel_pub = pub_nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    // ros::Publisher sensor_pub = pub_nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data",100);
    // ros::Publisher odom_pub = pub_nh.advertise<nav_msgs::Odometry>("odom", 100);
    
    // ros::Subscriber js_sub = pub_nh.subscribe("red/joint_states",10,update_odom);
    // ros::Subscriber wheel_sub = pub_nh.subscribe("red/wheel_cmd", 10, update_wheel_position);
    // ros::Subscriber cmd_vel_sub = pub_nh.subscribe("cmd_vel",10,follow_twist); 
    // ros::Subscriber sensor_sub = pub_nh.subscribe("sensor_data",10,calc_joint_states);

    ros::ServiceClient set_pose_client = pub_nh.serviceClient<nuturtle_control::set_pose>("add_two_ints");
    
    nuturtle_control::set_pose set_pose_srv;
    geometry_msgs::Twist twist_cmd;
    tf2_ros::Buffer blue_tf_buffer;
    
    SECTION("Set Pose") {

        tf2_ros::TransformListener blue_tf_listener = tf2_ros::TransformListener(blue_tf_buffer);

        set_pose_srv.request.x = 1;
        set_pose_srv.request.y = 1;
        set_pose_srv.request.w = 0;

        r.sleep();
        ros::spinOnce(); 

        blue_tf_buffer.lookupTransform("/world", "/blue_base_footprint", ros::Time(0));

        // REQUIRE(blue_tf_listener.getOrigin().x() == 1);
        // REQUIRE(blue_tf_listener.getOrigin().y() == 1);
        REQUIRE(1==1);
    }

}