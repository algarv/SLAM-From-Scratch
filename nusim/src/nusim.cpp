#include <iostream>
#include <string>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt64.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include "nusim/teleport.h"
#include "nusim/add_obstacle.h"


static int rate;
static std::string left_wheel = "red_wheel_left_joint";
static std::string right_wheel = "red_wheel_right_joint";
static std_msgs::UInt64 ts;
static sensor_msgs::JointState wheels;
static geometry_msgs::TransformStamped tfStamped;
static ros::Publisher obj_pub, js_pub, ts_pub;
static ros::ServiceServer rs_service, tp_service;

bool restart(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){

    ts.data = 0;

    return true;

}

bool teleport(nusim::teleport::Request &pos, nusim::teleport::Response &response){

        // wheels.position = {0, 0};
        // js_pub.publish(wheels);
        
        tf2_ros::TransformBroadcaster broadcaster;

        tfStamped.header.stamp = ros::Time();
        tfStamped.header.frame_id = "world";
        tfStamped.child_frame_id = "red_base_footprint";
        tfStamped.transform.translation.x = pos.x;
        tfStamped.transform.translation.y = pos.y;
        tfStamped.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, pos.w);
        tfStamped.transform.rotation.x = q.x();
        tfStamped.transform.rotation.y = q.y();
        tfStamped.transform.rotation.z = q.z();
        tfStamped.transform.rotation.w = q.w();

        broadcaster.sendTransform(tfStamped);   

    return true;
}

void obstacle(nusim::add_obstacle::Request &pos, nusim::add_obstacle::Response &response){

    visualization_msgs::Marker obstacle;

    obstacle.header.frame_id = "world";
    obstacle.header.stamp = ros::Time();
    obstacle.type = visualization_msgs::Marker::CYLINDER;
    obstacle.action = visualization_msgs::Marker::ADD;
    obstacle.pose.position.x = pos.x;
    obstacle.pose.position.y = pos.y;
    obstacle.pose.position.z = 0;
    obstacle.pose.orientation.x = 0.0;
    obstacle.pose.orientation.y = 0.0;
    obstacle.pose.orientation.z = 0.0;
    obstacle.pose.orientation.w = 1.0;
    obstacle.scale.x = pos.d;
    obstacle.scale.y = pos.d;
    obstacle.scale.z = .25;
    obstacle.color.a = 1.0;
    obstacle.color.r = 1.0;
    obstacle.color.g = 0.0;
    obstacle.color.b = 0.0;

    obj_pub.publish(obstacle);

}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "nusim");
    
    ros::NodeHandle nh("~"), pub_nh;
    
    nh.getParam("rate", rate);
    ros::Rate r(rate);

    ts.data = 0;

    static tf2_ros::TransformBroadcaster broadcaster;

    ts_pub = nh.advertise<std_msgs::UInt64>("timestep",rate);
    js_pub = pub_nh.advertise<sensor_msgs::JointState>("joint_states", rate);
    obj_pub = nh.advertise<visualization_msgs::Marker>( "obstacle", rate);

    rs_service = nh.advertiseService("Restart", restart);
    tp_service = nh.advertiseService("Teleport", teleport);

    float x, y, theta;
    nh.getParam("x0", x);
    nh.getParam("y0", y);
    nh.getParam("theta0", theta);

    while(ros::ok()) {
        ts.data += 1;

        // wheels.header = ros::Time::now();
        wheels.name = {left_wheel, right_wheel};
        wheels.position = {0, 0};

        js_pub.publish(wheels);
        ts_pub.publish(ts);

        tfStamped.header.stamp = ros::Time();
        tfStamped.header.frame_id = "world";
        tfStamped.child_frame_id = "red_base_footprint";
        tfStamped.transform.translation.x = x;
        tfStamped.transform.translation.y = y;
        tfStamped.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        tfStamped.transform.rotation.x = q.x();
        tfStamped.transform.rotation.y = q.y();
        tfStamped.transform.rotation.z = q.z();
        tfStamped.transform.rotation.w = q.w();

        broadcaster.sendTransform(tfStamped);
    }

    // ros::ServiceServer service = nh.advertiseService("Add_Obstacle",obstacle)
    // sensor_msgs::JointState joint_states;

    // ros::Publisher obs_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    return 0;
}