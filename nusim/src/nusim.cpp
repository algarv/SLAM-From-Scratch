/// \file nusim.cpp
/// \brief The nusim node sets up the turtlebot in rviz with obstacles and services for restarting and teleporting.
///
/// PARAMETERS:
///     rate (rate.yaml): ros rate
///     x0 (initial.yaml): x coordinate of the initial turtle position
///     y0 (initial.yaml): y coordinate of the initial turtle position
///     theta0 (initial.yaml): orientation of the initial turtle position
///     objx (initial.yaml): list of x coorindates for the obstacles
///     objy (initial.yaml): list of y coordinates for the obstacles
///     objd (initial.yaml): list of obstacle diamters
/// PUBLISHES:
///     /nusim_node/visualization_marker_array (visualization_msgs/MarkerArray): publishes a marker array to spawn obstacles in rviz
///     nusim_node/timestep (std_msgs/UInt64): counter tracking runs of ros::Spin
///     /joint_states (sensor_msgs/JointState): publishes wheel joint states 
/// SERVICES:
///     nusim_node/Restart (Empty): Sends the turtlebot back to the origin of the world frame
///     nusim_node/Teleport (teleport.srv): Teleports the turtle bot to the specfied x, y, and theta position

#include <iostream>
#include <string>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt64.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include "nusim/teleport.h"
#include "nusim/add_obstacle.h"

static int rate;
static float x = 0, y = 0, w = 0;
static std_msgs::UInt64 ts;
static sensor_msgs::JointState wheels;
static std::string left_wheel = "red_wheel_left_joint";
static std::string right_wheel = "red_wheel_right_joint";
static geometry_msgs::TransformStamped tfStamped;
static ros::Publisher obj_pub, js_pub, ts_pub;
static ros::Subscriber wheel_sub;
static ros::ServiceServer rs_service, tp_service;
std::vector<double> obj_x_list, obj_y_list, obj_d_list;
static visualization_msgs::MarkerArray obstacle, obj_array;

bool restart(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){
/// \brief Send the turtle bot back to the origin of the world frame and restart the timestep counter.
///
/// \param request - Empty::Request
/// \param response - Empty::Reponse
/// \returns true
    ts.data = 0;

    x = 0;
    y = 0;
    w = 0;

    return true;
}

bool teleport(nusim::teleport::Request &pos, nusim::teleport::Response &response){
/// \brief Teleports the turtle to the specified position and rotation.
///
/// \param pos - position input from parameter
/// \param response - Empty::Reponse
/// \returns true

    x = pos.x;
    y = pos.y;
    w = pos.w;
 
    return true;
}

visualization_msgs::MarkerArray add_obstacles(std::vector<double> obj_x_list, std::vector<double> obj_y_list, std::vector<double> obj_d_list){
/// \brief Spawns obstacles defined in the paramter.
///
/// \param obj_x_list - std::vector<double> list of x positions
/// \param obj_y_list - std::vector<double> list of y positions
/// \param obj_d_list - std::vector<double> list of diameters
/// \returns obstacle - visualization_msgs::MarkerArray message with obstacle information to publish

    int id = 0;
    obstacle.markers.resize(obj_x_list.size());
    for (int i = 0; i<obj_x_list.size(); i+=1) {
        
        obstacle.markers[i].header.frame_id = "world";
        obstacle.markers[i].ns = "nusim_node";
        obstacle.markers[i].header.stamp = ros::Time::now();
        obstacle.markers[i].type = visualization_msgs::Marker::CYLINDER;
        obstacle.markers[i].action = visualization_msgs::Marker::ADD;
        obstacle.markers[i].id = id;
        obstacle.markers[i].pose.position.x = obj_x_list[i];
        obstacle.markers[i].pose.position.y = obj_y_list[i];
        obstacle.markers[i].pose.position.z = 0;
        obstacle.markers[i].pose.orientation.x = 0.0;
        obstacle.markers[i].pose.orientation.y = 0.0;
        obstacle.markers[i].pose.orientation.z = 0.0;
        obstacle.markers[i].pose.orientation.w = 1.0;
        obstacle.markers[i].scale.x = obj_d_list[i];
        obstacle.markers[i].scale.y = obj_d_list[i];
        obstacle.markers[i].scale.z = .25;
        obstacle.markers[i].color.a = 1.0;
        obstacle.markers[i].color.r = 1.0;
        obstacle.markers[i].color.g = 0.0;
        obstacle.markers[i].color.b = 0.0;
        // obstacle.markers[i].lifetime = ros::Duration();
        id += 1;
    }
    
    return obstacle;
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "nusim");
    
    ros::NodeHandle nh("~"), pub_nh;
    
    nh.getParam("rate", rate);
    ros::Rate r(rate);

    ts.data = 0;

    static tf2_ros::TransformBroadcaster broadcaster;

    ts_pub = nh.advertise<std_msgs::UInt64>("timestep",100);
    js_pub = pub_nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    obj_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);

    wheel_sub = nh.subscribe("red/wheel_cmd", 1, callback);

    rs_service = nh.advertiseService("Restart", restart);
    tp_service = nh.advertiseService("Teleport", teleport);

    nh.getParam("x0", x);
    nh.getParam("y0", y);
    nh.getParam("theta0", w);

    while(ros::ok()) {

        wheels.name = {left_wheel, right_wheel};
        wheels.position = {0, 0};
        js_pub.publish(wheels);
        
        ts.data += 1;
        ts_pub.publish(ts);

        tf2_ros::TransformBroadcaster broadcaster;

        tfStamped.header.stamp = ros::Time::now();
        tfStamped.header.frame_id = "world";
        tfStamped.child_frame_id = "red_base_footprint";
        tfStamped.transform.translation.x = x;
        tfStamped.transform.translation.y = y;
        tfStamped.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, w);
        tfStamped.transform.rotation.x = q.x();
        tfStamped.transform.rotation.y = q.y();
        tfStamped.transform.rotation.z = q.z();
        tfStamped.transform.rotation.w = q.w();

        broadcaster.sendTransform(tfStamped);
        
        nh.getParam("obj_x",obj_x_list);
        nh.getParam("obj_y",obj_y_list);
        nh.getParam("obj_d",obj_d_list);

        obj_array = add_obstacles(obj_x_list, obj_y_list,obj_d_list);
        obj_pub.publish(obj_array);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}