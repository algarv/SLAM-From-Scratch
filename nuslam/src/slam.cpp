/// \file odometry.cpp
/// \brief Publishes odometry messages for the turtlebot to follow.
///
/// PARAMETERS:
///     rate: ros rate
///     body_id: The id of the body frame
///     odom_id: The id of the odom frame 
///     wheel_left: The name of the left wheel joint
///     wheel_right: The name of the right wheel joint
///     encoder_ticks_to_rad (diff_params.yaml): The number of radians per encoder tick
///
/// PUBLISHES:
///     odom (nav_msgs/Odometry): publishes the velocity of each wheel
/// 
/// SUBSCRIBERS:
///     joint_states (sensor_msgs/JointState): Receives the wheel joint angles

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
#include "nuturtle_control/set_pose.h"
#include <visualization_msgs/MarkerArray.h>
#include "kalman/kalman.hpp"
#include <armadillo>

int rate;
static bool teleporting = false;
static double x = 0, y = 0, w = 0;
std::string body_id, odom_id, wheel_left, wheel_right;
turtlelib::Wheel_Angle wheel_angles = {.L = 0, .R = 0}, old_wheel_angles = {.L = 0, .R = 0};
turtlelib::Wheel_Angular_Velocities wheel_vels;
turtlelib::DiffDrive D;
turtlelib::Twist2D twist;
turtlelib::q pos, old_pos;
static geometry_msgs::TransformStamped odom_tf, kalman_tf, map_tf;
ros::Subscriber js_sub, sensor_sub;
ros::Publisher odom_pub;
static ros::ServiceServer pose_service;
nav_msgs::Odometry odom_msg;
std::vector<double> obj_x_list, obj_y_list;

arma::mat A = arma::eye(5,5);
arma::mat A_T = A.t();
arma::mat B = arma::eye(3,3);
arma::mat Q = arma::eye(5,5);
arma::mat I = arma::eye(5,5);
arma::mat R = arma::zeros(2,1);
arma::mat K(5,2);
arma::mat H(2,5);
arma::mat h(2,1);
arma::mat u(3,1);
arma::mat z(2,3);
arma::mat v_k(2,1);
arma::mat z_k(2,1);
arma::mat z_0(2,1);
arma::mat x_0(5,1);
turtlelib::q x_pred; 
arma::mat S_0(5,5);
arma::mat S_k(5,5);

/// \brief Receives a wheel joint states and translates into a twist for the odometry message
///
/// \param wheels - wheel joint states
void update_odom(const sensor_msgs::JointState &wheels){

    
    wheel_vels.L = wheels.velocity[0]; 
    wheel_vels.R = wheels.velocity[1];

    wheel_angles.L = wheels.position[0] + (wheel_vels.L /rate);
    wheel_angles.R = wheels.position[1] + (wheel_vels.R/rate);

    // twist = D.get_twist(wheel_angles, old_wheel_angles);

    twist = D.get_twist(wheel_vels);

    // odom_msg.header.frame_id = "odom";
    odom_msg.twist.twist.linear.x = twist.vx;
    odom_msg.twist.twist.linear.y = twist.vy;
    odom_msg.twist.twist.angular.z = twist.w;

    old_wheel_angles = {.L = wheels.position[0], .R = wheels.position[1]};
}

void get_h(const visualization_msgs::MarkerArray &obstacles){
    std::vector<double> temp_x_list;
    std::vector<double> temp_y_list;

    for (auto obj: obstacles.markers) {
        temp_x_list.push_back(obj.pose.position.x);
        temp_y_list.push_back(obj.pose.position.y);
    }

    obj_x_list = temp_x_list;
    obj_y_list = temp_y_list;
}


int main(int argc, char *argv[]){

    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~"), pub_nh;

    // if (ros::param::has("body_id")){
        ros::param::get("body_id", body_id);
    // }
    // else {
    //     ROS_DEBUG_ONCE("body_id not defined");
    //     ros::shutdown();
    // }

    // if (ros::param::has("wheel_left")){
        ros::param::get("wheel_left", wheel_left);
    // }
    // else {
    //     ROS_DEBUG_ONCE("wheel_left not defined");
    //     ros::shutdown();
    // }

    // if (ros::param::has("wheel_right")){
        ros::param::get("wheel_right", wheel_right);
    // }
    // else {
    //     ROS_DEBUG_ONCE("wheel_right not defined");
    //     ros::shutdown();
    // }

    // if (ros::param::has("rate")){
        ros::param::get("rate", rate);
    // }
    // else {
    //     ROS_DEBUG_ONCE("rate not defined");
    //     ros::shutdown();
    // }

    nh.getParam("x0", x);
    nh.getParam("y0", y);
    nh.getParam("theta0", w);

    nh.param<std::string>("odom_id",odom_id,"odom");

    rate = 500;
    ros::Rate r(rate);
    
    odom_pub = pub_nh.advertise<nav_msgs::Odometry>("odom", rate);
    
    js_sub = pub_nh.subscribe("red/joint_states",10,update_odom);

    sensor_sub = pub_nh.subscribe("/fake_sensor",10,get_h);

    old_pos.theta = w;
    old_pos.x = x;
    old_pos.y = y;

    tf2_ros::TransformBroadcaster odom_broadcaster;
    tf2_ros::TransformBroadcaster kalman_broadcaster;
    tf2_ros::TransformBroadcaster map_broadcaster;

    x_0(0,0) = x;
    x_0(1,0) = y;
    x_0(2,0) = w;

    S_0 = arma::zeros(5,5);
    S_0(3,3) = 1000;
    S_0(4,4) = 1000;

    Q = arma::eye(5,5);

    //kalman::filter EKF_config(A,B,Q);
    
    while(ros::ok()){

        if (teleporting == false){
            pos = D.get_q(wheel_angles, old_wheel_angles, old_pos);
            // pos = D.get_q(twist,old_pos);
        }

        odom_tf.header.stamp = ros::Time::now();
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "blue_base_footprint";
        odom_tf.transform.translation.x = pos.x;
        odom_tf.transform.translation.y = pos.y;
        odom_tf.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, pos.theta);
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        odom_broadcaster.sendTransform(odom_tf);

        odom_msg.pose.pose.position.x = pos.x;
        odom_msg.pose.pose.position.y = pos.y;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_pub.publish(odom_msg);        

        old_pos = pos;

        teleporting = false;

        u(0,0) = twist.vx;
        u(1,0) = twist.vy;
        u(2,0) = twist.w;

        // Kalman Filter //
        ROS_WARN("A: ");
        A.print(std::cout);
        ROS_WARN("B: ");
        B.print(std::cout);
        ROS_WARN("x_0: ");
        x_0.print(std::cout);
        ROS_WARN("S_0: ");
        S_0.print(std::cout);
        ROS_WARN("u: ");
        u.print(std::cout);

        //EKF_config.predict(x_0, S_0, u);
        //Prediction
        turtlelib::q x_0_pos {.theta = x_0(2,0), .x = x_0(0,0), .y = x_0(1,0)};

        x_pred = D.get_q(twist,x_0_pos);
        x_0(0,0) = x_pred.x;
        x_0(1,0) = x_pred.y;
        x_0(2,0) = x_pred.theta; 

        ROS_WARN("x_0: ");
        x_0.print(std::cout);

        S_0 = A * S_0 * A_T + Q;

        ROS_WARN("S_0: ");
        S_0.print(std::cout);

        if (obj_x_list.size()>0){
            for (int i=0; i<=obj_x_list.size(); i++){
                double x = obj_x_list[i];
                double y = obj_y_list[i];
                z_0(0,0) = sqrt(pow(x,2)+pow(y,2));
                z_0(1,0) = atan2(y,x);
                
                ROS_WARN("z_0: ");
                z_0.print(std::cout);

                //Compute theoretical measurement
                h(0,0) = x_0(1,0);
                h(1,0) = x_0(2,0);
                
                v_k(0,0) = 1;
                v_k(1,0) = 1;
                
                z_k = h + v_k;

                ROS_WARN("z_k: ");
                z_k.print(std::cout);

                //Compute the Kalman gain
                double r = z_k(0,0);
                double phi = z_k(1,0);
                double dx = r * cos(phi);
                double dy = r * sin(phi);
                double d = sqrt(pow(dx,2)+pow(dy,2));

                H.set_size(2,5);

                H(0,0) = 0;  
                H(0,1) = -dx/d; 
                H(0,2) = -dy/d; 
                H(0,3) = dx/d; 
                H(0,4) = dy/d; 

                H(1,0) = -1; 
                H(1,1) = dy/pow(d,2); 
                H(1,2) = -dx/pow(d,2); 
                H(1,3) = -dy/pow(d,2); 
                H(1,4) = dx/pow(d,2); 
                
                ROS_WARN("H: ");
                H.print(std::cout);

                arma::mat H_T = H.t();

                R.set_size(2,2);
                R(0,0) = v_k(0,0);
                R(0,1) = 0;
                R(1,0) = 0;
                R(1,1) = v_k(1,0);

                ROS_WARN("R: ");
                R.print(std::cout);

                K = S_k * H_T * (H*S_k*H_T+R).i();

                ROS_WARN("K: ");
                K.print(std::cout);

                //Posterior State Update
                x_0 = x_0 + K * (z_0 - z_k);

                //Posterior Covariance
                S_0 = (I - K*H) * S_0; 
            }
        }

        ROS_WARN("--------------------");
        ///////////////////

        map_tf.header.stamp = ros::Time::now();
        map_tf.header.frame_id = "world";
        map_tf.child_frame_id = "map";
        map_tf.transform.translation.x = x_0(3,0);
        map_tf.transform.translation.y = x_0(4,0);
        map_tf.transform.translation.z = 0;
        map_tf.transform.rotation.x = 0;
        map_tf.transform.rotation.y = 0;
        map_tf.transform.rotation.z = 0;
        map_tf.transform.rotation.w = 1;

        map_broadcaster.sendTransform(map_tf);

        kalman_tf.header.stamp = ros::Time::now();
        kalman_tf.header.frame_id = "map";
        kalman_tf.child_frame_id = "green_base_footprint";
        kalman_tf.transform.translation.x = x_0(0,0);
        kalman_tf.transform.translation.y = x_0(1,0);
        kalman_tf.transform.translation.z = 0;
        q.setRPY(0, 0, x_0(2,0));
        kalman_tf.transform.rotation.x = q.x();
        kalman_tf.transform.rotation.y = q.y();
        kalman_tf.transform.rotation.z = q.z();
        kalman_tf.transform.rotation.w = q.w();

        kalman_broadcaster.sendTransform(kalman_tf);

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}



/*
        if (obj_x_list.size()>0){
            for (int i=0; i<=obj_x_list.size(); i++){
                double x = obj_x_list[i];
                double y = obj_y_list[i];

                h(0,0) = sqrt(pow(x,2)+pow(y,2));
                h(1,0) = atan2(y,x);

                v_k(0,0) = 0;
                v_k(1,0) = 0;

                h.print(std::cout);
                v_k.print(std::cout);

                x_0 = EKF_config.correct_x(h, v_k);
                ROS_WARN("Corrected X");
            }
            S_0 = EKF_config.correct_S();
        }
*/