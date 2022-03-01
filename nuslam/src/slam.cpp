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
arma::mat Q = arma::eye(5,5);
arma::mat I = arma::eye(5,5);
arma::mat R = arma::zeros(2,2);
arma::mat K(5,2);
arma::mat H(2,5);
arma::mat h(2,1);
arma::mat u(3,1);
arma::mat z(2,3);
arma::mat v_k(2,1);
arma::mat z_est(2,1);
arma::mat z_measured(2,1);
arma::mat x_0(5,1);
arma::mat x_prev(5,1);
arma::mat x_est(5,1);
arma::mat w_t = arma::zeros(3,1);
arma::mat S_0(5,5);
arma::mat S_prev(5,5);
arma::mat S_est(5,5);

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

void get_obj(const visualization_msgs::MarkerArray &obstacles){
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

    rate = 5;
    ros::Rate r(rate);
    
    odom_pub = pub_nh.advertise<nav_msgs::Odometry>("odom", rate);
    
    js_sub = pub_nh.subscribe("red/joint_states",10,update_odom);

    sensor_sub = pub_nh.subscribe("/fake_sensor",10,get_obj);

    old_pos.theta = w;
    old_pos.x = x;
    old_pos.y = y;

    tf2_ros::TransformBroadcaster odom_broadcaster;
    tf2_ros::TransformBroadcaster kalman_broadcaster;
    tf2_ros::TransformBroadcaster map_broadcaster;

    x_0(0,0) = w;
    x_0(1,0) = x;
    x_0(2,0) = y;
    x_0(3,0) = 0;
    x_0(4,0) = 0;
    
    x_prev(0,0) = x_0(0,0);
    x_prev(1,0) = x_0(1,0);
    x_prev(2,0) = x_0(2,0);
    x_prev(3,0) = x_0(3,0);
    x_prev(4,0) = x_0(4,0);

    x_est = arma::zeros(5,1);

    S_0 = arma::zeros(5,5);
    S_0(0,0) = 0;
    S_0(1,1) = 0;
    S_0(2,2) = 0;
    S_0(3,3) = 1000;
    S_0(4,4) = 1000;

    S_prev = arma::zeros(5,5);
    S_prev = S_0;
    S_est = arma::zeros(5,5);

    Q = arma::eye(5,5);
    Q(0,0) = .00001;
    Q(1,1) = .00001;
    Q(2,2) = .00001;
    Q(3,3) = 0;
    Q(4,4) = 0;

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

        map_tf.header.stamp = ros::Time::now();
        map_tf.header.frame_id = "world";
        map_tf.child_frame_id = "map";
        map_tf.transform.translation.x = 0;
        map_tf.transform.translation.y = 0;
        map_tf.transform.translation.z = 0;
        map_tf.transform.rotation.x = 0;
        map_tf.transform.rotation.y = 0;
        map_tf.transform.rotation.z = 0;
        map_tf.transform.rotation.w = 1;

        map_broadcaster.sendTransform(map_tf);

        kalman_tf.header.stamp = ros::Time::now();
        kalman_tf.header.frame_id = "map";
        kalman_tf.child_frame_id = "green_base_footprint";
        kalman_tf.transform.translation.x = x_0(1,0);
        kalman_tf.transform.translation.y = x_0(2,0);
        kalman_tf.transform.translation.z = 0;
        q.setRPY(0, 0, x_0(0,0));
        kalman_tf.transform.rotation.x = q.x();
        kalman_tf.transform.rotation.y = q.y();
        kalman_tf.transform.rotation.z = q.z();
        kalman_tf.transform.rotation.w = q.w();

        kalman_broadcaster.sendTransform(kalman_tf);

        old_pos = pos;

        teleporting = false;

        u(0,0) = twist.w / rate;
        u(1,0) = twist.vx / rate;
        u(2,0) = 0;
 
        ROS_WARN("u: ");
        u.print(std::cout);

        // Kalman Filter //
        if (obj_x_list.size()>0){
            for (int i=0; i<obj_x_list.size(); i++){

                if (twist.w != 0){
                    x_est(0,0) = x_prev(0,0) + u(0,0) + w_t(0,0);
                    x_est(1,0) = x_prev(1,0) - (u(1,0)/u(0,0))*sin(x_prev(0,0)) + (u(1,0)/u(0,0))*sin(x_prev(0,0)+u(0,0)) + w_t(1,0);
                    x_est(2,0) = x_prev(2,0) + (u(1,0)/u(0,0))*cos(x_prev(0,0)) - (u(1,0)/u(0,0))*cos(x_prev(0,0)+u(0,0)) + w_t(2,0);
                    x_est(3,0) = x_prev(3,0);
                    x_est(4,0) = x_prev(4,0);

                    A = arma::eye(5,5);
                    A(1,0) = -1*(u(1,0)/u(0,0)) * cos(x_est(0,0)) + (u(1,0)/u(0,0)) * cos(x_est(0,0)+ u(0,0));
                    A(2,0) = -1*(u(1,0)/u(0,0)) * sin(x_est(0,0)) + (u(1,0)/u(0,0)) * sin(x_est(0,0)+ u(0,0));

                }
                else{
                    x_est(0,0) = x_prev(0,0) + w_t(0,0);
                    x_est(1,0) = x_prev(1,0) + u(1,0)*cos(x_prev(0,0)) + w_t(1,0);
                    x_est(2,0) = x_prev(2,0) + u(1,0)*sin(x_prev(0,0)) + w_t(2,0);
                    x_est(3,0) = x_prev(3,0);
                    x_est(4,0) = x_prev(4,0);

                    A = arma::eye(5,5);
                    A(1,0) = -1*u(1,0) * sin(x_0(0,0));
                    A(2,0) = u(1,0) * cos(x_0(0,0));

                }

                ROS_WARN("x_est: ");
                x_est.print(std::cout);

                ROS_WARN("A: ");
                A.print(std::cout);

                arma::mat A_T = A.t();

                S_est = A * S_prev * A_T + Q;

                ROS_WARN("S_0: ");
                S_est.print(std::cout);

                double obj_x = obj_x_list[i];
                double obj_y = obj_y_list[i];
                z_measured(0,0) = sqrt(pow(obj_x,2)+pow(obj_y,2));
                z_measured(1,0) = atan2(obj_y,obj_x);
                
                ROS_WARN("z_measured: ");
                z_measured.print(std::cout);
                
                //Compute theoretical measurement
                x_est(3,0) = x_est(1,0) + z_measured(0,0)*cos(z_measured(1,0)+x_est(0,0));
                x_est(4,0) = x_est(2,0) + z_measured(0,0)*sin(z_measured(1,0)+x_est(0,0));
                
                h(0,0) = sqrt(pow(x_est(3,0) - x_est(1,0),2)+pow(x_est(4,0) - x_est(2,0),2));
                if ((x_est(3,0) - x_est(1,0)) != 0){
                    h(1,0) = atan2(x_est(4,0) - x_est(2,0), x_est(3,0) - x_est(1,0)) - x_est(0,0);
                }
                else {
                    h(1,0) = 0;
                }

                ROS_WARN("h: ");
                h.print(std::cout);

                v_k(0,0) = .000001;
                v_k(1,0) = .000001;
                
                z_est = h + v_k;

                ROS_WARN("z_est: ");
                z_est.print(std::cout);

                //Compute the Kalman gain
                double dx = x_est(3,0) - x_est(1,0);
                double dy = x_est(4,0) - x_est(2,0);
                double d = pow(dx,2)+pow(dy,2);

                H.set_size(2,5);

                H(0,0) = 0;  
                H(0,1) = -1*dx/sqrt(d); 
                H(0,2) = -1*dy/sqrt(d); 
                H(0,3) = dx/sqrt(d); 
                H(0,4) = dy/sqrt(d); 

                H(1,0) = -1; 
                H(1,1) = dy/d; 
                H(1,2) = -1*dx/d; 
                H(1,3) = -1*dy/d; 
                H(1,4) = dx/d; 
                
                ROS_WARN("H: ");
                H.print(std::cout);

                arma::mat H_T = H.t();

                R.set_size(2,2);
                R(0,0) = v_k(0,0);
                R(0,1) = 0;
                R(1,0) = 0;
                R(1,1) = v_k(1,0);

                K = S_est * H_T * (H*S_est*H_T+R).i();

                ROS_WARN("K: ");
                K.print(std::cout);

                //Posterior State Update
                arma::mat dz = z_measured - z_est;
                dz(1,0) = turtlelib::normalize_angle(dz(1,0));
                ROS_WARN("dz: ");
                dz.print(std::cout);
                
                x_0 = x_est + K * dz;
                x_0.print(std::cout);
                
                //Posterior Covariance
                S_0 = (I - K*H) * S_est; 

                ROS_WARN("S_0: ");
                S_0.print(std::cout);

                x_prev(0,0) = x_0(0,0);
                x_prev(1,0) = x_0(1,0);
                x_prev(2,0) = x_0(2,0);
                x_prev(3,0) = x_0(3,0);
                x_prev(4,0) = x_0(4,0);

                S_prev(0,0) = S_0(0,0);
                S_prev(1,0) = S_0(1,0);
                S_prev(2,0) = S_0(2,0);
                S_prev(3,0) = S_0(3,0);
                S_prev(4,0) = S_0(4,0);
            }
        }
        
        ROS_WARN("--------------------");
        /////////////////

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}