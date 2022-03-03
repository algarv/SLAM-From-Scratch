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
visualization_msgs::MarkerArray found_obstacles;

arma::mat A = arma::eye(9,9);
arma::mat I = arma::eye(9,9);
arma::mat K(9,2);
arma::mat H(2,9);
arma::mat h(2,1);
arma::mat u(3,1);
arma::mat z(2,3);
arma::mat v_k(2,1);
arma::mat z_est(2,1);
arma::mat z_measured(2,1);
arma::mat dz(2,1);
arma::mat x_0(9,1);
arma::mat x_prev(9,1);
arma::mat x_est(9,1);
arma::mat S_0(9,9);
arma::mat S_prev(9,9);
arma::mat S_est(9,9);
int run_count = 0;
double n = 3;

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

    found_obstacles = obstacles;

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

    //nh.getParam("rate", rate);
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

    x_0 = arma::zeros(9,1);
    x_0(0,0) = w;
    x_0(1,0) = x;
    x_0(2,0) = y;
    
    x_prev = x_0;

    x_est = x_0;

    S_0 = arma::zeros(9,9);
    S_0(0,0) = 0.01;
    S_0(1,1) = 0.01;
    S_0(2,2) = 0.01;
    S_0(3,3) = 1000;
    S_0(4,4) = 1000;
    S_0(5,5) = 1000;
    S_0(6,6) = 1000;
    S_0(7,7) = 1000;
    S_0(8,8) = 1000;

    S_prev = S_0;
    S_est = S_0;

    arma::mat Q = arma::eye(9,9);
    Q(0,0) = 1000.0; //.00001;
    Q(1,1) = 1000.0; //.00001;
    Q(2,2) = 1000.0; //.00001;
    Q(3,3) = 0; //Leave these as zero
    Q(4,4) = 0;
    Q(5,5) = 0;
    Q(6,6) = 0;
    Q(7,7) = 0;
    Q(8,8) = 0;

    arma::mat R = arma::mat{{100000.0,  0.0},
                            {0.0,  1000000.0}};
    
    while(ros::ok()){

        ROS_WARN("Q: ");
        Q.print(std::cout);

        if (teleporting == false){
            pos = D.get_q(wheel_angles, old_wheel_angles, old_pos);
            // pos = D.get_q(twist,old_pos);
        }

        odom_tf.header.stamp = ros::Time::now();
        odom_tf.header.frame_id = "world";
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

        u(0,0) = twist.w/rate;
        u(1,0) = twist.vx/rate;
        u(2,0) = 0;

        ROS_WARN("u: ");
        u.print(std::cout);

        ROS_WARN("x_prev: ");
        x_prev.print(std::cout);

        if (turtlelib::almost_equal(twist.w,0,.0001)){
            x_est(0,0) = x_prev(0,0);
            x_est(1,0) = x_prev(1,0) + u(1,0)*cos(x_prev(0,0));
            x_est(2,0) = x_prev(2,0) + u(1,0)*sin(x_prev(0,0));
            x_est(3,0) = x_prev(3,0);
            x_est(4,0) = x_prev(4,0);
            x_est(5,0) = x_prev(5,0);
            x_est(6,0) = x_prev(6,0);
            x_est(7,0) = x_prev(7,0);
            x_est(8,0) = x_prev(8,0);

            A = arma::eye(9,9);
            A(1,0) = -1*u(1,0) * sin(x_prev(0,0));
            A(2,0) = u(1,0) * cos(x_prev(0,0));
        }
        else{
            x_est(0,0) = x_prev(0,0) + u(0,0);
            x_est(1,0) = x_prev(1,0) - (u(1,0)/u(0,0))*sin(x_prev(0,0)) + (u(1,0)/u(0,0))*sin(x_prev(0,0)+u(0,0));
            x_est(2,0) = x_prev(2,0) + (u(1,0)/u(0,0))*cos(x_prev(0,0)) - (u(1,0)/u(0,0))*cos(x_prev(0,0)+u(0,0));
            x_est(3,0) = x_prev(3,0);
            x_est(4,0) = x_prev(4,0);
            x_est(5,0) = x_prev(5,0);
            x_est(6,0) = x_prev(6,0);
            x_est(7,0) = x_prev(7,0);
            x_est(8,0) = x_prev(8,0);

            A = arma::eye(9,9);
            A(1,0) = -1*(u(1,0)/u(0,0)) * cos(x_prev(0,0)) + (u(1,0)/u(0,0)) * cos(x_prev(0,0)+ u(0,0));
            A(2,0) = -1*(u(1,0)/u(0,0)) * sin(x_prev(0,0)) + (u(1,0)/u(0,0)) * sin(x_prev(0,0)+ u(0,0));
        }

        ROS_WARN("x_est: ");
        x_est.print(std::cout);

        ROS_WARN("A: ");
        A.print(std::cout);
        
        arma::mat A_T = A.t();

        S_est = A * S_prev * A_T + Q;

        ROS_WARN("S_est: ");
        S_est.print(std::cout);

        // Kalman Filter //
        ROS_WARN("Num Markers: %ld", found_obstacles.markers.size());
        if (found_obstacles.markers.size()>0){
            for (unsigned int i=0; i<found_obstacles.markers.size(); i++){
                ROS_WARN("Looking at marker %d",i);
                double obj_x = found_obstacles.markers[i].pose.position.x;
                double obj_y = found_obstacles.markers[i].pose.position.y;
                ROS_WARN("Found coordinates %3.2f, %3.2f", obj_x, obj_y);
                z_measured(0,0) = sqrt(pow(obj_x,2)+pow(obj_y,2));
                z_measured(1,0) = atan2(obj_y,obj_x);

                ROS_WARN("z_measured: ");
                z_measured.print(std::cout);

                ROS_WARN("x_est: ");
                x_est.print(std::cout);

                //Compute theoretical measurement
                ROS_WARN("run_count: %d",run_count);
                if (run_count < 3){
                    x_est(3+2*i,0) = x_est(1,0) + z_measured(0,0)*cos(z_measured(1,0)+x_est(0,0));
                    x_est(4+2*i,0) = x_est(2,0) + z_measured(0,0)*sin(z_measured(1,0)+x_est(0,0));
                }
                run_count = run_count + 1;

                ROS_WARN("Computed x_est map values: %3.2f,%3.2f",x_est(3+2*i,0),x_est(4+2*i,0));

                h(0,0) = sqrt(pow(x_est(3+2*i,0) - x_est(1,0),2)+pow(x_est(4+2*i,0) - x_est(2,0),2));
                h(1,0) = atan2(x_est(4+2*i,0) - x_est(2,0), x_est(3+2*i,0) - x_est(1,0)) - x_est(0,0);

                ROS_WARN("h: ");
                h.print(std::cout);
                
                z_est = h;

                ROS_WARN("z_est: ");
                z_est.print(std::cout);

                //Compute the Kalman gain
                double dx = x_est(3+2*i,0) - x_est(1,0);
                double dy = x_est(4+2*i,0) - x_est(2,0);
                double d = pow(dx,2)+pow(dy,2);

                H = arma::zeros(2,9);

                H(0,0) = 0;  

                H(1,0) = -1; 

                ROS_WARN("d = %3.2f",d);
                if (d != 0){
                    H(0,1) = -1*dx/sqrt(d); 
                    H(0,3+2*i) = dx/sqrt(d); 
                    H(0,2) = -1*dy/sqrt(d); 
                    H(0,4+2*i) = dy/sqrt(d); 

                    H(1,1) = dy/d; 
                    H(1,2) = -1*dx/d; 
                    H(1,3+2*i) = -1*dy/d; 
                    H(1,4+2*i) = dx/d;                     
                }

                ROS_WARN("H: ");
                H.print(std::cout);

                ROS_WARN("S_est: ");
                S_est.print(std::cout);

                arma::mat H_T = H.t();

                K = S_est * H_T * (H*S_est*H_T+R).i();

                ROS_WARN("K: ");
                K.print(std::cout);
                
                //Posterior State Update
                dz(0,0) = z_measured(0,0) - z_est(0,0);
                dz(1,0) = z_measured(1,0) - z_est(1,0);                
                //dz(1,0) = turtlelib::normalize_angle(dz(1,0));
                
                ROS_WARN("dz: ");
                dz.print(std::cout);
                
                x_est = x_est + K * dz;
                ROS_WARN("x_est: ");
                x_est.print(std::cout);
                
                ROS_WARN("Final x_est: ");
                x_est.print(std::cout);

                //Posterior Covariance
                S_est = (I - K*H) * S_est; 

                ROS_WARN("Final S_est: ");
                S_est.print(std::cout);

            }
            x_0 = x_est;
            S_0 = S_est;
        }
        
        x_prev = x_0;
        S_prev = S_0;
        
        ROS_WARN("--------------------");
        /////////////////
        
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}