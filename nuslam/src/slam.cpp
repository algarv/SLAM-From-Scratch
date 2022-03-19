/// \file slam.cpp
/// \brief Publishes odometry messages and the predicted state of the turtlebot using Kalman filter.
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
///     SLAM_Markers (visualization_msgs/MarkerArray): publishes a marker array corresponding to the estimated location of landmarks
///     blue_nav_msgs/Path (nav_msgs/Path): Publishes the path of the blue robot
///     green_nav_msgs/Path (nav_msgs/Path): Publishes the path of the green robot
/// SUBSCRIBERS:
///     joint_states (sensor_msgs/JointState): Receives the wheel joint angles
///     sensor (visualization_msgs/MarkerArray): Recieves the estimated obstacle position from the landmark detection node

#include <ros/ros.h>
#include <string>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
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
int num_obstacles = 0;
static bool teleporting = false;
static double x = 0, y = 0, w = 0;
static double max_range;
std::string body_id, odom_id, wheel_left, wheel_right;
turtlelib::Wheel_Angle wheel_angles = {.L = 0, .R = 0}, old_wheel_angles = {.L = 0, .R = 0};
turtlelib::Wheel_Angular_Velocities wheel_vels;
turtlelib::DiffDrive D;
turtlelib::Twist2D twist;
turtlelib::q pos, old_pos;
static geometry_msgs::TransformStamped odom_tf, kalman_tf, map_tf;
static ros::Subscriber js_sub, sensor_sub;
static ros::Publisher odom_pub, SLAM_marker_pub, odom_path_pub, SLAM_path_pub;
nav_msgs::Odometry odom_msg;
static nav_msgs::Path odom_path_msg, SLAM_path_msg;
static visualization_msgs::MarkerArray SLAM_marker_array;

arma::mat H;
arma::mat H_T;
arma::mat K;
arma::mat h(2,1);
arma::mat u(3,1);
arma::mat z(2,3);
arma::mat v_k(2,1);
arma::mat z_est(2,1);
arma::mat z_mes(2,1);
arma::mat z_measured(2,1);
arma::mat dz(2,1);
arma::mat x_0;
arma::mat x_prev(3,1);
arma::mat x_est(3,1);
arma::mat S_0;
arma::mat S_prev;
arma::mat S_est;
arma::mat R = arma::mat{{10.0,  0.0},
                        {0.0,  10.0}};

int run_count = 0;
double n = 3;

struct obstacle{
    float x;
    float y;
    float d;
};

std::vector<obstacle> new_obj_list;
bool first_run = true;

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

/// \brief Publishes a marker array corresponding to the estimated state of landmarks according to the Kalman filter
///
/// \param state - The state vector from the kalman filter
void update_obstacles(arma::mat state){
    int id = 0;

    if (state.n_rows > .5){
        double size = (state.n_rows - 3)/2;
        ROS_WARN("size: %f",size);
        SLAM_marker_array.markers.resize(size);
        // ros::Duration duration(3.0);

        for (unsigned int i = 0; i<size; i++) {
            double d = sqrt(pow(x_est(3+2*i,0)-state(1,0),2)+pow(x_est(4+2*i,0)-state(2,0),2));
            // ROS_WARN("Distance %d: %6.2f",i,d);
            if (d > max_range) {
                SLAM_marker_array.markers[i].action = visualization_msgs::Marker::DELETE;
                // ROS_WARN("Deleting %d",i);
            }
            else {
                SLAM_marker_array.markers[i].action = visualization_msgs::Marker::ADD;
            }
            SLAM_marker_array.markers[i].header.frame_id = "map";
            SLAM_marker_array.markers[i].header.stamp = ros::Time::now();
            SLAM_marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
            SLAM_marker_array.markers[i].id = id;
            SLAM_marker_array.markers[i].pose.position.x = state(3+2*i,0);
            SLAM_marker_array.markers[i].pose.position.y = state(4+2*i,0);
            SLAM_marker_array.markers[i].pose.position.z = .125;
            SLAM_marker_array.markers[i].pose.orientation.x = 0.0;
            SLAM_marker_array.markers[i].pose.orientation.y = 0.0;
            SLAM_marker_array.markers[i].pose.orientation.z = 0.0;
            SLAM_marker_array.markers[i].pose.orientation.w = 1.0;
            SLAM_marker_array.markers[i].scale.x = .25;
            SLAM_marker_array.markers[i].scale.y = .25;
            SLAM_marker_array.markers[i].scale.z = .25;
            SLAM_marker_array.markers[i].color.a = 1.0;
            SLAM_marker_array.markers[i].color.r = 0.0;
            SLAM_marker_array.markers[i].color.g = 1.0;
            SLAM_marker_array.markers[i].color.b = 0.0;
            // SLAM_marker_array.markers[i].lifetime = duration;
            SLAM_marker_array.markers[i].frame_locked = true;
            id += 1;
        }
        SLAM_marker_pub.publish(SLAM_marker_array);
    }
}

/// \brief Callback for the marker array subscriber that assigns data from the LiDAR to the global obstacle variable
///
/// \param obstacles - Marker array received from the sensor publisher
void get_obj(const visualization_msgs::MarkerArray &obstacles){

    unsigned long int N = (x_est.n_rows - 3)/2;

    new_obj_list.empty();
    ROS_WARN("Starting with %d obstacles",N);
    ROS_WARN("Sorting through %d potential new markers",  obstacles.markers.size());
    
    if (first_run){
        if (obstacles.markers.size() > 1){
            for (unsigned long int i = 0; i < obstacles.markers.size(); i++){
                obstacle new_obstacle; //in robot frame
                new_obstacle.d = obstacles.markers[i].scale.x;
                new_obstacle.x = obstacles.markers[i].pose.position.x;
                new_obstacle.y = obstacles.markers[i].pose.position.y;
                
                double dx = new_obstacle.x - x_est(1,0); //map frame
                double dy = new_obstacle.y - x_est(2,0);  
                double d = pow(dx,2)+pow(dy,2);

                S_est.resize(3+(obstacles.markers.size()*2),3+(obstacles.markers.size()*2));
                H.resize(2,3+(obstacles.markers.size()*2));
                H = arma::zeros(2,3+(obstacles.markers.size()*2));
                H(0,0) = 0;  
                H(1,0) = -1;

                ROS_WARN("d = %6.2f",d);
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

                new_obj_list.push_back(new_obstacle);
                ROS_WARN("Starting with Obstacle (%3.2f, %3.2f)", new_obstacle.x, new_obstacle.y);
            
                z_mes(0,0) = sqrt(pow(new_obstacle.x,2)+pow(new_obstacle.y,2));
                z_mes(1,0) = atan2(new_obstacle.y,new_obstacle.x);

                int new_size = x_est.n_rows + 2;
                x_est.resize(new_size,1);
                x_est(3+2*i,0) = x_est(1,0) + z_mes(0,0)*cos(z_mes(1,0)+x_est(0,0));
                x_est(4+2*i,0) = x_est(2,0) + z_mes(0,0)*sin(z_mes(1,0)+x_est(0,0));            
                ROS_WARN("Append to x_est: ");
                x_est.print();
            }
        first_run = false;
        }
    }
    else{
     for (unsigned long int i = 0; i < obstacles.markers.size(); i++){

        obstacle new_obstacle; //in robot frame
        new_obstacle.d = obstacles.markers[i].scale.x;
        new_obstacle.x = obstacles.markers[i].pose.position.x;
        new_obstacle.y = obstacles.markers[i].pose.position.y;
        ROS_WARN("Possible New Obstacle (%3.2f, %3.2f)", new_obstacle.x, new_obstacle.y);

        std::vector<double> d_list;
        
        std::vector<obstacle> obj_list; //in map frame
        for (unsigned long int k = 3; k < x_est.n_rows; k+=2){
            obstacle state_obstacle; 
            state_obstacle.d = .25;
            
            state_obstacle.x = x_est(k,0);
            state_obstacle.y = x_est(k+1,0);
            obj_list.push_back(state_obstacle);
        }
        
        // obj_list.push_back(new_obstacle);

        for (unsigned long int j = 0; j < obj_list.size(); j++){
            double dx = obj_list[j].x - x_est(1,0); //map frame
            double dy = obj_list[j].y - x_est(2,0);  
            double d = pow(dx,2)+pow(dy,2);

            S_est.resize(3+(obj_list.size()*2),3+(obj_list.size()*2));
            H.resize(2,3+(obj_list.size()*2));
            H = arma::zeros(2,3+(obj_list.size()*2));
            H(0,0) = 0;  
            H(1,0) = -1;

            ROS_WARN("d = %3.2f",d);
            if (d != 0){
                H(0,1) = -1*dx/sqrt(d); 
                H(0,3+2*j) = dx/sqrt(d); 
                H(0,2) = -1*dy/sqrt(d); 
                H(0,4+2*j) = dy/sqrt(d); 

                H(1,1) = dy/d; 
                H(1,2) = -1*dx/d; 
                H(1,3+2*j) = -1*dy/d; 
                H(1,4+2*j) = dx/d;                     
            }

            ROS_WARN("H: ");
            H.print(std::cout);
            
            
            arma::mat psi;
            psi = H*S_est*H.t() + R;
            
            ROS_WARN("Psi: ");
            psi.print(std::cout);

            z_mes(0,0) = sqrt(pow(new_obstacle.x,2)+pow(new_obstacle.y,2));
            z_mes(1,0) = atan2(new_obstacle.y,new_obstacle.x);
            ROS_WARN("z_measured: ");
            z_mes.print(std::cout);

            if (x_est.n_rows > 4+2*j){
                z_est(0,0) = sqrt(pow(x_est(3+2*j,0) - x_est(1,0),2)+pow(x_est(4+2*j,0) - x_est(2,0),2));
                z_est(1,0) = atan2(x_est(4+2*j,0) - x_est(2,0), x_est(3+2*j,0) - x_est(1,0)) - x_est(0,0);
            }
            else{
                z_est(0,0) = 0;
                z_est(1,0) = 0;
            }
            ROS_WARN("z_estimated: ");
            z_est.print(std::cout);

            dz(0,0) = z_mes(0,0) - z_est(0,0);
            dz(1,0) = z_mes(1,0) - z_est(1,0);                
            //dz(1,0) = turtlelib::normalize_angle(dz(1,0));
            ROS_WARN("dz: ");
            dz.print(std::cout);

            arma::mat d_mah_mat;
            d_mah_mat = dz.t() * psi.i() * dz;
            // ROS_WARN("Why does this need to be a matrix?");
            // d_mah_mat.print(std::cout);

            double d_mah = d_mah_mat(0,0);
            d_list.push_back(d_mah);
            // ROS_WARN("Added mahalanobis distance: %3.6f", d_mah);
        }
        ROS_WARN("d_list size: %ld",d_list.size());

        if (d_list.size()>0){    
            double d_sum = 0;
            for (unsigned long int j=0; j < d_list.size() - 1; j++){
                d_sum += d_list[j];
            }
            double mean_d = d_sum/(d_list.size() - 1);
            double d_StD = 0;
            for (unsigned long int j=0; j < d_list.size() - 1; j++){
                d_StD += pow((d_list[j] - mean_d),2);
            }
            d_StD /= d_list.size() - 1;

            double threshold = 3 * d_StD;
            
            // d_list[d_list.size() - 1] = threshold;
            // ROS_WARN("Adding threshold distance: %3.6f", threshold);


            auto min_addr = std::min_element(d_list.begin(),d_list.end());
            int min_index = std::min_element(d_list.begin(),d_list.end()) - d_list.begin();
            double min_value = *min_addr;
            // int min_index = 0;
            // for (int j=0; j<d_list.size(); j++){
            //     if (d_list[j] <= d_list[min_index]){
            //             min_index = j;
            //     }
            // }
            if (min_value >= threshold){
                int old_size = 3 + 2*obj_list.size();

                new_obj_list.push_back(new_obstacle);
                ROS_WARN("New obstacle");


                int new_size = x_est.n_rows + 2;
                x_est.resize(new_size,1);
                x_est(new_size - 2,0) = x_est(1,0) + z_mes(0,0)*cos(z_mes(1,0)+x_est(0,0));
                x_est(new_size - 1,0) = x_est(2,0) + z_mes(0,0)*sin(z_mes(1,0)+x_est(0,0));
                ROS_WARN("Append to x_est: ");
                x_est.print();
            }
            else{
                ROS_WARN("Associated with %d",min_index);
            }
       }
     }
    }
    num_obstacles = (x_est.n_rows - 3)/2;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "nusalm");
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
    nh.getParam("/max_range",max_range);

    nh.getParam("x0", x);
    nh.getParam("y0", y);
    nh.getParam("theta0", w);

    nh.param<std::string>("odom_id",odom_id,"odom");

    nh.getParam("rate", rate);
    // rate = 5;
    ros::Rate r(rate);
    
    odom_pub = pub_nh.advertise<nav_msgs::Odometry>("odom", rate);
    SLAM_marker_pub = pub_nh.advertise<visualization_msgs::MarkerArray>("/SLAM_markers", 10);
    odom_path_pub = pub_nh.advertise<nav_msgs::Path>("blue_nav_msgs/Path",10);
    SLAM_path_pub = pub_nh.advertise<nav_msgs::Path>("green_nav_msgs/Path",10);

    js_sub = pub_nh.subscribe("red/joint_states",10,update_odom);

    //sensor_sub = pub_nh.subscribe("/fake_sensor",10,get_obj);
    sensor_sub = pub_nh.subscribe("/sensor",10,get_obj);

    old_pos.theta = w;
    old_pos.x = x;
    old_pos.y = y;

    tf2_ros::TransformBroadcaster odom_broadcaster;
    tf2_ros::TransformBroadcaster kalman_broadcaster;
    tf2_ros::TransformBroadcaster map_broadcaster;

    // x_0 = arma::zeros(3 + num_obstacles*2,1);
    // x_0(0,0) = w;
    // x_0(1,0) = x;
    // x_0(2,0) = y;
    
    x_est(0,0) = w;
    x_est(1,0) = x;
    x_est(2,0) = y;

    x_prev = x_est;

    S_0 = arma::zeros(3+(num_obstacles*2),3+(num_obstacles*2));
    S_0(0,0) = 0.01;
    S_0(1,1) = 0.01;
    S_0(2,2) = 0.01;
    for (int i = 3; i < 3+(num_obstacles*2); i++){
        S_0(i,i) = 1000;
    }

    S_prev = S_0;
    S_est = S_0;
    
    arma::mat R = arma::mat{{10.0,  0.0},
                            {0.0,  10.0}};
    
    int old_obstacle_count = 0;
    while(ros::ok()){

        // int num_added_obstacles = num_obstacles - old_obstacle_count;
        // ROS_WARN("Added %d obstacles since last run. (%d to %d)", num_added_obstacles, old_obstacle_count, num_obstacles);
        // ROS_WARN("Q: ");
        // Q.print(std::cout);

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

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pos.x;
        pose.pose.position.y = pos.y;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        odom_path_msg.header.frame_id = "world";
        odom_path_msg.header.stamp = ros::Time::now();
        odom_path_msg.poses.push_back(pose);

        odom_path_pub.publish(odom_path_msg);

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
        kalman_tf.transform.translation.x = x_est(1,0);
        kalman_tf.transform.translation.y = x_est(2,0);
        kalman_tf.transform.translation.z = 0;
        q.setRPY(0, 0, x_est(0,0));
        kalman_tf.transform.rotation.x = q.x();
        kalman_tf.transform.rotation.y = q.y();
        kalman_tf.transform.rotation.z = q.z();
        kalman_tf.transform.rotation.w = q.w();

        kalman_broadcaster.sendTransform(kalman_tf);

        pose.pose.position.x = x_est(1,0);
        pose.pose.position.y = x_est(2,0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        SLAM_path_msg.header.frame_id = "map";
        SLAM_path_msg.header.stamp = ros::Time::now();
        SLAM_path_msg.poses.push_back(pose);

        SLAM_path_pub.publish(SLAM_path_msg);

        old_pos = pos;

        teleporting = false;

        u(0,0) = twist.w/rate;
        u(1,0) = twist.vx/rate;
        u(2,0) = 0;

        // ROS_WARN("u: ");
        // u.print(std::cout);

        x_prev.resize(3+(num_obstacles*2),1);
        // ROS_WARN("x_prev: ");
        // x_prev.print(std::cout);

        // x_est.resize(3+(num_obstacles*2),1);
        // ROS_WARN("x_est: ");
        // x_est.print(std::cout);

        S_prev.resize(3+(num_obstacles*2),3+(num_obstacles*2));
        // ROS_WARN("S_prev: ");
        // S_prev.print(std::cout);

        arma::mat A = arma::eye(3+(num_obstacles*2),3+(num_obstacles*2));
        // ROS_WARN("Initialize A: ");
        // A.print(std::cout);

        if (turtlelib::almost_equal(twist.w,0,.0001)){
            x_est(0,0) = x_prev(0,0);
            x_est(1,0) = x_prev(1,0) + u(1,0)*cos(x_prev(0,0));
            x_est(2,0) = x_prev(2,0) + u(1,0)*sin(x_prev(0,0));
            // if (num_obstacles != 0){
            //     for (int i = 3; i < 3+(num_obstacles*2); i++){
            //         x_est(i,0) = x_prev(i,0);
            //     }
            // }

            A(1,0) = -1*u(1,0) * sin(x_prev(0,0));
            A(2,0) = u(1,0) * cos(x_prev(0,0));
        }
        else{
            x_est(0,0) = x_prev(0,0) + u(0,0);
            x_est(1,0) = x_prev(1,0) - (u(1,0)/u(0,0))*sin(x_prev(0,0)) + (u(1,0)/u(0,0))*sin(x_prev(0,0)+u(0,0));
            x_est(2,0) = x_prev(2,0) + (u(1,0)/u(0,0))*cos(x_prev(0,0)) - (u(1,0)/u(0,0))*cos(x_prev(0,0)+u(0,0));
            // if (num_obstacles != 0){
            //     for (int i = 3; i < 3+(num_obstacles*2); i++){
            //         x_est(i,0) = x_prev(i,0);
            //     }
            // }
            A(1,0) = -1*(u(1,0)/u(0,0)) * cos(x_prev(0,0)) + (u(1,0)/u(0,0)) * cos(x_prev(0,0)+ u(0,0));
            A(2,0) = -1*(u(1,0)/u(0,0)) * sin(x_prev(0,0)) + (u(1,0)/u(0,0)) * sin(x_prev(0,0)+ u(0,0));
        }

        ROS_WARN("x_est: ");
        x_est.print(std::cout);

        // ROS_WARN("A: ");
        // A.print(std::cout);
        
        arma::mat A_T = A.t();

        arma::mat Q = arma::zeros(3+(num_obstacles*2),3+(num_obstacles*2));
        Q(0,0) = 100.0;
        Q(1,1) = 100.0;
        Q(2,2) = 100.0;

        S_est = A * S_prev * A_T + Q;

        ROS_WARN("S_est: ");
        S_est.print(std::cout);

        // Kalman Filter //
        // ROS_WARN("Num Markers: %ld", new_obj_list.size());
        // if (new_obj_list.size()>0){
        //     for (unsigned int i=0; i<new_obj_list.size(); i++){
        //         // ROS_WARN("Looking at marker %d",i);
        //         double obj_x = new_obj_list[i].x;
        //         double obj_y = new_obj_list[i].y;
        //         // ROS_WARN("Found coordinates %3.2f, %3.2f", obj_x, obj_y);
        //         z_measured(0,0) = sqrt(pow(obj_x,2)+pow(obj_y,2));
        //         z_measured(1,0) = atan2(obj_y,obj_x);

        //         // ROS_WARN("z_measured: ");
        //         // z_measured.print(std::cout);

        //         // ROS_WARN("x_est: ");
        //         // x_est.print(std::cout);

        //         // ROS_WARN("Computed x_est map values: %3.2f,%3.2f",x_est(3+2*i,0),x_est(4+2*i,0));

        //         // h(0,0) = sqrt(pow(x_est(3+2*i,0),2)+pow(x_est(4+2*i,0),2));
        //         // h(1,0) = atan2(x_est(4+2*i,0), x_est(3+2*i,0));
        //         h(0,0) = sqrt(pow(x_est(3+2*i,0) - x_est(1,0),2)+pow(x_est(4+2*i,0) - x_est(2,0),2));
        //         h(1,0) = atan2(x_est(4+2*i,0) - x_est(2,0), x_est(3+2*i,0) - x_est(1,0)) - x_est(0,0);

        //         // ROS_WARN("h: ");
        //         // h.print(std::cout);
                
        //         z_est = h;

        //         // ROS_WARN("z_est: ");
        //         // z_est.print(std::cout);

                // // Compute the Kalman gain
                // double dx = x_est(3+2*i,0) - x_est(1,0);
                // double dy = x_est(4+2*i,0) - x_est(2,0);
                // double d = pow(dx,2)+pow(dy,2);

                // arma::mat H;
                // H = arma::zeros(2,3+(num_obstacles*2));

                // H(0,0) = 0;  

                // H(1,0) = -1; 

                // // ROS_WARN("d = %3.2f",d);
                // if (d != 0){
                //     H(0,1) = -1*dx/sqrt(d); 
                //     H(0,3+2*i) = dx/sqrt(d); 
                //     H(0,2) = -1*dy/sqrt(d); 
                //     H(0,4+2*i) = dy/sqrt(d); 

                //     H(1,1) = dy/d; 
                //     H(1,2) = -1*dx/d; 
                //     H(1,3+2*i) = -1*dy/d; 
                //     H(1,4+2*i) = dx/d;                     
                // }

        //         // ROS_WARN("H: ");
        //         // H.print(std::cout);

        //         // ROS_WARN("S_est: ");
        //         // S_est.print(std::cout);

        //         arma::mat H_T = H.t();

        //         arma::mat K;
        //         K = S_est * H_T * (H*S_est*H_T+R).i();

        //         // ROS_WARN("K: ");
        //         // K.print(std::cout);
                
        //         //Posterior State Update
        //         dz(0,0) = z_measured(0,0) - z_est(0,0);
        //         dz(1,0) = z_measured(1,0) - z_est(1,0);                
        //         // dz(1,0) = turtlelib::normalize_angle(dz(1,0));
                
        //         // ROS_WARN("dz: ");
        //         // dz.print(std::cout);
                
        //         x_est = x_est + K * dz;
        //         // ROS_WARN("x_est: ");
        //         // x_est.print(std::cout);
                
        //         // ROS_WARN("Final x_est: ");
        //         // x_est.print(std::cout);

        //         arma::mat I = arma::eye(3+(num_obstacles*2),3+(num_obstacles*2));

        //         //Posterior Covariance
        //         S_est = (I - K*H) * S_est; 

        //         // ROS_WARN("Final S_est: ");
        //         // S_est.print(std::cout);

        H.resize(2,3 + 2*num_obstacles);
        S_est.resize(3 + 2*num_obstacles,3 + 2*num_obstacles);
        H_T = H.t();
        K = S_est * H_T * (H*S_est*H_T+R).i();
        x_est = x_est + K * dz;

        arma::mat I = arma::eye(3+(num_obstacles*2),3+(num_obstacles*2));
        S_est = (I - K*H) * S_est; 

        //     }
        // }
        // x_0 = x_est;
        // S_0 = S_est;
        update_obstacles(x_est);            

        x_prev = x_est;
        S_prev = S_est;
        
        ROS_WARN("--------------------");
        /////////////////
        
        old_obstacle_count = num_obstacles;

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}