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
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt64.h>
#include "std_msgs/String.h"
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include "nusim/teleport.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <ros/console.h>
#include <random>
#include <armadillo>

static int rate;
static bool teleporting = false;
static double x = 0, y = 0, w = 0;
static double x_length = 0, y_length = 0, width = 0;
static double left_rot_vel = 0.0, right_rot_vel = 0.0;
static double ada_L, ada_R;
static double r = .25/2;
static double mticks_radsec, eticks_rad, slip_min, slip_max, collision_radius, min_range, max_range, angle_min, angle_max, angle_increment;
static std::string left_wheel = "red_wheel_left_joint", right_wheel = "red_wheel_right_joint", odom_id;
std::vector<double> obj_x_list, obj_y_list, obj_d_list, radsec;
static std_msgs::UInt64 ts;
static sensor_msgs::JointState wheels;
static sensor_msgs::LaserScan laser_msg;
static geometry_msgs::TransformStamped sim_tf;
static nuturtlebot_msgs::SensorData sensor_data; 
static nav_msgs::Path path_msg;
static ros::Publisher obj_pub, js_pub, ts_pub, arena_pub, sensor_pub, fake_sensor_pub, laser_pub, path_pub;
static ros::Subscriber wheel_sub;
static ros::ServiceServer rs_service, tp_service;
static visualization_msgs::MarkerArray obstacle, obj_array, marker_arena, arena_array, fake_sensor_array;
static turtlelib::Wheel_Angle theoretical_wheel_angles, wheel_angles, old_wheel_angles;
static turtlelib::Wheel_Angular_Velocities wheel_vels;
static turtlelib::Twist2D twist;
static turtlelib::q pos, old_pos;

std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

bool restart(std_srvs::Empty::Request&, std_srvs::Empty::Response&){
/// \brief Send the turtle bot back to the origin of the world frame and restart the timestep counter.
///
/// \param request - Empty::Request
/// \param response - Empty::Reponse
/// \returns true

    ts.data = 0;

    pos.x = x;
    pos.y = y;
    pos.theta = w;

    wheel_angles = {.L = 0, .R = 0};
    old_wheel_angles = {.L = 0, .R = 0};

    sensor_data.left_encoder = 0;
    sensor_data.right_encoder = 0;

    teleporting = true;

    return true;
}

bool teleport(nusim::teleport::Request &pose, nusim::teleport::Response&){
/// \brief Teleports the turtle to the specified position and rotation.
///
/// \param pose - position input from parameter
/// \param response - Empty::Reponse
/// \returns true

    pos.x = pose.x;
    pos.y = pose.y;
    pos.theta = pose.w;
 
    teleporting = true;

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
    for (unsigned int i = 0; i<obj_x_list.size(); i+=1) {
        
        obstacle.markers[i].header.frame_id = "world";
        obstacle.markers[i].ns = "nusim_node";
        obstacle.markers[i].header.stamp = ros::Time::now();
        obstacle.markers[i].type = visualization_msgs::Marker::CYLINDER;
        obstacle.markers[i].action = visualization_msgs::Marker::ADD;
        obstacle.markers[i].id = id;
        obstacle.markers[i].pose.position.x = obj_x_list[i];
        obstacle.markers[i].pose.position.y = obj_y_list[i];
        obstacle.markers[i].pose.position.z = .125;
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

visualization_msgs::MarkerArray make_arena(float x_length, float y_length){
    
    arena_array.markers.resize(4);

    int i = obj_x_list.size();

    arena_array.markers[0].header.frame_id = "world";
    arena_array.markers[0].ns = "nusim_node";
    arena_array.markers[0].header.stamp = ros::Time::now();
    arena_array.markers[0].type = visualization_msgs::Marker::CUBE;
    arena_array.markers[0].action = visualization_msgs::Marker::ADD;
    arena_array.markers[0].id = i;
    arena_array.markers[0].pose.position.x = x_length/2;
    arena_array.markers[0].pose.position.y = 0;
    arena_array.markers[0].pose.position.z = 0.125;
    arena_array.markers[0].pose.orientation.x = 0.0;
    arena_array.markers[0].pose.orientation.y = 0.0;
    arena_array.markers[0].pose.orientation.z = 0.0;
    arena_array.markers[0].pose.orientation.w = 1.0;
    arena_array.markers[0].scale.x = width;
    arena_array.markers[0].scale.y = y_length - width;
    arena_array.markers[0].scale.z = .25;
    arena_array.markers[0].color.a = 1.0;
    arena_array.markers[0].color.r = 1.0;
    arena_array.markers[0].color.g = 0.0;
    arena_array.markers[0].color.b = 0.0;

    arena_array.markers[1].header.frame_id = "world";
    arena_array.markers[1].ns = "nusim_node";
    arena_array.markers[1].header.stamp = ros::Time::now();
    arena_array.markers[1].type = visualization_msgs::Marker::CUBE;
    arena_array.markers[1].action = visualization_msgs::Marker::ADD;
    arena_array.markers[1].id = i+3;
    arena_array.markers[1].pose.position.x = 0;
    arena_array.markers[1].pose.position.y = -y_length/2;
    arena_array.markers[1].pose.position.z = 0.125;
    arena_array.markers[1].pose.orientation.x = 0.0;
    arena_array.markers[1].pose.orientation.y = 0.0;
    arena_array.markers[1].pose.orientation.z = 0.0;
    arena_array.markers[1].pose.orientation.w = 1.0;
    arena_array.markers[1].scale.x = x_length + width;
    arena_array.markers[1].scale.y = width;
    arena_array.markers[1].scale.z = .25;
    arena_array.markers[1].color.a = 1.0;
    arena_array.markers[1].color.r = 1.0;
    arena_array.markers[1].color.g = 0.0;
    arena_array.markers[1].color.b = 0.0;

    arena_array.markers[2].header.frame_id = "world";
    arena_array.markers[2].ns = "nusim_node";
    arena_array.markers[2].header.stamp = ros::Time::now();
    arena_array.markers[2].type = visualization_msgs::Marker::CUBE;
    arena_array.markers[2].action = visualization_msgs::Marker::ADD;
    arena_array.markers[2].id = i+2;
    arena_array.markers[2].pose.position.x = -x_length/2;
    arena_array.markers[2].pose.position.y = 0;
    arena_array.markers[2].pose.position.z = 0.125;
    arena_array.markers[2].pose.orientation.x = 0.0;
    arena_array.markers[2].pose.orientation.y = 0.0;
    arena_array.markers[2].pose.orientation.z = 0.0;
    arena_array.markers[2].pose.orientation.w = 1.0;
    arena_array.markers[2].scale.x = width;
    arena_array.markers[2].scale.y = y_length - width;
    arena_array.markers[2].scale.z = .25;
    arena_array.markers[2].color.a = 1.0;
    arena_array.markers[2].color.r = 1.0;
    arena_array.markers[2].color.g = 0.0;
    arena_array.markers[2].color.b = 0.0;
    
    arena_array.markers[3].header.frame_id = "world";
    arena_array.markers[3].ns = "nusim_node";
    arena_array.markers[3].header.stamp = ros::Time::now();
    arena_array.markers[3].type = visualization_msgs::Marker::CUBE;
    arena_array.markers[3].action = visualization_msgs::Marker::ADD;
    arena_array.markers[3].id = i+1;
    arena_array.markers[3].pose.position.x = 0;
    arena_array.markers[3].pose.position.y = y_length/2;
    arena_array.markers[3].pose.position.z = 0.125;
    arena_array.markers[3].pose.orientation.x = 0.0;
    arena_array.markers[3].pose.orientation.y = 0.0;
    arena_array.markers[3].pose.orientation.z = 0.0;
    arena_array.markers[3].pose.orientation.w = 1.0;
    arena_array.markers[3].scale.x = x_length + width;
    arena_array.markers[3].scale.y = width;
    arena_array.markers[3].scale.z = .25;
    arena_array.markers[3].color.a = 1.0;
    arena_array.markers[3].color.r = 1.0;
    arena_array.markers[3].color.g = 0.0;
    arena_array.markers[3].color.b = 0.0;

    return arena_array;

}

void fake_sensor(const ros::TimerEvent& ){
    int id = 0;

    fake_sensor_array.markers.resize(obj_x_list.size());

    std::normal_distribution<> noise(0, 0.0);

    for (unsigned int i = 0; i<obj_x_list.size(); i+=1) {
        
        double noise_r = noise(get_random());
        double noise_phi = noise(get_random());

        turtlelib::Vector2D trans_wr;
        trans_wr.x = pos.x;
        trans_wr.y = pos.y;

        turtlelib::Transform2D T_wr(trans_wr, pos.theta);
        turtlelib::Transform2D T_rw(0);
        T_rw = T_wr.inv();

        turtlelib::Vector2D obstacle_world;
        obstacle_world.x = obj_x_list[i];
        obstacle_world.y = obj_y_list[i];

        //turtlelib::Vector2D obstacle_robot = T_rw(obstacle_world);
        ros::Duration duration(1.0);

        double d = sqrt(pow(pos.x-obstacle_world.x,2) + pow(pos.y-obstacle_world.y,2));
        // ROS_WARN("Distance %d: %6.2f",i,d);
        if (d > max_range) {
            fake_sensor_array.markers[i].action = visualization_msgs::Marker::DELETE;
            // ROS_WARN("Deleting %d",i);
        }
        else {
            fake_sensor_array.markers[i].action = visualization_msgs::Marker::ADD;
        }
        d += noise_r;

        double phi = atan2(obj_y_list[i]-pos.y, obj_x_list[i]-pos.x) - pos.theta;
        phi += noise_phi;

        double mx = d * cos(phi);
        double my = d * sin(phi);

        fake_sensor_array.markers[i].header.frame_id = "red_base_footprint";
        fake_sensor_array.markers[i].ns = "nusim_node";
        fake_sensor_array.markers[i].header.stamp = ros::Time::now();
        fake_sensor_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
        fake_sensor_array.markers[i].id = id;
        fake_sensor_array.markers[i].pose.position.x = mx; //obstacle_robot.x + noise_x;
        fake_sensor_array.markers[i].pose.position.y = my; //obstacle_robot.y + noise_y;
        fake_sensor_array.markers[i].pose.position.z = .125;
        fake_sensor_array.markers[i].pose.orientation.x = 0.0;
        fake_sensor_array.markers[i].pose.orientation.y = 0.0;
        fake_sensor_array.markers[i].pose.orientation.z = 0.0;
        fake_sensor_array.markers[i].pose.orientation.w = 1.0;
        fake_sensor_array.markers[i].scale.x = obj_d_list[i];
        fake_sensor_array.markers[i].scale.y = obj_d_list[i];
        fake_sensor_array.markers[i].scale.z = .25;
        fake_sensor_array.markers[i].color.a = 1.0;
        fake_sensor_array.markers[i].color.r = 1.0;
        fake_sensor_array.markers[i].color.g = 1.0;
        fake_sensor_array.markers[i].color.b = 0.0;
        fake_sensor_array.markers[i].lifetime = duration;
        fake_sensor_array.markers[i].frame_locked = true;
        id += 1;
    }
    
    fake_sensor_pub.publish(fake_sensor_array);
}

void update_wheel_position(const nuturtlebot_msgs::WheelCommands::ConstPtr &wheel_cmd){

    double L_cmd = wheel_cmd->left_velocity;
    double R_cmd = wheel_cmd->right_velocity;

    left_rot_vel = L_cmd * mticks_radsec;
    right_rot_vel = R_cmd * mticks_radsec;
    
    double mean_L = left_rot_vel;
    double mean_R = right_rot_vel;
    double std = 0;

    std::normal_distribution<> L(mean_L, std);
    std::normal_distribution<> R(mean_R, std);

    wheel_vels.L = left_rot_vel; //L(get_random());
    wheel_vels.R = right_rot_vel; //R(get_random());

    // ROS_WARN("Without Noise: %6.2f, With Noise: %6.2f \n",L_cmd * mticks_radsec, wheel_vels.L);
    std::uniform_real_distribution dist(slip_min, slip_max);

    ada_L = dist(get_random());
    ada_R = dist(get_random());

    // ROS_WARN("L: %6.5f, R: %6.5f", slip_min, slip_max);

    theoretical_wheel_angles.L = ((left_rot_vel / rate) + old_wheel_angles.L);
    theoretical_wheel_angles.R = ((right_rot_vel / rate) + old_wheel_angles.R);

    wheel_angles.L = (ada_L * wheel_vels.L) + theoretical_wheel_angles.L;
    wheel_angles.R = (ada_R * wheel_vels.R) + theoretical_wheel_angles.R;

    sensor_data.stamp = ros::Time::now();
    sensor_data.left_encoder = wheel_angles.L / eticks_rad;
    sensor_data.right_encoder = wheel_angles.R / eticks_rad;
    sensor_data.left_encoder = sensor_data.left_encoder % 4096;
    sensor_data.right_encoder = sensor_data.right_encoder % 4096;
    // ROS_WARN("Left: %6.2d Right: %6.2d", sensor_data.left_encoder, sensor_data.right_encoder);

}

void laser_scan(turtlelib::q robot_pos, std::vector<double> obj_x_list, std::vector<double> obj_y_list){
    
    double num_measurements = (angle_max - angle_min)/angle_increment;
    std::vector<float> laser_hits(num_measurements,0);
    double range = max_range - min_range;

    turtlelib::Vector2D robot_w;
    robot_w.x = robot_pos.x;
    robot_w.y = robot_pos.y;
    turtlelib::Transform2D T_wr(robot_w, robot_pos.theta);

    turtlelib::Transform2D T_rw(0);
    T_rw = T_wr.inv();

    turtlelib::Vector2D wall1_w = {.x = x_length/2, .y = y_length/2};
    turtlelib::Vector2D wall2_w = {.x = x_length/2, .y = -1*y_length/2};
    turtlelib::Vector2D wall3_w = {.x = -1 * x_length/2, .y = -1*y_length/2};
    turtlelib::Vector2D wall4_w = {.x = -1 * x_length/2, .y = y_length/2};
    std::vector<turtlelib::Vector2D> wall_pts = {wall1_w, wall2_w, wall3_w, wall4_w};

    int j = 0;
    for(double angle = angle_min; angle<=angle_max; angle+=angle_increment){
        
        turtlelib::Vector2D v1_r;
        v1_r.x = min_range*std::cos(angle);
        v1_r.y = min_range*std::sin(angle);
        
        double dx_r = range * std::cos(angle); 
        double dy_r = range * std::sin(angle);

        turtlelib::Vector2D v2_r;
        v2_r.x = v1_r.x + dx_r; //max_range*std::cos(angle); //v1_r.x + dx_r;
        v2_r.y = v1_r.y + dy_r; //max_range*std::sin(angle);//v1_r.y + dy_r;

        double slope = std::tan(angle+robot_pos.theta);
        
        double x_min_r = std::min({v1_r.x,v2_r.x});
        double x_max_r = std::max({v1_r.x,v2_r.x});
        double y_min_r = std::min({v1_r.y,v2_r.y});
        double y_max_r = std::max({v1_r.y,v2_r.y});

        // double int_x = 0;
        // double int_y = 0;
        // double m = 0;
        turtlelib::Vector2D a;
        turtlelib::Vector2D b;
        for (int k=0; k<=4; k++){
            if (k==4){
                a = wall_pts[4];
                b = wall_pts[0];
            }
            else{
                a = wall_pts[k];
                b = wall_pts[k+1];
            }

            if (a.y == b.y){
                // turtlelib::Vector2D wall_r;
                // wall_r = T_rw(wall_pts[k]);
                // y = slope * x + b
                double int_y_w = wall_pts[k].y;
                double int_x_w = robot_w.x + (int_y_w - robot_w.y) / slope;
                turtlelib::Vector2D int_w{.x = int_x_w, .y = int_y_w};
                turtlelib::Vector2D int_r;
                int_r = T_rw(int_w);
                double m = std::sqrt(std::pow(int_r.x,2)+std::pow(int_r.y,2));
                if ((int_r.x > x_min_r) && (int_r.x < x_max_r) && (int_r.y > y_min_r) && (int_r.y < y_max_r)){
                    if ((m < laser_hits[j]) || (laser_hits[j] == 0)){
                        laser_hits[j] = m;
                    }
                }
            }  
            if (a.x == b.x){
                // turtlelib::Vector2D wall_r;
                // wall_r = T_rw(wall_pts[k]);
                double int_x_w = wall_pts[k].x;
                double int_y_w = robot_w.y + (int_x_w-robot_w.x) * slope;
                turtlelib::Vector2D int_w{.x = int_x_w, .y = int_y_w};
                turtlelib::Vector2D int_r;
                int_r = T_rw(int_w);
                double m = std::sqrt(std::pow(int_r.x,2)+std::pow(int_r.y,2));
                // ROS_WARN("min_x: %3.2f, max_x: %3.2f", x_min_r, x_max_r);
                // ROS_WARN("int_x: %3.2f, int_y: %3.2f",int_x_w,int_y_w);
                if ((abs(int_r.x) > x_min_r) && (int_r.x < x_max_r) && (int_r.y > y_min_r) && (int_r.y < y_max_r)){
                    if ((m < laser_hits[j]) || (laser_hits[j]==0)){
                        laser_hits[j] = m;
                    }
                }
            }
        }

        for (unsigned int i = 0; i<obj_x_list.size(); i+=1) { 
            turtlelib::Vector2D trans_wo;
            trans_wo.x = obj_x_list[i];
            trans_wo.y = obj_y_list[i];

            turtlelib::Transform2D T_wo(trans_wo, 0);
            turtlelib::Transform2D T_ow(0);
            T_ow = T_wo.inv();

            turtlelib::Transform2D T_or(0);
            T_or = T_ow * T_wr; 

            turtlelib::Transform2D T_ro(0);
            T_ro = T_or.inv();

            turtlelib::Vector2D robot_o;
            robot_o = T_ow(robot_w);

            //object frame

            turtlelib::Vector2D v1_o;
            v1_o = T_or(v1_r);

            double x1 = v1_o.x;
            double y1 = v1_o.y;

            turtlelib::Vector2D v2_o;
            v2_o = T_or(v2_r);

            double x2 = v2_o.x;
            double y2 = v2_o.y;

            double D = x1*y2 - x2*y1;
            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = sqrt(pow(dx,2)+pow(dy,2));
            //ROS_WARN("range: %3.2f",range);
            //ROS_WARN("dr: %3.2f",dr);

            int sgn = 1;
            if (dy < 0){
                sgn = -1;
            }
        
            // Check for intersection with object
            if ((pow(.25/2,2)*pow(dr,2)-pow(D,2))>=0){
                double int_x_plus = (D*dy + sgn*dx*sqrt((pow(r,2)*pow(dr,2))-pow(D,2)))/pow(dr,2);
                double int_x_minus = (D*dy - sgn*dx*sqrt((pow(r,2)*pow(dr,2))-pow(D,2)))/pow(dr,2);          
                double int_y_plus = (-1*D*dx + abs(dy)*sqrt((pow(r,2)*pow(dr,2))-pow(D,2)))/pow(dr,2);
                double int_y_minus = (-1*D*dx - abs(dy)*sqrt((pow(r,2)*pow(dr,2))-pow(D,2)))/pow(dr,2);
            

                //Calculate the 4 possible vectors
                turtlelib::Vector2D r1 = {.x = int_x_plus, .y = int_y_plus};
                turtlelib::Vector2D r2 = {.x = int_x_minus, .y = int_y_minus};
                // turtlelib::Vector2D r3 = {.x = int_x_minus, .y = int_y_plus};
                // turtlelib::Vector2D r4 = {.x = int_x_minus, .y = int_y_minus};
                
                //Transform to robot frame
                r1 = T_ro(r1);
                r2 = T_ro(r2);
                // r3 = T_ro(r3);
                // r4 = T_ro(r4);

                //Consider the intersection point closest to the robot frame
                double m1 = sqrt(pow(r1.x,2)+pow(r1.y,2));
                double m2 = sqrt(pow(r2.x,2)+pow(r2.y,2));
                // double m3 = sqrt(pow(r3.x,2)+pow(r3.y,2));
                // double m4 = sqrt(pow(r4.x,2)+pow(r4.y,2));

                double x_min = std::min({x1,x2});
                double x_max = std::max({x1,x2});
                double y_min = std::min({y1,y2});
                double y_max = std::max({y1,y2});
                if ((int_x_plus > x_min) & (int_x_minus > x_min) & (int_x_minus < x_max) & (int_x_plus < x_max) & (int_y_plus > y_min) & (int_y_minus > y_min) & (int_y_minus < y_max) & (int_y_plus < y_max)){
                    if ((std::min({m1,m2}) < laser_hits[j]) || (laser_hits[j]==0)){
                        laser_hits[j] = std::min({m1,m2}); 
                    }
                }
                else if ((int_x_plus > x_min) & (int_x_plus < x_max) & (int_y_plus > y_min) & (int_y_plus < y_max)){
                    if ((m1 < laser_hits[j]) || (laser_hits[j]==0)){
                        laser_hits[j] = m1; 
                    }
                }
                else if ((int_x_minus > x_min) & (int_x_minus < x_max) & (int_y_minus > y_min) & (int_y_minus < y_max)){
                    if ((m2 < laser_hits[j]) || (laser_hits[j]==0)){
                        laser_hits[j] = m2;
                    } 
                }
            }
        }    
    j++;  
    } 
    laser_msg.ranges = laser_hits;        
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "nusim");
    
    ros::NodeHandle nh("~"), pub_nh;
    
    pub_nh.param("rate", rate, 500);
    ros::Rate r(rate);

    ts.data = 0;

    tf2_ros::TransformBroadcaster broadcaster;

    ts_pub = nh.advertise<std_msgs::UInt64>("timestep", 10);
    // js_pub = pub_nh.advertise<sensor_msgs::JointState>("red/joint_states", rate);
    obj_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    arena_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    sensor_pub = pub_nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 10);
    fake_sensor_pub = pub_nh.advertise<visualization_msgs::MarkerArray>("/fake_sensor", 10);
    laser_pub = pub_nh.advertise<sensor_msgs::LaserScan>("laser_scan",10);
    path_pub = pub_nh.advertise<nav_msgs::Path>("nav_msgs/Path",10);

    wheel_sub = pub_nh.subscribe("red/wheel_cmd", 10, update_wheel_position);

    ros::Timer timer_5Hz = nh.createTimer(ros::Duration(0.2), fake_sensor);

    rs_service = nh.advertiseService("Restart", restart);
    tp_service = nh.advertiseService("Teleport", teleport);

    nh.getParam("x0", x);
    nh.getParam("y0", y);
    nh.getParam("theta0", w);
    nh.getParam("x_length", x_length);
    nh.getParam("y_length", y_length);
    nh.getParam("wall_width", width);
    nh.getParam("/motor_cmd_to_radsec", mticks_radsec);
    nh.getParam("/encoder_ticks_to_rad", eticks_rad);
    nh.getParam("/slip_min",slip_min);
    nh.getParam("/slip_max",slip_max);
    nh.getParam("/collision_radius",collision_radius);
    nh.getParam("/min_range",min_range);
    nh.getParam("/max_range",max_range);
    nh.getParam("/angle_min",angle_min);
    nh.getParam("/angle_max",angle_max);
    nh.getParam("/angle_increment",angle_increment);

    nh.param<std::string>("odom_id",odom_id,"odom");

    radsec.resize(2);

    old_pos.theta = w;
    old_pos.x = x;
    old_pos.y = y;

    sensor_data.stamp = ros::Time::now();
    sensor_data.left_encoder = 0;
    sensor_data.right_encoder = 0;
    sensor_pub.publish(sensor_data);

    turtlelib::DiffDrive DD;

    wheels.header.stamp = ros::Time::now();
    wheels.name = {left_wheel, right_wheel};

    laser_msg.header.frame_id = "red_base_footprint";
    laser_msg.angle_min = angle_min;
    laser_msg.angle_max = angle_max;
    laser_msg.angle_increment = angle_increment;
    laser_msg.time_increment = 1/1800;
    laser_msg.scan_time = 1/5;
    laser_msg.range_min = min_range;
    laser_msg.range_max = max_range;

    while(ros::ok()) {
        
        ts.data += 1;
        ts_pub.publish(ts);

        obj_array = add_obstacles(obj_x_list, obj_y_list,obj_d_list);
        obj_pub.publish(obj_array);

        arena_array = make_arena(x_length,y_length);
        arena_pub.publish(arena_array);

        sensor_pub.publish(sensor_data);

        // twist = DD.get_twist(wheel_vels);

        wheels.header.stamp = ros::Time::now();
        wheels.position = {wheel_angles.L, wheel_angles.R};

        // js_pub.publish(wheels);

        if (teleporting == false){            
            pos = DD.get_q(theoretical_wheel_angles, old_wheel_angles, old_pos);
            // pos = DD.get_q(twist,old_pos);
        }

        for (unsigned int i = 0; i<obj_x_list.size(); i+=1) {
            double dx = pos.x - obj_x_list[i];
            double dy = pos.y - obj_y_list[i];
            double dist = sqrt(pow(dx,2)+pow(dy,2));
            double dif = dist - (collision_radius + obj_d_list[i]/2);
            if (dif < 0){
               pos = old_pos;
            }
        }
        
        sim_tf.header.stamp = ros::Time::now();
        sim_tf.header.frame_id = "world";
        sim_tf.child_frame_id = "red_base_footprint";
        sim_tf.transform.translation.x = pos.x;
        sim_tf.transform.translation.y = pos.y;
        sim_tf.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, pos.theta);
        sim_tf.transform.rotation.x = q.x();
        sim_tf.transform.rotation.y = q.y();
        sim_tf.transform.rotation.z = q.z();
        sim_tf.transform.rotation.w = q.w();

        broadcaster.sendTransform(sim_tf);
        
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pos.x;
        pose.pose.position.y = pos.y;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        path_msg.header.frame_id = "world";
        path_msg.header.stamp = ros::Time::now();
        path_msg.poses.push_back(pose);

        path_pub.publish(path_msg);

        //fake_sensor_array = fake_sensor(pos, obj_x_list, obj_y_list);
        

        laser_scan(pos, obj_x_list, obj_y_list);
        laser_pub.publish(laser_msg);

        nh.getParam("/obj_x",obj_x_list);
        nh.getParam("/obj_y",obj_y_list);
        nh.getParam("/obj_d",obj_d_list);

        old_wheel_angles = theoretical_wheel_angles;
        old_pos = pos; 

        teleporting = false;

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}