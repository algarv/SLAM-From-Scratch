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

static int rate;
static bool teleporting = false;
static double x = 0, y = 0, w = 0, x_length = 0, y_length = 0, width = 0, left_rot_vel = 0.0, right_rot_vel = 0.0, mticks_radsec, eticks_rad, max_range, slip_min, slip_max, ada_L, ada_R;
static std::string left_wheel = "red_wheel_left_joint", right_wheel = "red_wheel_right_joint", odom_id;
std::vector<double> obj_x_list, obj_y_list, obj_d_list, radsec;
static std_msgs::UInt64 ts;
static sensor_msgs::JointState wheels;
static geometry_msgs::TransformStamped sim_tf;
static nuturtlebot_msgs::SensorData sensor_data; 
static ros::Publisher obj_pub, js_pub, ts_pub, arena_pub, sensor_pub, fake_sensor_pub;
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
    arena_array.markers[1].id = i+1;
    arena_array.markers[1].pose.position.x = 0;
    arena_array.markers[1].pose.position.y = y_length/2;
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
    arena_array.markers[3].id = i+3;
    arena_array.markers[3].pose.position.x = 0;
    arena_array.markers[3].pose.position.y = -y_length/2;
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

visualization_msgs::MarkerArray fake_sensor(turtlelib::q robot_pos, std::vector<double> obj_x_list, std::vector<double> obj_y_list){

    int id = 0;

    fake_sensor_array.markers.resize(obj_x_list.size());

    std::normal_distribution<> noise(0, .01);

    for (unsigned int i = 0; i<obj_x_list.size(); i+=1) {
        
        double noise_x = noise(get_random());
        double noise_y = noise(get_random());

        turtlelib::Vector2D trans_wr;
        trans_wr.x = robot_pos.x;
        trans_wr.y = robot_pos.y;

        turtlelib::Transform2D T_wr(trans_wr, robot_pos.theta);
        turtlelib::Transform2D T_rw(0);
        T_rw = T_wr.inv();

        turtlelib::Vector2D obstacle_world;
        obstacle_world.x = obj_x_list[i];
        obstacle_world.y = obj_y_list[i];

        turtlelib::Vector2D obstacle_robot = T_rw(obstacle_world);
        ros::Duration duration(1.0);

        double d = sqrt(pow(robot_pos.x-obj_x_list[i],2) + pow(robot_pos.y-obj_y_list[i],2));
        ROS_WARN("Distance %d: %6.2f",i,d);
        if (d > max_range) {
            fake_sensor_array.markers[i].action = visualization_msgs::Marker::DELETE;
            ROS_WARN("Deleting %d",i);
        }
        else {
            fake_sensor_array.markers[i].action = visualization_msgs::Marker::ADD;
        }

        fake_sensor_array.markers[i].header.frame_id = "red_base_footprint";
        fake_sensor_array.markers[i].ns = "nusim_node";
        fake_sensor_array.markers[i].header.stamp = ros::Time::now();
        fake_sensor_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
        fake_sensor_array.markers[i].id = id;
        fake_sensor_array.markers[i].pose.position.x = obstacle_robot.x + noise_x;
        fake_sensor_array.markers[i].pose.position.y = obstacle_robot.y + noise_y;
        fake_sensor_array.markers[i].pose.position.z = .125;
        fake_sensor_array.markers[i].pose.orientation.x = 0.0;
        fake_sensor_array.markers[i].pose.orientation.y = 0.0;
        fake_sensor_array.markers[i].pose.orientation.z = 0.0;
        fake_sensor_array.markers[i].pose.orientation.w = 1.0;
        fake_sensor_array.markers[i].scale.x = obj_d_list[i];
        fake_sensor_array.markers[i].scale.y = obj_d_list[i];
        fake_sensor_array.markers[i].scale.z = .25;
        fake_sensor_array.markers[i].color.a = 1.0;
        fake_sensor_array.markers[i].color.r = 0.0;
        fake_sensor_array.markers[i].color.g = 1.0;
        fake_sensor_array.markers[i].color.b = 0.0;
        fake_sensor_array.markers[i].lifetime = duration;
        fake_sensor_array.markers[i].frame_locked = true;
        id += 1;
    }

    return fake_sensor_array;
}

void update_wheel_position(const nuturtlebot_msgs::WheelCommands::ConstPtr &wheel_cmd){

    double L_cmd = wheel_cmd->left_velocity;
    double R_cmd = wheel_cmd->right_velocity;

    left_rot_vel = L_cmd * mticks_radsec;
    right_rot_vel = R_cmd * mticks_radsec;
    
    double mean_L = left_rot_vel;
    double mean_R = right_rot_vel;
    double std = .1;

    std::normal_distribution<> L(mean_L, std);
    std::normal_distribution<> R(mean_R, std);

    wheel_vels.L = L(get_random());
    wheel_vels.R = R(get_random());

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

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "nusim");
    
    ros::NodeHandle nh("~"), pub_nh;
    
    pub_nh.param("rate", rate, 500);
    ros::Rate r(rate);

    ts.data = 0;

    tf2_ros::TransformBroadcaster broadcaster;

    ts_pub = nh.advertise<std_msgs::UInt64>("timestep", rate);
    // js_pub = pub_nh.advertise<sensor_msgs::JointState>("red/joint_states", rate);
    obj_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", rate);
    arena_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", rate);
    sensor_pub = pub_nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data", rate);
    fake_sensor_pub = nh.advertise<visualization_msgs::MarkerArray>("/fake_sensor", 5);

    wheel_sub = pub_nh.subscribe("red/wheel_cmd", 10, update_wheel_position);

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
    nh.getParam("/max_range",max_range);
    nh.getParam("/slip_min",slip_min);
    nh.getParam("/slip_max",slip_max);

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
        
        fake_sensor_array = fake_sensor(pos, obj_x_list, obj_y_list);
        fake_sensor_pub.publish(fake_sensor_array);

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