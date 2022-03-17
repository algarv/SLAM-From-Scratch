#include <ros/ros.h>
#include <string>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <armadillo>
#include <cmath> 
#include "measurement/measurement.hpp"


ros::Publisher obstacle_pub;
ros::Publisher sensor_pub;

std::vector<measurement::cluster> found_clusters;
std::vector<measurement::circle> confirmed_circles;
measurement::point saved_pt;

void get_clusters(const sensor_msgs::LaserScan &scan_data){

    found_clusters.clear();
    int id = 0;
    for(double angle = scan_data.angle_min; angle<scan_data.angle_max; angle+=scan_data.angle_increment){
        
        measurement::point new_pt = {.x = scan_data.ranges[id]*cos(angle), .y = scan_data.ranges[id]*sin(angle)};
        // ROS_WARN("New Pt (#%d): (%3.2f,%3.2f)",id,new_pt.x, new_pt.y);
        if (id == 0){
            measurement::cluster new_cluster;
            new_cluster.pt.push_back(new_pt);
            found_clusters.push_back(new_cluster);
            // ROS_WARN("Making new cluster %d",found_clusters.size());

            saved_pt = new_pt;
        }
        else{
            double dx = saved_pt.x - new_pt.x;
            double dy = saved_pt.y - new_pt.y;
            double d = sqrt(pow(dx,2)+pow(dy,2));

            if (d < threshold){
                int index = found_clusters.size() - 1;
                found_clusters[index].pt.push_back(new_pt);
                // ROS_WARN("Adding pt to cluster %d",found_clusters.size());
            }
            else{
                measurement::cluster new_cluster;
                new_cluster.pt.push_back(new_pt);
                found_clusters.push_back(new_cluster);
                // ROS_WARN("Making new cluster %d",found_clusters.size());
            }
            saved_pt = new_pt;
        }
        id ++;
    }
    
    measurement::point first_pt = {.x = scan_data.ranges[0]*cos(scan_data.angle_min), .y = scan_data.ranges[0]*sin(scan_data.angle_min)};
    measurement::point last_pt = {.x = scan_data.ranges[id - 1]*cos(scan_data.angle_max), .y = scan_data.ranges[id - 1]*sin(scan_data.angle_max)};

    double dx = first_pt.x - last_pt.x;
    double dy = first_pt.y - last_pt.y;
    double d = sqrt(pow(dx,2)+pow(dy,2));

    if (d<threshold){
        int index = found_clusters.size() - 1;
        for(long unsigned int i = 0; i < found_clusters[index].pt.size(); i++){
            // ROS_WARN("Adding pt to cluster %d",0);
            found_clusters[0].pt.push_back(found_clusters[index].pt[i]);
        }
        found_clusters.erase(found_clusters.begin()+index);
    }
    // ROS_WARN("Clustered Successfully");
}

void publish_landmarks(const ros::TimerEvent&){
    int id = 0;
    static visualization_msgs::MarkerArray obstacle;
    obstacle.markers.resize(confirmed_circles.size());
    for (unsigned int i = 0; i<confirmed_circles.size(); i+=1) {
        
        obstacle.markers[i].header.frame_id = "red_base_scan";
        obstacle.markers[i].ns = "nusim_node";
        obstacle.markers[i].header.stamp = ros::Time::now();
        obstacle.markers[i].type = visualization_msgs::Marker::CYLINDER;
        obstacle.markers[i].action = visualization_msgs::Marker::ADD;
        obstacle.markers[i].id = id;
        obstacle.markers[i].pose.position.x = confirmed_circles[i].a;
        obstacle.markers[i].pose.position.y = confirmed_circles[i].b;
        obstacle.markers[i].pose.position.z = 0;
        obstacle.markers[i].pose.orientation.x = 0.0;
        obstacle.markers[i].pose.orientation.y = 0.0;
        obstacle.markers[i].pose.orientation.z = 0.0;
        obstacle.markers[i].pose.orientation.w = 1.0;
        obstacle.markers[i].scale.x = 2 * std::sqrt(confirmed_circles[i].R2);
        obstacle.markers[i].scale.y = 2 * std::sqrt(confirmed_circles[i].R2);
        obstacle.markers[i].scale.z = .25;
        obstacle.markers[i].color.a = 1.0;
        obstacle.markers[i].color.r = 1.0;
        obstacle.markers[i].color.g = 0.0;
        obstacle.markers[i].color.b = 1.0;
        // obstacle.markers[i].lifetime = ros::Duration();
        id += 1;
    }

    obstacle_pub.publish(obstacle);
    sensor_pub.publish(obstacle);
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh("~"), pub_nh;

    double rate;
    nh.getParam("/rate", rate);
    ros::Rate r(rate);

    ros::Timer timer_5Hz = nh.createTimer(ros::Duration(0.2), publish_landmarks);
    ros::Subscriber sensor_sub = pub_nh.subscribe("laser_scan", 10, get_clusters);
    obstacle_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    sensor_pub =  pub_nh.advertise<visualization_msgs::MarkerArray>("/sensor", 10);

    while(ros::ok()){

        measurement::clustering cluster;

        std::vector<measurement::circle> potential_circles = cluster.circle_detection(found_clusters);
        confirmed_circles = cluster.circle_classification(found_clusters);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}