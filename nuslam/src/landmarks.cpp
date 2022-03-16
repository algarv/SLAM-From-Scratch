#include <ros/ros.h>
#include <string>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include "nuturtle_control/set_pose.h"
#include <visualization_msgs/MarkerArray.h>
#include <armadillo>
#include <cmath> 

#define threshold .05

struct point{
    double x;
    double y;
};

struct cluster{
    std::vector<point> pt;
};

struct circle{
    double a;
    double b;
    double R2;
};

std::vector<cluster> clusters;
std::vector<point> centroids;
std::vector<double> z_bar;
std::vector<circle> circles;
std::vector<circle> confirmed_circles;

arma::mat M(4,4);
arma::mat H(4,4);
arma::mat H_inv(4,4);

point saved_pt;
void get_clusters(const sensor_msgs::LaserScan &scan_data){

    int id = 0;
    for(double angle = scan_data.angle_min; angle<scan_data.angle_max; angle+=scan_data.angle_increment){
        
        point new_pt = {.x = scan_data.ranges[id]*cos(angle), .y = scan_data.ranges[id]*sin(angle)};
        ROS_WARN("New Pt (#%d): (%3.2f,%3.2f)",id,new_pt.x, new_pt.y);
        if (id == 0){
            cluster new_cluster;
            new_cluster.pt.push_back(new_pt);
            clusters.push_back(new_cluster);
            ROS_WARN("Making new cluster %d",clusters.size());

            saved_pt = new_pt;
        }
        else{
            double dx = saved_pt.x - new_pt.x;
            double dy = saved_pt.y - new_pt.y;
            double d = sqrt(pow(dx,2)+pow(dy,2));

            if (d < threshold){
                int index = clusters.size() - 1;
                clusters[index].pt.push_back(new_pt);
                ROS_WARN("Adding pt to cluster %d",clusters.size());
            }
            else{
                cluster new_cluster;
                new_cluster.pt.push_back(new_pt);
                clusters.push_back(new_cluster);
                ROS_WARN("Making new cluster %d",clusters.size());
            }
            saved_pt = new_pt;
        }
        id ++;
    }
    
    point first_pt = {.x = scan_data.ranges[0]*cos(scan_data.angle_min), .y = scan_data.ranges[0]*sin(scan_data.angle_min)};
    point last_pt = {.x = scan_data.ranges[id - 1]*cos(scan_data.angle_max), .y = scan_data.ranges[id - 1]*sin(scan_data.angle_max)};

    double dx = first_pt.x - last_pt.x;
    double dy = first_pt.y - last_pt.y;
    double d = sqrt(pow(dx,2)+pow(dy,2));

    if (d<threshold){
        int index = clusters.size() - 1;
        for(long unsigned int i = 0; i < clusters[index].pt.size(); i++){
            ROS_WARN("Adding pt to cluster %d",0);
            clusters[0].pt.push_back(clusters[index].pt[i]);
        }
        clusters.erase(clusters.begin()+index);
    }
    ROS_WARN("Clustered Successfully");
}

void circle_detection(std::vector<cluster> clusters){
    
    int size = clusters.size();
    centroids.resize(size);
    circles.resize(size);
    z_bar.resize(size);

    ROS_WARN("Detecting circles from %d clusters", size);
    for (unsigned long int i=0; i < clusters.size(); i++){
        double cluster_size = clusters[i].pt.size();
        if (cluster_size>3 && ((clusters[i].pt[0].x > 0)||(clusters[i].pt[0].y > 0))){
            ROS_WARN("Cluster %d is valid. Size: %d", i, cluster_size);
            double sum_x = 0;
            double sum_y = 0;
            double sum_z = 0;

            for (unsigned long int j=0; j < cluster_size; j++){
                sum_x += (clusters[i].pt[j].x);
                sum_y += (clusters[i].pt[j].y);
            }
            
            centroids[i] = {.x = sum_x/cluster_size, .y = sum_y/cluster_size};
            ROS_WARN("Found cluster centroid: (%3.2f,%3.2f)", centroids[i].x, centroids[i].y);

            arma::mat Z (cluster_size,4, arma::fill::zeros);
            for (unsigned long int j=0; j < cluster_size; j++){       
                double x = clusters[i].pt[j].x - centroids[i].x;
                double y = clusters[i].pt[j].y - centroids[i].y;
                double z = pow((x),2)+ pow((y),2);
                sum_z += z;
                Z(j,0) = z;
                Z(j,1) = x;
                Z(j,2) = y;
                Z(j,3) = 1; 
            }
            ROS_WARN("Z: ");
            Z.print(std::cout);
            
            z_bar[i] = sum_z/cluster_size;
        
            arma::mat Z_T = Z.t();
            M = (Z_T * Z)/cluster_size;
            ROS_WARN("M: ");
            M.print(std::cout);

            H = arma::zeros(4,4);
            H(0,0) = 8 * z_bar[i];
            H(1,1) = 1;
            H(2,2) = 1;
            H(0,3) = 2;
            H(3,0) = 2;
            ROS_WARN("H: ");
            H.print(std::cout);

            H_inv = arma::zeros(4,4);
            H_inv(0,3) = 0.5;
            H_inv(1,1) = 1;
            H_inv(2,2) = 1;
            H_inv(3,0) = 0.5;
            H_inv(3,3) = -2 * z_bar[i];
            ROS_WARN("H_inv: ");
            H.print(std::cout);

            arma::mat U;
            arma::vec S_vec;
            arma::mat V;

            arma::svd(U, S_vec, V, Z);
            ROS_WARN("U: ");
            U.print(std::cout);
            ROS_WARN("S_vec: ");
            S_vec.print(std::cout);
            ROS_WARN("V: ");
            V.print(std::cout);

            arma::mat S_mat;
            S_mat = arma::zeros(S_vec.n_rows,S_vec.n_rows);
            float s;
            for (int i=0; i < S_vec.n_rows; i++){
                S_mat(i,i) = S_vec(i);
                if (i==0){
                    s = S_vec(i);
                }
                else{
                    if (S_mat(i,i) < s){
                        s = S_vec(i);
                    }
                }
            }
            ROS_WARN("S_mat: ");
            S_mat.print(std::cout);

            ROS_WARN("s: %3.2f", s);

            arma::vec A(4);
            ROS_WARN("s < %f",pow(10,-12));
            if (s<pow(10,-12)){
                A = V.col(3);

            }
            else{
                arma::mat Y;
                Y = V * S_mat * V.t();

                arma::mat Q;
                Q = Y * H.i() * Y;

                arma::colvec eigval;
                arma::mat eigvec;

                arma::eig_sym(eigval, eigvec, Q);
                ROS_WARN("Eigvec: ");
                eigvec.print(std::cout);
                ROS_WARN("Eigval: ");
                eigval.print(std::cout);

                int min_index;
                for (int i=0; i<eigval.n_rows; i++){
                    if (i == 0){
                        min_index = i;
                    }
                    else{
                        if (eigval(i) < eigval(min_index)){
                            min_index = i;
                        }
                    }
                }
                arma::vec e;
                e = eigvec.col(min_index);
                A = Y.i() * e;
            }
            ROS_WARN("A: ");
            A.print(std::cout);

            circles[i].a = (-1*A(1,0) / (2 * A(0,0))) + centroids[i].x;
            circles[i].b = (-1*A(2,0) / (2 * A(0,0))) + centroids[i].y;
            circles[i].R2 = (pow(A(1,0),2)+pow(A(2,0),2)-4*A(0,0)*A(3,0))/(4*pow(A(0,0),2));
        }
    }

}

void circle_classification(std::vector<cluster> clusters){

    ROS_WARN("Unfiltered Cluster List Size: %d", clusters.size());
    for (unsigned long int i=0; i < clusters.size(); i++){
        if (clusters[i].pt.size()>3){
            double angles[clusters[i].pt.size() - 2]; 
            double angle_sum = 0;
            point p1 = clusters[i].pt[0];
            point p2 = clusters[i].pt[clusters[i].pt.size() - 1];
            ROS_WARN("Point 1: (%3.2f,%3.2f), Point 2: (%3.2f,%3.2f)", p1.x, p1.y, p2.x, p2.y);

            for (unsigned long int j=1; j < clusters[i].pt.size() - 1; j++){
                point p3 = clusters[i].pt[j];
                ROS_WARN("Point 3: (%3.2f,%3.2f)", p3);

                point a = {.x = p1.x - p3.x, .y = p1.y - p3.y};
                point b = {.x = p2.x - p3.x, .y = p2.y - p3.y};
                angles[j-1] = (a.x * b.x + a.y * b.y) / (sqrt(pow(a.x,2)+pow(a.y,2)) * sqrt(pow(b.x,2)+pow(b.y,2)));
                angle_sum += angles[j-1];
            }

            double mean_angle = angle_sum/(clusters[i].pt.size() - 2);
            double angle_StD = 0;
            ROS_WARN("Mean Cluster Angles: %3.2f",mean_angle);
            for (unsigned long int j=0; j < clusters[i].pt.size() - 2; j++){
                angle_StD += pow((angles[i] - angle_StD),2);
            }
            angle_StD /= clusters[i].pt.size() - 2;
            ROS_WARN("STD: %3.2f",angle_StD);
            if (angle_StD < .15 && mean_angle > 90 && mean_angle < 135){
                confirmed_circles.push_back(circles[i]);
                ROS_WARN("Confirmed circle %d", i);
            }
        } 
    }

}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh("~"), pub_nh;

    double rate;
    nh.getParam("/rate", rate);
    ros::Rate r(rate);

    ros::Subscriber sensor_sub = pub_nh.subscribe("laser_scan", 10, get_clusters);
    ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    while(ros::ok()){

        circle_detection(clusters);
        circle_classification(clusters);
    

        int id = 0;
        static visualization_msgs::MarkerArray obstacle;
        obstacle.markers.resize(confirmed_circles.size());
        for (unsigned int i = 0; i<confirmed_circles.size(); i+=1) {
            
            obstacle.markers[i].header.frame_id = "world";
            obstacle.markers[i].ns = "nusim_node";
            obstacle.markers[i].header.stamp = ros::Time::now();
            obstacle.markers[i].type = visualization_msgs::Marker::CYLINDER;
            obstacle.markers[i].action = visualization_msgs::Marker::ADD;
            obstacle.markers[i].id = id;
            obstacle.markers[i].pose.position.x = confirmed_circles[i].a;
            obstacle.markers[i].pose.position.y = confirmed_circles[i].b;
            obstacle.markers[i].pose.position.z = .125;
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

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}