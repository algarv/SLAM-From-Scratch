#include <iostream>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

void restart(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){

    ts = 0

    // while ros(ok()){
    //     switch(state)
    //     {

    //     }
    // }

}

void obstacle(inputs, std_srvs::Empty::Response &response){

    visualization_msgs::Marker obstacle;

    obstacle.header.frame_id = "world";
    obstacle.header.stamp = ros::Time();
    obstacle.type = visualization_msgs::Marker::CYLINDER;
    obstacle.action = visualization_msgs::Marker::ADD;
    obstacle.pose.position.x = inputs.x;
    obstacle.pose.position.y = inputs.y;
    obstacle.pose.position.z = 0;
    obstacle.pose.orientation.x = 0.0;
    obstacle.pose.orientation.y = 0.0;
    obstacle.pose.orientation.z = 0.0;
    obstacle.pose.orientation.w = 1.0;
    marker.scale.x = inputs.d;
    marker.scale.y = inputs.d;
    marker.scale.z = .25;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = .75;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    obs_pub.publish( obstacle );

}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;
    std_msgs::UInt64.msg ts;

    int r = nh.getParam("~rate");
    int ts = 0
    nh.setParam("~timestep",ts);

    ros::ServiceServer service = nh.advertiseService("Restart", restart);
    ros::ServiceServer service = nh.advertiseService("Add_Obstacle",obstacle)
    sensor_msgs::JointState joint_states;

    ros::Publisher obs_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    return 0;
}