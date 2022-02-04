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
#include "nuturtle_control/control.h"
#include <std_srvs/Empty.h>

static double v, r;
bool stopped;

void control(nuturtle_control::control::Request &params, nuturtle_control::control::Return &return){

    v = params.velocity;
    r = params.radius;

}

void reverse(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){
    v = -1 * v;
}

void stop(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){
    stopped = true;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "circle");
    ros::NodeHandle nh("~"), pub_nh;

    int rate;
    nh.getParam("rate", rate);
    ros::Rate r(rate);

    ros::ServiceServer control_service = nh.advertiseService("control",control);
    ros::ServiceServer reverse_service = nh.advertiseService("reverse",reverse);
    ros::ServiceServer stop_service = nh.advertiseService("stop",stop);

    ros::Publisher cmd_vel_pub = pub_nh.advertise<geometry_msgs::Twist>("red/wheel_cmd", 100);
    stopped = false;

    while(stopped == false){
        turtlelib::Twist2D twist_cmd{.vx = v, .vy = 0, .w = v/r};
        ros::spinOnce();
        r.sleep();
    }

    turtlelib::Twist2D twist_cmd{.vx = 0, .vy = 0, .w = 0};
    cmd_vel_pub.publish(twist_cmd);

    }
    }

    return 0;
}