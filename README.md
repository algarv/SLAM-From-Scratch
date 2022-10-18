# EKF SLAM with Turtlebot3
* Anna Garverick
* Winter 2022

# Package List

- ## nuturtle_description ###
    Modifies the TurtleBot3 URDF file to spawn multiple turtlebots in RVIZ with color arguments

    <p align="center">
    <img src=nuturtle_description/img/all_robots.png width=250/>
    </p>

- ## nusim ##
    Spawns a turlebot with obstacles in RVIZ

    <p align="center">
    <img src=nusim/images/nusim_cropped.png width=250/>
    </p>

- ## nuturtle_control ##
    Interfaces commands with the real or simulated turtlebot

- ## nuslam ## 
    Maps the environment and localizes the turtlebot in the map frame using data association with LiDAR point cloud data and an extended kalman filter

    <p align="center">
    <img src=nuslam/img/Kalman_Filter.png width=500/>
    </p>
