The kalman filter library is currently under construction and should not be used. The nuslam node, however, is fully functional and simulates a turtlebot navigating turtlebot using teleop keyboard control. Adding unknown_data_assoc incorporates LiDAR data with automatic data association for the turtlebot to localize itself in the map. 

        roslaunch nuslam unknown_data_assoc.launch

![SLAM Demo](Kalman_Filter.png)
Red: The state of the simulated real-world 

Yellow: The state according to the LiDAR sensor 

Blue: The state according to odometry 

Green: The state according to the kalman filter
