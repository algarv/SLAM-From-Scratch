The nuslam node simulates a turtlebot navigating using teleop keyboard control. Adding unknown_data_assoc incorporates LiDAR data with data association for the turtlebot to localize itself in the map. 

To run the simulation with known landmark locations (easy mode):

        roslaunch nuslam slam.launch

To run the simulation with simuated LiDAR data and data association (unknown landmark associaion -- hard mode):

        roslaunch nuslam unknown_data_assoc.launch

#### EKF SLAM With Fake Sensor Data (known landmark locations, no data association)
![SLAM Demo](img/Kalman_Filter.png)


Red: The state of the simulated real-world 

Yellow: The state according to the fake sensor data

Blue: The state according to odometry 

Green: The state according to the kalman filter

#### EKF SLAM With Realistic Sensor Data (initially unknown landmark locations)

![SLAM Demo](img/data_assoc.png)

Red: The state of the simulated real-world 

Yellow: The state according to the clustered and circle-fit LiDAR sensor data

Blue: The state according to odometry 

Green: The state according to EKF SLAM

|Robot|Starting Location|Ending Location|Error| 
|---------|----------|---------|-----|
|Red| (0,0) | (0.0670, 0.0091) | (0,0)
|Green| (0,0) | (0.06989, 0.00905) | (.00198,-0.00005)
|Blue| (0,0) | (0.1175,-0.00164) | (0.0505, -0.01074)

EKF SLAM outperformed odometry! 

![SLAM Demo](img/final_demo.gif)
