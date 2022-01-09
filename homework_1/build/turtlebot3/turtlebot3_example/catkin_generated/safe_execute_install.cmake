execute_process(COMMAND "/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/build/turtlebot3/turtlebot3_example/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/build/turtlebot3/turtlebot3_example/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
