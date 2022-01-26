cmake_minimum_required(VERSION 3.9)
project(turtlelib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES turtlelib
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

# create the turtlelib library 
add_library(${PROJECT_NAME} src/rigid2d.cpp)
# The add_library function just added turtlelib as a "target"
# A "target" is a name that CMake uses to refer to some type of output
# In this case it is a library but it could also be an executable or some other items

# Public include directories can be used by other targets that link against turtlelib
# By adding include/ to the include path, this means that files in e.g., include/turtlelib
# can be included with #include"turtlelib/file.hpp"
target_include_directories (${PROJECT_NAME} PUBLIC include/)

# enable C++ 17
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17) 

# warnings are your friend!
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)

# create the executable target  and link it with the rigid2d library
# It is also possible specify multiple cpp files and they will be linked
# into a single executable (as long as exactly one of these files includes a main() function).
# However, by creating a library (as we are doing here) the library files
# can be compiled once and used
add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main ${PROJECT_NAME})

# Use the cmake testing functionality. A test is just an executable.
enable_testing()
add_executable(${PROJECT_NAME}_test tests/test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
add_test(NAME Test_of_${PROJECT_NAME} COMMAND ${PROJECT_NAME}_test)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS frame_main
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# CMake also has the ability to generate doxygen documentation
# find_package(Doxygen)
# set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
# doxygen_add_docs(doxygen include/ src/ README.md ALL)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)