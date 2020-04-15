cmake_minimum_required(VERSION 2.8.3)
project(odometry_robot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)


## Declare ROS messages and services
# add_message_files(FILES Message1.msg Message2.msg)
# add_service_files(FILES Service1.srv Service2.srv)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

## Declare catkin package
catkin_package()

## Specify additional locations of header FILES
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(OdometryRobot src/OdomRobot.cpp src/run_odometry_robot.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(OdometryRobot ${catkin_LIBRARIES})