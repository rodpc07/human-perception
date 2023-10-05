cmake_minimum_required(VERSION 3.0.2)
project(human_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    geometric_shapes
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    tf
    tf_conversions
    eigen_conversions
    std_msgs
    genmsg
    message_generation
    hri
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning_interface
    tf2
    tf2_geometry_msgs
    hri
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(human_perception src/human_perception.cpp)
target_link_libraries(human_perception ${catkin_LIBRARIES} ${Boost_LIBRARIES})