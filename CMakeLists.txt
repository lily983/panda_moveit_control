cmake_minimum_required(VERSION 3.0.2)
project(panda_moveit_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  std_msgs
  tf2
  trajectory_msgs
  eigen_conversions
  actionlib
  franka_gripper
  geometric_shapes
  sound_play
)

## System dependencies are found with CMake's conventions
find_package(Assimp REQUIRED all)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PandaGraspHandle.srv
  GraspHandleInfo.srv
  DisplayStompTraj.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  moveit_msgs   std_msgs   trajectory_msgs
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
 INCLUDE_DIRS include
#  LIBRARIES panda_moveit_control
#  CATKIN_DEPENDS franka_ros joint_state_publisher message_generation moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools robot_state_publis roscpp rospy std_msgs tf2 trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_planning_scene
  src/planning_scene/planning_scene_operation.cc
)
target_link_libraries(${PROJECT_NAME}_planning_scene
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_robot 
  src/robot/panda_arm_planning_control.cc
  include/robot/panda_gripper_control.h
)
target_link_libraries(${PROJECT_NAME}_robot
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(panda_grasp_handle_server src/panda_grasp_handle_server.cpp)
target_link_libraries(panda_grasp_handle_server
  ${catkin_LIBRARIES}
  ${Assimp_LIBRARIES}
  ${PROJECT_NAME}_robot
)
