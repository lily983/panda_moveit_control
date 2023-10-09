cmake_minimum_required(VERSION 3.0.2)
project(panda_moveit_control)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
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

# # Generate services in the 'srv' folder
add_service_files(
  FILES
  PandaGraspHandle.srv
  GraspHandleInfo.srv
  DisplayStompTraj.srv
  ExecuteStompTraj.srv
  VisualizeStompTraj.srv
  AddCollisionMesh.srv
  RemoveCollisionMesh.srv
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  moveit_msgs std_msgs trajectory_msgs
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include

  # LIBRARIES panda_moveit_control
  # CATKIN_DEPENDS franka_gripper
  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}_planning_scene_operation src/planning_scene/planning_scene_operation.cc)
target_link_libraries(${PROJECT_NAME}_planning_scene_operation ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_robot src/robot/panda_arm_planning_control.cc src/robot/panda_gripper_control.cc)
target_link_libraries(${PROJECT_NAME}_robot ${catkin_LIBRARIES})

add_library(ExecuteStompTraj src/stomp/execute_stomp_traj.cc)
target_link_libraries(ExecuteStompTraj ${PROJECT_NAME}_robot
                                       ${PROJECT_NAME}_planning_scene_operation)

# add_library(Pouring src/stomp/pouring.cc)
# target_link_libraries(Pouring ExecuteStompTraj)

# add_library(Transporting src/stomp/transporting.cc)
# target_link_libraries(Transporting ExecuteStompTraj)

# add_library(Scooping src/stomp/scooping.cc)
# target_link_libraries(Scooping ExecuteStompTraj)

# add_library(OpeningSliding src/stomp/opening_sliding.cc)
# target_link_libraries(OpeningSliding ExecuteStompTraj)

# add_library(OpeningRotating src/stomp/opening_rotating.cc)
# target_link_libraries(OpeningRotating ExecuteStompTraj)

# # Declare a C++ executable
add_executable(panda_grasp_handle_server src/panda_grasp_handle_server.cpp)
target_link_libraries(panda_grasp_handle_server ${PROJECT_NAME}_robot)

add_executable(operate_planning_scene src/operate_planning_scene.cpp)
target_link_libraries(operate_planning_scene ${PROJECT_NAME}_robot
                                             ${PROJECT_NAME}_planning_scene_operation)

add_executable(test_panda_class src/test_panda_class.cpp)
target_link_libraries(test_panda_class ${PROJECT_NAME}_robot)
