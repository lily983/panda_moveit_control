#include "include/stomp/execute_stomp_traj.h"

ExecuteStompTraj::ExecuteStompTraj(ros::NodeHandle& n)
    : n_(n), planning_scene_(n), panda_arm_(0.05, 0.05) {
    n_.getParam("stomp_trajectory_topic", stomp_traj_topic_);
    ROS_INFO("Subscribing to stomp trjaectory topic which name is %s",
             stomp_traj_topic_.c_str());
    sub_trajectory_ = n_.subscribe(stomp_traj_topic_, 1,
                                   &ExecuteStompTraj::CallbackStompTraj, this);

    srv_execute_stomp_ = n_.advertiseService(
        "execute_stomp", &ExecuteStompTraj::CallbackExecuteStompTraj, this);

    ROS_INFO("[ExecuteStompTraj]: finished initialization!");
}

ExecuteStompTraj::~ExecuteStompTraj() {}

void ExecuteStompTraj::CallbackStompTraj(
    const moveit_msgs::DisplayTrajectoryPtr& msg) {
    all_stomp_trajectory_ = *msg;
}

moveit_msgs::RobotTrajectory ExecuteStompTraj::GetTargetStompTrajectory(
    const int num) {
    if (all_stomp_trajectory_.trajectory.size() == 0) {
        ROS_ERROR("!!!Haven't received any stomp trajectory!!!");
    }

    return all_stomp_trajectory_.trajectory.at(num);
}

bool ExecuteStompTraj::CallbackExecuteStompTraj(
    panda_moveit_control::ExecuteStompTraj::Request& req,
    panda_moveit_control::ExecuteStompTraj::Response& res) {
    ROS_INFO("Class execute_stomp_traj receive ExecuteStompTraj service call");
    return true;
}