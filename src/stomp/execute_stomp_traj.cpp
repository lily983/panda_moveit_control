#include "include/stomp/execute_stomp_traj.h"

ExecuteStompTraj::ExecuteStompTraj(ros::NodeHandle& n) : n_(n) {
    n_.getParam("stomp_trajectory_topic", stomp_traj_topic_);
    ROS_INFO("Subscribed to stomp trjaectory topic which name is %s",
             stomp_traj_topic_.c_str());
    sub_trajectory_ = n_.subscribe(stomp_traj_topic_, 1,
                                   &ExecuteStompTraj::CallbackStompTraj, this);
    ROS_INFO("Subscribe to STOMP trojectory in topic named %s",
             stomp_traj_topic_.c_str());

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
