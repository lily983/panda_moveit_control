#pragma once

#include "include/utils/data_parse.h"

#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <eigen3/Eigen/Core>

typedef Eigen::Transform<double, 3, Eigen::Affine> T;

/**
 * \brief Class for subscribe STOMP planned trajectories and publish target
 * execution trajectory. All STOMP tasks(scooping, pouring, transporting,
 * openning) will use this class.
 *
 */
class ExecuteStompTraj {
  public:
    ExecuteStompTraj(ros::NodeHandle& n);
    ~ExecuteStompTraj();

    /**
     * \brief Get the num-th trajectory from the moveit_msgs::DisplayTrajectory
     * published by stomp
     * \param num the num-th trajectory to execute
     */
    moveit_msgs::RobotTrajectory GetTargetStompTrajectory(int num);

  protected:
    /**
     * \brief Receive and store moveit_msgs::DisplayTrajectory
     * published by stomp
     */
    void CallbackStompTraj(const moveit_msgs::DisplayTrajectoryPtr& msg);

    ros::NodeHandle n_;
    std::string stomp_traj_topic_;
    moveit_msgs::DisplayTrajectory all_stomp_trajectory_;
    ros::Subscriber sub_trajectory_;
};
