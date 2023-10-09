#pragma once

#include "include/stomp/data_parse.h"

#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <eigen3/Eigen/Core>

#include "panda_moveit_control/AddCollisionMesh.h"
#include "panda_moveit_control/RemoveCollisionMesh.h"

typedef Eigen::Transform<double, 3, Eigen::Affine> T;

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
    void StoreMeshToRepo();

    /**
     * \brief Receive and store moveit_msgs::DisplayTrajectory
     * published by stomp
     *
     * \param msg
     */
    void CallbackStompTraj(const moveit_msgs::DisplayTrajectoryPtr& msg);

    ros::NodeHandle n_;
    std::string web_mesh_path_;
    std::string repo_mesh_path_;
    std::string stomp_traj_topic_;
    moveit_msgs::DisplayTrajectory all_stomp_trajectory_;
    ros::Subscriber sub_trajectory_;

  public:
    ros::ServiceClient client_add_collision_mesh_;
    ros::ServiceClient client_remove_collision_mesh_;
};
