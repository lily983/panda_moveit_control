#pragma once

#include "include/utils/data_parser.h"

#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <eigen3/Eigen/Core>

#include <panda_moveit_control/ExecuteStompTraj.h>

#include <include/planning_scene/planning_scene_operation.h>
#include <include/robot/panda_arm_planning_control.h>
#include <include/robot/panda_gripper_control.h>

typedef Eigen::Transform<double, 3, Eigen::Affine> T;

/**
 * \brief Class for subscribe STOMP planned trajectories and publish target
 * execution trajectory. All STOMP tasks(scooping, pouring, transporting,
 * openning) will be derived from this class.
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
    moveit_msgs::RobotTrajectory GetTargetStompTrajectory(const int num);

  protected:
    /**
     * \brief Receive and store moveit_msgs::DisplayTrajectory
     * published by stomp
     */
    void CallbackStompTraj(const moveit_msgs::DisplayTrajectoryPtr& msg);

    /** \brief Callback function of service ExecuteStompTraj*/
    virtual bool CallbackExecuteStompTraj(
        panda_moveit_control::ExecuteStompTraj::Request& req,
        panda_moveit_control::ExecuteStompTraj::Response& res);

    ros::NodeHandle n_;

    /** \brief Name of stomp planned trajectories*/
    std::string stomp_traj_topic_;

    /** \brief Received stomp trajectories*/
    moveit_msgs::DisplayTrajectory all_stomp_trajectory_;

    /** \brief Stomp trajectories subscriber*/
    ros::Subscriber sub_trajectory_;

    /** \brief Specified n-th trajectory in stomp trajectories to be
     * visualized/executed*/
    moveit_msgs::RobotTrajectory stomp_trajectory_;

    /** \brief Class PandaArmPlanningControl for planning/controlling*/
    PandaArmPlanningControl panda_arm_;

    /** \brief Class PlanningSceneOperation for operating planning scene */
    PlanningSceneOperation planning_scene_;

    /** \brief Service server for executing n-th stomp trajectory*/
    ros::ServiceServer srv_execute_stomp_;
};
