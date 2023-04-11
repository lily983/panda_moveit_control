#pragma once

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <math.h>

#include "include/planning_scene/planning_scene_operation.h"

class PandaArmPlanningControl
{
    public:
    PandaArmPlanningControl(ros::NodeHandle nh, std::string planning_group, double_t max_vel_scale = 0.01, double_t max_acc_scale = 0.01);
    ~PandaArmPlanningControl();

    bool GoHome();

    bool MoveToJointTarget(std::vector<double> joint_target);

    bool MoveToPoseTarget(geometry_msgs::Pose pose_target);

    moveit_msgs::RobotTrajectory PlanningToJointTarget(std::vector<double> joint_target);

    moveit_msgs::RobotTrajectory PlanningToJointTarget(std::vector<double> joint_target, moveit::core::RobotState start_state);

    moveit_msgs::RobotTrajectory PlanningToPoseTarget(geometry_msgs::Pose pose_target);

    moveit_msgs::RobotTrajectory PlanningToPoseTarget(geometry_msgs::Pose pose_target, moveit::core::RobotState start_state);

    bool ExecuteTrajectory(moveit_msgs::RobotTrajectory traj);

    private:
    ros::NodeHandle nh_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
    PlanningSceneOperation planning_scene_;
    const std::vector<double> home_config_;
};