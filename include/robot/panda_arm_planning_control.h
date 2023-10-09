#pragma once

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <math.h>

class PandaArmPlanningControl {
  public:
    PandaArmPlanningControl(const double max_vel_scale = 0.1,
                            const double max_acc_scale = 0.05);
    ~PandaArmPlanningControl();

    bool GoHome();

    bool MoveToJointTarget(const std::vector<double>& joint_target);

    bool MoveToPoseTarget(const geometry_msgs::Pose& pose_target);

    moveit_msgs::RobotTrajectory PlanningToJointTarget(
        const std::vector<double>& joint_target);

    moveit_msgs::RobotTrajectory PlanningToJointTarget(
        const std::vector<double>& joint_target,
        const std::vector<double>& start_joint_values);

    moveit_msgs::RobotTrajectory PlanningToPoseTarget(
        const geometry_msgs::Pose& pose_target);

    moveit_msgs::RobotTrajectory PlanningToPoseTarget(
        const geometry_msgs::Pose& pose_target,
        const std::vector<double>& start_joint_values);

    bool ExecuteTrajectory(const moveit_msgs::RobotTrajectory& traj);

    moveit::core::RobotState GetMoveGroupState();

  private:
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
    const std::vector<double> home_config_ = {
        0.0, -M_PI / 4, 0.0, -3 * M_PI / 4, 0.0, M_PI / 2, M_PI / 4};
};