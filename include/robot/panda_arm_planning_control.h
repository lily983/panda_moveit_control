#pragma once

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <math.h>

/**
 * \brief Class for planning and controlling the robot arm. Functions include
 * plan/move to target joint/pose target from current/given start joint
 * configuration.
 * \param max_vel_scale adjust the velocity of the arm during execution
 * \param max_acc_scale adjust the acceleration of the arm during execution
 */
class PandaArmPlanningControl {
  public:
    PandaArmPlanningControl(const double max_vel_scale = 0.1,
                            const double max_acc_scale = 0.05);
    ~PandaArmPlanningControl();

    /**
     * \brief Move arm back to home configuration.
     */
    bool GoHome();

    /**
     * \brief Move to joint target from current state
     *
     * \param joint_target target joint configuration
     */
    bool MoveToJointTarget(const std::vector<double>& joint_target);

    /**
     * \brief Move to pose target from current state
     *
     * \param pose_target target pose of panda_link8 relative to world
     */
    bool MoveToPoseTarget(const geometry_msgs::Pose& pose_target);

    /**
     * \brief Return planned trajectory to joint target from current state
     *
     * \param joint_target target joint configuration
     * \return moveit_msgs::RobotTrajectory planned trajectory. Return empty
     * trajectory if planning is failed
     */
    moveit_msgs::RobotTrajectory PlanningToJointTarget(
        const std::vector<double>& joint_target);

    /**
     * \brief Return planned trajectory to the joint target from the given start
     * joint configuration
     *
     * \param joint_target target joint configuration
     * \param start_joint_values  joint configuration as robot's start state
     * \return moveit_msgs::RobotTrajectory planned trajectory. Return empty
     * trajectory if planning is failed
     */
    moveit_msgs::RobotTrajectory PlanningToJointTarget(
        const std::vector<double>& joint_target,
        const std::vector<double>& start_joint_values);

    /**
     * \brief Return planned trajectory to the pose target from current state
     *
     * \param pose_target target pose of panda_link8 relative to world
     * \return moveit_msgs::RobotTrajectory planned trajectory. Return empty
     * trajectory if planning is failed
     */
    moveit_msgs::RobotTrajectory PlanningToPoseTarget(
        const geometry_msgs::Pose& pose_target);

    /**
     * \brief Return planned trajectory to the pose target from the given start
     * joint configuration
     *
     * \param pose_target target pose of panda_link8 relative to world
     * \param start_joint_values  joint configuration as robot's start state
     * \return moveit_msgs::RobotTrajectory planned trajectory. Return empty
     * trajectory if planning is failed
     */
    moveit_msgs::RobotTrajectory PlanningToPoseTarget(
        const geometry_msgs::Pose& pose_target,
        const std::vector<double>& start_joint_values);

    /**
     * \brief Control the arm to execute the trajectory
     *
     * \param traj target trajectory to execute
     * \return return true if execution is success, otherwise return false
     */
    bool ExecuteTrajectory(const moveit_msgs::RobotTrajectory& traj);

    /**
     * \brief Get the current robot state of the arm
     *
     * \return moveit::core::RobotState
     */
    moveit::core::RobotState GetMoveGroupState();

  private:
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
    const std::vector<double> home_config_ = {
        0.0, -M_PI / 4, 0.0, -3 * M_PI / 4, 0.0, M_PI / 2, M_PI / 4};
};