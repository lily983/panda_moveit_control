#include "include/robot/panda_arm_planning_control.h"

PandaArmPlanningControl::PandaArmPlanningControl(ros::NodeHandle nh, std::string planning_group, double_t max_vel_scale, double_t max_acc_scale)
: planning_group_(planning_group)
, move_group_(planning_group)
{
    joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(planning_group_);
    move_group_.setMaxAccelerationScalingFactor(max_acc_scale);
    move_group_.setMaxVelocityScalingFactor(max_vel_scale);
    home_config_[0.0, -M_PI/4, 0.0, -2*M_PI/3, 0.0, M_PI/3, M_PI/4];
    ROS_INFO("[PandaArmPlanningControl]: finished initialization!");
}

bool PandaArmPlanningControl::GoHome()
{
    ROS_INFO("Prepare to set arm back to home joint configuration!");
    return MoveToJointTarget(home_config_);
}

bool PandaArmPlanningControl::MoveToJointTarget(std::vector<double> joint_target)
{
    return ExecuteTrajectory(PlanningToJointTarget(joint_target));
}

bool PandaArmPlanningControl::MoveToPoseTarget(geometry_msgs::Pose pose_target)
{
    return ExecuteTrajectory(PlanningToPoseTarget(pose_target));
}

moveit_msgs::RobotTrajectory PandaArmPlanningControl::PlanningToJointTarget(std::vector<double> joint_target)
{
    moveit_msgs::RobotTrajectory result_traj;
    move_group_.setJointValueTarget(joint_target);
    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;
    if(move_group_.plan(moveit_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Succefully planning to target joint configuration...");
        result_traj = moveit_plan.trajectory_;
    }
    else{
        ROS_ERROR("Failed to plan to target joint configuration!");
    }
    return result_traj;
}

moveit_msgs::RobotTrajectory PandaArmPlanningControl::PlanningToJointTarget(std::vector<double> joint_target, moveit::core::RobotState start_state)
{
    moveit_msgs::RobotTrajectory result_traj;

    moveit::core::RobotStatePtr backup_state;
    backup_state = move_group_.getCurrentState();

    move_group_.setStartState(start_state);
    result_traj = PlanningToJointTarget(joint_target);
    if(result_traj.joint_trajectory.points.size() == 0){
        ROS_ERROR("Failed to find path from the given start state to target joint configuration!");
    }

    move_group_.setStartState(*backup_state);

    return result_traj;
}

moveit_msgs::RobotTrajectory PandaArmPlanningControl::PlanningToPoseTarget(geometry_msgs::Pose pose_target)
{
    moveit_msgs::RobotTrajectory result_traj;
    move_group_.setPoseTarget(pose_target);
    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;
    if(move_group_.plan(moveit_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Succefully planning to target pose...");
        result_traj = moveit_plan.trajectory_;
    }
    else{
        ROS_ERROR("Failed to plan to target pose!");
    }
    return result_traj;
}

moveit_msgs::RobotTrajectory PandaArmPlanningControl::PlanningToPoseTarget(geometry_msgs::Pose pose_target, moveit::core::RobotState start_state)

{
    moveit_msgs::RobotTrajectory result_traj;

    moveit::core::RobotStatePtr backup_state;
    backup_state = move_group_.getCurrentState();

    move_group_.setStartState(start_state);
    result_traj = PlanningToPoseTarget(pose_target);
    if(result_traj.joint_trajectory.points.size() == 0){
        ROS_ERROR("Failed to find path from the given start state to target pose!");
    }

    move_group_.setStartState(*backup_state);

    return result_traj;
}

bool PandaArmPlanningControl::ExecuteTrajectory(moveit_msgs::RobotTrajectory traj)
{
    bool status = true;
    if(move_group_.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Finished executing robot trajectory");
    }
    else{
        ROS_ERROR("Failed to execute robot trajectory!");
        status = false;
    }
    return status;
}


