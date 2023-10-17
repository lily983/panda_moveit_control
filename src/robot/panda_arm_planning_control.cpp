#include "include/robot/panda_arm_planning_control.h"

PandaArmPlanningControl::PandaArmPlanningControl(double_t max_vel_scale,
                                                 double_t max_acc_scale)
    : planning_group_("panda_arm"), move_group_("panda_arm") {
    joint_model_group_ =
        move_group_.getCurrentState()->getJointModelGroup(planning_group_);
    move_group_.setMaxAccelerationScalingFactor(max_acc_scale);
    move_group_.setMaxVelocityScalingFactor(max_vel_scale);
    ROS_INFO("[PandaArmPlanningControl]: finished initialization!");
}

PandaArmPlanningControl::~PandaArmPlanningControl(){};

bool PandaArmPlanningControl::GoHome() {
    ROS_INFO("Prepare to set arm back to home joint configuration!");
    return MoveToJointTarget(home_config_);
}

bool PandaArmPlanningControl::MoveToJointTarget(
    const std::vector<double>& joint_target) {
    return ExecuteTrajectory(PlanningToJointTarget(joint_target));
}

bool PandaArmPlanningControl::MoveToPoseTarget(
    const geometry_msgs::Pose& pose_target) {
    return ExecuteTrajectory(PlanningToPoseTarget(pose_target));
}

moveit_msgs::RobotTrajectory PandaArmPlanningControl::PlanningToJointTarget(
    const std::vector<double>& joint_target) {
    moveit_msgs::RobotTrajectory result_traj;
    std::cout << "Joint target is: "
              << "\n";
    for (auto joint_i : joint_target) std::cout << joint_i << "\n";
    if (move_group_.setJointValueTarget(joint_target) == false) {
        ROS_ERROR("!!!!!Failed to set to joint value target!!!!!");
        return result_traj;
    }

    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;
    if (move_group_.plan(moveit_plan) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Succefully planning to target joint configuration...");
        result_traj = moveit_plan.trajectory_;
    } else {
        ROS_ERROR("!!!Failed to plan to target joint configuration!!!");
    }
    return result_traj;
}

moveit_msgs::RobotTrajectory PandaArmPlanningControl::PlanningToJointTarget(
    const std::vector<double>& joint_target,
    const std::vector<double>& start_joint_values) {
    moveit_msgs::RobotTrajectory result_traj;

    moveit::core::RobotStatePtr backup_state;
    backup_state = move_group_.getCurrentState();

    moveit::core::RobotStatePtr start_state = move_group_.getCurrentState();
    start_state->setJointGroupPositions(joint_model_group_, start_joint_values);

    move_group_.setStartState(*start_state);
    result_traj = PlanningToJointTarget(joint_target);
    if (result_traj.joint_trajectory.points.size() == 0) {
        ROS_ERROR(
            "Failed to find path from the given start state to target joint "
            "configuration!");
    }

    move_group_.setStartState(*backup_state);

    return result_traj;
}

moveit_msgs::RobotTrajectory PandaArmPlanningControl::PlanningToPoseTarget(
    const geometry_msgs::Pose& pose_target) {
    moveit_msgs::RobotTrajectory result_traj;
    std::cout << "The target pose is: "
              << "\n";
    std::cout << pose_target << "\n";
    if (move_group_.setPoseTarget(pose_target, "panda_link8") == false) {
        ROS_ERROR("!!!Failed to set to pose target!!!");
        return result_traj;
    }
    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;
    if (move_group_.plan(moveit_plan) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Succefully planning to target pose...");
        result_traj = moveit_plan.trajectory_;
    } else {
        ROS_ERROR("Failed to plan to target pose!");
    }
    return result_traj;
}

moveit_msgs::RobotTrajectory PandaArmPlanningControl::PlanningToPoseTarget(
    const geometry_msgs::Pose& pose_target,
    const std::vector<double>& start_joint_values) {
    moveit_msgs::RobotTrajectory result_traj;
    moveit::core::RobotStatePtr backup_state;
    backup_state = move_group_.getCurrentState();

    moveit::core::RobotStatePtr start_state = move_group_.getCurrentState();
    start_state->setJointGroupPositions(joint_model_group_, start_joint_values);

    move_group_.setStartState(*start_state);
    result_traj = PlanningToPoseTarget(pose_target);
    if (result_traj.joint_trajectory.points.size() == 0) {
        ROS_ERROR(
            "Failed to find path from the given start state to target "
            "pose!");
    }

    move_group_.setStartState(*backup_state);

    return result_traj;
}

bool PandaArmPlanningControl::ExecuteTrajectory(
    const moveit_msgs::RobotTrajectory& traj) {
    bool status = true;
    if (move_group_.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Finished executing robot trajectory");
    } else {
        ROS_ERROR("Failed to execute robot trajectory!");
        status = false;
    }
    return status;
}

moveit::core::RobotState PandaArmPlanningControl::GetMoveGroupState() {
    return *(move_group_.getCurrentState());
}
