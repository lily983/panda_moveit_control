#include "include/robot/panda_gripper_control.h"

bool GripperMoveAction(const double width, const double speed,
                       const double timeout_duration) {
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_ac(
        "/franka_gripper/move");
    while (!move_ac.waitForServer(ros::Duration(timeout_duration))) {
        ROS_ERROR("Failed to wait for franka_gripper move server");
        return false;
    }
    franka_gripper::MoveGoal move_goal;
    move_goal.width = width;
    move_goal.speed = speed;
    ROS_INFO("Prepare to move gripper!");
    move_ac.sendGoal(move_goal);
    if (move_ac.waitForResult(ros::Duration(timeout_duration))) {
        ROS_INFO("Franka gripper sucessfully complish move to distance %f.",
                 width);
    } else {
        ROS_ERROR("Franka gripper failed to complish move action.");
    }
    return true;
}

bool GripperHomingAction() { return GripperMoveAction(0.08, 0.05); }
