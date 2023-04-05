#pragma once

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/HomingAction.h>


bool GripperMoveAction(double width, double speed)
{
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_ac("franka_gripper/move");
    while(!move_ac.waitForServer(ros::Duration(5.0))){
        ROS_ERROR("Failed to wait for franka_gripper move server");
        return false;
    }
    franka_gripper::MoveGoal move_goal;
    move_goal.width = width;
    move_goal.speed = speed;
    ROS_INFO("Prepare to move gripper!");
    move_ac.sendGoal(move_goal);
    if(move_ac.waitForResult(ros::Duration(20.0)))
    {
        ROS_INFO("Franka gripper sucessfully complish move to distance %d.", width);
    }
    else
    {
        ROS_ERROR("Franka gripper failed to complish move action.");
    }
    return true;
}

bool GripperMoveAction(double width)
{
    GripperMoveAction(width, 0.03);
}

bool GripperHomingAction()
{
    GripperMoveAction(0.8, 0.05);
}
