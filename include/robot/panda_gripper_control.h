#pragma once

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

bool GripperMoveAction(const double width, const double speed = 0.03);

bool GripperHomingAction();