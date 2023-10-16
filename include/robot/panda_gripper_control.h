#pragma once

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

/**
 * \brief Control the panda gripper's fingers to move to the given width
 *
 * \param width target width between fingers
 * \param speed speed of the finger movement
 * \return return true if fingers move to the target width within the defined
 * time duration
 */
bool GripperMoveAction(const double width, const double speed = 0.03);

bool GripperHomingAction();