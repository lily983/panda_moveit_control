#pragma once

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

/**
 * \brief ros duration to wait for the action server of franka gripper
 *
 */
const double kActionServerTimeout = 5.0;

/**
 * \brief ros duration to wait for the action result of franka gripper
 *
 */
const double kActionResultTimeout = 5.0;

/**
 * \brief Control the panda gripper's fingers to move to the given width
 *
 * \param width target width between fingers
 * \param speed speed of the finger movement
 * \param timeout_duration time duration of waitting for the gripper action
 * result
 * \return return true if fingers move to the target width within the
 * defined time duration
 */
bool GripperMoveAction(const double width, const double speed = 0.03);

bool GripperHomingAction();