#include "include/planning_scene/planning_scene_operation.h"
#include "include/robot/panda_arm_planning_control.h"
#include "include/robot/panda_gripper_control.h"
#include "include/stomp/execute_stomp_traj.h"
#include "panda_moveit_control/AddCollisionMesh.h"
#include "panda_moveit_control/RemoveCollisionMesh.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "scooping_task");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    int num_th_traj = 0;
    ros::param::get("execute_num_th_trajectory", num_th_traj);

    PandaArmPlanningControl panda_arm(0.1, 0.1);

    ExecuteStompTraj execute_stomp(nh);

    ros::Duration(1.0).sleep();

    moveit_msgs::RobotTrajectory execure_traj;
    execure_traj = execute_stomp.GetTargetStompTrajectory(num_th_traj);

    trajectory_msgs::JointTrajectoryPoint stomp_first_point;
    stomp_first_point = execure_traj.joint_trajectory.points.at(0);

    ROS_INFO("===Move to the first joint configuration!!!===");
    if (panda_arm.MoveToJointTarget(stomp_first_point.positions) == false) {
        ROS_ERROR("===Failed to get to the first joint configuration!!!===");
        return 0;
    }
    ROS_INFO(
        "===Reached to the start joint configuration. Please handover the "
        "spoon!===");
    GripperHomingAction();
    if (GripperMoveAction(0.008, 0.05) == false) {
        ROS_WARN("===Franka gripper failed to execute move command!===");
    }

    ROS_INFO("===Ready to execute STOMP trajectory! Please leave the arm!===");
    ros::Duration(1).sleep();

    if (panda_arm.ExecuteTrajectory(execure_traj) == false) {
        ROS_ERROR("===Failed to execute STOMP trajectory!===");
        return 0;
    }

    ROS_INFO(
        "===Finished executing STOMP trajectory for the scooping task!===");

    ros::waitForShutdown();
    return 0;
}