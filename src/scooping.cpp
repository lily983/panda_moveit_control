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

    if (argc != 2) {
        ROS_ERROR("!!!Please indicate which stomp trajectory to execute!!!");
        return 0;
    }
    // Execute the num-th trajectory from all the candidate stomp trajectories
    int num_th_traj = atoi(argv[1]);

    PandaArmPlanningControl panda_arm(0.1, 0.1);

    ExecuteStompTraj execute_stomp(nh);

    moveit_msgs::RobotTrajectory execure_traj;
    execure_traj = execute_stomp.GetTargetStompTrajectory(num_th_traj);

    trajectory_msgs::JointTrajectoryPoint stomp_first_point;
    stomp_first_point = execure_traj.joint_trajectory.points.at(0);

    ROS_INFO("===Add collision scene and object to the planning scene===");
    std::string collision_scene_mesh;
    std::string collision_model_mesh;
    collision_scene_mesh =
        "package://panda_moveit_control/models/mesh_scene.obj";
    collision_model_mesh = "package://panda_moveit_control/models/mesh_obj.obj";

    panda_moveit_control::AddCollisionMesh srv_add_collision_mesh;
    srv_add_collision_mesh.request.collsion_obj_name = "collision_object";
    srv_add_collision_mesh.request.mesh_path = collision_model_mesh;
    if (execute_stomp.client_add_collision_mesh_.call(srv_add_collision_mesh) ==
        false) {
        ROS_ERROR("!!!Failed to add collision model into planning scene!!!");
        return 0;
    }

    srv_add_collision_mesh.request.collsion_obj_name = "collision_scene";
    srv_add_collision_mesh.request.mesh_path = collision_scene_mesh;
    if (execute_stomp.client_add_collision_mesh_.call(srv_add_collision_mesh) ==
        false) {
        ROS_ERROR("!!!Failed to add collision scene into planning scene!!!");
        return 0;
    }

    ROS_INFO("===Move to the first joint configuration!!!===");
    ros::Duration(1.0).sleep();
    if (panda_arm.MoveToJointTarget(stomp_first_point.positions) == false) {
        ROS_ERROR("===Failed to get to the first joint configuration!!!===");
        return 0;
    }
    ROS_INFO(
        "===Reached to the start joint configuration. Please handover the "
        "spoon!===");
    ros::Duration(5.0).sleep();
    if (GripperMoveAction(0.008, 0.01) == false) {
        ROS_WARN("===Franka gripper failed to execute move command!===");
    }

    ROS_INFO("===Ready to execute STOMP trajectory! Please leave the arm!===");
    ros::Duration(5.0).sleep();

    if (panda_arm.ExecuteTrajectory(execure_traj) == false) {
        ROS_ERROR("===Failed to execute STOMP trajectory!===");
        return 0;
    }

    ROS_INFO(
        "===Finished executing STOMP trajectory for the scooping task!===");

    ros::waitForShutdown();
    return 0;
}