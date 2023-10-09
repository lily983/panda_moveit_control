#include "include/stomp/execute_stomp_traj.h"

ExecuteStompTraj::ExecuteStompTraj(ros::NodeHandle& n) : n_(n) {
    StoreMeshToRepo();

    // ros::param::get("~stomp_traj_topic", stomp_traj_topic_);
    stomp_traj_topic_ = "/server_stomp_planning/display_planned_path";
    sub_trajectory_ = n_.subscribe(stomp_traj_topic_, 1,
                                   &ExecuteStompTraj::CallbackStompTraj, this);
    ROS_INFO("Subscribe to STOMP trojectory in topic named %s",
             stomp_traj_topic_.c_str());

    client_add_collision_mesh_ =
        n_.serviceClient<panda_moveit_control::AddCollisionMesh>(
            "add_collision_mesh");

    client_remove_collision_mesh_ =
        n_.serviceClient<panda_moveit_control::RemoveCollisionMesh>(
            "remove_collision_mesh");

    ROS_INFO("[ExecuteStompTraj]: finished initialization!");
}

ExecuteStompTraj::~ExecuteStompTraj() {}

void ExecuteStompTraj::StoreMeshToRepo() {
    ros::param::get("~web_mesh_path", web_mesh_path_);
    ros::param::get("~repo_mesh_path", repo_mesh_path_);
    std::cout << web_mesh_path_ << "\n";
    std::cout << repo_mesh_path_ << "\n";

    std::string web_scene_path, web_obj_path, repo_scene_path, repo_obj_path;
    web_scene_path = web_mesh_path_ + "/mesh_scene.obj";
    web_obj_path = web_mesh_path_ + "/mesh_obj.obj";
    repo_scene_path = repo_mesh_path_ + "/mesh_scene.obj";
    repo_obj_path = repo_mesh_path_ + "/mesh_obj.obj";

    CopyMeshFromWebToRepo(web_scene_path, repo_scene_path);
    CopyMeshFromWebToRepo(web_obj_path, repo_obj_path);
}

void ExecuteStompTraj::CallbackStompTraj(
    const moveit_msgs::DisplayTrajectoryPtr& msg) {
    all_stomp_trajectory_ = *msg;
}

moveit_msgs::RobotTrajectory ExecuteStompTraj::GetTargetStompTrajectory(
    int num) {
    if (all_stomp_trajectory_.trajectory.size() == 0) {
        ROS_ERROR("!!!Haven't received any stomp trajectory!!!");
    }

    return all_stomp_trajectory_.trajectory.at(num);
}
