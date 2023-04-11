#include "include/stomp/execute_stomp_traj.h"

ExecuteStompTraj::ExecuteStompTraj(ros::NodeHandle n)
: n_(n)
{
    StoreMeshToRepo();

    ros::param::get("~stomp_traj_topic", stomp_traj_topic_);
    sub_trajectory_ = n_.subscribe(stomp_traj_topic_, 1, &ExecuteStompTraj::CallbackStompTraj, this);
    ROS_INFO("Subscribe to STOMP trojectory in topic named %s", stomp_traj_topic_.c_str());

    srv_display_stomp_traj_ = n_.advertiseService("display_stomp_traj", &ExecuteStompTraj::CallBackDisplayStompTraj, this);
    display_stomp_traj_publisher_ = n.advertise<moveit_msgs::DisplayTrajectory>("display_target_traj", 1);

    ROS_INFO("[ExecuteStompTraj]: finished initialization!");
}

ExecuteStompTraj::~ExecuteStompTraj(){}

void ExecuteStompTraj::StoreMeshToRepo()
{
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

void ExecuteStompTraj::CallbackStompTraj(const moveit_msgs::DisplayTrajectoryPtr& msg)
{
    all_stomp_trajectory_ = *msg;
}

bool ExecuteStompTraj::CallBackDisplayStompTraj(panda_moveit_control::DisplayStompTraj::Request& req,
                                                panda_moveit_control::DisplayStompTraj::Response& res)
{
    moveit_msgs::DisplayTrajectory display_traj;
    display_traj.trajectory.push_back(all_stomp_trajectory_.trajectory.at(req.num));
    if(all_stomp_trajectory_.trajectory.size()==0){
        ROS_ERROR("Recevied empty stomp trajectories list, failed to display the target trajectory!");
        return false;
    }
    display_traj.trajectory_start = all_stomp_trajectory_.trajectory_start;
    std::cout << "Dispaly " << req.num << "th STOMP trajectory" << "\n";
    display_stomp_traj_publisher_.publish(display_traj);
    return true;
}



