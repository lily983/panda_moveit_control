#pragma once

#include "include/planning_scene/planning_scene_operation.h"
#include "include/robot/panda_arm_planning_control.h"
#include "include/robot/panda_gripper_control.h"
#include "include/stomp/data_parse.h"
#include "panda_moveit_control/ExecuteStompTraj.h"
#include "panda_moveit_control/DisplayStompTraj.h"

#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

typedef Eigen::Transform<double, 3, Eigen::Affine> T;

class ExecuteStompTraj
{
    public:
    ExecuteStompTraj(ros::NodeHandle n);
    ~ExecuteStompTraj();

    void StoreMeshToRepo();
    void CallbackStompTraj(const moveit_msgs::DisplayTrajectoryPtr& msg);
    bool CallBackDisplayStompTraj(panda_moveit_control::DisplayStompTraj::Request& req,
                                panda_moveit_control::DisplayStompTraj::Response& res);
    virtual bool CallbackExecuteStompTraj(panda_moveit_control::ExecuteStompTraj::Request &req,
                                panda_moveit_control::ExecuteStompTraj::Response &res) = 0;

    private:
    ros::NodeHandle n_;
    std::string web_mesh_path_;
    std::string repo_mesh_path_;
    std::string stomp_traj_topic_;
    moveit_msgs::DisplayTrajectory all_stomp_trajectory_;
    moveit_msgs::RobotTrajectory stomp_trajectory_;
    ros::ServiceServer srv_display_stomp_traj_;
    ros::ServiceServer srv_execute_stomp_traj_;
    ros::Subscriber sub_trajectory_;
    ros::Publisher display_stomp_traj_publisher_;
};
