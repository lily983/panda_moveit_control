#pragma once

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const double kPI = 3.1415926;

class PandaArmPlanningControl
{
    public:
    bool GoHome();

    bool MoveToJointTarget(std::vector<double> joint_target);

    bool MoveToPoseTarget(geometry_msgs::Pose pose_target);

    moveit_msgs::RobotTrajectory PlanningToJointTarget(std::vector<double> joint_target);

    moveit_msgs::RobotTrajectory PlanningToPoseTarget(geometry_msgs::Pose pose_target);

    std::vector<moveit_msgs::RobotTrajectory> PreGraspPlanning(geometry_msgs::Pose pre_grasp_pose,
                                                            geometry_msgs::Pose grasp_pose);

    std::vector<moveit_msgs::RobotTrajectory> PreGraspPlanning(geometry_msgs::Pose pre_grasp_pose,
                                                            std::vector<double> grasp_pose_joint_config);
                                                            
    bool ExecuteTrajectory(moveit_msgs::RobotTrajectory traj);


    private:
    moveit::planning_interface::MoveGroupInterface::Plan  planning_plan_;
    moveit::planning_interface::MoveGroupInterface move_group_;

};