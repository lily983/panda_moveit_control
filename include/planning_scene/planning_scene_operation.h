#pragma once

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shape_operations.h>

class PlanningSceneOperation{
    public:
    PlanningSceneOperation(ros::NodeHandle nh);
    ~PlanningSceneOperation();

    void AddCollisionObj(std::string obj_name, std::string mesh_path, std::string header_frame = "panda_link0");

    void AddCollisionObj(bool obj_or_scene);

    void RemoveCollisionObj(std::string obj_name, std::string header_frame = "panda_link0");

    void RemoveCollisionObj(bool obj_or_scene);

    void VisualizeTrajectory(moveit_msgs::DisplayTrajectory traj);

    protected:
    ros::NodeHandle nh_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools_;
    ros::Publisher pub_planning_scene_;
    ros::Publisher pub_trajectory_;
};