#pragma once

#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

class PlanningSceneOperation {
  public:
    PlanningSceneOperation(const ros::NodeHandle nh);
    ~PlanningSceneOperation();

    void AddCollisionObj(const std::string& obj_name,
                         const std::string& mesh_path,
                         const std::string& header_frame = "panda_link0");

    void AddCollisionObj(const bool obj_or_scene);

    void RemoveCollisionObj(const std::string& obj_name,
                            const std::string& header_frame = "panda_link0");

    void RemoveCollisionObj(const bool obj_or_scene);

    void VisualizeTrajectory(const moveit_msgs::DisplayTrajectory& traj);

  protected:
    ros::NodeHandle nh_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools_;
    ros::Publisher pub_planning_scene_;
    ros::Publisher pub_trajectory_;
};