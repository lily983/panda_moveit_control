#pragma once

#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <panda_moveit_control/AddCollisionMesh.h>
#include <panda_moveit_control/RemoveCollisionMesh.h>
#include <panda_moveit_control/VisualizeStompTraj.h>
#include <ros/ros.h>

/**
 * \brief Class for interacting with the moveit planning scene, including
 * adding/removing mesh-formatted obstacles and publishing robot trajectory to
 * rviz.
 */
class PlanningSceneOperation {
  public:
    PlanningSceneOperation(ros::NodeHandle& nh);
    ~PlanningSceneOperation();

    /**
     * \brief Publish trajectory to rviz for visualization, such as trajectory
     * from current state to pre-grasp pose
     *
     * \param traj trajectory message
     * \param max_num_attempt max number of trail to wait for subscriber,
     * functioning as timeout
     */
    void VisualizeTrajectory(const moveit_msgs::DisplayTrajectory& traj,
                             const int max_num_attempt = 5);

  protected:
    /**
     * \brief Remove the mesh-formatted collision object from the planning scene
     *
     * \param obj_name name of the collision object in the planning scene
     * \param header_frame header frame of which the collision object is
     * attached to, default one is the world frame
     */
    void RemoveCollisionMesh(const std::string& obj_name,
                             const std::string& header_frame = "world");

    /**
     * \brief Add the mesh-formatted collision object into the planning scene
     *
     * \param obj_name name of the collision object. Should match with the
     * name of its mesh file in the folder /models
     * \param header_frame attach the collision object to the given frame,
     * default one is the world frame
     */
    void AddCollisionMesh(const std::string& obj_name,
                          const std::string& header_frame = "world");

    /**
     * \brief Callback function of the service for adding mesh-formatted
     * collision object into the planning scene
     */
    bool CallbackAddCollisionMesh(
        panda_moveit_control::AddCollisionMesh::Request& req,
        panda_moveit_control::AddCollisionMesh::Response& res);

    /**
     * \brief Callback function of the service for removing the mesh-formatted
     * collision object from the planning scene
     */
    bool CallbackRemoveCollisionMesh(
        panda_moveit_control::RemoveCollisionMesh::Request& req,
        panda_moveit_control::RemoveCollisionMesh::Response& res);

    /**
     * \brief Callback function of the service for visualizing stomp's n-th
     * trajectory
     */
    bool CallbackVisualizeStompTraj(
        panda_moveit_control::VisualizeStompTraj::Request& req,
        panda_moveit_control::VisualizeStompTraj::Response& res);

    /**
     * \brief Receive and store msg of type moveit_msgs::DisplayTrajectory
     */
    void CallbackStompTraj(const moveit_msgs::DisplayTrajectoryPtr& msg);

  private:
    ros::NodeHandle nh_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools_;
    ros::Publisher pub_planning_scene_;
    ros::Publisher pub_trajectory_;
    ros::ServiceServer srv_visualize_stomp_trajectory_;
    ros::ServiceServer srv_add_collision_mesh_;
    ros::ServiceServer srv_remove_collision_mesh_;
    moveit_msgs::DisplayTrajectory all_stomp_trajectory_;
    ros::Subscriber sub_trajectory_;
};