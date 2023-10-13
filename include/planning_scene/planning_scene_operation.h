#pragma once

#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <panda_moveit_control/AddCollisionMesh.h>
#include <panda_moveit_control/RemoveCollisionMesh.h>
#include <panda_moveit_control/VisualizeStompTraj.h>
#include <ros/ros.h>

class PlanningSceneOperation {
  public:
    PlanningSceneOperation(ros::NodeHandle& nh);
    ~PlanningSceneOperation();

    /**
     * \brief Publish a trajectory to rviz for visualization, such as trajectory
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
     * \brief Remove mesh formatted collision object/scene from planning scene
     *
     * \param obj_name
     * \param header_frame
     */
    void RemoveCollisionMesh(const std::string& obj_name,
                             const std::string& header_frame = "world");

    /**
     * \brief Add mesh formatted collision object/scene from planning scene
     *
     * \param obj_name
     * \param header_frame
     */
    void AddCollisionMesh(const std::string& obj_name,
                          const std::string& header_frame = "world");

    /**
     * \brief Callback function to service of adding mesh formatted collision
     * object to planning scene
     *
     * \param req
     * \param res
     * \return true
     * \return false
     */
    bool CallbackAddCollisionMesh(
        panda_moveit_control::AddCollisionMesh::Request& req,
        panda_moveit_control::AddCollisionMesh::Response& res);

    /**
     * \brief Callback function to service of removing mesh formatted collision
     * object to planning scene
     *
     * \param req
     * \param res
     * \return true
     * \return false
     */
    bool CallbackRemoveCollisionMesh(
        panda_moveit_control::RemoveCollisionMesh::Request& req,
        panda_moveit_control::RemoveCollisionMesh::Response& res);

    /**
     * \brief Callback function for visualize stomp n-th trajectory service
     *
     * \param req request of srv VisualizeStompTraj
     * \param res response of srv VisualizeStompTraj
     * \return true
     * \return false failed to receive stomp trajectory
     */
    bool CallbackVisualizeStompTraj(
        panda_moveit_control::VisualizeStompTraj::Request& req,
        panda_moveit_control::VisualizeStompTraj::Response& res);

    /**
     * \brief Receive and store moveit_msgs::DisplayTrajectory
     * published by stomp
     *
     * \param msg
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