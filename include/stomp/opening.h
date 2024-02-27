#include "include/stomp/execute_stomp_traj.h"

class Opening : public ExecuteStompTraj {
  public:
    Opening(ros::NodeHandle& n);
    ~Opening();

  protected:
    bool CallbackExecuteStompTraj(
        panda_moveit_control::ExecuteStompTraj::Request& req,
        panda_moveit_control::ExecuteStompTraj::Response& res) override;

    /**
     * \brief Generate and publish pre-grasp, grasp, post-grasp poses for door
     * handles to tf tree
     *
     * \param tf_world_to_marker transformation matrix from world frame to
     * marker frame
     */
    void GenerateGraspPose(
        const geometry_msgs::TransformStamped& tf_world_to_marker);

  private:
    geometry_msgs::Pose grasp_pose_;
    geometry_msgs::Pose pre_grasp_pose_;
    geometry_msgs::Pose post_grasp_pose_;

    /** \brief Mode of opening: sliding(drawer), rotating_left(sink door),
     * rotating_down(oven)*/
    std::string mode_;

    /** \brief Shortest distance from the handle to the rotation axis of the
     * door*/
    std::double_t rotation_axis_distance_ = 0.0;

    /** \brief Rotation angle of rotating door*/
    std::double_t rotation_angle_ = 0.0;

    /** \brief Sliding distance of sliding door*/
    std::double_t sliding_distance_ = 0.0;
};