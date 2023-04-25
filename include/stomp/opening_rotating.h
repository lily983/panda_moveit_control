#include "include/stomp/execute_stomp_traj.h"

class OpeningRotating : public ExecuteStompTraj {
  public:
    OpeningRotating(ros::NodeHandle n);
    ~OpeningRotating();

  protected:
    bool CallbackExecuteStompTraj(
        panda_moveit_control::ExecuteStompTraj::Request& req,
        panda_moveit_control::ExecuteStompTraj::Response& res);

    void GenerateGraspPose(
        const geometry_msgs::TransformStamped& tf_world_to_marker);

  private:
    geometry_msgs::Pose grasp_pose_;
    geometry_msgs::Pose pre_grasp_pose_;
    geometry_msgs::Pose post_grasp_pose_;
};