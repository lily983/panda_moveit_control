#include "include/stomp/opening.h"

Opening::Opening(ros::NodeHandle& n) : ExecuteStompTraj(n) {
    n_.getParam("mode", mode_);
    n_.getParam("rotation_axis_distance", rotation_axis_distance_);
    n_.getParam("rotation_angle", rotation_angle_);
    n_.getParam("sliding_distance", sliding_distance_);

    GenerateGraspPose(ReadHandleMarker("handle_marker"));

    ROS_INFO("[Opening]: finished initialization!");
}

Opening::~Opening() {}

bool Opening::CallbackExecuteStompTraj(
    panda_moveit_control::ExecuteStompTraj::Request& req,
    panda_moveit_control::ExecuteStompTraj::Response& res) {
    ROS_INFO(
        "===Received request to execute STOMP trajectory for the "
        "Opening "
        "task!===");
    if (all_stomp_trajectory_.trajectory.size() == 0) {
        ROS_ERROR("Haven't received STOMP trajectroy ");
        res.success = false;
        return 0;
    }

    GripperHomingAction();
    ros::Duration(0.5).sleep();

    stomp_trajectory_ = all_stomp_trajectory_.trajectory.at(req.num);
    trajectory_msgs::JointTrajectoryPoint stomp_first_point;
    stomp_first_point = stomp_trajectory_.joint_trajectory.points.at(0);

    ROS_INFO("===Add collision scene to the planning scene===");
    planning_scene_.AddCollisionMesh("mesh_scene");

    moveit_msgs::RobotTrajectory traj_stomp_to_pregrasp;
    moveit_msgs::RobotTrajectory traj_start_to_pregrasp;

    traj_stomp_to_pregrasp = panda_arm_.PlanningToPoseTarget(
        pre_grasp_pose_, stomp_first_point.positions);

    if (traj_stomp_to_pregrasp.joint_trajectory.points.size() == 0) {
        ROS_ERROR("Failed to plan path pre-grasp pose!");
        res.success = false;
        return false;
    }

    ROS_INFO("===Add collision object to the planning scene===");
    planning_scene_.AddCollisionMesh("mesh_obj");
    ros::Duration(2).sleep();

    trajectory_msgs::JointTrajectoryPoint pre_grasp_last_point;
    pre_grasp_last_point = traj_stomp_to_pregrasp.joint_trajectory.points.at(
        traj_stomp_to_pregrasp.joint_trajectory.points.size() - 1);

    traj_start_to_pregrasp =
        panda_arm_.PlanningToJointTarget(pre_grasp_last_point.positions);

    if (traj_start_to_pregrasp.joint_trajectory.points.size() == 0) {
        ROS_ERROR(
            "===Failed to plan path pre-grasp pose joint configuration!===");
        res.success = false;
        return false;
    }

    ROS_INFO("===Remove collision object from the planning scene===");
    planning_scene_.RemoveCollisionMesh("mesh_obj");

    ROS_INFO("===Executing pre-grasp trajectory!===");
    panda_arm_.ExecuteTrajectory(traj_start_to_pregrasp);

    ROS_INFO("===Ready to grasp handle!===");
    if (panda_arm_.MoveToJointTarget(stomp_first_point.positions) == 0) {
        ROS_ERROR("===Failed to grasp handle!===");
        res.success = false;
        return false;
    }

    ros::Duration(0.5).sleep();
    GripperMoveAction(0.04);

    ROS_INFO("===Ready to execute STOMP trajectory!===");
    if (panda_arm_.ExecuteTrajectory(stomp_trajectory_) == 0) {
        ROS_ERROR("===Failed to execute STOMP trajectory!===");
        res.success = false;
        return false;
    }

    GripperHomingAction();
    ros::Duration(0.5).sleep();
    panda_arm_.GoHome();

    res.success = true;
    return true;
}

void Opening::GenerateGraspPose(
    const geometry_msgs::TransformStamped& tf_world_to_marker) {
    T ei_marker_to_grasp, ei_marker_to_pregrasp, ei_marker_to_postgrasp;

    Eigen::Quaterniond quat_marker_to_grasp(0.0001, -0.3832, 0.9237, 0.001);
    ei_marker_to_grasp =
        Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, 0.083)) *
        quat_marker_to_grasp.normalized().toRotationMatrix();
    ei_marker_to_pregrasp =
        Eigen::Translation3d(Eigen::Vector3d(0.0, 0.05, 0.183)) *
        quat_marker_to_grasp.normalized().toRotationMatrix();
    ei_marker_to_postgrasp =
        Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, 0.083)) *
        quat_marker_to_grasp.normalized().toRotationMatrix();

    T ei_world_to_marker, ei_world_to_grasp, ei_world_to_pregrasp,
        ei_world_to_postgrasp;

    tf::transformMsgToEigen(tf_world_to_marker.transform, ei_world_to_marker);
    ei_world_to_grasp = ei_world_to_marker * ei_marker_to_grasp;
    ei_world_to_pregrasp = ei_world_to_marker * ei_marker_to_pregrasp;

    // For rotating door, start pose and goal pose are related by rotating along
    // an axis
    // For sliding door, start pose and goal pose are related by translating
    // along an axis
    std::cout << rotation_axis_distance_ << "\n";
    std::cout << rotation_angle_ << "\n";
    T ei_marker_start_to_goal;
    if (mode_ == "rotating_left") {
        ei_marker_start_to_goal.matrix() << std::cos(rotation_angle_),
            -std::sin(rotation_angle_), 0,
            rotation_axis_distance_ * (1 - std::cos(rotation_angle_)),
            std::sin(rotation_angle_), std::cos(rotation_angle_), 0,
            -rotation_axis_distance_ * std::sin(rotation_angle_), 0, 0, 1, 0, 0,
            0, 0, 1;
    } else if (mode_ == "rotating_down") {
        ei_marker_start_to_goal.matrix() << 1, 0, 0, 0, 0,
            std::cos(rotation_angle_), -std::sin(rotation_angle_),
            rotation_axis_distance_ * std::sin(-rotation_angle_), 0,
            std::sin(rotation_angle_), std::cos(rotation_angle_),
            -rotation_axis_distance_ * (1 - std::cos(-rotation_angle_)), 0, 0,
            0, 1;
    } else if (mode_ == "sliding") {
        ei_marker_start_to_goal.matrix() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 1;

        ei_marker_to_postgrasp =
            Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, 0.083)) *
            quat_marker_to_grasp.normalized().toRotationMatrix();
    }

    T ei_world_to_marker_goal;
    ei_world_to_marker_goal = ei_world_to_marker * ei_marker_start_to_goal;

    ei_world_to_postgrasp = ei_world_to_marker_goal * ei_marker_to_postgrasp;

    tf::poseEigenToMsg(ei_world_to_grasp, grasp_pose_);
    tf::poseEigenToMsg(ei_world_to_pregrasp, pre_grasp_pose_);
    tf::poseEigenToMsg(ei_world_to_postgrasp, post_grasp_pose_);

    PubPoseToStaticTF(pre_grasp_pose_, "pre_grasp_pose");
    PubPoseToStaticTF(grasp_pose_, "start_pose");
    PubPoseToStaticTF(post_grasp_pose_, "goal_pose");

    ROS_INFO(
        "===Finished generating grasp pose, pre-grasp pose, and post-grasp "
        "pose!===");
}
