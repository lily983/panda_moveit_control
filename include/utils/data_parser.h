#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <filesystem>
#include <string>

/**
 * \brief Copy the object's mesh file to the folder /models in order to add the
 * object as the collision object into the moveit planning scene
 *
 * \param web_path full path of the location of the mesh file
 * \param repo_path full path of the location of the mesh file to the repo
 * folder /models
 */
void CopyMeshFromWebToRepo(const std::string& web_path,
                           const std::string& repo_path);

/**
 * \brief Look up handle marker's pose relative to the panda_link0, which is
 * used in the tasks related to opening door
 *
 * \param marker_name name of the marker frame
 * \return geometry_msgs::TransformStamped marker's pose. Return identity pose
 * if failed to find the marker frame
 */
geometry_msgs::TransformStamped ReadHandleMarker(
    const std::string& marker_name = "handle_marker");

/**
 * \brief Publish a frame to the tf tree, such as publishing pre-grasp pose.
 *
 * @param pose pose of the frame
 * @param name name of the frame
 * @param header_frame header frame's name to attach the frame to
 */
void PubPoseToStaticTF(const geometry_msgs::Pose& pose, const std::string& name,
                       const std::string& header_frame = "panda_link0");
